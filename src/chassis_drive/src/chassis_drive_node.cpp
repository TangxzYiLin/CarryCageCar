#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <sensor_msgs/Joy.h>
#include "chassis_drive/chassis_alarm.h"
#include "chassis_drive/chassis_bat.h"
#include "chassis_drive/chassis_cmd.h"
#include "chassis_drive/chassis_state.h"

#include "chassis_drive_node.h"

#include"agvs_control/date_pads_cmd.h"
//test head file 

#include "diagnostic_msgs/DiagnosticStatus.h"
#include "diagnostic_updater/diagnostic_updater.h"
#include "diagnostic_updater/update_functions.h"
#include "diagnostic_updater/DiagnosticStatusWrapper.h"
#include "diagnostic_updater/publisher.h"
#include <std_srvs/Empty.h>
#include <sensor_msgs/JointState.h>

#include"chassis_drive/cmd_lift.h"
//user extern libmodbus  lib 
#ifdef _cplusplus
extern  "c"{
#endif

#include "modbus.h"

#ifdef _cplusplus
}
#endif

//AGV paraments 
#define PI 3.1415926535
#define AGVS_MIN_COMMAND_REC_FREQ     5.0
#define AGVS_MAX_COMMAND_REC_FREQ     150.0

#define AGVS_WHEEL_DIAMETER	      0.2195      // Default wheel diameter
#define DEFAULT_DIST_CENTER_TO_WHEEL  0.479       // Default distance center to motorwheel
    
#define MAX_ELEVATOR_POSITION	      0.05		// meters

#define angle_limit                   900
#define speed_limit                   280

using namespace std;


class chassis_control_class
{
public:
	chassis_control_class(ros::NodeHandle h);

	int starting();
	void update_chassis_state();
	void publish_chassis_state();
	bool chassis_control_loop();
	
	void agvs_pads_control_callback(const agvs_control::date_pads_cmdConstPtr &cmd_control);
	void stoping();

	bool srvCallback_RaiseElevator(chassis_drive::cmd_lift::Request &request, chassis_drive::cmd_lift::Response &response );
	bool srvCallback_LowerElevator(chassis_drive::cmd_lift::Request &request, chassis_drive::cmd_lift::Response &response );
        bool srvCallback_TestMode(std_srvs::Empty::Request &request, std_srvs::Empty::Response &response);
        void led_state(uint16_t colour);

private:
	ros::NodeHandle node_handle_;
	ros::NodeHandle private_node_handle_;
	double desired_freq_;

	ros::Time last_command_time_;// Last moment when the component received a command
	//Topic publisher the state of the chassis 
	ros::Publisher chassis_alarm_;
	ros::Publisher chassis_bat_;
	ros::Publisher chassis_state_;

	//Topic subsription the control vel and angle from the navigation 
	ros::Subscriber chassis_cmd_;

	ros::ServiceServer srv_RaiseElevator_; //升降机 升高
  	ros::ServiceServer srv_LowerElevator_; //升降机 降低

        ros::ServiceServer srv_ageingtest_;

	void  chassis_write_registers(int16_t addr_register,int16_t register_len,const uint16_t*date_tab_rq_registersaddr);
	void  chassis_read_regisers(int16_t addr_register,int16_t register_len, uint16_t*date_tab_rq_registersaddr);

	int16_t saturation(int16_t u, int16_t min, int16_t max);

        void chassis_mov_cmd(float speed, float angle);
        void heatbeat();

        void ageing_test();

	//the chassis date topic 
	std::string  chassis_alarm_topic_;
	std::string  chassis_bat_topic_;
	std::string  chassis_state_topic_;
	std::string  chassis_cmd_topic_;

	//the libmodbus param  
	modbus_t *ctx;
        int16_t nb_fail;

	//flag
	bool read_state_; 
        bool ageing_test_flag;

        int32_t odometer_current;
	
        //TODO test 
	sensor_msgs::JointState joint_state_;  
	uint16_t safe_flag;

	chassis_drive_reg *read_regbuf=new chassis_drive_reg();
        chassis_drive_reg *write_regbuf=new chassis_drive_reg();
};

	
chassis_control_class::chassis_control_class(ros::NodeHandle h): node_handle_(h),safe_flag(0),private_node_handle_("~"), desired_freq_(20.0)
{
	ROS_INFO("agvs_chassis_control_node - Init ");
	ros::NodeHandle chassis_drive_node_handle(node_handle_,"chassis_drive");

	//init all string param 
	private_node_handle_.param<std::string>("chassis_state_topic", chassis_state_topic_, "chassis_drive/chassis_state_topic");
	private_node_handle_.param<std::string>("chassis_bat_topic", chassis_bat_topic_, "chassis_drive/chassis_bat_topic");
	private_node_handle_.param<std::string>("chassis_cmd_topic", chassis_cmd_topic_, std::string("/agvs_control/pads_cmd"));
	private_node_handle_.param<std::string>("chassis_alarm_topic", chassis_alarm_topic_, "chassis_drive/chassis_alarm_topic");

	//publish and subscribe init 
	chassis_bat_ = private_node_handle_.advertise<chassis_drive::chassis_bat>(chassis_bat_topic_,50);
	chassis_alarm_ = private_node_handle_.advertise<chassis_drive::chassis_alarm>(chassis_alarm_topic_,50);
	chassis_state_ = private_node_handle_.advertise<chassis_drive::chassis_state>(chassis_state_topic_,50);

	chassis_cmd_= private_node_handle_.subscribe<agvs_control::date_pads_cmd>(chassis_cmd_topic_,30,&chassis_control_class::agvs_pads_control_callback,this);

 	srv_RaiseElevator_ = private_node_handle_.advertiseService("/agvs_pad/raise_elevator",  &chassis_control_class::srvCallback_RaiseElevator, this);
 	srv_LowerElevator_ = private_node_handle_.advertiseService("/agvs_pad/lower_elevator",  &chassis_control_class::srvCallback_LowerElevator, this);
	
        srv_ageingtest_ =private_node_handle_.advertiseService("/agvs_pad/ageing_test",&chassis_control_class::srvCallback_TestMode,this);
        // variable
	read_state_ = false;  //cmd_control has been read
        ageing_test_flag = false;

	//new modbus object
	ctx = modbus_new_rtu("/dev/ttyUSB0", 115200, 'N', 8, 1);
        modbus_set_slave(ctx, SERVER_ID);

	if (modbus_connect(ctx) == -1) {
                ROS_INFO("Connection failed: %s\n",modbus_strerror(errno));
                modbus_free(ctx);
	} else{
                ROS_INFO("Connection success: %s\n",modbus_strerror(errno));
	}
}
/*
//check all depend topic is ready and chassis self_check has completed
int chassis_control_class::starting()
{
	if (read_state_) {
        vector<string> joint_names = joint_state_.name;
        fwd_vel_ = find (joint_names.begin(),joint_names.end(), string(joint_front_wheel)) - joint_names.begin();
        bwd_vel_ = find (joint_names.begin(),joint_names.end(), string(joint_back_wheel)) - joint_names.begin();
        fwd_pos_ = find (joint_names.begin(),joint_names.end(), string(joint_front_motor_wheel)) - joint_names.begin();
        bwd_pos_ = find (joint_names.begin(),joint_names.end(), string(joint_back_motor_wheel)) - joint_names.begin();
        return 0;
    }
  else return -1;
}
*/

void chassis_control_class::chassis_write_registers(int16_t addr,int16_t nb,const uint16_t*tab_rq_registers)//FIXME the const
{
	int16_t rc;
        uint16_t *tab_rp_registers;

	tab_rp_registers = (uint16_t *) malloc(nb * sizeof(uint16_t));
        memset(tab_rp_registers, 0, nb * sizeof(uint16_t));

	rc = modbus_write_registers(ctx, addr, nb, tab_rq_registers);

	if (rc!= nb) {
		ROS_INFO("ERROR modbus_write_registers (%d)\n", rc);
		ROS_INFO("Address = %d, nb = %d\n", addr, nb);
		nb_fail++;
	} else {
		ROS_INFO("success........................", addr, nb);
	}
#if 0  
        else { 
                
		ROS_INFO("breakpoint_3\n");
		rc= modbus_read_registers(ctx, addr, nb, tab_rp_registers);

		if (rc != nb) {
			ROS_INFO("breakpoint_4\n");
			ROS_INFO("ERROR modbus_read_registers (%d)\n", rc);
			ROS_INFO("Address = %d, nb = %d\n", addr, nb);
			nb_fail++;
		} else {
			ROS_INFO("breakpoint_5\n");
			for (int i=0; i< nb; i++) {
				if (tab_rq_registers[i] == tab_rp_registers[i]) {
                                        ROS_INFO("success modbus_read_registers\n");
                                        ROS_INFO("Address = %d, value %d = %d \n",
                                        addr, tab_rq_registers[i],
                                        tab_rp_registers[i]);
                                        nb_fail++;
				}
			}
		}
	}
#endif

}

void chassis_control_class::chassis_read_regisers(int16_t addr,int16_t nb, uint16_t *tab_rq_registersaddr)
{	
	int16_t rc;
        rc= modbus_read_registers(ctx, addr, nb, tab_rq_registersaddr);

	if (rc != nb) {
		ROS_INFO("ERROR modbus_read_registers (%d)\n", rc);
		ROS_INFO("Address = %d, nb = %d\n", addr, nb);
		nb_fail++;
	} else {
		ROS_INFO("success........................", addr, nb);
	}
}

void chassis_control_class::chassis_mov_cmd(float speed, float angle) 
{
        //speed,angle limit
        write_regbuf->write_motor_cmd_.date_info.reg_motor_speed_= saturation(speed, -speed_limit, speed_limit);
        write_regbuf->write_motor_cmd_.date_info.reg_motor_angle_= -saturation(angle, -angle_limit, angle_limit);
        //safe avoidance

        chassis_write_registers(reg_motor_vel, 2,(uint16_t*)&write_regbuf->write_motor_cmd_);      
}

void chassis_control_class::update_chassis_state()
{
        //read sate of the chassis
	chassis_read_regisers(reg_chassis_states,read_buf_size_state,(uint16_t*)&read_regbuf->read_state_cmd_);
        ROS_INFO("odometer= %d",read_regbuf->read_state_cmd_.date_info.reg_odometer_);

#if 0
	ROS_INFO("speed_feedback= %d",(int16_t) read_regbuf->read_state_cmd_.date_info.reg_speed_feedback_);
	ROS_INFO("lift_high_feedback= %d",read_regbuf->read_state_cmd_.date_info.reg_lifthigh_feedback_);
	ROS_INFO("angle_feedback= %d",(int16_t)read_regbuf->read_state_cmd_.date_info.reg_angle_feedback_);

	ROS_INFO("safe_date= %x",read_regbuf->read_state_cmd_.date_info.reg_safe_state_);

	ROS_INFO("roate_motor_erro= %d",read_regbuf->read_state_cmd_.date_info.reg_roate_motor_erro_);
	ROS_INFO("self_selfcheck_erro= %d",read_regbuf->read_state_cmd_.date_info.reg_selfcheck_erro_);
	ROS_INFO("walk_motor_erro= %d",read_regbuf->read_state_cmd_.date_info.reg_walk_motor_erro_);
	ROS_INFO("lift_motor_erro= %d",read_regbuf->read_state_cmd_.date_info.reg_lift_motor_erro_);

	ROS_INFO("task_id= %d",read_regbuf->read_state_cmd_.date_info.reg_task_id_feedback_);
	ROS_INFO("task_state= %d",read_regbuf->read_state_cmd_.date_info.reg_task_state_feedback_);

	ROS_INFO("odometer= %d",read_regbuf->read_state_cmd_.date_info.reg_odometer_);
	ROS_INFO("odometer_param= %d",read_regbuf->read_state_cmd_.date_info.reg_odometer_param_);

	//read test bat
	ROS_INFO("update...");
#endif

#if 1
	if(((read_regbuf->read_state_cmd_.date_info.reg_safe_state_.all_status-0x60c)!=0) && read_regbuf->read_state_cmd_.date_info.reg_speed_feedback_!= 0){

                // write_regbuf->write_motor_cmd_.date_info.reg_motor_speed_=0; //FIXME the should change
                //chassis_write_registers(reg_motor_vel, 1,(uint16_t*)&write_regbuf->write_motor_cmd_.date_info.reg_motor_speed_);

                led_state(rgb_read);
                                                                                                        
                //pubish the safe_state                    
                } else if((read_regbuf->read_state_cmd_.date_info.reg_safe_state_.all_status-0x60c)==0){
                        led_state(rgb_green);
	        }
#endif

#if 0

	ROS_INFO("read_batdate...");
	chassis_read_regisers(reg_bat_states, read_buf_size_bat,(uint16_t*)&read_regbuf->read_bat_state_cmd_);
	ROS_INFO("date= %d",read_regbuf->read_bat_state_cmd_.date_info.reg_bat_current_);
	ROS_INFO("date= %d",read_regbuf->read_bat_state_cmd_.date_info.reg_bat_erro_);
	ROS_INFO("date= %d",read_regbuf->read_bat_state_cmd_.date_info.reg_bat_power_);
	ROS_INFO("date= %d",read_regbuf->read_bat_state_cmd_.date_info.reg_bat_voltage_);
#endif
        heatbeat();
}

void chassis_control_class::led_state(const uint16_t colour)
{
        //current state read
        static uint16_t led_state_prior =NULL; 
        if (led_state_prior == colour) return;
        else{
                write_regbuf->write_led_cmd_.all_status=colour;//read:2400  blue:1200  green :0900
                chassis_write_registers(reg_indicator, 1,(uint16_t*)&write_regbuf->write_led_cmd_.all_status);//FIXME ADD PUBLISH  
                led_state_prior =colour; 
        }                    
}

void chassis_control_class::heatbeat()
{
        static bool task_id_pack =false;
        task_id_pack=!task_id_pack;
	chassis_write_registers(reg_task_state,1,(uint16_t*)&task_id_pack);
}

void chassis_control_class::publish_chassis_state()
{	
	//ROS_INFO("publish...");
}

int16_t chassis_control_class::saturation(int16_t u, int16_t min, int16_t max) //速度 角度 限幅滤波
{
	if (u>max) u=max;
	if (u<min) u=min;
	return u; 
}

//save new date of agvs_pads
void chassis_control_class::agvs_pads_control_callback(const agvs_control::date_pads_cmdConstPtr &cmd_control)  //FIXME the mssage need genearte
{
	// Safety check
	last_command_time_ = ros::Time::now();
	// subs_command_freq->tick();			//TODO For diagnostics need add 
        if (ageing_test_flag == false){
                chassis_mov_cmd(cmd_control->speed_date,cmd_control->angle_date);  //TODO  speed is still publish
                ROS_INFO("chassis_drive::chassis_cmdConstPtr: agv_vel = %d, agv_angle = %d",write_regbuf->write_motor_cmd_.date_info.reg_motor_speed_,write_regbuf->write_motor_cmd_.date_info.reg_motor_angle_);         
        }
}

// Service Raise Elevator  
bool chassis_control_class::srvCallback_RaiseElevator(chassis_drive::cmd_lift::Request &request, chassis_drive::cmd_lift::Response &response )
{
        write_regbuf->reg_lift_high_=rise;
	chassis_write_registers(reg_high_control,1,(uint16_t*)&write_regbuf->reg_lift_high_);
	
        ROS_INFO("rise_elevator..."); 
        return true;
}

// Service Lower Elevator 
bool chassis_control_class::srvCallback_LowerElevator(chassis_drive::cmd_lift::Request &request, chassis_drive::cmd_lift::Response &response )
{
	write_regbuf->reg_lift_high_=land;
	chassis_write_registers(reg_high_control,1,(uint16_t*)&write_regbuf->reg_lift_high_);
        
	ROS_INFO("land_elevator..."); 
	return true;  
}

bool chassis_control_class::srvCallback_TestMode(std_srvs::Empty::Request &request, std_srvs::Empty::Response &response)
{
        ageing_test_flag = !ageing_test_flag;

        if (ageing_test_flag == false) chassis_mov_cmd(0.0f,0.0f);
        else {
                odometer_current =(int32_t)read_regbuf->read_state_cmd_.date_info.reg_odometer_;
                chassis_mov_cmd(100.0f,0.0f);
                ROS_INFO("ageing_test_callback\n");
        } 
        return true;
}

void  chassis_control_class::ageing_test()
{
        int32_t odometer_tmp;
        int32_t odometer_test =2000;

        if (ageing_test_flag){
                odometer_tmp = (int32_t)read_regbuf->read_state_cmd_.date_info.reg_odometer_ - odometer_current;
                ROS_INFO("odometer= %d",odometer_tmp);
                ROS_INFO("odometer= %d",read_regbuf->read_state_cmd_.date_info.reg_odometer_);

                if ((odometer_tmp >= odometer_test )&&(int16_t)read_regbuf->read_state_cmd_.date_info.reg_speed_feedback_ <0){
                        chassis_mov_cmd(0.0f,0.0f);
                        
                        write_regbuf->reg_lift_high_=land;
	                //chassis_write_registers(reg_high_control,1,(uint16_t*)&write_regbuf->reg_lift_high_);
                        usleep(1000000);
                        ROS_INFO("111111111111\n");

                } else if ((odometer_tmp <= -odometer_test )&&(int16_t)read_regbuf->read_state_cmd_.date_info.reg_speed_feedback_ >0){
                        
                        chassis_mov_cmd(0.0f,0.0f);

                        write_regbuf->reg_lift_high_=rise;
	                //chassis_write_registers(reg_high_control,1,(uint16_t*)&write_regbuf->reg_lift_high_);
                        usleep(1000000);
                        ROS_INFO("2222222222222222\n");

                } else if ((odometer_tmp <= -odometer_test )&&(int16_t)read_regbuf->read_state_cmd_.date_info.reg_speed_feedback_ ==0){
                        
                        chassis_mov_cmd(-200.0f,0.0f);
                        ROS_INFO("aaaaaaaaaaaaaaaaaaa\n");

                } else if ((odometer_tmp >= odometer_test )&&(int16_t)read_regbuf->read_state_cmd_.date_info.reg_speed_feedback_ ==0){
                                chassis_mov_cmd(200.0f,0.0f);
                                //ROS_INFO("chassis_drive::chassis_cmdConstPtr: agv_vel = %d, agv_angle = %d",write_regbuf->write_motor_cmd_.date_info.reg_motor_speed_,write_regbuf->write_motor_cmd_.date_info.reg_motor_angle_);
                        ROS_INFO("bbbbbbbbbbbbbbbbbbbbb\n");

                } else if ((odometer_tmp == 0)&&(int16_t)read_regbuf->read_state_cmd_.date_info.reg_speed_feedback_ ==0){
                        chassis_mov_cmd(-200.0f,0.0f);
                }
        } 
        
}

bool chassis_control_class::chassis_control_loop()
{
	ROS_INFO("chassis_drive_loop");
        ros::Rate r(desired_freq_); 
        // Using ros::isShuttingDown to avoid restarting the node during a shutdown

        while (!ros::isShuttingDown()) {
                if (1){
                        ROS_INFO("chassis_drive_while");
                        while(ros::ok() && node_handle_.ok()) {
                                update_chassis_state();
                                ageing_test();
                                //publish_chassis_state();
                                ros::spinOnce();
                                r.sleep();
                        }
                        ROS_INFO("END OF ros::ok() !!!");

                } else{
                        // No need for diagnostic here since a broadcast occurs in start
                        // when there is an error.
                        usleep(1000000);
                        ros::spinOnce();
                }
        }

        ROS_INFO("chassis_drive::spin() - end");
        return true;
}

int main(int argc, char **argv)
{
	ros::init(argc, argv, "chassis_drive");
	ros::NodeHandle n;		
  	chassis_control_class sxlrc(n);

        sxlrc.chassis_control_loop();

	return (0);
}

