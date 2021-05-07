#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <sensor_msgs/Joy.h>
#include <std_srvs/Empty.h>
#include <sensor_msgs/JointState.h>

#include "diagnostic_msgs/DiagnosticStatus.h"
#include "diagnostic_updater/diagnostic_updater.h"
#include "diagnostic_updater/update_functions.h"
#include "diagnostic_updater/DiagnosticStatusWrapper.h"
#include "diagnostic_updater/publisher.h"

#include "chassis_drive/chassis_alarm.h"
#include "chassis_drive/chassis_bat.h"
#include "chassis_drive/chassis_cmd.h"
#include "chassis_drive/chassis_state.h"
#include "chassis_drive/agvs_mode.h"
#include "chassis_drive/agvs_led_state.h"

#include "agvs_control/date_pads_cmd.h"

#include "chassis_regmap_define.h"

//user extern libmodbus  lib 
#ifdef _cplusplus
extern  "c"{
#endif

#include "modbus-rtu.h"

#ifdef _cplusplus
}
#endif

//AGV paraments 

#define AGVS_MIN_COMMAND_REC_FREQ               5.0
#define AGVS_MAX_COMMAND_REC_FREQ               150.0

#define DEFAULT_AGVS_WHEEL_DIAMETER	        0.2195      // Default wheel diameter
#define DEFAULT_DIST_CENTER_TO_WHEEL            0.479       // Default distance center to motorwheel
#define DEFAULT_ELEVATOR_POSITION_MAX	        0.05	    // meters

#define DEFAULT_ANGLE_MAX                       900
#define DEFAULT_SPEED_MAX                       800

using namespace std;

class class_chassis_control
{

public:

	class_chassis_control(ros::NodeHandle h);
        bool loop();
	int  starting();
        
	void publish_chassis_state(float update_freq);
        void publish_bate_state(float update_freq);
	
	void callback_agvs_manual_control(const agvs_control::date_pads_cmdConstPtr &cmd_control);
        void callback_agvs_auto_control(const agvs_control::date_pads_cmdConstPtr &cmd_control);

	bool srvcallback_elevator_raise(std_srvs::Empty::Request &request, std_srvs::Empty::Response &response );
	bool srvcallback_elevator_down(std_srvs::Empty::Request &request, std_srvs::Empty::Response &response );
        bool srvcallback_aging_test(std_srvs::Empty::Request &request, std_srvs::Empty::Response &response);
	
        void update_led_state(uint16_t colour);
        
        void stoping();

private:

	ros::NodeHandle node_handle_;
	ros::NodeHandle private_node_handle_;
	double desired_freq;

        void chassis_write_registers(int16_t addr_register,int16_t register_len,const uint16_t*date_tab_rq_registersaddr);
	void chassis_read_registers(int16_t addr_register,int16_t register_len, uint16_t*date_tab_rq_registersaddr);

	int16_t saturation(int16_t u, int16_t min, int16_t max);

        void chassis_motor_cmd(float speed, float angle);
        void update_chassis_heart_state(float frequecne_tmp);
        void chassis_ageing_test();
        void update_chassis_led_state(chassis_drive::agvs_led_state state_tmp,float frequecne_tmp);

        void update_chassis_motor_state();

        //Last moment when the component received a command
	ros::Time last_command_time_;
	//Topic publisher the state of the chassis 
	ros::Publisher chassis_alarm_;
	ros::Publisher chassis_bat_;
	ros::Publisher chassis_state_;

	//Topic subsription the control vel and angle from the navigation 
	ros::Subscriber chassis_manual_cmd_;
        ros::Subscriber chassis_auto_cmd_;

	ros::ServiceServer srv_raise_elevator_; 
  	ros::ServiceServer srv_lower_elevator_; 

        ros::ServiceServer srv_ageing_test_;

	//the chassis date topic 
	std::string  chassis_alarm_topic_;
	std::string  chassis_bat_topic_;
	std::string  chassis_state_topic_;

	std::string  chassis_manual_cmd_topic_;
        std::string  chassis_auto_cmd_topic_;

	//libmodbus param  
	modbus_t *ctx;
        int16_t nb_fail;

        bool event_ageing_test = false;
        bool sem_led_abnormal_state = false;

	sensor_msgs::JointState joint_state; 
        chassis_drive::agvs_mode agvs_mode = { };
        chassis_drive::agvs_led_state agvs_led_state = { };

        int32_t chassis_odometer_current;

	chassis_drive_reg_s *p_chassis_reg_buf = new chassis_drive_reg_s();

};

class_chassis_control::class_chassis_control(ros::NodeHandle h): node_handle_(h),private_node_handle_("~"), desired_freq(20.0)
{

	ROS_INFO("agvs_chassis_control_node - Init ");
	ros::NodeHandle chassis_drive_node_handle(node_handle_,"chassis_drive");

	//init all string param 
	private_node_handle_.param<std::string>("chassis_state_topic", chassis_state_topic_, std::string("chassis_drive/chassis_state_topic"));
	private_node_handle_.param<std::string>("chassis_bat_topic", chassis_bat_topic_, std::string("chassis_drive/chassis_bat_topic"));
	private_node_handle_.param<std::string>("chassis_alarm_topic", chassis_alarm_topic_, std::string("chassis_drive/chassis_alarm_topic"));

        private_node_handle_.param<std::string>("chassis_cmd_topic", chassis_manual_cmd_topic_, std::string("/agvs_control/pads_cmd"));
        private_node_handle_.param<std::string>("/auto/cmd", chassis_auto_cmd_topic_, std::string("/auto/cmd"));

	//publish and subscribe init 
	chassis_bat_ = private_node_handle_.advertise<chassis_drive::chassis_bat>(chassis_bat_topic_,10);
	chassis_alarm_ = private_node_handle_.advertise<chassis_drive::chassis_alarm>(chassis_alarm_topic_,10);
	chassis_state_ = private_node_handle_.advertise<chassis_drive::chassis_state>(chassis_state_topic_,10);

	chassis_manual_cmd_ = private_node_handle_.subscribe<agvs_control::date_pads_cmd>(chassis_manual_cmd_topic_,30,&class_chassis_control::callback_agvs_manual_control,this);
        chassis_auto_cmd_ = private_node_handle_.subscribe<agvs_control::date_pads_cmd>(chassis_auto_cmd_topic_,10,&class_chassis_control::callback_agvs_auto_control,this);

 	srv_raise_elevator_ = private_node_handle_.advertiseService("/agvs_pad/button_raise_fork",  &class_chassis_control::srvcallback_elevator_raise, this);
 	srv_lower_elevator_ = private_node_handle_.advertiseService("/agvs_pad/button_dwon_fork",  &class_chassis_control::srvcallback_elevator_down, this);
	
        srv_ageing_test_ = private_node_handle_.advertiseService("/agvs_pad/autmode",&class_chassis_control::srvcallback_aging_test,this);
        
        //new modbus connect
	ctx = modbus_new_rtu("/dev/ttyUSB0", 115200, 'N', 8, 1);
        modbus_set_slave(ctx, SERVER_ID);

	if (modbus_connect(ctx) == -1) {

                ROS_INFO("Connection failed: %s\n",modbus_strerror(errno));
                modbus_free(ctx);

	} else{

                ROS_INFO("Connection success: %s\n",modbus_strerror(errno));
	}
}

void class_chassis_control::chassis_write_registers(int16_t addr,int16_t nb,const uint16_t*tab_rq_registers)//FIXME the const
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

void class_chassis_control::chassis_read_registers(int16_t addr,int16_t nb, uint16_t *tab_rq_registersaddr)
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

void class_chassis_control::chassis_motor_cmd(float speed, float angle) 
{

        //speed,angle limit
        p_chassis_reg_buf->write_motor_cmd_.date_info.reg_motor_speed= saturation(speed, -DEFAULT_SPEED_MAX, DEFAULT_SPEED_MAX);
        p_chassis_reg_buf->write_motor_cmd_.date_info.reg_motor_angle= -saturation(angle, -DEFAULT_ANGLE_MAX, DEFAULT_ANGLE_MAX);
        chassis_write_registers(REG_MOTOR_VEL, 2,(uint16_t*)&p_chassis_reg_buf->write_motor_cmd_);      

}

void class_chassis_control::update_chassis_led_state(chassis_drive::agvs_led_state state_tmp,float frequecne_tmp)
{
        static uint8_t time_led_update;
        static chassis_drive::agvs_led_state led_state_prior = { };

        time_led_update++;

        if ((float)(time_led_update/desired_freq) >= frequecne_tmp )
        {

                time_led_update = 0;

                static bool led_flash_sem;
                led_flash_sem = !led_flash_sem;

                switch (state_tmp.update_state)
                {

                case chassis_led_state_all_off:

                        if (led_state_prior.update_state != state_tmp.update_state)
                        {
                                led_state_prior.update_state = state_tmp.update_state;
                                p_chassis_reg_buf->write_led_cmd_.all_status = 0x0000;//read:2400  blue:1200  green :0900
                                chassis_write_registers(REG_INDICATOR, 1,(uint16_t*)&p_chassis_reg_buf->write_led_cmd_.all_status);
                        }
                        break;

                 case chassis_led_state_red_on:

                        if (led_state_prior.update_state != state_tmp.update_state)
                        {
                                led_state_prior.update_state = state_tmp.update_state;
                                p_chassis_reg_buf->write_led_cmd_.all_status = 0x2400;
                                chassis_write_registers(REG_INDICATOR, 1,(uint16_t*)&p_chassis_reg_buf->write_led_cmd_.all_status);
                        }

                        break;

                case chassis_led_state_green_on:

                        if (led_state_prior.update_state != state_tmp.update_state)
                        {
                                led_state_prior.update_state = state_tmp.update_state;
                                p_chassis_reg_buf->write_led_cmd_.all_status = 0x0900;
                                chassis_write_registers(REG_INDICATOR, 1,(uint16_t*)&p_chassis_reg_buf->write_led_cmd_.all_status);
                        }
                        break;

                case chassis_led_state_bule_on:

                        if (led_state_prior != state_tmp)
                        {
                                led_state_prior.update_state = state_tmp.update_state;
                                p_chassis_reg_buf->write_led_cmd_.all_status =0x1200 ;
                                chassis_write_registers(REG_INDICATOR, 1,(uint16_t*)&p_chassis_reg_buf->write_led_cmd_.all_status);
                        }
                        break;

                case chassis_led_state_red_flash:

                        if(led_flash_sem != 0 )
                        {
                                led_state_prior.update_state = state_tmp.update_state;
                                p_chassis_reg_buf->write_led_cmd_.all_status = 0x2400 ;
                                chassis_write_registers(REG_INDICATOR, 1,(uint16_t*)&p_chassis_reg_buf->write_led_cmd_.all_status);

                        } else {
                                
                                led_state_prior.update_state = state_tmp.update_state;
                                p_chassis_reg_buf->write_led_cmd_.all_status = 0x0000 ;
                                chassis_write_registers(REG_INDICATOR, 1,(uint16_t*)&p_chassis_reg_buf->write_led_cmd_.all_status);
                        }
                        break;

                case chassis_led_state_green_flash:
                        
                        if(led_flash_sem != 0 )
                        {
                                led_state_prior.update_state = state_tmp.update_state;
                                p_chassis_reg_buf->write_led_cmd_.all_status = 0x0900 ;
                                chassis_write_registers(REG_INDICATOR, 1,(uint16_t*)&p_chassis_reg_buf->write_led_cmd_.all_status);

                        } else {
                                
                                led_state_prior.update_state = state_tmp.update_state;
                                p_chassis_reg_buf->write_led_cmd_.all_status = 0x0000 ;
                                chassis_write_registers(REG_INDICATOR, 1,(uint16_t*)&p_chassis_reg_buf->write_led_cmd_.all_status);
                        }
                        break;

                case chassis_led_state_blue_flash:
                        
                        if(led_flash_sem != 0 )
                        {
                                led_state_prior.update_state = state_tmp.update_state;
                                p_chassis_reg_buf->write_led_cmd_.all_status = 0x1200 ;
                                chassis_write_registers(REG_INDICATOR, 1,(uint16_t*)&p_chassis_reg_buf->write_led_cmd_.all_status);

                        } else {
                                
                                led_state_prior.update_state = state_tmp.update_state;
                                p_chassis_reg_buf->write_led_cmd_.all_status = 0x0000 ;
                                chassis_write_registers(REG_INDICATOR, 1,(uint16_t*)&p_chassis_reg_buf->write_led_cmd_.all_status);
                        }
                        break;

                default:
                        break;
                }
        }
}

void class_chassis_control::update_chassis_motor_state()
{
        
        if (agvs_mode.current_mode != agvs_mode.agvs_mode_manual)
        {
#if 1

                if(((p_chassis_reg_buf->read_state_cmd_.date_info.reg_safe_state_.all_status-0x60c)!=0) && p_chassis_reg_buf->read_state_cmd_.date_info.reg_speed_feedback_!= 0)
                {

                        p_chassis_reg_buf->write_motor_cmd_.date_info.reg_motor_angle = 0.0f;
                        p_chassis_reg_buf->write_motor_cmd_.date_info.reg_motor_angle = 0.0f;
                        
                        agvs_led_state.update_state = chassis_led_state_red_on;
                        sem_led_abnormal_state = true ;

                        //p_chassis_reg_buf->write_motor_cmd_.date_info.reg_motor_speed=0; 
                        //chassis_write_registers(REG_MOTOR_VEL, 1,(uint16_t*)&p_chassis_reg_buf->write_motor_cmd_.date_info.reg_motor_speed);
                        update_led_state(RGB_READ);
                                                                                                                                
                } else if((p_chassis_reg_buf->read_state_cmd_.date_info.reg_safe_state_.all_status-0x60c)==0){
                        
                        
                        update_led_state(RGB_GREEN);

                } else if (sem_led_abnormal_state == true){
                        
                        agvs_led_state.update_state = chassis_led_state_green_on;
                        sem_led_abnormal_state = false;
                        update_led_state(RGB_GREEN);

                }

                chassis_motor_cmd(p_chassis_reg_buf->write_motor_cmd_.date_info.reg_motor_speed,p_chassis_reg_buf->write_motor_cmd_.date_info.reg_motor_angle);

#endif
        } else if (agvs_mode.current_mode == agvs_mode.agvs_mode_manual) {

                chassis_motor_cmd(p_chassis_reg_buf->write_motor_cmd_.date_info.reg_motor_speed,p_chassis_reg_buf->write_motor_cmd_.date_info.reg_motor_angle);
        }

}

void class_chassis_control::update_chassis_heart_state(float frequecne_tmp)
{

        static uint8_t time_heart_beat;
        static bool task_id_pack =false;
        time_heart_beat++;

        if ( (float)(time_heart_beat / desired_freq) >= frequecne_tmp )
        {
                time_heart_beat = 0;
                task_id_pack=!task_id_pack;
                chassis_write_registers(REG_TASK_STATE,1,(uint16_t*)&task_id_pack);
        }  

}

void class_chassis_control::publish_bate_state(float update_freq)
{
        chassis_drive_reg_s chassis_reg_buf = { };  //TODO 
        static uint8_t time_tmp_bat_state =0;
        time_tmp_bat_state++;
        
        if ( (float)(time_tmp_bat_state/desired_freq) >= update_freq) 
        { 

                time_tmp_bat_state =0;
                chassis_drive::chassis_bat msgdate;
                chassis_read_registers(REG_BAT_STATES,READ_BUF_BAT_SIZE,(uint16_t*)&chassis_reg_buf.read_bat_state_cmd_);

                msgdate.bat_cap_ = chassis_reg_buf.read_bat_state_cmd_.date_info.reg_bat_power_;
                msgdate.bat_cur_ = chassis_reg_buf.read_bat_state_cmd_.date_info.reg_bat_current_;
                msgdate.bat_error_code_ = chassis_reg_buf.read_bat_state_cmd_.date_info.reg_bat_erro_;
                msgdate.bat_vol_ = chassis_reg_buf.read_bat_state_cmd_.date_info.reg_bat_voltage_;

                chassis_bat_.publish(msgdate);
        }

}

void class_chassis_control::publish_chassis_state(float update_freq)
{
        chassis_drive_reg_s chassis_reg_buf = { }; 
        static uint8_t time_tmp_chassis_state =0;
        time_tmp_chassis_state++;
        
        if ((float)(time_tmp_chassis_state / desired_freq) >= update_freq) 
        { 

                time_tmp_chassis_state = 0;
                chassis_drive::chassis_state msg_state1;
                chassis_drive::chassis_alarm msg_date2;
               
                //chassis state update
                chassis_read_registers(REG_CHASSIS_STATES,READ_BUF_STATE_SIZE,(uint16_t*)&chassis_reg_buf.read_state_cmd_);
                ROS_INFO("odometer= %d",chassis_reg_buf.read_state_cmd_.date_info.reg_odometer_);

                msg_state1.chassis_drive_angle_ = chassis_reg_buf.read_state_cmd_.date_info.reg_angle_feedback_;
                msg_state1.chassis_drive_speed_ = chassis_reg_buf.read_state_cmd_.date_info.reg_speed_feedback_;
                msg_state1.chassis_mileage_record_ = chassis_reg_buf.read_state_cmd_.date_info.reg_odometer_;

                msg_state1.chassis_drivemotor_error_code_ = chassis_reg_buf.read_state_cmd_.date_info.reg_walk_motor_erro_;
                msg_state1.chassis_liftmotor_erro_code_ = chassis_reg_buf.read_state_cmd_.date_info.reg_lift_motor_erro_;
                msg_state1.chassis_selfcheck_error_code_ = chassis_reg_buf.read_state_cmd_.date_info.reg_selfcheck_erro_;
                msg_state1.chassis_whirlmotor_erro_code_ = chassis_reg_buf.read_state_cmd_.date_info.reg_roate_motor_erro_;
                chassis_state_.publish(msg_state1);

                //safe date update
                msg_date2.alarm_auto_man_swtich_ = chassis_reg_buf.read_state_cmd_.date_info.reg_safe_state_.bit.alarm_auto_man_swtich_;
                
                msg_date2.alarm_cargophotos_left_= chassis_reg_buf.read_state_cmd_.date_info.reg_safe_state_.bit.alarm_cargophotos_left_;
                msg_date2.alarm_cargophotos_right_= chassis_reg_buf.read_state_cmd_.date_info.reg_safe_state_.bit.alarm_cargophotos_right_;
                
                msg_date2.alarm_down_limit_ = chassis_reg_buf.read_state_cmd_.date_info.reg_safe_state_.bit.alarm_down_limit_;
                msg_date2.alarm_up_limit_ = chassis_reg_buf.read_state_cmd_.date_info.reg_safe_state_.bit.alarm_up_limit_;

                msg_date2.alarm_forkphotoe_left_ = chassis_reg_buf.read_state_cmd_.date_info.reg_safe_state_.bit.alarm_forkphotoe_left_;
                msg_date2.alarm_forkphotoe_right_ = chassis_reg_buf.read_state_cmd_.date_info.reg_safe_state_.bit.alarm_forkphotoe_right_;
                
                msg_date2.alarm_collision_avoidance_= chassis_reg_buf.read_state_cmd_.date_info.reg_safe_state_.bit.alarm_collision_avoidance_;
                msg_date2.alarm_emergency_stop_swtich_ = chassis_reg_buf.read_state_cmd_.date_info.reg_safe_state_.bit.alarm_emergency_stop_swtich_;
                msg_date2.alarm_micro_swtich_ = chassis_reg_buf.read_state_cmd_.date_info.reg_safe_state_.bit.alarm_micro_swtich_;
                chassis_alarm_.publish(msg_date2);

#if 0

                ROS_INFO("speed_feedback= %d",(int16_t) chassis_reg_buf.read_state_cmd_.date_info.reg_speed_feedback_);
                ROS_INFO("lift_high_feedback= %d",chassis_reg_buf.read_state_cmd_.date_info.reg_lifthigh_feedback_);
                ROS_INFO("angle_feedback= %d",(int16_t)chassis_reg_buf.read_state_cmd_.date_info.reg_angle_feedback_);

                ROS_INFO("safe_date= %x",chassis_reg_buf.read_state_cmd_.date_info.reg_safe_state_);

                ROS_INFO("roate_motor_erro= %d",chassis_reg_buf.read_state_cmd_.date_info.reg_roate_motor_erro_);
                ROS_INFO("self_selfcheck_erro= %d",chassis_reg_buf.read_state_cmd_.date_info.reg_selfcheck_erro_);
                ROS_INFO("walk_motor_erro= %d",chassis_reg_buf.read_state_cmd_.date_info.reg_walk_motor_erro_);
                ROS_INFO("lift_motor_erro= %d",chassis_reg_buf.read_state_cmd_.date_info.reg_lift_motor_erro_);

                ROS_INFO("task_id= %d",chassis_reg_buf.read_state_cmd_.date_info.reg_task_id_feedback_);
                ROS_INFO("task_state= %d",chassis_reg_buf.read_state_cmd_.date_info.reg_task_state_feedback_);

                ROS_INFO("odometer= %d",chassis_reg_buf.read_state_cmd_.date_info.reg_odometer_);
                ROS_INFO("odometer_param= %d",chassis_reg_buf.read_state_cmd_.date_info.reg_odometer_param_);

                //read test bat
                ROS_INFO("update...");

#endif
               
        }

}

void class_chassis_control::update_led_state(const uint16_t colour)
{
        //current state read
        static uint16_t led_state_prior =NULL; 

        if (led_state_prior == colour) return;
        else{

                p_chassis_reg_buf->write_led_cmd_.all_status=colour;//read:2400  blue:1200  green :0900
                chassis_write_registers(REG_INDICATOR, 1,(uint16_t*)&p_chassis_reg_buf->write_led_cmd_.all_status);//FIXME ADD PUBLISH  
                led_state_prior =colour; 
        }   

}

int16_t class_chassis_control::saturation(int16_t u, int16_t min, int16_t max) 
{
	if (u>max) u=max;
	if (u<min) u=min;
	return u; 
}

void class_chassis_control::callback_agvs_manual_control(const agvs_control::date_pads_cmdConstPtr &cmd_control) 
{
	last_command_time_ = ros::Time::now();
        // subs_command_freq->tick(); //for diagnostics need add 

        p_chassis_reg_buf->write_motor_cmd_.date_info.reg_motor_angle = cmd_control->angle_date;
        p_chassis_reg_buf->write_motor_cmd_.date_info.reg_motor_speed = cmd_control->speed_date;
        
        //chassis_motor_cmd(cmd_control->speed_date,cmd_control->angle_date);
        //ROS_INFO("chassis_drive::chassis_cmdConstPtr: agv_vel = %d, agv_angle = %d",p_chassis_reg_buf->write_motor_cmd_.date_info.reg_motor_speed,p_chassis_reg_buf->write_motor_cmd_.date_info.reg_motor_angle);   
        update_chassis_motor_state();      
}

void class_chassis_control::callback_agvs_auto_control(const agvs_control::date_pads_cmdConstPtr &cmd_control)  
{

	last_command_time_ = ros::Time::now();
        p_chassis_reg_buf->write_motor_cmd_.date_info.reg_motor_angle = cmd_control->angle_date;
        p_chassis_reg_buf->write_motor_cmd_.date_info.reg_motor_speed = cmd_control->speed_date;
        
        //chassis_motor_cmd(cmd_control->speed_date,cmd_control->angle_date);
        //ROS_INFO("chassis_drive::chassis_cmdConstPtr: agv_vel = %d, agv_angle = %d",p_chassis_reg_buf->write_motor_cmd_.date_info.reg_motor_speed,p_chassis_reg_buf->write_motor_cmd_.date_info.reg_motor_angle);       
        update_chassis_motor_state();    

}
 
bool class_chassis_control::srvcallback_elevator_raise(std_srvs::Empty::Request &request, std_srvs::Empty::Response &response)
{
        p_chassis_reg_buf->reg_lift_high_=RISE;
	chassis_write_registers(REG_HIGH_CONTROL,1,(uint16_t*)&p_chassis_reg_buf->reg_lift_high_);
	
        ROS_INFO("rise_elevator.>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>"); 
        return true;

}

bool class_chassis_control::srvcallback_elevator_down(std_srvs::Empty::Request &request, std_srvs::Empty::Response &response )
{
	p_chassis_reg_buf->reg_lift_high_=DOWN;
	chassis_write_registers(REG_HIGH_CONTROL,1,(uint16_t*)&p_chassis_reg_buf->reg_lift_high_);
        
	ROS_INFO("land_elevator>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>"); 
	return true;  
}

bool class_chassis_control::srvcallback_aging_test(std_srvs::Empty::Request &request, std_srvs::Empty::Response &response)
{

        event_ageing_test = !event_ageing_test;
#if 0
        if (event_ageing_test == false) chassis_motor_cmd(0.0f,0.0f);
        else {
                chassis_odometer_current =(int32_t)p_chassis_reg_buf->read_state_cmd_.date_info.reg_odometer_;
                chassis_motor_cmd(100.0f,0.0f);
                ROS_INFO("ageing_test_callback\n");
        } 
        
#endif
        return true;
        ROS_INFO("test_moder>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>..."); 

}

void  class_chassis_control::chassis_ageing_test()
{
        
        int32_t odometer_tmp;
        int32_t odometer_test =2000;

        if (event_ageing_test){

                odometer_tmp = (int32_t)p_chassis_reg_buf->read_state_cmd_.date_info.reg_odometer_ - chassis_odometer_current;
                ROS_INFO("odometer= %d",odometer_tmp);
                ROS_INFO("odometer= %d",p_chassis_reg_buf->read_state_cmd_.date_info.reg_odometer_);

                if ((odometer_tmp >= odometer_test )&&(int16_t)p_chassis_reg_buf->read_state_cmd_.date_info.reg_speed_feedback_ <0){
                       
                        chassis_motor_cmd(0.0f,0.0f);
                        p_chassis_reg_buf->reg_lift_high_=DOWN;
	                //chassis_write_registers(REG_HIGH_CONTROL,1,(uint16_t*)&p_chassis_reg_buf->reg_lift_high_);
                        usleep(1000000);

                } else if ((odometer_tmp <= -odometer_test )&&(int16_t)p_chassis_reg_buf->read_state_cmd_.date_info.reg_speed_feedback_ >0){
                        
                        chassis_motor_cmd(0.0f,0.0f);
                        p_chassis_reg_buf->reg_lift_high_=RISE;
	                //chassis_write_registers(REG_HIGH_CONTROL,1,(uint16_t*)&p_chassis_reg_buf->reg_lift_high_);
                        usleep(1000000);

                } else if ((odometer_tmp <= -odometer_test )&&(int16_t)p_chassis_reg_buf->read_state_cmd_.date_info.reg_speed_feedback_ ==0){
                        
                        chassis_motor_cmd(-200.0f,0.0f);

                } else if ((odometer_tmp >= odometer_test )&&(int16_t)p_chassis_reg_buf->read_state_cmd_.date_info.reg_speed_feedback_ ==0){
                                
                        chassis_motor_cmd(200.0f,0.0f);
                        //ROS_INFO("chassis_drive::chassis_cmdConstPtr: agv_vel = %d, agv_angle = %d",p_chassis_reg_buf->write_motor_cmd_.date_info.reg_motor_speed,p_chassis_reg_buf->write_motor_cmd_.date_info.reg_motor_angle);

                } else if ((odometer_tmp == 0)&&(int16_t)p_chassis_reg_buf->read_state_cmd_.date_info.reg_speed_feedback_ ==0){
                       
                        chassis_motor_cmd(-200.0f,0.0f);
                }
        } 
        
}

bool class_chassis_control::loop()
{
	ROS_INFO("chassis_drive_loop");
        ros::Rate r(desired_freq); 

        // Using ros::isShuttingDown to avoid restarting the node during a shutdown
        while (!ros::isShuttingDown()) {

                if (1){

                        ROS_INFO("chassis_drive_while");
                        while(ros::ok() && node_handle_.ok()) {
                                
                                publish_chassis_state(0.2); //0.2s  update
                                publish_bate_state(5);
                                //chassis_ageing_test();
                                update_chassis_led_state(agvs_led_state ,0.5);
                                update_chassis_heart_state(1);

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

        return true;
}

int main(int argc, char **argv)
{
        
	ros::init(argc, argv, "chassis_drive");
	ros::NodeHandle n;		
  	class_chassis_control sxlrc(n);

        sxlrc.loop();

	return (0);

}

