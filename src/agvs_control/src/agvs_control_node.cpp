#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <sensor_msgs/Joy.h>
#include "chassis_drive/chassis_alarm.h"
#include "chassis_drive/chassis_bat.h"
#include "chassis_drive/chassis_cmd.h"
#include "chassis_drive/chassis_state.h"

#include "agvs_control/cmd_control_task.h"
#include "agvs_control/date_realtime_feedback.h"
#include "agvs_control/cmd_control_mode.h"
//pid related param
#include "cmath"

#include "diagnostic_msgs/DiagnosticStatus.h"
#include "diagnostic_updater/diagnostic_updater.h"
#include "diagnostic_updater/update_functions.h"
#include "diagnostic_updater/DiagnosticStatusWrapper.h"
#include "diagnostic_updater/publisher.h"

//roat move 
#define _CONTROL_ROATE_PID_KP		(1.0f)
#define _CONTROL_ROATE_PID_KI		(0.0f)
#define _CONTROL_ROATE_PID_KD		(1.0f)
#define _CONTROL_ROATE_PID_MIN		(-1000.0f)
#define _CONTROL_ROATE_PID_MAX		(1000.0f)

//straight move
#define _CONTROL_LINE_PID_KP		(1.0f)
#define _CONTROL_LINE_PID_KI		(0.0f)
#define _CONTROL_LINE_PID_KD		(1.0f)
#define _CONTROL_LINE_PID_MIN		(-1000.0f)
#define _CONTROL_LINE_PID_MAX		(1000.0f)

//carbody related param 
#define PI 3.1415926535
#define AGVS_CAR_LENGTH			(1.0f)	    //1m

#define pid_mode                        1

#define max_y_speed                     200
#define max_x_angle                     450

using namespace std;

typedef enum  _control_mode_t
{	
	_CONTROL_MODE_IDLE,			
	_CONTROL_MODE_LASER,		
	_CONTROL_MDOE_T265,			
	_CONTROL_MODE_RADAR,	

}control_mode_t;

typedef  struct _lib_control_iface_t
{
	void *	(*p_os_memset)(void *s, int c, uint16_t count);
	void *	(*p_os_memcpy)(void *dst, const void *src,uint16_t count);
}lib_control_iface_t;

typedef  struct _lib_control_pid_t
{	
	lib_control_iface_t *p_iface;

	float_t kp;
	float_t ki;
	float_t kd;
	
	float_t input;
	float_t feedback;
	float_t output;
	float_t last_error;
	float_t i_sum;
	float_t ts;
	float_t max;
	float_t min;
}lib_control_pid_t;

/**pid:mode 2 tandem**/
typedef struct
{
	float ek;
	float ek1;
	float esum;
	float kp;
	float ki;
	float kd;

} PID_StructureDef;

struct pid_incremental
{
        float kp;
        float ki;
        float kd; 

        float iError;
        float LastError;
        float PrevError;
};

/*******************************class************************************/
class agvs_control_class
{
public:
        ros::NodeHandle node_handle_;
        ros::NodeHandle private_node_handle_;
        double desired_freq_;

	// Diagnostics
/*	diagnostic_updater::Updater diagnostic_;                          // General status diagnostic updater
	diagnostic_updater::FrequencyStatus freq_diag_;                   // Component frequency diagnostics
	diagnostic_updater::HeaderlessTopicDiagnostic *subs_command_freq; // Topic reception frequency diagnostics
	ros::Time last_command_time_;                                     // Last moment when the component received a command
	diagnostic_updater::FunctionDiagnosticTask command_freq_;
*/	
        agvs_control_class(ros::NodeHandle h);

        int starting();

        //callback navigation function
        void navigation_node_callback(const agvs_control::date_realtime_feedbackConstPtr& date_msg);
	void srvcallback_setmode(agvs_control::cmd_control_modeRequest &request,agvs_control::cmd_control_modeResponse &response);
        //callback padsconttrol function
        void pads_node_callback(const chassis_drive::chassis_cmdConstPtr& pad_cmd);

        void auto_control();

        //update function
	void (*update_mode_control_cmd)();
        //publish function
        void publish_chassis_control_cmd();
        //loop function
        void agvs_control_loop();
	  // Robot model
  	std::string robot_model_;	

private:
    /*****date block*****/
	int16_t update_x_control_cmd(float target_distance, float current_distance,float target_angle,    float current_angle);
        int16_t update_y_control_cmd(float target_distance,float current_distance,int16_t target_speed,int16_t current_speed);

        void update_auto_control_cmd();


	void update_manual_control_cmd();
        //inverse kinematics
        void inverse_kinematics(float_t roate_speed ,float_t line_speed ,float_t *p_theta,float_t *p_speed );
        //forward_kinematics
        void forward_kinematics(float_t theta, float_t speed, float_t *p_roate_speed , float_t *p_speed);
        //pid controller
        void lib_pid_postion(lib_control_pid_t *pid); 
        //pid conttroller init 
        void lib_pid_init(float_t kp, float_t ki, float_t kd, lib_control_pid_t *p_pid_control, lib_control_iface_t *p_iface);
        int16_t saturation(int16_t u, int16_t min, int16_t max);
	//mode select
	int PID_Calculate(float error,float gyro,PID_StructureDef *PID_shell_Structure,PID_StructureDef *PID_core_Structure); 

        /*****date block*****/
        ros::Time last_command_time_;

        //publisher 
        ros::Publisher  pub_chassis_cmd;

        //subscriber
        ros::Subscriber sub_navigation_cmd;
        ros::Subscriber sub_pad_cmd;

	//serviceserver
	ros::ServiceServer srv_set_mode_;
	ros::ServiceServer srv_get_moede_;
	ros::ServiceServer srv_raisefork_;
	ros::ServiceServer srv_lowerfork_;

        //string date 
        std::string pub_chassis_cmd_;
        std::string sub_pad_cmd_;
        std::string sub_navigation_cmd_;
        
        //pid variable date 
        _lib_control_iface_t lib_control_ifcae;

        _lib_control_pid_t lib_control_pid_straight;
        _lib_control_pid_t lib_control_pid_roate;
	

	//topic
	chassis_drive::chassis_cmd p_control;  //receive the pad_node cmd
	agvs_control::date_realtime_feedback navigation_feedback_date; //receive the navigation_node cmd
	chassis_drive::chassis_cmd pad_control_date_;

        //new pid 

        pid_incremental *_p_ys_pid =new pid_incremental();
        pid_incremental *_p_yv_pid =new pid_incremental();

        pid_incremental *_p_xs_pid =new pid_incremental();
        pid_incremental *_p_xangle_pid =new pid_incremental();
};

agvs_control_class::agvs_control_class(ros::NodeHandle h):node_handle_(h),private_node_handle_("~"),  desired_freq_(100.0)
{
	// Get robot model from the parameters
        if (!private_node_handle_.getParam("model", robot_model_)){
                ROS_ERROR("Robot model not defined.");
                exit(-1);
        }else  {ROS_INFO("Robot Model : %s", robot_model_.c_str());}
        //param_init 
	ros::NodeHandle agvs_control_node_handle(node_handle_, "agvs_control");

	//control configuration - topics (control actions)
        private_node_handle_.param<std::string>("pub_chassis_cmd_", pub_chassis_cmd_, std::string("/chassis_drive_node/command"));
        private_node_handle_.param<std::string>("sub_chassis_cmd_", sub_pad_cmd_, std::string("/agvs_pad_node/command"));
        private_node_handle_.param<std::string>("sub_navigation_cmd_", sub_navigation_cmd_, std::string("/agvs/navigation_node/command"));

        //publiser init 
	pub_chassis_cmd =private_node_handle_.advertise<chassis_drive::chassis_cmd>(pub_chassis_cmd_,50);

        //subscriber init
	sub_pad_cmd =private_node_handle_.subscribe<chassis_drive::chassis_cmd>("pads_control",1,&agvs_control_class::pads_node_callback,this);
	sub_navigation_cmd=private_node_handle_.subscribe<agvs_control::date_realtime_feedback>("navigation",1,&agvs_control_class::navigation_node_callback,this);
        //general variable init 



}
void agvs_control_class::lib_pid_postion(lib_control_pid_t *p_pid)
{
   
        float_t error =0.0f;
        float_t tmp_output;

        error =p_pid->input-p_pid->feedback;
        #if 0
	//抗积分饱和环节
	if(p_pid->output>=p_pid->max)
	{
		if(error<0.0f)
		{
			p_pid->i_sum+=error*p_pid->ts;
			
		}
	}
	else if(p_pid->output <=p_pid->min)
	{
		if(error>0.0f)
		{
			p_pid->i_sum+=error*p_pid->ts;
			
		}
	}
	else
#endif
        {
                p_pid->i_sum =error*p_pid->ts;
        }

        tmp_output=p_pid->kp*error+p_pid->ki*p_pid->i_sum+p_pid->kd*(error-p_pid->last_error); //pid control

        if(tmp_output >=p_pid->max){
                tmp_output=p_pid->max;

        }else if(tmp_output <=p_pid->min){
                tmp_output=p_pid->min;

        }

        p_pid->last_error=error;
        p_pid->output=tmp_output;
}

void agvs_control_class::forward_kinematics(float_t theta, float_t speed, float_t *p_roate_speed , float_t *p_line_speed)
{
        *p_roate_speed=speed*sin(theta)/AGVS_CAR_LENGTH;
	*p_line_speed=speed*cos(theta);
}

void agvs_control_class::inverse_kinematics(float_t roate_speed ,float_t line_speed ,float_t *p_theta,float_t *p_speed ){

        float_t theta,speed;
        float_t theta_ant;

	theta=atan2((roate_speed)*AGVS_CAR_LENGTH,line_speed);
	speed=line_speed/cos(theta);

	*p_theta=theta;
	*p_speed=speed;
}

int16_t agvs_control_class::saturation(int16_t u, int16_t min, int16_t max) //速度 角度 限幅滤波
{
	if (u>max) u=max;
	if (u<min) u=min;

	return u; 
}

#if (pid_mode!=1)

void agvs_control_class::update_auto_control_cmd()
{
	float theta,speed;

	lib_control_pid_t *p_pid_roate ;	
	lib_control_pid_t *p_pid_line  ;

	//pd控制
	p_pid_roate->input=0.0f;
	p_pid_roate->feedback=-(navigation_feedback_date.theta_angle);//这部分需要调试确认
	lib_pid_postion(p_pid_roate);

	p_pid_line->input=0.0f;
	p_pid_line->feedback=-(navigation_feedback_date.theta_x);//这部分需要调试确认
	lib_pid_postion(p_pid_line);

	p_control.chassis_angle_cmd_= p_pid_roate->output;
        p_control.chassis_vel_cmd_= p_pid_line->output;

	//逆运动学
	inverse_kinematics(p_control.chassis_angle_cmd_,p_control.chassis_vel_cmd_,&theta,&speed); //FIXME error paramtype flaut
	p_control.chassis_angle_cmd_=theta;
	p_control.chassis_vel_cmd_=speed;
}

#endif

void agvs_control_class::update_manual_control_cmd()
{
	float theta,speed;
	//逆运动学
	inverse_kinematics(pad_control_date_.chassis_angle_cmd_,pad_control_date_.chassis_vel_cmd_,&theta,&speed); //FIXME error paramtype flaut
	p_control.chassis_angle_cmd_=theta;
	p_control.chassis_vel_cmd_=speed;

}



/*
int agvs_controlclass::starting()
{
	if (0) {
                vector<string> joint_names = joint_state_.name;
                fwd_vel_ = find (joint_names.begin(),joint_names.end(), string(joint_front_wheel)) - joint_names.begin();
                bwd_vel_ = find (joint_names.begin(),joint_names.end(), string(joint_back_wheel)) - joint_names.begin();
                fwd_pos_ = find (joint_names.begin(),joint_names.end(), string(joint_front_motor_wheel)) - joint_names.begin();
                bwd_pos_ = find (joint_names.begin(),joint_names.end(), string(joint_back_motor_wheel)) - joint_names.begin();
                return 0;
        }else return -1;
}
*/

#if 0
/********************pid mode 2************/
int agvs_control_class::PID_Calculate(float error,float gyro,PID_StructureDef *PID_shell_Structure,PID_StructureDef *PID_core_Structure)
{
	float shell_output,core_output;
	
	PID_shell_Structure->ek=error;
	//积分限幅
	if(PID_shell_Structure->esum>300)
		PID_shell_Structure->esum=300;

	else if(PID_shell_Structure->esum<-300)
		PID_shell_Structure->esum=-300;
                
	else    PID_shell_Structure->esum+=error;

	shell_output=(PID_shell_Structure->ek)*(PID_shell_Structure->kp)+(PID_shell_Structure->esum)*(PID_shell_Structure->ki);
	
	//外环输出，作为内环输入 用陀螺仪当前的角速度作为实际值
	PID_core_Structure->ek=shell_output-gyro;
	//内环积分限幅
	if(PID_core_Structure->esum>500)
		PID_core_Structure->esum=500;

	else if(PID_core_Structure->esum<-500)
		PID_core_Structure->esum=-500;

	else
		PID_core_Structure->esum+=PID_core_Structure->ek;
                
                core_output=(PID_core_Structure->ek)*(PID_core_Structure->kp)+(PID_core_Structure->esum)
                *(PID_core_Structure->ki)+(PID_core_Structure->ek-PID_core_Structure->ek1)*(PID_core_Structure->kd);
                
                PID_core_Structure->ek1=PID_core_Structure->ek;
	
	        return (int)core_output;
}
#endif

#if (pid_mode==1)
        //串级PID控制	

	float lib_pid_incremental(float NextPoint, float TargetVal, volatile pid_incremental *ptr)
	{
		float iError = 0.0f, iIncpid = 0.0f; 

		iError = TargetVal - NextPoint;	
		if ((iError < 0.8f) && (iError > -0.8f)) iError = 0.0f;	
                						
		iIncpid = (ptr->kp * iError)- (ptr->ki * ptr->LastError)+ (ptr->kd * ptr->PrevError); 
	
		ptr->PrevError = ptr->LastError; 
		ptr->LastError = iError;
		return (iIncpid);
	}

        int16_t agvs_control_class::update_y_control_cmd(float target_distance,float current_distance,
                                                         int16_t target_speed,int16_t current_speed )
        {
                float dis_exp_val =0.0f, vel_exp_val=0.0f ;

                //位置环
                dis_exp_val = lib_pid_incremental(current_distance, target_distance, _p_ys_pid);//pid

                if (dis_exp_val <= 30.0f) dis_exp_val = 0.0f;
                if (dis_exp_val >= target_speed) dis_exp_val = target_speed;

                //速度环
                vel_exp_val += lib_pid_incremental(current_speed, target_speed, _p_yv_pid);//pid
                if (vel_exp_val <= 10.0f) vel_exp_val = 0.0f;
                if (vel_exp_val >= max_y_speed)   vel_exp_val = max_y_speed;

                return (int16_t)vel_exp_val;
        }

        int16_t agvs_control_class::update_x_control_cmd(float target_distance, float current_distance,
                                                      float target_angle,    float current_angle)
        {
                float dis_exp_val =0.0f, angle_exp_val=0.0f ;

                //position
                dis_exp_val = lib_pid_incremental(current_distance, target_distance, _p_xs_pid);//pid

                if (dis_exp_val <= 30.0f) dis_exp_val = 0.0f;
                if (dis_exp_val >= target_angle) dis_exp_val = target_angle;

                //angle 
                angle_exp_val += lib_pid_incremental(current_angle, target_angle, _p_xangle_pid);//pid  //TODO 

                if (angle_exp_val <= 10.0f) angle_exp_val = 0.0f;
                if (angle_exp_val >= 150.0f) angle_exp_val = 150.0f;
                
                return (int16_t)angle_exp_val;
        }

        void agvs_control_class::auto_control()
        {
                float theta,speed;
                
                static uint8_t angle_pid_flag =0;
                float feedback_y_distance =0.0f,feedback_y_speed =0.0f;
                float feedback_x_distance =0.0f,feedback_x_angle =0.0f;

                p_control.chassis_vel_cmd_ = update_y_control_cmd( 1000.0f, navigation_feedback_date.theta_y, max_y_speed, navigation_feedback_date.speed_y);

                //if the x distance is small ,keep straight line running
                if((navigation_feedback_date.theta_x >= 30.0f)||(angle_pid_flag ==1) ){
                        
                        p_control.chassis_angle_cmd_ = update_x_control_cmd( 0.0f, feedback_x_distance, 0.0f, feedback_x_angle);
                        angle_pid_flag = 1;
                        
                        if(navigation_feedback_date.theta_x <=10.0f) {
                                angle_pid_flag =0;
                                p_control.chassis_angle_cmd_ =0.0f;
                        }

                } else p_control.chassis_angle_cmd_ = 0.0f;
  
                inverse_kinematics(p_control.chassis_angle_cmd_,p_control.chassis_vel_cmd_,&theta,&speed); //FIXME error paramtype flaut
                p_control.chassis_angle_cmd_=theta;
                p_control.chassis_vel_cmd_=speed;

                //publish theta ,speed
        
        }

#endif

void agvs_control_class::publish_chassis_control_cmd()
{
	pub_chassis_cmd.publish(p_control);
}

void agvs_control_class::pads_node_callback(const chassis_drive::chassis_cmdConstPtr& pad_cmd)
{
	
        // Safety check
        last_command_time_ = ros::Time::now();
        //subs_command_freq->tick(); // For diagnostics
        
        double speed_limit = 200.0; // m/s
        double angle_limit = PI;

        pad_control_date_.chassis_angle_cmd_ = saturation(pad_cmd->chassis_vel_cmd_,-speed_limit, speed_limit);    //FIXME the unit need unified
        pad_control_date_.chassis_vel_cmd_ = saturation(pad_cmd->chassis_angle_cmd_, -angle_limit, angle_limit); 
}

void agvs_control_class::navigation_node_callback(const agvs_control::date_realtime_feedbackConstPtr& date_msg){

	navigation_feedback_date.theta_angle=date_msg->theta_angle;
	navigation_feedback_date.theta_x=date_msg->theta_x;
	navigation_feedback_date.theta_y=date_msg->theta_y;
}

void agvs_control_class::srvcallback_setmode(agvs_control::cmd_control_modeRequest &request,agvs_control::cmd_control_modeResponse &response) //TODO 
{
	robot_model_= request.mode_run;
}

void agvs_control_class::agvs_control_loop()
{
	  
        ROS_INFO("agvs_robot_control::agvs_control_loop()");
        ros::Rate r(desired_freq_);  // 50.0 

        // Using ros::isShuttingDown to avoid restarting the node during a shutdown.
        while (!ros::isShuttingDown()) {
                        if (1)//if (starting() == 0)  //FIXME navigation mode 
                        {
                                while(ros::ok() && node_handle_.ok()){
                                        //control logic code
                                        update_mode_control_cmd();
                                        publish_chassis_control_cmd();
                                        ros::spinOnce();
                                        r.sleep();
                                }

                                ROS_INFO("END OF ros::ok() !!!");
                        } else {
                                // No need for diagnostic here since a broadcast occurs in start
                                // when there is an error.
                                usleep(1000000);
                                ros::spinOnce();
                        }
                }

        ROS_INFO("agvs_robot_control::spin() - end");
}

int main(int argc, char** argv)
{
	ros::init(argc, argv, "agvs_robot_control");

	ros::NodeHandle n;		
  	agvs_control_class sxlrc(n);

        sxlrc.agvs_control_loop();
	return (0);
}