#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <sensor_msgs/Joy.h>
#include <std_srvs/Empty.h>
#include "cmath"
#include "chassis_drive/chassis_alarm.h"
#include "chassis_drive/chassis_bat.h"
#include "chassis_drive/chassis_cmd.h"
#include "chassis_drive/chassis_state.h"

#include "agvs_control/cmd_control_task.h"
#include "agvs_control/slam_data.h"
#include "agvs_control/cmd_control_mode.h"
#include"agvs_control/date_pads_cmd.h"
#include "agvs_task/route_target.h"

#include "diagnostic_msgs/DiagnosticStatus.h"
#include "diagnostic_updater/diagnostic_updater.h"
#include "diagnostic_updater/update_functions.h"
#include "diagnostic_updater/DiagnosticStatusWrapper.h"
#include "diagnostic_updater/publisher.h"
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
#define AGVS_CAR_LENGTH			(1.20f)	    //1m

#define PID_MODE                       0

#define MAX_Y_SPEED                    200
#define MAX_X_ANGLE                    450

using namespace std;

typedef enum  _control_mode_t
{	
	_CONTROL_MODE_IDLE,			
	_CONTROL_MODE_LASER,		
	_CONTROL_MDOE_T265,			
	_CONTROL_MODE_RADAR,	

}control_mode_t;
typedef  struct _lib_control_pid_t
{	
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
        
        agvs_control_class(ros::NodeHandle h);
        int starting();

        void navigation_node_callback(const agvs_control::slam_dataConstPtr& date_msg);
        void avgs_task_route_callback(const agvs_task::route_targetConstPtr& date_msg);
        void pads_node_callback(const chassis_drive::chassis_cmdConstPtr& pad_cmd);

        void incremental_control();
        void run_route_test(float distance);
        void run_route_task();
        void agvs_control_loop();

private:

	int16_t update_x_control_cmd(float target_distance, float current_distance,float target_angle,    float current_angle);
        int16_t update_y_control_cmd(float target_distance,float current_distance,int16_t target_speed,int16_t current_speed);
       
        
        bool automode_srvcallback(std_srvs::Empty::Request &request, std_srvs::Empty::Response &response);
	
        //inverse kinematics
        void inverse_kinematics(float_t roate_speed ,float_t line_speed ,float_t *p_theta,float_t *p_speed );
        //forward_kinematics
        void forward_kinematics(float_t theta, float_t speed, float_t *p_roate_speed , float_t *p_speed);
        //pid controller
        void lib_pid_postion(lib_control_pid_t *pid);
        
        int16_t saturation(int16_t u, int16_t min, int16_t max);
        
        ros::Time last_command_time_;

        ros::Publisher  pub_chassis_cmd;

        ros::Subscriber sub_navigation_cmd;
        ros::Subscriber sub_pad_cmd;
        ros::Subscriber sub_route_cmd;

	//serviceserver
	ros::ServiceServer srv_set_mode_;
	ros::ServiceServer srv_get_mode_;
	ros::ServiceServer srv_raise_fork_;
	ros::ServiceServer srv_lower_fork_;
        ros::ServiceServer srv_auto_mode_;

        //string name
        std::string pub_chassis_cmd_;
        std::string sub_pad_cmd_;
        std::string sub_navigation_cmd_;

        //topic 
	agvs_control::date_pads_cmd p_control_msg ;  //receive the pad_node cmd
	agvs_control::slam_data navigation_feedback_msg; //receive the navigation_node cmd
	chassis_drive::chassis_cmd pad_control_msg;
        agvs_task::route_target task_route_msg;

        _lib_control_pid_t lib_control_pid_y;
        _lib_control_pid_t lib_control_pid_x;
        _lib_control_pid_t lib_control_pid_angle;

        //incremental pid 
        pid_incremental *_p_ys_pid =new pid_incremental();
        pid_incremental *_p_yv_pid =new pid_incremental();

        pid_incremental *_p_xs_pid =new pid_incremental();
        pid_incremental *_p_xangle_pid =new pid_incremental();
        
        double desired_freq =15.0f;
        float launch_test =0.0f;
        float odometer_current_date= 0.0f;

        bool auto_mode_srv_event= false;     
        bool task_route_msg_event = false;
        
};

agvs_control_class::agvs_control_class(ros::NodeHandle h):node_handle_(h),private_node_handle_("~")
{
        //param_init 
	ros::NodeHandle agvs_control_node_handle(node_handle_, "agvs_control");

	//control configuration - topics (control actions)
        private_node_handle_.param<std::string>("pub_chassis_cmd_", pub_chassis_cmd_, std::string("/auto/cmd"));
        private_node_handle_.param<std::string>("sub_navigation_cmd_", sub_navigation_cmd_, std::string("/trajectory_data"));

        //parameter test
        bool test2 = private_node_handle_.getParam("launch_test",launch_test);

        //subscriber init
        sub_navigation_cmd=private_node_handle_.subscribe<agvs_control::slam_data>(sub_navigation_cmd_,1,&agvs_control_class::navigation_node_callback,this);       
        sub_route_cmd=private_node_handle_.subscribe<agvs_task::route_target>(sub_navigation_cmd_,1,&agvs_control_class::avgs_task_route_callback,this); 
        srv_auto_mode_ =private_node_handle_.advertiseService("/agvs_pad/ageing_test",&agvs_control_class::automode_srvcallback,this);

        //publiser init 
	pub_chassis_cmd =private_node_handle_.advertise<agvs_control::date_pads_cmd>(this->pub_chassis_cmd_,50);

        memset(&p_control_msg,0,sizeof(p_control_msg));

        _p_ys_pid->kd =2.0;

        lib_control_pid_y={

                kp:0.4f,
                ki:0.0f,
                kd:0.0f,

                input:0.0f,
                feedback:0.0f,
                output:0.0f,

                last_error:0.0f,
                i_sum:0.0f,
                ts:1.0f, 
                max:400.0f,
                min:-400.0f                 
        };

        lib_control_pid_x={

                kp:0.1f,
                ki:0.0f,
                kd:0.0f,

                input:0.0f,
                feedback:0.0f,
                output:0.0f,

                last_error:0.0f,
                i_sum:0.0f,
                ts:20.0f, 
                max:200.0f,
                min:-200.0f                 
        };
        
        lib_control_pid_angle={

                kp:30.0f,
                ki:0.0f,
                kd:0.0f,

                input:0.0f,
                feedback:0.0f,
                output:0.0f,
                
                last_error:0.0f,
                i_sum:0.0f,
                ts:10.0f, 
                max:450.0f,
                min:-450.0f   
        };

}

void agvs_control_class::lib_pid_postion(lib_control_pid_t *p_pid)
{
   
        float_t error =0.0f;
        float_t tmp_output;

        error =p_pid->input-p_pid->feedback;
       if (abs(error*p_pid->ts) <=0.2f) {
                p_pid->output=0.0f;
                return;
       }
#if 0
	//抗积分饱和环
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

void agvs_control_class::inverse_kinematics(float_t roate_speed ,float_t line_speed ,float_t *p_theta,float_t *p_speed )
{
        float_t theta,speed;
        float_t theta_ant;

	theta=atan((roate_speed)*AGVS_CAR_LENGTH/line_speed);
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

void agvs_control_class::run_route_test(float distance)
{
        if (auto_mode_srv_event){

                float theta,speed;
                float distance_feedback_y;

                //distance_feedback_y= navigation_feedback_msg.theta_y- odometer_current_date;
                distance_feedback_y= navigation_feedback_msg.theta_y;
                ROS_INFO("navigation_feedback_msg.angle= %f",distance_feedback_y);

                lib_control_pid_t *p_pid_roate =&lib_control_pid_angle;	
                lib_control_pid_t *p_pid_line_y=&lib_control_pid_y;
                lib_control_pid_t *p_pid_line_x=&lib_control_pid_x;

	        //x direction pid control
                p_pid_line_x->input=0.0f;
                p_pid_line_x->feedback=navigation_feedback_msg.theta_x;
                lib_pid_postion(p_pid_line_x);
                ROS_INFO("p_pid_line_x = %f\n",-p_pid_line_x->output);
                p_pid_roate->input=-p_pid_line_x->output;
                p_pid_roate->feedback=navigation_feedback_msg.theta_angle;
                lib_pid_postion(p_pid_roate);
                p_control_msg.angle_date= p_pid_roate->output;

                //y direction pid control
                p_pid_line_y->input=distance;
                p_pid_line_y->feedback=distance_feedback_y; 
                lib_pid_postion(p_pid_line_y);
                p_control_msg.speed_date= -(p_pid_line_y->output);
                
                ROS_INFO("angle = %f\n",p_control_msg.angle_date);
                ROS_INFO("speed = %f\n",p_control_msg.speed_date);

               // inverse_kinematics(p_control_msg.angle_date,p_control_msg.speed_date,&theta,&speed); //FIXME error paramtype flaut
               // p_control_msg.angle_date=theta;
               // p_control.speed_date=speed;

                ROS_INFO("theta = %f\n",theta);
                ROS_INFO("speed = %f\n",speed);

                if (abs(p_control_msg.speed_date)<=2.0f) {

                        auto_mode_srv_event= false;
                        p_control_msg.speed_date=0.0f;
                        p_control_msg.angle_date=0.0f;
                        pub_chassis_cmd.publish(p_control_msg); 
                        return ;

                }

                pub_chassis_cmd.publish(p_control_msg);   

        } else {

                p_control_msg.angle_date=0;
                p_control_msg.speed_date=0;
                pub_chassis_cmd.publish(p_control_msg);       
        }
}

void agvs_control_class::run_route_task()
{
        float theta,speed;
        float distance_feedback_y;
        float distance_target_y;
        
        //distance_feedback_y= navigation_feedback_msg.theta_y- odometer_current_date;
        distance_target_y = task_route_msg.target_location_y;
        distance_feedback_y= navigation_feedback_msg.theta_y;
        ROS_INFO("navigation_feedback_msg.angle= %f",distance_feedback_y);

        lib_control_pid_t *p_pid_roate =&lib_control_pid_angle;	
        lib_control_pid_t *p_pid_line_y=&lib_control_pid_y;
        lib_control_pid_t *p_pid_line_x=&lib_control_pid_x;

        //x direction pid control 
        p_pid_line_x->input=0.0f;
        p_pid_line_x->feedback=navigation_feedback_msg.theta_x;
        lib_pid_postion(p_pid_line_x);
        ROS_INFO("p_pid_line_x = %f\n",-p_pid_line_x->output);
        p_pid_roate->input=-p_pid_line_x->output;
        p_pid_roate->feedback=navigation_feedback_msg.theta_angle;
        lib_pid_postion(p_pid_roate);
        p_control_msg.angle_date= p_pid_roate->output;

        //y direction pid control
        p_pid_line_y->max = task_route_msg.target_speed;
        p_pid_line_y->min = -task_route_msg.target_speed;
        p_pid_line_y->input=distance_target_y;
        p_pid_line_y->feedback=distance_feedback_y; 
        lib_pid_postion(p_pid_line_y);
        p_control_msg.speed_date= -(p_pid_line_y->output);

        ROS_INFO("angle = %f\n",p_control_msg.angle_date);
        ROS_INFO("speed = %f\n",p_control_msg.speed_date);

        // inverse_kinematics(p_control_msg.angle_date,p_control_msg.speed_date,&theta,&speed); //FIXME error paramtype flaut
        // p_control_msg.angle_date=theta;
        // p_control.speed_date=speed;

        ROS_INFO("theta = %f\n",theta);
        ROS_INFO("speed = %f\n",speed);

        if (abs(p_control_msg.speed_date)<=2.0f) {

                task_route_msg_event =false; 

                p_control_msg.speed_date=0.0f;
                p_control_msg.angle_date=0.0f;
                pub_chassis_cmd.publish(p_control_msg); 
                
                return ;
        }

        pub_chassis_cmd.publish(p_control_msg);   
}

/********* pid mode 2:incremental pid **********/
#if 1
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

                //位置�??
                dis_exp_val = lib_pid_incremental(current_distance, target_distance, _p_ys_pid);//pid

                if (dis_exp_val <= 30.0f) dis_exp_val = 0.0f;
                if (dis_exp_val >= target_speed) dis_exp_val = target_speed;

                //速度�??
                vel_exp_val += lib_pid_incremental(current_speed, dis_exp_val, _p_yv_pid);//pid
                if (vel_exp_val <= 10.0f) vel_exp_val = 0.0f;
                if (vel_exp_val >= MAX_Y_SPEED)   vel_exp_val = MAX_Y_SPEED;

                return (int16_t)vel_exp_val;
        }

        int16_t agvs_control_class::update_x_control_cmd(float target_distance, float current_distance,
                                                      float target_angle,    float current_angle)
        {
                float dis_exp_val =0.0f, angle_exp_val=0.0f ;

                //position git
                dis_exp_val = lib_pid_incremental(current_distance, target_distance, _p_xs_pid);//pid

                if (dis_exp_val <= 30.0f) dis_exp_val = 0.0f;
                if (dis_exp_val >= target_angle) dis_exp_val = target_angle;

                //angle
                angle_exp_val += lib_pid_incremental(current_angle, dis_exp_val, _p_xangle_pid);//pid  //TODO 

                if (angle_exp_val <= 10.0f) angle_exp_val = 0.0f;
                if (angle_exp_val >= 150.0f) angle_exp_val = 150.0f;
                
                return (int16_t)angle_exp_val;
        }

        void agvs_control_class::incremental_control()
        {
                float theta,speed;
               // float target_distacne = navigation_feedback_msg.theta_y;

                static uint8_t angle_pid_flag =0;

                if (auto_mode_srv_event){
                        p_control_msg.speed_date = update_y_control_cmd( 1000.0f, navigation_feedback_msg.theta_y, MAX_Y_SPEED, navigation_feedback_msg.speed_y);

                        //if the x distance is small ,keep straight line running
#if 0
                        
                        if((navigation_feedback_msg.theta_x >= 30.0f)||(angle_pid_flag ==1) ){ 
                                
                                p_control.angle_date= update_x_control_cmd( 0.0f, navigation_feedback_msg.speed_x, 0.0f, navigation_feedback_msg.theta_angle);
                                angle_pid_flag = 1;
                                
                                if(navigation_feedback_msg.theta_x <=10.0f) {
                                        angle_pid_flag =0;
                                        p_control.angle_date =0.0f;
                                }

                        } else p_control.angle_date = 0.0f;

                        inverse_kinematics(p_control.angle_date,p_control.speed_date,&theta,&speed); //FIXME error paramtype flaut
                
                        p_control.angle_date=theta;
                        p_control.speed_date=speed;
                        pub_chassis_cmd.publish(p_control);
                         ROS_INFO("navigation runing\n");
#else
                        p_control_msg.angle_date=p_control_msg.angle_date;
                        p_control_msg.speed_date=p_control_msg.speed_date;
                        pub_chassis_cmd.publish(p_control_msg);

#endif 

                } else {
                        p_control_msg.angle_date=0;
                        p_control_msg.speed_date=0;
                        pub_chassis_cmd.publish(p_control_msg);

                }

                //publish theta ,speed
        }

#endif

//navigation message recive function
void agvs_control_class::navigation_node_callback(const agvs_control::slam_dataConstPtr& date_msg)
{
        navigation_feedback_msg.theta_angle=date_msg->theta_angle;
	navigation_feedback_msg.theta_y=date_msg->theta_y;
	navigation_feedback_msg.theta_x=date_msg->theta_x;
        navigation_feedback_msg.speed_y=date_msg->speed_y;

        ROS_INFO("navigation_feedback_msg.theta_x= %f",navigation_feedback_msg.theta_x);
        ROS_INFO("navigation_feedback_msg.theta_y= %f",navigation_feedback_msg.theta_y);
        ROS_INFO("navigation_feedback_msg.angle= %f",navigation_feedback_msg.theta_angle);  
}

void agvs_control_class::avgs_task_route_callback(const agvs_task::route_targetConstPtr& date_msg)
{
        task_route_msg.target_location_x = date_msg->target_location_x;
        task_route_msg.target_location_y = date_msg->target_location_y;
        task_route_msg.target_speed = date_msg->target_speed;
        task_route_msg.task_direction =date_msg->task_direction;
        task_route_msg.task_route_id = date_msg->task_route_id;
        
        this->task_route_msg_event =true;
        ROS_INFO("task_route_msg receive complete");
}

//navigation mode flag
bool agvs_control_class::automode_srvcallback(std_srvs::Empty::Request &request, std_srvs::Empty::Response &response)
{
        auto_mode_srv_event = !auto_mode_srv_event;

        odometer_current_date =navigation_feedback_msg.theta_y;
        ROS_INFO("auto control is alread running>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>\n");
        return true;
}

void agvs_control_class::agvs_control_loop()
{
        ROS_INFO("agvs_robot_control::agvs_control_loop()");
        ros::Rate r(desired_freq);  // 15.0 

        // Using ros::isShuttingDown to avoid restarting the node during a shutdown.
        while (!ros::isShuttingDown()) {
                        if (1)//if (starting() == 0)  //FIXME navigation mode 
                        {
                                while(ros::ok() && node_handle_.ok()){

                                        ROS_INFO("agvs_control_ok");
                                        ROS_INFO("agvs_control_ok=%f",lib_control_pid_y.kp);
                                        //control logic code
                                        
                                        //ROS_INFO("auto_cmd_angle = %f", p_control.angle_date);
                                        //ROS_INFO("auto_cmd_vel = %f", p_control.speed_date); 
                                        
                                        //auto_control();0
                                        run_route_test(0.0f);

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

        ROS_INFO("agvs_control::spin() - end");
}

int main(int argc, char** argv)
{
	ros::init(argc, argv, "agvs_control");

	ros::NodeHandle n;		
  	agvs_control_class sxlrc(n);

        sxlrc.agvs_control_loop();
	return (0);
}