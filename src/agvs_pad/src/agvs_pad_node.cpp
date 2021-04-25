#include <ros/ros.h>
#include <sensor_msgs/Joy.h>  //the date source come from 
#include <geometry_msgs/Twist.h>
#include <std_msgs/Int32.h>
#include <unistd.h>
#include <vector>
#include <std_srvs/Empty.h>

#include "diagnostic_msgs/DiagnosticStatus.h"
#include "diagnostic_updater/diagnostic_updater.h"
#include "diagnostic_updater/update_functions.h"
#include "diagnostic_updater/DiagnosticStatusWrapper.h"
#include "diagnostic_updater/publisher.h"

#include "chassis_drive/chassis_cmd.h"
#include"agvs_control/date_pads_cmd.h"
#include"chassis_drive/cmd_lift.h"

//socket test
#include <sys/socket.h>
#include <sys/types.h>

#define DEFAULT_MAX_SKID_LINEAR_SPEED	280.0 // m/s
#define DEFAULT_MAX_ANGULAR_POSITION	900.0 // rads/s

#define MAX_NUM_OF_BUTTONS		16
#define MAX_NUM_OF_AXES			8

#define MAX_NUM_OF_BUTTONS_F710		19
#define MAX_NUM_OF_AXES_F710		20

#define DEFAULT_NUM_OF_BUTTONS		16
#define DEFAULT_NUM_OF_AXES		8

#define DEFAULT_AXIS_LINEAR_X		3
#define DEFAULT_AXIS_ANGULAR		2	

#define DEFAULT_SCALE_LINEAR		1.0
#define DEFAULT_SCALE_ANGULAR		1.0

#define DEFAULT_HZ			20.0

#define DEFAULT_BUTTON_AGEING_TEST      0

#define DEFAULT_BUTTON_SPEED_UP         6
#define DEFAULT_BUTTON_SPEED_DOWN       2

#define DEFAULT_BUTTON_FORK_UP		3
#define DEFAULT_BUTTON_FORK_DOWN	1

#define DEFAULT_BUTTON_ULOCKA	5
#define DEFAULT_BUTTON_ULOCKB	4

#define EPS_ZERO  1e-6

class Button
{
	int iPressed;
	bool bReleased;
	
public:
	
	Button()
	{
		iPressed = 0;
		bReleased = false;
	}

	//! Set the button as 'pressed'/'released'
	void Press(int value)
	{		
		if(iPressed and !value)
		{
			bReleased = true;

		}else if(bReleased and value)
			bReleased = false;

		iPressed = value;			
	}
	
	int IsPressed()
	{
		return iPressed;
	}
	
	bool IsReleased()
	{
		bool b = bReleased;
		bReleased = false;
		return b;
	}
};

class AgvsPad
{
public:

	ros::NodeHandle nh_;
	AgvsPad();
        void control_loop();
	
private:

        void update();
	void joy_callback(const sensor_msgs::Joy::ConstPtr& joy);
	
	ros::Publisher pub_manual_chassis_cmd_;
	ros::Subscriber sub_joy_;

	ros::ServiceClient client_raise_elevator_;
	ros::ServiceClient client_lower_elevator_;

        ros::ServiceClient client_ageing_;
        ros::ServiceClient client_auto_mode_;

	std::string topic_joy_name;	
	std::string topic_vel_name;
	std::string topic_state_name;

	std::string service_raise_elevator_name;
	std::string service_lower_elevator_name;
        std::string service_ageing_test_name;
	std::string service_auto_mode_name;
	
	std::vector<float> fAxes_;
	std::vector<Button> vButtons_;

	int num_of_buttons_;
	int num_of_axes_;
	int button_ageing_test_;

	int button_speed_up_, button_speed_down_;
	int button_modeswtich_1_, button_modeswtich_2_;
	int button_up_car_, lower_elevator;
        int axis_linear_speed_, axis_angular_position_;

	double max_linear_speed_, max_angular_position_;
        double current_speed_lvl;
	double desired_freq;
	bool handle_ulock_event = 0; //! Flag to enable/disable the handle
};

AgvsPad::AgvsPad():  nh_("~")
{

	current_speed_lvl = -1;

	nh_.param("num_of_buttons", num_of_buttons_, DEFAULT_NUM_OF_BUTTONS);
	nh_.param("num_of_axes", num_of_axes_, DEFAULT_NUM_OF_AXES);
	nh_.param("desired_freq", desired_freq, DEFAULT_HZ);
	
	nh_.param("button_dead_man", button_ageing_test_, DEFAULT_BUTTON_AGEING_TEST);
	nh_.param("button_speed_up", button_speed_up_, DEFAULT_BUTTON_SPEED_UP);
	nh_.param("button_speed_down", button_speed_down_, DEFAULT_BUTTON_SPEED_DOWN); 

	nh_.param("button_modeswtich_1", button_modeswtich_1_, DEFAULT_BUTTON_ULOCKA);
	nh_.param("button_modeswtich_2", button_modeswtich_2_, DEFAULT_BUTTON_ULOCKB);
	
	nh_.param("max_angular_position", max_angular_position_, DEFAULT_MAX_ANGULAR_POSITION); 
	nh_.param("max_linear_speed_", max_linear_speed_, DEFAULT_MAX_SKID_LINEAR_SPEED); 
	
	nh_.param("axis_linear_speed", axis_linear_speed_, DEFAULT_AXIS_LINEAR_X); 
	nh_.param("axis_angular_position", axis_angular_position_, DEFAULT_AXIS_ANGULAR); 

	nh_.param("button_up_car", button_up_car_, DEFAULT_BUTTON_FORK_UP);
	nh_.param("button_down_car", lower_elevator, DEFAULT_BUTTON_FORK_DOWN);

	nh_.param("topic_joy", topic_joy_name, std::string("/joy"));	
	nh_.param("topic_vel_name", topic_vel_name, std::string("/agvs_control/pads_cmd"));
	nh_.param("topic_state", topic_state_name, std::string("/agvs_pad/state"));

	nh_.param("service_raise_elevator", service_raise_elevator_name, std::string("/agvs_pad/raise_elevator"));
	nh_.param("service_lower_elevator", service_lower_elevator_name, std::string("/agvs_pad/lower_elevator"));
        nh_.param("service_ageing_test",service_ageing_test_name,std::string("/agvs_pad/ageing_test"));
        nh_.param("auto_mode",service_auto_mode_name,std::string("/agvs_pad/autmode"));


	if(num_of_axes_ > MAX_NUM_OF_AXES){

		num_of_axes_ = MAX_NUM_OF_AXES;
		ROS_INFO("AgvsPad::AgvsPad: Limiting the max number of axes to %d", MAX_NUM_OF_AXES);

	}
	if(num_of_buttons_ > MAX_NUM_OF_BUTTONS){

		num_of_buttons_ = MAX_NUM_OF_BUTTONS;
		ROS_INFO("AgvsPad::AgvsPad: Limiting the max number of buttons to %d", MAX_NUM_OF_BUTTONS);

	}

	for(int i = 0; i < MAX_NUM_OF_BUTTONS_F710; i++){

		Button b;
		ROS_INFO("add bottom");
		vButtons_.push_back(b);
	}
	
	for(int i = 0; i < MAX_NUM_OF_AXES_F710; i++){

		fAxes_.push_back(0.0);
                ROS_INFO("axes");
	}
        
        this->pub_manual_chassis_cmd_ = nh_.advertise<agvs_control::date_pads_cmd>(this->topic_vel_name, 50); 
	this->sub_joy_ = nh_.subscribe<sensor_msgs::Joy>(topic_joy_name, 1, &AgvsPad::joy_callback, this);

	client_raise_elevator_ = nh_.serviceClient<std_srvs::Empty>(service_raise_elevator_name);
	client_lower_elevator_ = nh_.serviceClient<std_srvs::Empty>(service_lower_elevator_name);
        
        client_ageing_  = nh_.serviceClient<std_srvs::Empty>(service_ageing_test_name);
        client_auto_mode_ = nh_.serviceClient<std_srvs::Empty>(service_auto_mode_name);
}

void AgvsPad::update(){}

void AgvsPad::joy_callback(const sensor_msgs::Joy::ConstPtr& joy)
{
	ROS_INFO("JOY_CALLBACK");
	for(int i = 0; i < joy->axes.size(); i++){

		this->fAxes_[i] = joy->axes[i];
	}

	for(int i = 0; i < joy->buttons.size(); i++){

		this->vButtons_[i].Press(joy->buttons[i]);
	}

	if (vButtons_[button_modeswtich_1_].IsPressed()&&vButtons_[button_modeswtich_2_].IsPressed()){

		handle_ulock_event=!handle_ulock_event;
	}

	//ROS_INFO("AgvsPad::joy_callback: num_of_axes = %d, buttons = %d", (int)(joy->axes.size()), (int)(joy->buttons.size()));
}

void AgvsPad::control_loop(){
        
	agvs_control::date_pads_cmd ref_msg;  
	ros::Rate r(desired_freq);   

        double desired_linear_speed = 0.0;
        double desired_angular_position = 0.0;
        float speed,angle;
        
        while(ros::ok()) {
                        
                if(handle_ulock_event){

                        ROS_INFO("breakpoint_2\n");
#if 0
                        if(fabs(fAxes_[axis_angular_position_])<=EPS_ZERO){
                                speed = fAxes_[axis_linear_speed_];
                                angle =0.0f;

                        } else {
                                speed =sqrt(fAxes_[axis_linear_speed_]*fAxes_[axis_linear_speed_]+fAxes_[axis_angular_position_]*fAxes_[axis_angular_position_]);
                                angle =(atan2(fAxes_[axis_linear_speed_],fAxes_[axis_angular_position_]));

                        }
#endif 
                        desired_linear_speed = max_linear_speed_ * current_speed_lvl * fAxes_[axis_linear_speed_];
                        desired_angular_position = max_angular_position_ * fAxes_[axis_angular_position_];

                        ref_msg.angle_date=desired_angular_position;
                        ref_msg.speed_date=desired_linear_speed;
                        pub_manual_chassis_cmd_.publish(ref_msg);
        
                        if(vButtons_[button_speed_up_].IsReleased()){

                                current_speed_lvl += 0.1;
                                if(current_speed_lvl > 1.0)
                                current_speed_lvl = 1.0;

                        }

                        if(vButtons_[button_speed_down_].IsReleased()){

                                current_speed_lvl -= 0.1;
                                if(current_speed_lvl < 0.0)
                                current_speed_lvl = 0.0;

                        }

                        if (vButtons_[button_up_car_].IsReleased()){  

                                std_srvs::Empty empty_srv;
                                client_raise_elevator_.call(empty_srv);
                                ROS_INFO("Raise elevator");

                        }

                        if (vButtons_[lower_elevator].IsReleased()){

                                std_srvs::Empty empty_srv;
                                client_lower_elevator_.call(empty_srv);
                                ROS_INFO("Lower elevator");

                        }	
                        //ageing_test
                        if(vButtons_[button_ageing_test_].IsReleased()){         

                                std_srvs::Empty empty_srv;
                                client_ageing_.call(empty_srv);
                                client_auto_mode_.call(empty_srv);

                                ROS_INFO("ageing_test!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!\n");
                        }				
                }

                ros::spinOnce();
                r.sleep();
        }   	
}

int main(int argc, char** argv)
{
	ros::init(argc, argv, "agvs_pad");
	AgvsPad pad;

	pad.control_loop();
}

