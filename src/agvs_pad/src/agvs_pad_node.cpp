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
#include "agvs_control/date_pads_cmd.h"
#include "chassis_drive/cmd_lift.h"

//socket test
#include <sys/socket.h>
#include <sys/types.h>

#define PI 3.1415926535
#define DEFAULT_HZ			20.0

#define DEFAULT_MAX_LINEAR_SPEED	800.0 // m/s
#define DEFAULT_MAX_ANGULAR_POSITION	900.0 // rads/s

#define MAX_NUM_OF_BUTTONS		16    //F710
#define MAX_NUM_OF_AXES			8


#define DEFAULT_NUM_OF_BUTTONS		16
#define DEFAULT_NUM_OF_AXES		8


#define DEFAULT_BUTTON_AGEING_TEST      0

#define DEFAULT_BUTTON_AXIS_LINEAR_X    3
#define DEFAULT_BUTTON_AXIS_ANGULAR     2

#define DEFAULT_BUTTON_SPEED_UP         7
#define DEFAULT_BUTTON_SPEED_DOWN       6

#define DEFAULT_BUTTON_FORK_UP		3
#define DEFAULT_BUTTON_FORK_DOWN	1

#define DEFAULT_BUTTON_ULOCKA	        5
#define DEFAULT_BUTTON_ULOCKB	        4

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

class class_agvs_pad
{
public:

        ros::NodeHandle node_handle_;
	ros::NodeHandle private_node_handle_;
	class_agvs_pad(ros::NodeHandle h);
        void loop();
	
private:

        void publish_manaul_cmd();
	void callback_joy_node(const sensor_msgs::Joy::ConstPtr& joy);
	
	ros::Publisher pub_manual_mode_cmd_;
	ros::Subscriber sub_joy_;

	ros::ServiceClient client_raise_elevator_;
	ros::ServiceClient client_lower_elevator_;

        ros::ServiceClient client_ageing_test_;
        ros::ServiceClient client_auto_mode_;

	std::string name_sub_joy;	
	std::string name_pub_vel;

	std::string name_client_raise_fork;
	std::string name_client_down_fork;
        std::string name_service_ageing_test;
	std::string name_service_auto_mode;
	
	std::vector<float> fAxes_;
	std::vector<Button> vButtons_;

	int num_of_buttons;
	int num_of_axes;
	int button_ageing_test;

	int button_speed_up, button_speed_down;
	int button_ulock_a, button_ulock_b;
	int button_raise_fork, button_dwon_fork;
        int axis_linear_speed, axis_angular_position;

	double max_linear_speed, max_angular_position;
        double current_speed_lvl;
	double desired_freq;
	bool handle_ulock_event = 0; //! Flag to enable/disable the handle
};

class_agvs_pad::class_agvs_pad(ros::NodeHandle h): node_handle_(h),  private_node_handle_("~"),current_speed_lvl(0.3)
{

	private_node_handle_.param("num_of_buttons", num_of_buttons, MAX_NUM_OF_BUTTONS);
	private_node_handle_.param("num_of_axes", num_of_axes, MAX_NUM_OF_AXES);
	private_node_handle_.param("desired_freq", desired_freq, DEFAULT_HZ);
	
	private_node_handle_.param("button_test", button_ageing_test, DEFAULT_BUTTON_AGEING_TEST);
	private_node_handle_.param("button_speed_up", button_speed_up, DEFAULT_BUTTON_SPEED_UP);
	private_node_handle_.param("button_speed_down", button_speed_down, DEFAULT_BUTTON_SPEED_DOWN); 
	private_node_handle_.param("button_modeswtich_1", button_ulock_a, DEFAULT_BUTTON_ULOCKA);
	private_node_handle_.param("button_modeswtich_2", button_ulock_b, DEFAULT_BUTTON_ULOCKB);
	private_node_handle_.param("button_up_fork", button_raise_fork, DEFAULT_BUTTON_FORK_UP);
	private_node_handle_.param("button_down_fork", button_dwon_fork, DEFAULT_BUTTON_FORK_DOWN);
	private_node_handle_.param("axis_linear_speed", axis_linear_speed, DEFAULT_BUTTON_AXIS_LINEAR_X); 
	private_node_handle_.param("axis_angular_position", axis_angular_position, DEFAULT_BUTTON_AXIS_ANGULAR); 

        private_node_handle_.param("max_angular_position", max_angular_position, DEFAULT_MAX_ANGULAR_POSITION); 
	private_node_handle_.param("max_linear_speed", max_linear_speed, DEFAULT_MAX_LINEAR_SPEED); 

	private_node_handle_.param("name_sub_joy", name_sub_joy, std::string("/joy"));	
	private_node_handle_.param("name_pub_vel", name_pub_vel, std::string("/agvs_control/pads_cmd"));

	private_node_handle_.param("service_raise_fork", name_client_raise_fork, std::string("/agvs_pad/button_raise_fork"));
	private_node_handle_.param("service_lower_fork", name_client_down_fork, std::string("/agvs_pad/button_dwon_fork"));

        private_node_handle_.param("name_service_ageing_test",name_service_ageing_test,std::string("/agvs_pad/ageing_test"));
        private_node_handle_.param("name_service_auto_mode",name_service_auto_mode,std::string("/agvs_pad/automode"));

        pub_manual_mode_cmd_ = private_node_handle_.advertise<agvs_control::date_pads_cmd>(name_pub_vel, 50); 
	sub_joy_ = private_node_handle_.subscribe<sensor_msgs::Joy>(name_sub_joy,1,&class_agvs_pad::callback_joy_node,this);

	client_raise_elevator_ = private_node_handle_.serviceClient<std_srvs::Empty>(name_client_raise_fork);
	client_lower_elevator_ = private_node_handle_.serviceClient<std_srvs::Empty>(name_client_down_fork);
        
        client_ageing_test_  = private_node_handle_.serviceClient<std_srvs::Empty>(name_service_ageing_test);
        client_auto_mode_ = private_node_handle_.serviceClient<std_srvs::Empty>(name_service_auto_mode);

	for(int i = 0; i < MAX_NUM_OF_BUTTONS; i++){

		Button b;
		ROS_INFO("add bottom");
		vButtons_.push_back(b);
	}
	
	for(int i = 0; i < MAX_NUM_OF_AXES; i++){

		fAxes_.push_back(0.0);
                ROS_INFO("axes");
	}
}

void class_agvs_pad::publish_manaul_cmd(){
        
        agvs_control::date_pads_cmd manual_cmd_msg;  

        double desired_linear_speed = 0.0;
        double desired_angular_position = 0.0;
                        
#if 0
        //the feature of thequadrant about atan2:http://c.biancheng.net/ref/atan2.html  
        fAxes_[axis_angular_position] = (atan2(fAxes_[axis_linear_speed],fAxes_[axis_angular_position]));

        if ((fAxes_[axis_angular_position] > EPS_ZERO) && (fAxes_[axis_angular_position] <= (PI/2))){            //quadrant 1

                fAxes_[axis_linear_speed] = sqrt(fAxes_[axis_linear_speed]*fAxes_[axis_linear_speed]+fAxes_[axis_angular_position]*fAxes_[axis_angular_position]);
                fAxes_[axis_angular_position] = ( 1 - fAxes_[axis_angular_position] / (PI/2) );

        } else if ((fAxes_[axis_angular_position] > (PI/2)) && (fAxes_[axis_angular_position] <= PI)){           //quadrant 2

                fAxes_[axis_linear_speed] = sqrt(fAxes_[axis_linear_speed]*fAxes_[axis_linear_speed]+fAxes_[axis_angular_position]*fAxes_[axis_angular_position]);
                fAxes_[axis_angular_position] = -( (fAxes_[axis_angular_position]-PI/2) / (PI/2) );

        } else if ((fAxes_[axis_angular_position] > (-PI)) && (fAxes_[axis_angular_position] <= (-PI/2))){       //quadrant 3

                fAxes_[axis_linear_speed] = -sqrt(fAxes_[axis_linear_speed]*fAxes_[axis_linear_speed]+fAxes_[axis_angular_position]*fAxes_[axis_angular_position]);
                fAxes_[axis_angular_position] = (fAxes_[axis_angular_position] + (PI/2) ) / (PI/2) ;

        } else if ((fAxes_[axis_angular_position] > (-PI/2)) && (fAxes_[axis_angular_position] <= (EPS_ZERO))){  //quadrant 4

                fAxes_[axis_linear_speed] = -sqrt(fAxes_[axis_linear_speed]*fAxes_[axis_linear_speed]+fAxes_[axis_angular_position]*fAxes_[axis_angular_position]);
                fAxes_[axis_angular_position] =(fAxes_[axis_angular_position] + (PI/2))/(PI/2) ;

        }


#else
/*
        if(fabs(fAxes_[axis_angular_position]) <= EPS_ZERO){

                fAxes_[axis_linear_speed] = fAxes_[axis_linear_speed];
                fAxes_[axis_angular_position] =0.0f;

        } else if ((fAxes_[axis_angular_position] > EPS_ZERO) && (fAxes_[axis_linear_speed] > EPS_ZERO) ){      //quadrant 1

                fAxes_[axis_linear_speed] = sqrt(fAxes_[axis_linear_speed]*fAxes_[axis_linear_speed]+fAxes_[axis_angular_position]*fAxes_[axis_angular_position]);
                fAxes_[axis_angular_position] = (atan2(fAxes_[axis_linear_speed],fAxes_[axis_angular_position]));

        } else if ((fAxes_[axis_angular_position] < EPS_ZERO) && (fAxes_[axis_linear_speed] > EPS_ZERO) ){      

                fAxes_[axis_linear_speed] = sqrt(fAxes_[axis_linear_speed]*fAxes_[axis_linear_speed]+fAxes_[axis_angular_position]*fAxes_[axis_angular_position]);
                fAxes_[axis_angular_position] = (atan2(fAxes_[axis_linear_speed],fAxes_[axis_angular_position]));

        } else if ((fAxes_[axis_angular_position] < EPS_ZERO) && (fAxes_[axis_linear_speed] < EPS_ZERO) ){

                fAxes_[axis_linear_speed] = sqrt(fAxes_[axis_linear_speed]*fAxes_[axis_linear_speed]+fAxes_[axis_angular_position]*fAxes_[axis_angular_position]);
                fAxes_[axis_angular_position] = (atan2(fAxes_[axis_linear_speed],fAxes_[axis_angular_position]));

        } else if ((fAxes_[axis_angular_position] > EPS_ZERO) && (fAxes_[axis_linear_speed] < EPS_ZERO) ){

                fAxes_[axis_linear_speed] = sqrt(fAxes_[axis_linear_speed]*fAxes_[axis_linear_speed]+fAxes_[axis_angular_position]*fAxes_[axis_angular_position]);
                fAxes_[axis_angular_position] = (atan2(fAxes_[axis_linear_speed],fAxes_[axis_angular_position]));

        }
*/       
#endif 

        desired_linear_speed = -max_linear_speed * current_speed_lvl * fAxes_[axis_linear_speed];
        desired_angular_position = -max_angular_position * fAxes_[axis_angular_position];

        manual_cmd_msg.angle_date = desired_angular_position;
        manual_cmd_msg.speed_date = desired_linear_speed;
        pub_manual_mode_cmd_.publish(manual_cmd_msg);

        if(vButtons_[button_speed_up].IsReleased()){

                current_speed_lvl += 0.1;
                if(current_speed_lvl > 1.0)
                current_speed_lvl = 1.0;
                ROS_INFO("client speed up = %f",desired_linear_speed);

        }

        if(vButtons_[button_speed_down].IsReleased()){

                current_speed_lvl -= 0.1;
                if(current_speed_lvl < 0.0)
                current_speed_lvl = 0.0;
                ROS_INFO("client speed down = %f",desired_linear_speed);

        }

        if (vButtons_[button_raise_fork].IsReleased()){  

                std_srvs::Empty empty_srv;
                client_raise_elevator_.call(empty_srv);
                ROS_INFO("client raise fork");

        }

        if (vButtons_[button_dwon_fork].IsReleased()){

                std_srvs::Empty empty_srv;
                client_lower_elevator_.call(empty_srv);
                ROS_INFO("client down fork");

        }	

        if(vButtons_[button_ageing_test].IsReleased()){         

                std_srvs::Empty empty_srv;
                //client_ageing_test_.call(empty_srv);
                client_auto_mode_.call(empty_srv);
                ROS_INFO("client test mode");
        }	
}

void class_agvs_pad::callback_joy_node(const sensor_msgs::Joy::ConstPtr& joy)
{
        
	//ROS_INFO("JOY_CALLBACK");
	for(int i = 0; i < joy->axes.size(); i++){

		this->fAxes_[i] = joy->axes[i];
	}

	for(int i = 0; i < joy->buttons.size(); i++){

		this->vButtons_[i].Press(joy->buttons[i]);
	}

	if (vButtons_[button_ulock_a].IsPressed()&&vButtons_[button_ulock_b].IsPressed()){

		handle_ulock_event=!handle_ulock_event;
	}
}


void class_agvs_pad::loop()
{
        ROS_INFO("agvs_pad_loop");
	ros::Rate r(desired_freq);  

        while (!ros::isShuttingDown())
        {
                if(1) //judgment start() function is ok 
                {
                        ROS_INFO("agvs_pad_loop_while");
                        while(ros::ok()) {
                                
                                if(handle_ulock_event){

                                        publish_manaul_cmd();
                                }

                                ros::spinOnce();
                                r.sleep();
                        }   	
                }

        }
}

int main(int argc, char** argv)
{
	ros::init(argc, argv, "agvs_pad");
        ros::NodeHandle n;
	class_agvs_pad agvs_pad(n);

	agvs_pad.loop();
        return (0);
}

