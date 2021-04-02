/*
 * agvs_pad
 * Copyright (c) 2013, Robotnik Automation, SLL
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *     * Neither the name of the Robotnik Automation, SLL. nor the names of its
 *       contributors may be used to endorse or promote products derived from
 *       this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 * \author Robotnik Automation, SLL
 * \brief Allows to use a pad with the robot controller, sending the messages received from the joystick device
 */
#include <ros/ros.h>
#include <sensor_msgs/Joy.h>  //the date source come from 
#include <geometry_msgs/Twist.h>
#include <std_msgs/Int32.h>
#include <unistd.h>
#include <vector>

#include "diagnostic_msgs/DiagnosticStatus.h"
#include "diagnostic_updater/diagnostic_updater.h"
#include "diagnostic_updater/update_functions.h"
#include "diagnostic_updater/DiagnosticStatusWrapper.h"
#include "diagnostic_updater/publisher.h"
// #include <agvs_controller/AckermannDriveStamped.h>
//#include <agvs_robot_control/AckermannDriveStamped.h>
#include <std_srvs/Empty.h>

#include "chassis_drive/chassis_cmd.h"
#include"agvs_control/date_pads_cmd.h"
#include"chassis_drive/cmd_lift.h"


//socket test
#include <sys/socket.h>
#include <sys/types.h>

#define DEFAULT_MAX_SKID_LINEAR_SPEED	280.0// m/s
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


#define DEFAULT_JOY			"/joy"
#define DEFAULT_HZ			20.0

#define default_button_ageing_test_  	0

#define default_button_speed_up_        6
#define default_button_speed_down_      2

#define default_button_up_car_		3
#define default_button_down_car_	1

#define default_button_modeswtich_1 	5
#define default_button_modeswtich_2	4

#define eps_zero  1e-6

//! Class to save the state of the buttons
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

////////////////////////////////////////////////////////////////////////
//                            class      		              //
////////////////////////////////////////////////////////////////////////
class AgvsPad
{
public:
	
	AgvsPad();
	
	void ControlLoop();
	int SetStateMode(int state, int arm_mode, int platform_mode);
	
private:
	
	void joyCallback(const sensor_msgs::Joy::ConstPtr& joy);
	
	char * StateToString(int state);
	int SwitchToState(int new_state);
	
	void PublishState();
	//! Enables/Disables the joystick
	bool EnableDisable(chassis_drive::chassis_cmd &req, chassis_drive::chassis_cmd &res );
	void Update();
	
	ros::NodeHandle nh_;
	
	int axis_linear_speed_, axis_angular_position_;
	double l_scale_, a_scale_;
	double current_speed_lvl;
	//! Set the max speed sent to the robot
	double max_linear_speed_, max_angular_position_;
	//! Desired component's freq
	double desired_freq_;
	
	// TOPICS
	//! It will publish into command velocity (for the robot)
	ros::Publisher vel_pub_;
	
	//! they will be suscribed to the joysticks
	ros::Subscriber joy_sub_;

	ros::ServiceClient raise_elevator_client_;
	ros::ServiceClient lower_elevator_client_;
        ros::ServiceClient ageing_test_client_;

	//! // Name of the joystick's topic
	std::string  joy_topic_;	
	//! Name of the topic where it will be publishing the velocity
	std::string cmd_topic_vel;
	//! Name of the service where it will be modifying the digital outputs
	std::string cmd_service_io_;
	//! topic name for the state
	std::string topic_state_;
	//! Topic to publish the state
	ros::Publisher state_pub_;

	//! Name of the service called to raise the elevator
	std::string service_raise_elevator_;
	//! Name of the service called to lower the elevator
	std::string service_lower_elevator_;
        std::string service_ageing_test_;
	
	// JOYSTICK
	//! Current number of buttons of the joystick
	int num_of_buttons_;
	int num_of_axes_;
	
	//! Vector to save the axis values //轴径
	std::vector<float> fAxes;
	//! Vector to save and control the axis values
	std::vector<Button> vButtons;
	//! Number of the DEADMAN button
	int button_ageing_test_;
	//! Number of the button for increase or decrease the speed max of the joystick	
	int button_speed_up_, button_speed_down_;
	int button_modeswtich_1_, button_modeswtich_2_;
	int button_up_car_, button_down_car_;
	int output_1_, output_2_;
	bool bOutput1, bOutput2;
		
	//! Flag to enable/disable the handle
	bool bEnable;
};

AgvsPad::AgvsPad():  nh_("~")
{
	current_speed_lvl = -1;
	// JOYSTICK CONFIG
	nh_.param("num_of_buttons", num_of_buttons_, DEFAULT_NUM_OF_BUTTONS);
	nh_.param("num_of_axes", num_of_axes_, DEFAULT_NUM_OF_AXES);
	nh_.param("desired_freq", desired_freq_, DEFAULT_HZ);
	
	if(num_of_axes_ > MAX_NUM_OF_AXES){
		num_of_axes_ = MAX_NUM_OF_AXES;
		ROS_INFO("AgvsPad::AgvsPad: Limiting the max number of axes to %d", MAX_NUM_OF_AXES);
	}
	if(num_of_buttons_ > MAX_NUM_OF_BUTTONS){
		num_of_buttons_ = MAX_NUM_OF_BUTTONS;
		ROS_INFO("AgvsPad::AgvsPad: Limiting the max number of buttons to %d", MAX_NUM_OF_BUTTONS);
	}
	
	nh_.param("topic_joy", joy_topic_, std::string(DEFAULT_JOY));	
	
	// MOTION CONF
	nh_.param("cmd_topic_vel", cmd_topic_vel, std::string("/agvs_control/pads_cmd"));
	
	nh_.param("button_dead_man", button_ageing_test_, default_button_ageing_test_);
	nh_.param("button_speed_up", button_speed_up_, default_button_speed_up_);
	nh_.param("button_speed_down", button_speed_down_, default_button_speed_down_); 

	nh_.param("button_modeswtich_1", button_modeswtich_1_, default_button_modeswtich_1);
	nh_.param("button_modeswtich_2", button_modeswtich_2_, default_button_modeswtich_2);
	
	nh_.param("max_angular_position", max_angular_position_, DEFAULT_MAX_ANGULAR_POSITION); 
	nh_.param("max_linear_speed_", max_linear_speed_, DEFAULT_MAX_SKID_LINEAR_SPEED); 
	
	nh_.param("axis_linear_speed", axis_linear_speed_, DEFAULT_AXIS_LINEAR_X); 
	nh_.param("axis_angular_position", axis_angular_position_, DEFAULT_AXIS_ANGULAR); 

	ROS_INFO("axis_linear_speed_ = %d, axis_angular = %d", axis_linear_speed_, axis_angular_position_);
	ROS_INFO("max_linear_speed = %lf, max_angular_speed = %lf", max_linear_speed_, max_angular_position_);
	
	// DIGITAL OUTPUTS CONF
	nh_.param("cmd_service_io", cmd_service_io_, cmd_service_io_);

	nh_.param("button_up_car", button_up_car_, default_button_up_car_);
	nh_.param("button_down_car", button_down_car_, default_button_down_car_);

	nh_.param("output_1", output_1_, output_1_);
	nh_.param("output_2", output_2_, output_2_);
	
	nh_.param("topic_state", topic_state_, std::string("/agvs_pad/state"));

	nh_.param("service_raise_elevator", service_raise_elevator_, std::string("/agvs_pad/raise_elevator"));
	nh_.param("service_lower_elevator", service_lower_elevator_, std::string("/agvs_pad/lower_elevator"));

        nh_.param("service_ageing_test",service_ageing_test_,std::string("/agvs_pad/ageing_test"));
	
	ROS_INFO("AgvsPad num_of_buttons_ = %d, axes = %d, topic controller: %s, hz = %.2lf", num_of_buttons_, num_of_axes_, cmd_topic_vel.c_str(), desired_freq_);	
	
	for(int i = 0; i < MAX_NUM_OF_BUTTONS_F710; i++){
		Button b;
		ROS_INFO("add bottom");
		vButtons.push_back(b);
	}
	
	for(int i = 0; i < MAX_NUM_OF_AXES_F710; i++){
		fAxes.push_back(0.0);
                ROS_INFO("axes");
	}
	//FIXME TXZ change to msg "chassis_drive/chassis_cmd.h"
        this->vel_pub_ = nh_.advertise<agvs_control::date_pads_cmd>(this->cmd_topic_vel, 50); 
	
	this->joy_sub_ = nh_.subscribe<sensor_msgs::Joy>(joy_topic_, 1, &AgvsPad::joyCallback, this);


	raise_elevator_client_ = nh_.serviceClient<chassis_drive::cmd_lift>(service_raise_elevator_);
	lower_elevator_client_ = nh_.serviceClient<chassis_drive::cmd_lift>(service_lower_elevator_);
        ageing_test_client_    = nh_.serviceClient<std_srvs::Empty>(service_ageing_test_);
		
	bOutput1 = bOutput2 = 0;
	//enable_disable_srv_ = nh_.advertiseService("/agvs_pad/enable_disable",  &AgvsPad::EnableDisable, this);  //TODO auto or manual
	
	bEnable = 0;	// Communication flag enabled by default
	
}

/*
 *	\brief Updates the diagnostic component. Diagnostics
 * 		   Publishes the state
 *
 */

void AgvsPad::Update()
{
        //PublishState();
        //ROS_INFO("update...\n");
}

//! 
void AgvsPad::PublishState()
{
	/*agvs_pad::rescuer_pad_state pad_state;
	
	pad_state.state = StateToString(iState);
	pad_state.arm_mode = ModeToString(iArmMode);
	pad_state.platform_mode = ModeToString(iPlatformMode);
	pad_state.speed_level = current_speed_lvl;
	pad_state.deadman_active = (bool) vButtons[button_dead_man_].IsPressed();
	state_pub_.publish(pad_state);*/
}

/*
 *	\brief Enables/Disables the pad
 *
 *///TODO manual or auto mode 
bool AgvsPad::EnableDisable(chassis_drive::chassis_cmd &req, chassis_drive::chassis_cmd &res )
{
	bEnable = req.chassis_brake_cmd_;

	ROS_INFO("AgvsPad::EnablaDisable: Setting to %d", req.chassis_brake_cmd_);
	res.chassis_brake_cmd_ = true;
	return true;
}

void AgvsPad::joyCallback(const sensor_msgs::Joy::ConstPtr& joy)
{
	ROS_INFO("JOY_CALLBACK");
	// First joystick being saved
	for(int i = 0; i < joy->axes.size(); i++){
		this->fAxes[i] = joy->axes[i];
	}

	for(int i = 0; i < joy->buttons.size(); i++){
		this->vButtons[i].Press(joy->buttons[i]);
	}

	if (vButtons[button_modeswtich_1_].IsPressed()&&vButtons[button_modeswtich_2_].IsPressed())
	{
		bEnable=!bEnable;
	}
	//ROS_INFO("AgvsPad::joyCallback: num_of_axes = %d, buttons = %d", (int)(joy->axes.size()), (int)(joy->buttons.size()));
}


//! Controls the actions and states
void AgvsPad::ControlLoop(){
        
	double desired_linear_speed = 0.0, desired_angular_position = 0.0;
	//agvs_controller::AckermannDriveStamped ref_msg;
	agvs_control::date_pads_cmd ref_msg;  
	ros::Rate r(desired_freq_);   

        float speed,angle;
        
        while(ros::ok()) {
                        
                Update();         
                if(bEnable){
                        ROS_INFO("breakpoint_2\n");
                        //adjustment the ratio parameter
                        //angle_speed_analyze
#if 0
                        if(fabs(fAxes[axis_angular_position_])<=eps_zero){
                                speed = fAxes[axis_linear_speed_];
                                angle =0.0f;

                        } else {
                                speed =sqrt(fAxes[axis_linear_speed_]*fAxes[axis_linear_speed_]+fAxes[axis_angular_position_]*fAxes[axis_angular_position_]);
                                angle =(atan2(fAxes[axis_linear_speed_],fAxes[axis_angular_position_]));

                        }
#endif 
                        desired_linear_speed = max_linear_speed_ * current_speed_lvl * fAxes[axis_linear_speed_];
                        desired_angular_position = max_angular_position_ * fAxes[axis_angular_position_];

                        ref_msg.angle_date=desired_angular_position;
                        ref_msg.speed_date=desired_linear_speed;
                                
                        // Publish into command_vel topic
                        vel_pub_.publish(ref_msg);
        
                        if(vButtons[button_speed_up_].IsReleased()){
                                current_speed_lvl += 0.1;
                                if(current_speed_lvl > 1.0)
                                current_speed_lvl = 1.0;
                        }
                        if(vButtons[button_speed_down_].IsReleased()){
                                current_speed_lvl -= 0.1;
                                if(current_speed_lvl < 0.0)
                                current_speed_lvl = 0.0;
                        }				
                        if (vButtons[button_up_car_].IsReleased()){  //升起升降机构
                                chassis_drive::cmd_lift empty_srv;
                                raise_elevator_client_.call(empty_srv);
                                ROS_INFO("Raise elevator");
                        }
                        if (vButtons[button_down_car_].IsReleased()){//降低升降机构
                                chassis_drive::cmd_lift empty_srv;
                                lower_elevator_client_.call(empty_srv);
                                ROS_INFO("Lower elevator");
                        }	
                        if(vButtons[button_ageing_test_].IsReleased()){//ageing_test
                                std_srvs::Empty empty_srv;
                                ageing_test_client_.call(empty_srv);
                                ROS_INFO("ageing_test\n");
                        }				
                }
                ros::spinOnce();
                r.sleep();
        }   	
}

///////////////////////// MAIN /////////////////////////////////
int main(int argc, char** argv)
{
	ros::init(argc, argv, "agvs_pad");
	AgvsPad pad;
	
	pad.ControlLoop();
}

