#include <iostream>
#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <std_srvs/Empty.h>

#include "agvs_task/route_target.h"
#include "chassis_drive/chassis_alarm.h"
#include "chassis_drive/chassis_bat.h"
#include "chassis_drive/chassis_cmd.h"
#include "chassis_drive/chassis_state.h"
#include "agvs_task/route_target.h"

//user extern libmodbus  lib 
#ifdef _cplusplus
extern  "c"{
#endif

#include "modbus-tcp.h"

#ifdef _cplusplus
}
#endif

typedef enum _Taskid
{  
        task_1 = 1,
        task_2,
        task_3,
        task_4

}TaskidRoute;

typedef struct _Eventid
{
        bool  event_1;
        bool  event_2;
        bool  event_3;
        bool  event_4;
        bool  event_5;
        bool  event_6;
        bool  event_7;
        bool  event_8;
        bool  event_9;
        bool  event_10;
        bool  event_11;

} Eventid;

typedef enum _State
{
        state_1 = 1, //idle state
        state_2,     //prepare work
        state_3,     //working
        state_4,     //work complete
        state_5,     //work complete
        state_6,     //work complete
        state_7,     //work complete
        state_8      //work complete

}StateRoute;

typedef enum _StateRouteSon
{
        start = 1, //idle state
        update,    //prepare work
        stop       //working

}StateRouteSon;      

class agvs_task_node
{
public:

        agvs_task_node(ros::NodeHandle h);

        void agvs_task_loop();
        void route_firstfloor_mode();
        void route_secondfloor_mode();
        void route_test_mode();

        bool automode_srvcallback(std_srvs::Empty::Request &request, std_srvs::Empty::Response &response);

private:
        void elevator_read_regisers(int16_t addr,int16_t nb, uint16_t *tab_rq_registersaddr);
        void elevator_write_registers(int16_t addr,int16_t nb,const uint16_t*tab_rq_registers);

        ros::NodeHandle node_handle_;
        ros::NodeHandle private_node_handle_;
        ros::Time last_command_time_;

        chassis_drive::chassis_alarm chassis_safe_msg;
        chassis_drive::chassis_state chassis_state_msg;
        agvs_task::route_target message_date;

        ros::Publisher pub_route_task;
        ros::Subscriber sub_pad_cmd;

        ros::ServiceServer  srv_auto_mode_;
	ros::ServiceServer  srv_route_1;        //idel mode (default mode)
	ros::ServiceServer  srv_route_2;        //test mode        
        ros::ServiceServer  srv_route_3;        //first floor mode 
        ros::ServiceServer  srv_route_4;        //second floor mode 
        
        ros::ServiceClient srv_lowerfork_2 ;
        ros::ServiceClient srv_raisefork_2 ;
        
        Eventid event_id = { };
        double desired_freq = 20;

        modbus_t *ctx={};
        int rc;
        int nb_fail;
        int nb_loop;
        int addr;
        int nb;

};

agvs_task_node::agvs_task_node(ros::NodeHandle h):node_handle_(h),private_node_handle_("~"),desired_freq(15.0)
{
        //pub_navigation_mode_cmd = private_node_handle_.serviceClient<std_srvs::Empty>(pub_navigation_mode_cmd_);
        pub_route_task = private_node_handle_.advertise<agvs_task::route_target>(std::string("/agvs_task_node/route/cmd"),10);
        srv_raisefork_2 = private_node_handle_.serviceClient<std_srvs::Empty>(std::string("/agvs_pad/raise_elevator"));            
        srv_lowerfork_2 = private_node_handle_.serviceClient<std_srvs::Empty>(std::string("/agvs_pad/lower_elevator"));
        
        //subscriber 
        srv_auto_mode_ = private_node_handle_.advertiseService("/agvs_pad/ageing_test",&agvs_task_node::automode_srvcallback,this);
        //modbus tcp connect...
#if 0
        do
        {
                ctx = modbus_new_tcp("192.168.2.3", 1502);
                modbus_set_debug(ctx, TRUE);
                ROS_INFO("modbus tcp connecting...\n");

        } while (modbus_connect(ctx) == -1);

#else
        ctx = modbus_new_tcp("192.168.2.3", 502);
        modbus_set_debug(ctx, TRUE)
        ;
        if (modbus_connect(ctx) == -1) {

                while(modbus_connect(ctx) == -1){

                        ROS_INFO("erro:Connection failed: %s\n");
                        ctx = modbus_new_tcp("192.168.2.3", 502);
                        //modbus_strerror(errno);
                        //modbus_free(ctx);
                }      
        }  
#endif
}

void agvs_task_node ::elevator_write_registers(int16_t addr,int16_t nb,const uint16_t*tab_rq_registers)//FIXME the const
{
	int16_t rc;
        uint16_t *tab_rp_registers;

	tab_rp_registers = (uint16_t *) malloc(nb * sizeof(uint16_t));
        memset(tab_rp_registers, 0, nb * sizeof(uint16_t));

	rc = modbus_write_registers(ctx, addr, nb, tab_rq_registers);

	if (rc!= nb) {
 
                for (uint8_t i = 0; i < 3; i++)
                {
                      rc = modbus_write_registers(ctx, addr, nb, tab_rq_registers);
                      if (rc!= nb){
                                ROS_INFO("erro: modbus_write_registers (%d)\n", rc);
                                //ROS_INFO("Address = %d, nb = %d\n", addr, nb);
                                nb_fail++;
                       }
                }
                
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

void agvs_task_node ::elevator_read_regisers(int16_t addr,int16_t nb, uint16_t *tab_rq_registersaddr)
{	
	int16_t rc;
        rc= modbus_read_registers(ctx, addr, nb, tab_rq_registersaddr);

	if (rc != nb) {

		ROS_INFO("erro: modbus_read_registers (%d)\n", rc);
		nb_fail++;
	} 
}

bool agvs_task_node ::automode_srvcallback(std_srvs::Empty::Request &request, std_srvs::Empty::Response &response)
{      
        event_id.event_1= true;
        //ROS_INFO("auto control is alread running\n");
        return true;
}

void agvs_task_node::route_test_mode()
{
        static StateRoute state_father=state_1;
        static uint16_t time_test =0;
        static StateRouteSon state_son =start;

        switch (state_father)
        {  

        case state_1: //idle wait new task 

                if (chassis_safe_msg.alarm_up_limit_)
                {
                        std_srvs::Empty empty_srv;
                        srv_lowerfork_2.call(empty_srv);
                }
                else if (event_id.event_1)
                {

                        message_date.target_location_x=0.0;
                        message_date.target_location_y=0.0;
                        message_date.target_speed=0;
                        message_date.task_direction=1;
                        message_date.task_route_id=1;
                        pub_route_task.publish(message_date);
                        
                        event_id.event_1=false;
                        state_father =state_2;
                } 
                break;

        case state_2: //running to the target location

                if(event_id.event_2)
                {
                        if (chassis_safe_msg.alarm_cargophotos_left_||chassis_safe_msg.alarm_cargophotos_right_)
                        {
                                state_father =state_3;
                        } else  state_father =state_4;
                }
                break;

        case state_3: //stop and raise fork 

                switch (state_son)
                {

                case start:
                {
                        std_srvs::Empty empty_srv;
                        srv_raisefork_2.call(empty_srv);
                        state_son =update;
                        break;
                } 

                case update:
                        time_test++;
                        if (chassis_safe_msg.alarm_up_limit_)
                        {
                                state_son =stop;
                        }
                        break;

                case stop:
                {
                        state_son =start;
                        state_father =state_4;
                        break;
                }
                
                default:
                        break;
                }
                
                break;

        case state_4: //backrunning to second floor:

                switch (state_son)
                {

                case start:
                        message_date.target_location_x=0.0;
                        message_date.target_location_y=0.0;
                        message_date.target_speed=0;
                        message_date.task_direction=0;
                        message_date.task_route_id=2;
                        pub_route_task.publish(message_date);
                        state_son =update;
                        break;
                case update:
                        if(event_id.event_4){
                                state_son =stop;}
 
                        break;
                case stop:
                        state_son =start;
                        state_father =state_1;
                break;

                default:
                        break;
                }
                break;

        case state_5://go to first floor:

                switch (state_son)
                {

                case start:
                        state_son =update;
                        break;
                case update:
                        state_son =stop;
                        break;
                case stop:
                        state_son =start;
                        state_father =state_5;
                break;
                default:
                        break;
                }
                break;
        
        default:
                break;
        }
           
}

void agvs_task_node::agvs_task_loop()
{
        ROS_INFO("agvs_robot_control::agvs_control_loop()");
        ros::Rate r(desired_freq);  // 15.0 

        while (!ros::isShuttingDown()) 
        {
                if (1)//if (starting() == 0)  //FIXME navigation mode 
                {
                        while(ros::ok() && node_handle_.ok())
                        {
                                ROS_INFO("agvs_task_ok");
                                //route_test_mode();

                                {
                                        ros::spinOnce();
                                        r.sleep();
                                }
                        }

                } else {

                        usleep(1000000);
                        ros::spinOnce();
                }
        }

        ROS_INFO("agvs_control::spin() - end");
}

int main(int argc, char** argv)
{
	ros::init(argc, argv, "agvs_task");

	ros::NodeHandle n;		
  	agvs_task_node sxlrc(n);
        sxlrc.agvs_task_loop();
	return (0);       
}

