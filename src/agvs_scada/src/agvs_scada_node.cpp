#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <errno.h>
#include <sys/types.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <netdb.h>

#include <string.h>
#include <stdio.h>

#include <iostream>
#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <std_srvs/Empty.h>

#include "agvs_msg.h"

#define HOST "192.168.0.217"        
#define PORT 3000                       

class class_agvs_scada
{

public:

        class_agvs_scada(ros::NodeHandle h);
          
        void start(); //wait for other node have been running
        void publish();
        bool loop(); 
        void stop();

private:

        void callback_chassis_drive_alarm(const chassis_drive::chassis_alarmConstPtr &msg_alarm_tmp);
        void callback_chassis_drive_bat(const chassis_drive::chassis_batConstPtr &msg_bat_tmp);
        void callback_chassis_drive_state(const chassis_drive::chassis_stateConstPtr &mag_state_tmp);
        void callback_agvs_task_route_task(const agvs_task::route_targetConstPtr &msg_route_task_tmp);

        bool init_tcp_connect();

        ros::NodeHandle node_handle_;
        ros::NodeHandle private_node_handle_;
        double desired_freq;

        //tcp connect param
        struct sockaddr_in server;
        int sockfd, ret;

        //sub
        ros::Subscriber sub_chassis_drive_alarm_;
        ros::Subscriber sub_chassis_drive_bat_;
        ros::Subscriber sub_chassis_drive_state_;
        ros::Subscriber sub_agvs_task_route_task_;

};

class_agvs_scada::class_agvs_scada(ros::NodeHandle h) :desired_freq(10),private_node_handle_("~")
{
        
        sub_chassis_drive_alarm_ = private_node_handle_.subscribe<chassis_drive::chassis_alarm>(std::string("chassis_drive/chassis_alarm_topic"),10,&class_agvs_scada::callback_chassis_drive_alarm,this);
        sub_chassis_drive_bat_ = private_node_handle_.subscribe<chassis_drive::chassis_bat>(std::string("chassis_drive/chassis_bat_topic"),10,class_agvs_scada::callback_chassis_drive_bat,this);
        sub_chassis_drive_state_ = private_node_handle_.subscribe<chassis_drive::chassis_state>(std::string("chassis_drive/chassis_state_topic"),10,class_agvs_scada::callback_chassis_drive_state,this);
        
        sub_agvs_task_route_task_ = private_node_handle_.subscribe<agvs_task::route_target>(std::string("/agvs_task_node/route/cmd"),10,class_agvs_scada::callback_agvs_task_route_task,this);

}

bool class_agvs_scada::init_tcp_connect()
{

        // 创建套接字描述符
        if ((sockfd = socket(AF_INET, SOCK_STREAM, 0)) == -1) {
                ROS_INFO("create an endpoint for communication fail!\n");
                //exit(1);
        }

        bzero(&server, sizeof(server));
        server.sin_family = AF_INET;
        server.sin_port = htons(PORT);
        server.sin_addr.s_addr = inet_addr(HOST);

        // 建立TCP连接
        if (connect(sockfd, (struct sockaddr *)&server, sizeof(struct sockaddr)) == -1) {
                ROS_INFO("connect server fail...\n");
                close(sockfd);
                return 0;
        }

        ROS_INFO("connect server success...\n");
        return 1;
        //write(sockfd, buffer, sizeof(buffer)); 
}

void class_agvs_scada::callback_chassis_drive_alarm(const chassis_drive::chassis_alarmConstPtr &msg_alarm_tmp)
{

}

void class_agvs_scada::callback_chassis_drive_bat(const chassis_drive::chassis_batConstPtr &msg_bat_tmp)
{

}

void class_agvs_scada::callback_chassis_drive_state(const chassis_drive::chassis_stateConstPtr &mag_state_tmp)
{

}

void class_agvs_scada::callback_agvs_task_route_task(const agvs_task::route_targetConstPtr &msg_route_task_tmp)
{

}

bool class_agvs_scada::loop()
{

	ROS_INFO("agvs_scada_loop");
        ros::Rate r(desired_freq); 

        //using ros::isShuttingDown to avoid restarting the node during a shutdown
        while (!ros::isShuttingDown()) {

                if (init_tcp_connect() !=0 ){ //if (starting() == 0)
                        
                        ROS_INFO("agvs_scada_while");
                        while(ros::ok() && node_handle_.ok()) {
                                        
                                ros::spinOnce();
                                r.sleep();
                        }

                        close(sockfd);
                        ROS_INFO("end ros::ok() !!!");

                } else {

                        
                        // no need for diagnostic here since a broadcast occurs in start
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
  	class_agvs_scada agvs_scada(n);

        agvs_scada.loop();

	return (0);

}