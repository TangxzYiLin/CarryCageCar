#! /bin/bash
#一，输入用户名与密码################################
#在start.sh 脚本里面实现了

#二，运行roscore####################################
source /opt/ros/melodic/setup.bash
gnome-terminal --geometry 60x20+10+10 bash -- roscore 
sleep 2

#三，游戏手柄配置###################################
#1.Configuring the Joystick
ls /dev/input/; sleep 1
#2.手柄节点串口配置
gnome-terminal --geometry 60x20+625+10 bash -- sudo jstest /dev/input/js0; sleep 1
#3.配置手柄串口读写权限
sudo chmod a+rw /dev/input/js0; sleep 0.5
#4.打印手柄控制param 参数
rosparam set joy_node/dev "/dev/input/js0"; sleep 0.5
#5.启动串口节点Starting the Joy Node
gnome-terminal --geometry 60x20+1180+10 bash -- rosrun joy joy_node; sleep 1
#6.打印消息话题
gnome-terminal --geometry 60x20+10+500 bash -- rostopic echo joy; sleep 1

#四，ros节点运行####################################
#1.启动手动模式控制接收节点
source /home/iha/avg_cage-master/devel/setup.bash
gnome-terminal --geometry 60x20+625+500 bash -- rosrun agvs_pad agvs_pad; sleep 2

#配置232串口读写权限
sudo chmod 777 /dev/ttyUSB0; sleep 1
#2.启动底盘驱动节点
source /home/iha/avg_cage-master/devel/setup.bash
gnome-terminal --geometry 60x20+1180+500 bash -- rosrun chassis_drive chassis_drive; sleep 2

#3.启动运动控制算法节点


wait
exit 0
