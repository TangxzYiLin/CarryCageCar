#ifndef CHAASSIS_DRIVE_NODE_H
#define CHAASSIS_DRIVE_NODE_H

#include <stdbool.h>　　 
#include <stdint.h>　　　
using namespace std;

/* The goal of this program is to check all major functions of
   libmodbus:
   All these functions are called with random values on a address
   range defined by the following defines.
*/
#define LOOP             	 1
#define SERVER_ID     		 1
#define ADDRESS_START    	 100
#define ADDRESS_END    		 133

//chassis PLC register define
#define	reg_indicator		100
#define reg_high_control	101

#define	reg_motor_vel		102
#define	reg_motor_angle         103

#define	reg_task_id    		104
#define	reg_task_state  	105

#define	reg_chassis_states	110
#define	reg_bat_states  	130

//RGB color define //read:2400  blue:1200  green :0900
                //led state     low->high

                //0000 0001  null
                //0000 0010  null
                
                //0000 0100  l-red
                //0000 1000  l-b
                //0001 0000  l-g

                //0010 0000  r-red
                //0100 0000  r-b
                //1000 0000  r-g
#define  rgb_read               0x2400
#define  rgb_blue               0x1200
#define  rgb_green              0x0900

//lift controlflag
#define rise    1
#define land    0

//sturct date buf len

#define write_buf_size_motor    2
#define write_buf_size_task     2

#define read_buf_size_state     14
#define read_buf_size_bat       4

union safe_status
{
        uint16_t all_status;
        struct {
                bool alarm_forkphotoe_left_:   1; 
                bool alarm_forkphotoe_right_:  1; 
                bool alarm_cargophotos_left_:  1; 
                bool alarm_cargophotos_right_: 1; 
                bool alarm_micro_swtich_:      1; 
                bool alarm_collision_avoidance_:   1; 
                bool alarm_emergency_stop_swtich_: 1; 
                bool alarm_auto_man_swtich_:   1; 
                bool alarm_up_limit_:   1;
                bool alarm_down_limit_: 1;
        }bit;
};

union reserve_status
{
        uint16_t all_status;
        struct {
                bool led_state_:   3; 
                bool reserve_1:    1; 
                bool reserve_2:    1; 
                bool reserve_3:    1; 
                bool reserve_4:    1; 
                bool reserve_5:    1; 
                bool reserve_6:    1; 
                bool reserve_7:    1; 
                bool reserve_8:    1; 
                bool reserve_9:    1; 
                bool reserve_10:   1; 
                bool reserve_11:   1; 
                bool reserve_12:   1; 
                bool reserve_13:   1; 
        }bit;
};

//read register
#pragma pack(push)
#pragma pack(1)
typedef struct read_chassis_state
{
        union safe_status reg_safe_state_;
        uint32_t reg_odometer_param_;
        uint32_t reg_odometer_;
        
        uint16_t reg_speed_feedback_;
        uint16_t reg_angle_feedback_;
        uint16_t reg_lifthigh_feedback_;
        uint16_t reg_walk_motor_erro_;
        uint16_t reg_roate_motor_erro_;
        uint16_t reg_lift_motor_erro_;
        
        uint16_t reg_task_id_feedback_;
        uint16_t reg_task_state_feedback_;

        uint16_t reg_selfcheck_erro_;       
    // ...
}read_chassis_state_;
#pragma pack(pop)

union reg_read_state_date
{
	uint16_t date_buffer[read_buf_size_state];
        read_chassis_state_ date_info;
};

//read bat register

union reg_read_bat_date
{
	uint16_t date_buffer[read_buf_size_bat];
	struct {
		uint16_t reg_bat_erro_;
		uint16_t reg_bat_power_;
		uint16_t reg_bat_current_;
		uint16_t reg_bat_voltage_;
		// ...
	}date_info;
};

//write motor register
union reg_write_date_motor
{
	uint16_t date_buffer[write_buf_size_motor];

	struct {
		uint16_t reg_motor_speed_;
		uint16_t reg_motor_angle_;
	}date_info;
};

//write task register
union reg_write_date_task
{
        uint16_t date_buffer[write_buf_size_task];

        struct{
                uint16_t reg_task_id_;
                uint16_t reg_heartbeat_;
        }date_info;
};

//package chassis register
#pragma pack(push)
#pragma pack(1)
typedef struct chassis_drive_reg_
{
        union reserve_status write_led_cmd_;

        uint16_t reg_lift_high_;

        union reg_write_date_motor write_motor_cmd_;
        union reg_write_date_task  write_task_cmd_; 
        union reg_read_state_date  read_state_cmd_;
        union reg_read_bat_date    read_bat_state_cmd_;
}chassis_drive_reg;
#pragma pack(pop)


#endif