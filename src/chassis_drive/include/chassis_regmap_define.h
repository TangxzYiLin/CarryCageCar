#ifndef CHASSIS_REGMAP_IDEFINE_H
#define CHASSIS_REGMAP_IDEFINE_H

#include <stdbool.h>
#include <stdint.h>
using namespace std;

#define LOOP             	 1
#define SERVER_ID     		 1
#define ADDRESS_START    	 100
#define ADDRESS_END    		 133

//chassis PLC register define
#define	REG_INDICTOR            100
#define REG_INDICATOR   	101

#define	REG_MOTOR_VEL		102
#define	REG_HIGH_CONTROL        103

#define	REG_TASK_ID    		104
#define	REG_TASK_STATE  	105

#define	REG_CHASSIS_STATES	110
#define	REG_BAT_STATES  	130



#define  RGB_READ              0x2400
#define  RGB_BULE              0x1200
#define  RGB_GREEN             0x0900

//lift controlflag
#define RISE    1
#define DOWN    0

//sturct date buf len

#define WRITE_BUF_MOTOR_SIZE    2
#define WRITE_BUF_TASK_SIZE     2

#define READ_BUF_STATE_SIZE    14
#define READ_BUF_BAT_SIZE      4



//RGB color define //red:2400  blue:1200  green :0900
                //led state     low->high

                //0000 0001  null
                //0000 0010  null
                
                //0000 0100  l-red
                //0000 1000  l-b
                //0001 0000  l-g

                //0010 0000  r-red
                //0100 0000  r-b
                //1000 0000  r-g

//led state define 
enum chassis_led_state
{
        chassis_led_state_all_off = 0,
 
        chassis_led_state_red_on,
        chassis_led_state_bule_on ,
        chassis_led_state_green_on, 

        chassis_led_state_red_flash,
        chassis_led_state_green_flash,
        chassis_led_state_blue_flash
};

//lift state define 

enum chassis_swtich_state
{
        chassis_swtich_state_low = 0,
        chassis_swtich_state_high
};


//chassis_reg read and write struct define 
union safe_status_u
{
        uint16_t all_status;
        struct {

                bool alarm_forkphotoe_left_:   1;  //avoidance safe
                bool alarm_forkphotoe_right_:  1;  //avoidance safe
                bool alarm_cargophotos_left_:  1;  //avoidance safe
                bool alarm_cargophotos_right_: 1;  //avoidance safe
                bool alarm_micro_swtich_:      1; 
                bool alarm_collision_avoidance_:   1; //avoidance safe
                bool alarm_emergency_stop_swtich_: 1; 
                bool alarm_auto_man_swtich_:   1; 
                bool alarm_up_limit_:   1;
                bool alarm_down_limit_: 1;
        }bit;
};

union reserve_status_u
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
typedef struct read_chassis_state_
{
        union safe_status_u reg_safe_state_;
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

}read_chassis_state_s;
#pragma pack(pop)

union reg_read_state_date_u
{
	uint16_t date_buffer[READ_BUF_STATE_SIZE];
        read_chassis_state_s date_info;
};

//read bat register
union reg_read_bat_date_u
{
	uint16_t date_buffer[READ_BUF_BAT_SIZE];
	struct {

		uint16_t reg_bat_erro_;
		uint16_t reg_bat_power_;
		uint16_t reg_bat_current_;
		uint16_t reg_bat_voltage_;

	}date_info;
};

//write motor register
union reg_write_date_motor_u
{
	uint16_t date_buffer[WRITE_BUF_MOTOR_SIZE];

	struct {
		uint16_t reg_motor_speed;
		uint16_t reg_motor_angle;
	}date_info;
};

//write task register
union reg_write_date_task_u
{
        uint16_t date_buffer[WRITE_BUF_TASK_SIZE];

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
        union reserve_status_u write_led_cmd_;

        uint16_t reg_lift_high_;

        union reg_write_date_motor_u write_motor_cmd_;
        union reg_write_date_task_u  write_task_cmd_; 
        union reg_read_state_date_u  read_state_cmd_;
        union reg_read_bat_date_u    read_bat_state_cmd_;

}chassis_drive_reg_s;
#pragma pack(pop)

#endif
