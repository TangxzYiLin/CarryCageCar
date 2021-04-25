#ifndef ELEVATOR_REGMAP_DEFINE_H
#define ELEVATOR_REGMAP_DEFINE_H

#include <stdbool.h>
#include <stdint.h>
using namespace std;
/*
union safe_status_u
{
        uint16_t all_status;
        struct {

                bool swtich_hitchhiking:   1; 
                bool swtich_reset:  1; 
                bool swtich_stop:  1; 
                
                bool raster_first_floor: 1; 
                bool raster_second_floor:      1; 

                bool alarm_collision_avoidance_:   1; 
                bool alarm_emergency_stop_swtich_: 1; 
                bool alarm_auto_man_swtich_:   1; 
                bool alarm_up_limit_:   1;
                bool alarm_down_limit_: 1;
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

*/

#endif