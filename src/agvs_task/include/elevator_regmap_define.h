#ifndef ELEVATOR_REGMAP_DEFINE_H
#define ELEVATOR_REGMAP_DEFINE_H

#include <stdbool.h>
#include <stdint.h>

//read cmd register
#define REG_SAFE_STATUS_U       140
#define REG_BUTTON_CONFIRM_1    141
#define REG_BUTTON_CONFIRM_2    142
#define REG_BUTTON_RUN_MODE     143
#define REG_BUTTON_ONE_SECOND_MODE      144
#define REG_ELEVATOR_STATE_1    145
#define REG_ELEVATOR_STATE_2    146
#define REG_SAFETY_DOOR         147
#define REG_TEMP_STORAGE        148
#define REG_PLC_ERRO            149
#define LEN_REG_READ            10
//write cmd register
#define REG_ELEVATOR_CMD        150
#define REG_SAFE_DOOR_CMD       151
#define REG_RECHARGE_CMD        152
#define REG_QR_CODE_CMD         153
#define REG_TOWER_LIGHT_CMD     154
#define REG_HEART_BEAT_CMD      155
#define REG_TASK_ID_            156



union safe_status_u
{
        uint16_t all_status;
        struct {

                bool swtich_hitch_hik:     1; 
                bool swtich_reset:         1; 
                bool swtich_stop:          1; 
                
                bool raster_first_floor:   1; 
                bool raster_second_floor:  1; 

                bool reserve_1:   1; 
                bool reserve_2:   1; 
                bool reserve_3:   1; 
                bool reserve_4:   1;
                bool reserve_5:   1;

                bool reserve_6:   1; 
                bool reserve_7:   1; 
                bool reserve_8:   1; 
                bool reserve_9:   1;
                bool reserve_10:  1;
                bool reserve_11:  1;

        }bit;

};

enum button_run_mode:uint16_t
{
        default_mode_run =0,
        button_run_mode_manual_mode,
        button_run_mode_auto_mode,
        button_run_mode_maintain_mode

};

enum button_route_mode:uint16_t
{
        default_mode_route = 0,
        button_route_mode_first_floor_mode,
        button_route_mode_second_floor_mode
};

enum elevator_state_1:uint16_t
{
        default_sate_1 = 0,
        elevator_state_1_origin_location,
        elevator_state_1_hitch_hik_running,
        elevator_state_1_hitch_hik_completed,
        elevator_state_1_reset_running

};

enum elevator_state_2:uint16_t
{
        default_sate_2 = 0,
        elevator_state_2_origin_location,
        elevator_state_2_align_location,
        elevator_state_2_second_floor_location,
        elevator_state_2_rise_running,
        elevator_state_2_align_running,
        elevator_state_2_reset_running

};

enum safety_door:uint16_t
{
        default_state_sd = 0,
        safety_door_closed_state,
        safety_door_opened_state,
        safety_door_close_running,
        safety_door_open_running

};

enum temp_storage:uint16_t
{
        default_state_ts = 0,
        temp_storage_completed,
        temp_storage_no_completed

};

enum erro_plc:uint16_t
{
        normal_state =0,
        erro_plc_1,
        erro_plc_2,
        erro_plc_3,
        erro_plc_4,
        erro_plc_5,
        erro_plc_6,
        erro_plc_7,
        erro_plc_8,
        erro_plc_9,
        erro_plc_10,
        erro_plc_11,
        erro_plc_12,
        erro_plc_13
};

//read register
#pragma pack(push)
#pragma pack(1)
struct read_elevator_state
{
        union safe_status_u reg_safe_state_u;
        uint16_t button_confirm_1; //first floor
        uint16_t button_confirm_2; //seconde floor

        enum button_run_mode button_run_mode_e; 
        enum button_route_mode button_route_mode_e;
        enum elevator_state_1 elevator_state_1_e;
        enum elevator_state_2 elevator_state_2_e;

        enum safety_door safety_door_e;
        enum temp_storage temp_storage_e;    

        enum erro_plc erro_plc_e;

};
#pragma pack(pop)

enum elevator_cmd:uint16_t
{
        default_state_ec = 0,
        elevator_1_hitch_hik_cmd,
        elevator_1_reset_cmd,
        
        elevator_2_rise_cmd,
        elevator_2_align_cmd,
        elevator_2_reset_cmd,

};

enum open_close_cmd:uint16_t
{
        default_state_occ = 0,
        open_cmd,
        close_cmd

};

enum tower_light_cmd:uint16_t
{
        default_state_tlc = 0,
        _1s,
        _2s,
        _3s,
        _4s,
        _5s,
        _6s,
        _7s,
        _8s,
        _9s,
        _10s,
        _11s,
        _12s,
        _13s
};

#pragma pack(push)
#pragma pack(1)
struct write_elevator_cmd
{
        enum elevator_cmd elevator_cmd_e;
        enum open_close_cmd safe_door_cmd_e;
        enum open_close_cmd recharge_cmd_e;
        enum open_close_cmd qr_code_cmd_e;
        enum tower_light_cmd tower_light_cmd_e; 
};
#pragma pack(pop)

#endif
