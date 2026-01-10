//
// Created by 21481 on 2025/11/9.
//
#include "cmsis_os.h"
#include "CAN_receive.h"
#include "can_comm.h"
#include "pid.h"
#include "xiaomi_motor_task.h"


pid_type_def xiaomi_can2_ID1_speed_pid;
pid_type_def xiaomi_can2_ID2_speed_pid;

void xiaomi_motor_task()
{
    osDelay(6000);//必要，等待电机初始化完成才可通讯

    CanComm_ControlCmd(CMD_MOTOR_MODE,xiaomimotors[1]);//电机1使能
    CanComm_ControlCmd(CMD_MOTOR_MODE,xiaomimotors[0]);//电机2使能
    osDelay(100);
    xiaomimotors[1].give_tor = 0.0f ;
    xiaomimotors[0].give_tor = 0.0f ;
    CanComm_SendControlPara(xiaomimotors[1]);//电流归零
    CanComm_SendControlPara(xiaomimotors[0]);//电流归零

    while (1)
    {
        xiaomi_pid_compute();

//        osDelay(6000);
//        CanComm_ControlCmd(CMD_RESET_MODE,xiaomimotors[4]);//电机1使能
//        osDelay(1000);
//        CanComm_ControlCmd(CMD_ZERO_POSITION,xiaomimotors[4]);//电机1使能

        //力矩发送部分已归到can_sent任务

        osDelay(1);
    }
}


void xiaomi_pid_compute()
{
    xiaomimotors[1].given_speed = -0.002f * (float )rc_ch4 ;
    xiaomimotors[1].give_tor = xiaomi_can2_id1_speed_pid_loop(xiaomimotors[1].given_speed);
    xiaomimotors[0].given_speed = 0.002f * (float )rc_ch4 ;
    xiaomimotors[0].give_tor = xiaomi_can2_id2_speed_pid_loop(xiaomimotors[0].given_speed);
}


//xiaomi_id1_can2_pid
void xiaomi_can2_id1_speed_pid_init(void)
{
    static fp32 xiaomi_can2_id1_kpkikd[3] = {XIAOMI_CAN2_ID1_SPEED_PID_KP,XIAOMI_CAN2_ID1_SPEED_PID_KI,XIAOMI_CAN2_ID1_SPEED_PID_KD};
    PID_init(&xiaomi_can2_ID1_speed_pid, PID_POSITION, xiaomi_can2_id1_kpkikd, XIAOMI_CAN2_ID1_SPEED_PID_OUT_MAX, XIAOMI_CAN2_ID1_SPEED_PID_KI_MAX);

}

float xiaomi_can2_id1_speed_pid_loop(float xiaomi_can2_id1_speed_set_loop)
{
    PID_calc(&xiaomi_can2_ID1_speed_pid, xiaomimotors[1].fifilter_compute_speed , xiaomi_can2_id1_speed_set_loop);
    float xiaomi_can2_id1_speed_loop = (float)(xiaomi_can2_ID1_speed_pid.out);

    return xiaomi_can2_id1_speed_loop ;

}



//xiaomi_can2_id2_pid
void xiaomi_can2_id2_speed_pid_init(void)
{
    static fp32 xiaomi_can2_id2_kpkikd[3] = {XIAOMI_CAN2_ID2_SPEED_PID_KP,XIAOMI_CAN2_ID2_SPEED_PID_KI,XIAOMI_CAN2_ID2_SPEED_PID_KD};
    PID_init(&xiaomi_can2_ID2_speed_pid, PID_POSITION, xiaomi_can2_id2_kpkikd, XIAOMI_CAN2_ID2_SPEED_PID_OUT_MAX, XIAOMI_CAN2_ID2_SPEED_PID_KI_MAX);

}

float xiaomi_can2_id2_speed_pid_loop(float xiaomi_can2_id2_speed_set_loop)
{
    PID_calc(&xiaomi_can2_ID2_speed_pid, xiaomimotors[0].fifilter_compute_speed , xiaomi_can2_id2_speed_set_loop);
    float xiaomi_can2_id2_speed_loop = (float)(xiaomi_can2_ID2_speed_pid.out);

    return xiaomi_can2_id2_speed_loop ;

}

