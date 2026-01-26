//
// Created by 21481 on 2025/11/9.
//
#include "cmsis_os.h"
#include "CAN_receive.h"
#include "can_comm.h"
#include "pid.h"
#include "xiaomi_motor_task.h"


pid_type_def xiaomi_can1_ID1_speed_pid;
pid_type_def xiaomi_can1_ID2_speed_pid;

pid_type_def xiaomi_can1_ID1_angle_pid;
pid_type_def xiaomi_can1_ID2_angle_pid;


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
        xiaomi_angle_limit();
        osDelay(1);
    }
}


void xiaomi_pid_compute()
{
    // xiaomi_can1_id1_given_angle = xiaomi_can1_id1_given_angle + 0.00001f * (float )rc_ch1 ;
    // xiaomi_can1_id1_given_speed = 20.0f * xiaomi_can1_id1_angle_pid_loop(xiaomi_can1_id1_given_angle) ;
    // xiaomimotors[0].give_tor = xiaomi_can1_id1_speed_pid_loop(xiaomi_can1_id1_given_speed);
    //
    // xiaomi_can1_id2_given_angle = xiaomi_can1_id2_given_angle - 0.00001f * (float )rc_ch3;
    // xiaomi_can1_id2_given_speed  = 20.0f * xiaomi_can1_id2_angle_pid_loop(xiaomi_can1_id2_given_angle) ;
    // xiaomimotors[1].give_tor = xiaomi_can1_id2_speed_pid_loop(xiaomi_can1_id2_given_speed);

    // xiaomi_can1_id1_given_speed =  0.01f * (float )rc_ch4 ;
    // xiaomimotors[0].give_tor = xiaomi_can1_id1_speed_pid_loop(xiaomi_can1_id1_given_speed);
    //
    // xiaomi_can1_id2_given_speed =  0.01f * (float )rc_ch4 ;
    // xiaomimotors[1].give_tor = xiaomi_can1_id2_speed_pid_loop(xiaomi_can1_id2_given_speed);

    if (rc_s0 == 1)
    {
        xiaomi_can1_id1_given_angle = xiaomi_can1_id1_given_angle + 0.00001f * (float )rc_ch1 ;
        xiaomi_can1_id2_given_angle = xiaomi_can1_id2_given_angle - 0.00001f * (float )rc_ch3;
    }
    else if (rc_s0 == 3 )
    {
        xiaomi_can1_id1_given_angle = xiaomimotors[0].return_angle ;
        xiaomi_can1_id2_given_angle = xiaomimotors[1].return_angle;
    }
    xiaomi_can1_id1_given_speed = 20.0f * xiaomi_can1_id1_angle_pid_loop(xiaomi_can1_id1_given_angle) ;
    xiaomimotors[0].give_tor = xiaomi_can1_id1_speed_pid_loop(xiaomi_can1_id1_given_speed);


    xiaomi_can1_id2_given_speed  = 20.0f * xiaomi_can1_id2_angle_pid_loop(xiaomi_can1_id2_given_angle) ;
    xiaomimotors[1].give_tor = xiaomi_can1_id2_speed_pid_loop(xiaomi_can1_id2_given_speed);


}


void xiaomi_angle_limit()
{
     xiaomi_can1_id1_init_angle = xiaomimotors[0].return_angle;
     xiaomi_can1_id2_init_angle = xiaomimotors[1].return_angle;

    if (xiaomi_can1_id1_given_angle > xiaomi_can1_id1_init_angle + 0.75f)
    {
        xiaomi_can1_id1_given_angle = xiaomi_can1_id1_init_angle + 0.75f;
    }
   else if (xiaomi_can1_id1_given_angle < xiaomi_can1_id1_init_angle - 0.75f)
    {
       xiaomi_can1_id1_given_angle = xiaomi_can1_id1_init_angle - 0.75f;
    }


    if (xiaomi_can1_id2_given_angle > xiaomi_can1_id2_init_angle + 0.75f)
    {
        xiaomi_can1_id2_given_angle = xiaomi_can1_id2_init_angle + 0.75f;
    }
    else if (xiaomi_can1_id2_given_angle < xiaomi_can1_id2_init_angle - 0.75f)
    {
       xiaomi_can1_id2_given_angle = xiaomi_can1_id2_init_angle - 0.75f;
    }
}


//xiaomi_id1_can2_pid
void xiaomi_can1_id1_speed_pid_init(void)
{
    static fp32 xiaomi_can1_id1_kpkikd[3] = {XIAOMI_CAN1_ID1_SPEED_PID_KP,XIAOMI_CAN1_ID1_SPEED_PID_KI,XIAOMI_CAN1_ID1_SPEED_PID_KD};
    PID_init(&xiaomi_can1_ID1_speed_pid, PID_POSITION, xiaomi_can1_id1_kpkikd, XIAOMI_CAN1_ID1_SPEED_PID_OUT_MAX, XIAOMI_CAN1_ID1_SPEED_PID_KI_MAX);

}

float xiaomi_can1_id1_speed_pid_loop(float xiaomi_can1_id1_speed_set_loop)
{
    PID_calc(&xiaomi_can1_ID1_speed_pid, xiaomimotors[0].fifilter_compute_speed , xiaomi_can1_id1_speed_set_loop);
    float xiaomi_can1_id1_speed_loop = (float)(xiaomi_can1_ID1_speed_pid.out);

    return xiaomi_can1_id1_speed_loop ;
}

void xiaomi_can1_id1_angle_pid_init(void)
{
    static fp32 xiaomi_can1_id1_angle_kpkikd[3] = {XIAOMI_CAN1_ID1_ANGLE_PID_KP, XIAOMI_CAN1_ID1_ANGLE_PID_KI, XIAOMI_CAN1_ID1_ANGLE_PID_KD};
    PID_init(&xiaomi_can1_ID1_angle_pid, PID_POSITION, xiaomi_can1_id1_angle_kpkikd, XIAOMI_CAN1_ID1_ANGLE_PID_OUT_MAX, XIAOMI_CAN1_ID1_ANGLE_PID_KI_MAX);

}

float xiaomi_can1_id1_angle_pid_loop(float xiaomi_can1_ID1_angle_set_loop)
{
    PID_calc(&xiaomi_can1_ID1_angle_pid,  xiaomimotors[0].return_angle , xiaomi_can1_ID1_angle_set_loop);
    float xiaomi_can1_ID1_given_speed_loop = (float)(xiaomi_can1_ID1_angle_pid.out);

    return xiaomi_can1_ID1_given_speed_loop ;

}






//xiaomi_can2_id2_pid
void xiaomi_can1_id2_speed_pid_init(void)
{
    static fp32 xiaomi_can1_id2_kpkikd[3] = {XIAOMI_CAN1_ID2_SPEED_PID_KP,XIAOMI_CAN1_ID2_SPEED_PID_KI,XIAOMI_CAN1_ID2_SPEED_PID_KD};
    PID_init(&xiaomi_can1_ID2_speed_pid, PID_POSITION, xiaomi_can1_id2_kpkikd, XIAOMI_CAN1_ID2_SPEED_PID_OUT_MAX, XIAOMI_CAN1_ID2_SPEED_PID_KI_MAX);

}

float xiaomi_can1_id2_speed_pid_loop(float xiaomi_can1_id2_speed_set_loop)
{
    PID_calc(&xiaomi_can1_ID2_speed_pid, xiaomimotors[1].fifilter_compute_speed , xiaomi_can1_id2_speed_set_loop);
    float xiaomi_can1_id2_speed_loop = (float)(xiaomi_can1_ID2_speed_pid.out);

    return xiaomi_can1_id2_speed_loop ;

}
void xiaomi_can1_id2_angle_pid_init(void)
{
    static fp32 xiaomi_can1_id2_angle_kpkikd[3] = {XIAOMI_CAN1_ID2_ANGLE_PID_KP, XIAOMI_CAN1_ID2_ANGLE_PID_KI, XIAOMI_CAN1_ID2_ANGLE_PID_KD};
    PID_init(&xiaomi_can1_ID2_angle_pid, PID_POSITION, xiaomi_can1_id2_angle_kpkikd, XIAOMI_CAN1_ID2_ANGLE_PID_OUT_MAX, XIAOMI_CAN1_ID2_ANGLE_PID_KI_MAX);

}

float xiaomi_can1_id2_angle_pid_loop(float xiaomi_can1_ID2_angle_set_loop)
{
    PID_calc(&xiaomi_can1_ID2_angle_pid, xiaomimotors[1].return_angle , xiaomi_can1_ID2_angle_set_loop);
    float xiaomi_can1_ID2_given_speed_loop = (float)(xiaomi_can1_ID2_angle_pid.out);

    return xiaomi_can1_ID2_given_speed_loop ;

}

