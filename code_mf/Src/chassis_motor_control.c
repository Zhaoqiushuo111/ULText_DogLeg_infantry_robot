//
// Created by 111 on 2025/12/22.
//

#include "chassis_motor_control.h"
#include "cmsis_os.h"
#include "main.h"
#include "remote_control.h"
#include "CAN_receive.h"
#include "pid.h"
#include <math.h>

pid_type_def chassis_3508_ID1_speed_pid;
pid_type_def chassis_3508_ID2_speed_pid;
pid_type_def chassis_3508_ID3_speed_pid;
pid_type_def chassis_3508_ID4_speed_pid;

pid_type_def chassis_follow_gimbal_pid;


void chassis_motor_control() {

    while (1) {

        chassis_speed_compute();//底盘速度解算
        chassis_settlement();//轮系速度解算
        motor_chassis_pid_compute();//pid速度

        osDelay((1/CHASSIS_PID_COMPUTE_FREQUENCY)*1000);




#if CHASSIS_PID_COMPUTE_FREQUENCY == 0
        osDelay(1);

#endif




        osDelay(10);
    }


}

void rc_to_gimbal_speed_compute()
{

        gimbal_vy = (float)(4 * rc_ch1 ) ;
        gimbal_vx = (float)(4 * rc_ch0 ) ;

}



void chassis_speed_compute()
{
     chassis_vx = rc_ch0*5.0f;
     chassis_vy = rc_ch1*5.8f;//上限11.8f
//     chassis_vround = rc_ch0*5.0f;
    // chassis_vx = gimbal_vx * (float)cos((double)yaw_radian_difference) - gimbal_vy * (float)sin((double)yaw_radian_difference);
    // chassis_vy = gimbal_vx * (float)sin((double)yaw_radian_difference) + gimbal_vy * (float)cos((double)yaw_radian_difference);
 }



void chassis_settlement()
{


    //先进行普通目标速度计算
    CHASSIS_3508_ID1_VXY_COMPUTE_SPEED = (int16_t)(-chassis_vy + chassis_vx ) ;
    CHASSIS_3508_ID2_VXY_COMPUTE_SPEED = (int16_t)(chassis_vy + chassis_vx ) ;
    CHASSIS_3508_ID3_VXY_COMPUTE_SPEED = (int16_t)(chassis_vy - chassis_vx ) ;
    CHASSIS_3508_ID4_VXY_COMPUTE_SPEED = (int16_t)(-chassis_vy - chassis_vx ) ;



    CHASSIS_3508_ID1_GIVEN_SPEED = (int16_t)((float)CHASSIS_3508_ID1_VXY_COMPUTE_SPEED + chassis_vround ) ;
    CHASSIS_3508_ID2_GIVEN_SPEED = (int16_t)((float)CHASSIS_3508_ID2_VXY_COMPUTE_SPEED + chassis_vround ) ;
    CHASSIS_3508_ID3_GIVEN_SPEED = (int16_t)((float)CHASSIS_3508_ID3_VXY_COMPUTE_SPEED + chassis_vround ) ;
    CHASSIS_3508_ID4_GIVEN_SPEED = (int16_t)((float)CHASSIS_3508_ID4_VXY_COMPUTE_SPEED + chassis_vround ) ;


    CHASSIS_FOLLOW_GIMBAL_GIVEN_SPEED = chassis_follow_gimbal_pid_loop(YAW_MID_ECD);//底盘跟随
    chassis_vround = CHASSIS_FOLLOW_GIMBAL_GIVEN_SPEED ;

    // send_out_all_speed =
    //         fabsf((float)CHASSIS_3508_ID1_GIVEN_SPEED) +
    //         fabsf((float)CHASSIS_3508_ID2_GIVEN_SPEED) +
    //         fabsf((float)CHASSIS_3508_ID3_GIVEN_SPEED) +
    //         fabsf((float)CHASSIS_3508_ID4_GIVEN_SPEED) ;


}


//loop
void motor_chassis_pid_compute()
{
    CHASSIS_3508_ID1_GIVEN_CURRENT = chassis_3508_id1_speed_pid_loop(CHASSIS_3508_ID1_GIVEN_SPEED);
    CHASSIS_3508_ID2_GIVEN_CURRENT = chassis_3508_id2_speed_pid_loop(CHASSIS_3508_ID2_GIVEN_SPEED);
    CHASSIS_3508_ID3_GIVEN_CURRENT = chassis_3508_id3_speed_pid_loop(CHASSIS_3508_ID3_GIVEN_SPEED);
    CHASSIS_3508_ID4_GIVEN_CURRENT = chassis_3508_id4_speed_pid_loop(CHASSIS_3508_ID4_GIVEN_SPEED);

}



//pid control

//1号电机
void chassis_3508_id1_speed_pid_init(void)
{
    static fp32 chassis_3508_id1_speed_kpkikd[3] = {CHASSIS_3508_ID1_SPEED_PID_KP,CHASSIS_3508_ID1_SPEED_PID_KI,CHASSIS_3508_ID1_SPEED_PID_KD};
    PID_init(&chassis_3508_ID1_speed_pid,PID_POSITION,chassis_3508_id1_speed_kpkikd,CHASSIS_3508_SPEED_PID_OUT_MAX,CHASSIS_3508_SPEED_PID_KI_MAX);

}

int16_t chassis_3508_id1_speed_pid_loop(int16_t chassis_3508_ID1_speed_set_loop)
{
    PID_calc(&chassis_3508_ID1_speed_pid, motor_can1_data[0].speed_rpm, chassis_3508_ID1_speed_set_loop);
    int16_t chassis_3508_ID1_given_current_loop = (int16_t)(chassis_3508_ID1_speed_pid.out);

    return chassis_3508_ID1_given_current_loop ;

}



//2号电机
void chassis_3508_id2_speed_pid_init(void)
{
    static fp32 chassis_3508_id2_speed_kpkikd[3] = {CHASSIS_3508_ID2_SPEED_PID_KP,CHASSIS_3508_ID2_SPEED_PID_KI,CHASSIS_3508_ID2_SPEED_PID_KD};
    PID_init(&chassis_3508_ID2_speed_pid,PID_POSITION,chassis_3508_id2_speed_kpkikd,CHASSIS_3508_SPEED_PID_OUT_MAX,CHASSIS_3508_SPEED_PID_KI_MAX);

}

int16_t chassis_3508_id2_speed_pid_loop(int16_t chassis_3508_ID2_speed_set_loop)
{
    PID_calc(&chassis_3508_ID2_speed_pid, motor_can1_data[1].speed_rpm, chassis_3508_ID2_speed_set_loop);
    int16_t chassis_3508_ID2_given_current_loop = (int16_t)(chassis_3508_ID2_speed_pid.out);

    return chassis_3508_ID2_given_current_loop ;

}



//3号电机
void chassis_3508_id3_speed_pid_init(void)
{
    static fp32 chassis_3508_id3_speed_kpkikd[3] = {CHASSIS_3508_ID3_SPEED_PID_KP,CHASSIS_3508_ID3_SPEED_PID_KI,CHASSIS_3508_ID3_SPEED_PID_KD};
    PID_init(&chassis_3508_ID3_speed_pid,PID_POSITION,chassis_3508_id3_speed_kpkikd,CHASSIS_3508_SPEED_PID_OUT_MAX,CHASSIS_3508_SPEED_PID_KI_MAX);

}

int16_t chassis_3508_id3_speed_pid_loop(int16_t chassis_3508_ID3_speed_set_loop)
{
    PID_calc(&chassis_3508_ID3_speed_pid, motor_can1_data[2].speed_rpm , chassis_3508_ID3_speed_set_loop);
    int16_t chassis_3508_ID3_given_current_loop = (int16_t)(chassis_3508_ID3_speed_pid.out);

    return chassis_3508_ID3_given_current_loop ;

}



//4号电机
void chassis_3508_id4_speed_pid_init(void)
{
    static fp32 chassis_3508_id4_speed_kpkikd[3] = {CHASSIS_3508_ID4_SPEED_PID_KP,CHASSIS_3508_ID4_SPEED_PID_KI,CHASSIS_3508_ID4_SPEED_PID_KD};
    PID_init(&chassis_3508_ID4_speed_pid,PID_POSITION,chassis_3508_id4_speed_kpkikd,CHASSIS_3508_SPEED_PID_OUT_MAX,CHASSIS_3508_SPEED_PID_KI_MAX);

}

int16_t chassis_3508_id4_speed_pid_loop(int16_t chassis_3508_ID4_speed_set_loop)
{
    PID_calc(&chassis_3508_ID4_speed_pid, motor_can1_data[3].speed_rpm , chassis_3508_ID4_speed_set_loop);
    int16_t chassis_3508_ID4_given_current_loop = (int16_t)(chassis_3508_ID4_speed_pid.out);

    return chassis_3508_ID4_given_current_loop ;

}


//CHASSIS_FOLLOW_GIMBAL_ANGLE_PID
void chassis_follow_gimbal_angle_pid_init(void)
{
    static fp32 chassis_follow_gimbal_angle_kpkikd[3] = {CHASSIS_FOLLOW_GIMBAL_ANGLE_PID_KP,CHASSIS_FOLLOW_GIMBAL_ANGLE_PID_KI,CHASSIS_FOLLOW_GIMBAL_ANGLE_PID_KD};
    PID_init(&chassis_follow_gimbal_pid, PID_POSITION, chassis_follow_gimbal_angle_kpkikd, CHASSIS_FOLLOW_GIMBAL_ANGLE_PID_OUT_MAX, CHASSIS_FOLLOW_GIMBAL_ANGLE_PID_KI_MAX);

}

float chassis_follow_gimbal_pid_loop(float PITCH_6020_ID2_angle_set_loop)
{
    PID_calc(&chassis_follow_gimbal_pid, motor_can1_data[4].ecd , PITCH_6020_ID2_angle_set_loop);
    float chassis_follow_gimbal_angle_loop = (float)(chassis_follow_gimbal_pid.out);

    return chassis_follow_gimbal_angle_loop ;

}


