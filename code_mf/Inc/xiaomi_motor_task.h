//
// Created by 21481 on 2025/11/9.
//

#ifndef BUBING_RM2025_XIAOMI_MOTOR_TASK_H
#define BUBING_RM2025_XIAOMI_MOTOR_TASK_H

#define XIAOMI_CAN1_ID1_SPEED_PID_KP        0.0001f
#define XIAOMI_CAN1_ID1_SPEED_PID_KI        0.01f
#define XIAOMI_CAN1_ID1_SPEED_PID_KD        0.0f
#define XIAOMI_CAN1_ID1_SPEED_PID_OUT_MAX   5.0f
#define XIAOMI_CAN1_ID1_SPEED_PID_KI_MAX    15.0f

#define XIAOMI_CAN1_ID1_ANGLE_PID_KP        0.5f
#define XIAOMI_CAN1_ID1_ANGLE_PID_KI        0.0f
#define XIAOMI_CAN1_ID1_ANGLE_PID_KD        0.0f
#define XIAOMI_CAN1_ID1_ANGLE_PID_OUT_MAX   3.0f
#define XIAOMI_CAN1_ID1_ANGLE_PID_KI_MAX    15.0f




#define XIAOMI_CAN2_ID2_SPEED_PID_KP        1.0f
#define XIAOMI_CAN2_ID2_SPEED_PID_KI        0.01f
#define XIAOMI_CAN2_ID2_SPEED_PID_KD        0.0f
#define XIAOMI_CAN2_ID2_SPEED_PID_OUT_MAX   1.0f
#define XIAOMI_CAN2_ID2_SPEED_PID_KI_MAX    15.0f


#define XIAOMI_CAN2_ID2_ANGLE_PID_KP        0.0f
#define XIAOMI_CAN2_ID2_ANGLE_PID_KI        0.0f
#define XIAOMI_CAN2_ID2_ANGLE_PID_KD        0.0f
#define XIAOMI_CAN2_ID2_ANGLE_PID_OUT_MAX   1.0f
#define XIAOMI_CAN2_ID2_ANGLE_PID_KI_MAX    15.0f



extern pid_type_def xiaomi_can2_ID1_speed_pid;
extern pid_type_def xiaomi_can2_ID2_speed_pid;

void xiaomi_pid_compute();
void xiaomi_angle_limit();

void xiaomi_can1_id1_speed_pid_init(void);
float xiaomi_can1_id1_speed_pid_loop(float xiaomi_can2_id1_speed_set_loop);
void xiaomi_can1_id1_angle_pid_init(void);
float xiaomi_can1_id1_angle_pid_loop(float xiaomi_can1_ID1_angle_set_loop);
void xiaomi_can1_id2_speed_pid_init(void);
float xiaomi_can1_id2_speed_pid_loop(float xiaomi_can2_id2_speed_set_loop);
void xiaomi_can1_id2_angle_pid_init(void);
float xiaomi_can1_id2_angle_pid_loop(float xiaomi_can1_ID2_angle_set_loop);


#endif //BUBING_RM2025_XIAOMI_MOTOR_TASK_H
