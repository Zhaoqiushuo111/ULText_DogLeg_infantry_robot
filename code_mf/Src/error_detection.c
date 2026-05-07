//
// Created by 21481 on 2025/3/18.
//
#include "main.h"
#include "cmsis_os.h"
#include "error_detection.h"


void error_detection()
{
    while (1)
    {

        rc_connection_status();//遥控器离线判断
        osDelay(1);
    }


}


void rc_connection_status()
{
    if(HAL_GetTick() - rc_receive_time > RC_NO_DATA_TIMEOUT)
    {
        rc_receive_state = RC_OFFLINE ;//遥控器离线
    }
    else
    {
        rc_receive_state = RC_ONLINE ;//遥控器在线
    }
}



//留下一个电机过温反馈模板
// void yaw_6020_status()
// {
//     if(GM6020_TEMP_MAX > motor_can1_data[4].temperate)
//     {
//         yaw_6020_state = GM6020_SAFE ;//安全
//     }
//     else
//     {
//         yaw_6020_state = GM6020_DIE ;//过温
//     }
//
// }


