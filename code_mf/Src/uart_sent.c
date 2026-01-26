//
// Created by 21481 on 2025/3/16.
//

#include <stdio.h>
#include <string.h>
#include "main.h"
#include "cmsis_os.h"
#include "dma.h"
#include "usart.h"
#include "gpio.h"
#include "uart_printf.h"
#include "uart_sent.h"
#include "bsp_rc.h"
#include "remote_control.h"
#include "CAN_receive.h"
#include "jy61p.h"
#include "chassis_motor_control.h"
#include "auto_aim.h"
#include "can_comm.h"


void uart_sent_debug()
{
    while (1)
    {
        // usart6_printf("%d,%d\r\n",
        //               motor_can1_data[0].speed_rpm,
        //               motor_can1_data[1].speed_rpm);
         usart6_printf("%d,%d,%d,%f,%f,%f  \r\n",
       track_2006_can1_id8_speed,
        track_2006_can1_id8_current,
         motor_can1_data[7].speed_rpm,
         xiaomimotors[1].return_angle,
         xiaomimotors[1].last_angle,
        xiaomimotors[0].last_angle);



        osDelay(5);




    }

}


void aim_uart_sent()
{
    while (1)
    {
        sent_data_update();

        osDelay(5);
        osDelay(1);
    }
}




