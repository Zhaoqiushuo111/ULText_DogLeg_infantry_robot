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
        // usart6_printf("%f,%f,%f \r\n",
        //               xiaomimotors[0].given_speed,
        //               xiaomimotors[0].fifilter_compute_speed,
        //               xiaomimotors[0].give_tor);
        usart6_printf("%f,%f,%d,%f,%f \r\n",
       YAW_6020_ID1_GIVEN_SPEED,
       auto_aim_rx_packet.yaw ,
        YAW_6020_ID1_GIVEN_CURRENT,
        YAW_6020_ID1_GIVEN_ANGLE,
        yaw_angle_from_bmi088
      );



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




