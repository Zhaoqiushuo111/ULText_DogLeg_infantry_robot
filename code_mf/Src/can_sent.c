//
// Created by 21481 on 2025/3/17.
//
#include "main.h"
#include "cmsis_os.h"
#include "can.h"
#include "dma.h"
#include "usart.h"
#include "gpio.h"

#include "board_LED.h"
#include "uart_printf.h"
#include "uart_sent.h"
#include "bsp_rc.h"
#include "remote_control.h"
#include "get_rc.h"
#include "bsp_can.h"
#include "CAN_receive.h"
#include "error_detection.h"
#include "can_sent.h"
#include "can_comm.h"

void can_sent()
{
    while (1)
    {
        if( rc_receive_state == RC_OFFLINE | yaw_6020_state == GM6020_DIE | pitch_6020_state == GM6020_DIE)//ң�������ߣ�ȫ���ϵ�
        {
            can_rm_cmd_all(0, 0, 0, 0, 0, 0, 0, 0, 0,0,0);
            //can_xiaomi_cmd_all(CLOSE_XIAOMI);
        }
        else//ң�������ߣ��������п��ؿ���
        {
            if(rc_s0 == 2)
            {
                can_rm_cmd_all(0, 0, 0, 0, 0, 0, 0, 0, 0,0,0);
                //can_xiaomi_cmd_all(CLOSE_XIAOMI);
            }
            else if(rc_s0 == 3 | rc_s0 == 1)//�˶�ģʽ
            {
//                can_cmd_all(0,
//                            0,
//                            0,
//                            0,
//                            0,
//                            PITCH_6020_ID2_GIVEN_CURRENT,
//                            0,
//                            0,
//                            0);
                //can_xiaomi_cmd_all(OPEN_XIAOMI);
                can_rm_cmd_all(CHASSIS_3508_ID1_GIVEN_CURRENT,
                               CHASSIS_3508_ID2_GIVEN_CURRENT,
                               CHASSIS_3508_ID3_GIVEN_CURRENT,
                               CHASSIS_3508_ID4_GIVEN_CURRENT,
                               YAW_6020_ID1_GIVEN_CURRENT,
                               PITCH_6020_ID2_GIVEN_CURRENT,
                               FRICTION_WHEEL_3510_ID1_GIVEN_CURRENT,
                               FRICTION_WHEEL_3510_ID2_GIVEN_CURRENT,
                               SHOOT_2006_ID3_GIVEN_CURRENT,
                               track_2006_can1_id7_current,
                               track_2006_can1_id8_current);
            }

            else//ң�������ݳ�ʼ���л����ȫ���ϵ�
            {
                can_rm_cmd_all(0, 0, 0, 0, 0, 0, 0, 0, 0,0,0);
                //can_xiaomi_cmd_all(CLOSE_XIAOMI);
            }

        }

        osDelay(1);


    if( rc_receive_state == RC_OFFLINE | yaw_6020_state == GM6020_DIE | pitch_6020_state == GM6020_DIE)//ң�������ߣ�ȫ���ϵ�
    {
        can_xiaomi_cmd_all(CLOSE_XIAOMI);
    }
    else//ң�������ߣ��������п��ؿ���
    {
        if(rc_s0 == 2)
        {
            can_xiaomi_cmd_all(CLOSE_XIAOMI);
        }
        else if(rc_s0 == 3 | rc_s0 == 1)//�˶�ģʽ
        {

            can_xiaomi_cmd_all(OPEN_XIAOMI);

        }

        else//ң�������ݳ�ʼ���л����ȫ���ϵ�
        {
            can_xiaomi_cmd_all(CLOSE_XIAOMI);
        }

    }

    osDelay(1);
    }
}


void can_rm_cmd_all(int16_t chassis_id1 , int16_t chassis_id2 ,
                    int16_t chassis_id3 , int16_t chassis_id4 ,
                    int16_t yaw_id1 , int16_t pitch_id2 ,
                    int16_t friction_wheel_id1, int16_t friction_wheel_id2 ,
                    int16_t shoot_id3, int16_t track_id7, int16_t track_id8 )
{
    CAN2_cmd_pitch(pitch_id2,0,0,0);
    CAN2_cmd_friction_wheels(friction_wheel_id1,friction_wheel_id2,0,0);
    CAN1_cmd_chassis(chassis_id1, chassis_id2, chassis_id3, chassis_id4);
    CAN1_cmd_yaw(yaw_id1,shoot_id3,track_id7,track_id8);

}

void can_xiaomi_cmd_all(uint8_t key)
{
    if(key == OPEN_XIAOMI)
    {
        CanComm_SendControlPara(xiaomimotors[1]);
        CanComm_SendControlPara(xiaomimotors[0]);
    } else
    {
        struct xiaomi_motor xiaomi_STOP ;
        xiaomi_STOP.can_channel = 0x01 ;//can1������С��

        xiaomi_STOP.can_id = 0x01 ;
        CanComm_SendControlPara(xiaomi_STOP);
        xiaomi_STOP.can_id = 0x02 ;
        CanComm_SendControlPara(xiaomi_STOP);
        xiaomi_STOP.can_id = 0x03 ;
        CanComm_SendControlPara(xiaomi_STOP);
        xiaomi_STOP.can_id = 0x04 ;
        CanComm_SendControlPara(xiaomi_STOP);

        xiaomi_STOP.can_channel = 0x02 ;//can1������С��

        xiaomi_STOP.can_id = 0x01 ;
        CanComm_SendControlPara(xiaomi_STOP);
        xiaomi_STOP.can_id = 0x02 ;
        CanComm_SendControlPara(xiaomi_STOP);
        xiaomi_STOP.can_id = 0x03 ;
        CanComm_SendControlPara(xiaomi_STOP);
        xiaomi_STOP.can_id = 0x04 ;
        CanComm_SendControlPara(xiaomi_STOP);





    }


}



