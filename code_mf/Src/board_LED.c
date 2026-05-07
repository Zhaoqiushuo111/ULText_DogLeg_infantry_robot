//
// Created by 21481 on 2025/3/16.
//
#include "main.h"
#include "cmsis_os.h"
#include "boards/bsp_buzzer.h"

uint8_t light = 1;

uint16_t psc = 0;


void board_LED()
{

    while (1)
    {
        RC_ctrl_Air_pump();
        //Max_Light();
        //KEY_Scan();

        switch (light)
        {
            case(0):
            {
                    HAL_GPIO_WritePin( GPIOE,  GPIO_PIN_11, GPIO_PIN_SET);

                HAL_GPIO_WritePin( GPIOH,  GPIO_PIN_11, GPIO_PIN_SET);
                osDelay(100);
                HAL_GPIO_WritePin( GPIOH,  GPIO_PIN_11, GPIO_PIN_RESET);
                osDelay(100);
                    break;
            }
            case(1):
            {

                    HAL_GPIO_WritePin( GPIOE,  GPIO_PIN_11, GPIO_PIN_RESET);

                HAL_GPIO_WritePin( GPIOH,  GPIO_PIN_12, GPIO_PIN_SET);
                osDelay(100);
                HAL_GPIO_WritePin( GPIOH,  GPIO_PIN_12, GPIO_PIN_RESET);
                osDelay(100);
                    break;
            }
            // case(2):
            // {
            //         HAL_GPIO_WritePin( GPIOH,  GPIO_PIN_12, GPIO_PIN_SET);
            //         osDelay(100);
            //         HAL_GPIO_WritePin( GPIOH,  GPIO_PIN_12, GPIO_PIN_RESET);
            //         osDelay(100);
            //         break;
            // }
            default:

        }
        osDelay(1);

    }

}

void RC_ctrl_Air_pump()
{
   if (rc_s0 == 2)
   {
       light = 1;
   }
    if (rc_s0 == 3)
    {
        light = 0;
    }
}

//
// void Max_Light()
// {
//     if (light > 1)
//     {
//         light = 0;
//     }
// }
//
//
//
// void KEY_Scan()
// {
//     //低电平 = 按下
//     if (HAL_GPIO_ReadPin(GPIOA,GPIO_PIN_0) == GPIO_PIN_RESET)
//     {
//         osDelay(20);//消抖
//         if (HAL_GPIO_ReadPin(GPIOA,GPIO_PIN_0) == GPIO_PIN_RESET)
//         {
//             while (HAL_GPIO_ReadPin(GPIOA,GPIO_PIN_0) == GPIO_PIN_RESET);//等待松开
//             buzzer_on(0, 500);
//             osDelay(100);
//             buzzer_on(1, 500);
//             osDelay(100);
//             buzzer_off();
//             light ++ ;
//
//         }
//     }
//
// }
