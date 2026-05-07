//
// Created by 21481 on 2025/3/16.
//

#include "cmsis_os.h"
#include "uart_printf.h"
#include "uart_sent.h"




void uart_sent_debug()
{
    while (1)
    {

       usart6_printf("hello world!\r\n");
        osDelay(5);





    }

}




