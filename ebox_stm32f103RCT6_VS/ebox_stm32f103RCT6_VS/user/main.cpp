/**
  ******************************************************************************
  * @file   : *.cpp
  * @author : shentq
  * @version: V1.2
  * @date   : 2016/08/14

  * @brief   ebox application example .
  *
  * Copyright 2016 shentq. All Rights Reserved.         
  ******************************************************************************
 */

#include "ebox.h"
#include "uart_num.h"

void setup()
{
    ebox_init();
    uart1.begin(115200);
}
int main(void)
{
    setup();
    while(1)
    {
        uart1.printf("ok\n");
        delay_ms(1000);

    }

}


