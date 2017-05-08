#ifndef __ULRRASONIC_WAVE_UART
#define __ULRRASONIC_WAVE_UART

#include "ebox.h"


//基础超声波模块驱动
class UltrasonicWaveUart
{
    Uart *uart;
    uint16_t dis;
    bool isHigh;
    bool isReady;
public:

    //构造函数，确定Uart
    UltrasonicWaveUart(Uart *uartX);

    //初始化函数，初始化uart口
    void begin();

    //uart字节处理中断函数
    void rxEvent();

    //基于忙等的read函数
    uint16_t read();

    //发送触发指令，开始测距
    void trig();
};


#endif