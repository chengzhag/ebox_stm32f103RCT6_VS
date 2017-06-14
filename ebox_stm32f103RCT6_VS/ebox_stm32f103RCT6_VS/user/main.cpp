#include "ebox.h"
#include "led.h"
#include "tb6612fng.h"

Led led(&PC13, 1);
TB6612FNG motor1(&PA4, &PA12, &PA0);
TB6612FNG motor2(&PA15, &PB2, &PA1);
TB6612FNG motor3(&PB12, &PC6, &PA2);

void setup()
{
    ebox_init();
    uart1.begin(115200);
	led.begin();
	motor1.begin();
	motor2.begin();
	motor3.begin();

}


//ÏìÓ¦²âÊÔ
//int main(void)
//{
//	setup();
//	float pct = 100;
//	while (1)
//	{
//
//		step1.setPercent(pct);
//		step2.setPercent(pct);
//
//		pct = -pct;
//
//		led.toggle();
//		uart1.printf("%f\r\n", step1.getPercent());
//		delay_ms(100);
//
//	}
//
//}

//ËÙ¶È·¶Î§²âÊÔ
int main(void)
{
	setup();
	float pct = 0, increase = 2;
	while (1)
	{

		motor1.setPercent(pct);
		motor2.setPercent(pct);
		motor3.setPercent(pct);

		pct += increase;
		if (pct >= 50 || pct <= -50)
		{
			increase = -increase;
		}

		led.toggle();
		uart1.printf("%f\r\n", pct);
		delay_ms(100);

	}

}


