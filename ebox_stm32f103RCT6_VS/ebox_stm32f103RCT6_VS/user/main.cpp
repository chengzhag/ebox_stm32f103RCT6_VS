#include "ebox.h"
#include "led.h"
#include "servo.h"


Led led(&PC13, 1);
Servo servo1(&PA6);
Servo servo2(&PA7);
Servo servo3(&PB0);
Servo servo4(&PB1);

void setup()
{
    ebox_init();
    uart1.begin(115200);
	led.begin();

	servo1.begin();
	servo2.begin();
	servo3.begin();
	servo4.begin();
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
	float pct = 0, increase = 5;
	while (1)
	{

		pct += increase;
		if (pct >= 100 || pct <= 0)
		{
			increase = -increase;
		}

		servo1.setPct(pct);
		servo2.setPct(pct);
		servo3.setPct(pct);
		servo4.setPct(pct);

		led.toggle();
		uart1.printf("%f\r\n", pct);
		delay_ms(50);

	}

}


