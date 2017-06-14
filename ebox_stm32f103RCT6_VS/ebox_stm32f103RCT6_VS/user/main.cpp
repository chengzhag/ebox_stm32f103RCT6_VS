#include "ebox.h"
#include "led.h"
#include "drv8825.h"

Led led(&PC13, 1);
//DRV8825 step1(&PA8, &PC7);
DRV8825 step2(&PA0, &PA4);
DRV8825 step3(&PA6, &PC8);
DRV8825 step4(&PB8, &PC9);

void setup()
{
    ebox_init();
    uart1.begin(115200);
	led.begin();
	//step1.begin();
	step2.begin();
	step3.begin();
	step4.begin();
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
	float pct = 0, increase = 0.05;
	while (1)
	{

		//step1.setPct(pct);
		step2.setPct(pct);
		step3.setPct(pct);
		step4.setPct(pct);

		pct += increase;
		if (pct >= 3 || pct <= -3)
		{
			increase = -increase;
		}

		led.toggle();
		uart1.printf("%f %d\r\n", step3.getPct(), step3.getFre());
		delay_ms(200);

	}

}


