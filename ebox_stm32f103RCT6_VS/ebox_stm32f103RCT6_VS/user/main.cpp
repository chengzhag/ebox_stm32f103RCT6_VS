#include "ebox.h"
#include "led.h"
#include "drv8825.h"

Led led(&PC13, 1);
DRV8825 step(&PB8, &PC9);

void setup()
{
	ebox_init();
	uart1.begin(115200);
	led.begin();
	step.begin();
}
int main(void)
{
	setup();
	float pct = 0, increase = 0.001;
	while (1)
	{
		pct += increase;
		if (pct >= 0.03 || pct <= -0.03)
		{
			increase = -increase;
		}
		step.setPct(pct);


		led.toggle();
		uart1.printf("%f\r\n", step.getPct());
		delay_ms(100);
		if (millis()>5000)
		{
			break;
		}
	}

	//测试设置频率为0时的情况
	step.setPct(0);
	while (1);

}


