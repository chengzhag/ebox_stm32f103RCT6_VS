
#include "ebox.h"
#include "powerLed.h"
PowerLed led1(&PA0);
void setup()
{
     ebox_init();
	led1.begin(500);
	PA8.mode(OUTPUT_PP);
}
int main()
{
	setup();
	int pct = 10;
	int increase = 1;
	while (1)
	{
	    if (pct<=0||pct>=1000)
	    {
			increase = -increase;
	    }
		pct = pct + increase;
		led1.setPct(pct);
		PA8.toggle();
		delay_ms(2);

	}
}
