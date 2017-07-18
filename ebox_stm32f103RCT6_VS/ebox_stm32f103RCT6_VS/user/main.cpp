#include "ebox.h"
#include "signal_stream.h"
#include <math.h>
#include "uart_vcan.h"

RcFilter rcFilter(50, 1);
UartVscan vscan(&uart1);
void setup()
{
	ebox_init();
	vscan.begin(115200);
	PA8.mode(OUTPUT_PP);
}
int main()
{
	int f0 = 1;
	int fs = 100;
	float y[3];
	float signal[2];
	setup();
	while (1)
	{
		for (int i = 0; i < 500; i++)
		{
			y[0] = 100*sin(2 * pi*f0*i / fs);
			y[1] = rand() % 200 - 100;
			signal[0] = y[0] + 0.3*y[1];
			//vscan.sendOscilloscope(y, 2);
			signal[1] = rcFilter.getFilterOut(signal[0]);
			vscan.sendOscilloscope(signal, 2);
		}
		PA8.toggle();
	}
	return 0;

}