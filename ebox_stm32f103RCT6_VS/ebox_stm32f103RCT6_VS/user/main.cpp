#include "ebox.h"
#include "led.h"
#include "oled_i2c.h"
#include "Button.h"
#include "font.h"

OLEDI2C oled(&i2c2);

void setup()
{
	ebox_init();
	uart1.begin(115200);
	oled.begin();
}
int main(void)
{
	setup();
	int num = 1512;
	while (1)
	{
		oled.show_string(0, 0, "ok!!!!", 1);
		delay_ms(1000);
		oled.show_string(0, 0, "ok!!!!", 2);
		delay_ms(1000);
		oled.show_chinese(0, 2, 0);
		delay_ms(1000);
		oled.printf(0, 4, 1, "ok!!%.3f %d", 0.41, num);
		delay_ms(1000);
		oled.draw_bmp(0, 0, 128, 8, (unsigned char *)BMP1);
		delay_ms(1000);
		oled.draw_bmp(0, 0, 128, 8, (unsigned char *)BMP2);
		delay_ms(1000);
		oled.clear();
	}

}


