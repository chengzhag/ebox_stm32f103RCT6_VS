#include "ebox.h"
#include "mpu6500.h"
#include "my_math.h"
#include "uart_vcan.h"



FpsCounter fps;
MPU6500 mpu(&i2c1,MPU6500_Model_6555);
UartVscan vscan(&uart1);

//#define SEND_ANGLE
#define SEND_RAW

void setup()
{
	ebox_init();
	uart1.begin(115200);
	mpu.begin();
	fps.begin();
}
int main(void)
{
	setup();
#ifdef SEND_RAW
	float t, gx, gy, gz, ax, ay, az, mx, my, mz;
#endif // SEND_RAW
#ifdef SEND_ANGLE
	float data[4];
#endif // SEND_ANGLE

	
	while (1)
	{
#ifdef SEND_ANGLE
		mpu.getAngle(data, data + 1, data + 2);
		data[3] = fps.getFps();
		vscan.sendOscilloscope(data, 4);
#endif // SEND_ANGLE

#ifdef SEND_RAW
		mpu.getGyroscope(&gx, &gy, &gz);
		mpu.getAccelerometer(&ax, &ay, &az);
		//mpu.getMagnetometer(&mx, &my, &mz);
		t = mpu.getTemperature();
		uart1.printf("%.3f\t%.3f\t%.3f\t%.3f\t%.3f\t%.3f\t%.3f\t%.3f\r\n",
			t, gx, gy, gz, ax, ay, az, fps.getFps());//读取六轴标准单位数据
		//uart1.printf("%.3f\t%.3f\t%.3f\t%.3f\t%.3f\t%.3f\t%.3f\t%.3f\t%.3f\t%.3f\t%.3f\r\n",
		//	t, gx, gy, gz, ax, ay, az, mx, my, mz,fps.getFps());
		//uart1.printf("%d\t%d\t%d\t%d\t%d\t%d\t%d\t%.3f\r\n",
		//	t, gx, gy, gz, ax, ay, az, fps.getFps());//读取六轴原始数据
		//uart1.printf("%d\t%d\t%d\t%d\t%d\t%d\t%d\t%d\t%d\t%d\t%.3f\r\n",
		//	t, gx, gy, gz, ax, ay, az, mx, my, mz, fps.getFps());

#endif // SEND_RAW
		
		//delay_ms(7);
	}

}


