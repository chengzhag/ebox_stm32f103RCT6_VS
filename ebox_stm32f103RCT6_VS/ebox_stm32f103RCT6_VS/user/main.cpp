#include "ebox.h"
#include "mpu9250.h"
#include "my_math.h"
#include "uart_vcan.h"

FpsCounter fps;
MPU9250AHRS mpu(&i2c1,MPU9250_Model_9255);
UartVscan vscan(&uart1);

void setup()
{
	ebox_init();
	uart1.begin(115200);
	mpu.begin(400000, 200, MPU9250_Gyro_Full_Scale_2000dps, MPU9250_Accel_Full_Scale_8g);
	fps.begin();
}
int main(void)
{
	setup();
	//float t, gx, gy, gz, ax, ay, az, mx, my, mz;
	//float pitch, roll, yaw;
	float data[4];
	while (1)
	{
		mpu.getAngle(data, data + 1, data + 2);
		//uart1.printf("%f %f %f\r\n", pitch, roll, yaw);
		data[3] = fps.getFps();
		vscan.sendOscilloscope(data, 4);

		//mpu.getGyroscope(&gx, &gy, &gz);
		//mpu.getAccelerometer(&ax, &ay, &az);
		//mpu.getMagnetometer(&mx, &my, &mz);
		//t = mpu.getTemperature();
		//uart1.printf("%.3f\t%.3f\t%.3f\t%.3f\t%.3f\t%.3f\t%.3f\t%.3f\t%.3f\t%.3f\t%.3f\r\n",
		//	t, gx, gy, gz, ax, ay, az, mx, my, mz,fps.getFps());
		//uart1.printf("%d %d %d %d %d %d %d %d %d %d\r\n",
		//	t, gx, gy, gz, ax, ay, az, mx, my, mz);

		delay_ms(1);
	}

}


