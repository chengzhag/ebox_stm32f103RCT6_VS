#include "mpu9250.h"

const uint16_t MPU9250::gyroFsrList[] = { 250,500,1000,2000 };
const uint8_t MPU9250::accelFsrList[] = { 2,4,8,16 };

u8 MPU9250::writeByte(u8 addr, u8 reg, u8 data)
{
	u8 err = 0;
	i2c->take_i2c_right(speed);
	err = i2c->write_byte(addr, reg, data);
	i2c->release_i2c_right();
	return err;
}

u8 MPU9250::readByte(u8 addr, u8 reg)
{
	u8 data;
	i2c->take_i2c_right(speed);
	i2c->read_byte(addr, reg, &data);
	i2c->release_i2c_right();
	return data;
}

u8 MPU9250::readBytes(u8 addr, u8 reg, u8 len, u8 *buf)
{
	u8 err = 0;
	i2c->take_i2c_right(speed);
	//err = i2c->read_byte(addr, reg, buf, len);
	for (int i = 0; i < len; i++)
	{
		err = i2c->read_byte(addr, reg + i, buf + i);
	}
	i2c->release_i2c_right();
	return err;
}

u8 MPU9250::writeBytes(u8 addr, u8 reg, u8 len, u8 *buf)
{
	u8 err = 0;
	i2c->take_i2c_right(speed);
	//err = i2c->write_byte(addr, reg, buf, len);
	for (int i = 0; i < len; i++)
	{
		err = i2c->write_byte(addr, reg, buf[i]);
	}
	i2c->release_i2c_right();
	return err;
}

MPU9250::MPU9250(I2c* i2c, MPU6500_Model_Typedef model/*=Mpu9250_Model_9250*/) :
	i2c(i2c),
	model(model)
{
	if (model == MPU6500_Model_6500)
	{
		mpuID = MPU6500_ID;
	}
	else if (model == MPU6500_Model_6555)
	{
		mpuID = MPU6555_ID;
	}
}

u8 MPU9250::begin(uint32_t speed /*= 400000*/, uint16_t sampleRate /*= 100*/,
	MPU6500_Gyro_Full_Scale_Typedef gyroFsrReg/* = MPU9250_Gyro_Full_Scale_2000dps*/,
	MPU6500_Accel_Full_Scale_Typedef accelFsrReg/* = MPU9250_Accel_Full_Scale_8g*/)
{
	this->gyroFsrReg = gyroFsrReg;
	this->gyroFsr = this->gyroFsrList[gyroFsrReg];
	this->accelFsrReg = accelFsrReg;
	this->accelFsr = this->accelFsrList[accelFsrReg];
	this->sampleRate = sampleRate;
	this->speed = speed;
	i2c->take_i2c_right(speed);
	i2c->begin(speed);
	i2c->release_i2c_right();

	u8 res = 0;
	writeByte(MPU6500_ADDRESS, MPU6500_PWR_MGMT1_REG, 0X80);//复位MPU9250
	delay_ms(100);  //延时100ms
	writeByte(MPU6500_ADDRESS, MPU6500_PWR_MGMT1_REG, 0X00);//唤醒MPU9250
	setGyroFsr(gyroFsrReg);					        	//陀螺仪传感器,±2000dps
	setAccelFsr(accelFsrReg);					       	 	//加速度传感器,±2g
	setSampleRate(sampleRate);						       	 	//设置采样率50Hz
	writeByte(MPU6500_ADDRESS, MPU6500_INT_EN_REG, 0X00);   //关闭所有中断
	writeByte(MPU6500_ADDRESS, MPU6500_USER_CTRL_REG, 0X00);//I2C主模式关闭
	writeByte(MPU6500_ADDRESS, MPU6500_FIFO_EN_REG, 0X00);	//关闭FIFO
	writeByte(MPU6500_ADDRESS, MPU6500_INTBP_CFG_REG, 0X82);//INT引脚低电平有效，开启bypass模式，可以直接读取磁力计
	res = readByte(MPU6500_ADDRESS, MPU6500_DEVICE_ID_REG);  //读取MPU6500的ID
	if (res == mpuID) //器件ID正确
	{
		writeByte(MPU6500_ADDRESS, MPU6500_PWR_MGMT1_REG, 0X01);  	//设置CLKSEL,PLL X轴为参考
		writeByte(MPU6500_ADDRESS, MPU6500_PWR_MGMT2_REG, 0X00);  	//加速度与陀螺仪都工作
		setSampleRate(sampleRate);						       	//设置采样率为50Hz   
	}
	else return 1;

	res = readByte(AK8963_ADDRESS, AK8963_WHO_AM_I);    			//读取AK8963 ID   
	if (res == AK8963_ID)
	{
		writeByte(AK8963_ADDRESS, AK8963_CNTL, 0X11);		//设置AK8963为单次测量模式

		//// First extract the factory calibration for each magnetometer axis
		//uint8_t rawData[3];  // x/y/z gyro calibration data stored here
		//writeByte(AK8963_ADDRESS, AK8963_CNTL, 0x00); // Power down magnetometer  
		//delay_ms(10);
		//writeByte(AK8963_ADDRESS, AK8963_CNTL, 0x0F); // Enter Fuse ROM access mode
		//delay_ms(10);
		//readBytes(AK8963_ADDRESS, AK8963_ASAX, 3, &rawData[0]);  // Read the x-, y-, and z-axis calibration values
		//destination[0] = (float)(rawData[0] - 128) / 256.0f + 1.0f;   // Return x-axis sensitivity adjustment values, etc.
		//destination[1] = (float)(rawData[1] - 128) / 256.0f + 1.0f;
		//destination[2] = (float)(rawData[2] - 128) / 256.0f + 1.0f;
		//writeByte(AK8963_ADDRESS, AK8963_CNTL, 0x00); // Power down magnetometer  
		//wait(0.01);
		//// Configure the magnetometer for continuous read and highest resolution
		//// set Mscale bit 4 to 1 (0) to enable 16 (14) bit resolution in CNTL register,
		//// and enable continuous mode data acquisition Mmode (bits [3:0]), 0010 for 8 Hz and 0110 for 100 Hz sample rates
		//writeByte(AK8963_ADDRESS, AK8963_CNTL, Mscale << 4 | Mmode); // Set magnetometer data resolution and sample ODR
		//wait(0.01);
	}
	else return 1;

	return 0;
}

u8 MPU9250::setGyroFsr(u8 fsr)
{
	return writeByte(MPU6500_ADDRESS, MPU6500_GYRO_CFG_REG, fsr << 3);//设置陀螺仪满量程范围
}

u8 MPU9250::setAccelFsr(u8 fsr)
{
	return writeByte(MPU6500_ADDRESS, MPU6500_ACCEL_CFG_REG, fsr << 3);//设置加速度传感器满量程范围
}

u8 MPU9250::setLPF(u16 lpf)
{
	u8 data = 0;
	if (lpf >= 188)data = 1;
	else if (lpf >= 98)data = 2;
	else if (lpf >= 42)data = 3;
	else if (lpf >= 20)data = 4;
	else if (lpf >= 10)data = 5;
	else data = 6;
	return writeByte(MPU6500_ADDRESS, MPU6500_CFG_REG, data);//设置数字低通滤波器
}

u8 MPU9250::setSampleRate(u16 rate)
{
	u8 data;
	if (rate > 1000)rate = 1000;
	if (rate < 4)rate = 4;
	data = 1000 / rate - 1;
	data = writeByte(MPU6500_ADDRESS, MPU6500_SAMPLE_RATE_REG, data);	//设置数字低通滤波器
	return setLPF(rate / 2);	//自动设置LPF为采样率的一半
}

float MPU9250::getTemperature(void)
{
	u8 buf[2];
	short raw;
	float temp;
	readBytes(MPU6500_ADDRESS, MPU6500_TEMP_OUTH_REG, 2, buf);
	raw = ((u16)buf[0] << 8) | buf[1];
	temp = 21 + ((double)raw) / 333.87;
	return temp;
}

u8 MPU9250::getGyroscope(short *gx, short *gy, short *gz)
{
	u8 buf[6], res;
	res = readBytes(MPU6500_ADDRESS, MPU6500_GYRO_XOUTH_REG, 6, buf);
	if (res == 0)
	{
		*gx = ((u16)buf[0] << 8) | buf[1];
		*gy = ((u16)buf[2] << 8) | buf[3];
		*gz = ((u16)buf[4] << 8) | buf[5];
	}
	return res;
}

u8 MPU9250::getGyroscope(float *gx, float *gy, float *gz)
{
	u8  res;
	short x, y, z;
	res = getGyroscope(&x, &y, &z);
	//16.4 = 2^16/4000 lsb °/s     1/16.4=0.061     0.0174 = 3.14/180
	//陀螺仪数据从ADC转化为弧度每秒(这里需要减去偏移值)
	*gx = 2 * (float)x*gyroFsr / 65536 * 0.0174;
	*gy = 2 * (float)y*gyroFsr / 65536 * 0.0174;
	*gz = 2 * (float)z*gyroFsr / 65536 * 0.0174;	//读出值减去基准值乘以单位，计算陀螺仪角速度
	return res;
}

u8 MPU9250::getAccelerometer(short *ax, short *ay, short *az)
{
	u8 buf[6], res;
	res = readBytes(MPU6500_ADDRESS, MPU6500_ACCEL_XOUTH_REG, 6, buf);
	if (res == 0)
	{
		*ax = ((u16)buf[0] << 8) | buf[1];
		*ay = ((u16)buf[2] << 8) | buf[3];
		*az = ((u16)buf[4] << 8) | buf[5];
	}
	return res;
}

u8 MPU9250::getAccelerometer(float *ax, float *ay, float *az)
{
	u8  res;
	short x, y, z;
	res = getAccelerometer(&x, &y, &z);
	//+-8g,2^16/16=4096lsb/g--0.244mg/lsb
	//此处0.0098是：(9.8m/s^2)/1000,乘以mg得m/s^2
	*ax = (float)x*(2 * accelFsr)  * 9.8 / 65536;
	*ay = (float)y*(2 * accelFsr)  * 9.8 / 65536;
	*az = (float)z*(2 * accelFsr)  * 9.8 / 65536;
	return res;
}

u8 MPU9250::getMagnetometer(short *mx, short *my, short *mz)
{
	if (readByte(AK8963_ADDRESS, AK8963_ST1) & 0x01) // wait for magnetometer data ready bit to be set
	{
		u8 buf[7], res;
		res = readBytes(AK8963_ADDRESS, AK8963_XOUT_L, 7, buf);
		if (res == 0 && !(buf[6] & 0x08))// Check if magnetic sensor overflow set, if not then report data
		{
			*mx = ((u16)buf[1] << 8) | buf[0];
			*my = ((u16)buf[3] << 8) | buf[2];
			*mz = ((u16)buf[5] << 8) | buf[4];
		}
		else
		{
			*mx = 0;
			*my = 0;
			*mz = 0;
		}
		res = 1;
		writeByte(AK8963_ADDRESS, AK8963_CNTL, 0X11); //AK8963每次读完以后都需要重新设置为单次测量模式
		return res;
	}
	*mx = 0;
	*my = 0;
	*mz = 0;
	return 1;
}

u8 MPU9250::getMagnetometer(float *mx, float *my, float *mz)
{
	u8  res;
	short x, y, z;
	res = getMagnetometer(&x, &y, &z);
	//±4800uT 2^16/9600 = 6.83lsb/uT     1/6.83 = 0.1465
	//地磁强度为 5-6 x 10^(-5) T = 50 - 60 uT
	*mx = (float)x*0.1465;
	*my = (float)y*0.1465;
	*mz = (float)z*0.1465;
	return res;
}

u16 MPU9250::getSampleRate()
{
	return sampleRate;
}
