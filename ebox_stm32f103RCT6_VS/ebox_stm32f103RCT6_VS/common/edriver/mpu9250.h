#ifndef __MPU6050_H
#define __MPU6050_H

#include "ebox.h"
#include <math.h>

////////////////////////////////////////////////////////////////////////////////// 	
//如果AD0脚(9脚)接地,IIC地址为0X68(不包含最低位).
//如果接V3.3,则IIC地址为0X69(不包含最低位).
//#define MPU9250_ADDR            0X68    //MPU6500的器件IIC地址
#define MPU9250_ADDR            0xD0
#define MPU6500_ID				0X71  	//MPU6500的器件ID
#define MPU6555_ID				0X73

//MPU9250内部封装了一个AK8963磁力计,地址和ID如下:
//#define AK8963_ADDR				0X0C	//AK8963的I2C地址
#define AK8963_ADDR				0x18	//AK8963的I2C地址
#define AK8963_ID				0X48	//AK8963的器件ID


//AK8963的内部寄存器
#define MAG_WIA					0x00	//AK8963的器件ID寄存器地址
#define MAG_CNTL1          	  	0X0A    
#define MAG_CNTL2            	0X0B

#define MAG_XOUT_L				0X03	
#define MAG_XOUT_H				0X04
#define MAG_YOUT_L				0X05
#define MAG_YOUT_H				0X06
#define MAG_ZOUT_L				0X07
#define MAG_ZOUT_H				0X08

//MPU6500的内部寄存器
#define MPU_SELF_TESTX_REG		0X0D	//自检寄存器X
#define MPU_SELF_TESTY_REG		0X0E	//自检寄存器Y
#define MPU_SELF_TESTZ_REG		0X0F	//自检寄存器Z
#define MPU_SELF_TESTA_REG		0X10	//自检寄存器A
#define MPU_SAMPLE_RATE_REG		0X19	//采样频率分频器
#define MPU_CFG_REG				0X1A	//配置寄存器
#define MPU_GYRO_CFG_REG		0X1B	//陀螺仪配置寄存器
#define MPU_ACCEL_CFG_REG		0X1C	//加速度计配置寄存器
#define MPU_MOTION_DET_REG		0X1F	//运动检测阀值设置寄存器
#define MPU_FIFO_EN_REG			0X23	//FIFO使能寄存器
#define MPU_I2CMST_CTRL_REG		0X24	//IIC主机控制寄存器
#define MPU_I2CSLV0_ADDR_REG	0X25	//IIC从机0器件地址寄存器
#define MPU_I2CSLV0_REG			0X26	//IIC从机0数据地址寄存器
#define MPU_I2CSLV0_CTRL_REG	0X27	//IIC从机0控制寄存器
#define MPU_I2CSLV1_ADDR_REG	0X28	//IIC从机1器件地址寄存器
#define MPU_I2CSLV1_REG			0X29	//IIC从机1数据地址寄存器
#define MPU_I2CSLV1_CTRL_REG	0X2A	//IIC从机1控制寄存器
#define MPU_I2CSLV2_ADDR_REG	0X2B	//IIC从机2器件地址寄存器
#define MPU_I2CSLV2_REG			0X2C	//IIC从机2数据地址寄存器
#define MPU_I2CSLV2_CTRL_REG	0X2D	//IIC从机2控制寄存器
#define MPU_I2CSLV3_ADDR_REG	0X2E	//IIC从机3器件地址寄存器
#define MPU_I2CSLV3_REG			0X2F	//IIC从机3数据地址寄存器
#define MPU_I2CSLV3_CTRL_REG	0X30	//IIC从机3控制寄存器
#define MPU_I2CSLV4_ADDR_REG	0X31	//IIC从机4器件地址寄存器
#define MPU_I2CSLV4_REG			0X32	//IIC从机4数据地址寄存器
#define MPU_I2CSLV4_DO_REG		0X33	//IIC从机4写数据寄存器
#define MPU_I2CSLV4_CTRL_REG	0X34	//IIC从机4控制寄存器
#define MPU_I2CSLV4_DI_REG		0X35	//IIC从机4读数据寄存器

#define MPU_I2CMST_STA_REG		0X36	//IIC主机状态寄存器
#define MPU_INTBP_CFG_REG		0X37	//中断/旁路设置寄存器
#define MPU_INT_EN_REG			0X38	//中断使能寄存器
#define MPU_INT_STA_REG			0X3A	//中断状态寄存器

#define MPU_ACCEL_XOUTH_REG		0X3B	//加速度值,X轴高8位寄存器
#define MPU_ACCEL_XOUTL_REG		0X3C	//加速度值,X轴低8位寄存器
#define MPU_ACCEL_YOUTH_REG		0X3D	//加速度值,Y轴高8位寄存器
#define MPU_ACCEL_YOUTL_REG		0X3E	//加速度值,Y轴低8位寄存器
#define MPU_ACCEL_ZOUTH_REG		0X3F	//加速度值,Z轴高8位寄存器
#define MPU_ACCEL_ZOUTL_REG		0X40	//加速度值,Z轴低8位寄存器

#define MPU_TEMP_OUTH_REG		0X41	//温度值高八位寄存器
#define MPU_TEMP_OUTL_REG		0X42	//温度值低8位寄存器

#define MPU_GYRO_XOUTH_REG		0X43	//陀螺仪值,X轴高8位寄存器
#define MPU_GYRO_XOUTL_REG		0X44	//陀螺仪值,X轴低8位寄存器
#define MPU_GYRO_YOUTH_REG		0X45	//陀螺仪值,Y轴高8位寄存器
#define MPU_GYRO_YOUTL_REG		0X46	//陀螺仪值,Y轴低8位寄存器
#define MPU_GYRO_ZOUTH_REG		0X47	//陀螺仪值,Z轴高8位寄存器
#define MPU_GYRO_ZOUTL_REG		0X48	//陀螺仪值,Z轴低8位寄存器

#define MPU_I2CSLV0_DO_REG		0X63	//IIC从机0数据寄存器
#define MPU_I2CSLV1_DO_REG		0X64	//IIC从机1数据寄存器
#define MPU_I2CSLV2_DO_REG		0X65	//IIC从机2数据寄存器
#define MPU_I2CSLV3_DO_REG		0X66	//IIC从机3数据寄存器

#define MPU_I2CMST_DELAY_REG	0X67	//IIC主机延时管理寄存器
#define MPU_SIGPATH_RST_REG		0X68	//信号通道复位寄存器
#define MPU_MDETECT_CTRL_REG	0X69	//运动检测控制寄存器
#define MPU_USER_CTRL_REG		0X6A	//用户控制寄存器
#define MPU_PWR_MGMT1_REG		0X6B	//电源管理寄存器1
#define MPU_PWR_MGMT2_REG		0X6C	//电源管理寄存器2 
#define MPU_FIFO_CNTH_REG		0X72	//FIFO计数寄存器高八位
#define MPU_FIFO_CNTL_REG		0X73	//FIFO计数寄存器低八位
#define MPU_FIFO_RW_REG			0X74	//FIFO读写寄存器
#define MPU_DEVICE_ID_REG		0X75	//器件ID寄存器

typedef enum
{
	MPU9250_Model_9250,
	MPU9250_Model_9255
}MPU9250_Model_Typedef;

typedef enum
{
	MPU9250_Gyro_Full_Scale_250dps=0,
	MPU9250_Gyro_Full_Scale_500dps =1,
	MPU9250_Gyro_Full_Scale_1000dps = 2,
	MPU9250_Gyro_Full_Scale_2000dps = 3
}MPU9250_Gyro_Full_Scale_Typedef;

typedef enum
{
	MPU9250_Accel_Full_Scale_2g = 0,
	MPU9250_Accel_Full_Scale_4g = 1,
	MPU9250_Accel_Full_Scale_8g = 2,
	MPU9250_Accel_Full_Scale_16g = 3
}MPU9250_Accel_Full_Scale_Typedef;

class MPU9250
{
protected:
	I2c* i2c;
	uint32_t speed;
	u8 model;
	u8 mpuID;
	u16 sampleRate;
	u8 gyroFsrReg;
	u8 accelFsrReg;
	uint16_t gyroFsr, accelFsr;

	//IIC写一个字节 
	//devaddr:器件IIC地址
	//reg:寄存器地址
	//data:数据
	//返回值:0,正常
	//    其他,错误代码
	u8 writeByte(u8 addr, u8 reg, u8 data);

	//IIC读一个字节 
	//reg:寄存器地址 
	//返回值:读到的数据
	u8 readByte(u8 addr, u8 reg);

	//IIC连续读
	//addr:器件地址
	//reg:要读取的寄存器地址
	//len:要读取的长度
	//buf:读取到的数据存储区
	//返回值:0,正常
	//    其他,错误代码
	u8 readLen(u8 addr, u8 reg, u8 len, u8 *buf);

	//IIC连续写
	//addr:器件地址 
	//reg:寄存器地址
	//len:写入长度
	//buf:数据区
	//返回值:0,正常
	//    其他,错误代码
	u8 writeLen(u8 addr, u8 reg, u8 len, u8 *buf);

public:
	MPU9250(I2c* i2c,MPU9250_Model_Typedef model=MPU9250_Model_9250);

	u8 begin(uint32_t speed = 400000, uint16_t sampleRate = 100,
		MPU9250_Gyro_Full_Scale_Typedef gyroFsr=MPU9250_Gyro_Full_Scale_2000dps, 
		MPU9250_Accel_Full_Scale_Typedef accelFsr = MPU9250_Accel_Full_Scale_8g);

	//设置MPU9250陀螺仪传感器满量程范围
	//fsr:0,±250dps;1,±500dps;2,±1000dps;3,±2000dps
	//返回值:0,设置成功
	//    其他,设置失败 
	u8 setGyroFsr(u8 fsr);

	//设置MPU9250加速度传感器满量程范围
	//fsr:0,±2g;1,±4g;2,±8g;3,±16g
	//返回值:0,设置成功
	//    其他,设置失败 
	u8 setAccelFsr(u8 fsr);

	//设置MPU9250的数字低通滤波器
	//lpf:数字低通滤波频率(Hz)
	//返回值:0,设置成功
	//    其他,设置失败 
	u8 setLPF(u16 lpf);

	//设置MPU9250的采样率(假定Fs=1KHz)
	//rate:4~1000(Hz)
	//返回值:0,设置成功
	//    其他,设置失败 
	u8 setSampleRate(u16 sampleRate);

	//得到温度值
	//返回值:温度值(扩大了100倍)
	short getTemperature(void);

	//得到陀螺仪值(原始值)
	//gx,gy,gz:陀螺仪x,y,z轴的原始读数(带符号)
	//返回值:0,成功
	//    其他,错误代码
	u8 getGyroscope(short *gx, short *gy, short *gz);
	u8 getGyroscope(float *gx, float *gy, float *gz)
	{
		short x, y, z;
		getGyroscope(&x, &y, &z);
		//16.4 = 2^16/4000 lsb °/s     1/16.4=0.061     0.0174 = 3.14/180
		//陀螺仪数据从ADC转化为弧度每秒(这里需要减去偏移值)
		*gx = 2 * (float)x*gyroFsr / 65536 * 0.0174;
		*gy = 2 * (float)y*gyroFsr / 65536 * 0.0174;
		*gz = 2 * (float)z*gyroFsr / 65536 * 0.0174;	//读出值减去基准值乘以单位，计算陀螺仪角速度
	}

	//得到加速度值(原始值)
	//gx,gy,gz:陀螺仪x,y,z轴的原始读数(带符号)
	//返回值:0,成功
	//    其他,错误代码
	u8 getAccelerometer(short *ax, short *ay, short *az);
	u8 getAccelerometer(float *ax, float *ay, float *az)
	{
		short x, y, z;
		getAccelerometer(&x, &y, &z);
		//+-8g,2^16/16=4096lsb/g--0.244mg/lsb
		//此处0.0098是：(9.8m/s^2)/1000,乘以mg得m/s^2
		*ax = (float)x*(2 * accelFsr)  * 9.8 / 65536;
		*ay = (float)y*(2 * accelFsr)  * 9.8 / 65536;
		*az = (float)z*(2 * accelFsr)  * 9.8 / 65536;
	}

	//得到磁力计值(原始值)
	//mx,my,mz:磁力计x,y,z轴的原始读数(带符号)
	//返回值:0,成功
	//    其他,错误代码
	u8 getMagnetometer(short *mx, short *my, short *mz);
	u8 getMagnetometer(float *mx, float *my, float *mz)
	{
		short x, y, z;
		getMagnetometer(&x, &y, &z);
		//±4800uT 2^16/9600 = 6.83lsb/uT     1/6.83 = 0.1465
		//地磁强度为 5-6 x 10^(-5) T = 50 - 60 uT
		*mx = (float)x*0.1465;
		*my = (float)y*0.1465;
		*mz = (float)z*0.1465;
	}

	//获取采样率
	u16 getSampleRate();

};

class AHRS9
{
protected:
	//传感器原始数据
	typedef struct  sensor_data
	{
		short X;
		short Y;
		short Z;
	sensor_data():X(0), Y(0), Z(0){}
	}SENSOR_DATA;

	SENSOR_DATA Gyrobuf;//陀螺仪
	SENSOR_DATA Accbuf;//加速度
	SENSOR_DATA Magbuf;//磁力计

	SENSOR_DATA Accoffset;//加速度偏移量
	SENSOR_DATA Gyrooffset;//陀螺仪偏移量
	SENSOR_DATA Magoffset;//磁力计偏移量

	//处理后的数据
	typedef struct  imu_data
	{
		float X;
		float Y;
		float Z;
		imu_data() :X(0), Y(0), Z(0) {}
	}IMU_DATA;

	IMU_DATA GyroFinal, AccFinal, MagFinal;

	float q0, q1, q2, q3;	// quaternion of sensor frame relativ
	float beta;								// 2 * proportional gain (Kp)
	float sampleFreq;		// sample frequency in Hz  采样率 100 HZ  10ms  修改此频率可增加变化速度

	float Pitch;
	float Roll;
	float Yaw;

	//快速逆平方根
	float invSqrt(float x)
	{
		float halfx = 0.5f * x;
		float y = x;
		long i = *(long*)&y;
		i = 0x5f3759df - (i >> 1);
		y = *(float*)&i;
		y = y * (1.5f - (halfx * y * y));
		return y;
	}


public:
	AHRS9(float sampleFreq = 100, float beta = 0.1) :
		sampleFreq(sampleFreq),
		beta(beta),
		q0(1), q1(0), q2(0), q3(0)
	{

	}

	void AHRS_Dataprepare(short* gyroXYZ, short* accXYZ, short* magXYZ)
	{
		Gyrobuf.X = gyroXYZ[0];
		Gyrobuf.Y = gyroXYZ[1];
		Gyrobuf.Z = gyroXYZ[2];//读取陀螺仪数据-ADC数字量
		Accbuf.X = accXYZ[0];
		Accbuf.Y = accXYZ[1];
		Accbuf.Z = accXYZ[2];//读取加速度计数据-ADC数字量
		Magbuf.X = magXYZ[0];
		Magbuf.Y = magXYZ[1];
		Magbuf.Z = magXYZ[2];//读取磁力计数据-ADC数字量

		//16.4 = 2^16/4000 lsb °/s     1/16.4=0.061     0.0174 = 3.14/180
		//陀螺仪数据从ADC转化为弧度每秒(这里需要减去偏移值)
		GyroFinal.X = (Gyrobuf.X - Gyrooffset.X)*0.061*0.0174;
		GyroFinal.Y = (Gyrobuf.Y - Gyrooffset.Y)*0.061*0.0174;
		GyroFinal.Z = (Gyrobuf.Z - Gyrooffset.Z)*0.061*0.0174;		//读出值减去基准值乘以单位，计算陀螺仪角速度

		//+-8g,2^16/16=4096lsb/g--0.244mg/lsb
		//此处0.0098是：(9.8m/s^2)/1000,乘以mg得m/s^2
		AccFinal.X = (float)((Accbuf.X - Accoffset.X)*0.244)*0.0098;
		AccFinal.Y = (float)((Accbuf.Y - Accoffset.Y)*0.244)*0.0098;
		AccFinal.Z = (float)((Accbuf.Z - Accoffset.Z)*0.244)*0.0098;


		//±4800uT 2^16/9600 = 6.83lsb/uT     1/6.83 = 0.1465
		//地磁强度为 5-6 x 10^(-5) T = 50 - 60 uT
		MagFinal.X = (float)(Magbuf.X - Magoffset.X)*0.1465;
		MagFinal.Y = (float)(Magbuf.Y - Magoffset.Y)*0.1465;
		MagFinal.Z = (float)(Magbuf.Z - Magoffset.Z)*0.1465;

	}

	//9轴姿态解算
	void AHRSupdate()
	{
		AHRSupdate(GyroFinal.X, GyroFinal.Y, GyroFinal.Z,
			AccFinal.X, AccFinal.Y, AccFinal.Z,
			MagFinal.X, MagFinal.Y, MagFinal.Z);
	}

	void AHRSupdate(float gx, float gy, float gz, float ax, float ay, float az, float mx, float my, float mz)
	{
		float recipNorm;
		float s0, s1, s2, s3;
		float qDot1, qDot2, qDot3, qDot4;
		float hx, hy;
		float _2q0mx, _2q0my, _2q0mz, _2q1mx, _2bx, _2bz, _4bx, _4bz, _2q0, _2q1, _2q2, _2q3, _2q0q2, _2q2q3, q0q0, q0q1, q0q2, q0q3, q1q1, q1q2, q1q3, q2q2, q2q3, q3q3;

		// Rate of change of quaternion from gyroscope
		//四元数变化率
		qDot1 = 0.5f * (-q1 * gx - q2 * gy - q3 * gz);
		qDot2 = 0.5f * (q0 * gx + q2 * gz - q3 * gy);
		qDot3 = 0.5f * (q0 * gy - q1 * gz + q3 * gx);
		qDot4 = 0.5f * (q0 * gz + q1 * gy - q2 * gx);

		// Compute feedback only if accelerometer measurement valid (avoids NaN in accelerometer normalisation)
		//
		if (!((ax == 0.0f) && (ay == 0.0f) && (az == 0.0f)))
		{

			// Normalise accelerometer measurement
			//正常化的加速度测量值
			recipNorm = invSqrt(ax * ax + ay * ay + az * az);
			ax *= recipNorm;
			ay *= recipNorm;
			az *= recipNorm;

			// Normalise magnetometer measurement
			//正常化的磁力计测量值
			recipNorm = invSqrt(mx * mx + my * my + mz * mz);
			mx *= recipNorm;
			my *= recipNorm;
			mz *= recipNorm;

			// Auxiliary variables to avoid repeated arithmetic
			//辅助变量，避免重复运算
			_2q0mx = 2.0f * q0 * mx;
			_2q0my = 2.0f * q0 * my;
			_2q0mz = 2.0f * q0 * mz;
			_2q1mx = 2.0f * q1 * mx;
			_2q0 = 2.0f * q0;
			_2q1 = 2.0f * q1;
			_2q2 = 2.0f * q2;
			_2q3 = 2.0f * q3;
			_2q0q2 = 2.0f * q0 * q2;
			_2q2q3 = 2.0f * q2 * q3;
			q0q0 = q0 * q0;
			q0q1 = q0 * q1;
			q0q2 = q0 * q2;
			q0q3 = q0 * q3;
			q1q1 = q1 * q1;
			q1q2 = q1 * q2;
			q1q3 = q1 * q3;
			q2q2 = q2 * q2;
			q2q3 = q2 * q3;
			q3q3 = q3 * q3;

			// Reference direction of Earth's magnetic field
			//地球磁场参考方向
			hx = mx * q0q0 - _2q0my * q3 + _2q0mz * q2 + mx * q1q1 + _2q1 * my * q2 + _2q1 * mz * q3 - mx * q2q2 - mx * q3q3;
			hy = _2q0mx * q3 + my * q0q0 - _2q0mz * q1 + _2q1mx * q2 - my * q1q1 + my * q2q2 + _2q2 * mz * q3 - my * q3q3;
			_2bx = sqrt(hx * hx + hy * hy);
			_2bz = -_2q0mx * q2 + _2q0my * q1 + mz * q0q0 + _2q1mx * q3 - mz * q1q1 + _2q2 * my * q3 - mz * q2q2 + mz * q3q3;
			_4bx = 2.0f * _2bx;
			_4bz = 2.0f * _2bz;

			// Gradient decent algorithm corrective step
			//梯度校正算法
			s0 = -_2q2 * (2.0f * q1q3 - _2q0q2 - ax) + _2q1 * (2.0f * q0q1 + _2q2q3 - ay) - _2bz * q2 * (_2bx * (0.5f - q2q2 - q3q3) + _2bz * (q1q3 - q0q2) - mx) + (-_2bx * q3 + _2bz * q1) * (_2bx * (q1q2 - q0q3) + _2bz * (q0q1 + q2q3) - my) + _2bx * q2 * (_2bx * (q0q2 + q1q3) + _2bz * (0.5f - q1q1 - q2q2) - mz);
			s1 = _2q3 * (2.0f * q1q3 - _2q0q2 - ax) + _2q0 * (2.0f * q0q1 + _2q2q3 - ay) - 4.0f * q1 * (1 - 2.0f * q1q1 - 2.0f * q2q2 - az) + _2bz * q3 * (_2bx * (0.5f - q2q2 - q3q3) + _2bz * (q1q3 - q0q2) - mx) + (_2bx * q2 + _2bz * q0) * (_2bx * (q1q2 - q0q3) + _2bz * (q0q1 + q2q3) - my) + (_2bx * q3 - _4bz * q1) * (_2bx * (q0q2 + q1q3) + _2bz * (0.5f - q1q1 - q2q2) - mz);
			s2 = -_2q0 * (2.0f * q1q3 - _2q0q2 - ax) + _2q3 * (2.0f * q0q1 + _2q2q3 - ay) - 4.0f * q2 * (1 - 2.0f * q1q1 - 2.0f * q2q2 - az) + (-_4bx * q2 - _2bz * q0) * (_2bx * (0.5f - q2q2 - q3q3) + _2bz * (q1q3 - q0q2) - mx) + (_2bx * q1 + _2bz * q3) * (_2bx * (q1q2 - q0q3) + _2bz * (q0q1 + q2q3) - my) + (_2bx * q0 - _4bz * q2) * (_2bx * (q0q2 + q1q3) + _2bz * (0.5f - q1q1 - q2q2) - mz);
			s3 = _2q1 * (2.0f * q1q3 - _2q0q2 - ax) + _2q2 * (2.0f * q0q1 + _2q2q3 - ay) + (-_4bx * q3 + _2bz * q1) * (_2bx * (0.5f - q2q2 - q3q3) + _2bz * (q1q3 - q0q2) - mx) + (-_2bx * q0 + _2bz * q2) * (_2bx * (q1q2 - q0q3) + _2bz * (q0q1 + q2q3) - my) + _2bx * q1 * (_2bx * (q0q2 + q1q3) + _2bz * (0.5f - q1q1 - q2q2) - mz);
			recipNorm = invSqrt(s0 * s0 + s1 * s1 + s2 * s2 + s3 * s3); // normalise step magnitude
			s0 *= recipNorm;
			s1 *= recipNorm;
			s2 *= recipNorm;
			s3 *= recipNorm;

			// Apply feedback step
			//应用反馈步骤
			qDot1 -= beta * s0;
			qDot2 -= beta * s1;
			qDot3 -= beta * s2;
			qDot4 -= beta * s3;
		}

		// Integrate rate of change of quaternion to yield quaternion
		//四元数变化率的集成率
		q0 += qDot1 * (1.0f / sampleFreq);
		q1 += qDot2 * (1.0f / sampleFreq);
		q2 += qDot3 * (1.0f / sampleFreq);
		q3 += qDot4 * (1.0f / sampleFreq);

		// 正常化四元数
		recipNorm = invSqrt(q0 * q0 + q1 * q1 + q2 * q2 + q3 * q3);
		q0 *= recipNorm;
		q1 *= recipNorm;
		q2 *= recipNorm;
		q3 *= recipNorm;


		//四元数转换成欧拉角
		Pitch = asin(2 * q0*q2 - 2 * q1*q3) / 3.14 * 180;
		Roll = atan2(2 * q0q1 + 2 * q2q3, 1 - 2 * q1q1 - 2 * q2q2) / 3.14 * 180;
		Yaw = atan2(2 * q0q3 + 2 * q1*q2, 1 - 2 * q2*q2 - 2 * q3*q3) / 3.14 * 180;

	}

	void getAngle(float* Pitch, float* Roll, float* Yaw)
	{
		*Pitch = this->Pitch;
		*Roll = this->Roll;
		*Yaw = this->Yaw;
	}

	void setSampleRate(float sampleRate)
	{
		this->sampleFreq = sampleRate;
	}
};

class MPU9250AHRS :public MPU9250, private AHRS9
{
protected:

public:
	MPU9250AHRS(I2c* i2c, MPU9250_Model_Typedef model = MPU9250_Model_9250) :
		MPU9250(i2c, model)
	{

	}

	void begin(uint32_t speed = 400000, uint16_t sampleRate = 100,
		MPU9250_Gyro_Full_Scale_Typedef gyroFsr = MPU9250_Gyro_Full_Scale_2000dps,
		MPU9250_Accel_Full_Scale_Typedef accelFsr = MPU9250_Accel_Full_Scale_8g)
	{
		MPU9250::begin(speed, sampleRate, gyroFsr, accelFsr);
		AHRS9::setSampleRate(sampleRate);
	}

	void getAngle(float* pitch, float* roll, float* yaw)
	{
		short g[3], a[3], m[3];
		getGyroscope(g, g+1, g+2);
		getAccelerometer(a, a + 1, a + 2);
		getMagnetometer(m, m + 1, m + 2);

		AHRS9::AHRS_Dataprepare(g, a, m);
		AHRS9::AHRSupdate();

		AHRS9::getAngle(pitch, roll, yaw);

	}
};



#endif
