#ifndef __MPU9250_H
#define __MPU9250_H

#include "ebox.h"
#include <math.h>

////////////////////////////////////////////////////////////////////////////////// 	
//如果AD0脚(9脚)接地,IIC地址为0X68(不包含最低位).
//如果接V3.3,则IIC地址为0X69(不包含最低位).
#define MPU6500_ADDRESS            0xD0  //MPU6500的器件IIC地址 0X68<<1
#define MPU6500_ID				0X71  	//MPU6500的器件ID
#define MPU6555_ID				0X73

//MPU9250内部封装了一个AK8963磁力计,地址和ID如下:
#define AK8963_ADDRESS				0x18	//AK8963的I2C地址 0X0C<<1
#define AK8963_ID				0X48	//AK8963的器件ID


//AK8963的内部寄存器
#define AK8963_WHO_AM_I		0x00	//AK8963的器件ID寄存器地址 should return 0x48
#define AK8963_INFO      0x01
#define AK8963_ST1       0x02  // data ready status bit 0
#define AK8963_XOUT_L    0x03  // data
#define AK8963_XOUT_H    0x04
#define AK8963_YOUT_L    0x05
#define AK8963_YOUT_H    0x06
#define AK8963_ZOUT_L    0x07
#define AK8963_ZOUT_H    0x08
#define AK8963_ST2       0x09  // Data overflow bit 3 and data read error status bit 2
#define AK8963_CNTL          	  	0X0A    // Power down (0000), single-measurement (0001), self-test (1000) and Fuse ROM (1111) modes on bits 3:0
#define AK8963_ASTC      0x0C  // Self test control
#define AK8963_I2CDIS    0x0F  // I2C disable
#define AK8963_ASAX      0x10  // Fuse ROM x-axis sensitivity adjustment value
#define AK8963_ASAY      0x11  // Fuse ROM y-axis sensitivity adjustment value
#define AK8963_ASAZ      0x12  // Fuse ROM z-axis sensitivity adjustment value

//MPU6500的内部寄存器
#define MPU6500_SELF_TESTX_REG		0X0D	//自检寄存器X
#define MPU6500_SELF_TESTY_REG		0X0E	//自检寄存器Y
#define MPU6500_SELF_TESTZ_REG		0X0F	//自检寄存器Z
#define MPU6500_SELF_TESTA_REG		0X10	//自检寄存器A
#define MPU6500_SAMPLE_RATE_REG		0X19	//采样频率分频器
#define MPU6500_CFG_REG				0X1A	//配置寄存器
#define MPU6500_GYRO_CFG_REG		0X1B	//陀螺仪配置寄存器
#define MPU6500_ACCEL_CFG_REG		0X1C	//加速度计配置寄存器
#define MPU6500_MOTION_DET_REG		0X1F	//运动检测阀值设置寄存器
#define MPU6500_FIFO_EN_REG			0X23	//FIFO使能寄存器
#define MPU6500_I2CMST_CTRL_REG		0X24	//IIC主机控制寄存器
#define MPU6500_I2CSLV0_ADDR_REG	0X25	//IIC从机0器件地址寄存器
#define MPU6500_I2CSLV0_REG			0X26	//IIC从机0数据地址寄存器
#define MPU6500_I2CSLV0_CTRL_REG	0X27	//IIC从机0控制寄存器
#define MPU6500_I2CSLV1_ADDR_REG	0X28	//IIC从机1器件地址寄存器
#define MPU6500_I2CSLV1_REG			0X29	//IIC从机1数据地址寄存器
#define MPU6500_I2CSLV1_CTRL_REG	0X2A	//IIC从机1控制寄存器
#define MPU6500_I2CSLV2_ADDR_REG	0X2B	//IIC从机2器件地址寄存器
#define MPU6500_I2CSLV2_REG			0X2C	//IIC从机2数据地址寄存器
#define MPU6500_I2CSLV2_CTRL_REG	0X2D	//IIC从机2控制寄存器
#define MPU6500_I2CSLV3_ADDR_REG	0X2E	//IIC从机3器件地址寄存器
#define MPU6500_I2CSLV3_REG			0X2F	//IIC从机3数据地址寄存器
#define MPU6500_I2CSLV3_CTRL_REG	0X30	//IIC从机3控制寄存器
#define MPU6500_I2CSLV4_ADDR_REG	0X31	//IIC从机4器件地址寄存器
#define MPU6500_I2CSLV4_REG			0X32	//IIC从机4数据地址寄存器
#define MPU6500_I2CSLV4_DO_REG		0X33	//IIC从机4写数据寄存器
#define MPU6500_I2CSLV4_CTRL_REG	0X34	//IIC从机4控制寄存器
#define MPU6500_I2CSLV4_DI_REG		0X35	//IIC从机4读数据寄存器

#define MPU6500_I2CMST_STA_REG		0X36	//IIC主机状态寄存器
#define MPU6500_INTBP_CFG_REG		0X37	//中断/旁路设置寄存器
#define MPU6500_INT_EN_REG			0X38	//中断使能寄存器
#define MPU6500_INT_STA_REG			0X3A	//中断状态寄存器

#define MPU6500_ACCEL_XOUTH_REG		0X3B	//加速度值,X轴高8位寄存器
#define MPU6500_ACCEL_XOUTL_REG		0X3C	//加速度值,X轴低8位寄存器
#define MPU6500_ACCEL_YOUTH_REG		0X3D	//加速度值,Y轴高8位寄存器
#define MPU6500_ACCEL_YOUTL_REG		0X3E	//加速度值,Y轴低8位寄存器
#define MPU6500_ACCEL_ZOUTH_REG		0X3F	//加速度值,Z轴高8位寄存器
#define MPU6500_ACCEL_ZOUTL_REG		0X40	//加速度值,Z轴低8位寄存器

#define MPU6500_TEMP_OUTH_REG		0X41	//温度值高八位寄存器
#define MPU6500_TEMP_OUTL_REG		0X42	//温度值低8位寄存器

#define MPU6500_GYRO_XOUTH_REG		0X43	//陀螺仪值,X轴高8位寄存器
#define MPU6500_GYRO_XOUTL_REG		0X44	//陀螺仪值,X轴低8位寄存器
#define MPU6500_GYRO_YOUTH_REG		0X45	//陀螺仪值,Y轴高8位寄存器
#define MPU6500_GYRO_YOUTL_REG		0X46	//陀螺仪值,Y轴低8位寄存器
#define MPU6500_GYRO_ZOUTH_REG		0X47	//陀螺仪值,Z轴高8位寄存器
#define MPU6500_GYRO_ZOUTL_REG		0X48	//陀螺仪值,Z轴低8位寄存器

#define MPU6500_I2CSLV0_DO_REG		0X63	//IIC从机0数据寄存器
#define MPU6500_I2CSLV1_DO_REG		0X64	//IIC从机1数据寄存器
#define MPU6500_I2CSLV2_DO_REG		0X65	//IIC从机2数据寄存器
#define MPU6500_I2CSLV3_DO_REG		0X66	//IIC从机3数据寄存器

#define MPU6500_I2CMST_DELAY_REG	0X67	//IIC主机延时管理寄存器
#define MPU6500_SIGPATH_RST_REG		0X68	//信号通道复位寄存器
#define MPU6500_MDETECT_CTRL_REG	0X69	//运动检测控制寄存器
#define MPU6500_USER_CTRL_REG		0X6A	//用户控制寄存器
#define MPU6500_PWR_MGMT1_REG		0X6B	//电源管理寄存器1
#define MPU6500_PWR_MGMT2_REG		0X6C	//电源管理寄存器2 
#define MPU6500_FIFO_CNTH_REG		0X72	//FIFO计数寄存器高八位
#define MPU6500_FIFO_CNTL_REG		0X73	//FIFO计数寄存器低八位
#define MPU6500_FIFO_RW_REG			0X74	//FIFO读写寄存器
#define MPU6500_DEVICE_ID_REG		0X75	//器件ID寄存器

typedef enum
{
	MPU6500_Model_6500,
	MPU6500_Model_6555
}MPU6500_Model_Typedef;

typedef enum
{
	MPU6500_Gyro_Full_Scale_250dps=0,
	MPU6500_Gyro_Full_Scale_500dps =1,
	MPU6500_Gyro_Full_Scale_1000dps = 2,
	MPU6500_Gyro_Full_Scale_2000dps = 3
}MPU6500_Gyro_Full_Scale_Typedef;

typedef enum
{
	MPU6500_Accel_Full_Scale_2g = 0,
	MPU6500_Accel_Full_Scale_4g = 1,
	MPU6500_Accel_Full_Scale_8g = 2,
	MPU6500_Accel_Full_Scale_16g = 3
}MPU6500_Accel_Full_Scale_Typedef;

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
	static const uint16_t gyroFsrList[];
	static const uint8_t accelFsrList[];

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
	u8 readBytes(u8 addr, u8 reg, u8 len, u8 *buf);

	//IIC连续写
	//addr:器件地址 
	//reg:寄存器地址
	//len:写入长度
	//buf:数据区
	//返回值:0,正常
	//    其他,错误代码
	u8 writeBytes(u8 addr, u8 reg, u8 len, u8 *buf);

public:
	MPU9250(I2c* i2c,MPU6500_Model_Typedef model=MPU6500_Model_6500);

	u8 begin(uint32_t speed = 400000, uint16_t sampleRate = 100,
		MPU6500_Gyro_Full_Scale_Typedef gyroFsr=MPU6500_Gyro_Full_Scale_2000dps, 
		MPU6500_Accel_Full_Scale_Typedef accelFsr = MPU6500_Accel_Full_Scale_8g);

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
	float getTemperature(void);

	//得到陀螺仪值(原始值)
	//gx,gy,gz:陀螺仪x,y,z轴的原始读数(带符号)
	//返回值:0,成功
	//    其他,错误代码
	u8 getGyroscope(short *gx, short *gy, short *gz);
	u8 getGyroscope(float *gx, float *gy, float *gz);

	//得到加速度值(原始值)
	//gx,gy,gz:陀螺仪x,y,z轴的原始读数(带符号)
	//返回值:0,成功
	//    其他,错误代码
	u8 getAccelerometer(short *ax, short *ay, short *az);
	u8 getAccelerometer(float *ax, float *ay, float *az);

	//得到磁力计值(原始值)
	//mx,my,mz:磁力计x,y,z轴的原始读数(带符号)
	//返回值:0,成功
	//    其他,错误代码
	u8 getMagnetometer(short *mx, short *my, short *mz);
	u8 getMagnetometer(float *mx, float *my, float *mz);

	//获取采样率
	u16 getSampleRate();

};



class MahonyAHRS9
{
protected:
	float q0, q1, q2, q3;	// quaternion of sensor frame relativ
	float exInt, eyInt, ezInt;
	float Ki, Kp;
	float halfT;
	float sampleFreq;

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
	MahonyAHRS9(float sampleFreq = 100, float kp = 10, float ki = 0) :
		Kp(kp),Ki(ki),
		q0(1), q1(0), q2(0), q3(0),
		exInt(0), eyInt(0), ezInt(0)
	{
		setSampleRate(sampleFreq);
	}

	virtual void update(float gx, float gy, float gz, float ax, float ay, float az, float mx, float my, float mz)
	{
		float norm;
		float q0q0, q0q1, q0q2, q0q3, q1q1, q1q2, q1q3, q2q2, q2q3, q3q3;
		float hx, hy, hz, bx, bz;
		float vx, vy, vz, wx, wy, wz;
		float ex, ey, ez;
		float qa, qb, qc;
		if (!((ax == 0.0f) && (ay == 0.0f) && (az == 0.0f)))
		{

			// Normalise accelerometer measurement
			//正常化的加速度测量值
			norm = invSqrt(ax * ax + ay * ay + az * az);
			ax *= norm;
			ay *= norm;
			az *= norm;

			// Normalise magnetometer measurement
			//正常化的磁力计测量值
			norm = invSqrt(mx * mx + my * my + mz * mz);
			mx *= norm;
			my *= norm;
			mz *= norm;

			//预先进行四元数数据运算，以避免重复运算带来的效率问题。
			// Auxiliary variables to avoid repeated arithmetic
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

			hx = 2.0f * (mx * (0.5f - q2q2 - q3q3) + my * (q1q2 - q0q3) + mz * (q1q3 + q0q2));
			hy = 2.0f * (mx * (q1q2 + q0q3) + my * (0.5f - q1q1 - q3q3) + mz * (q2q3 - q0q1));
			hz = 2.0f * (mx * (q1q3 - q0q2) + my * (q2q3 + q0q1) + mz * (0.5f - q1q1 - q2q2));
			bx = sqrt(hx * hx + hy * hy);
			bz = hz;

			vx = q1q3 - q0q2;
			vy = q0q1 + q2q3;
			vz = q0q0 - 0.5f + q3q3;
			wx = bx * (0.5f - q2q2 - q3q3) + bz * (q1q3 - q0q2);
			wy = bx * (q1q2 - q0q3) + bz * (q0q1 + q2q3);
			wz = bx * (q0q2 + q1q3) + bz * (0.5f - q1q1 - q2q2);

			//使用叉积来计算重力和地磁误差。
			// Error is sum of cross product between estimated direction and measured direction of field vectors
			ex = (ay * vz - az * vy) + (my * wz - mz * wy);
			ey = (az * vx - ax * vz) + (mz * wx - mx * wz);
			ez = (ax * vy - ay * vx) + (mx * wy - my * wx);

			//对误差进行积分
			exInt += Ki * ex * (1.0f / sampleFreq); // integral error scaled by Ki
			eyInt += Ki * ey * (1.0f / sampleFreq);
			ezInt += Ki * ez * (1.0f / sampleFreq);

			//将真实的加速度测量值以一定比例作用于陀螺仪，0就是完全信任陀螺仪，1就是完全信任加速度，大于1？
			gx = gx + Kp*ex + exInt;
			gy = gy + Kp*ey + eyInt;
			gz = gz + Kp*ez + ezInt;

			qa = q0;
			qb = q1;
			qc = q2;
			q0 += (-qb * gx - qc * gy - q3 * gz)*halfT;
			q1 += (qa * gx + qc * gz - q3 * gy)*halfT;
			q2 += (qa * gy - qb * gz + q3 * gx)*halfT;
			q3 += (qa * gz + qb * gy - qc * gx)*halfT;

			// Normalise quaternion

			norm = invSqrt(q0 * q0 + q1 * q1 + q2 * q2 + q3 * q3);
			q0 *= norm;
			q1 *= norm;
			q2 *= norm;
			q3 *= norm;
			
		}

	}

	void setSampleRate(float sampleRate)
	{
		this->sampleFreq = sampleRate;
		halfT = 1 / (2 * sampleFreq);
	}

	virtual void getAngle(float* Pitch, float* Roll, float* Yaw)
	{
		//四元数转换成欧拉角
		*Pitch = asin(2 * q0*q2 - 2 * q1*q3) / 3.14 * 180;
		*Roll = atan2(2 * q0*q1 + 2 * q2*q3, 1 - 2 * q1*q1 - 2 * q2*q2) / 3.14 * 180;
		*Yaw = atan2(2 * q0*q3 + 2 * q1*q2, 1 - 2 * q2*q2 - 2 * q3*q3) / 3.14 * 180;
	}

	virtual void getQuaternion(float* q0, float* q1, float* q2, float* q3)
	{
		*q0 = this->q0;
		*q1 = this->q1;
		*q2 = this->q2;
		*q3 = this->q3;
	}
};

class MPU9250AHRS :public MPU9250, private MahonyAHRS9
{
protected:
	void update()
	{
		float g[3], a[3], m[3];
		getGyroscope(g, g + 1, g + 2);
		getAccelerometer(a, a + 1, a + 2);
		getMagnetometer(m, m + 1, m + 2);
		MahonyAHRS9::update(
			g[0], g[1], g[2],
			a[0], a[1], a[2],
			m[0], m[1], m[2]);
	}
public:
	MPU9250AHRS(I2c* i2c, MPU6500_Model_Typedef model = MPU6500_Model_6500) :
		MPU9250(i2c, model),
		MahonyAHRS9(100, 10, 0.05)
	{

	}

	void begin(uint32_t speed = 400000, uint16_t sampleRate = 100,
		MPU6500_Gyro_Full_Scale_Typedef gyroFsr = MPU6500_Gyro_Full_Scale_2000dps,
		MPU6500_Accel_Full_Scale_Typedef accelFsr = MPU6500_Accel_Full_Scale_8g)
	{
		MPU9250::begin(speed, sampleRate, gyroFsr, accelFsr);
		MahonyAHRS9::setSampleRate(sampleRate);
	}



	virtual void getAngle(float* pitch, float* roll, float* yaw)
	{
		update();
		MahonyAHRS9::getAngle(pitch, roll, yaw);
	}

	virtual void getQuaternion(float* q0, float* q1, float* q2, float* q3)
	{
		update();
		MahonyAHRS9::getQuaternion(q0, q1, q2, q3);
	}
};



#endif
