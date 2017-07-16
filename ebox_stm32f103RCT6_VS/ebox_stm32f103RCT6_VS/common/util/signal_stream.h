#ifndef __SIGNAL_STREAM
#define __SIGNAL_STREAM
#define  pi 3.14159265
#include <math.h>

//用circle buffer实现的试试信号处理类
//仅做信号存储，通过继承或实例化实现信号处理函数
//注意模板类的函数实现必须写在头文件中
template<typename T, int Size>
class SignalStream
{
	int indexHead;
public:
	T buf[Size];

	//初始化buf为0
	SignalStream();

	//清空buf为0
	void clear();

	//返回buf的长度
	int size();

	//压入最新的数据并弹出最旧的数据
	void push(T signal);

	//重载[]操作符，从0开始为从新到旧的数据
	T &operator [](int index);
};

template<typename T, int Size>
SignalStream<T, Size>::SignalStream() :
	indexHead(0)
{
	clear();
}

template<typename T, int Size>
void SignalStream<T, Size>::clear()
{
	for (int i = 0; i < Size; i++)
	{
		buf[i] = 0;
	}
}

template<typename T, int Size>
int SignalStream<T, Size>::size()
{
	return Size;
}

template<typename T, int Size>
void SignalStream<T, Size>::push(T signal)
{
	indexHead--;
	indexHead %= Size;
	if (indexHead < 0)
	{
		indexHead += Size;
	}
	buf[indexHead] = signal;
}

template<typename T, int Size>
T & SignalStream<T, Size>::operator[](int index)
{
	return buf[(index + indexHead) % Size];
}

//用SignalStream实现的均值滤波类
template<int filterWindow>
class AverageFilter :public SignalStream<float, filterWindow>
{
public:
	float getFilterOut(float newNum)
	{

		SignalStream<float, filterWindow>::push(newNum);
		float temp = 0;
		for (int i = 0; i < filterWindow; i++)
		{
			temp += SignalStream<float, filterWindow>::operator[](i);
		}
		temp /= filterWindow;
		return temp;
	}
};

//用SignalStream实现的均值滤波类增强版，减少运算量
template<int sampleFrq, int stopFrq>
class AverageFilterEnhance :public SignalStream<float, int(0.443*sampleFrq / stopFrq)>
{
public:
	float getFilterOut(float newNum)
	{
		float old = SignalStream<float, int(0.443*sampleFrq / stopFrq)>::operator [](int(0.443*sampleFrq / stopFrq) - 1);
		SignalStream<float, int(0.443*sampleFrq / stopFrq)>::push(newNum);
		float temp = 0;
		this->sumTemp = this->sumTemp + newNum - old;
		temp = this->sumTemp / int(0.443*sampleFrq / stopFrq);
		return temp;
	}
private:
	float sumTemp;

};

class Butterworth

{
public:
	Butterworth(float sampleFrq,float stopFrq);
	float getFilterOut(float x);
	~Butterworth();
private:
	float stopFrq;
	float sampleFrq;
	float tempY;
	float tempX;
};

Butterworth::Butterworth(float sampleFrq, float stopFrq)
{
	this->sampleFrq = sampleFrq;
//	this->stopFrq = tan(3.14159265*stopFrq/sampleFrq);
	this->stopFrq = (2 * stopFrq / sampleFrq);
	this->tempY = 0;
	this->tempX = 0;

}

Butterworth::~Butterworth()
{
}

float Butterworth::getFilterOut(float x)
{

	//float y =- 1 / stopFrq*tempY + (1+1 / stopFrq)*x;
//	float y = (stopFrq - 1) / (stopFrq + 1)*tempY + stopFrq / (1+stopFrq)*(x + tempX);
	float y = tempY*(1-stopFrq)+stopFrq*x;
	tempY = y;
	tempX = x;
	return y;
}
#endif
