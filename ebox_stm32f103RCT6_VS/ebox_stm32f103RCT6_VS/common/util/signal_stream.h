#ifndef __SIGNAL_STREAM
#define __SIGNAL_STREAM
#define  pi 3.14159265
#include <math.h>

//��circle bufferʵ�ֵ������źŴ�����
//�����źŴ洢��ͨ���̳л�ʵ����ʵ���źŴ�����
//ע��ģ����ĺ���ʵ�ֱ���д��ͷ�ļ���
template<typename T, int Size>
class SignalStream
{
	int indexHead;
public:
	T buf[Size];

	//��ʼ��bufΪ0
	SignalStream();

	//���bufΪ0
	void clear();

	//����buf�ĳ���
	int size();

	//ѹ�����µ����ݲ�������ɵ�����
	void push(T signal);

	//����[]����������0��ʼΪ���µ��ɵ�����
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

//��SignalStreamʵ�ֵľ�ֵ�˲���
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

//��SignalStreamʵ�ֵľ�ֵ�˲�����ǿ�棬����������
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

class RcFilter

{
public:
	RcFilter(float sampleFrq, float stopFrq);
	float getFilterOut(float x);
	~RcFilter();
private:
	float stopFrq;
	float sampleFrq;
	float tempY;
	float tempX;
};

RcFilter::RcFilter(float sampleFrq, float stopFrq)
{
	this->sampleFrq = sampleFrq;
//	this->stopFrq = tan(3.14159265*stopFrq/sampleFrq);
//	this->stopFrq = (2 * stopFrq / sampleFrq)/2;    //���Է���
	this->stopFrq = 1 /(1+ (2 * pi*stopFrq /(sampleFrq)));    //��������
	this->tempY = 0;
	this->tempX = 0;

}

RcFilter::~RcFilter()
{
}

float RcFilter::getFilterOut(float x)
{

	//float y = tempY*(1-stopFrq)+stopFrq*x;  //���Է���
	float y = tempY*stopFrq + (1 - stopFrq)*x;  //��������
	tempY = y;
	tempX = x;
	return y;
}
#endif
