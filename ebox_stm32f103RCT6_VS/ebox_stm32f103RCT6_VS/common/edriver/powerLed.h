#ifndef POWERLED
#define POWERLED
//#include "ebox.h"
#include "accurate_pwm.h"
class PowerLed
{
public:
	//���캯��
	PowerLed(Gpio *pwm);
	//��ʼ�������ó�ʼռ�ձ�
	void begin(int16_t duty);
	//�޸�ռ�ձ�
	void setPct(int16_t duty);
private:
	AccuratePwm ledPwm;

};
#endif // !POWERLED

