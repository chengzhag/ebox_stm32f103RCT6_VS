#ifndef POWERLED
#define POWERLED
//#include "ebox.h"
#include "accurate_pwm.h"
class PowerLed
{
public:
	//构造函数
	PowerLed(Gpio *pwm);
	//初始化，设置初始占空比
	void begin(int16_t duty);
	//修改占空比
	void setPct(int16_t duty);
private:
	AccuratePwm ledPwm;

};
#endif // !POWERLED

