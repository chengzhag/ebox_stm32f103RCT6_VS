#ifndef __ACCURATE_PWM_H
#define __ACCURATE_PWM_H

#include "ebox.h"
#include "my_math.h"

class AcurratePwm :public Pwm
{
	float duty;
public:
	AcurratePwm(Gpio *pwm_pin);
	void begin(uint32_t frq, uint16_t duty);
	void set_duty(float duty);
};

#endif
