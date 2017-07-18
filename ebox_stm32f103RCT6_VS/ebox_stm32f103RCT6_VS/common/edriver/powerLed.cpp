#include "powerLed.h"
PowerLed::PowerLed(Gpio *pwm):ledPwm(pwm)
{
}
void PowerLed::begin(int16_t duty)
{
	ledPwm.begin(1000, duty);
	ledPwm.set_oc_polarity(1);
}
void PowerLed::setPct(int16_t duty)
{
	ledPwm.set_duty(duty);
}