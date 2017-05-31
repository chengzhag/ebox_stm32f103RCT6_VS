#include "accurate_pwm.h"
#include "stm32f10x_tim.h"

AcurratePwm::AcurratePwm(Gpio *pwm_pin) :
	duty(0),
	Pwm(pwm_pin)
{

}

void AcurratePwm::begin(uint32_t frq, uint16_t duty)
{
	Pwm::begin(frq, duty);
}

void AcurratePwm::set_duty(float duty)
{
	limit<float>(duty, 0, 1000);
	this->duty = duty;
	uint16_t compare = period*(duty / 1000);
	switch (ch)
	{
	case 1:
		TIM_SetCompare1(TIMx, compare);
		break;
	case 2:
		TIM_SetCompare2(TIMx, compare);
		break;
	case 3:
		TIM_SetCompare3(TIMx, compare);
		break;
	case 4:
		TIM_SetCompare4(TIMx, compare);
		break;
	default:
		break;
	}
}
