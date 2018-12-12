#include "main.h"
#include "stm32f4xx_hal.h"
#include "tim.h"
#include "gpio.h"
#include "mecanum.h"
#include "pwm.h"
#include "steer.h"

int main(void)
{  
	while(1)
	{
		steer1_180();
	}
}

