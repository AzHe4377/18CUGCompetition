#ifndef __steer_H
#define __steer_H
#ifdef __cplusplus
 extern "C" {
#endif
	 
#include "stm32f4xx_hal.h"
#include "main.h"

void SystemClock_Config(void);
void _Error_Handler(char *file, int line);
	 
void PWM_SetDuty(TIM_HandleTypeDef *tim,uint32_t tim_channel,float duty);
	 

#ifdef __cplusplus
}
#endif
#endif /*__ pinoutConfig_H */

