#include "main.h"
#include "stm32f4xx_hal.h"
#include "tim.h"
#include "gpio.h"
#include "pwm.h"
#include "steer.h"

	//PI2     ------> TIM8_CH4 	Z接舵机1（机械爪抓取舵机）
	//PI7     ------> TIM8_CH3	Y接舵机2（机械爪抓取舵机）
	//PI6     ------> TIM8_CH2	x接舵机3（机械臂水平转动舵机）

void steer1_0(void)
{
	HAL_Init();

	SystemClock_Config();

  MX_GPIO_Init();
  MX_TIM8_Init();

	led_off();
	

	HAL_TIM_PWM_Start(&htim8,TIM_CHANNEL_4);//Z--PI2
	PWM_SetDuty(&htim8,TIM_CHANNEL_4,0.025);//Z--PI2 PWM输出

}

void steer1_90(void)//舵机1-90度
{
	HAL_Init();

	SystemClock_Config();

  MX_GPIO_Init();
  MX_TIM8_Init();

	led_off();
	
	HAL_TIM_PWM_Start(&htim8,TIM_CHANNEL_4);//Z--PI2
	PWM_SetDuty(&htim8,TIM_CHANNEL_4,0.075);//Z--PI2 PWM输出

}

void steer1_180(void)//舵机1-180度
{
	HAL_Init();

	SystemClock_Config();

  MX_GPIO_Init();
  MX_TIM8_Init();

	led_off();
	
	HAL_TIM_PWM_Start(&htim8,TIM_CHANNEL_4);//Z--PI2
	PWM_SetDuty(&htim8,TIM_CHANNEL_4,0.125);//Z--PI2 PWM输出

}

void steer2_0(void)//舵机2-0度
{
	HAL_Init();

	SystemClock_Config();

  MX_GPIO_Init();
  MX_TIM8_Init();

	led_off();
	
	HAL_TIM_PWM_Start(&htim8,TIM_CHANNEL_3);//Y--PI7
	PWM_SetDuty(&htim8,TIM_CHANNEL_3,0.00);//Y--PI7 PWM输出

}

void steer2_90(void)//舵机2-90度
{
	HAL_Init();

	SystemClock_Config();

  MX_GPIO_Init();
  MX_TIM8_Init();

	led_off();
	
	HAL_TIM_PWM_Start(&htim8,TIM_CHANNEL_3);//Y--PI7
	PWM_SetDuty(&htim8,TIM_CHANNEL_3,0.50);//Y--PI7 PWM输出

}

void steer2_180(void)//舵机2-180度
{
	HAL_Init();

	SystemClock_Config();

  MX_GPIO_Init();
  MX_TIM8_Init();

	led_off();
	
	HAL_TIM_PWM_Start(&htim8,TIM_CHANNEL_3);//Y--PI7
	PWM_SetDuty(&htim8,TIM_CHANNEL_3,1.00);//Y--PI7 PWM输出

}

void steer3_0(void)//舵机3-0度
{
	HAL_Init();

	SystemClock_Config();

  MX_GPIO_Init();
  MX_TIM8_Init();

	led_off();
	
	HAL_TIM_PWM_Start(&htim8,TIM_CHANNEL_2);//X--PI6
	PWM_SetDuty(&htim8,TIM_CHANNEL_2,0.00);//X--PI6 PWM输出

}

void steer3_90(void)//舵机3-90度
{
	HAL_Init();

	SystemClock_Config();

  MX_GPIO_Init();
  MX_TIM8_Init();

	led_off();
	
	HAL_TIM_PWM_Start(&htim8,TIM_CHANNEL_2);//X--PI6
	PWM_SetDuty(&htim8,TIM_CHANNEL_2,0.50);//X--PI6 PWM输出

}

void steer3_180(void)//舵机3-180度
{
	HAL_Init();

	SystemClock_Config();

  MX_GPIO_Init();
  MX_TIM8_Init();

	led_off();
	
	HAL_TIM_PWM_Start(&htim8,TIM_CHANNEL_2);//X--PI6
	PWM_SetDuty(&htim8,TIM_CHANNEL_2,1.00);//X--PI6 PWM输出

}

