#include "main.h"
#include "stm32f4xx_hal.h"
#include "tim.h"
#include "gpio.h"
#include "pwm.h"
#include "steer.h"

	//PI2     ------> TIM8_CH4 	Z�Ӷ��1����еצץȡ�����
	//PI7     ------> TIM8_CH3	Y�Ӷ��2����еצץȡ�����
	//PI6     ------> TIM8_CH2	x�Ӷ��3����е��ˮƽת�������

void steer1_0(void)
{
	HAL_Init();

	SystemClock_Config();

  MX_GPIO_Init();
  MX_TIM8_Init();

	led_off();
	

	HAL_TIM_PWM_Start(&htim8,TIM_CHANNEL_4);//Z--PI2
	PWM_SetDuty(&htim8,TIM_CHANNEL_4,0.025);//Z--PI2 PWM���

}

void steer1_90(void)//���1-90��
{
	HAL_Init();

	SystemClock_Config();

  MX_GPIO_Init();
  MX_TIM8_Init();

	led_off();
	
	HAL_TIM_PWM_Start(&htim8,TIM_CHANNEL_4);//Z--PI2
	PWM_SetDuty(&htim8,TIM_CHANNEL_4,0.075);//Z--PI2 PWM���

}

void steer1_180(void)//���1-180��
{
	HAL_Init();

	SystemClock_Config();

  MX_GPIO_Init();
  MX_TIM8_Init();

	led_off();
	
	HAL_TIM_PWM_Start(&htim8,TIM_CHANNEL_4);//Z--PI2
	PWM_SetDuty(&htim8,TIM_CHANNEL_4,0.125);//Z--PI2 PWM���

}

void steer2_0(void)//���2-0��
{
	HAL_Init();

	SystemClock_Config();

  MX_GPIO_Init();
  MX_TIM8_Init();

	led_off();
	
	HAL_TIM_PWM_Start(&htim8,TIM_CHANNEL_3);//Y--PI7
	PWM_SetDuty(&htim8,TIM_CHANNEL_3,0.00);//Y--PI7 PWM���

}

void steer2_90(void)//���2-90��
{
	HAL_Init();

	SystemClock_Config();

  MX_GPIO_Init();
  MX_TIM8_Init();

	led_off();
	
	HAL_TIM_PWM_Start(&htim8,TIM_CHANNEL_3);//Y--PI7
	PWM_SetDuty(&htim8,TIM_CHANNEL_3,0.50);//Y--PI7 PWM���

}

void steer2_180(void)//���2-180��
{
	HAL_Init();

	SystemClock_Config();

  MX_GPIO_Init();
  MX_TIM8_Init();

	led_off();
	
	HAL_TIM_PWM_Start(&htim8,TIM_CHANNEL_3);//Y--PI7
	PWM_SetDuty(&htim8,TIM_CHANNEL_3,1.00);//Y--PI7 PWM���

}

void steer3_0(void)//���3-0��
{
	HAL_Init();

	SystemClock_Config();

  MX_GPIO_Init();
  MX_TIM8_Init();

	led_off();
	
	HAL_TIM_PWM_Start(&htim8,TIM_CHANNEL_2);//X--PI6
	PWM_SetDuty(&htim8,TIM_CHANNEL_2,0.00);//X--PI6 PWM���

}

void steer3_90(void)//���3-90��
{
	HAL_Init();

	SystemClock_Config();

  MX_GPIO_Init();
  MX_TIM8_Init();

	led_off();
	
	HAL_TIM_PWM_Start(&htim8,TIM_CHANNEL_2);//X--PI6
	PWM_SetDuty(&htim8,TIM_CHANNEL_2,0.50);//X--PI6 PWM���

}

void steer3_180(void)//���3-180��
{
	HAL_Init();

	SystemClock_Config();

  MX_GPIO_Init();
  MX_TIM8_Init();

	led_off();
	
	HAL_TIM_PWM_Start(&htim8,TIM_CHANNEL_2);//X--PI6
	PWM_SetDuty(&htim8,TIM_CHANNEL_2,1.00);//X--PI6 PWM���

}

