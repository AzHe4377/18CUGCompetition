#include "main.h"
#include "stm32f4xx_hal.h"
#include "tim.h"
#include "gpio.h"
#include "mecanum.h"
#include "pwm.h"



//使用A--PI0，B--PH12驱动左前电机
//使用C--PH11，D--PH10驱动右前电机
//使用E--PD15，F--PD14驱动左后电机
//使用G--PD13，H--PD12驱动右后电机
void move_forward(void)	//向前移动
{
	HAL_Init();

	SystemClock_Config();

  MX_GPIO_Init();
  MX_TIM4_Init();
  MX_TIM5_Init();

	led_off();
	
	
	HAL_TIM_PWM_Start(&htim5,TIM_CHANNEL_4);//A--PI0
	HAL_TIM_PWM_Start(&htim5,TIM_CHANNEL_3);//B--PH12
	PWM_SetDuty(&htim5,TIM_CHANNEL_4,0.30);//A--PI0 PWM输出
	PWM_SetDuty(&htim5,TIM_CHANNEL_3,0.00);//B--PH12 PWM输出
	//左前电机正转

	HAL_TIM_PWM_Start(&htim5,TIM_CHANNEL_2);//C--PH11
	HAL_TIM_PWM_Start(&htim5,TIM_CHANNEL_1);//D--PH10
	PWM_SetDuty(&htim5,TIM_CHANNEL_2,0.30);//C--PH11 PWM输出
	PWM_SetDuty(&htim5,TIM_CHANNEL_1,0.00);//D--PH10 PWM输出
	//右前电机正转	

	HAL_TIM_PWM_Start(&htim4,TIM_CHANNEL_4);//E--PD15
	HAL_TIM_PWM_Start(&htim4,TIM_CHANNEL_3);//F--PD14
	PWM_SetDuty(&htim4,TIM_CHANNEL_4,0.30);//E--PD15 PWM输出
	PWM_SetDuty(&htim4,TIM_CHANNEL_3,0.00);//F--PD14 PWM输出
	//左后电机正转
	
	HAL_TIM_PWM_Start(&htim4,TIM_CHANNEL_2);//G--PD13
	HAL_TIM_PWM_Start(&htim4,TIM_CHANNEL_1);//H--PD12
	PWM_SetDuty(&htim4,TIM_CHANNEL_2,0.30);//G--PD13 PWM输出
	PWM_SetDuty(&htim4,TIM_CHANNEL_1,0.00);//H--PD12 PWM输出
	//右后电机正转
	
}

void backwards(void)	//向后移动
{
	HAL_Init();

	SystemClock_Config();

  MX_GPIO_Init();
  MX_TIM4_Init();
  MX_TIM5_Init();

	led_off();
	
	
	HAL_TIM_PWM_Start(&htim5,TIM_CHANNEL_4);//A--PI0
	HAL_TIM_PWM_Start(&htim5,TIM_CHANNEL_3);//B--PH12
	PWM_SetDuty(&htim5,TIM_CHANNEL_4,0.00);//A--PI0 PWM输出
	PWM_SetDuty(&htim5,TIM_CHANNEL_3,0.30);//B--PH12 PWM输出
	//左前电机反转

	HAL_TIM_PWM_Start(&htim5,TIM_CHANNEL_2);//C--PH11
	HAL_TIM_PWM_Start(&htim5,TIM_CHANNEL_1);//D--PH10
	PWM_SetDuty(&htim5,TIM_CHANNEL_2,0.00);//C--PH11 PWM输出
	PWM_SetDuty(&htim5,TIM_CHANNEL_1,0.30);//D--PH10 PWM输出
	//右前电机反转

	HAL_TIM_PWM_Start(&htim4,TIM_CHANNEL_4);//E--PD15
	HAL_TIM_PWM_Start(&htim4,TIM_CHANNEL_3);//F--PD14
	PWM_SetDuty(&htim4,TIM_CHANNEL_4,0.00);//E--PD15 PWM输出
	PWM_SetDuty(&htim4,TIM_CHANNEL_3,0.30);//F--PD14 PWM输出
	//左后电机反转
	
	HAL_TIM_PWM_Start(&htim4,TIM_CHANNEL_2);//G--PD13
	HAL_TIM_PWM_Start(&htim4,TIM_CHANNEL_1);//H--PD12
	PWM_SetDuty(&htim4,TIM_CHANNEL_2,0.00);//G--PD13 PWM输出
	PWM_SetDuty(&htim4,TIM_CHANNEL_1,0.30);//H--PD12 PWM输出
	//右后电机反转
	
}


void move_left(void)	 		//向左平移
{
	HAL_Init();

	SystemClock_Config();

  MX_GPIO_Init();
  MX_TIM4_Init();
  MX_TIM5_Init();

	led_off();
	
	
	HAL_TIM_PWM_Start(&htim5,TIM_CHANNEL_4);//A--PI0
	HAL_TIM_PWM_Start(&htim5,TIM_CHANNEL_3);//B--PH12
	PWM_SetDuty(&htim5,TIM_CHANNEL_4,0.00);//A--PI0 PWM输出
	PWM_SetDuty(&htim5,TIM_CHANNEL_3,0.30);//B--PH12 PWM输出
	//左前电机反转

	HAL_TIM_PWM_Start(&htim5,TIM_CHANNEL_2);//C--PH11
	HAL_TIM_PWM_Start(&htim5,TIM_CHANNEL_1);//D--PH10
	PWM_SetDuty(&htim5,TIM_CHANNEL_2,0.30);//C--PH11 PWM输出
	PWM_SetDuty(&htim5,TIM_CHANNEL_1,0.00);//D--PH10 PWM输出
	//右前电机正转	

	HAL_TIM_PWM_Start(&htim4,TIM_CHANNEL_4);//E--PD15
	HAL_TIM_PWM_Start(&htim4,TIM_CHANNEL_3);//F--PD14
	PWM_SetDuty(&htim4,TIM_CHANNEL_4,0.30);//E--PD15 PWM输出
	PWM_SetDuty(&htim4,TIM_CHANNEL_3,0.00);//F--PD14 PWM输出
	//左后电机正转
	
	HAL_TIM_PWM_Start(&htim4,TIM_CHANNEL_2);//G--PD13
	HAL_TIM_PWM_Start(&htim4,TIM_CHANNEL_1);//H--PD12
	PWM_SetDuty(&htim4,TIM_CHANNEL_2,0.00);//G--PD13 PWM输出
	PWM_SetDuty(&htim4,TIM_CHANNEL_1,0.30);//H--PD12 PWM输出
	//右后电机反转
	
}

void move_right(void)		//向右平移
{
	HAL_Init();

	SystemClock_Config();

  MX_GPIO_Init();
  MX_TIM4_Init();
  MX_TIM5_Init();

	led_off();
	
	
	HAL_TIM_PWM_Start(&htim5,TIM_CHANNEL_4);//A--PI0
	HAL_TIM_PWM_Start(&htim5,TIM_CHANNEL_3);//B--PH12
	PWM_SetDuty(&htim5,TIM_CHANNEL_4,0.30);//A--PI0 PWM输出
	PWM_SetDuty(&htim5,TIM_CHANNEL_3,0.00);//B--PH12 PWM输出
	//左前电机正转

	HAL_TIM_PWM_Start(&htim5,TIM_CHANNEL_2);//C--PH11
	HAL_TIM_PWM_Start(&htim5,TIM_CHANNEL_1);//D--PH10
	PWM_SetDuty(&htim5,TIM_CHANNEL_2,0.00);//C--PH11 PWM输出
	PWM_SetDuty(&htim5,TIM_CHANNEL_1,0.30);//D--PH10 PWM输出
	//右前电机反转

	HAL_TIM_PWM_Start(&htim4,TIM_CHANNEL_4);//E--PD15
	HAL_TIM_PWM_Start(&htim4,TIM_CHANNEL_3);//F--PD14
	PWM_SetDuty(&htim4,TIM_CHANNEL_4,0.00);//E--PD15 PWM输出
	PWM_SetDuty(&htim4,TIM_CHANNEL_3,0.30);//F--PD14 PWM输出
	//左后电机反转
	
	HAL_TIM_PWM_Start(&htim4,TIM_CHANNEL_2);//G--PD13
	HAL_TIM_PWM_Start(&htim4,TIM_CHANNEL_1);//H--PD12
	PWM_SetDuty(&htim4,TIM_CHANNEL_2,0.30);//G--PD13 PWM输出
	PWM_SetDuty(&htim4,TIM_CHANNEL_1,0.00);//H--PD12 PWM输出
	//右后电机正转
	
}


void turn_left(void)			//向左转头
{
	HAL_Init();

	SystemClock_Config();

  MX_GPIO_Init();
  MX_TIM4_Init();
  MX_TIM5_Init();

	led_off();
	
	
	HAL_TIM_PWM_Start(&htim5,TIM_CHANNEL_4);//A--PI0
	HAL_TIM_PWM_Start(&htim5,TIM_CHANNEL_3);//B--PH12
	PWM_SetDuty(&htim5,TIM_CHANNEL_4,0.00);//A--PI0 PWM输出
	PWM_SetDuty(&htim5,TIM_CHANNEL_3,0.30);//B--PH12 PWM输出
	//左前电机反转

	HAL_TIM_PWM_Start(&htim5,TIM_CHANNEL_2);//C--PH11
	HAL_TIM_PWM_Start(&htim5,TIM_CHANNEL_1);//D--PH10
	PWM_SetDuty(&htim5,TIM_CHANNEL_2,0.30);//C--PH11 PWM输出
	PWM_SetDuty(&htim5,TIM_CHANNEL_1,0.00);//D--PH10 PWM输出
	//右前电机正转	

	HAL_TIM_PWM_Start(&htim4,TIM_CHANNEL_4);//E--PD15
	HAL_TIM_PWM_Start(&htim4,TIM_CHANNEL_3);//F--PD14
	PWM_SetDuty(&htim4,TIM_CHANNEL_4,0.00);//E--PD15 PWM输出
	PWM_SetDuty(&htim4,TIM_CHANNEL_3,0.30);//F--PD14 PWM输出
	//左后电机反转
	
	HAL_TIM_PWM_Start(&htim4,TIM_CHANNEL_2);//G--PD13
	HAL_TIM_PWM_Start(&htim4,TIM_CHANNEL_1);//H--PD12
	PWM_SetDuty(&htim4,TIM_CHANNEL_2,0.30);//G--PD13 PWM输出
	PWM_SetDuty(&htim4,TIM_CHANNEL_1,0.00);//H--PD12 PWM输出
	//右后电机正转
	
}



void turn_right(void)		//向右转头
{
	HAL_Init();

	SystemClock_Config();

  MX_GPIO_Init();
  MX_TIM4_Init();
  MX_TIM5_Init();

	led_off();
	
	
	HAL_TIM_PWM_Start(&htim5,TIM_CHANNEL_4);//A--PI0
	HAL_TIM_PWM_Start(&htim5,TIM_CHANNEL_3);//B--PH12
	PWM_SetDuty(&htim5,TIM_CHANNEL_4,0.30);//A--PI0 PWM输出
	PWM_SetDuty(&htim5,TIM_CHANNEL_3,0.00);//B--PH12 PWM输出
	//左前电机正转

	HAL_TIM_PWM_Start(&htim5,TIM_CHANNEL_2);//C--PH11
	HAL_TIM_PWM_Start(&htim5,TIM_CHANNEL_1);//D--PH10
	PWM_SetDuty(&htim5,TIM_CHANNEL_2,0.00);//C--PH11 PWM输出
	PWM_SetDuty(&htim5,TIM_CHANNEL_1,0.30);//D--PH10 PWM输出
	//右前电机反转	

	HAL_TIM_PWM_Start(&htim4,TIM_CHANNEL_4);//E--PD15
	HAL_TIM_PWM_Start(&htim4,TIM_CHANNEL_3);//F--PD14
	PWM_SetDuty(&htim4,TIM_CHANNEL_4,0.30);//E--PD15 PWM输出
	PWM_SetDuty(&htim4,TIM_CHANNEL_3,0.00);//F--PD14 PWM输出
	//左后电机正转
	
	HAL_TIM_PWM_Start(&htim4,TIM_CHANNEL_2);//G--PD13
	HAL_TIM_PWM_Start(&htim4,TIM_CHANNEL_1);//H--PD12
	PWM_SetDuty(&htim4,TIM_CHANNEL_2,0.00);//G--PD13 PWM输出
	PWM_SetDuty(&htim4,TIM_CHANNEL_1,0.30);//H--PD12 PWM输出
	//右后电机反转
	
}





