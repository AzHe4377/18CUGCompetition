#include "main.h"
#include "stm32f4xx_hal.h"
#include "tim.h"
#include "gpio.h"
#include "mecanum.h"
#include "pwm.h"



//ʹ��A--PI0��B--PH12������ǰ���
//ʹ��C--PH11��D--PH10������ǰ���
//ʹ��E--PD15��F--PD14���������
//ʹ��G--PD13��H--PD12�����Һ���
void move_forward(void)	//��ǰ�ƶ�
{
	HAL_Init();

	SystemClock_Config();

  MX_GPIO_Init();
  MX_TIM4_Init();
  MX_TIM5_Init();

	led_off();
	
	
	HAL_TIM_PWM_Start(&htim5,TIM_CHANNEL_4);//A--PI0
	HAL_TIM_PWM_Start(&htim5,TIM_CHANNEL_3);//B--PH12
	PWM_SetDuty(&htim5,TIM_CHANNEL_4,0.30);//A--PI0 PWM���
	PWM_SetDuty(&htim5,TIM_CHANNEL_3,0.00);//B--PH12 PWM���
	//��ǰ�����ת

	HAL_TIM_PWM_Start(&htim5,TIM_CHANNEL_2);//C--PH11
	HAL_TIM_PWM_Start(&htim5,TIM_CHANNEL_1);//D--PH10
	PWM_SetDuty(&htim5,TIM_CHANNEL_2,0.30);//C--PH11 PWM���
	PWM_SetDuty(&htim5,TIM_CHANNEL_1,0.00);//D--PH10 PWM���
	//��ǰ�����ת	

	HAL_TIM_PWM_Start(&htim4,TIM_CHANNEL_4);//E--PD15
	HAL_TIM_PWM_Start(&htim4,TIM_CHANNEL_3);//F--PD14
	PWM_SetDuty(&htim4,TIM_CHANNEL_4,0.30);//E--PD15 PWM���
	PWM_SetDuty(&htim4,TIM_CHANNEL_3,0.00);//F--PD14 PWM���
	//�������ת
	
	HAL_TIM_PWM_Start(&htim4,TIM_CHANNEL_2);//G--PD13
	HAL_TIM_PWM_Start(&htim4,TIM_CHANNEL_1);//H--PD12
	PWM_SetDuty(&htim4,TIM_CHANNEL_2,0.30);//G--PD13 PWM���
	PWM_SetDuty(&htim4,TIM_CHANNEL_1,0.00);//H--PD12 PWM���
	//�Һ�����ת
	
}

void backwards(void)	//����ƶ�
{
	HAL_Init();

	SystemClock_Config();

  MX_GPIO_Init();
  MX_TIM4_Init();
  MX_TIM5_Init();

	led_off();
	
	
	HAL_TIM_PWM_Start(&htim5,TIM_CHANNEL_4);//A--PI0
	HAL_TIM_PWM_Start(&htim5,TIM_CHANNEL_3);//B--PH12
	PWM_SetDuty(&htim5,TIM_CHANNEL_4,0.00);//A--PI0 PWM���
	PWM_SetDuty(&htim5,TIM_CHANNEL_3,0.30);//B--PH12 PWM���
	//��ǰ�����ת

	HAL_TIM_PWM_Start(&htim5,TIM_CHANNEL_2);//C--PH11
	HAL_TIM_PWM_Start(&htim5,TIM_CHANNEL_1);//D--PH10
	PWM_SetDuty(&htim5,TIM_CHANNEL_2,0.00);//C--PH11 PWM���
	PWM_SetDuty(&htim5,TIM_CHANNEL_1,0.30);//D--PH10 PWM���
	//��ǰ�����ת

	HAL_TIM_PWM_Start(&htim4,TIM_CHANNEL_4);//E--PD15
	HAL_TIM_PWM_Start(&htim4,TIM_CHANNEL_3);//F--PD14
	PWM_SetDuty(&htim4,TIM_CHANNEL_4,0.00);//E--PD15 PWM���
	PWM_SetDuty(&htim4,TIM_CHANNEL_3,0.30);//F--PD14 PWM���
	//�������ת
	
	HAL_TIM_PWM_Start(&htim4,TIM_CHANNEL_2);//G--PD13
	HAL_TIM_PWM_Start(&htim4,TIM_CHANNEL_1);//H--PD12
	PWM_SetDuty(&htim4,TIM_CHANNEL_2,0.00);//G--PD13 PWM���
	PWM_SetDuty(&htim4,TIM_CHANNEL_1,0.30);//H--PD12 PWM���
	//�Һ�����ת
	
}


void move_left(void)	 		//����ƽ��
{
	HAL_Init();

	SystemClock_Config();

  MX_GPIO_Init();
  MX_TIM4_Init();
  MX_TIM5_Init();

	led_off();
	
	
	HAL_TIM_PWM_Start(&htim5,TIM_CHANNEL_4);//A--PI0
	HAL_TIM_PWM_Start(&htim5,TIM_CHANNEL_3);//B--PH12
	PWM_SetDuty(&htim5,TIM_CHANNEL_4,0.00);//A--PI0 PWM���
	PWM_SetDuty(&htim5,TIM_CHANNEL_3,0.30);//B--PH12 PWM���
	//��ǰ�����ת

	HAL_TIM_PWM_Start(&htim5,TIM_CHANNEL_2);//C--PH11
	HAL_TIM_PWM_Start(&htim5,TIM_CHANNEL_1);//D--PH10
	PWM_SetDuty(&htim5,TIM_CHANNEL_2,0.30);//C--PH11 PWM���
	PWM_SetDuty(&htim5,TIM_CHANNEL_1,0.00);//D--PH10 PWM���
	//��ǰ�����ת	

	HAL_TIM_PWM_Start(&htim4,TIM_CHANNEL_4);//E--PD15
	HAL_TIM_PWM_Start(&htim4,TIM_CHANNEL_3);//F--PD14
	PWM_SetDuty(&htim4,TIM_CHANNEL_4,0.30);//E--PD15 PWM���
	PWM_SetDuty(&htim4,TIM_CHANNEL_3,0.00);//F--PD14 PWM���
	//�������ת
	
	HAL_TIM_PWM_Start(&htim4,TIM_CHANNEL_2);//G--PD13
	HAL_TIM_PWM_Start(&htim4,TIM_CHANNEL_1);//H--PD12
	PWM_SetDuty(&htim4,TIM_CHANNEL_2,0.00);//G--PD13 PWM���
	PWM_SetDuty(&htim4,TIM_CHANNEL_1,0.30);//H--PD12 PWM���
	//�Һ�����ת
	
}

void move_right(void)		//����ƽ��
{
	HAL_Init();

	SystemClock_Config();

  MX_GPIO_Init();
  MX_TIM4_Init();
  MX_TIM5_Init();

	led_off();
	
	
	HAL_TIM_PWM_Start(&htim5,TIM_CHANNEL_4);//A--PI0
	HAL_TIM_PWM_Start(&htim5,TIM_CHANNEL_3);//B--PH12
	PWM_SetDuty(&htim5,TIM_CHANNEL_4,0.30);//A--PI0 PWM���
	PWM_SetDuty(&htim5,TIM_CHANNEL_3,0.00);//B--PH12 PWM���
	//��ǰ�����ת

	HAL_TIM_PWM_Start(&htim5,TIM_CHANNEL_2);//C--PH11
	HAL_TIM_PWM_Start(&htim5,TIM_CHANNEL_1);//D--PH10
	PWM_SetDuty(&htim5,TIM_CHANNEL_2,0.00);//C--PH11 PWM���
	PWM_SetDuty(&htim5,TIM_CHANNEL_1,0.30);//D--PH10 PWM���
	//��ǰ�����ת

	HAL_TIM_PWM_Start(&htim4,TIM_CHANNEL_4);//E--PD15
	HAL_TIM_PWM_Start(&htim4,TIM_CHANNEL_3);//F--PD14
	PWM_SetDuty(&htim4,TIM_CHANNEL_4,0.00);//E--PD15 PWM���
	PWM_SetDuty(&htim4,TIM_CHANNEL_3,0.30);//F--PD14 PWM���
	//�������ת
	
	HAL_TIM_PWM_Start(&htim4,TIM_CHANNEL_2);//G--PD13
	HAL_TIM_PWM_Start(&htim4,TIM_CHANNEL_1);//H--PD12
	PWM_SetDuty(&htim4,TIM_CHANNEL_2,0.30);//G--PD13 PWM���
	PWM_SetDuty(&htim4,TIM_CHANNEL_1,0.00);//H--PD12 PWM���
	//�Һ�����ת
	
}


void turn_left(void)			//����תͷ
{
	HAL_Init();

	SystemClock_Config();

  MX_GPIO_Init();
  MX_TIM4_Init();
  MX_TIM5_Init();

	led_off();
	
	
	HAL_TIM_PWM_Start(&htim5,TIM_CHANNEL_4);//A--PI0
	HAL_TIM_PWM_Start(&htim5,TIM_CHANNEL_3);//B--PH12
	PWM_SetDuty(&htim5,TIM_CHANNEL_4,0.00);//A--PI0 PWM���
	PWM_SetDuty(&htim5,TIM_CHANNEL_3,0.30);//B--PH12 PWM���
	//��ǰ�����ת

	HAL_TIM_PWM_Start(&htim5,TIM_CHANNEL_2);//C--PH11
	HAL_TIM_PWM_Start(&htim5,TIM_CHANNEL_1);//D--PH10
	PWM_SetDuty(&htim5,TIM_CHANNEL_2,0.30);//C--PH11 PWM���
	PWM_SetDuty(&htim5,TIM_CHANNEL_1,0.00);//D--PH10 PWM���
	//��ǰ�����ת	

	HAL_TIM_PWM_Start(&htim4,TIM_CHANNEL_4);//E--PD15
	HAL_TIM_PWM_Start(&htim4,TIM_CHANNEL_3);//F--PD14
	PWM_SetDuty(&htim4,TIM_CHANNEL_4,0.00);//E--PD15 PWM���
	PWM_SetDuty(&htim4,TIM_CHANNEL_3,0.30);//F--PD14 PWM���
	//�������ת
	
	HAL_TIM_PWM_Start(&htim4,TIM_CHANNEL_2);//G--PD13
	HAL_TIM_PWM_Start(&htim4,TIM_CHANNEL_1);//H--PD12
	PWM_SetDuty(&htim4,TIM_CHANNEL_2,0.30);//G--PD13 PWM���
	PWM_SetDuty(&htim4,TIM_CHANNEL_1,0.00);//H--PD12 PWM���
	//�Һ�����ת
	
}



void turn_right(void)		//����תͷ
{
	HAL_Init();

	SystemClock_Config();

  MX_GPIO_Init();
  MX_TIM4_Init();
  MX_TIM5_Init();

	led_off();
	
	
	HAL_TIM_PWM_Start(&htim5,TIM_CHANNEL_4);//A--PI0
	HAL_TIM_PWM_Start(&htim5,TIM_CHANNEL_3);//B--PH12
	PWM_SetDuty(&htim5,TIM_CHANNEL_4,0.30);//A--PI0 PWM���
	PWM_SetDuty(&htim5,TIM_CHANNEL_3,0.00);//B--PH12 PWM���
	//��ǰ�����ת

	HAL_TIM_PWM_Start(&htim5,TIM_CHANNEL_2);//C--PH11
	HAL_TIM_PWM_Start(&htim5,TIM_CHANNEL_1);//D--PH10
	PWM_SetDuty(&htim5,TIM_CHANNEL_2,0.00);//C--PH11 PWM���
	PWM_SetDuty(&htim5,TIM_CHANNEL_1,0.30);//D--PH10 PWM���
	//��ǰ�����ת	

	HAL_TIM_PWM_Start(&htim4,TIM_CHANNEL_4);//E--PD15
	HAL_TIM_PWM_Start(&htim4,TIM_CHANNEL_3);//F--PD14
	PWM_SetDuty(&htim4,TIM_CHANNEL_4,0.30);//E--PD15 PWM���
	PWM_SetDuty(&htim4,TIM_CHANNEL_3,0.00);//F--PD14 PWM���
	//�������ת
	
	HAL_TIM_PWM_Start(&htim4,TIM_CHANNEL_2);//G--PD13
	HAL_TIM_PWM_Start(&htim4,TIM_CHANNEL_1);//H--PD12
	PWM_SetDuty(&htim4,TIM_CHANNEL_2,0.00);//G--PD13 PWM���
	PWM_SetDuty(&htim4,TIM_CHANNEL_1,0.30);//H--PD12 PWM���
	//�Һ�����ת
	
}





