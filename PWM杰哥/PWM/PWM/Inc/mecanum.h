#ifndef __mecanum_H
#define __mecanum_H
#ifdef __cplusplus
 extern "C" {
#endif
	 
#include "stm32f4xx_hal.h"
#include "main.h"

void move_forward(void);	//��ǰ�ƶ�
void backwards(void);			//����ƶ�
void move_left(void);	 		//����ƽ��
void move_right(void);		//����ƽ��
void turn_left(void);			//����תͷ
void turn_right(void);		//����תͷ


#ifdef __cplusplus
}
#endif
#endif /*__ pinoutConfig_H */


