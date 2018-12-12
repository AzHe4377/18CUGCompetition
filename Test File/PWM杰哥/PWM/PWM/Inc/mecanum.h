#ifndef __mecanum_H
#define __mecanum_H
#ifdef __cplusplus
 extern "C" {
#endif
	 
#include "stm32f4xx_hal.h"
#include "main.h"

void move_forward(void);	//向前移动
void backwards(void);			//向后移动
void move_left(void);	 		//向左平移
void move_right(void);		//向右平移
void turn_left(void);			//向左转头
void turn_right(void);		//向右转头


#ifdef __cplusplus
}
#endif
#endif /*__ pinoutConfig_H */


