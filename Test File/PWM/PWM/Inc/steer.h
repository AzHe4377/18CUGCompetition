#ifndef __steer_H
#define __steer_H
#ifdef __cplusplus
 extern "C" {
#endif
	 
#include "stm32f4xx_hal.h"
#include "main.h"

void steer1_0();//舵机1-0度
void steer1_90();//舵机1-90度
void steer1_180();//舵机1-180度
	 
void steer2_0();//舵机2-0度
void steer2_90();//舵机2-90度
void steer2_180();//舵机2-180度

void steer3_0();//舵机3-0度
void steer3_90();//舵机3-90度
void steer3_180();//舵机3-180度


	 
#ifdef __cplusplus
}
#endif
#endif /*__ pinoutConfig_H */


