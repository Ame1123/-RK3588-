#ifndef __PWM_H
#define __PWM_H

void PWM_Init(void);
void PWM_SetCompare2(uint16_t Compare);        // TIM2通道2 (PA1-舵机1)
void PWM_SetCompare3(uint16_t Compare);        // TIM2通道3 (PA2-舵机2)
void PWM_SetCompare4(uint16_t Compare);        // TIM2通道4 (PA3-舵机3)
void PWM_SetCompare4_TIM4(uint16_t Compare);   // TIM4通道1 (PB6-舵机4)
void PWM_SetCompare_TIM4CH2(uint16_t Compare); // TIM4通道2 (PB7-舵机5)
void PWM_SetCompare_TIM4CH4(uint16_t Compare); // TIM4通道4 (PB9-舵机6)

#endif
