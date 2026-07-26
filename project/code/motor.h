#ifndef _MOTOR_H_
#define _MOTOR_H_

#include "zf_common_headfile.h"
#include "global.h"

/*电机接口引脚定义*/
#define MAX_DUTY            	 (50 )                                               // 最大 MAX_DUTY% 占空比
#define MOTOR1_DIR               (A1 )
#define MOTOR1_PWM               (PWM_TIM_A0_CH0_A0)

#define MOTOR2_DIR               (B13 )
#define MOTOR2_PWM               (PWM_TIM_A0_CH2_B12 )

/*电机转向*/
#define FORWARD  GPIO_LOW
#define BACK     GPIO_HIGH

void motor_init(void);
void motor_set(int8 left_rpm,int8 right_rpm);

#endif
