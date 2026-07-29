#ifndef _MOTOR_H_
#define _MOTOR_H_

#include "zf_common_headfile.h"
#include "global.h"

extern int motor_enable_flag;
extern int stop_line_flag;      // 停车线识别标志，=1时强制停车
extern int stop_line_count;     // 已识别的停车线次数
extern int prev_on_line;        // 上一周期停车线状态（边沿检测）

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
