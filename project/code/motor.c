#include "global.h"

int motor_enable_flag = 0;        // 电机使能标志，=1时电机才能运转
int stop_line_flag  = 0;        // 停车线识别标志，=1时强制停车
int stop_line_count = 0;        // 已识别的停车线次数
int prev_on_line    = 0;        // 上一周期停车线状态（边沿检测）

/*电机初始化*/
void motor_init(void)
{
    gpio_init(MOTOR1_DIR, GPO, GPIO_HIGH, GPO_PUSH_PULL);   
    gpio_init(MOTOR2_DIR, GPO, GPIO_HIGH, GPO_PUSH_PULL);   

    pwm_init(MOTOR1_PWM, 17000, 0);                 //初始化两个通道pwm均输出频率为17000Hz，占空比为0
    pwm_init(MOTOR2_PWM, 17000, 0);     
}

 /*电机设置*/
 void motor_set(int8 left_rpm,int8 right_rpm)
 {
     // 电机使能未开启时，不输出任何驱动
     if(!motor_enable_flag) {
         pwm_set_duty(MOTOR1_PWM, 0);
         pwm_set_duty(MOTOR2_PWM, 0);
         return;
     }

     if(right_rpm >= 0)
	{			 
         gpio_set_level(MOTOR2_DIR,FORWARD);              //转速为正时为前进，gpio输出高电平
         pwm_set_duty(MOTOR2_PWM,right_rpm * (PWM_DUTY_MAX / 100));           //设置PWM输出占空比为pid输出值
     }
     else                                               //电机速度小于零则与上面相反，注意占空比设置前加负号
     {
         gpio_set_level(MOTOR2_DIR,BACK);
         pwm_set_duty(MOTOR2_PWM,(-right_rpm) * (PWM_DUTY_MAX / 100));
     }
     
     if(left_rpm >= 0)
     {
        gpio_set_level(MOTOR1_DIR,FORWARD);
        pwm_set_duty(MOTOR1_PWM,left_rpm * (PWM_DUTY_MAX / 100));
     }
     else
     {
        gpio_set_level(MOTOR1_DIR,BACK);
        pwm_set_duty(MOTOR1_PWM,(-left_rpm) * (PWM_DUTY_MAX / 100));
     }
 }