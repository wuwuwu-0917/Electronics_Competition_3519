#include "global.h"

PID left_pid;
PID right_pid;

/*增量式PID参数的初始化*/
void Incremental_PID_Init(PID *pid, float p, float i, float d, float maxOutput)
{
	pid->kp = p;
	pid->ki = i;
	pid->kd = d;
	pid->maxOutput = maxOutput;

    return;
}

/*增量式pi控制器（根据两次偏差进行调整）*/
void Incremental_PID_Calc(PID *pid, float set_value,float get_value)
{
	pid->error = set_value - get_value;      									//计算偏差
	pid->output += pid->kp*(pid->error - pid->lastError) + pid->ki*pid->error + 
	pid->kd*(pid->error - 2*pid->lastError + pid->lastlastError);			//增量式PI控制器
	pid->lastlastError = pid->lastError;    											//保存上上次误差
	pid->lastError = pid->error;	           											//保存上一次偏差

	if (pid->output > pid->maxOutput)
        pid->output = pid->maxOutput;
    else if (pid->output < -pid->maxOutput)
        pid->output = -pid->maxOutput;													//输出限幅
}

/*位置式PID参数的初始化*/
void Positional_PID_Init (PID *pid, float p, float i, float d, float maxOutput)
{
    pid->kp = p;
    pid->ki = i;
    pid->kd = d;
    pid->maxOutput = maxOutput;
}

/*位置式PID控制器*/
void Positional_PID_Calc(PID *pid, float set_value, float get_value)
{
    float dout,pout;
    //更新数据
    pid->lastError = pid->error; 							//将旧error存起来
    pid->error = set_value - get_value; 					//计算新error

    dout = (pid->error - pid->lastError) * pid->kd;         //计算微分
    pout = pid->error * pid->kp;                            //计算比例
    pid->integral += pid->error * pid->ki;                  //计算积分

    //积分限幅
    if (pid->integral > pid->maxIntegral)
        pid->integral = pid->maxIntegral;
    else if (pid->integral < -pid->maxIntegral)
        pid->integral = -pid->maxIntegral;

    pid->output = pout + dout + pid->integral;              //计算输出
    
    //输出限幅
    if (pid->output > pid->maxOutput)
        pid->output = pid->maxOutput;
    else if (pid->output < -pid->maxOutput)
        pid->output = -pid->maxOutput;
}
