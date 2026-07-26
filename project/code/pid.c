#include "global.h"

PID pid;

/*增量式PID参数的初始化*/
void Incremental_PID_Init(PID *pid, int16 p, int16 i, int16 d, int32 maxOutput)
{
	pid->kp = p;
	pid->ki = i;
	pid->kd = d;
	pid->maxOutput = maxOutput;

  return;
}

/*增量式pi控制器（根据两次偏差进行调整）*/
void Incremental_PID_Calc(PID *pid, int32 set_value,int32 get_value)
{
	pid->error = set_value - get_value;      									//计算偏差
	pid->output += (pid->kp*(pid->error - pid->lastError) + pid->ki*pid->error + 
	pid->kd*(pid->error - 2*pid->lastError + pid->lastlastError)) / PID_SCALE;			//增量式PI控制器
	pid->lastlastError = pid->lastError;    											//保存上上次误差
	pid->lastError = pid->error;	           											//保存上一次偏差

	if (pid->output > pid->maxOutput)
        pid->output = pid->maxOutput;
    else if (pid->output < -pid->maxOutput)
        pid->output = -pid->maxOutput;													//输出限幅
}

/*位置式PID参数的初始化*/
void Positional_PID_Init (PID *pid, int16 p, int16 i, int16 d, int32 maxOutput)
{
    pid->kp = p;
    pid->ki = i;
    pid->kd = d;
    pid->maxOutput = maxOutput;
}

/*位置式PID控制器*/
void Positional_PID_Calc(PID *pid, int32 set_value, int32 get_value)
{
    pid->error = set_value - get_value;         // 计算偏差
    pid->output = (pid->kp * pid->error + pid->ki * pid->integral + pid->kd * (pid->error - 2 * pid->lastError + pid->lastlastError)) / PID_SCALE; // 位置式PID控制器
    pid->lastlastError = pid->lastError;       // 保存上上次误差
    pid->lastError = pid->error;                // 保存上一次偏差

    if (pid->output > pid->maxOutput)
        pid->output = pid->maxOutput;
    else if (pid->output < -pid->maxOutput)
        pid->output = -pid->maxOutput;          // 输出限幅
}