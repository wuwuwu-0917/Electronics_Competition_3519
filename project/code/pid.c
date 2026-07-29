#include "global.h"

PID left_pid;
PID right_pid;
PID turn_pid;

MovingAverage turn_filter;                                  // 方向环输出滤波器


/* 滑动平均滤波器初始化 */
void MovingAverage_Init(MovingAverage *f)
{
    uint8 i;
    for (i = 0; i < MOVING_AVG_NUM; i++)
        f->buffer[i] = 0.0f;
    f->index = 0;
    f->count = 0;
    f->sum   = 0.0f;
}

/* 滑动平均滤波器计算：输入新值，返回滤波后的平均值 */
float MovingAverage_Calc(MovingAverage *f, float input)
{
    f->sum += input;                                        // 累加新值
    if (f->count < MOVING_AVG_NUM)
    {
        f->buffer[f->index] = input;                        // 填缓冲区
        f->count++;
        f->index = (f->index + 1) % MOVING_AVG_NUM;
        return f->sum / (float)f->count;                    // 未填满时按实际个数平均
    }
    else
    {
        f->sum -= f->buffer[f->index];                      // 减去最旧的值
        f->buffer[f->index] = input;                        // 覆盖最旧的值
        f->index = (f->index + 1) % MOVING_AVG_NUM;
        return f->sum / (float)MOVING_AVG_NUM;              // 已填满，除以窗口大小
    }
}


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
void Incremental_PID_Calc(PID *pid, float set_value, float get_value)
{
	pid->error = set_value - get_value;      									//计算偏差
	pid->output += (pid->kp * (pid->error - pid->lastError) + pid->ki * pid->error +
	pid->kd * (pid->error - 2 * pid->lastError + pid->lastlastError));			//增量式PI控制器
	pid->lastlastError = pid->lastError;    										//保存上上次误差
	pid->lastError = pid->error;	           									//保存上一次偏差

	if (pid->output > pid->maxOutput)
        pid->output = pid->maxOutput;
    else if (pid->output < -pid->maxOutput)
        pid->output = -pid->maxOutput;											//输出限幅
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
