#ifndef _PID_H_
#define _PID_H_

#include "zf_common_headfile.h"
#include "global.h"

typedef struct          //正常的PID结构
{
    float kp;
    float ki;
    float kd;
    float error;
    float lastError;
    float lastlastError;
    float integral;
    float maxIntegral;
    float output;
    float maxOutput;
}PID;

/* 滑动平均滤波器结构体 */
#define MOVING_AVG_NUM  5                                   // 滤波窗口大小
typedef struct
{
    float buffer[MOVING_AVG_NUM];                           // 数据缓冲区（循环队列）
    uint8  index;                                           // 当前写入位置
    uint8  count;                                           // 已填充数据个数（< N 时未填满）
    float  sum;                                             // 当前总和
}MovingAverage;

extern PID left_pid;
extern PID right_pid;
extern PID turn_pid;

extern MovingAverage turn_filter;                           // 方向环输出滤波器

void Incremental_PID_Init(PID *pid, float p, float i, float d, float maxOutput);
void Incremental_PID_Calc(PID *pid, float set_value,float get_value);
void Positional_PID_Init (PID *pid, float p, float i, float d, float maxOutput);
void Positional_PID_Calc(PID *pid, float set_value, float get_value);

void MovingAverage_Init(MovingAverage *f);
float MovingAverage_Calc(MovingAverage *f, float input);

#endif
