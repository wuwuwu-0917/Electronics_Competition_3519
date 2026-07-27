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

extern PID left_pid;
extern PID right_pid;
extern PID turn_pid;

void Incremental_PID_Init(PID *pid, float p, float i, float d, float maxOutput);
void Incremental_PID_Calc(PID *pid, float set_value,float get_value);
void Positional_PID_Init (PID *pid, float p, float i, float d, float maxOutput);
void Positional_PID_Calc(PID *pid, float set_value, float get_value);

#endif
