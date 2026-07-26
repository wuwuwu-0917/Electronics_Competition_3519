#ifndef _PID_H_
#define _PID_H_

#include "zf_common_headfile.h"
#include "global.h"

typedef struct          //正常的PID结构
{
    int32 kp;
    int32 ki;
    int32 kd;
    int32 error;
    int32 lastError;
    int32 lastlastError;
    int32 integral;
    int32 maxIntegral;
    int32 output;
    int32 maxOutput;
}PID;

void Incremental_PID_Init(PID *pid, int16 p, int16 i, int16 d, int32 maxOutput);
void Incremental_PID_Calc(PID *pid, int32 set_value,int32 get_value);
void Positional_PID_Init (PID *pid, int16 p, int16 i, int16 d, int32 maxOutput);
void Positional_PID_Calc(PID *pid, int32 set_value, int32 get_value);
#endif
