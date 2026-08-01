#ifndef _PIT_H_
#define _PIT_H_

#include "global.h"

extern int8 rpm;
extern int8 rpm_target;       // 最终目标速度（按键设定）
extern uint16 start_cnt;      // 慢速启动进度计数（S曲线缓启）
extern float brake_rpm;       // 缓停参考速度（固定小步长递减）

// 计时系统：按键2按下开始计时，停车线触发停止
extern volatile uint32 g_timer_count;    // 计时计数器（每10ms+1）
extern volatile uint8  g_timer_running;  // 1=正在计时, 0=已停止
extern uint32 g_timer_result;            // 停车时冻结的最终计时值

void pit_callback(uint32 event, void *ptr);

#endif
