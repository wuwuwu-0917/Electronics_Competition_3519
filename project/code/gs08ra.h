#ifndef _GS08RA_H_
#define _GS08RA_H_
#include "zf_common_headfile.h"

#define LS_CHANNEL1            ( B25 )
#define LS_CHANNEL2            ( B20 )
#define LS_CHANNEL3            ( B18 )
#define LS_CHANNEL4            ( B17 )
#define LS_CHANNEL5            ( A17 )
#define LS_CHANNEL6            ( A16 )

// 光电管 最大最小值采集
extern uint16 gray_max[6];
extern uint16 gray_min[6];  // 12位ADC默认最大
extern uint8  gray_threshold;	  // 灰度阈值

extern float deviation[6];
extern float turn_div;

void gray_max_min_update(void);     // 采集并更新光电管最大最小值
void gray_max_min_reset(void);      // 重置最大最小值
void gray_save_max_min_to_array(void);      // 将采集的最大最小值直接写入 gs08ra_max_val / gs08ra_min_val


void get_deviation(void);       //获取巡线误差值

#endif
