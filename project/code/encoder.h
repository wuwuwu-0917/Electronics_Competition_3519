#ifndef _ENCODER_H_
#define _ENCODER_H_
#include "zf_common_headfile.h"

#define ENCODER1_TIMER  TIM_G7                  // 定义编码器方向引脚
#define ENCODER1_LSB    TIMG7_ENCODER1_CH1_A26  // 定义编码器脉冲引脚    
#define ENCODER1_DIR    B27                     // 定义编码器方向引脚

#define ENCODER2_TIMER  TIM_G6                  // 定义编码器方向引脚
#define ENCODER2_LSB    TIMG6_ENCODER1_CH1_B10  // 定义编码器脉冲引脚    
#define ENCODER2_DIR    B11                     // 定义编码器方向引脚

#define PIT_TIMER       ( PIT_TIM_G0 )          // 定义周期中断用的定时器

extern int16 encoder[2];                               // 编码器数据

void get_encoder(uint32 event, void *ptr);
void encoder_init(void);

#endif
