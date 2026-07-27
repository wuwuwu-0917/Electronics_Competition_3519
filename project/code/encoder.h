#ifndef _ENCODER_H_
#define _ENCODER_H_
#include "zf_common_headfile.h"

#define dir 0
#define quad 1

#if  dir
#define ENCODER1_TIMER  TIM_G7                  // 定义编码器方向引脚
#define ENCODER1_LSB    TIMG7_ENCODER1_CH1_A26  // 定义编码器脉冲引脚    
#define ENCODER1_DIR    B27                     // 定义编码器方向引脚

#define ENCODER2_TIMER  TIM_G6                  // 定义编码器方向引脚
#define ENCODER2_LSB    TIMG6_ENCODER1_CH1_B10  // 定义编码器脉冲引脚    
#define ENCODER2_DIR    B11                     // 定义编码器方向引脚
#endif

#if  quad
#define ENCODER1_TIMER  TIM_G8                  // 定义编码器方向引脚
#define ENCODER1_A      TIMG8_ENCODER1_CH1_A26  // 定义编码器脉冲引脚    
#define ENCODER1_B      TIMG8_ENCODER1_CH2_A27  // 定义编码器方向引脚

#define ENCODER2_TIMER  TIM_G9                  // 定义编码器方向引脚
#define ENCODER2_A      TIMG9_ENCODER1_CH1_B7  // 定义编码器脉冲引脚    
#define ENCODER2_B      TIMG9_ENCODER1_CH2_B9  // 定义编码器方向引脚
#endif

#define PIT_TIMER       ( PIT_TIM_G0 )          // 定义周期中断用的定时器

extern int16 encoder[2];                               // 编码器数据 0为左 1为右

void get_encoder(void);
void encoder_init(void);

#endif
