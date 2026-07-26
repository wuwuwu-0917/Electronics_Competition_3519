#include "zf_common_headfile.h"

int16 encoder[2];                               // 编码器数据

void get_encoder(uint32 event, void *ptr)       //读取编码器数据的中断回调函数
{
    encoder[0] = encoder_get_count(ENCODER1_TIMER);     // 采集编码器数据    
    encoder[1] = encoder_get_count(ENCODER2_TIMER);     // 采集编码器数据     
    encoder_clear_count(ENCODER1_TIMER);                // 编码器数据采集完成后务必清零 
    encoder_clear_count(ENCODER2_TIMER);                // 编码器数据采集完成后务必清零
}

void encoder_init(void)     //初始化编码器和中断
{
    encoder_dir_init(ENCODER1_TIMER, ENCODER1_LSB, ENCODER1_DIR);  // 初始化编码器1端口  
    encoder_dir_init(ENCODER2_TIMER, ENCODER2_LSB, ENCODER2_DIR);  // 初始化编码器2端口  
    pit_ms_init(PIT_TIMER, 10, get_encoder, NULL);//10ms读取一次编码器
}
