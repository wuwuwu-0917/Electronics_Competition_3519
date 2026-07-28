#include "zf_common_headfile.h"
#include "global.h"

int16 encoder[2];                               // 编码器数据

void get_encoder(void)       //读取编码器数据
{
    encoder[0] = encoder_get_count(ENCODER1_TIMER);     // 采集编码器数据    
    encoder[1] = encoder_get_count(ENCODER2_TIMER);     // 采集编码器数据     
    encoder_clear_count(ENCODER1_TIMER);                // 编码器数据采集完成后务必清零 
    encoder_clear_count(ENCODER2_TIMER);                // 编码器数据采集完成后务必清零
}

void encoder_init(void)     //初始化编码器和中断
{
	
		#if  dir
			encoder_dir_init(ENCODER1_TIMER, ENCODER1_LSB, ENCODER1_DIR);  // 初始化编码器1端口  
			encoder_dir_init(ENCODER2_TIMER, ENCODER2_LSB, ENCODER2_DIR);  // 初始化编码器2端口  
		#endif

		#if  quad
			encoder_quad_init(ENCODER1_TIMER, ENCODER1_A, ENCODER1_B);  // 初始化编码器1端口  
			encoder_quad_init(ENCODER2_TIMER, ENCODER2_A, ENCODER2_B);  // 初始化编码器2端口  
		#endif
}
