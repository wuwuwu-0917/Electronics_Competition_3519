/*********************************************************************************************************************
* 修改记录
* 日期              作者                备注
* 2026-07-26        wwc            first version
* 2026-07-26        mqy            copy
* 2026-07-26        ljh            copy
********************************************************************************************************************/

#include "zf_common_headfile.h"
// 打开新的工程或者工程移动了位置务必执行以下操作
// 第一步 关闭上面所有打开的文件
// 第二步 project->clean  等待下方进度条走完


// **************************** 代码区域 ****************************

int main (void)
{
    clock_init(SYSTEM_CLOCK_80M);                                               // 时钟配置及系统初始化<务必保留>
    debug_init();                                                               // 调试串口信息初始化

    gpio_init(A14, GPO, 0, GPO_PUSH_PULL);
	
    gpio_set_level(A14, 0);
    
    // 此处编写用户代码 例如外设初始化代码等

    tft180_set_dir(TFT180_PORTAIT);
    tft180_set_color(RGB565_BLACK, RGB565_WHITE);
    tft180_init();

    encoder_init();
    // 此处编写用户代码 例如外设初始化代码等
    while(true)
    {
        // 此处编写需要循环执行的代码
        // tft180_clear();
        tft180_show_string( 0,  0*16,   "en_le:");                            // 显示字符串
        tft180_show_string( 0,  1*16,   "en_ri:");                            // 显示字符串
        tft180_show_int(    64,  0*16,  encoder[0],          4);    // 显示 int16 数据
        tft180_show_int(    64,  1*16,  encoder[1],          4);    // 显示 int16 数据
        // printf("ENCODER_1 counter \t%d .    ", encoder[0]);                 // 输出编码器计数信息
        // printf("ENCODER_2 counter \t%d .\r\n", encoder[1]);                 // 输出编码器计数信息  
        system_delay_ms(100);
        // 此处编写需要循环执行的代码
    }
}

// **************************** 代码区域 ****************************
