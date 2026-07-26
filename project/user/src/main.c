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

    system_delay_ms(300);           //等待主板其他外设上电完成

    // 电机初始化
    motor_init();
    motor_set(-20, -20);

    // 编码器初始化
    encoder_init();

    // 初始化光电管
	gs08ra_init();

    // 屏幕初始化
    tft180_set_dir(TFT180_PORTAIT);
    tft180_set_color(RGB565_BLACK, RGB565_WHITE);
    tft180_init();

    // 按键初始化
	key_init(10);
    
    // 板载亮灯
    gpio_init(A14, GPO, 0, GPO_PUSH_PULL);
    gpio_set_level(A14, 0);

    // 全局中断使能
    interrupt_global_enable(0);
    while(true)
    {
        // 此处编写需要循环执行的代码

        // -------------------- 按键处理 --------------------
		key_scanner();

        // tft180_clear();
        tft180_show_string( 0,  0*16,   "en_le:");                            // 显示字符串
        tft180_show_string( 0,  1*16,   "en_ri:");                            // 显示字符串
        tft180_show_int(    64,  0*16,  encoder[0],          4);    // 显示 int16 数据
        tft180_show_int(    64,  1*16,  encoder[1],          4);    // 显示 int16 数据

        gs08ra_scan_read();		// 获取灰度传感器的值
		gray_max_min_update();	// 更新灰度传感器最大最小值
        tft180_show_uint(    0*8,  4*16,  gs08ra_bin_val[0],          1);
        tft180_show_uint(    1*8,  4*16,  gs08ra_bin_val[1],          1);
        tft180_show_uint(    2*8,  4*16,  gs08ra_bin_val[2],          1);
        tft180_show_uint(    3*8,  4*16,  gs08ra_bin_val[3],          1);
        tft180_show_uint(    4*8,  4*16,  gs08ra_bin_val[4],          1);
        tft180_show_uint(    5*8,  4*16,  gs08ra_bin_val[5],          1);
        tft180_show_uint(    6*8,  4*16,  gs08ra_bin_val[6],          1);
        tft180_show_uint(    7*8,  4*16,  gs08ra_bin_val[7],          1);
        
        system_delay_ms(50);
        // 此处编写需要循环执行的代码
    }
}

// **************************** 代码区域 ****************************
