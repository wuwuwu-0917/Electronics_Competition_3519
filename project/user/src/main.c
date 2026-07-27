/*********************************************************************************************************************
* 修改记录
* 日期              作者                备注
* 2026-07-26        wwc            first version
* 2026-07-26        mqy            copy
* 2026-07-26        ljh            copy
********************************************************************************************************************/

#include "zf_common_headfile.h"
#include "global.h"

// 打开新的工程或者工程移动了位置务必执行以下操作
// 第一步 关闭上面所有打开的文件
// 第二步 project->clean  等待下方进度条走完


// **************************** 代码区域 ****************************

int main (void)
{
    clock_init(SYSTEM_CLOCK_80M);                                               // 时钟配置及系统初始化<务必保留>
    debug_init();                                                               // 调试串口信息初始化

    system_delay_ms(300);           //等待主板其他外设上电完成

    // 此处编写用户代码 例如外设初始化代码等

    // 电机初始化
    motor_init();

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

    // 初始化PID参数    
    Incremental_PID_Init(&left_pid, 0.68, 0.0009 , 0, 30, 50);     // 初始化左电机PID参数（积分限幅20）
    Incremental_PID_Init(&right_pid, 0.7, 0.0008, 0, 30, 50);    // 初始化右电机PID参数（积分限幅20）
  
    encoder_init();

    pit_ms_init(PIT_TIM_G0, 10, pit_callback, NULL);            // 初始化PIT周期中断，10ms执行一次控制

    interrupt_global_enable(0);                                  // 所有外设初始化完成后，开启全局中断

    // 此处编写用户代码 例如外设初始化代码等
    while(true)
    {
        // 此处编写需要循环执行的代码

        // -------------------- 按键处理 --------------------
		key_scanner();

        // tft180_clear();
        tft180_show_string( 0,  0*16,   "en_le:");                            // 显示左编码器
        tft180_show_string( 0,  1*16,   "en_ri:");                            // 显示右编码器
        tft180_show_int(    64,  0*16,  encoder[0],          4);              // 显示 int16 数据
        tft180_show_int(    64,  1*16,  encoder[1],          4);              // 显示 int16 数据
        tft180_show_string( 0,  2*16,   "pid_l:");                            // 显示左轮PID输出
        tft180_show_string( 0,  3*16,   "pid_r:");                            // 显示右轮PID输出
        tft180_show_float(  48,  2*16,  left_pid.output,  6, 1);              // 显示 float（总6位，小数1位）
        tft180_show_float(  48,  3*16,  right_pid.output, 6, 1);              // 显示 float（总6位，小数1位）
		
		tft180_show_string( 0,  4*16,   "errL:");                            // 显示左轮PID输出
        tft180_show_string( 0,  5*16,   "errR:");                            // 显示右轮PID输出
        tft180_show_float(  48,  4*16,  left_pid.error,  6, 1);              // 显示 float（总6位，小数1位）
        tft180_show_float(  48,  5*16,  right_pid.error, 6, 1);              // 显示 float（总6位，小数1位）
		
		tft180_show_string( 0,  6*16,   "LerrL:");                            // 显示左轮PID输出
        tft180_show_string( 0,  7*16,   "LerrR:");                            // 显示右轮PID输出
        tft180_show_float(  48,  6*16,  left_pid.lastError,  6, 1);              // 显示 float（总6位，小数1位）
        tft180_show_float(  48,  7*16,  right_pid.lastError, 6, 1);              // 显示 float（总6位，小数1位）
		
        system_delay_ms(100);
        // 此处编写需要循环执行的代码
    }
}

// **************************** 代码区域 ****************************
