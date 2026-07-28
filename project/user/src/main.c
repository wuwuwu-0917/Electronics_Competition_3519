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

    // 摄像头UART初始化（ISR 直接解析，FIFO阈值=1字节，优先级最高）
    camera_uart_init();

    // 板载亮灯
    gpio_init(A14, GPO, 0, GPO_PUSH_PULL);
    gpio_set_level(A14, 0);

    // 初始化PID参数    
    Positional_PID_Init(&turn_pid, 1, 0, 0, 100);
    Incremental_PID_Init(&left_pid, 7.0, 0.2, 0, 100);        // 初始化左电机PID参数
    Incremental_PID_Init(&right_pid, 7.0, 0.2, 0, 100);       // 初始化右电机PID参数
  
    //初始化IMU66RC并开启外部中断触发计算四元数欧拉角
    while(1)        
    {
        if(imu660rc_init(IMU660RC_QUARTERNION_120HZ))														// 设置 IMU660RC 以120HZ的速度产生中断触发信号
        {
            printf("\r\nIMU660RC init error.");                                 // IMU660RC 初始化失败
        }
        else
        {
            break;
        }
        gpio_toggle_level(A14);                                                // 翻转 LED 引脚输出电平 控制 LED 亮灭 初始化出错这个灯会闪的很慢
    }

    pit_ms_init(PIT_TIM_G0, 10, pit_callback, NULL);

    interrupt_global_enable(0);

    while(true)
    {
		key_scanner();
		Key_Command();
		

        static uint8 menu_tick = 0;
			if (++menu_tick >= 10) { menu_tick = 0; Show_Menu(); }

	        system_delay_ms(5);
	}
}

// **************************** 代码区域 ****************************
