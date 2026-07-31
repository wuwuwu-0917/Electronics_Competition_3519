#include "zf_common_headfile.h"
#include "global.h"

// 计时变量
extern volatile uint32 g_timer_count;
extern volatile uint8  g_timer_running;
extern uint32 g_timer_result;

// 目标速度变量
extern int8 rpm;

void Key_Command (void)
{
	//---------------------------按键1：下一页 (循环翻页) --------------------
	if(key_get_state(KEY_1) == KEY_SHORT_PRESS)
	{
                tft180_clear();
                Menu = (MenuState)((Menu + 1) % MENU_NUM);
	}

	//---------------------------按键2：电机使能 + 重新发车 (rpm=30) -----------------------------
	if(key_get_state(KEY_2) == KEY_SHORT_PRESS)
	{
                rpm = 30;                     // 设置目标速度为30
                turn_pid.kp = 1.3;            // 设置方向环PID参数
                stop_line_flag    = 0;
                stop_line_count   = 0;
                prev_on_line      = 0;
                motor_enable_flag = 1;

                // 计时复位并启动
                g_timer_count   = 0;
                g_timer_result  = 0;
                g_timer_running = 1;
	}

	//---------------------------按键3：电机使能 + 重新发车 (rpm=20) -----------------------------
	if(key_get_state(KEY_3) == KEY_SHORT_PRESS)
	{
                rpm = 20;                     // 设置目标速度为20
                stop_line_flag    = 0;
                stop_line_count   = 0;
                prev_on_line      = 0;
                motor_enable_flag = 1;

                // 计时复位并启动
                g_timer_count   = 0;
                g_timer_result  = 0;
                g_timer_running = 1;
	}
}
