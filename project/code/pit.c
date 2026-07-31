#include "global.h"

int8 rpm = 0;  // 目标速度变量（单位：rpm，正数为前进，负数为后退）

// 计时变量
volatile uint32 g_timer_count   = 0;
volatile uint8  g_timer_running = 0;
uint32 g_timer_result = 0;

/*********************************************************************************************************************
 * 函数简介     PIT 周期中断回调函数
 * 参数说明     event               中断事件（保留参数，暂未使用）
 *             *ptr                 回调参数指针（保留参数，暂未使用）
 * 返回参数     void
 * 使用示例     在 main 中通过 pit_ms_init 注册该回调
 * 备注信息     每个周期执行：读取编码器 → PID 计算 → 设置电机 PWM
 ********************************************************************************************************************/
void pit_callback(uint32 event, void *ptr)
{
    get_deviation();        //读取灰度传感器数据计算误差

    // ==== 停车线检测（仅电机使能且未停车时）====
    if (motor_enable_flag && !stop_line_flag)
    {
        int black_cnt = 0;
        for (uint8 i = 0; i < 6; i++)
        {
            if (gs08ra_gpio_val[i] == 0) 
                black_cnt++;
        }

        // 判断当前是否在停车线上（6个传感器中至少3个为黑）
        uint8 on_line = (black_cnt >= 3) ? 1 : 0;

        // 上升沿检测：之前不在线上 → 现在在线上，计一次
        if (on_line && !prev_on_line)
        {
            stop_line_count++;
        }
        prev_on_line = on_line;

        // 检测到第2条停车线 → 停车 + 冻结计时
        if (stop_line_count >= 2)
        {
            stop_line_flag = 1;
            g_timer_running = 0;              // 停止计时
            g_timer_result = g_timer_count;    // 冻结最终时间
        }
    }

    get_encoder();                                        // 读取编码器数据

    // ==== 计时器（运行时才累加）====
    if (g_timer_running && !stop_line_flag)
    {
        g_timer_count++;  // 每10ms +1
    }

    // 摄像头数据快照 → g_ball_detect / g_ball_zone_val / g_ball_x / g_ball_y
    camera_uart_update();

    // 转向误差源切换：有球→摄像头巡线，无球→灰度巡线
    float steering_error;
    if (g_ball_detect)
    {
        steering_error = g_camera_turn;
    }
    else
    {
        steering_error = turn_div;
    }
    
    Positional_PID_Calc(&turn_pid, 0.0, steering_error);

    // 摄像头巡线时额外限制转向输出幅度
    if (g_ball_detect)
    {
        if (turn_pid.output >  g_camera_max_turn)  turn_pid.output =  g_camera_max_turn;
        if (turn_pid.output < -g_camera_max_turn)  turn_pid.output = -g_camera_max_turn;
    }

    // 方向环输出经 5 次滑动平均滤波后，再叠加到电机目标值
    float turn_filtered = MovingAverage_Calc(&turn_filter, turn_pid.output);

    Incremental_PID_Calc(&left_pid, rpm + (turn_filtered),-encoder[0]);                 // 左电机PID计算（目标值20）
    Incremental_PID_Calc(&right_pid, rpm +(-turn_filtered), encoder[1]);                // 右电机PID计算（目标值20）
//	  Incremental_PID_Calc(&left_pid, 10 , -encoder[0]);                 // 左电机PID计算（目标值20）
//    Incremental_PID_Calc(&right_pid, 10 , encoder[1]);                // 右电机PID计算（目标值20）
    // 停车线标志有效时强制停电机，否则正常输出
    if (stop_line_flag)
        motor_set(0, 0);
    else
        motor_set((int8)left_pid.output, (int8)right_pid.output);       // 电机输出

    // gpio_toggle_level(A14);              // 翻转引脚电平（测试用，暂屏蔽）
}