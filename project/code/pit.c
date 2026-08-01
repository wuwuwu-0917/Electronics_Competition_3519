#include "global.h"

#define START_RAMP_CYCLES   600     // 慢速启动总周期数（每周期10ms）→ 约6秒缓加
#define BRAKE_RPM_STEP      0.02f   // 缓停时目标速度每周期递减量(rpm)：恒定小减速率，不设停车时间
                                    // 越小越平缓：0.02≈10秒停稳，0.05≈4秒，0.01≈20秒

int8 rpm = 0;  // 当前目标速度变量（单位：rpm，正数为前进，负数为后退）
int8 rpm_target = 0;        // 最终目标速度（按键设定；慢速模式启动时从0缓慢加到该值）
uint16 start_cnt = 0;       // 慢速启动进度计数（每10ms/周期 +1）
float brake_rpm = 0.0f;     // 缓停参考速度（从目标值按固定小步长递减到0）

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

        // 上升沿检测：第一次进入停车线 → 立即停车 + 冻结计时
        if (on_line && !prev_on_line)
        {
            stop_line_flag   = 1;
            brake_rpm        = (float)rpm_target;   // 缓停参考从目标速度开始递减
            g_timer_running  = 0;                   // 停止计时
            g_timer_result   = g_timer_count;       // 冻结最终时间
        }
        prev_on_line = on_line;
    }

    get_encoder();                                        // 读取编码器数据

    // ==== 计时器（运行时才累加）====
    if (g_timer_running && !stop_line_flag)
    {
        g_timer_count++;  // 每10ms +1
    }

    // ==== 慢速启动：仅慢速模式(rpm_target=25)从START_RPM按S曲线平滑加速到目标 ====
    if ((rpm_target == 25) && motor_enable_flag && !stop_line_flag && (start_cnt < START_RAMP_CYCLES))
    {
        start_cnt++;
        float t = (float)start_cnt / (float)START_RAMP_CYCLES;
        // 与停车一致的 smoothstep S曲线（两端斜率0，平缓无突变），从 START_RPM 缓加到目标
        rpm = (int8)((float)START_RPM + (float)(rpm_target - START_RPM) * (t * t * (3.0f - 2.0f * t)));
    }

    // // 摄像头数据快照 → g_ball_detect / g_ball_zone_val / g_ball_x / g_ball_y
    // camera_uart_update();

    // // 转向误差源切换：有球→摄像头巡线，无球→灰度巡线
    // float steering_error;
    // if (g_ball_detect)
    // {
    //     steering_error = g_camera_turn;
    // }
    // else
    // {
    //     steering_error = turn_div;
    // }
    
    // ==== 按下按键2/3（motor_enable_flag=1）后才开始PID运算 ====
    if (motor_enable_flag)
    {
        Positional_PID_Calc(&turn_pid, 0.0, turn_div);  // 方向环 PID 计算（目标值0，误差为巡线偏差）

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
    }
    // 停车线标志有效时：慢速模式(rpm_target=25)恒定小减速率缓停（时间不设限），快速模式直接停车
    if (stop_line_flag)
    {
        if (rpm_target == 25)                             // 仅慢速模式缓慢停车
        {
            if (brake_rpm > 0.0f)
            {
                brake_rpm -= BRAKE_RPM_STEP;              // 恒定小步长递减 → 平缓刹车
                if (brake_rpm < 0.0f)
                    brake_rpm = 0.0f;
                rpm = (int8)brake_rpm;                    // 速度环跟随，PWM单调下降
                motor_set((int8)left_pid.output, (int8)right_pid.output);   // 正常走PID
            }
            else
            {
                rpm = 0;
                motor_set(0, 0);                          // 降到0，彻底断电防爬行
            }
        }
        else
        {
            motor_set(0, 0);                              // 快速模式直接停车
        }
    }
    else
        motor_set((int8)left_pid.output, (int8)right_pid.output);       // 电机输出

    // gpio_toggle_level(A14);              // 翻转引脚电平（测试用，暂屏蔽）
}