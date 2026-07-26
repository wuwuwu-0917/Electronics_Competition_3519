#include "global.h"

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
    get_encoder();                                        // 读取编码器数据
    
    Incremental_PID_Calc(&left_pid, 1, encoder[0]/10);                 // 左电机PID计算（目标值20）
    Incremental_PID_Calc(&right_pid, 1, -encoder[1]/10);                // 右电机PID计算（目标值20）
    motor_set((int8)left_pid.output, (int8)right_pid.output);       // 电机输出

    gpio_toggle_level(A14);              // 翻转引脚电平
}