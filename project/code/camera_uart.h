#ifndef _UART_H_
#define _UART_H_

#include "zf_common_headfile.h"

/*============================ 用户配置宏 ============================*/
#define CAMERA_UART_INDEX       UART_7          // 摄像头使用的串口号：UART7
#define CAMERA_UART_BAUD        115200          // 波特率，与 MaixCAM2 一致
#define CAMERA_UART_TX_PIN      UART7_TX_B15    // MCU TX(B15) → 摄像头 RX
#define CAMERA_UART_RX_PIN      UART7_RX_B16    // 摄像头 TX → MCU RX(B16)
                                                //
// 软件 FIFO 已移除，ISR 直接解析

/*============================ ISR 写入的原始变量 ============================*/
extern volatile bool g_new_frame_ready;   // 新帧就绪标志
extern uint8         g_has_ball;          // 0=无球 1=有球
extern int8          g_ball_zone;         // 球区域: -2/-1/0/1/2
extern uint16        g_target_cx;         // 最近球中心 X 坐标（像素）
extern uint16        g_target_cy;         // 最近球中心 Y 坐标（像素）

/*============================ PIT 快照变量（供控制算法使用） ============================*/
extern uint8  g_ball_detect;    // 0/1  有球否
extern int8   g_ball_zone_val;  // -2~2 区域
extern uint16 g_ball_x;         // 最近球 X
extern uint16 g_ball_y;         // 最近球 Y
extern float  g_camera_turn;      // 摄像头转向误差（调试用）
extern float  g_camera_max_turn;  // 摄像头巡线时转向输出上限

extern volatile uint32 g_uart_isr_count;   // 调试：ISR触发次数
extern volatile uint32 g_uart_byte_count;  // 调试：接收字节数
extern volatile uint32 g_uart_frame_count; // 调试：成功解析帧数
extern volatile uint32 g_uart_err_count;   // 调试：错误中断次数

/*============================ 函数声明 ============================*/
void  camera_uart_init(void);
void  camera_uart_callback(uint32 state, void *ptr);
void  camera_uart_update(void);                    // PIT 中调用，快照 ISR 数据
void  camera_uart_send_response(uint8 ack_type);    // 发送应答给 Camera
float get_camera_deviation(void);                  // 由 ball_zone 计算转向误差

#endif
