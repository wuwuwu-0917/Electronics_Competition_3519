/*********************************************************************************************************************
 * 文件名称          uart
 * 功能描述          摄像头 UART 通信库
 *                   - 接收 MaixCAM2 发来的钢球检测数据帧（协议见 camera/通信协议文档.md）
 *                   - 帧解析（状态机 + CRC16 校验）
 *                   - 存储多目标位置信息
 *                   - 发送应答帧
 * 使用说明          在 main() 中调用 camera_uart_init() 初始化
 *                   在主循环或 PIT 回调中调用 camera_uart_parse_frame() 处理接收数据
 *                   检查 g_new_frame_ready 标志，消费后手动清零
 * 适用平台          MSPM0G3519
 ********************************************************************************************************************/

#include "global.h"

/*============================ 全局变量定义 ============================*/
volatile bool g_new_frame_ready = false;        // 新帧就绪标志（用户消费后需手动清零）
uint8         g_has_ball  = 0;                  // 0=无球 1=有球
int8          g_ball_zone = 0;                  // 球区域: -2/-1/0/1/2
uint16        g_target_cx = 0;                  // 最近球中心 X 坐标
uint16        g_target_cy = 0;                  // 最近球中心 Y 坐标

/*  PIT 快照变量 — 由 camera_uart_update() 更新，供控制算法使用 */
uint8  g_ball_detect   = 0;
int8   g_ball_zone_val = 0;
uint16 g_ball_x        = 0;
uint16 g_ball_y        = 0;
float  g_camera_turn      = 0.0f;   // 摄像头转向误差（调试用，与灰度 turn_div 对应）
float  g_camera_max_turn  = 5.0f;  // 摄像头巡线时转向输出上限

/*============================ 内部静态变量 ============================*/
// 软件 FIFO 已移除 — ISR 直接逐字节解析，零拷贝零竞争

/* ---- 帧解析状态机 ---- */
// 协议 v3: AA 55 | has_ball(1B) | zone(1B) | cx(2B LE) | cy(2B LE) | CRC16(2B LE) | DD
#define PAYLOAD_LEN  6

typedef enum
{
    WAIT_HEADER_AA,     // 等待帧头第一个字节 0xAA
    WAIT_HEADER_55,     // 等待帧头第二个字节 0x55
    READ_PAYLOAD,       // 读取 6 字节载荷
    READ_CRC,           // 读取 CRC16（2 字节 LE）
    READ_TAIL,          // 读取帧尾 0xDD
} FrameState;

static FrameState   frame_state = WAIT_HEADER_AA;    // 当前状态机状态
static uint8        frame_payload[PAYLOAD_LEN];       // 载荷缓冲区 (has_ball, zone, cxL, cxH, cyL, cyH)
static uint8        frame_payload_idx = 0;            // 载荷读取索引
static uint8        frame_crc_byte = 0;               // CRC 读取计数（0/1）
static uint16       frame_crc_received = 0;           // 接收到的 CRC16 值
static uint16       frame_crc_calc = 0;               // 本地计算的 CRC16 值

/*============================ CRC16 查表 ============================*/
// CRC16-CCITT (XMODEM)，多项式 0x1021
static const uint16 crc16_table[256] =
{
    0x0000, 0x1021, 0x2042, 0x3063, 0x4084, 0x50A5, 0x60C6, 0x70E7,
    0x8108, 0x9129, 0xA14A, 0xB16B, 0xC18C, 0xD1AD, 0xE1CE, 0xF1EF,
    0x1231, 0x0210, 0x3273, 0x2252, 0x52B5, 0x4294, 0x72F7, 0x62D6,
    0x9339, 0x8318, 0xB37B, 0xA35A, 0xD3BD, 0xC39C, 0xF3FF, 0xE3DE,
    0x2462, 0x3443, 0x0420, 0x1401, 0x64E6, 0x74C7, 0x44A4, 0x5485,
    0xA56A, 0xB54B, 0x8528, 0x9509, 0xE5EE, 0xF5CF, 0xC5AC, 0xD58D,
    0x3653, 0x2672, 0x1611, 0x0630, 0x76D7, 0x66F6, 0x5695, 0x46B4,
    0xB75B, 0xA77A, 0x9719, 0x8738, 0xF7DF, 0xE7FE, 0xD79D, 0xC7BC,
    0x48C4, 0x58E5, 0x6886, 0x78A7, 0x0840, 0x1861, 0x2802, 0x3823,
    0xC9CC, 0xD9ED, 0xE98E, 0xF9AF, 0x8948, 0x9969, 0xA90A, 0xB92B,
    0x5AF5, 0x4AD4, 0x7AB7, 0x6A96, 0x1A71, 0x0A50, 0x3A33, 0x2A12,
    0xDBFD, 0xCBDC, 0xFBBF, 0xEB9E, 0x9B79, 0x8B58, 0xBB3B, 0xAB1A,
    0x6CA6, 0x7C87, 0x4CE4, 0x5CC5, 0x2C22, 0x3C03, 0x0C60, 0x1C41,
    0xEDAE, 0xFD8F, 0xCDEC, 0xDDCD, 0xAD2A, 0xBD0B, 0x8D68, 0x9D49,
    0x7E97, 0x6EB6, 0x5ED5, 0x4EF4, 0x3E13, 0x2E32, 0x1E51, 0x0E70,
    0xFF9F, 0xEFBE, 0xDFDD, 0xCFFC, 0xBF1B, 0xAF3A, 0x9F59, 0x8F78,
    0x9188, 0x81A9, 0xB1CA, 0xA1EB, 0xD10C, 0xC12D, 0xF14E, 0xE16F,
    0x1080, 0x00A1, 0x30C2, 0x20E3, 0x5004, 0x4025, 0x7046, 0x6067,
    0x83B9, 0x9398, 0xA3FB, 0xB3DA, 0xC33D, 0xD31C, 0xE37F, 0xF35E,
    0x02B1, 0x1290, 0x22F3, 0x32D2, 0x4235, 0x5214, 0x6277, 0x7256,
    0xB5EA, 0xA5CB, 0x95A8, 0x8589, 0xF56E, 0xE54F, 0xD52C, 0xC50D,
    0x34E2, 0x24C3, 0x14A0, 0x0481, 0x7466, 0x6447, 0x5424, 0x4405,
    0xA7DB, 0xB7FA, 0x8799, 0x97B8, 0xE75F, 0xF77E, 0xC71D, 0xD73C,
    0x26D3, 0x36F2, 0x0691, 0x16B0, 0x6657, 0x7676, 0x4615, 0x5634,
    0xD94C, 0xC96D, 0xF90E, 0xE92F, 0x99C8, 0x89E9, 0xB98A, 0xA9AB,
    0x5844, 0x4865, 0x7806, 0x6827, 0x18C0, 0x08E1, 0x3882, 0x28A3,
    0xCB7D, 0xDB5C, 0xEB3F, 0xFB1E, 0x8BF9, 0x9BD8, 0xABBB, 0xBB9A,
    0x4A75, 0x5A54, 0x6A37, 0x7A16, 0x0AF1, 0x1AD0, 0x2AB3, 0x3A92,
    0xFD2E, 0xED0F, 0xDD6C, 0xCD4D, 0xBDAA, 0xAD8B, 0x9DE8, 0x8DC9,
    0x7C26, 0x6C07, 0x5C64, 0x4C45, 0x3CA2, 0x2C83, 0x1CE0, 0x0CC1,
    0xEF1F, 0xFF3E, 0xCF5D, 0xDF7C, 0xAF9B, 0xBFBA, 0x8FD9, 0x9FF8,
    0x6E17, 0x7E36, 0x4E55, 0x5E74, 0x2E93, 0x3EB2, 0x0ED1, 0x1EF0
};

/*============================ 内部函数声明 ============================*/
static uint16 crc16(const uint8 *data, uint32 len);
static void   parse_targets(const uint8 *data, uint8 count);

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     CRC16-CCITT (XMODEM) 校验计算
// 参数说明     *data               待校验数据首地址
// 参数说明     len                 数据长度（字节）
// 返回参数     uint16              16 位 CRC 校验值
// 使用示例     uint16 crc = crc16(buffer, length);
// 备注信息     多项式 0x1021，初始值 0x0000
//-------------------------------------------------------------------------------------------------------------------
static uint16 crc16(const uint8 *data, uint32 len)
{
    uint16 crc = 0;
    for (uint32 i = 0; i < len; i++)
    {
        crc = (crc << 8) ^ crc16_table[((crc >> 8) ^ data[i]) & 0xFF];
    }
    return crc;
}

// parse_targets() 已移除 — v3 协议无坐标数据段

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     摄像头 UART 初始化
// 参数说明     void
// 返回参数     void
// 使用示例     camera_uart_init();
// 备注信息     初始化 UART7（B15=TX, B16=RX），配置 115200 波特率、接收中断、FIFO 缓冲区
//-------------------------------------------------------------------------------------------------------------------
void camera_uart_init(void)
{
    // 初始化 UART 外设（uart_init 末尾调用 DL_UART_Main_enable 使能了 UART）
    uart_init(CAMERA_UART_INDEX, CAMERA_UART_BAUD, CAMERA_UART_TX_PIN, CAMERA_UART_RX_PIN);

    // ★ 启用 UART FIFO（uart_init → DL_UART_Main_init 会清除 FEN 位，必须重新打开）
    //   DL_UART_enableFIFOs 操作 CTL0 寄存器，文档要求 UART 先禁用再修改再使能
    //   8 字节硬件 FIFO：缓冲窗口 ~87μs → ~696μs，杜绝 overrun
    DL_UART_Main_changeConfig(UART7);       // 禁用 UART + 等待不忙
    DL_UART_Main_enableFIFOs(UART7);        // 置位 FEN（FIFO Enable）
    DL_UART_Main_enable(UART7);             // 重新使能 UART

    // 注册接收中断回调函数
    uart_set_callback(CAMERA_UART_INDEX, camera_uart_callback, NULL);

    // 使能接收中断
    uart_set_interrupt_config(CAMERA_UART_INDEX, UART_INTERRUPT_CONFIG_RX_ENABLE);

    // ★ 同时使能 overrun 错误中断 — 防止 overrun 标志锁死 UART 接收器
    //   如果 overrun 发生且 RX 中断不再触发，OE 标志永远不清 → UART 停止接收
    DL_UART_Main_enableInterrupt(UART7, DL_UART_MAIN_INTERRUPT_OVERRUN_ERROR);

    // ★ 提升 UART7 中断优先级高于 PIT (TIMG0)，确保 PIT ISR 执行期间
    //    UART ISR 仍能抢占并排空硬件 FIFO，防止 overflow 丢字节
    NVIC_SetPriority(UART7_INT_IRQn, 0);   // 最高优先级
    NVIC_SetPriority(TIMG0_INT_IRQn, 1);   // 低于 UART

    // ★ 设置 RX FIFO 阈值为 1 字节（收到就中断），防止低于默认阈值时丢数据
    DL_UART_Main_setRXFIFOThreshold(UART7, DL_UART_MAIN_RX_FIFO_LEVEL_ONE_ENTRY);

    // 状态机复位
    frame_state = WAIT_HEADER_AA;
    frame_payload_idx = 0;
    frame_crc_byte = 0;
    g_has_ball  = 0;
    g_ball_zone = 0;
    g_target_cx = 0;
    g_target_cy = 0;
    g_new_frame_ready = false;
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     UART 接收中断回调函数（由 ISR 中的 uart_callback_list[6] 调用）
// 参数说明     state               中断状态（UART_INTERRUPT_STATE_RX / TX）
// 参数说明     *ptr                 用户自定义指针（未使用）
// 返回参数     void
// 使用示例     由 camera_uart_init() 中的 uart_set_callback() 注册，无需手动调用
// 备注信息     该函数在中断上下文中执行，仅做 FIFO 写入，不做帧解析
//-------------------------------------------------------------------------------------------------------------------
volatile uint32 g_uart_isr_count  = 0;  // 调试：ISR 触发次数
volatile uint32 g_uart_byte_count  = 0;  // 调试：接收字节数
volatile uint32 g_uart_frame_count = 0;  // 调试：成功解析帧数
volatile uint32 g_uart_err_count   = 0;  // 调试：错误中断次数

void camera_uart_callback(uint32 state, void *ptr)
{
    uint8 byte;

    if (UART_INTERRUPT_STATE_RX != state)
        return;

    g_uart_isr_count++;

    // 直接逐字节解析 — 无 FIFO，无竞争
    while (uart_query_byte(CAMERA_UART_INDEX, &byte))
    {
        g_uart_byte_count++;
        switch (frame_state)
        {
            case WAIT_HEADER_AA:
                if (0xAA == byte)
                    frame_state = WAIT_HEADER_55;
                break;

            case WAIT_HEADER_55:
                if (0x55 == byte)
                {
                    frame_state = READ_PAYLOAD;
                    frame_payload_idx = 0;
                }
                else
                {
                    frame_state = WAIT_HEADER_AA;
                    if (0xAA == byte)
                        frame_state = WAIT_HEADER_55;
                }
                break;

            case READ_PAYLOAD:
                frame_payload[frame_payload_idx++] = byte;
                if (frame_payload_idx >= PAYLOAD_LEN)
                {
                    frame_state = READ_CRC;
                    frame_crc_byte = 0;
                }
                break;

            case READ_CRC:
                if (0 == frame_crc_byte)
                {
                    frame_crc_received = byte;
                    frame_crc_byte = 1;
                }
                else
                {
                    frame_crc_received |= ((uint16)byte << 8);
                    frame_crc_byte = 0;
                    frame_state = READ_TAIL;
                }
                break;

            case READ_TAIL:
                if (0xDD == byte)
                {
                    frame_crc_calc = crc16(frame_payload, PAYLOAD_LEN);
                    if (frame_crc_calc == frame_crc_received)
                    {
                        g_has_ball  = frame_payload[0] & 0x01;
                        g_ball_zone = (int8)frame_payload[1];
                        g_target_cx = (uint16)frame_payload[2] | ((uint16)frame_payload[3] << 8);
                        g_target_cy = (uint16)frame_payload[4] | ((uint16)frame_payload[5] << 8);
                        g_new_frame_ready = true;
                        g_uart_frame_count++;
                    }
                }
                frame_state = WAIT_HEADER_AA;
                break;

            default:
                frame_state = WAIT_HEADER_AA;
                break;
        }
    }
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     快照 ISR 更新的数据，供控制算法使用
// 参数说明     void
// 返回参数     void
// 使用示例     在 PIT 回调中调用 camera_uart_update();
// 备注信息     关中断保护读取，防止 UART ISR 在半路改写
//-------------------------------------------------------------------------------------------------------------------
void camera_uart_update(void)
{
    // ★ UART 健康检查：即使 OE 中断已被使能，这里作为二级安全网
    //   直接检查 RIS 中的 overrun 标志，防止任何情况下 UART 接收器锁死
    if (UART7->CPU_INT.RIS & UART_CPU_INT_RIS_OVRERR_MASK)
    {
        while (!DL_UART_isRXFIFOEmpty(UART7))
            DL_UART_Main_receiveData(UART7);
        DL_UART_clearInterruptStatus(UART7, UART_CPU_INT_RIS_OVRERR_MASK);
        g_uart_err_count++;
    }

    __disable_irq();
    g_ball_detect   = g_has_ball;
    g_ball_zone_val = g_ball_zone;
    g_ball_x        = g_target_cx;
    g_ball_y        = g_target_cy;
    __enable_irq();

    // 计算摄像头转向误差（与灰度 turn_div 对称）
    if (!g_ball_detect)
    {
        g_camera_turn = 0.0f;
        return;
    }
    switch (g_ball_zone_val)
    {
        case -2: g_camera_turn =  20.0f; break;
        case -1: g_camera_turn =  10.0f; break;
        case  0: g_camera_turn =   0.0f; break;
        case  1: g_camera_turn = -10.0f; break;
        case  2: g_camera_turn = -20.0f; break;
        default: g_camera_turn =   0.0f; break;
    }
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     发送应答帧给摄像头
// 参数说明     ack_type            应答类型（0x00=OK, 0x01=CRC错误）
// 返回参数     void
// 使用示例     camera_uart_send_response(0x00);
// 备注信息     应答帧格式: AA 55 [ack_type:1B] [count:1B] [CRC16:2B LE] DD（共 7 字节）
//-------------------------------------------------------------------------------------------------------------------
void camera_uart_send_response(uint8 ack_type)
{
    uint8  response[4];   // 载荷: ack_type + count（用于 CRC 计算）
    uint8  packet[7];     // 完整帧
    uint16 crc;

    // 构造载荷
    response[0] = ack_type;
    response[1] = g_has_ball;

    // 计算 CRC
    crc = crc16(response, 2);

    // 组包
    packet[0] = 0xAA;       // 帧头
    packet[1] = 0x55;
    packet[2] = ack_type;   // 应答类型
    packet[3] = g_has_ball; // 目标数量
    packet[4] = (uint8)(crc & 0xFF);        // CRC 低字节 (LE)
    packet[5] = (uint8)((crc >> 8) & 0xFF); // CRC 高字节 (LE)
    packet[6] = 0xDD;       // 帧尾

    // 发送
    uart_write_buffer(CAMERA_UART_INDEX, packet, 7);
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     根据球区域计算转向误差
// 参数说明     void
// 返回参数     float  正=需右转 负=需左转 0=直行
// 使用示例     steering_error = get_camera_deviation();
// 备注信息     无球时返回 0；有球时根据 g_ball_zone_val 映射
//-------------------------------------------------------------------------------------------------------------------
float get_camera_deviation(void)
{
    if (!g_ball_detect)
    {
        g_camera_turn = 0.0f;
        return 0.0f;
    }

    switch (g_ball_zone_val)
    {
        case -2: g_camera_turn =  1.0f; break;
        case -1: g_camera_turn =  0.5f; break;
        case  0: g_camera_turn =   0.0f; break;
        case  1: g_camera_turn = -0.50f; break;
        case  2: g_camera_turn = -1.0f; break;
        default: g_camera_turn =   0.0f; break;
    }
    return g_camera_turn;
}
