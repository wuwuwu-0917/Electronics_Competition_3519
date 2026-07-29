/*********************************************************************************************************************
* MSPM0G3519 Opensource Library 即（MSPM0G3519 开源库）是一个基于官方 SDK 接口的第三方开源库
* Copyright (c) 2026 SEEKFREE 逐飞科技
* 
* 本文件是 MSPM0G3519 开源库的一部分
* 
* MSPM0G3519 开源库 是免费软件
* 您可以根据自由软件基金会发布的 GPL（GNU General Public License，即 GNU通用公共许可证）的条款
* 即 GPL 的第3版（即 GPL3.0）或（您选择的）任何后来的版本，重新发布和/或修改它
* 
* 本开源库的发布是希望它能发挥作用，但并未对其作任何的保证
* 甚至没有隐含的适销性或适合特定用途的保证
* 更多细节请参见 GPL
* 
* 您应该在收到本开源库的同时收到一份 GPL 的副本
* 如果没有，请参阅<https://www.gnu.org/licenses/>
* 
* 额外注明：
* 本开源库使用 GPL3.0 开源许可证协议 以上许可申明为译文版本
* 许可申明英文版在 libraries/doc 文件夹下的 GPL3_permission_statement.txt 文件中
* 许可证副本在 libraries 文件夹下 即该文件夹下的 LICENSE 文件
* 欢迎各位使用并传播本程序 但修改内容时必须保留逐飞科技的版权声明（即本声明）
* 
* 文件名称          zf_device_gs08ra
* 公司名称          成都逐飞科技有限公司
* 版本信息          查看 libraries/doc 文件夹内 version 文件版本说明
* 开发环境          MDK 5.38
* 适用平台          MSPM0G3519
* 店铺链接          https://seekfree.taobao.com/
* 
* 修改记录
* 日期              作者                备注
* 2025-06-1        SeekFree            first version
********************************************************************************************************************/
/*********************************************************************************************************************
* 接线定义：
*                   ------------------------------------
*                   模块管脚        单片机管脚
*                   S0              查看 zf_device_gs08ra.h 中 GS08RA_S0_PIN
*                   S1              查看 zf_device_gs08ra.h 中 GS08RA_S1_PIN
*                   S2              查看 zf_device_gs08ra.h 中 GS08RA_S2_PIN
*                   S3              不需要连接，悬空即可
*                   OUT             查看 zf_device_gs08ra.h 中 GS08RA_OUT_PIN
*                   3V3             3.3V电源
*                   GND             电源地
*                   ------------------------------------
********************************************************************************************************************/

#include "zf_common_function.h"
#include "zf_driver_adc.h"

#include "zf_device_gs08ra.h"

#include "global.h"


uint8  gs08ra_threshold  = 65;   // 用于二值化的阈值
uint16 gs08ra_max_val [GS08A_CHANNEL_NUM] = {3500, 3500, 3500, 3500, 3500, 3500};    // 最大值
uint16 gs08ra_min_val [GS08A_CHANNEL_NUM] = {500, 500, 500, 500, 500, 500};          // 最小值
uint16 gs08ra_raw_val [GS08A_CHANNEL_NUM];                                            // 原始灰度数据
uint8  gs08ra_deal_val[GS08A_CHANNEL_NUM];                                            // 归一化处理后的数据
uint8  gs08ra_bin_val [GS08A_CHANNEL_NUM];                                            // 使用归一化之后的数据进行二值化
uint8  gs08ra_gpio_val[GS08A_CHANNEL_NUM];                                           // GPIO读取的数字量数据

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     按键状态扫描（已废弃 - 现使用独立ADC引脚直读，无需复用器切换）
// 参数说明     channel     通道值 0 - 7
// 返回参数     void
// 使用示例     gs08ra_set_channel(0);
// 备注信息     内部使用
//-------------------------------------------------------------------------------------------------------------------
// static void gs08ra_set_channel(uint8 channel)
// {
//     zf_assert(GS08A_CHANNEL_NUM > channel);
//     gpio_set_level(ADC_CHANNEL4, 0x01 & channel);
//     gpio_set_level(ADC_CHANNEL5, 0x02 & channel);
//     gpio_set_level(ADC_CHANNEL6, 0x04 & channel);
// }

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     灰度传感器归一化
// 参数说明     void
// 返回参数     void
// 使用示例     gs08ra_normalize();
// 备注信息     内部使用
//-------------------------------------------------------------------------------------------------------------------
static void gs08ra_normalize(void)
{
    uint8 i = 0;
    int32 val;                                                          // int16 改 int32，防止 12位ADC值*100 溢出
	do
    {
        val = gs08ra_raw_val[i] - gs08ra_min_val[i];                // 减去最小值
        val = val * 100 / (gs08ra_max_val[i] - gs08ra_min_val[i]);  // 缩放到 0~100
		val = func_limit_ab(val, 0, 100);
        gs08ra_deal_val[i] = (uint8)val;
    }while(GS08A_CHANNEL_NUM > ++i);
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     灰度传感器二值化
// 参数说明     void
// 返回参数     void
// 使用示例     gs08ra_binaryzation();
// 备注信息     内部使用
//-------------------------------------------------------------------------------------------------------------------
static void gs08ra_binaryzation(void)
{
    uint8 i = 0;
    int16 val;
	do
    {
        if(gs08ra_threshold < gs08ra_deal_val[i])
        {
            gs08ra_bin_val[i] = 1;  // 白色
        }
        else
        {
            gs08ra_bin_val[i] = 0;  // 黑色
        }
    }while(GS08A_CHANNEL_NUM > ++i);
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     灰度传感器设置最大值
// 参数说明     void
// 返回参数     void
// 使用示例     gs08ra_set_max();
// 备注信息     将当前的灰度数据设置为最大值
//-------------------------------------------------------------------------------------------------------------------
void gs08ra_set_max(void)
{
    memcpy(gs08ra_max_val, gs08ra_raw_val, sizeof(gs08ra_raw_val));
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     灰度传感器设置最小值
// 参数说明     void
// 返回参数     void
// 使用示例     gs08ra_set_min();
// 备注信息     将当前的灰度数据设置为最小值
//-------------------------------------------------------------------------------------------------------------------
void gs08ra_set_min(void)
{
    memcpy(gs08ra_min_val, gs08ra_raw_val, sizeof(gs08ra_raw_val));
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     灰度传感器设置二值化的阈值
// 参数说明     void
// 返回参数     void
// 使用示例     gs08ra_set_threshold(50); // 阈值设置为50
// 备注信息     
//-------------------------------------------------------------------------------------------------------------------
void gs08ra_set_threshold(uint8 threshold)
{
    gs08ra_threshold = threshold;
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     灰度传感器数据读取
// 参数说明     void
// 返回参数     void
// 使用示例     gs08ra_scan_read();
// 备注信息     获取灰度传感器数据之后进行归一化和二值化处理
//-------------------------------------------------------------------------------------------------------------------
void gs08ra_scan_read(void)
{
//    uint8 i = 0;
//    const uint16 adc_max[] = {4095, 1023, 255};
//    // do
//    // {
//    //     gs08ra_set_channel(i);  // 切换通道
//    //     gs08ra_raw_val[i] = adc_max[GS08RA_ADC_RESLUTION] - adc_mean_filter_convert(GS08RA_OUT_PIN, 5);
//    // }while(GS08A_CHANNEL_NUM > ++i);

//    gs08ra_raw_val[0] = adc_max[ADC_12BIT] - adc_mean_filter_convert(LS_CHANNEL1, 5);
//    gs08ra_raw_val[1] = adc_max[ADC_12BIT] - adc_mean_filter_convert(LS_CHANNEL2, 5);
//    gs08ra_raw_val[2] = adc_max[ADC_12BIT] - adc_mean_filter_convert(LS_CHANNEL3, 5);
//    gs08ra_raw_val[3] = adc_max[ADC_12BIT] - adc_mean_filter_convert(LS_CHANNEL4, 5);
//    gs08ra_raw_val[4] = adc_max[ADC_12BIT] - adc_mean_filter_convert(LS_CHANNEL5, 5);
//    gs08ra_raw_val[5] = adc_max[ADC_12BIT] - adc_mean_filter_convert(LS_CHANNEL6, 5);

//    // 对数据进行归一化处理
//    gs08ra_normalize();
//    
//    // 对数据进行二值化
//    gs08ra_binaryzation();
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     灰度传感器数据读取
// 参数说明     void
// 返回参数     void
// 使用示例     gs08ra_gpio_read();
// 备注信息     获取灰度传感器数据数字量
//-------------------------------------------------------------------------------------------------------------------
void gs08ra_gpio_read(void)
{
    // do
    // {
    //     gs08ra_set_channel(i);  // 切换通道
    //     gs08ra_bin_val[i] = gpio_get_level(GS08RA_OUT_PIN);
    // }while(GS08A_CHANNEL_NUM > ++i);

    gs08ra_gpio_val[0] = !gpio_get_level(LS_CHANNEL1);
    gs08ra_gpio_val[1] = !gpio_get_level(LS_CHANNEL2);
    gs08ra_gpio_val[2] = !gpio_get_level(LS_CHANNEL3);
    gs08ra_gpio_val[3] = !gpio_get_level(LS_CHANNEL4);
    gs08ra_gpio_val[4] = !gpio_get_level(LS_CHANNEL5);
    gs08ra_gpio_val[5] = !gpio_get_level(LS_CHANNEL6);
}




//-------------------------------------------------------------------------------------------------------------------
// 函数简介     灰度传感器初始化
// 参数说明     void
// 返回参数     void
// 使用示例     gs08ra_init();
// 备注信息     
//-------------------------------------------------------------------------------------------------------------------
void gs08ra_init(void)
{
    // 初始化6个独立的ADC引脚（跳过原复用器模式）
    // adc_init(LS_CHANNEL1, ADC_12BIT);
    // adc_init(LS_CHANNEL2, ADC_12BIT);
    // adc_init(LS_CHANNEL3, ADC_12BIT);
    // adc_init(LS_CHANNEL4, ADC_12BIT);
    // adc_init(LS_CHANNEL5, ADC_12BIT);
    // adc_init(LS_CHANNEL6, ADC_12BIT);
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     灰度传感器数字量初始化
// 参数说明     void
// 返回参数     void
// 使用示例     gs08ra_gpio_init();
// 备注信息     
//-------------------------------------------------------------------------------------------------------------------
void gs08ra_gpio_init(void)
{
    // 初始化6个独立的ADC引脚（跳过原复用器模式）
    gpio_init(LS_CHANNEL1, GPI, 0, GPI_PULL_UP);
    gpio_init(LS_CHANNEL2, GPI, 0, GPI_PULL_UP);
    gpio_init(LS_CHANNEL3, GPI, 0, GPI_PULL_UP);
    gpio_init(LS_CHANNEL4, GPI, 0, GPI_PULL_UP);
    gpio_init(LS_CHANNEL5, GPI, 0, GPI_PULL_UP);
    gpio_init(LS_CHANNEL6, GPI, 0, GPI_PULL_UP);
}





