#include "zf_common_headfile.h"

// 光电管 最大最小值采集
uint16 gray_max[8] = {0};
uint16 gray_min[8] = {4095,4095,4095,4095,4095,4095,4095,4095};  // 12位ADC默认最大
uint8  gray_threshold = 30;	  // 灰度阈值

// 采集并更新光电管最大最小值
void gray_max_min_update(void)
{
    uint8 i;
    for(i=0; i<8; i++)
    {
        if(gs08ra_raw_val[i] > gray_max[i]) gray_max[i] = gs08ra_raw_val[i];
        if(gs08ra_raw_val[i] < gray_min[i]) gray_min[i] = gs08ra_raw_val[i];
    }
}

// 重置最大最小值
void gray_max_min_reset(void)
{
    uint8 i;
    for(i=0; i<8; i++)
    {
        gray_max[i] = 0;
        gray_min[i] = 4095;
    }
}
// 将采集的最大最小值直接写入 gs08ra_max_val / gs08ra_min_val
void gray_save_max_min_to_array(void)
{
    uint8 i;
    for(i=0; i<8; i++)
    {
        gs08ra_max_val[i] = gray_max[i];  // 写入最大值数组
        gs08ra_min_val[i] = gray_min[i];  // 写入最小值数组
    }
}
