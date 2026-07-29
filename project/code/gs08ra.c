#include "zf_common_headfile.h"
#include "global.h"

// 光电管 最大最小值采集
uint16 gray_max[8] = {0};
uint16 gray_min[8] = {4095,4095,4095,4095,4095,4095,4095,4095};  // 12位ADC默认最大
uint8  gray_threshold = 65;	  // 灰度阈值

float deviation[8] = {0};   //误差赋值
float turn_div = 0;              //总误差

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

void get_deviation(void)
{
    gs08ra_scan_read();          //读取灰度传感器数据

    deviation[0] =  12*(!gs08ra_bin_val[0]);     //黑为0，白为1，左负右正
    deviation[1] =  8*(!gs08ra_bin_val[1]);
    deviation[2] =   3*(!gs08ra_bin_val[2]);
    deviation[3] =   1*(!gs08ra_bin_val[3]);
    deviation[4] =  -1*(!gs08ra_bin_val[4]);
    deviation[5] =  -3*(!gs08ra_bin_val[5]);
    deviation[6] = -8*(!gs08ra_bin_val[6]);
    deviation[7] = -13*(!gs08ra_bin_val[7]);
    
    turn_div = 0;    //误差清零
    for (uint8 i = 0; i < 8; i++)
    {
        turn_div += deviation[i];        //误差加和
    }
    
}