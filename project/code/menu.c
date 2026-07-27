/*=====================头文件=====================*/
#include "menu.h"
#include "zf_common_headfile.h"
/*====================宏定义=====================*/

/*====================全局变量====================*/
MenuState Menu = PID_MENU;

/*====================外部变量====================*/

/*====================主体函数====================*/

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     菜单展示画面
// 参数说明     void
// 返回参数     void
// 使用示例     Show_Menu();
//-------------------------------------------------------------------------------------------------------------------
void Show_Menu (void)
{
	switch (Menu){
		
		//显示灰度参数
		case GS_MENU:
            tft180_show_string( 0,  16*0,   "XUNJI"); 

            tft180_show_uint(    0*8,  2*16,  gs08ra_bin_val[0],          1);
            tft180_show_uint(    1*8,  2*16,  gs08ra_bin_val[1],          1);
            tft180_show_uint(    2*8,  2*16,  gs08ra_bin_val[2],          1);
            tft180_show_uint(    3*8,  2*16,  gs08ra_bin_val[3],          1);
            tft180_show_uint(    4*8,  2*16,  gs08ra_bin_val[4],          1);
            tft180_show_uint(    5*8,  2*16,  gs08ra_bin_val[5],          1);
            tft180_show_uint(    6*8,  2*16,  gs08ra_bin_val[6],          1);
            tft180_show_uint(    7*8,  2*16,  gs08ra_bin_val[7],          1);
		
		break;
		
		//显示PID数据
		case PID_MENU:

            tft180_show_string( 0,  0*16,   "en_le:");                            // 显示左编码器
            tft180_show_string( 0,  1*16,   "en_ri:");                            // 显示右编码器
            tft180_show_int(    64,  0*16,  encoder[0],          4);              // 显示 int16 数据
            tft180_show_int(    64,  1*16,  encoder[1],          4);              // 显示 int16 数据

            tft180_show_string( 0,  2*16,   "pid_l:");                            // 显示左轮PID输出
            tft180_show_string( 0,  3*16,   "pid_r:");                            // 显示右轮PID输出
            tft180_show_float(  48,  2*16,  left_pid.output,  6, 1);              // 显示 float（总6位，小数1位）
            tft180_show_float(  48,  3*16,  right_pid.output, 6, 1);              // 显示 float（总6位，小数1位）
                  
            tft180_show_string( 0,  4*16,   "errL:");                            // 显示左轮PID输出
            tft180_show_string( 0,  5*16,   "errR:");                            // 显示右轮PID输出
            tft180_show_float(  48,  4*16,  left_pid.error,  6, 1);              // 显示 float（总6位，小数1位）
            tft180_show_float(  48,  5*16,  right_pid.error, 6, 1);              // 显示 float（总6位，小数1位）
                  
            tft180_show_string( 0,  6*16,   "LerrL:");                            // 显示左轮PID输出
            tft180_show_string( 0,  7*16,   "LerrR:");                            // 显示右轮PID输出
            tft180_show_float(  48,  6*16,  left_pid.lastError,  6, 1);              // 显示 float（总6位，小数1位）
            tft180_show_float(  48,  7*16,  right_pid.lastError, 6, 1);              // 显示 float（总6位，小数1位）

            tft180_show_string( 0,  8*16,   "Tur_er:");                            // 显示左轮PID输出
            tft180_show_float(  48,  8*16,  turn_pid.error,  6, 1);              // 显示 float（总6位，小数1位）
            tft180_show_string( 0,  9*16,   "Tur_out:");                            // 显示左轮PID输出
            tft180_show_float(  48,  9*16,  turn_pid.output,  6, 1);              // 显示 float（总6位，小数1位）
            
		break;
		
		//显示angle数据
		case IMU_MENU:

		break;
			
	}
}
