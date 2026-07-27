#include "zf_common_headfile.h"
#include "global.h"

void Key_Command (void)
{
	
	//标志位置位之后，可以使用标志位执行自己想要做的事件
//---------------------------按键1-----------------------------
	if(key_get_state(KEY_1) == KEY_SHORT_PRESS)   
	{
        tft180_clear();
        Menu = PID_MENU;
	}

//---------------------------按键2-----------------------------
	if(key_get_state(KEY_2) == KEY_SHORT_PRESS)   
	{
        tft180_clear();
        Menu = GS_MENU;
	}
	
//---------------------------按键3-----------------------------
	if(key_get_state(KEY_3) == KEY_SHORT_PRESS)   
	{
        tft180_clear();
        Menu = IMU_MENU;
	}
	
//---------------------------按键4-----------------------------
	if(key_get_state(KEY_3) == KEY_SHORT_PRESS)   
	{

	}
}