#include "zf_common_headfile.h"
#include "global.h"

void Key_Command (void)
{
	//---------------------------按键1：PID菜单 -----------------------------
	if(key_get_state(KEY_1) == KEY_SHORT_PRESS)
	{
        tft180_clear();
        Menu = PID_MENU;
	}

	//---------------------------按键2：灰度菜单 -----------------------------
	if(key_get_state(KEY_2) == KEY_SHORT_PRESS)
	{
        tft180_clear();
        Menu = GS_MENU;
	}

	//---------------------------按键3：IMU菜单 -----------------------------
	if(key_get_state(KEY_3) == KEY_SHORT_PRESS)
	{
        tft180_clear();
        Menu = IMU_MENU;
	}

	//---------------------------按键4：摄像头菜单 ---------------------------
	if(key_get_state(KEY_4) == KEY_SHORT_PRESS)
	{
        tft180_clear();
        Menu = CAM_MENU;
	}
}
