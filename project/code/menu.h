#ifndef __MENU_H_
#define __MENU_H_

#include "zf_common_typedef.h"

typedef enum
{
	PID_MENU,
	GS_MENU,
	IMU_MENU,
}MenuState;

extern  MenuState Menu;

void Show_Menu (void);


#endif