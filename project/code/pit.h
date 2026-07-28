#ifndef _PIT_H_
#define _PIT_H_

#include "global.h"

extern int8 rpm;

void pit_callback(uint32 event, void *ptr);

#endif
