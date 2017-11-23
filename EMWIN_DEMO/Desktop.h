#ifndef _DESKTOP_H
#define _DESKTOP_H
#include "WM.h"
#include "DIALOG.h"

WM_HWIN ExecDesktop(void);
WM_HWIN CreateDesktop(void);
void Desktop_Init(void);
void InitDesktop(WM_MESSAGE * pMsg);
void InitIconview(WM_MESSAGE *pMsg);
#endif


