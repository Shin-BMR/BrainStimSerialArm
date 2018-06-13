
// stdafx.h : ���� ��������� ���� ��������� �ʴ�
// ǥ�� �ý��� ���� ���� �� ������Ʈ ���� ���� ������ 
// ��� �ִ� ���� �����Դϴ�.

#pragma once

#ifndef _SECURE_ATL
#define _SECURE_ATL 1
#endif

#ifndef VC_EXTRALEAN
#define VC_EXTRALEAN            // ���� ������ �ʴ� ������ Windows ������� �����մϴ�.
#endif

#include "targetver.h"

#define _ATL_CSTRING_EXPLICIT_CONSTRUCTORS      // �Ϻ� CString �����ڴ� ��������� ����˴ϴ�.

// MFC�� ���� �κа� ���� ������ ��� �޽����� ���� ����⸦ �����մϴ�.
#define _AFX_ALL_WARNINGS

#include <afxwin.h>         // MFC �ٽ� �� ǥ�� ���� ����Դϴ�.
#include <afxext.h>         // MFC Ȯ���Դϴ�.
#include <afxdisp.h>        // MFC �ڵ�ȭ Ŭ�����Դϴ�.

#ifndef _AFX_NO_OLE_SUPPORT
#include <afxdtctl.h>           // Internet Explorer 4 ���� ��Ʈ�ѿ� ���� MFC �����Դϴ�.
#endif
#ifndef _AFX_NO_AFXCMN_SUPPORT
#include <afxcmn.h>             // Windows ���� ��Ʈ�ѿ� ���� MFC �����Դϴ�.
#endif // _AFX_NO_AFXCMN_SUPPORT

#include <afxcontrolbars.h>     // MFC�� ���� �� ��Ʈ�� ���� ����



//--------------------------------------------------------------------------//
//------------------------------���� ���� ���------------------------------//

extern HANDLE MaxonHandle;//,m_hInputPipeHandle1; // Maxon motor �ڵ� , namedpipe �ڵ�
extern double D2R,R2D,l6,alp,beta,radius,l_link6,l_trnsdcr,l_trnsdcrFix,l_trnsdcrFocal, l_tms; 
extern CCriticalSection g_critical;  // ��Ƽ������ ��� ��, ���ÿ� ���� ������ �����ϴ� ���� ���� ���ؼ�. ���� ��� X 
extern double q0[4],qx[4],qy[4],qz[4],tx[4],ty[4],tz[4]; // NDI Marker
extern double Target_q0,Target_qx,Target_qy,Target_qz,Target_tx,Target_ty,Target_tz; // Navigation���� �Էµ� Ÿ����ǥ
extern double Kp,Ki,Kd,Ki_Z_axis;

extern int timerCount , recordCount;
extern BOOL bFlag[4]; // NDI Marker FLAG
extern BOOL bTargetFlag; // Ÿ�� Flag
extern BOOL bConstraint; // �κ� ���� �� constraint ��� ����.
extern BOOL bInitEncodr; // Ÿ�� Flag
extern unsigned long lFlag[4]; // NDI Marker FLAG(NDI API ��� ��), FLAG�� long Ÿ��

extern int failedNodeId;

struct NDIData
{
	char m_strData[300];
	char m_strA[50];
	bool Targetting;
};
typedef struct _MARKER // NDI API ��� �� ���Ǵ� ����ü
{
	float px[4],py[4],pz[4],qw[4],qx[4],qy[4],qz[4];
	unsigned long flag[4];
}Marker;

//--------------------------------------------------------------------------//
//--------------------------------------------------------------------------//


#ifdef _UNICODE
#if defined _M_IX86
#pragma comment(linker,"/manifestdependency:\"type='win32' name='Microsoft.Windows.Common-Controls' version='6.0.0.0' processorArchitecture='x86' publicKeyToken='6595b64144ccf1df' language='*'\"")
#elif defined _M_X64
#pragma comment(linker,"/manifestdependency:\"type='win32' name='Microsoft.Windows.Common-Controls' version='6.0.0.0' processorArchitecture='amd64' publicKeyToken='6595b64144ccf1df' language='*'\"")
#else
#pragma comment(linker,"/manifestdependency:\"type='win32' name='Microsoft.Windows.Common-Controls' version='6.0.0.0' processorArchitecture='*' publicKeyToken='6595b64144ccf1df' language='*'\"")
#endif
#endif

//------------------------------------//
//--------����� �� CMDâ �̿�--------//
#ifdef _DEBUG
#pragma comment(linker, "/entry:WinMainCRTStartup /subsystem:console")
#endif
//-----------------------------------//
//-----------------------------------//
