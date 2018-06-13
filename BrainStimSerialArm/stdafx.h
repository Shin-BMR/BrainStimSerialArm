
// stdafx.h : 자주 사용하지만 자주 변경되지는 않는
// 표준 시스템 포함 파일 및 프로젝트 관련 포함 파일이 
// 들어 있는 포함 파일입니다.

#pragma once

#ifndef _SECURE_ATL
#define _SECURE_ATL 1
#endif

#ifndef VC_EXTRALEAN
#define VC_EXTRALEAN            // 거의 사용되지 않는 내용은 Windows 헤더에서 제외합니다.
#endif

#include "targetver.h"

#define _ATL_CSTRING_EXPLICIT_CONSTRUCTORS      // 일부 CString 생성자는 명시적으로 선언됩니다.

// MFC의 공통 부분과 무시 가능한 경고 메시지에 대한 숨기기를 해제합니다.
#define _AFX_ALL_WARNINGS

#include <afxwin.h>         // MFC 핵심 및 표준 구성 요소입니다.
#include <afxext.h>         // MFC 확장입니다.
#include <afxdisp.h>        // MFC 자동화 클래스입니다.

#ifndef _AFX_NO_OLE_SUPPORT
#include <afxdtctl.h>           // Internet Explorer 4 공용 컨트롤에 대한 MFC 지원입니다.
#endif
#ifndef _AFX_NO_AFXCMN_SUPPORT
#include <afxcmn.h>             // Windows 공용 컨트롤에 대한 MFC 지원입니다.
#endif // _AFX_NO_AFXCMN_SUPPORT

#include <afxcontrolbars.h>     // MFC의 리본 및 컨트롤 막대 지원



//--------------------------------------------------------------------------//
//------------------------------전역 변수 사용------------------------------//

extern HANDLE MaxonHandle;//,m_hInputPipeHandle1; // Maxon motor 핸들 , namedpipe 핸들
extern double D2R,R2D,l6,alp,beta,radius,l_link6,l_trnsdcr,l_trnsdcrFix,l_trnsdcrFocal, l_tms; 
extern CCriticalSection g_critical;  // 멀티쓰레드 사용 시, 동시에 같은 변수에 접근하는 것을 막기 위해서. 아직 사용 X 
extern double q0[4],qx[4],qy[4],qz[4],tx[4],ty[4],tz[4]; // NDI Marker
extern double Target_q0,Target_qx,Target_qy,Target_qz,Target_tx,Target_ty,Target_tz; // Navigation에서 입력된 타겟좌표
extern double Kp,Ki,Kd,Ki_Z_axis;

extern int timerCount , recordCount;
extern BOOL bFlag[4]; // NDI Marker FLAG
extern BOOL bTargetFlag; // 타겟 Flag
extern BOOL bConstraint; // 로봇 구동 시 constraint 모드 설정.
extern BOOL bInitEncodr; // 타겟 Flag
extern unsigned long lFlag[4]; // NDI Marker FLAG(NDI API 사용 시), FLAG가 long 타입

extern int failedNodeId;

struct NDIData
{
	char m_strData[300];
	char m_strA[50];
	bool Targetting;
};
typedef struct _MARKER // NDI API 사용 시 사용되는 구조체
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
//--------디버깅 시 CMD창 이용--------//
#ifdef _DEBUG
#pragma comment(linker, "/entry:WinMainCRTStartup /subsystem:console")
#endif
//-----------------------------------//
//-----------------------------------//
