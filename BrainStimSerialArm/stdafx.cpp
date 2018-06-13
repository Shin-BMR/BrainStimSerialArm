
// stdafx.cpp : 표준 포함 파일만 들어 있는 소스 파일입니다.
// BrainStimSerialArm.pch는 미리 컴파일된 헤더가 됩니다.
// stdafx.obj에는 미리 컴파일된 형식 정보가 포함됩니다.

#define _USE_MATH_DEFINES
#include "stdafx.h"
#include <math.h>


//--------------------------------------------------------------------------//
//------------------------------전역 변수 사용------------------------------//

HANDLE MaxonHandle;
double D2R			=	M_PI/180;
double R2D			=	180/M_PI;
double alp			=	50*D2R;			//First link
double beta			=	64.43*D2R;		//Second link
double radius		=	336.14;		//340.44; Radius , 3번축에서 common축까지거리 
double l_link6			=	61;			// 로봇 링크 길이

//////////////////////////// 초음파 ///////////////////////////////////////
double l_trnsdcr		=	29.13;		// Ultrasonic 트랜스듀서 두께
double l_trnsdcrFix		=	15;			// Ultrasonic 트랜스듀서 고정장치 두께
double l_trnsdcrFocal	=	50;			// Ultrasonic 트랜스듀서 초점거리
//double l6			=	l_link6 + l_trnsdcr + l_trnsdcrFix + l_trnsdcrFocal; 
///////////////////////////////////////////////////////////////////////////

//////////////////////////// TMS ///////////////////////////////////////
double l_tmsFix = 58;
double l6 = l_link6 + l_tmsFix;
///////////////////////////////////////////////////////////////////////////


int failedNodeId = 0;
NDIData ndidata; // 전역 구조체
Marker marker; // 전역 구조체쓸때 header만 해도 되면 이 부분 지우기.

double q0[4],qx[4],qy[4],qz[4],tx[4],ty[4],tz[4];
double Target_q0,Target_qx,Target_qy,Target_qz,Target_tx,Target_ty,Target_tz;

BOOL bFlag[4];
long unsigned lFlag[4]; // NDI Marker FLAG(NDI API 사용 시, FLAG가 long 타입)
BOOL bTargetFlag;
BOOL bConstraint;
BOOL bInitEncodr;

int timerCount = 0;
int recordCount = 0;
double Kp = 1.0;
double Ki = 0.3;
double Ki_Z_axis = Ki*1.1;
double Kd = 0.001;
CCriticalSection g_critical;

//--------------------------------------------------------------------------//
//--------------------------------------------------------------------------//
