
// stdafx.cpp : ǥ�� ���� ���ϸ� ��� �ִ� �ҽ� �����Դϴ�.
// BrainStimSerialArm.pch�� �̸� �����ϵ� ����� �˴ϴ�.
// stdafx.obj���� �̸� �����ϵ� ���� ������ ���Ե˴ϴ�.

#define _USE_MATH_DEFINES
#include "stdafx.h"
#include <math.h>


//--------------------------------------------------------------------------//
//------------------------------���� ���� ���------------------------------//

HANDLE MaxonHandle;
double D2R			=	M_PI/180;
double R2D			=	180/M_PI;
double alp			=	50*D2R;			//First link
double beta			=	64.43*D2R;		//Second link
double radius		=	336.14;		//340.44; Radius , 3���࿡�� common������Ÿ� 
double l_link6			=	61;			// �κ� ��ũ ����

//////////////////////////// ������ ///////////////////////////////////////
double l_trnsdcr		=	29.13;		// Ultrasonic Ʈ�����༭ �β�
double l_trnsdcrFix		=	15;			// Ultrasonic Ʈ�����༭ ������ġ �β�
double l_trnsdcrFocal	=	50;			// Ultrasonic Ʈ�����༭ �����Ÿ�
//double l6			=	l_link6 + l_trnsdcr + l_trnsdcrFix + l_trnsdcrFocal; 
///////////////////////////////////////////////////////////////////////////

//////////////////////////// TMS ///////////////////////////////////////
double l_tmsFix = 58;
double l6 = l_link6 + l_tmsFix;
///////////////////////////////////////////////////////////////////////////


int failedNodeId = 0;
NDIData ndidata; // ���� ����ü
Marker marker; // ���� ����ü���� header�� �ص� �Ǹ� �� �κ� �����.

double q0[4],qx[4],qy[4],qz[4],tx[4],ty[4],tz[4];
double Target_q0,Target_qx,Target_qy,Target_qz,Target_tx,Target_ty,Target_tz;

BOOL bFlag[4];
long unsigned lFlag[4]; // NDI Marker FLAG(NDI API ��� ��, FLAG�� long Ÿ��)
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
