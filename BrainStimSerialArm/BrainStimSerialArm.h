
// BrainStimSerialArm.h : PROJECT_NAME ���� ���α׷��� ���� �� ��� �����Դϴ�.
//

#pragma once

#ifndef __AFXWIN_H__
	#error "PCH�� ���� �� ������ �����ϱ� ���� 'stdafx.h'�� �����մϴ�."
#endif

#include "resource.h"		// �� ��ȣ�Դϴ�.


// CBrainStimSerialArmApp:
// �� Ŭ������ ������ ���ؼ��� BrainStimSerialArm.cpp�� �����Ͻʽÿ�.
//

class CBrainStimSerialArmApp : public CWinApp
{
public:
	CBrainStimSerialArmApp();

// �������Դϴ�.
public:
	virtual BOOL InitInstance();

// �����Դϴ�.

	DECLARE_MESSAGE_MAP()
};

extern CBrainStimSerialArmApp theApp;