
#pragma once

#include "Definitions.h"

typedef class MAXONLIB{
	
public:
		
	BOOL getStatus(HANDLE mHandle,int InitNodeId,int FinalNodeId);
	BOOL OpenDevice(int InitNodeId,int FinalNodeId);
	BOOL Enable(HANDLE MaxonHandle, int InitNodeId,int FinalNodeId);
	BOOL Disable(HANDLE MaxonHandle, int InitNodeId,int FinalNodeId);
	BOOL Move(HANDLE MaxonHandle, int joint,double targetPosition,int absoluteRelative);
	BOOL Move(HANDLE MaxonHandle, int joint,double targetPosition,long MaxVelocity,int absoluteRelative);
	BOOL Move(HANDLE MaxonHandle, int joint,double targetPosition,long MaxVelocity,long Acc,int absoluteRelative);

	BOOL Halt(HANDLE MaxonHandle, int joint);
	BOOL ShowErrorInformation(DWORD p_ulErrorCode);
	BOOL torqControl(HANDLE mHandle,int joint,short qddot);
	BOOL FindHomePose(HANDLE mHandle);

	BOOL CheckMovement();
	//BOOL CheckMovement(struct IKJoint* InvK);
	
	BOOL DataRecord(HANDLE mHandle);
	BOOL StopRecord(HANDLE mHandle);
	
	__int8 pOperationMode;
	
	double encoderResolution;
	
	BOOL bAbsConstraint;
	BOOL bTargetReached;
	BOOL bMovingCheck;
	CWinThread *m_findHomeThread; //find home thread ��ȯ�� 
	//////////////////////////////// UINT findHomeThread(LPVOID p) �����Ҷ� ��� ���ϸ� �� ���� ������ ��.
	int NodeId;
	long EncoderValue[6];
	float JointValue[6];
	BOOL bEnable[6];	
	BOOL bTotalStatus;

    HANDLE m_KeyHandle;
    DWORD m_ulErrorCode;
    __int8 m_bMode;
    DWORD m_ulProfileAcceleration;
    DWORD m_ulProfileDeceleration;
    DWORD m_ulProfileVelocity;	
    BOOL m_oImmediately;

	//~MAXONLIB();

}MaxonLib;