
#include "stdafx.h"
#include "MaxonLib.h"
#include "Kinematics.h"
#include <atlstr.h>
#include <iostream>
#include <fstream>
#include <bitset>
using namespace std;

UINT findHomeThread(LPVOID p);
UINT movingThread(LPVOID p);

BOOL MaxonLib::OpenDevice(int InitNodeId,int FinalNodeId)
{
	NodeId = InitNodeId;
    HANDLE hNewKeyHandle;		
	//Close Previous Device
    if(m_KeyHandle)
    {
        if(m_KeyHandle) VCS_CloseDevice(m_KeyHandle, &m_ulErrorCode);
        m_KeyHandle = 0;
    }

    hNewKeyHandle = VCS_OpenDeviceDlg(&m_ulErrorCode);
    if(hNewKeyHandle)
    {
        m_KeyHandle = hNewKeyHandle;		
		for(NodeId;NodeId<FinalNodeId+1;NodeId++)
		{
		 //Clear Error History
        if(VCS_ClearFault(m_KeyHandle, NodeId, &m_ulErrorCode))
        {            //Read Operation Mode
            if(VCS_GetOperationMode(m_KeyHandle, NodeId, &m_bMode, &m_ulErrorCode))
            {                //Read Position Profile Objects
                if(VCS_GetPositionProfile(m_KeyHandle, NodeId, &m_ulProfileVelocity, &m_ulProfileAcceleration, &m_ulProfileDeceleration, &m_ulErrorCode))
                {                    //Write Profile Position Mode
                    if(VCS_SetOperationMode(m_KeyHandle, NodeId, OMD_PROFILE_POSITION_MODE, &m_ulErrorCode))
                    {                        //Write Profile Position Objects
                        if(VCS_SetPositionProfile(m_KeyHandle, NodeId, 30, 100, 100, &m_ulErrorCode))
                        {                            //Read Actual Position
                            if(VCS_GetPositionIs(m_KeyHandle, NodeId, &EncoderValue[NodeId-1], &m_ulErrorCode))
                            {
								cout<<"Joint "<<NodeId<<" was connected. "<<endl; 
								if(NodeId==FinalNodeId) 
								{
								//cout<<"Joint "<<NodeId<<" was connected. "<<endl<<endl; 
								return TRUE;
								}
                            }
                        }
                    }
                }
            }
        }
		else
			ShowErrorInformation(m_ulErrorCode);				
		}
		
	}
	else 
		{
			AfxMessageBox(TEXT("Can't open device!"), MB_ICONINFORMATION);
			return FALSE;
		}

	// �޸� ������ �����ϱ� ���� 
	// delete [] EncoderValue;
    return TRUE;
}
BOOL MaxonLib::Enable(HANDLE mHandle,int InitNodeId,int FinalNodeId)
{
	m_KeyHandle = mHandle;
	BOOL oFault = FALSE;
	NodeId = InitNodeId;

    for(NodeId;NodeId<FinalNodeId+1;NodeId++)	
	{
		if(!VCS_GetFaultState(m_KeyHandle, NodeId, &oFault, &m_ulErrorCode))
		{    ShowErrorInformation(m_ulErrorCode);        return FALSE;}
		if(oFault)
		{     if(!VCS_ClearFault(m_KeyHandle, NodeId, &m_ulErrorCode))
			{     ShowErrorInformation(m_ulErrorCode);        return FALSE;}    
		}
		if(!VCS_SetEnableState(m_KeyHandle, NodeId, &m_ulErrorCode))
			{   ShowErrorInformation(m_ulErrorCode);   return FALSE;}	
		cout<<"Joint "<<NodeId<<" was enabled. "<<endl; 
	}
	cout<<endl;
	return TRUE;
}
BOOL MaxonLib::Disable(HANDLE mHandle,int InitNodeId,int FinalNodeId)
{		
	m_KeyHandle = mHandle;
	NodeId = InitNodeId;	
	//bDisable = new BOOL[FinalNodeId];
    for(NodeId;NodeId<FinalNodeId+1;NodeId++)	
	{
		if(!VCS_SetDisableState(m_KeyHandle, NodeId, &m_ulErrorCode))
			 {ShowErrorInformation(m_ulErrorCode);return FALSE;}		
		cout<<"Joint "<<NodeId<<" was disabled. "<<endl; 
	}
	cout<<endl;
	return TRUE;
}
BOOL MaxonLib::Move(HANDLE mHandle,int joint,double targetPosition,int absoluteRelative)
{	
	m_KeyHandle = mHandle;
	NodeId = joint; 	double qc = targetPosition; 
	
	//------------ Default Setting	------------------------//
	//------------ Max. Vel = 200	------------------------//
	//------------ Accel = 1000 , Decel = 1000---------------//

	VCS_ActivateProfilePositionMode(m_KeyHandle, joint,  &m_ulErrorCode);
	VCS_SetPositionProfile(m_KeyHandle, joint, 200, 1000, 1000, &m_ulErrorCode);

	switch(joint)
	{
		case 1:
			if(absoluteRelative)
			{
				if(-27 < targetPosition < 120)
				{	bAbsConstraint = TRUE;	}
				else{	bAbsConstraint = FALSE;	}
			}
			else // Relative , Joint Control �� ���. 
			{
				bAbsConstraint = TRUE;
			}
				encoderResolution = (1024*4)*100;
				qc=	(targetPosition/360) * encoderResolution;
				break;
		case 2:
			if(absoluteRelative)
			{
			if(0 < targetPosition < 120)
				{	bAbsConstraint = TRUE;	}
				else{	bAbsConstraint = FALSE;	}
			}
			else // Relative , Joint Control �� ���. 
			{
				bAbsConstraint = TRUE;
			}
				encoderResolution = (1024*4)*100;
				qc=	(targetPosition/360) * encoderResolution;
				break;
		case 3:
			if(absoluteRelative)
			{
			if(0 < targetPosition < 150)
				{	bAbsConstraint = TRUE;	}
				else{	bAbsConstraint = FALSE;	}
			}
			else // Relative , Joint Control �� ���. 
			{
				bAbsConstraint = TRUE;
			}
				encoderResolution = (2048*4);
				qc=	- (targetPosition)*encoderResolution;
				break;			
		case 4:
			if(absoluteRelative)
			{
			if(-122 < targetPosition < 58)
				{	bAbsConstraint = TRUE;	}
				else{	bAbsConstraint = FALSE;	}
			}
			else // Relative , Joint Control �� ���. 
			{
				bAbsConstraint = TRUE;
			}
				encoderResolution = (512*4)*100;
				qc=(targetPosition)/360 * encoderResolution; 
				break;	
		case 5:
			if(absoluteRelative)
			{
			if(-30 < targetPosition < 30)
				{	bAbsConstraint = TRUE;	}
				else{	bAbsConstraint = FALSE;	}
			}
			else // Relative , Joint Control �� ���. 
			{
				bAbsConstraint = TRUE;
			}
				encoderResolution = (512*4)*100;
				qc= - (targetPosition)/360 * encoderResolution;
				break;	
		case 6:
			if(absoluteRelative)
			{
			if(-52 < targetPosition < 30)
				{	bAbsConstraint = TRUE;	}
				else{	bAbsConstraint = FALSE;	}
			}
			else // Relative , Joint Control �� ���. 
			{
				bAbsConstraint = TRUE;
			}
				encoderResolution = (512*4)*100;
				qc=(targetPosition)/360 * encoderResolution;
				break;	
		case 7:
			encoderResolution = (1024*4);
			qc=targetPosition * encoderResolution/2;
			break;
		default:
			qc=0;
	}

	if(bAbsConstraint) // ������ �� �ִ� ����
	{
		if(!VCS_MoveToPosition(m_KeyHandle, joint, qc, absoluteRelative, m_oImmediately, &m_ulErrorCode)) 
			{
				ShowErrorInformation(m_ulErrorCode); return	FALSE;
			}
	}

	else if(!bAbsConstraint) // ������ �� ���� ����
	{
		cout << "Can't move it." << endl;
	}

	return TRUE;
}
BOOL MaxonLib::Move(HANDLE mHandle, int joint,double targetPosition,long MaxVelocity,int absoluteRelative)
{
	m_KeyHandle = mHandle;
	NodeId = joint; 	double qc = targetPosition;	double relativePosition = 0;

	VCS_ActivateProfilePositionMode(m_KeyHandle, joint,  &m_ulErrorCode);
	VCS_SetPositionProfile(m_KeyHandle, joint, MaxVelocity, 1000, 1000, &m_ulErrorCode);

	switch(joint)
	{
		case 1:
			if(absoluteRelative)
			{
				if(-27 < targetPosition < 120)
				{	bAbsConstraint = TRUE;	}
				else{	bAbsConstraint = FALSE;	}
			}
			else // Relative , Joint Control �� ���. 
			{
				bAbsConstraint = TRUE;
			}
				encoderResolution = (1024*4)*100;
				qc=	(targetPosition/360) * encoderResolution;
				break;
		case 2:
			if(absoluteRelative)
			{
			if(0 < targetPosition < 120)
				{	bAbsConstraint = TRUE;	}
				else{	bAbsConstraint = FALSE;	}
			}
			else // Relative , Joint Control �� ���. 
			{
				bAbsConstraint = TRUE;
			}
				encoderResolution = (1024*4)*100;
				qc=	(targetPosition/360) * encoderResolution;
				break;
		case 3:
			if(absoluteRelative)
			{
			if(0 < targetPosition < 150)
				{	bAbsConstraint = TRUE;	}
				else{	bAbsConstraint = FALSE;	}
			}
			else // Relative , Joint Control �� ���. 
			{
				bAbsConstraint = TRUE;
			}
				encoderResolution = (2048*4);
				qc=	- (targetPosition)*encoderResolution;
				break;			
		case 4:
			if(absoluteRelative)
			{
			if(-122 < targetPosition < 58)
				{	bAbsConstraint = TRUE;	}
				else{	bAbsConstraint = FALSE;	}
			}
			else // Relative , Joint Control �� ���. 
			{
				bAbsConstraint = TRUE;
			}
				encoderResolution = (512*4)*100;
				qc=(targetPosition)/360 * encoderResolution; 
				break;	
		case 5:
			if(absoluteRelative)
			{
			if(-30 < targetPosition < 30)
				{	bAbsConstraint = TRUE;	}
				else{	bAbsConstraint = FALSE;	}
			}
			else // Relative , Joint Control �� ���. 
			{
				bAbsConstraint = TRUE;
			}
				encoderResolution = (512*4)*100;
				qc= - (targetPosition)/360 * encoderResolution;
				break;	
		case 6:
			if(absoluteRelative)
			{
			if(-52 < targetPosition < 30)
				{	bAbsConstraint = TRUE;	}
				else{	bAbsConstraint = FALSE;	}
			}
			else // Relative , Joint Control �� ���. 
			{
				bAbsConstraint = TRUE;
			}
				encoderResolution = (512*4)*100;
				qc=(targetPosition)/360 * encoderResolution;
				break;	
		case 7:
			encoderResolution = (1024*4);
			qc=targetPosition * encoderResolution/2;
			break;
		default:
			qc=0;
	}

	if(bAbsConstraint) // ������ �� �ִ� ����
	{
		if(!VCS_MoveToPosition(m_KeyHandle, joint, qc, absoluteRelative, m_oImmediately, &m_ulErrorCode)) 
			{
				ShowErrorInformation(m_ulErrorCode); return	FALSE;
			}
	}

	else if(!bAbsConstraint) // ������ �� ���� ����
	{
		cout << "Can't move it." << endl;
	}
	return TRUE;
}
BOOL MaxonLib::Move(HANDLE mHandle, int joint,double targetPosition,long MaxVelocity,long Acc,int absoluteRelative)
{
	m_KeyHandle = mHandle;
	NodeId = joint; 	double qc = targetPosition;	double relativePosition = 0;

	VCS_ActivateProfilePositionMode(m_KeyHandle, joint,  &m_ulErrorCode);
	VCS_SetPositionProfile(m_KeyHandle, joint, MaxVelocity, Acc, Acc, &m_ulErrorCode);

	switch(joint)
	{
		case 1:
			if(absoluteRelative)
			{
				if(-27 < targetPosition < 120)
				{	bAbsConstraint = TRUE;	}
				else{	bAbsConstraint = FALSE;	}
			}
			else // Relative , Joint Control �� ���. 
			{
				bAbsConstraint = TRUE;
			}
				encoderResolution = (1024*4)*100;
				qc=	(targetPosition/360) * encoderResolution;
				break;
		case 2:
			if(absoluteRelative)
			{
			if(0 < targetPosition < 120)
				{	bAbsConstraint = TRUE;	}
				else{	bAbsConstraint = FALSE;	}
			}
			else // Relative , Joint Control �� ���. 
			{
				bAbsConstraint = TRUE;
			}
				encoderResolution = (1024*4)*100;
				qc=	(targetPosition/360) * encoderResolution;
				break;
		case 3:
			if(absoluteRelative)
			{
			if(0 < targetPosition < 150)
				{	bAbsConstraint = TRUE;	}
				else{	bAbsConstraint = FALSE;	}
			}
			else // Relative , Joint Control �� ���. 
			{
				bAbsConstraint = TRUE;
			}
				encoderResolution = (2048*4);
				qc=	- (targetPosition)*encoderResolution;
				break;			
		case 4:
			if(absoluteRelative)
			{
			if(-122 < targetPosition < 58)
				{	bAbsConstraint = TRUE;	}
				else{	bAbsConstraint = FALSE;	}
			}
			else // Relative , Joint Control �� ���. 
			{
				bAbsConstraint = TRUE;
			}
				encoderResolution = (512*4)*100;
				qc=(targetPosition)/360 * encoderResolution; 
				break;	
		case 5:
			if(absoluteRelative)
			{
			if(-30 < targetPosition < 30)
				{	bAbsConstraint = TRUE;	}
				else{	bAbsConstraint = FALSE;	}
			}
			else // Relative , Joint Control �� ���. 
			{
				bAbsConstraint = TRUE;
			}
				encoderResolution = (512*4)*100;
				qc= - (targetPosition)/360 * encoderResolution;
				break;	
		case 6:
			if(absoluteRelative)
			{
			if(-52 < targetPosition < 30)
				{	bAbsConstraint = TRUE;	}
				else{	bAbsConstraint = FALSE;	}
			}
			else // Relative , Joint Control �� ���. 
			{
				bAbsConstraint = TRUE;
			}
				encoderResolution = (512*4)*100;
				qc=(targetPosition)/360 * encoderResolution;
				break;	
		case 7:
			encoderResolution = (1024*4);
			qc=targetPosition * encoderResolution/2;
			break;
		default:
			qc=0;
	}

	if(bAbsConstraint) // ������ �� �ִ� ����
	{
		if(!VCS_MoveToPosition(m_KeyHandle, joint, qc, absoluteRelative, m_oImmediately, &m_ulErrorCode)) 
			{
				ShowErrorInformation(m_ulErrorCode); return	FALSE;
			}
	}

	else if(!bAbsConstraint) // ������ �� ���� ����
	{
		cout << "Can't move it." << endl;
	}
	return TRUE;
}
BOOL MaxonLib::Halt(HANDLE mHandle,int joint)
{	
	m_KeyHandle = mHandle;
	NodeId = joint;	
	if(!VCS_HaltPositionMovement(m_KeyHandle, NodeId, &m_ulErrorCode))
    {        ShowErrorInformation(m_ulErrorCode);	return FALSE;    }
	return TRUE;
}
BOOL MaxonLib::getStatus(HANDLE mHandle,int InitNodeId,int FinalNodeId)
{
	m_KeyHandle = mHandle;
	NodeId = InitNodeId;
	bTotalStatus = TRUE;	
	double encoderResolution;
	
	for(NodeId;NodeId<FinalNodeId+1;NodeId++)
	{
		VCS_GetPositionIs(m_KeyHandle, NodeId, &EncoderValue[NodeId-1], &m_ulErrorCode);
		switch(NodeId)
		{
			case 1:
				encoderResolution=(1024.0 * 4.0 * 100.0);
				JointValue[0]=EncoderValue[0] *360 / encoderResolution;
				break;
			case 2:	
				encoderResolution=(1024.0 * 4.0 * 100.0);
				JointValue[1]=EncoderValue[1] * 360 / encoderResolution;
				break;
			case 3:
				encoderResolution=(2048.0 * 4.0);
				JointValue[2]= - EncoderValue[2] / encoderResolution;
				break;
			case 4:
				encoderResolution=(512.0 * 4.0 * 100.0);
				JointValue[3]= EncoderValue[3] * 360 / encoderResolution;
				break;
			case 5:
				encoderResolution=(512.0 * 4.0 * 100.0);
				JointValue[4]= - EncoderValue[4] * 360 / encoderResolution;
				break; 
			case 6:			
				encoderResolution=(512.0 * 4.0 * 100.0);
				JointValue[5]=EncoderValue[5] * 360 / encoderResolution;
				break;
		}

		if(!VCS_GetEnableState(m_KeyHandle, NodeId, &bEnable[NodeId-1] , &m_ulErrorCode))
		{		
			ShowErrorInformation(m_ulErrorCode);     return FALSE;  	
		}

		bTotalStatus = bTotalStatus*bEnable[NodeId-1]; // for������ ���͸� ��� ������ Ȯ���ϱ� ������, ������� 1�� ���� disable�� ���, ��ü�� disable�� ��Ÿ����.
													// �ڵ带 �ٸ� ������� �ٲ���ҰŰ���.
	}
	return TRUE;
}
BOOL MaxonLib::ShowErrorInformation(DWORD p_ulErrorCode)
{
    char* pStrErrorInfo;
    CString strDescription;

    if((pStrErrorInfo = (char*)malloc(100)) == NULL)
    {
		AfxMessageBox(TEXT("Not enough memory to allocate buffer for error information string\n"));
        return FALSE;
    }
	// ������� , VCS_GetErrorInfo(DWORD ErrorCodeValue, char* pErrorInfo, WORD MaxStrSize);
    if(VCS_GetErrorInfo(p_ulErrorCode, pStrErrorInfo, 100))
    {
        strDescription = pStrErrorInfo;
        AfxMessageBox(strDescription, MB_ICONINFORMATION);
        free(pStrErrorInfo);
        return TRUE;
    }
    else
    {
        free(pStrErrorInfo);
        AfxMessageBox(TEXT("Error information can't be read!"), MB_ICONINFORMATION);
        return FALSE;
    }
}
BOOL MaxonLib::torqControl(HANDLE mHandle,int joint,short qddot)
{	
	m_KeyHandle = mHandle;
	// NodeId = joint;	 �� �Է��� �ȵ���? joint �޾ƿ͵� �ʱ�ȭ�� �ȵǾ��ֳ�.

	VCS_GetOperationMode(m_KeyHandle, joint, &pOperationMode , &m_ulErrorCode);
	if (pOperationMode != OMD_CURRENT_MODE)
	{
		VCS_ActivateCurrentMode(m_KeyHandle, joint, &m_ulErrorCode);
	}

	if(abs(qddot) < 600) // ������ ���� �Ӱ谪 ���س���. 
	{
	if(!VCS_SetCurrentMust(m_KeyHandle, joint, qddot, &m_ulErrorCode))
		{ShowErrorInformation(m_ulErrorCode);	return FALSE;  };
		return TRUE;
	}
	return TRUE;
}
// findHomeThread ���� ��� ���ϴ� ��..
UINT findHomeThread(LPVOID p) 
{
	MaxonLib *m = (MaxonLib*)p;
	DWORD errCode = m->m_ulErrorCode;
	//int node = (m->NodeId)-1; // ������ ������ġ �����ΰ�, node ��ȣ�� 1�� Ŀ���� �����淡 ������ ����.
	int node = 1;
	//WORD digIN[6]; // Magnetic sensor ,  VCS_GetAllDigitalInputs 3��° ����
	WORD digIN;
	BOOL bState;
	for(;;)
	{
		VCS_GetAllDigitalInputs(MaxonHandle, node, &digIN, &errCode);
		if (digIN == 0) // ������ ������ ���
		{
			cout<<"Detect sensor "<< node <<"."<<endl;
			VCS_HaltPositionMovement(MaxonHandle, node, &errCode);				
			//break;	
		}
		VCS_GetMovementState(MaxonHandle, node, &bState, &errCode); 
		if(bState == 1) // Ȩ ã�ٰ� ���� ���, �����带 �����ϰ� �ٽ� �ϱ� ���ؼ�
		{
			cout<<"Can't detect sensor "<< node <<"."<<endl;
			//break;
		}
	}
	delete m;
	return 0;
}
BOOL MaxonLib::FindHomePose(HANDLE mHandle)
{
	m_KeyHandle = mHandle;
	long lHome[6];
	lHome[0] = -20;	lHome[1] = -15;	lHome[2] = 0;	lHome[3] = 55;	lHome[4] = 30;	lHome[5] = -5;
	BOOL bDetectHome[6];
	long lHomeOffset;	WORD digIN;		BOOL bState;
	cout << "Homing Start.. "<<endl;
	DWORD HomeAccel,SpeedSwch,SpeedIdx;	WORD CurntTresld;	long HomeOffst,HomePos;
	CurntTresld = 500;	HomeAccel = 1500;	HomeOffst = 0;
	HomePos = 0;	SpeedIdx = 10;	SpeedSwch = 400;
	//DWORD pHomeAccel,pSpeedSwch,pSpeedIdx;	WORD pCurntTresld;	long pHomeOffst,pHomePos;
	//VCS_GetHomingParameter(m_KeyHandle,3, &pHomeAccel, &pSpeedSwch, &pSpeedIdx, &pHomeOffst, &pCurntTresld, &pHomePos, &m_ulErrorCode);
	VCS_SetHomingParameter(m_KeyHandle,3, HomeAccel, SpeedSwch, SpeedIdx, HomeOffst, CurntTresld, HomePos, &m_ulErrorCode);
	VCS_ActivateHomingMode(m_KeyHandle, 3, &m_ulErrorCode);
	VCS_FindHome(m_KeyHandle, 3, -4 ,&m_ulErrorCode);
	BOOL bHomeStartFlag;
	bHomeStartFlag = VCS_WaitForHomingAttained(m_KeyHandle, 3, 7000, &m_ulErrorCode);

	if(bHomeStartFlag)
	{
		//if(bDetectHome[0]*bDetectHome[1]*bDetectHome[2]*bDetectHome[3]*bDetectHome[4]*bDetectHome[5])
		//{
		//	NodeId = failedNodeID;
		//}
		//else
		//{
		//	NodeId=6;
		//}
		for(NodeId=6;NodeId>0;NodeId--)
		{
			switch(NodeId)
			{
				case 1:
					VCS_DigitalInputConfiguration (m_KeyHandle, NodeId, 1, 68, 1, 1, 0, &m_ulErrorCode);
					Move(MaxonHandle,NodeId,-lHome[NodeId-1],0);
					Sleep(2000);
					Move(MaxonHandle,NodeId,lHome[NodeId-1]-10,0);
					while(1)
					{
						VCS_GetAllDigitalInputs(m_KeyHandle, NodeId, &digIN, &m_ulErrorCode);
						if (digIN == 0) // ������ ������ ���
						{
							cout<<"Detect sensor "<< NodeId <<"."<<endl;
							VCS_HaltPositionMovement(m_KeyHandle, NodeId, &m_ulErrorCode);
							Move(MaxonHandle, NodeId, -2 ,0); //�߰� offset
							NodeId = 1;
							bDetectHome[NodeId-1] = TRUE;
							break;	
						}
						VCS_GetMovementState(m_KeyHandle, NodeId, &bState, &m_ulErrorCode); 
						if(bState == 1)
						{
							cout<<"Can't detect sensor "<< NodeId <<"."<<endl;
							bDetectHome[NodeId-1] = FALSE;
							break;
						}
					}
					break;

				case 2:
					VCS_DigitalInputConfiguration (m_KeyHandle, NodeId, 1, 68, 1, 1, 0, &m_ulErrorCode);
					Move(MaxonHandle,NodeId,lHome[NodeId-1],0);
					while(1)
					{
						VCS_GetAllDigitalInputs(m_KeyHandle, NodeId, &digIN, &m_ulErrorCode);
						if (digIN == 0) // ������ ������ ���
						{
							cout<<"Detect sensor "<< NodeId <<"."<<endl;
							VCS_HaltPositionMovement(m_KeyHandle, NodeId, &m_ulErrorCode);
							Move(MaxonHandle,NodeId,-1,0); //�߰� offset
							bDetectHome[NodeId-1] = TRUE;
							break;	
						}
						VCS_GetMovementState(m_KeyHandle, NodeId, &bState, &m_ulErrorCode); 
						if(bState == 1)
						{
							cout<<"Can't detect sensor "<< NodeId <<"."<<endl;
							bDetectHome[NodeId-1] = FALSE;
							break;
						}
					}			
					break;
			
				case 3:
					VCS_DigitalInputConfiguration (m_KeyHandle, NodeId, 1, 68, 1, 1, 0, &m_ulErrorCode);
					Move(MaxonHandle,NodeId,-40,0);
					while(1)
					{
						VCS_GetAllDigitalInputs(m_KeyHandle, NodeId, &digIN, &m_ulErrorCode);
						if (digIN == 0) // ������ ������ ���
						{
							cout<<"Detect sensor "<< NodeId <<"."<<endl;
							VCS_HaltPositionMovement(m_KeyHandle, NodeId, &m_ulErrorCode);						
							//Move(MaxonHandle,NodeId,-55,600,0);
							bDetectHome[NodeId-1] = TRUE;
							break;	
						}
						VCS_GetMovementState(m_KeyHandle, NodeId, &bState, &m_ulErrorCode); 
						if(bState == 1)
						{
							cout<<"Can't detect sensor "<< NodeId <<"."<<endl;
							bDetectHome[NodeId-1] = FALSE;
							break;
						}
					}
					break;

				case 4:
					VCS_DigitalInputConfiguration (m_KeyHandle, NodeId, 1, 68, 1, 1, 0, &m_ulErrorCode);
					Move(MaxonHandle,NodeId,lHome[NodeId-1],0);
					while(1)
					{
						VCS_GetAllDigitalInputs(m_KeyHandle, NodeId, &digIN, &m_ulErrorCode);
						if (digIN == 0) // ������ ������ ���
						{
							cout<<"Detect sensor "<< NodeId <<"."<<endl;
							VCS_HaltPositionMovement(m_KeyHandle, NodeId, &m_ulErrorCode);
							Move(MaxonHandle,NodeId,1,0); //�߰� offset
							bDetectHome[NodeId-1] = TRUE;
							break;	
						}
						VCS_GetMovementState(m_KeyHandle, NodeId, &bState, &m_ulErrorCode); 
						if(bState == 1)
						{
							cout<<"Can't detect sensor "<< NodeId <<"."<<endl;
							bDetectHome[NodeId-1] = FALSE;
							break;
						}
					}
					break;

				case 5:
					VCS_DigitalInputConfiguration (m_KeyHandle, NodeId, 1, 68, 1, 1, 0, &m_ulErrorCode);
					Move(MaxonHandle,NodeId,	- lHome[NodeId-1],0);
					Sleep(2000);
					Move(MaxonHandle,NodeId,   lHome[NodeId-1]+20 , 0);


					while(1)
					{
						VCS_GetAllDigitalInputs(m_KeyHandle, NodeId, &digIN, &m_ulErrorCode);
						if (digIN == 0) // ������ ������ ���
						{
							cout<<"Detect sensor "<< NodeId <<"."<<endl;
							VCS_HaltPositionMovement(m_KeyHandle, NodeId, &m_ulErrorCode);
							Move(MaxonHandle,NodeId,5,0);
							bDetectHome[NodeId-1] = TRUE;
							break;	
						}
						VCS_GetMovementState(m_KeyHandle, NodeId, &bState, &m_ulErrorCode); 
						if(bState == 1)
						{
							cout<<"Can't detect sensor "<< NodeId <<"."<<endl;
							bDetectHome[NodeId-1] = FALSE;
							break;
						}
					}
					break;

				case 6:

					
					/// ���� Ȩ �ڵ� ���� //					
					VCS_DigitalInputConfiguration (m_KeyHandle, NodeId, 1, 68, 1, 1, 0, &m_ulErrorCode);				
					Move(MaxonHandle,NodeId,  - lHome[NodeId-1],0);
					Sleep(1000);
					Move(MaxonHandle,NodeId,   lHome[NodeId-1]-10 , 0);
					while(1)
					{
						VCS_GetAllDigitalInputs(m_KeyHandle, NodeId, &digIN, &m_ulErrorCode);
						if (digIN == 0) // ������ ������ ���
						{
							cout<<"Detect sensor "<< NodeId <<"."<<endl;
							VCS_HaltPositionMovement(m_KeyHandle, NodeId, &m_ulErrorCode);
						
							Move(MaxonHandle,NodeId,-4,0);
						
							bDetectHome[NodeId-1] = TRUE;
							break;	
						}
						VCS_GetMovementState(m_KeyHandle, NodeId, &bState, &m_ulErrorCode); 
						if(bState == 1)
						{
							cout<<"Can't detect sensor "<< NodeId <<"."<<endl;
							bDetectHome[NodeId-1] = FALSE;
							break;
						}
					}


					break;
				}//switch�� ����


				if(!bDetectHome[NodeId-1])
				{
					cout << "Again.. " << endl;
					bInitEncodr = FALSE;
					failedNodeId = NodeId;
					return FALSE;
				}
			}//for�� ����

		Sleep(2000);
		cout<<"Now, Change encoder values for home pose. " <<endl;
		Sleep(2000);
		
		for(NodeId=1;NodeId<7;NodeId++)
		{
			if(NodeId==1)
			{
				if(bDetectHome[NodeId-1] == TRUE) 
					{lHomeOffset = ((45.0) / 360.0) * (1024.0 * 4.0 * 100.0) ; // ((45.0-3.0) / 360.0) * (1024.0 * 4.0 * 100.0) ; // +3��
					cout<<"..."<<endl;}
			}
			else if(NodeId==2)
			{
				if(bDetectHome[NodeId-1] == TRUE) 
				{lHomeOffset = ( (60.0) / 360.0) * (1024.0 * 4.0 * 100.0) ; //( (60.0-3.0) / 360.0) * (1024.0 * 4.0 * 100.0) ; 
				cout<<"..."<<endl;}			
			}
			else if(NodeId==3)
			{
				if(bDetectHome[NodeId-1] == TRUE) 
				{lHomeOffset = ( -120 * (2048.0 * 4.0) ) ; 
				cout<<"..."<<endl;}			
			}
			else if(NodeId==4)
			{
				if(bDetectHome[NodeId-1] == TRUE) 
				{
					lHomeOffset = 0 ; // ( (-6.0)/ 360.0 ) * (512.0 * 4.0 * 100)  ; 
				cout<<"..."<<endl;	}		
			}
			else if(NodeId==5)
			{
				if(bDetectHome[NodeId-1] == TRUE) 
				{   
					lHomeOffset = 0;
					//lHomeOffset = ( (-4.0)/ 360.0 ) * (512.0 * 4.0 * 100)  ;  // 0; //
				cout<<"..."<<endl;	}		
			}
			else if(NodeId==6)
			{
				if(bDetectHome[NodeId-1] == TRUE) 
				{
					/// ������ Ȩ �ڵ� ���� //
					// lHomeOffset = 0 ; 
					/// ���� Ȩ �ڵ� ���� //
					lHomeOffset = 0;
					//lHomeOffset = ( (4.0)/ 360.0 ) * (512.0 * 4.0 * 100)  ; 
				cout<<"..."<<endl;	}
			}
			VCS_DefinePosition(m_KeyHandle, NodeId, lHomeOffset, &m_ulErrorCode);		
		}

			if(bDetectHome[0]&&bDetectHome[1]&&bDetectHome[2]&&bDetectHome[3]&&bDetectHome[4]&&bDetectHome[5]){
				cout << "Homing completed ! "<<endl;
				bInitEncodr = TRUE;
			}
			else
			{
				cout << "Again.. " << endl;
				bInitEncodr = FALSE;
				return FALSE;
			}
		}//3�� ���� TIMEOUT ���� if��

		else
		{
			cout<<"Click again 'Initializing Encoder Value'."<<endl<<endl;
			bInitEncodr = FALSE;
			return FALSE;
		}
		return TRUE;
}

UINT movingThread(LPVOID p)
{
	DWORD dwErrorCode;
	BOOL bStateMotor;
	int node, iMoving;
	bitset<6> bMoving;
	for(;;)
	{
		node = 1;	
		bMoving.reset();
		for(node; node < 7; node++)
		{
			VCS_GetMovementState(MaxonHandle,node,&bStateMotor,&dwErrorCode);
			if(bStateMotor)		bMoving.set(node-1, true);
		}

		iMoving = bMoving.to_ulong();
		
		if(iMoving == 63) // ��� ���Ͱ� ����.	
		{
			cout<<"Every motor was stopped."<<endl;
			return 0;  
		}
		//else
		//{
		//cout << iMoving << " <- This decimal value is 'iMoving' variable." << endl;
		//}
	}
	return -1;
}
BOOL MaxonLib::CheckMovement()
{
	DWORD exitMessage;
	CWinThread *mThread = AfxBeginThread(movingThread,this);
	exitMessage = WaitForSingleObject(mThread->m_hThread,2000);
	if( exitMessage == 0 )	return TRUE;
	else	return FALSE;
}

/*
UINT movingThread(LPVOID p)
{
	// Prismatic �������� �������� �ϱ� ����
	Ikjoint *InvK_temp = (Ikjoint*)p;
	Ikjoint *InvK	   = new Ikjoint;

	memcpy( InvK, InvK_temp, sizeof(Ikjoint));
	if(InvK->bIKFlag)
	{
		int i = 1;	bitset<6> bMoving;
		int iMoving;	bMoving.reset();
		MaxonLib *m = new MaxonLib;

		m->Move(MaxonHandle,3,70,1);			// 3�� �ø���

		// ------------------------------------------------------------ //
		for(;;)									// 3�� Ȯ���ϴٰ� ���߸�
		{
			// �����̸� 0, ���߸� 1
			VCS_GetMovementState(MaxonHandle, 3 , &(m->bTargetReached), &(m->m_ulErrorCode));
			if(m->bTargetReached)
			{
				break;
			}
		}
	
		// ------------------------------------------------------------ //
		for(i=1;i<7;i++)						// ������ ���� ������(3�� ����)
		{
			if(i == 3)
				continue;
			m->Move(MaxonHandle,i,InvK->InvJoint[i-1],1);
		}	

		// ------------------------------------------------------------ //
		for(;;)									// ������ ���� ������ Ȯ���ϴٰ� ���߸�
		{
			bMoving.reset();					// ���� ������ �߻������� �ٸ� �� �־
		
			for(i=1;i<7;i++)
			{	
				if(i == 3)
					continue;
				VCS_GetMovementState(MaxonHandle, i , &(m->bTargetReached), &(m->m_ulErrorCode));
				if(m->bTargetReached)
				{
					bMoving.set(i-1, true);
					cout<<i<<" motor was stopped."<<endl;
				}
			}
			iMoving = bMoving.to_ulong();
			if(iMoving == 59) // 3�� ���� �� ����.
			{
				cout<<"Every motor was stopped !!"<<endl;
		// ------------------------------------------------------------ //
				m->Move(MaxonHandle,3,InvK->InvJoint[2],1);		// 3�� ���� ������.
		// ------------------------------------------------------------ //
				return 0;  
			}
			else
			{
				cout << iMoving << " <- This decimal value is 'iMoving' variable." << endl;
			}
		}
	}	
	return 0;  
	// ------------------------------------------------------------ //
}

BOOL MaxonLib::CheckMovement(struct IKJoint* InvK)
{
	AfxBeginThread(movingThread,(IKJoint*)InvK);
	return TRUE;
}*/
BOOL MaxonLib::DataRecord(HANDLE mHandle)
{
	for(int i=1;i<7;i++)
	{
	VCS_SetRecorderParameter(mHandle, i, 1000, 0, &m_ulErrorCode); // 100ms
	// Position Actual Value : 24676
	// Current Actual Value : 0x6078
	// Current Demand Value : 0x2031
	VCS_ActivateChannel(mHandle, i, 1, 0x6078, 0, 4, &m_ulErrorCode);
	//VCS_ActivateChannel(mHandle, i, 2, 0x2031, 0, 4, &m_ulErrorCode);
	VCS_StartRecorder(mHandle, i , &m_ulErrorCode);
	}
	return TRUE;
}
BOOL MaxonLib::StopRecord(HANDLE mHandle)
{
	char* DataAddress[6];
	DataAddress[0] = "D:\\Project_byH\\_Feedback\\BrainStimSerialArm\\data_encoder1.txt";
	DataAddress[1] = "D:\\Project_byH\\_Feedback\\BrainStimSerialArm\\data_encoder2.txt";
	DataAddress[2] = "D:\\Project_byH\\_Feedback\\BrainStimSerialArm\\data_encoder3.txt";
	DataAddress[3] = "D:\\Project_byH\\_Feedback\\BrainStimSerialArm\\data_encoder4.txt";
	DataAddress[4] = "D:\\Project_byH\\_Feedback\\BrainStimSerialArm\\data_encoder5.txt";
	DataAddress[5] = "D:\\Project_byH\\_Feedback\\BrainStimSerialArm\\data_encoder6.txt";

	for(int i=1;i<7;i++)
	{VCS_StopRecorder(mHandle, i , &m_ulErrorCode);}
	for(int i=1;i<7;i++)
	{VCS_ExportChannelDataToFile(mHandle, i, DataAddress[i-1] , &m_ulErrorCode);}

	WORD pKP[6]	,pKI[6], pKD[6];
	for(int i=1;i<7;i++)
	{VCS_GetPositionRegulatorGain(mHandle, i, &pKP[i-1], &pKI[i-1], &pKD[i-1], &m_ulErrorCode);}
	ofstream data_gain;
	data_gain.open("data_gain.txt");
	for(int i=0;i<6;i++)
	{data_gain << pKP[i] << " " << pKI[i] << " " << pKD[i] << "\n" ;}
//	VCS_GetPositionRegulatorGain(HANDLE KeyHandle, WORD NodeId, WORD* pP, WORD* pI, WORD* pD, DWORD* pErrorCode);
//  VCS_GetPositionRegulatorFeedForward(HANDLE KeyHandle, WORD NodeId, WORD* pVelocityFeedForward, WORD* pAccelerationFeedForward, DWORD* pErrorCode);
	return TRUE;
}
