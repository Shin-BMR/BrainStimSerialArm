// JointControl.cpp : ���� �����Դϴ�.
//

#include "stdafx.h"
#include "BrainStimSerialArm.h"
#include "BrainStimSerialArmDlg.h" // �θ� ���̾�α��� ����Լ���� �������� �޾ƿ��� ���ؼ�.
#include "JointControl.h"
#include "afxdialogex.h"
#include <iostream>
#include <fstream>
// JointControl ��ȭ �����Դϴ�.

using namespace std;
IMPLEMENT_DYNAMIC(JointControl, CDialogEx)


JointControl::JointControl(CWnd* pParent /*=NULL*/)
	: CDialogEx(JointControl::IDD, pParent)
	, jointNodeId(0)
	, jointTargetPosition(0)
	, jointMaxVelocity(200)
{
}

JointControl::~JointControl()
{
}

void JointControl::DoDataExchange(CDataExchange* pDX)
{
	CDialogEx::DoDataExchange(pDX);
	DDX_Text(pDX,IDC_jointNodeId,jointNodeId);
	DDX_Text(pDX,IDC_jointTargetPosition,jointTargetPosition);
	DDX_Text(pDX,IDC_jointMaxVelocity,jointMaxVelocity);	
	DDX_Text(pDX,IDC_JointStatus1,m_JointStatus1);
	DDX_Text(pDX,IDC_JointStatus2,m_JointStatus2);
	DDX_Text(pDX,IDC_JointStatus3,m_JointStatus3);
	DDX_Text(pDX,IDC_JointStatus4,m_JointStatus4);
	DDX_Text(pDX,IDC_JointStatus5,m_JointStatus5);
	DDX_Text(pDX,IDC_JointStatus6,m_JointStatus6);
	DDX_Control(pDX, IDC_PositionGain, m_PositionGain);
}


BEGIN_MESSAGE_MAP(JointControl, CDialogEx)
	ON_BN_CLICKED(IDC_jointMove, &JointControl::OnBnClickedjointmove)
	ON_EN_CHANGE(IDC_jointNodeId, OnChangeEditJointNodeId)
	ON_EN_CHANGE(IDC_jointTargetPosition, OnChangeEditJointTargetPosition)
	ON_EN_CHANGE(IDC_jointMaxVelocity, OnChangeEditJointMaxVelocity)
	ON_BN_CLICKED(IDC_jointHalt, &JointControl::OnBnClickedjointhalt)
	ON_BN_CLICKED(IDC_CheckEnable, &JointControl::OnBnClickedCheckenable)
	ON_BN_CLICKED(IDC_FindHome, &JointControl::OnBnClickedFindhome)
	ON_BN_CLICKED(IDOK, &JointControl::OnBnClickedOk)
	ON_BN_CLICKED(IDC_Constraint, &JointControl::OnBnClickedConstraint)
	ON_BN_CLICKED(IDC_GetPositionGain, &JointControl::OnBnClickedGetpositiongain)
END_MESSAGE_MAP()



void JointControl::OnBnClickedjointmove()
{
	MaxonLib *m = new MaxonLib;
	m->Move(MaxonHandle,jointNodeId,jointTargetPosition,jointMaxVelocity,0); //Relative 0 	
	cout << "[Joint Control Mode] " ;
	if(jointNodeId == 3)
	{
		cout << " Joint " <<jointNodeId << " has moved by " << jointTargetPosition << " [mm] with relative. " << endl;
	}
	else
	cout << " Joint " <<jointNodeId << " has moved by " << jointTargetPosition << " [degrees] with relative. " << endl;
	
	delete m;
}

void JointControl::OnBnClickedjointhalt()
{
	MaxonLib *m = new MaxonLib;
	m->Halt(MaxonHandle,jointNodeId);
	cout << " joint " << jointNodeId << " has stopped. " << endl;	
	delete m;
}
void JointControl::OnBnClickedCheckenable()
{
	MaxonLib *maxon = new MaxonLib;

	int InitNodeId,FinalNodeId;
	InitNodeId = ((CBrainStimSerialArmDlg *)GetParent())->InitNodeId; // ���δ��̾�α��� ������� �ޱ�.
	FinalNodeId = ((CBrainStimSerialArmDlg *)GetParent())->FinalNodeId;

	maxon->getStatus(MaxonHandle,InitNodeId,FinalNodeId);

	CString StrEnable[6],StrDisable[6];
	StrEnable[0]="1";StrDisable[0]="";
	StrEnable[1]="2";StrDisable[1]="";
	StrEnable[2]="3";StrDisable[2]="";
	StrEnable[3]="4";StrDisable[3]="";
	StrEnable[4]="5";StrDisable[4]="";
	StrEnable[5]="6";StrDisable[5]="";

	for(int i=InitNodeId-1;i<FinalNodeId;i++) 
	{
		if(maxon->bEnable[i])
		{
			switch(i){
			case 0:
			m_JointStatus1 = StrEnable[i];break;
			case 1:
			m_JointStatus2 = StrEnable[i];break;
			case 2:
			m_JointStatus3 = StrEnable[i];break;
			case 3:
			m_JointStatus4 = StrEnable[i];break;
			case 4:
			m_JointStatus5 = StrEnable[i];break;
			case 5:
			m_JointStatus6 = StrEnable[i];break;
			}
		}
		else
		{
			switch(i){
			case 0:
			m_JointStatus1 = StrDisable[i];break;
			case 1:
			m_JointStatus2 = StrDisable[i];break;
			case 2:
			m_JointStatus3 = StrDisable[i];break;
			case 3:
			m_JointStatus4 = StrDisable[i];break;
			case 4:
			m_JointStatus5 = StrDisable[i];break;
			case 5:
			m_JointStatus6 = StrDisable[i];break;
			}
		}
	}
	UpdateData(false);
	delete maxon;
}
void JointControl::OnBnClickedFindhome()
{
	MaxonLib *maxon = new MaxonLib;
	if(maxon->FindHomePose(MaxonHandle))
	{		
		ofstream InitialSetting;
		InitialSetting.open("InitialSetting.txt");
		InitialSetting << "1.Initializing the encoder value(TRUE/FALSE) : \n"  ;
		InitialSetting << " TRUE " << "\n" ;
	}
	delete maxon;
}

void JointControl::OnChangeEditJointNodeId()
{
    if(m_hWnd) UpdateData(true);
}
void JointControl::OnChangeEditJointTargetPosition()
{
    if(m_hWnd) UpdateData(true);
}
void JointControl::OnChangeEditJointMaxVelocity()
{
    if(m_hWnd) UpdateData(true);
}
void JointControl::OnBnClickedOk()
{
	CDialogEx::OnOK();
}


void JointControl::OnBnClickedConstraint()
{	
	if(bConstraint)
	{
		bConstraint = FALSE;
		GetDlgItem(IDC_Constraint)->SetWindowTextW(L"Contraint");
		cout<<"Release the constraint. "<< endl;
		cout << "Constraint Flag is 'FALSE' ." <<endl;
	}
	else
	{
		bConstraint = TRUE;
		GetDlgItem(IDC_Constraint)->SetWindowTextW(L"Contraint Release");		
		cout<<"Set the constraint. "<< endl;
		cout << "Constraint Flag is 'TRUE' ." <<endl;
	}
}


void JointControl::OnBnClickedGetpositiongain()
{	
	// JointControl Dialog ���� �� Control List ��� ������ �ڵ����� ǥ�ð� �ȵż�
	// �Ʒ� if������ ������ ������ �ʱ� ������� �ƴ��� Ȯ��
	if(m_PositionGain.GetHeaderCtrl()->GetItemCount()==0)
	{
		LVCOLUMN LC;
		LC.mask = LVCF_FMT | LVCF_SUBITEM | LVCF_TEXT | LVCF_WIDTH;	LC.fmt = LVCFMT_CENTER;
		LC.iSubItem = 0;	LC.pszText = _T("Node");	LC.cx = 55;
		m_PositionGain.InsertColumn(0,&LC);	
		LC.iSubItem = 1;	LC.pszText = _T("Kp");	LC.cx = 50;
		m_PositionGain.InsertColumn(1,&LC);
		LC.iSubItem = 2;	LC.pszText = _T("Ki");	LC.cx = 50;
		m_PositionGain.InsertColumn(2,&LC);
		LC.iSubItem = 3;	LC.pszText = _T("Kd");	LC.cx = 50;
		m_PositionGain.InsertColumn(3,&LC);	

		m_PositionGain.InsertItem(0, _T("1")); 	m_PositionGain.InsertItem(1, _T("2")); 	m_PositionGain.InsertItem(2, _T("3")); 
		m_PositionGain.InsertItem(3, _T("4")); 	m_PositionGain.InsertItem(4, _T("5"));	m_PositionGain.InsertItem(5, _T("6"));
	}

	WORD position_pgain[6],position_igain[6],position_dgain[6];	
	CString strP[6],strI[6],strD[6];	
	DWORD m_ulErrorCode;

	for(int i=1;i<7;i++)
	{
		VCS_GetPositionRegulatorGain(MaxonHandle,i, &position_pgain[i-1], &position_igain[i-1], &position_dgain[i-1], &m_ulErrorCode);
		cout << i << " ��° P, I , D : " << position_pgain << " , " << position_igain << " , " << position_dgain << endl;
		strP[i-1].Format(_T("%.1f"),position_pgain[i-1]);	
		strI[i-1].Format(_T("%.1f"),position_igain[i-1]);	
		strD[i-1].Format(_T("%.1f"),position_dgain[i-1]);	
	}

	// memcpy���� CString Ÿ���� ������ ���� �ʱ�ȭ�� ���� �ϰ� �ؾ���. 
	CString strK[6] = {0};

	for(int j = 1; j < 4; j++)
	{
		switch(j)
		{	
			case 1:	memcpy(strK,strP,sizeof(strK));
				break;
			case 2: memcpy(strK,strI,sizeof(strK));
				break;
			case 3: memcpy(strK,strD,sizeof(strK));
				break;
		}				
		for(int i = 0; i < 6; i++)
		{
			m_PositionGain.SetItem(i, j, LVIF_TEXT, strK[i], 0, 0, 0, NULL ); 
		}
	}

	
	//VCS_SetPositionRegulatorGain(MaxonHandle, 2, 700, 100, 500, &m_ulErrorCode);
}
