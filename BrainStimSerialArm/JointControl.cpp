// JointControl.cpp : 구현 파일입니다.
//

#include "stdafx.h"
#include "BrainStimSerialArm.h"
#include "BrainStimSerialArmDlg.h" // 부모 다이얼로그의 멤버함수들과 변수들을 받아오기 위해서.
#include "JointControl.h"
#include "afxdialogex.h"
#include <iostream>
#include <fstream>
// JointControl 대화 상자입니다.

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
END_MESSAGE_MAP()


// JointControl 메시지 처리기입니다.

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
	InitNodeId = ((CBrainStimSerialArmDlg *)GetParent())->InitNodeId; // 메인다이얼로그의 멤버변수 받기.
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
