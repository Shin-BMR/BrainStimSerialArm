
// BrainStimSerialArmDlg.cpp : 구현 파일
//
#include "stdafx.h"
#include "BrainStimSerialArm.h"
#include "BrainStimSerialArmDlg.h"
#include "JointControl.h"
#include "afxdialogex.h"
#include <iostream>
#include <fstream>
#define _CRTDGB_MAP_ALLOC

char *tms_port_name = "\\\\.\\COM11";
SerialPort tms_arduino(tms_port_name);
char *emg_port_name = "\\\\.\\COM12";
SerialPort emg_arduino(emg_port_name);

#include <stdlib.h>
#include <crtdbg.h> // 메모리 누수 확인 때 디버그 모드에 사용
#include <openvr.h>
bool m_rbShowTrackedDevice[vr::k_unMaxTrackedDeviceCount];
#define UM_UPDATE WM_USER+1    // Encoder value update
#ifdef _DEBUG
#define new DEBUG_NEW
#endif
using namespace Eigen;
using namespace std;

// Data recoding /////////////////
float data[10000]; float data2[10000]; 
float data3[10000]; float data4[10000]; 
float data5[10000]; float data6[10000]; 
int data_count = 0; int file_flag = 1;
//////////////////////////////////

class CAboutDlg : public CDialogEx
{
public:
	CAboutDlg();
	enum { IDD = IDD_ABOUTBOX };
	protected:
	virtual void DoDataExchange(CDataExchange* pDX);  
protected:
	DECLARE_MESSAGE_MAP()
};
CAboutDlg::CAboutDlg() : CDialogEx(CAboutDlg::IDD)
{
}
void CAboutDlg::DoDataExchange(CDataExchange* pDX)
{
	CDialogEx::DoDataExchange(pDX);
}
BEGIN_MESSAGE_MAP(CAboutDlg, CDialogEx)
END_MESSAGE_MAP()


// CBrainStimSerialArmDlg 대화 상자
CBrainStimSerialArmDlg::CBrainStimSerialArmDlg(CWnd* pParent /*=NULL*/)
	: CDialogEx(CBrainStimSerialArmDlg::IDD, pParent) // MFC창 초기값 설정
	, inputRx(180)	, inputRy(90)	, inputRz(0)	, inputX(0)	, inputY(0)	, inputZ(110) , m_FineRotZ(0)	, m_FineRotY(0)	, m_FineRotX(0)
	, m_encoder1(0)	, m_encoder2(0)	, m_encoder3(0)	, m_encoder4(0)	, m_encoder5(0)	, m_encoder6(0) , m_SystemMode("Disable") , m_error(0) , m_Orierror(0)
	, m_DepthValue(100)
	
{
	m_hIcon = AfxGetApp()->LoadIcon(IDR_MAINFRAME);
}
void CBrainStimSerialArmDlg::DoDataExchange(CDataExchange* pDX)
{
	CDialogEx::DoDataExchange(pDX);
	DDX_Text(pDX, IDC_inputRz, inputRz);	DDX_Text(pDX, IDC_inputRy, inputRy);	DDX_Text(pDX, IDC_inputRx, inputRx);
	DDX_Text(pDX, IDC_inputZ, inputZ);	DDX_Text(pDX, IDC_inputY, inputY);	DDX_Text(pDX, IDC_inputX, inputX);
	DDX_Text(pDX, IDC_FineRotZ, m_FineRotZ);	DDX_Text(pDX, IDC_FineRotY, m_FineRotY);	DDX_Text(pDX, IDC_FineRotX, m_FineRotX);
	
	DDX_Text(pDX, IDC_Encoder1, m_encoder1 );	DDX_Text(pDX, IDC_Encoder2, m_encoder2 );	DDX_Text(pDX, IDC_Encoder3, m_encoder3 );
	DDX_Text(pDX, IDC_Encoder4, m_encoder4 );	DDX_Text(pDX, IDC_Encoder5, m_encoder5 );	DDX_Text(pDX, IDC_Encoder6, m_encoder6 );
	DDX_Text(pDX, IDC_Error, m_error);	DDX_Text(pDX, IDC_OriError, m_Orierror);
	
	DDX_Text(pDX, IDC_SystemMode, m_SystemMode );
	DDX_Control(pDX, IDC_ROBOTBASE_FLAG,RB_ledCtrl);
	DDX_Control(pDX, IDC_HEAD_FLAG,H_ledCtrl);
	DDX_Control(pDX, IDC_Depth, m_Depth);
	DDX_Text(pDX, IDC_DepthValue, m_DepthValue);
	DDX_Control(pDX, IDC_PresetListCtrl, m_CListCtrl);
}
BEGIN_MESSAGE_MAP(CBrainStimSerialArmDlg, CDialogEx)
	ON_WM_SYSCOMMAND()
	ON_WM_PAINT()
	ON_WM_QUERYDRAGICON()
	ON_BN_CLICKED(IDC_MoveEndEffector, &CBrainStimSerialArmDlg::OnBnClickedMoveendeffector)
	ON_BN_CLICKED(IDC_Halt, &CBrainStimSerialArmDlg::OnBnClickedHalt)
	ON_BN_CLICKED(IDC_OpenDevice, &CBrainStimSerialArmDlg::OnBnClickedOpendevice)
	ON_BN_CLICKED(IDC_EnableMotor, &CBrainStimSerialArmDlg::OnBnClickedEnablemotor)
	ON_EN_CHANGE(IDC_inputX, OnChangeEditTranslationXaxis)
	ON_EN_CHANGE(IDC_inputY, OnChangeEditTranslationYaxis)
	ON_EN_CHANGE(IDC_inputZ, OnChangeEditTranslationZaxis)	
	ON_EN_CHANGE(IDC_inputRx, OnChangeEditRotationXaxis)
	ON_EN_CHANGE(IDC_inputRy, OnChangeEditRotationYaxis)
	ON_EN_CHANGE(IDC_inputRz, OnChangeEditRotationZaxis)

	ON_EN_CHANGE(IDC_FineRotZ, OnChangeEditAdjRotationZaxis)
	ON_EN_CHANGE(IDC_FineRotY, OnChangeEditAdjRotationYaxis)
	ON_EN_CHANGE(IDC_FineRotX, OnChangeEditAdjRotationXaxis)

	ON_BN_CLICKED(IDC_DisableMotor, &CBrainStimSerialArmDlg::OnBnClickedDisablemotor)
	ON_BN_CLICKED(IDC_JointControl, &CBrainStimSerialArmDlg::OnBnClickedJointcontrol)
	ON_BN_CLICKED(IDC_PipeConnect, &CBrainStimSerialArmDlg::OnBnClickedPipeconnect)
	ON_BN_CLICKED(IDC_robotBaseSet, &CBrainStimSerialArmDlg::OnBnClickedrobotbaseset)

	ON_MESSAGE(UM_UPDATE,OnUpdateData)

	ON_BN_CLICKED(IDOK, &CBrainStimSerialArmDlg::OnBnClickedOk)
	ON_BN_CLICKED(IDC_Compensation, &CBrainStimSerialArmDlg::OnBnClickedCompensation)
	ON_BN_CLICKED(IDC_GoHome, &CBrainStimSerialArmDlg::OnBnClickedGohome)
	ON_BN_CLICKED(IDC_DataRecorder, &CBrainStimSerialArmDlg::OnBnClickedDatarecorder)
	ON_BN_CLICKED(IDC_DataRecorderStop, &CBrainStimSerialArmDlg::OnBnClickedDatarecorderstop)
	ON_BN_CLICKED(IDC_UpButton, &CBrainStimSerialArmDlg::OnBnClickedUpbutton)
	ON_BN_CLICKED(IDC_LeftButton, &CBrainStimSerialArmDlg::OnBnClickedLeftbutton)
	ON_BN_CLICKED(IDC_DownButton, &CBrainStimSerialArmDlg::OnBnClickedDownbutton)
	ON_BN_CLICKED(IDC_RightButton, &CBrainStimSerialArmDlg::OnBnClickedRightbutton)
	ON_NOTIFY(NM_RELEASEDCAPTURE, IDC_Depth, &CBrainStimSerialArmDlg::OnNMReleasedcaptureDepth)
	ON_BN_CLICKED(IDC_ViveCtrl, &CBrainStimSerialArmDlg::OnBnClickedVivectrl)
	ON_BN_CLICKED(IDC_SavePose, &CBrainStimSerialArmDlg::OnBnClickedSavePose)
	ON_BN_CLICKED(IDC_DeletePose, &CBrainStimSerialArmDlg::OnBnClickedDeletepose)
	ON_BN_CLICKED(IDC_Stimulate, &CBrainStimSerialArmDlg::OnBnClickedStimulate)
	ON_BN_CLICKED(IDC_LoadPose, &CBrainStimSerialArmDlg::OnBnClickedLoadpose)
	ON_BN_CLICKED(IDC_ExtractPose, &CBrainStimSerialArmDlg::OnBnClickedExtractpose)
END_MESSAGE_MAP()
// CBrainStimSerialArmDlg 메시지 처리기
LRESULT CBrainStimSerialArmDlg::OnUpdateData(WPARAM wParam, LPARAM lParam)// Encoder update
{
	UpdateData(FALSE);
	return 0;
} 
UINT updateThread(LPVOID p) // OpenDevice 버튼 클릭 시 생성됨, 엔코더 값 확인, 모터의 enable 상태에 따라서 버튼 클릭 가능/불가능 
{
	CBrainStimSerialArmDlg *dlg = (CBrainStimSerialArmDlg*)p;
	int i; int nCnt;  // 리스트 전체 개수
	for(;;)
	{
		MaxonLib *maxon = new MaxonLib;	
		maxon->getStatus(MaxonHandle,dlg->InitNodeId,dlg->FinalNodeId); //메모리 문제 해결하면 주석 풀기 17.09.06
		dlg->m_encoder1 = maxon->JointValue[0];		dlg->m_encoder2 = maxon->JointValue[1];
		dlg->m_encoder3 = maxon->JointValue[2];		dlg->m_encoder4 = maxon->JointValue[3];
		dlg->m_encoder5 = maxon->JointValue[4];		dlg->m_encoder6 = maxon->JointValue[5];

		dlg->SendMessage(UM_UPDATE); // 스레드 내에서 UpdateData함수 사용이 힘들어서 SendMessage 사용

		// 파이프에서 전달받은 마커 flag 데이터
		if(bFlag[2]) // 로봇 베이스 마커를 읽었을 때
		{dlg->RB_ledCtrl.SetLedState(LED_BUTTON_STATE_ON);}
		if(!bFlag[2])
		{dlg->RB_ledCtrl.SetLedState(LED_BUTTON_STATE_OFF);}
		if(bFlag[3]) // 헤드 마커를 읽었을 때
		{dlg->H_ledCtrl.SetLedState(LED_BUTTON_STATE_ON);}		
		if(!bFlag[3])
		{dlg->H_ledCtrl.SetLedState(LED_BUTTON_STATE_OFF);}

		nCnt = dlg->m_CListCtrl.GetItemCount();						
		for( i=0 ; i < nCnt ; i++)
		{
			if( dlg->m_CListCtrl.GetItemState(i,LVIS_SELECTED)!= 0 ) // 선택된 리스트 항목 확인
			{
				dlg->m_SystemMode = "Pre-defined Mode";
				dlg->bPredefined = TRUE;
			}
		}
		if(!maxon->bTotalStatus) //모터 6개 중 하나라도 disable인 경우 
		{
			dlg->GetDlgItem( IDC_MoveEndEffector )->EnableWindow(FALSE);	
			dlg->GetDlgItem( IDC_Halt )->EnableWindow(FALSE);	
			dlg->GetDlgItem( IDC_JointControl )->EnableWindow(FALSE);
			dlg->GetDlgItem( IDC_GoHome )->EnableWindow(FALSE);
			dlg->m_SystemMode = "Disable";
		}
		else if( (!dlg->bCompenFlag) && (!dlg->bPredefined) )  //Compensation 버튼 클릭할 때마다 bCompenFlag 바뀜.
		{
			dlg->GetDlgItem( IDC_MoveEndEffector )->EnableWindow(TRUE);	
			dlg->GetDlgItem( IDC_Halt )->EnableWindow(TRUE);
			dlg->GetDlgItem( IDC_JointControl )->EnableWindow(TRUE);
			dlg->GetDlgItem( IDC_GoHome )->EnableWindow(TRUE);
			dlg->m_SystemMode = "General Mode";
		}
		else if( (!dlg->bCompenFlag) && (!dlg->bPredefined) ) 
		{
			dlg->GetDlgItem( IDC_MoveEndEffector )->EnableWindow(TRUE);	
			dlg->GetDlgItem( IDC_Halt )->EnableWindow(TRUE);
			dlg->GetDlgItem( IDC_JointControl )->EnableWindow(TRUE);
			dlg->GetDlgItem( IDC_GoHome )->EnableWindow(TRUE);
			dlg->m_SystemMode = "Pre-defined Mode";
		}
		else if( dlg->bCompenFlag )
		{
			dlg->GetDlgItem( IDC_MoveEndEffector )->EnableWindow(TRUE);	
			dlg->GetDlgItem( IDC_Halt )->EnableWindow(TRUE);
			dlg->GetDlgItem( IDC_JointControl )->EnableWindow(TRUE);
			dlg->GetDlgItem( IDC_GoHome )->EnableWindow(TRUE);
			dlg->m_SystemMode = "Compensation Mode";
		}
		
		if( !bInitEncodr )
		{
			dlg->GetDlgItem( IDC_MoveEndEffector )->EnableWindow(FALSE);		
			dlg->GetDlgItem( IDC_GoHome )->EnableWindow(FALSE);
		}

		if(dlg->bHomePose)
		{
			dlg->GetDlgItem( IDC_robotBaseSet )->EnableWindow(TRUE);
		}		

		if(dlg->bRobotBaseSet)
		{
			dlg->GetDlgItem( IDC_SavePose )->EnableWindow(TRUE);
		}

		delete maxon;
	}
	delete dlg;
	return 0;
}


void CALLBACK CBrainStimSerialArmDlg::Mtimer(UINT wTimerID, UINT msg,DWORD dwUser, DWORD dw1, DWORD dw2)  
{  
	CBrainStimSerialArmDlg *pTimer = (CBrainStimSerialArmDlg*) dwUser;	
	MaxonLib *m = new MaxonLib;	
	Kinematics *k = new Kinematics;

	Matrix4f T_RBtoD,T_CtoDesired;


	////////////////////////////

	// 저장된 target 자세 코드 넣을 부분 // 

	////// ------^^------ //////

	k->GetTransform(Target_q0,Target_qx,Target_qy,Target_qz,Target_tx,Target_ty,Target_tz,&T_CtoDesired);
	T_RBtoD = pTimer->T_RBtoC * T_CtoDesired;	

		///  --------------- 오차 측정 --------------- ///
	Matrix4f T_CtoEEMarker, T_CtoEE;
	k->GetTransform(q0[2],qx[2],qy[2],qz[2],tx[2],ty[2],tz[2],&T_CtoEEMarker); // NDI로 E-E 마커 읽은 값

	Matrix4f T_EEMarkerToEE_r,T_EEMarkerToEE_t,T_EEMarkerToEE, T_RBtoEE;	// r: 회전변환, t: 위치변환

		/// TMS ///////
	//pTimer->mx = 54;		pTimer->my =  -67.175;		pTimer->mz = -67.175;
	//pTimer->mrz = 0;		pTimer->mry = 0;		pTimer->mrx = 135;
		// Ultrasound //
	//pTimer->mx = pTimer->ml*cos(pTimer->mTheta);		pTimer->my = 0;			pTimer->mz = -pTimer->ml*sin(pTimer->mTheta);
	//pTimer->mrz = 0;									pTimer->mry = -45;		pTimer->mrx = -90;
		// Mini-TMS //
	//pTimer->mx = 58;		pTimer->my =  0;		pTimer->mz = -95;
	//pTimer->mrz = 0;		pTimer->mry = 0;		pTimer->mrx = 0;

	//k->Translation(pTimer->mx,pTimer->my,pTimer->mz,&T_EEMarkerToEE_t);			k->Rotation(pTimer->mrz,pTimer->mry,pTimer->mrx,&T_EEMarkerToEE_r);	
	//T_EEMarkerToEE = T_EEMarkerToEE_t * T_EEMarkerToEE_r;	T_CtoEE = T_CtoEEMarker * T_EEMarkerToEE;

	T_EEMarkerToEE = pTimer->T_EEMarkerToD; //  T_EEMarkerToD 은 SavePose 함수에서 존재함. 그러나 위에 나온 치수들과 비교해봐야함. 현재 로봇 조립 전이라 한계. 2018.05.29
	T_CtoEE = T_CtoEEMarker		*	T_EEMarkerToEE;
	T_RBtoEE = pTimer->T_RBtoC	*	T_CtoEE;		
	
	/// Position error
	Vector3d err_p(T_RBtoD.coeff(0,3) - T_RBtoEE.coeff(0,3),T_RBtoD.coeff(1,3) - T_RBtoEE.coeff(1,3),T_RBtoD.coeff(2,3) - T_RBtoEE.coeff(2,3));
	// ************** 아래 코드로 해보고 되면 위에 내용 지우고 아래로 사용하자.
	// Vector3f err_p;
	// err_p << T_RBtoD.block(0,3,3,1) - T_RBtoEE.block(0,3,3,1) ;
	
	/// Orientation error
	Matrix3f R_RBtoEE,R_err;
	R_RBtoEE = T_RBtoEE.block<3,3>(0,0).transpose();	R_err = R_RBtoEE*T_RBtoD.block<3,3>(0,0);
	Orien ori;	k->GetEuler(&R_err,&ori);
	Vector3d err_o(ori.rz,ori.ry,ori.rx);

	/// E-E Numerical 미분, 피드백 부분에 Damping term에 사용됨. //
	double diff_px,diff_py,diff_pz;
	diff_px = (T_RBtoEE.coeff(0,3) - pTimer->bef_px) / pTimer->samplingTime;
	diff_py = (T_RBtoEE.coeff(1,3) - pTimer->bef_py) / pTimer->samplingTime;
	diff_pz = (T_RBtoEE.coeff(2,3) - pTimer->bef_pz) / pTimer->samplingTime;	
	///  ^^^^^^^^^^^ 오차 측정 ^^^^^^^^^^^ //


	// 피드백 부분 // 
	// 마커가 둘다 보일 때만 피드백하기.
	if((bFlag[2])&&(bFlag[3])) 
	{
		T_RBtoD.coeffRef(0,3) = Kp*T_RBtoD.coeff(0,3) + Ki*err_p.coeff(0)		- Kd*diff_px;	
		T_RBtoD.coeffRef(1,3) = Kp*T_RBtoD.coeff(1,3) + Ki*err_p.coeff(1)		- Kd*diff_py;	
		T_RBtoD.coeffRef(2,3) = Kp*T_RBtoD.coeff(2,3) + Ki_Z_axis*err_p.coeff(2) - Kd*diff_pz;	
	}
	// E-E 속도를 Numerical 구하기 위해 // 
	pTimer->bef_px = T_RBtoEE.coeff(0,3);	pTimer->bef_py = T_RBtoEE.coeff(1,3);	pTimer->bef_pz = T_RBtoEE.coeff(2,3);

	// Desired 값의 변화 구하기, 임계점 도달하면 피드백을 멈추기 위해, 그렇지 않으면 진동 발생 //
	pTimer->diff_target = sqrt((pTimer->bef_Target_tx - Target_tx)*(pTimer->bef_Target_tx - Target_tx)+(pTimer->bef_Target_ty - Target_ty)*(pTimer->bef_Target_ty - Target_ty)+(pTimer->bef_Target_tz - Target_tz)*(pTimer->bef_Target_tz - Target_tz));
	pTimer->bef_Target_tx = Target_tx;	pTimer->bef_Target_ty = Target_ty;	pTimer->bef_Target_tz = Target_tz;
				
	// Desired 명령 //
	Pose PoseD;	k->GetEuler(&T_RBtoD,&PoseD);
	Ikjoint InvK;		k->ik(&PoseD,&InvK);

	if( timerCount%1000 == 0 )
	{	
		cout << "-----------------------------------------------------------"<<endl;
		cout << "Desired target w.r.t RB coordinate : "<< endl << T_RBtoD << endl << endl;
		cout << "Orientation(Euler Angle, z-y-x ) : "<< PoseD.rz<< "  " <<PoseD.ry <<"  " <<PoseD.rx<< endl <<endl;
		cout << "Position(x y z) :" << PoseD.x<<" " << PoseD.y<<" " << PoseD.z <<endl;
		cout << "-----------------------------------------------------------"<<endl;
		cout << "# Joint Value #" <<endl;
		cout << InvK.InvJoint[0] << " , "<< InvK.InvJoint[1] << " , "<<InvK.InvJoint[2] << " , "<< InvK.InvJoint[3] << " , "<< InvK.InvJoint[4]<< " , " << InvK.InvJoint[5] << endl<< endl;
	}
	if( timerCount%100 == 0 )
	{
		cout << "-----------------------------------------------------------"<<endl;
		cout << "#Position error# "<<endl<<" x: "<<err_p.coeff(0) << ", y: "<<err_p.coeff(1) << ", z: "<< err_p.coeff(2)<<endl;
		cout<< " Norm :" << err_p.norm() << "[mm]"<<endl<<endl<<endl;
	}
	pTimer->m_error =  err_p.norm();
	pTimer->m_Orierror =  err_o.norm();
	if(bTargetFlag) // bTargetFlag : 네비게이션 시스템에서 전달받음
	{
		int i;
		if( (30 < pTimer->m_error)&&( pTimer->m_error < 150 ) )
		{
			if(InvK.bIKFlag)
			{
				for(i=1;i<7;i++)
				{if(i == 3)
					{	m->Move(MaxonHandle,i,InvK.InvJoint[i-1],  err_p.norm() * 40 , err_p.norm() * 10 , 1);
						continue;
					}
					m->Move(MaxonHandle,i,InvK.InvJoint[i-1],  err_p.norm() * 10 , err_p.norm() * 10 , 1);
				}//for문 종료
			}
		}
		else if((5 < pTimer->m_error)&&( pTimer->m_error< 30.1))
		{
			//pTimer->OnBnClickedrobotbaseset();
			if(InvK.bIKFlag)
			{
				for(i=1;i<7;i++)
				{if(i == 3)
					{	m->Move(MaxonHandle,i,InvK.InvJoint[i-1],  err_p.norm() * 20 , err_p.norm() * 30 , 1);
						continue;
					}
					m->Move(MaxonHandle,i,InvK.InvJoint[i-1],  err_p.norm() * 10 , err_p.norm() * 20 , 1);
				}//for문 종료
			}
		}
		else if((1.3 < pTimer->m_error)&&( pTimer->m_error< 5.1))
		{
			if(InvK.bIKFlag)
			{for(i=1;i<7;i++)
				{if(i == 3)
					{	m->Move(MaxonHandle,i,InvK.InvJoint[i-1],  err_p.norm() * 20 , err_p.norm() * 30 , 1);
						continue;
					}
					m->Move(MaxonHandle,i,InvK.InvJoint[i-1],  err_p.norm() * 10 , err_p.norm() * 20 , 1);
				}//for문 종료
			}
		}
		else
		{
			for(i=1;i<7;i++)
			{
				m->Halt(MaxonHandle,i);
			}
		}
		if( timerCount%100 == 0 )
		{cout << "-----------------------------------------------------------"<<endl<<endl;}		
	}//bTargetFlag 조건문 종료
	
	timerCount++;
	delete k,m;
}  

UINT FileSaveThread(LPVOID p)
{	
	int datasize = sizeof(data)/sizeof(int);

	for(;;)
	{
		if( file_flag == 0 ) // 저장이 아직 안되었다면 flag 0.
		{
			if(data_count > datasize-1) 
			{
				ofstream file_writer("data.txt");
			
				if(!file_writer.is_open())
				{
					cout<<"Could not open file!" <<endl;
					return 0;
				}
				for(int i=0; i<datasize ; i++)
				{
					file_writer << data[i] << " " << data2[i] << " "<< data3[i] << " "<< data4[i] << " "<< data5[i] << " "<< data6[i]<<endl;
				}
				cout<<"Data file was saved." <<endl;
				file_flag = 1;
				break;
			}
			break;
		}

		if(file_flag == 2) // 스레드 종료
			return 0;
	}
	return 0;
}
BOOL CBrainStimSerialArmDlg::OnInitDialog()
{
	CDialogEx::OnInitDialog();	ASSERT((IDM_ABOUTBOX & 0xFFF0) == IDM_ABOUTBOX);
	ASSERT(IDM_ABOUTBOX < 0xF000);	CMenu* pSysMenu = GetSystemMenu(FALSE);
	if (pSysMenu != NULL)
	{	BOOL bNameValid;CString strAboutMenu;bNameValid = strAboutMenu.LoadString(IDS_ABOUTBOX);ASSERT(bNameValid);
		if (!strAboutMenu.IsEmpty())
		{	pSysMenu->AppendMenu(MF_SEPARATOR);	pSysMenu->AppendMenu(MF_STRING, IDM_ABOUTBOX, strAboutMenu);
		}
	}
	SetIcon(m_hIcon, TRUE);			
	SetIcon(m_hIcon, FALSE);		

	InitNodeId = 1;
	FinalNodeId = 6;
	bCompenFlag = FALSE; 	
	bRecordFlag = FALSE;
	bConstraint = TRUE;
	bPredefined = FALSE;
	bRobotBaseSet = FALSE;
	bHomePose = FALSE;
	samplingTime = 50 ; //20Hz, 0.05초
	diff_target = 0;
	fill_n(intgrErr,6,0);
	fill_n(befq,6,0);
		 
	RB_ledCtrl.SetIcons(IDI_RED_LED, IDI_BLUE_LED);
	RB_ledCtrl.SetIcon(LED_BUTTON_STATE_OFF, IDI_RED_LED, 14,14);
	RB_ledCtrl.SetIcon(LED_BUTTON_STATE_ON, IDI_BLUE_LED, 14,14);	
	H_ledCtrl.SetIcons(IDI_RED_LED, IDI_BLUE_LED);
	H_ledCtrl.SetIcon(LED_BUTTON_STATE_OFF, IDI_RED_LED, 14,14);
	H_ledCtrl.SetIcon(LED_BUTTON_STATE_ON, IDI_BLUE_LED, 14,14);

	if(tms_arduino.isConnected()) cout << "Connection between TMS and Robot was established." << endl;
	else cout << "ERROR, check TMS port name..";
	if (emg_arduino.isConnected()) cout << "Connection between EMG and Robot was established." << endl;	
	else cout << "ERROR, check EMG port name..";

	m_Depth.SetRange(0,150);
	m_Depth.SetPos(100);
	m_Depth.SetTicFreq(1);
	//m_Depth.SetSelection(10,140);
	
	 //메뉴
	LVCOLUMN LC;
	LC.mask = LVCF_FMT | LVCF_SUBITEM | LVCF_TEXT | LVCF_WIDTH;	LC.fmt = LVCFMT_CENTER;
	LC.iSubItem = 0;	LC.pszText = _T("Num");	LC.cx = 55;
	m_CListCtrl.InsertColumn(0,&LC);	
	LC.iSubItem = 1;	LC.pszText = _T("X [mm]");	LC.cx = 95;
	m_CListCtrl.InsertColumn(1,&LC);
	LC.iSubItem = 2;	LC.pszText = _T("Y [mm]");	LC.cx = 95;
	m_CListCtrl.InsertColumn(2,&LC);
	LC.iSubItem = 3;	LC.pszText = _T("Z [mm]");	LC.cx = 95;
	m_CListCtrl.InsertColumn(3,&LC);
	LC.iSubItem = 4;	LC.pszText = _T("Rz");	LC.cx = 90;
	m_CListCtrl.InsertColumn(4,&LC);
	LC.iSubItem = 5;	LC.pszText = _T("Ry");	LC.cx = 90;
	m_CListCtrl.InsertColumn(5,&LC);
	LC.iSubItem = 6;	LC.pszText = _T("Rx");	LC.cx = 90;
	m_CListCtrl.InsertColumn(6,&LC);	
	m_CListCtrl.SetExtendedStyle(LVS_EX_FULLROWSELECT); // 행 전체 선택

	ifstream in("InitialSetting.txt");
	if(in.is_open())
	{
		string s,strTRUE,strFALSE;		char num;
		strTRUE = "TRUE";
		strFALSE = "FALSE";
		in.seekg(1,ios::beg);		int size_beg = in.tellg();
		in.seekg(0,ios::beg);		in.read(&num,size_beg);		getline(in,s);
		getline(in,s);
		MaxonLib *m = new MaxonLib;
		if(s.compare(strTRUE))
		{
			bInitEncodr = TRUE; // 모터 초기화 셋팅 후 디버깅 재시작 시 사용하기 위해	
			if(MaxonHandle = VCS_OpenDeviceDlg(&(m->m_ulErrorCode))){
			//cout << MaxonHandle;
			AfxBeginThread( updateThread , this); // PIPE를 통한 마커 표시등 확인 기능도 함께 있음.
			cout << "It has already been set." << endl;
			}
			else
			{
				cout << "Again Start." << endl;
				GetDlgItem( IDC_MoveEndEffector )->EnableWindow(FALSE);				GetDlgItem( IDC_EnableMotor )->EnableWindow(FALSE);
				GetDlgItem( IDC_DisableMotor )->EnableWindow(FALSE);				GetDlgItem( IDC_Halt )->EnableWindow(FALSE);
				GetDlgItem( IDC_JointControl )->EnableWindow(FALSE);				GetDlgItem( IDC_GoHome )->EnableWindow(FALSE);
				GetDlgItem( IDC_SavePose )->EnableWindow(FALSE);				GetDlgItem( IDC_robotBaseSet )->EnableWindow(FALSE);
				bInitEncodr = FALSE; // 모터 초기화 셋팅 후 디버깅 재시작 시 사용하기 위해
			}
		}
		else if(s.compare(strFALSE))
		{
			GetDlgItem( IDC_MoveEndEffector )->EnableWindow(FALSE);			GetDlgItem( IDC_EnableMotor )->EnableWindow(FALSE);
			GetDlgItem( IDC_DisableMotor )->EnableWindow(FALSE);			GetDlgItem( IDC_Halt )->EnableWindow(FALSE);
			GetDlgItem( IDC_JointControl )->EnableWindow(FALSE);			GetDlgItem( IDC_GoHome )->EnableWindow(FALSE);
			GetDlgItem( IDC_SavePose )->EnableWindow(FALSE);			GetDlgItem( IDC_robotBaseSet )->EnableWindow(FALSE);
			bInitEncodr = FALSE; // 모터 초기화 셋팅 후 디버깅 재시작 시 사용하기 위해
		}
		in.close();
	}
	else
	{
		GetDlgItem( IDC_MoveEndEffector )->EnableWindow(FALSE);			GetDlgItem( IDC_EnableMotor )->EnableWindow(FALSE);
		GetDlgItem( IDC_DisableMotor )->EnableWindow(FALSE);			GetDlgItem( IDC_Halt )->EnableWindow(FALSE);
		GetDlgItem( IDC_JointControl )->EnableWindow(FALSE);			GetDlgItem( IDC_GoHome )->EnableWindow(FALSE);
		GetDlgItem( IDC_SavePose )->EnableWindow(FALSE);			GetDlgItem( IDC_robotBaseSet )->EnableWindow(FALSE);
		bInitEncodr = FALSE; // 모터 초기화 셋팅 후 디버깅 재시작 시 사용하기 위해
	}
	return TRUE;  
}

void CBrainStimSerialArmDlg::OnSysCommand(UINT nID, LPARAM lParam)
{
	if ((nID & 0xFFF0) == IDM_ABOUTBOX)
	{
		CAboutDlg dlgAbout;
		dlgAbout.DoModal();
	}
	else
	{
		CDialogEx::OnSysCommand(nID, lParam);
	}
}
void CBrainStimSerialArmDlg::OnPaint()
{
	if (IsIconic())
	{
		CPaintDC dc(this); 	SendMessage(WM_ICONERASEBKGND, reinterpret_cast<WPARAM>(dc.GetSafeHdc()), 0);
		int cxIcon = GetSystemMetrics(SM_CXICON);		int cyIcon = GetSystemMetrics(SM_CYICON);
		CRect rect;		GetClientRect(&rect);		int x = (rect.Width() - cxIcon + 1) / 2;
		int y = (rect.Height() - cyIcon + 1) / 2;		dc.DrawIcon(x, y, m_hIcon);
	}
	else
	{
		CDialogEx::OnPaint();
	}
}
HCURSOR CBrainStimSerialArmDlg::OnQueryDragIcon()
{
	return static_cast<HCURSOR>(m_hIcon);
}

UINT movingCheck(LPVOID p)
{
	////struct IKJoint InvK = new ;
	////IKJoint *i = new IKJoint;
	////i = (IKJoint)p;
	////long lvalue = (long)p;
	Ikjoint *ikjoint = (Ikjoint*)p;
	MaxonLib *m = new MaxonLib;
	//while(m->bMovingCheck == 1) 
	//{
		//m->MovingCheck(MaxonHandle);
	//}
	//m->Move(MaxonHandle, 3, ikjoint->InvJoint[2] , 400, 1);
	////Sleep(2000);
	return 0;
}
// Maxon motor connection 
void CBrainStimSerialArmDlg::OnBnClickedOpendevice()
{
	BOOL bOpen;
	MaxonLib *m = new MaxonLib;
 	bOpen = m->OpenDevice(InitNodeId,FinalNodeId); 
	if(bOpen)
	{
		MaxonHandle = m->m_KeyHandle; // Success -> Get a MaxonHandle 
		GetDlgItem( IDC_EnableMotor )->EnableWindow(TRUE);
		GetDlgItem( IDC_DisableMotor )->EnableWindow(TRUE);
		AfxBeginThread( updateThread , this); // PIPE를 통한 마커 표시등 확인 기능도 함께 있음.
	}
	delete m;
}
// Motor Enable 
void CBrainStimSerialArmDlg::OnBnClickedEnablemotor()
{
	BOOL bEnable;
	MaxonLib *m = new MaxonLib;
	bEnable = m->Enable(MaxonHandle,InitNodeId,FinalNodeId); 	
	if(bEnable)
	{
		GetDlgItem( IDC_MoveEndEffector )->EnableWindow(TRUE);
		GetDlgItem( IDC_Halt )->EnableWindow(TRUE);
		GetDlgItem( IDC_JointControl )->EnableWindow(TRUE);
		GetDlgItem( IDC_GoHome )->EnableWindow(TRUE);
	}
	delete m;
}
// Motor Disable 
void CBrainStimSerialArmDlg::OnBnClickedDisablemotor()
{
	BOOL bDisable;
	MaxonLib *m = new MaxonLib;
	bDisable = m->Disable(MaxonHandle,InitNodeId,FinalNodeId);
	
	if(bDisable)
	{
	GetDlgItem( IDC_MoveEndEffector )->EnableWindow(FALSE);
	GetDlgItem( IDC_Halt )->EnableWindow(FALSE);
	GetDlgItem( IDC_JointControl )->EnableWindow(FALSE);
	GetDlgItem( IDC_GoHome )->EnableWindow(FALSE);
	}
	delete m;
}
// Cartesian Control
void CBrainStimSerialArmDlg::OnBnClickedMoveendeffector()
{
	Kinematics *k = new Kinematics;
	MaxonLib *m = new MaxonLib;	
	Ikjoint InvK;	cout << "------------------" << endl;
	if(bPredefined)
	{
		int nCnt = m_CListCtrl.GetItemCount(); // 리스트 전체 개수
		for( int i = 0; i < nCnt ; i++)
		{
			if( m_CListCtrl.GetItemState(i,LVIS_SELECTED)!=0 ) // 선택된 리스트 항목 확인
			{
				Matrix4f T_CtoHMarker,T_RBtoD;

				OnBnClickedrobotbaseset();

				int j = 3; // 3 번이 헤드 마커일 경우
				k->GetTransform(q0[j],qx[j],qy[j],qz[j],tx[j],ty[j],tz[j],&T_CtoHMarker);

				CString strx,stry,strz,strRx,strRy,strRz;	//double x,y,z,rx,ry,rz;
				strx = m_CListCtrl.GetItemText(i,1);	stry = m_CListCtrl.GetItemText(i,2);	strz = m_CListCtrl.GetItemText(i,3);
				strRz = m_CListCtrl.GetItemText(i,4);	strRy = m_CListCtrl.GetItemText(i,5);	strRx = m_CListCtrl.GetItemText(i,6);
				Pose selct_Pose;
				selct_Pose.x = _wtof(strx);	selct_Pose.y = _wtof(stry);	selct_Pose.z = _wtof(strz);
				selct_Pose.rz = _wtof(strRz);	selct_Pose.ry = _wtof(strRy);	selct_Pose.rx = _wtof(strRx);

				k->GetTransform(&selct_Pose,&T_HMarkerToD);	T_RBtoD = T_RBtoC * T_CtoHMarker * T_HMarkerToD;
				
				Pose RBtoD_Pose;
				k->GetEuler(&T_RBtoD,&RBtoD_Pose);					
				k->ik(&RBtoD_Pose,&InvK);
				cout << "Robot base to pre-defined Desired target.  " << endl << endl;
				cout << " Target pose (x,y,z, rz,ry,rx) : "<< RBtoD_Pose.x<<" , "<<RBtoD_Pose.y<<" , "<<RBtoD_Pose.z<<" , "<<RBtoD_Pose.rz<<" , "<<RBtoD_Pose.ry<<" , "<<RBtoD_Pose.rx<<endl;
				cout << " HeadMarker to Desired pose : " << endl << T_HMarkerToD << endl;
			}
		}
		if(InvK.bIKFlag == -858993460) // 초기화 안된 경우
		{
			cout << "Select a pre-defined target." << endl;
			return;
		}
	}
	else if(!bPredefined)
	{
		//k->ik(inputX,inputY,inputZ,inputRz,inputRy,inputRx,&InvK);
		k->ik_adj(inputX,inputY,inputZ,inputRz,inputRy,inputRx,m_FineRotZ,m_FineRotY,m_FineRotX,&InvK);
		cout << " Target pose (x,y,z, rz,ry,rx) : "<< inputX<<" , "<<inputY<<" , "<<inputZ<<" , "<<inputRz<<" , "<<inputRy<<" , "<<inputRx<<endl;
	}
	
	if(InvK.bIKFlag)
	{
		//m->CheckMovement(&InvK);
		cout << " Solution of IK " << endl;
		cout << "Joint 1: " << InvK.InvJoint[0] << "[deg] , Joint 2: " << InvK.InvJoint[1] << "[deg] , Joint 3: " << InvK.InvJoint[2] << "[mm], "<<endl;
		cout << "Joint 4: " << InvK.InvJoint[3] << "[deg] , Joint 5: " << InvK.InvJoint[4] << "[deg] , Joint 6: " << InvK.InvJoint[5] <<"[deg]"<<endl<<endl;
		cout << "------------------" << endl;
		
		for(int i=1;i<7;i++)
		{
			m->Move(MaxonHandle,i,InvK.InvJoint[i-1],1); // absolute
		}
		//AfxBeginThread(movingCheck,&InvK);
	}

	delete k,m;
}
void CBrainStimSerialArmDlg::OnBnClickedGohome()
{
	MaxonLib *m = new MaxonLib;
	Kinematics *k = new Kinematics;
	Ikjoint InvK;
	k->ik(0,0,110,0,90,180,&InvK);
	cout<<endl<<endl<<"Go to Home pose."<<endl<<endl<<endl;
	for(int i=1;i<7;i++)
	{m->Move(MaxonHandle,i,InvK.InvJoint[i-1],1);}
	bHomePose = TRUE;
	delete m;
}
// 모든 모터 멈추기.
void CBrainStimSerialArmDlg::OnBnClickedHalt()
{
	if(bCompenFlag) // 혹시 compensation 버튼이 작동하지 않을경우, HALT 버튼으로 타이머를 꺼야하기 때문에.
	{
		m_SystemMode = "General Mode";
		bCompenFlag = FALSE;
		UpdateData(false);
		bTimer = timeKillEvent(m_idEvent);
		if(bTimer == TIMERR_NOERROR)
			cout << "Timer has exited" << endl;
		timerCount = 0; // Reset the count.
	}

	MaxonLib *m = new MaxonLib;
	for(int i=1;i<7;i++)
	{
		m->Halt(MaxonHandle,i); 
	}
	delete m;
}
// Named Pipe 연결 ( 네비게이션과 연결 - 쓰레드 생성 후 마커 좌표 받아오기 )
void CBrainStimSerialArmDlg::OnBnClickedPipeconnect() 
{
	//AfxBeginThread( updateThread , this); // LED 표시를 위해서.
	namedPipe *pipe = new namedPipe;
	pipe->Connect();
	delete pipe;
	cout << "Connecting with navigation system..." <<endl<<endl;
}
// 카메라를 이용해서 로봇 베이스를 등록하는 과정
void CBrainStimSerialArmDlg::OnBnClickedrobotbaseset()
{
	Kinematics *k = new Kinematics;
	Matrix4f m06;	k->T06(m_encoder1,m_encoder2,m_encoder3,m_encoder4,m_encoder5,m_encoder6,&m06);

	Matrix4f T_EEToEEMarker,T_EEToEEMarker_t,T_EEToEEMarker_r; //T_WristToEEMarker r: 회전변환, t: 위치변환
	
	//	Wrist point부터 E-E Marker까지 //
	/// TMS ///////
	mx = 54; my = -67.175; mz = -67.175;
	mrz = 0; mry = 0; mrx = 135;
	///////////////
	/// Mini-TMS ///////
	//mx = 54; my = -95; mz = 0;
	//mrx = 90;
	///////////////
	/// Ultrasound ///////
	//mx = 54; my = -95; mz = 0;
	//mrz = 45; mry = 0; mrx = 90;
	
	endTofocal = l6 - l_link6; 
	mTheta = 45*D2R - atan(endTofocal/95);
	ml = sqrt(95*95 + endTofocal*endTofocal);	

	///////////////
	k->Translation(mx,my,mz,&T_EEToEEMarker_t);  k->Rotation(mrz,mry,mrx,&T_EEToEEMarker_r);	
	T_EEToEEMarker = T_EEToEEMarker_t * T_EEToEEMarker_r;
	
	Matrix4f T_EEMarkerToC , T_CtoEEMarker ,T_CtoHMarker , T_RBtoH , T_RBtoEEmarker; 
	int i = 2; // 2 : 베이스 마커
	k->GetTransform(q0[i],qx[i],qy[i],qz[i],tx[i],ty[i],tz[i],&T_CtoEEMarker);	k->Reverse(&T_CtoEEMarker,&T_EEMarkerToC);
	cout << "From Robot Base Marker to NDI : "<< endl << T_EEMarkerToC << endl;
	T_RBtoC = m06 * T_EEToEEMarker  * T_EEMarkerToC;
	T_RBtoEEmarker = m06 * T_EEToEEMarker;
	cout << "From Robot Base to E-E marker : "<< endl << T_RBtoEEmarker << endl;
	

	if(T_EEMarkerToC == MatrixXf::Identity(4,4))
	{
		cout << "----------------------" << endl;
		cout << " Registration failed." << endl;
		cout << "----------------------" << endl;
		return;
	}
	
	i = 3; // 3 : 헤드 마커
	k->GetTransform(q0[i],qx[i],qy[i],qz[i],tx[i],ty[i],tz[i],&T_CtoHMarker);
	//Matrix4f T_RBtoEEMarker;T_RBtoEEMarker = m06*T_EEToEEMarker;cout << "From Robot Base to E-E Marker : "<< endl << T_RBtoEEMarker << endl;
	cout << "From Robot Base to NDI : "<< endl << T_RBtoC << endl;
	cout << endl << " Setting the Robot Base..." << endl << "..." <<endl <<"..."<<endl;
	cout << "Robot base registration was completed." << endl << "Waiting for a target pose..." << endl;

	bRobotBaseSet = TRUE;
	UpdateData(false);
	delete k;
}
void CBrainStimSerialArmDlg::OnBnClickedCompensation()
{
	CString str,str1,str2,str3,str4;
	str1 = "Compensation Mode";
	str2 = "General Mode";
	str3 = "Pre-defined Mode";
	str4 = "Disable";

	GetDlgItemText(IDC_SystemMode, str);
	 //str.Compare("Compensation Mode") )   <--이 부분 공부하고, 수정하자. str1,2 없애기 위해
	
	unsigned int timerResolution;

	if( str == str1 ) // Compensation Mode -> General Mode로 변경할 때  
	{
		bCompenFlag = FALSE;
		bTimer = timeKillEvent(m_idEvent);
		m_SystemMode = "General Mode";
		UpdateData(false);
		
		MaxonLib *m = new MaxonLib;
		for(int i=1;i<7;i++)
		{m->Halt(MaxonHandle,i);}
		delete m;

		if(bTimer == TIMERR_NOERROR)
			cout << "Timer has exited" << endl;
		timerCount = 0; // reset the count.
	}

	else if(( str == str2 ) || (str == str3))// General Mode 이거나 Pre-defined Mode 경우  
	{
		cout << "-------- Head Compensation --------"<<endl;
		bCompenFlag = TRUE;   
		//System에서 가능한 Resolution을 구한다.멀티미디어 타이머의 Resolution을 최소로 하기 위한 코드    
		TIMECAPS tc;	timeGetDevCaps(&tc, sizeof(TIMECAPS));  timerResolution = min(max(tc.wPeriodMin, 0), tc.wPeriodMax);  
		timeBeginPeriod(timerResolution);  
		//타이머 생성  
		m_idEvent = timeSetEvent(samplingTime,timerResolution,Mtimer,(DWORD)this,TIME_PERIODIC);		
		m_SystemMode = "Compensation Mode";
		UpdateData(false);
		//AfxBeginThread(FileSaveThread,NULL); //데이터 출력을 위해서 실행하는 스레드
	}
	else if( str == str4 ) // Disable 모드일 때, 타이머 종료하기.
	{
		cout << "-------- Disable, Check state of motors. --------"<<endl;		
		bCompenFlag = FALSE;
		bTimer = timeKillEvent(m_idEvent);
		m_SystemMode = "Disable";
		UpdateData(false);
		
		MaxonLib *m = new MaxonLib;
		for(int i=1;i<7;i++)
		{m->Halt(MaxonHandle,i);}
		delete m;

		if(bTimer == TIMERR_NOERROR)
			cout << "Timer has exited" << endl;
		timerCount = 0; // reset the count.
	}

}




void CBrainStimSerialArmDlg::OnBnClickedOk()
{
	_CrtDumpMemoryLeaks();
	CDialogEx::OnOK();
	//bInitEncodr = TRUE; // 모터 초기화 셋팅 후 디버깅 재시작 시 사용하기 위해
}

void CBrainStimSerialArmDlg::OnBnClickedDatarecorder()
{
	//MaxonLib *m = new MaxonLib;
	//m->DataRecord(MaxonHandle);
	//bRecordFlag = TRUE;
	DWORD mError;
	VCS_DefinePosition(MaxonHandle,6,0,&mError);

}

void CBrainStimSerialArmDlg::OnBnClickedDatarecorderstop()
{
	//MaxonLib *m = new MaxonLib;
	//m->StopRecord(MaxonHandle);
	
	bRecordFlag = FALSE;
	ofstream end_effector;
	end_effector.open("end_effector.txt");
	for(int i=0;i<10000;i++)
	{end_effector << data[i] << " " << data2[i] << " " << data3[i] << "\n" ;}

}

void CBrainStimSerialArmDlg::OnBnClickedUpbutton() // latitude up
{
	Kinematics *k = new Kinematics;
	SpheCoordi sphe;
	
	Matrix4f T03,T0e;

	if(bCompenFlag){
	/*	cout<<"During compensation mode, the target was changed."<<endl;

	k->CarteToSphe(Target_tx,Target_ty,Target_tz,&des_sphe);
	double thetad;
	thetad = des_sphe.thetad - 1;
	Pos new_pos;
	k->SpheToCarte(des_sphe.R,des_sphe.phid,thetad,&new_pos);
	Target_tx = new_pos.x;
	Target_ty = new_pos.y;
	Target_tz = new_pos.z;*/
	}

	else if(!bCompenFlag){
		/*
		k->T0e(m_encoder1,m_encoder2,m_encoder3,m_encoder4,m_encoder5,m_encoder6,&T0e);
		k->CarteToSphe(T0e(0,3),T0e(1,3),T0e(2,3),&sphe);
		cout << " Up button " << endl;
		double thetad;
		thetad = sphe.thetad - 1 ;
		Pos new_pos;
		k->SpheToCarte(sphe.R, sphe.phid, thetad, &new_pos);
		cout << new_pos.x << " , " << new_pos.y << " , " << new_pos.z <<endl;
		Ikjoint InvK;
		k->ik(new_pos.x,new_pos.y,new_pos.z,&InvK);
		cout << " Solution of IK " << endl;
		cout << "Joint 1: " << InvK.InvJoint[0] << "[deg] , Joint 2: " << InvK.InvJoint[1] << "[deg] , Joint 3: " << InvK.InvJoint[2] << "[mm], "<<endl;
		cout << "Joint 4: " << InvK.InvJoint[3] << "[deg] , Joint 5: " << InvK.InvJoint[4] << "[deg] , Joint 6: " << InvK.InvJoint[5] <<"[deg]"<<endl<<endl;
		MaxonLib *m = new MaxonLib;
		if(InvK.bIKFlag)
			{
				for(int i=1;i<7;i++)
				{
					if(i==4)
						continue;
					m->Move(MaxonHandle,i,InvK.InvJoint[i-1],100,500,1); // absolute
				}
			}

		*/
		
		k->T03(m_encoder1,m_encoder2,m_encoder3,&T03);
		k->CarteToSphe(T03(0,3),T03(1,3),T03(2,3),&sphe);
		cout << " Up button " << endl;
		double thetad;
		thetad = sphe.thetad - 1 ;
		Pos new_pos;
		k->SpheToCarte(sphe.R, sphe.phid, thetad, &new_pos);
		cout << new_pos.x << " , " << new_pos.y << " , " << new_pos.z <<endl;
		Ikjoint InvK;
		k->ik(new_pos.x,new_pos.y,new_pos.z,&InvK);
		cout << " Solution of IK " << endl;
		cout << "Joint 1: " << InvK.InvJoint[0] << "[deg] , Joint 2: " << InvK.InvJoint[1] << "[deg] , Joint 3: " << InvK.InvJoint[2] << "[mm], "<<endl;
		cout << "Joint 4: " << InvK.InvJoint[3] << "[deg] , Joint 5: " << InvK.InvJoint[4] << "[deg] , Joint 6: " << InvK.InvJoint[5] <<"[deg]"<<endl<<endl;
		MaxonLib *m = new MaxonLib;
		if(InvK.bIKFlag)
			{
				for(int i=1;i<4;i++)
				{
					m->Move(MaxonHandle,i,InvK.InvJoint[i-1],100,500,1); // absolute
				}
			}
		
		else
				cout << "Can't."<< endl;
		delete m,k;
	}
}

void CBrainStimSerialArmDlg::OnBnClickedDownbutton() // latitude down
{
	Kinematics *k = new Kinematics;
	SpheCoordi sphe;	
	Matrix4f T03,T0e;

	if(bCompenFlag){
	/*cout<<"During compensation mode, the target was changed."<<endl;
	k->CarteToSphe(Target_tx,Target_ty,Target_tz,&sphe);
	double thetad;
	thetad = sphe.thetad - 1;
	Pos new_pos;
	k->SpheToCarte(sphe.R,sphe.phid,thetad,&new_pos);
	Target_tx = new_pos.x;	Target_ty = new_pos.y;	Target_tz = new_pos.z;
	*/
	}


	else if(!bCompenFlag){

		/*
		k->T0e(m_encoder1,m_encoder2,m_encoder3,m_encoder4,m_encoder5,m_encoder6,&T0e);
		k->CarteToSphe(T0e(0,3),T0e(1,3),T0e(2,3),&sphe);
		cout << " Up button " << endl;
		double thetad;
		thetad = sphe.thetad + 1 ;
		Pos new_pos;
		k->SpheToCarte(sphe.R, sphe.phid, thetad, &new_pos);
		cout << new_pos.x << " , " << new_pos.y << " , " << new_pos.z <<endl;
		Ikjoint InvK;
		k->ik(new_pos.x,new_pos.y,new_pos.z,&InvK);
		cout << " Solution of IK " << endl;
		cout << "Joint 1: " << InvK.InvJoint[0] << "[deg] , Joint 2: " << InvK.InvJoint[1] << "[deg] , Joint 3: " << InvK.InvJoint[2] << "[mm], "<<endl;
		cout << "Joint 4: " << InvK.InvJoint[3] << "[deg] , Joint 5: " << InvK.InvJoint[4] << "[deg] , Joint 6: " << InvK.InvJoint[5] <<"[deg]"<<endl<<endl;
		MaxonLib *m = new MaxonLib;
		if(InvK.bIKFlag)
			{
				for(int i=1;i<7;i++)
				{
					if(i==4)
						continue;
					m->Move(MaxonHandle,i,InvK.InvJoint[i-1],100,500,1); // absolute
				}
			}
		*/
		
		k->T03(m_encoder1,m_encoder2,m_encoder3,&T03);
		k->CarteToSphe(T03(0,3),T03(1,3),T03(2,3),&sphe);
		cout << " Down button " << endl;
		double thetad;
		thetad = sphe.thetad + 1 ;
		Pos new_pos;
		k->SpheToCarte(sphe.R, sphe.phid, thetad, &new_pos);
		Ikjoint InvK;
		k->ik(new_pos.x,new_pos.y,new_pos.z,&InvK);
		cout << " Solution of IK " << endl;
		cout << "Joint 1: " << InvK.InvJoint[0] << "[deg] , Joint 2: " << InvK.InvJoint[1] << "[deg] , Joint 3: " << InvK.InvJoint[2] << "[mm], "<<endl;
		cout << "Joint 4: " << InvK.InvJoint[3] << "[deg] , Joint 5: " << InvK.InvJoint[4] << "[deg] , Joint 6: " << InvK.InvJoint[5] <<"[deg]"<<endl<<endl;
		MaxonLib *m = new MaxonLib;
		if(InvK.bIKFlag)
			{
				for(int i=1;i<4;i++)
				{
					m->Move(MaxonHandle,i,InvK.InvJoint[i-1],100,500,1); // absolute
				}
			}
		
		else
				cout << "Can't."<< endl;
		delete m,k;
	}
}

void CBrainStimSerialArmDlg::OnBnClickedLeftbutton() // longitude 
{
	Kinematics *k = new Kinematics;
	SpheCoordi sphe;	
	Matrix4f T03,T0e;

	if(bCompenFlag){
		/*
	cout<<"During compensation mode, the target was changed."<<endl;
	k->CarteToSphe(Target_tx,Target_ty,Target_tz,&sphe);
	double phid;
	phid = sphe.phid + 1;
	Pos new_pos;
	k->SpheToCarte(sphe.R,phid,sphe.thetad,&new_pos);
	Target_tx = new_pos.x;	Target_ty = new_pos.y;	Target_tz = new_pos.z;
	*/
	}	
		else if(!bCompenFlag){
		k->T03(m_encoder1,m_encoder2,m_encoder3,&T03);
		k->CarteToSphe(T03(0,3),T03(1,3),T03(2,3),&sphe);
		cout << " Left button " << endl;		
		double phid;
		phid =  sphe.phid + 1 ;
		Pos new_pos;
		k->SpheToCarte(sphe.R,	phid,	sphe.thetad,	&new_pos);		
		MaxonLib *m = new MaxonLib;
		Ikjoint InvK;
		k->ik(new_pos.x,new_pos.y,new_pos.z,	phid, (90 + sphe.thetad) , 170 , &InvK);
		cout << " Solution of IK " << endl;
		cout << "Joint 1: " << InvK.InvJoint[0] << "[deg] , Joint 2: " << InvK.InvJoint[1] << "[deg] , Joint 3: " << InvK.InvJoint[2] << "[mm], "<<endl;
		cout << "Joint 4: " << InvK.InvJoint[3] << "[deg] , Joint 5: " << InvK.InvJoint[4] << "[deg] , Joint 6: " << InvK.InvJoint[5] <<"[deg]"<<endl<<endl;
		if(InvK.bIKFlag)
			{
				for(int i=1;i<4;i++)
				{
					m->Move(MaxonHandle,i,InvK.InvJoint[i-1],100,500,1); // absolute
				}
			}
		else
				cout << "Can't."<< endl;
		delete m,k;
	}
}

void CBrainStimSerialArmDlg::OnBnClickedRightbutton()
{
	Kinematics *k = new Kinematics;
	SpheCoordi sphe;	
	Matrix4f T03,T0e;

	if(bCompenFlag){
		/*
	cout<<"During compensation mode, the target was changed."<<endl;
	k->CarteToSphe(Target_tx,Target_ty,Target_tz,&sphe);
	double phid;
	phid = sphe.phid + 1;
	Pos new_pos;
	k->SpheToCarte(sphe.R,phid,sphe.thetad,&new_pos);
	Target_tx = new_pos.x;	Target_ty = new_pos.y;	Target_tz = new_pos.z;
	*/
	}
	else if(!bCompenFlag){		
		k->T03(m_encoder1,m_encoder2,m_encoder3,&T03);
		k->CarteToSphe(T03(0,3),T03(1,3),T03(2,3),&sphe);
		cout << " Right button " << endl;
		double phid;
		phid =  sphe.phid - 1 ;
		Pos new_pos;
		k->SpheToCarte(sphe.R,	phid,	sphe.thetad,	&new_pos);		
		MaxonLib *m = new MaxonLib;
		Ikjoint InvK;
		k->ik(new_pos.x,new_pos.y,new_pos.z,	phid, (90 + sphe.thetad) , 170 , &InvK);
		cout << " Solution of IK " << endl;
		cout << "Joint 1: " << InvK.InvJoint[0] << "[deg] , Joint 2: " << InvK.InvJoint[1] << "[deg] , Joint 3: " << InvK.InvJoint[2] << "[mm], "<<endl;
		cout << "Joint 4: " << InvK.InvJoint[3] << "[deg] , Joint 5: " << InvK.InvJoint[4] << "[deg] , Joint 6: " << InvK.InvJoint[5] <<"[deg]"<<endl<<endl;
		if(InvK.bIKFlag)
			{
				for(int i=1;i<4;i++)
				{
					m->Move(MaxonHandle,i,InvK.InvJoint[i-1],100,500,1); // absolute
				}
			}
		else
				cout << "Can't."<< endl;
		delete m,k;
	}
}

void CBrainStimSerialArmDlg::OnNMReleasedcaptureDepth(NMHDR *pNMHDR, LRESULT *pResult)
{
	m_DepthValue = m_Depth.GetPos();
	MaxonLib *m = new MaxonLib;
	m->Move(MaxonHandle,3,m_DepthValue,400,1); // absolute						
	UpdateData(false);
	*pResult = 0;
	delete m;
}
UINT viveControllerThread(LPVOID p)
{
	CBrainStimSerialArmDlg *dlg = (CBrainStimSerialArmDlg*)p;
	vr::IVRSystem *m_pHMD;
	vr::EVRInitError eError = vr::VRInitError_None;
	m_pHMD = vr::VR_Init(&eError, vr::VRApplication_Scene);	
	
	if(m_pHMD == NULL) // 컨트롤러 연결해도 실행 안되면 이부분 지워야함. 
	{
		return FALSE;
	}

	float cosA,sinA,normA,angleA;	
	uint64_t nowButton,previousButtonState;
	int64_t ToggleButton;
	ToggleButton = 0;
	previousButtonState = 0;

	MaxonLib *m = new MaxonLib;

	cout << " ----------------------- "<<endl;
	cout<<" Start to Controller.. "<<endl;
	cout << " ----------------------- "<<endl;
		while (1)
		{
			for (vr::TrackedDeviceIndex_t unDevice = 0; unDevice < vr::k_unMaxTrackedDeviceCount; unDevice++)
			{
				vr::VRControllerState_t state;
				if (m_pHMD->GetControllerState(unDevice, &state, sizeof(state)))
				{
					m_rbShowTrackedDevice[unDevice] = state.ulButtonPressed == 0;
				}

				normA = sqrt((state.rAxis->x * state.rAxis->x)+(state.rAxis->y * state.rAxis->y));
				cosA = (state.rAxis->x)/normA;	sinA = (state.rAxis->y)/normA;
				angleA = atan2(sinA,cosA);
				if(angleA < -45*D2R){angleA = angleA + 2*M_PI;}

				//ToggleButton = vr::ButtonMaskFromId(vr::k_EButton_ApplicationMenu);
				//if ((state.ulButtonPressed & ToggleButton) && !(previousButtonState & ToggleButton) ) {
				//	cout<<"click"<<endl;					
				//}		
				//previousButtonState = state.ulButtonPressed; 

				if(state.ulButtonPressed == 4294967296) // 엄지 버튼 Click
				{	
					if(  (45*D2R < angleA) && (angleA < 135*D2R)  ){
						dlg->OnBnClickedUpbutton();
					}
					else if (	(225*D2R < angleA) && (angleA < 315*D2R )	){
						dlg->OnBnClickedDownbutton();
					}
					else if (	(135*D2R<angleA) && (angleA < 225*D2R)	){
						dlg->OnBnClickedLeftbutton();
					}
					else if (	(-45*D2R < angleA ) && (angleA < 45*D2R )	){
						dlg->OnBnClickedRightbutton();
					}					
					else
					{
						//cout<<"Can't determine."<<endl;
					}
				}
				else if(state.ulButtonPressed == 8589934592) // 검지 버튼 Click
				{
					if(state.ulButtonTouched == 12884901888) // 터치 패드 감지 
					{
						cout<<"Depth control, Up"<<endl;
						if(dlg->m_encoder3 > 3){
						m->Move(MaxonHandle,3,-1,0); // relative
						}
						else
							cout<<"Can't go up anymore.."<<endl;
					}
					else if(state.ulButtonTouched == 8589934592) // 터치 패드 미 감지 
					{
						cout<<"Depth control, Down"<<endl;			
						if(dlg->m_encoder3 < 149){
						m->Move(MaxonHandle,3,1,0); // relative
						}
						else
							cout<<"Can't go down anymore.."<<endl;
					}
				}
			} // for문 종료
			Sleep(200);
		}// 무한루프
		delete m;
	return 0;
}

void CBrainStimSerialArmDlg::OnBnClickedVivectrl()
{
	AfxBeginThread( viveControllerThread , this); 
}

void CBrainStimSerialArmDlg::OnBnClickedSavePose()
{
	Kinematics *k = new Kinematics;	
	Matrix4f T_CtoHMarker , T_CtoEEMarker, T_HMarkerToC, T_EEMarkerToD_t, T_EEMarkerToD_r , T_DToEEMarker_r , T_DToEEMarker_t , T_DToEEMarker;
	int i;
	
	// 네비게이션 프로그램 - 2:베이스 마커 / 3:헤드 마커
	// NDI 예제 프로그램 - 0:베이스 마커 / 1:헤드 마커

	i = 0; 
	k->GetTransform(q0[i],qx[i],qy[i],qz[i],tx[i],ty[i],tz[i],&T_CtoEEMarker);
	i = 1; 
	k->GetTransform(q0[i],qx[i],qy[i],qz[i],tx[i],ty[i],tz[i],&T_CtoHMarker);
	k->Reverse(&T_CtoHMarker,&T_HMarkerToC);
	
	/// 자극점부터 E-E Marker 까지 변환///////
	/// TMS ///////
	mx = -58; my = -67.175; mz = -67.175;
	mrz = 0; mry = 0; mrx = 135;
	k->Translation(mx,my,mz,&T_DToEEMarker_t);  k->Rotation(mrz,mry,mrx,&T_DToEEMarker_r);	T_DToEEMarker = T_DToEEMarker_t * T_DToEEMarker_r;
	k->Reverse(&T_DToEEMarker,&T_EEMarkerToD);
	
	/// E-E Marker부터 자극점까지 변환///////
	// Mini-TMS //
	//mx = 58; my = 0; mz = -95;
	//mrz = 0; mry = 0; mrx = -90;
	////////// Ultrasound //
	//mx = ml*cos(mTheta); my = 0; mz = -ml*sin(mTheta);
	//mrz = 0; mry = -45; mrx = -90;
	//k->Translation(mx,my,mz,&T_EEMarkerToD_t);  	k->Rotation(mrz,mry,mrx,&T_EEMarkerToD_r);	T_EEMarkerToD = T_EEMarkerToD_t * T_EEMarkerToD_r;
	//////////////////////////
			
	T_HMarkerToD = T_HMarkerToC * T_CtoEEMarker * T_EEMarkerToD;
	// 확인용, 나중에 지우자
	//cout << "T_HMarkerToC : " << T_HMarkerToC << endl;
	//cout << "T_CtoEEMarker : " << T_CtoEEMarker << endl;
	//cout << "T_EEMarkerToD : " << T_EEMarkerToD << endl;
	//cout << T_HMarkerToD << endl;

	
	Pose HMarkerToD_Pose;
	k->GetEuler(&T_HMarkerToD,&HMarkerToD_Pose);

	bPredefined = TRUE;		m_SystemMode = "Pre-defined Mode";		UpdateData(false);		

	int j;	CString strItemCount;
	CString strX,strY,strZ,strRx,strRy,strRz;

	j = m_CListCtrl.GetItemCount();
	strItemCount.Format(_T("%d"), j+1);
	
	strX.Format(_T("%.1f"),HMarkerToD_Pose.x);	strY.Format(_T("%.1f"),HMarkerToD_Pose.y);	strZ.Format(_T("%.1f"),HMarkerToD_Pose.z);
	strRz.Format(_T("%.1f"),HMarkerToD_Pose.rz);	strRy.Format(_T("%.1f"),HMarkerToD_Pose.ry);	strRx.Format(_T("%.1f"),HMarkerToD_Pose.rx);

	LVITEM LI;
	LI.mask = LVIF_TEXT | LVIF_STATE;
	LI.iItem = j;		LI.iSubItem = 0;	LI.state = INDEXTOSTATEIMAGEMASK(1);
	LI.pszText = (LPWSTR)(LPCTSTR)strItemCount;
	m_CListCtrl.InsertItem(&LI);
	
	LI.pszText = (LPWSTR)(LPCTSTR)strX;		m_CListCtrl.SetItemText(j,1,strX);
	LI.pszText = (LPWSTR)(LPCTSTR)strY;		m_CListCtrl.SetItemText(j,2,strY);
	LI.pszText = (LPWSTR)(LPCTSTR)strZ;		m_CListCtrl.SetItemText(j,3,strZ);
	LI.pszText = (LPWSTR)(LPCTSTR)strRz;	m_CListCtrl.SetItemText(j,4,strRz);
	LI.pszText = (LPWSTR)(LPCTSTR)strRy;	m_CListCtrl.SetItemText(j,5,strRy);
	LI.pszText = (LPWSTR)(LPCTSTR)strRx;	m_CListCtrl.SetItemText(j,6,strRx);	
}

void CBrainStimSerialArmDlg::OnBnClickedDeletepose()
{	
	// 단, 한가지씩만 선택되고 삭제 또한 하나씩만 가능하게 함.

	int nCnt = m_CListCtrl.GetItemCount(); // 리스트 전체 개수
	for( int i=0; i < nCnt ; i++)
	{
		if(m_CListCtrl.GetItemState(i,LVIS_SELECTED)!=0) // 선택된 리스트 항목 확인
		{
			m_CListCtrl.DeleteItem(i); // 선택된 항목 삭제
		}
	}
}

void CBrainStimSerialArmDlg::OnBnClickedStimulate()
{
	float emgData[5];
	Ikjoint InvK;
	Pose P_desired;

	P_desired.x = 0; P_desired.y = 0; P_desired.z = 110;
	P_desired.rx = 0; P_desired.ry = 90; P_desired.rz = 180;

	Kinematics *k = new Kinematics;
	MaxonLib *m = new MaxonLib;
	
	for(int j=0;j<3;j++)
	{
		// emgData 배열을 가지고 가장 큰 값으로 위치를 변경시켜야함.
		for(int i=0;i<5;i++)
		{	
			P_desired.x = 0; P_desired.y = i * 5; P_desired.z = 110;
			P_desired.rx = 0; P_desired.ry = 90; P_desired.rz = 180;

			k->ik(&P_desired, &InvK);
			if(InvK.bIKFlag)
			{	for(int i=1;i<7;i++)	
				{	m->Move(MaxonHandle,i,InvK.InvJoint[i-1],1); 	}
			}
			if(m->CheckMovement())	// 모터 상태 확인
			{
				if ((tms_arduino.isConnected()) && (emg_arduino.isConnected())) 
				{
					char *trigger;		char tempEmg[MAX_DATA_LENGTH];
					char temp = '1';	trigger = &temp;
					char *pEmg;		
					emg_arduino.writeSerialPort(trigger, MAX_DATA_LENGTH);
					tms_arduino.writeSerialPort(trigger, MAX_DATA_LENGTH);
					int sizeData = emg_arduino.readSerialPort(tempEmg, MAX_DATA_LENGTH);
					pEmg = strtok(tempEmg,"\n");
					float emg = atof(pEmg);
					cout << " Peak-to-peak(emg) : " << emg << endl;
					emgData[i] = emg;
				}
				else cout << "Connection with arduino was failed."<<endl;
			}
		}//다섯번 반복하는 for문 종료
		cout << j <<"번째 :" << emgData[0] << "," << emgData[1] << "," << emgData[2] <<"," << emgData[3] <<"," << emgData[4] <<endl;
	}
}


void CBrainStimSerialArmDlg::OnBnClickedLoadpose()
{	
	ifstream fileReader("Target_180613.txt");			
	if(!fileReader.is_open())
	{
		cout<<"Could not open file!" <<endl;
	}
	int num = 0;	CString strItemCount; int j = 0;
	string line;
	while(std::getline(fileReader,line)){ // 저장된 파일을 한 줄씩 읽어옴. string 형식
		vector<char> c(line.begin(),line.end());	// string -> char 
		c.push_back('\0');		char* ptr = &c[0];		

		num++; //  한 줄씩 읽을 때마다 저장된 자세의 개수 추가
		strItemCount.Format(_T("%d"), num);
		LVITEM LI;
		LI.mask = LVIF_TEXT | LVIF_STATE;
		LI.iItem = num-1;		LI.iSubItem = 0;	LI.state = INDEXTOSTATEIMAGEMASK(1);
		LI.pszText = (LPWSTR)(LPCTSTR)strItemCount; // 전체 개수 표시
		m_CListCtrl.InsertItem(&LI); 	
	
		char* token = strtok(ptr," ");
		while(token != NULL) {		// char[] 를 " " 기준으로 파싱
			CString strBuffer;
			strBuffer = (LPSTR)token;			
			j++;	// 컬럼 위치 증가
			if(j==7) {j = 1;}			
			LI.pszText = (LPWSTR)(LPCTSTR)strBuffer;		m_CListCtrl.SetItemText(num-1,j,strBuffer);
			token = strtok(NULL," "); // 파싱 시 사용됨
		}		
	}
	fileReader.close();
}

void CBrainStimSerialArmDlg::OnBnClickedExtractpose()
{
	ofstream file_writer("Target_180613.txt");	

	if(!file_writer.is_open())
	{
		cout<<"Could not open file!" <<endl;
	}
	else
	{	
		int nCnt = m_CListCtrl.GetItemCount(); // 리스트 전체 개수

		for(int i = 0; i < nCnt ; i++)
		{
		file_writer << _wtof(m_CListCtrl.GetItemText(i,1)) << " " <<  _wtof(m_CListCtrl.GetItemText(i,2)) << " "<<  _wtof(m_CListCtrl.GetItemText(i,3)) << " "<<  _wtof(m_CListCtrl.GetItemText(i,4)) << " "<<  _wtof(m_CListCtrl.GetItemText(i,5)) << " "<<  _wtof(m_CListCtrl.GetItemText(i,6))<<endl;
		}
		cout<<"Data file was saved." <<endl;	
	}
}




// Joint space control 
void CBrainStimSerialArmDlg::OnBnClickedJointcontrol()
{
	JointControl jDlg;
	jDlg.DoModal();
}
void CBrainStimSerialArmDlg::OnChangeEditTranslationXaxis()
{
	bPredefined = FALSE;	m_SystemMode = "General Mode";
    if(m_hWnd) UpdateData(true);
}
void CBrainStimSerialArmDlg::OnChangeEditTranslationYaxis()
{
	bPredefined = FALSE;	m_SystemMode = "General Mode";
    if(m_hWnd) UpdateData(true);
}
void CBrainStimSerialArmDlg::OnChangeEditTranslationZaxis()
{
	bPredefined = FALSE;	m_SystemMode = "General Mode";
    if(m_hWnd) UpdateData(true);
}
void CBrainStimSerialArmDlg::OnChangeEditRotationXaxis()
{
	bPredefined = FALSE;	m_SystemMode = "General Mode";
    if(m_hWnd) UpdateData(true);
}
void CBrainStimSerialArmDlg::OnChangeEditRotationYaxis()
{
	bPredefined = FALSE;	m_SystemMode = "General Mode";
    if(m_hWnd) UpdateData(true);
}
void CBrainStimSerialArmDlg::OnChangeEditRotationZaxis()
{
	bPredefined = FALSE;	m_SystemMode = "General Mode";
    if(m_hWnd) UpdateData(true);
}
void CBrainStimSerialArmDlg::OnChangeEditAdjRotationZaxis()
{
	bPredefined = FALSE;	m_SystemMode = "General Mode";
    if(m_hWnd) UpdateData(true);
}
void CBrainStimSerialArmDlg::OnChangeEditAdjRotationYaxis()
{
	bPredefined = FALSE;	m_SystemMode = "General Mode";
    if(m_hWnd) UpdateData(true);
}
void CBrainStimSerialArmDlg::OnChangeEditAdjRotationXaxis()
{
	bPredefined = FALSE;	m_SystemMode = "General Mode";
    if(m_hWnd) UpdateData(true);
}
