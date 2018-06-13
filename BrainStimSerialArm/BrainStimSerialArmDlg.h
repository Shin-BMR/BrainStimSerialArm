
// BrainStimSerialArmDlg.h : 헤더 파일
//
#include "MaxonLib.h" 
#include "Eigen/dense"
#include "Kinematics.h"
#include "namedPipe.h"
#include "LedButton.h" // led button을 위해서.
#include <MMSystem.h> // multi-media timer를 위해서.

#include "SerialComm\SerialPort.h"

#pragma once

UINT updateThread(LPVOID p); // OpenDevice 버튼 클릭 시 생성
UINT FileSaveThread(LPVOID p); // data 텍스트 파일 저장. 
UINT movingCheck(LPVOID p); // 3번 joint 마지막에 움직이게 하기 위해서.
UINT viveControllerThread(LPVOID p);



// CBrainStimSerialArmDlg 대화 상자
class CBrainStimSerialArmDlg : public CDialogEx
{
// 생성입니다.
public:
	
	CBrainStimSerialArmDlg(CWnd* pParent = NULL);	// 표준 생성자입니다.
	// 대화 상자 데이터입니다.
	enum { IDD = IDD_BRAINSTIMSERIALARM_DIALOG };
	
	CLedButton RB_ledCtrl,H_ledCtrl;
	static void CALLBACK Mtimer(UINT uID, UINT uMsg, DWORD dwUser, DWORD dw1, DWORD dw2) ; // multimedia timer
	LRESULT OnUpdateData(WPARAM wParam, LPARAM lParam);
	
	protected:
	virtual void DoDataExchange(CDataExchange* pDX);	// DDX/DDV 지원입니다.
// 구현입니다.
protected:
	HICON m_hIcon;
//	CLedButton RB_ledCtrl,H_ledCtrl;
	// 생성된 메시지 맵 함수
	virtual BOOL OnInitDialog();
	afx_msg void OnSysCommand(UINT nID, LPARAM lParam);
	afx_msg void OnPaint();
	afx_msg HCURSOR OnQueryDragIcon();
	DECLARE_MESSAGE_MAP()
public:

	double inputRx,inputRy,inputRz,inputX,inputY,inputZ; // 편집창에 사용자가 입력하는 카테시안 컨트롤 목표자세
	double m_FineRotZ,m_FineRotY,m_FineRotX;
	float m_encoder1,m_encoder2,m_encoder3,m_encoder4,m_encoder5,m_encoder6; // 편집창에 출력되는 엔코더 변수
	float m_error,m_Orierror;
	CString m_SystemMode;
	int intgrErr[6];	// torqControl 사용할 때, 멀티미디어 타이머 함수 내에서 사용되던 변수들
	int befq[6];		// P, I, D term.

	double preDefine_x[10],preDefine_y[10],preDefine_z[10],preDefine_rz[10],preDefine_ry[10],preDefine_rx[10];
	
	double bef_Target_tx,bef_Target_ty,bef_Target_tz;
	double diff_target;
	double bef_px,bef_py,bef_pz;
			
	double mx,my,mz; // 말단부 마커부터 초음파 초점까지 이동변환 ( 설계 확인 해보기 )
	double mrx,mry,mrz;  // 말단부 마커부터 초음파 초점까지 회전변환(deg)
	double mTheta,ml,endTofocal;

	Matrix4f T_EEMarkerToD;
	//double preDefine_rx[10],preDefine_rz[10];
	//double preDefine_x[10] = {0,20,-10,30,-20,-50,10,60,-20,0};
	//double preDefine_y[10] = {0,20,-10,30,-20,-50,10,60,-20,0};
	//double preDefine_z[10] = {180,160,180,170,170,150,170,150,170,180};
	//double preDefine_ry[10] = {90,100,80,110,75,80,100,120,80,0};

	Eigen::Matrix4f T_RBtoC, T_CtoInitHead , T_HMarkerToD; // Robot Base Set 버튼 클릭 시 메인 다이얼로그의 멤버 변수로 저장.
											// 전역변수까지 할 필요없지만, Compensation 시에 필요하기 때문에.
	int samplingTime;
	int InitNodeId,FinalNodeId; // 모터의 시작노드와 마지막노드 번호
	BOOL bCompenFlag; // Compensation 버튼 클릭 시 TRUE
	BOOL bTimer; 	
	BOOL bRecordFlag;
	BOOL bPredefined;
	BOOL bRobotBaseSet;
	BOOL bHomePose;
	MMRESULT m_idEvent; //타이머 핸들러 선언
	
	void OnLvnItemChanged(NMHDR *pNMHDR,LRESULT *pResult);
	afx_msg void OnBnClickedMoveendeffector();
	afx_msg void OnBnClickedHalt();
	afx_msg void OnBnClickedOpendevice();
	afx_msg void OnBnClickedEnablemotor();	
	afx_msg void OnChangeEditAdjRotationZaxis();
	afx_msg void OnChangeEditAdjRotationYaxis();
	afx_msg void OnChangeEditAdjRotationXaxis();
	afx_msg void OnChangeEditRotationZaxis();
	afx_msg void OnChangeEditRotationYaxis();
	afx_msg void OnChangeEditRotationXaxis();		
	afx_msg void OnChangeEditTranslationZaxis();
	afx_msg void OnChangeEditTranslationYaxis();
	afx_msg void OnChangeEditTranslationXaxis();
	afx_msg void OnBnClickedDisablemotor();
	afx_msg void OnBnClickedJointcontrol();
	afx_msg void OnBnClickedPipeconnect();
	afx_msg void OnBnClickedrobotbaseset();
	afx_msg void OnBnClickedOk();
	afx_msg void OnBnClickedCompensation();
	afx_msg void OnBnClickedGohome();
	afx_msg void OnBnClickedDatarecorder();
	afx_msg void OnBnClickedDatarecorderstop();
	afx_msg void OnBnClickedUpbutton();
	afx_msg void OnBnClickedLeftbutton();
	afx_msg void OnBnClickedDownbutton();
	afx_msg void OnBnClickedRightbutton();
	afx_msg void OnNMReleasedcaptureDepth(NMHDR *pNMHDR, LRESULT *pResult);
	CSliderCtrl m_Depth;
	double m_DepthValue;
	afx_msg void OnBnClickedVivectrl();
	afx_msg void OnBnClickedSavePose();
	CListCtrl m_CListCtrl;
	afx_msg void OnBnClickedDeletepose();
	afx_msg void OnBnClickedStimulate();
};
