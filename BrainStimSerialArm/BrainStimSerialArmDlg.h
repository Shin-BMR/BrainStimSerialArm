
// BrainStimSerialArmDlg.h : ��� ����
//
#include "MaxonLib.h" 
#include "Eigen/dense"
#include "Kinematics.h"
#include "namedPipe.h"
#include "LedButton.h" // led button�� ���ؼ�.
#include <MMSystem.h> // multi-media timer�� ���ؼ�.

#include "SerialComm\SerialPort.h"

#pragma once

UINT updateThread(LPVOID p); // OpenDevice ��ư Ŭ�� �� ����
UINT FileSaveThread(LPVOID p); // data �ؽ�Ʈ ���� ����. 
UINT movingCheck(LPVOID p); // 3�� joint �������� �����̰� �ϱ� ���ؼ�.
UINT viveControllerThread(LPVOID p);



// CBrainStimSerialArmDlg ��ȭ ����
class CBrainStimSerialArmDlg : public CDialogEx
{
// �����Դϴ�.
public:
	
	CBrainStimSerialArmDlg(CWnd* pParent = NULL);	// ǥ�� �������Դϴ�.
	// ��ȭ ���� �������Դϴ�.
	enum { IDD = IDD_BRAINSTIMSERIALARM_DIALOG };
	
	CLedButton RB_ledCtrl,H_ledCtrl;
	static void CALLBACK Mtimer(UINT uID, UINT uMsg, DWORD dwUser, DWORD dw1, DWORD dw2) ; // multimedia timer
	LRESULT OnUpdateData(WPARAM wParam, LPARAM lParam);
	
	protected:
	virtual void DoDataExchange(CDataExchange* pDX);	// DDX/DDV �����Դϴ�.
// �����Դϴ�.
protected:
	HICON m_hIcon;
//	CLedButton RB_ledCtrl,H_ledCtrl;
	// ������ �޽��� �� �Լ�
	virtual BOOL OnInitDialog();
	afx_msg void OnSysCommand(UINT nID, LPARAM lParam);
	afx_msg void OnPaint();
	afx_msg HCURSOR OnQueryDragIcon();
	DECLARE_MESSAGE_MAP()
public:

	double inputRx,inputRy,inputRz,inputX,inputY,inputZ; // ����â�� ����ڰ� �Է��ϴ� ī�׽þ� ��Ʈ�� ��ǥ�ڼ�
	double m_FineRotZ,m_FineRotY,m_FineRotX;
	float m_encoder1,m_encoder2,m_encoder3,m_encoder4,m_encoder5,m_encoder6; // ����â�� ��µǴ� ���ڴ� ����
	float m_error,m_Orierror;
	CString m_SystemMode;
	int intgrErr[6];	// torqControl ����� ��, ��Ƽ�̵�� Ÿ�̸� �Լ� ������ ���Ǵ� ������
	int befq[6];		// P, I, D term.

	double preDefine_x[10],preDefine_y[10],preDefine_z[10],preDefine_rz[10],preDefine_ry[10],preDefine_rx[10];
	
	double bef_Target_tx,bef_Target_ty,bef_Target_tz;
	double diff_target;
	double bef_px,bef_py,bef_pz;
			
	double mx,my,mz; // ���ܺ� ��Ŀ���� ������ �������� �̵���ȯ ( ���� Ȯ�� �غ��� )
	double mrx,mry,mrz;  // ���ܺ� ��Ŀ���� ������ �������� ȸ����ȯ(deg)
	double mTheta,ml,endTofocal;

	Matrix4f T_EEMarkerToD;
	//double preDefine_rx[10],preDefine_rz[10];
	//double preDefine_x[10] = {0,20,-10,30,-20,-50,10,60,-20,0};
	//double preDefine_y[10] = {0,20,-10,30,-20,-50,10,60,-20,0};
	//double preDefine_z[10] = {180,160,180,170,170,150,170,150,170,180};
	//double preDefine_ry[10] = {90,100,80,110,75,80,100,120,80,0};

	Eigen::Matrix4f T_RBtoC, T_CtoInitHead , T_HMarkerToD; // Robot Base Set ��ư Ŭ�� �� ���� ���̾�α��� ��� ������ ����.
											// ������������ �� �ʿ������, Compensation �ÿ� �ʿ��ϱ� ������.
	int samplingTime;
	int InitNodeId,FinalNodeId; // ������ ���۳��� ��������� ��ȣ
	BOOL bCompenFlag; // Compensation ��ư Ŭ�� �� TRUE
	BOOL bTimer; 	
	BOOL bRecordFlag;
	BOOL bPredefined;
	BOOL bRobotBaseSet;
	BOOL bHomePose;
	MMRESULT m_idEvent; //Ÿ�̸� �ڵ鷯 ����
	
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
