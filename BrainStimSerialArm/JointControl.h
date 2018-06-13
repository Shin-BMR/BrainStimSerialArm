#pragma once

#include "MaxonLib.h"
// JointControl 대화 상자입니다.

class JointControl : public CDialogEx
{
	DECLARE_DYNAMIC(JointControl)

public:
	JointControl(CWnd* pParent = NULL);   // 표준 생성자입니다.
	virtual ~JointControl();

// 대화 상자 데이터입니다.
	enum { IDD = IDD_JointControl };

protected:
	virtual void DoDataExchange(CDataExchange* pDX);    // DDX/DDV 지원입니다.
	
	int jointNodeId;
	long jointTargetPosition;
	long jointMaxVelocity;
    CString m_JointStatus1,m_JointStatus2,m_JointStatus3,m_JointStatus4,m_JointStatus5,m_JointStatus6;
	
	CListCtrl m_PositionGain;


	DECLARE_MESSAGE_MAP()
public:
	//virtual BOOL OnInitDialog();
	afx_msg void OnBnClickedjointmove();
	afx_msg void OnChangeEditJointNodeId();
	afx_msg void OnChangeEditJointTargetPosition();
	afx_msg void OnChangeEditJointMaxVelocity();	
	afx_msg void OnBnClickedjointhalt();
	afx_msg void OnBnClickedCheckenable();
	afx_msg void OnBnClickedFindhome();
	afx_msg void OnBnClickedOk();
	afx_msg void OnBnClickedConstraint();
	afx_msg void OnBnClickedGetpositiongain();
};
