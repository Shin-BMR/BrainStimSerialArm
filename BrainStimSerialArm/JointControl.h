#pragma once

#include "MaxonLib.h"
// JointControl ��ȭ �����Դϴ�.

class JointControl : public CDialogEx
{
	DECLARE_DYNAMIC(JointControl)

public:
	JointControl(CWnd* pParent = NULL);   // ǥ�� �������Դϴ�.
	virtual ~JointControl();

// ��ȭ ���� �������Դϴ�.
	enum { IDD = IDD_JointControl };

protected:
	virtual void DoDataExchange(CDataExchange* pDX);    // DDX/DDV �����Դϴ�.
	
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
