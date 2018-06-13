#include "stdafx.h"
#include "Kinematics.h"
#include "Eigen/dense"
#include <math.h> //atan2함수
#include <iostream>
using namespace Eigen;
using namespace std;


void Kinematics::T01(double q1,Matrix4f *m01)
{
	Matrix4f ROT_X_90;
	ROT_X_90 << 1,0,0,0,0,0,-1,0,0,1,0,0,0,0,0,1;
	Matrix4f ROT_Z_th1;
	ROT_Z_th1 << cos(q1),-sin(q1),0,0,sin(q1),cos(q1),0,0,0,0,1,0,0,0,0,1;
	Matrix4f ROT_Z_90;
	ROT_Z_90 << 0,-1,0,0,1,0,0,0,0,0,1,0,0,0,0,1;

	Matrix4f T01;
	T01=ROT_X_90*ROT_Z_th1*ROT_Z_90;
	m01->row(0) = T01.row(0);
	m01->row(1) = T01.row(1);
	m01->row(2) = T01.row(2);
	m01->row(3) = T01.row(3);
}
void Kinematics::T12(double q2,Matrix4f *m12)
{
	Matrix4f ROT_X_alp1;
	ROT_X_alp1 << 1,0,0,0,  0,cos(alp),-sin(alp),0,  0,sin(alp),cos(alp),0,  0,0,0,1;
	Matrix4f ROT_Z_th2;
	ROT_Z_th2 << cos(q2),-sin(q2),0,0,	sin(q2),cos(q2),0,0,	0,0,1,0,	0,0,0,1;
	
	Matrix4f T12;
	T12=ROT_X_alp1*ROT_Z_th2;
	m12->row(0) = T12.row(0);
	m12->row(1) = T12.row(1);
	m12->row(2) = T12.row(2);
	m12->row(3) = T12.row(3);
}
void Kinematics::T23(double q3,Matrix4f *m23)
{	
	Matrix4f ROT_X_alp2;
	ROT_X_alp2 << 1,0,0,0,  0,cos(beta),-sin(beta),0,  0,sin(beta),cos(beta),0,  0,0,0,1;
	Matrix4f TRANS_Z_d3;
	TRANS_Z_d3 << 1,0,0,0,	0,1,0,0,	0,0,1,radius-q3,	0,0,0,1;
	
	Matrix4f T23;
	T23=ROT_X_alp2*TRANS_Z_d3;
	m23->row(0) = T23.row(0);
	m23->row(1) = T23.row(1);
	m23->row(2) = T23.row(2);
	m23->row(3) = T23.row(3);
}
void Kinematics::T34(double q4,Matrix4f *m34)
{
	Matrix4f ROT_Z_th4;
	ROT_Z_th4 << cos(q4),-sin(q4),0,0,sin(q4),cos(q4),0,0,0,0,1,0,0,0,0,1;

	Matrix4f T34;
	T34=ROT_Z_th4;
	m34->row(0) = T34.row(0);
	m34->row(1) = T34.row(1);
	m34->row(2) = T34.row(2);
	m34->row(3) = T34.row(3);
}
void Kinematics::T45(double q5,Matrix4f *m45)
{
	Matrix4f ROT_X_90;
	ROT_X_90 << 1,0,0,0,0,0,-1,0,0,1,0,0,0,0,0,1;
	Matrix4f ROT_Z_th5;
	ROT_Z_th5 << cos(q5),-sin(q5),0,0,sin(q5),cos(q5),0,0,0,0,1,0,0,0,0,1;	
	Matrix4f ROT_Z__90;
	ROT_Z__90 << 0,1,0,0,  -1,0,0,0,  0,0,1,0,  0,0,0,1;
	
	Matrix4f T45;
	T45=ROT_X_90*ROT_Z_th5*ROT_Z__90;
	m45->row(0) = T45.row(0);
	m45->row(1) = T45.row(1);
	m45->row(2) = T45.row(2);
	m45->row(3) = T45.row(3);
}
void Kinematics::T56(double q6,Matrix4f *m56)
{
	Matrix4f ROT_X__90;
	ROT_X__90 << 1,0,0,0,  0,0,1,0,  0,-1,0,0,  0,0,0,1;
	Matrix4f ROT_Z_th6;
	ROT_Z_th6 << cos(q6),-sin(q6),0,0,sin(q6),cos(q6),0,0,0,0,1,0,0,0,0,1;	
	Matrix4f T56;
	T56=ROT_X__90*ROT_Z_th6;
	m56->row(0) = T56.row(0);
	m56->row(1) = T56.row(1);
	m56->row(2) = T56.row(2);
	m56->row(3) = T56.row(3);
}
void Kinematics::T6e(Matrix4f *m6e)
{	
	Matrix4f TRANS_X_l6; 
	TRANS_X_l6 << 1,0,0,l6,  0,1,0,0,  0,0,1,0,  0,0,0,1;
	Matrix4f T6e;
	T6e=TRANS_X_l6;
	m6e->row(0) = T6e.row(0);
	m6e->row(1) = T6e.row(1);
	m6e->row(2) = T6e.row(2);
	m6e->row(3) = T6e.row(3);
}

void Kinematics::T03(double q1,double q2,double q3,Matrix4f *m03)
{
	q1 = q1 * D2R;	q2 = q2 * D2R;	q3 = q3 + l6;
	Matrix4f ROT_X_90;
	ROT_X_90 << 1,0,0,0,0,0,-1,0,0,1,0,0,0,0,0,1;
	Matrix4f ROT_Z_th1;
	ROT_Z_th1 << cos(q1),-sin(q1),0,0,sin(q1),cos(q1),0,0,0,0,1,0,0,0,0,1;
	Matrix4f ROT_Z_90;
	ROT_Z_90 << 0,-1,0,0,1,0,0,0,0,0,1,0,0,0,0,1;
	Matrix4f T01;
	T01=ROT_X_90*ROT_Z_th1*ROT_Z_90;

	Matrix4f ROT_X_alp1;
	ROT_X_alp1 << 1,0,0,0,  0,cos(alp),-sin(alp),0,  0,sin(alp),cos(alp),0,  0,0,0,1;
	Matrix4f ROT_Z_th2;
	ROT_Z_th2 << cos(q2),-sin(q2),0,0,	sin(q2),cos(q2),0,0,	0,0,1,0,	0,0,0,1;
	Matrix4f T12;
	T12=ROT_X_alp1*ROT_Z_th2;

	Matrix4f ROT_X_alp2;
	ROT_X_alp2 << 1,0,0,0,  0,cos(beta),-sin(beta),0,  0,sin(beta),cos(beta),0,  0,0,0,1;
	Matrix4f TRANS_Z_d3;
	TRANS_Z_d3 << 1,0,0,0,	0,1,0,0,	0,0,1,radius-q3,	0,0,0,1;
	Matrix4f T23;
	T23=ROT_X_alp2*TRANS_Z_d3;

	Matrix4f T03;
	T03 = T01*T12*T23;

	m03->row(0) = T03.row(0);
	m03->row(1) = T03.row(1);
	m03->row(2) = T03.row(2);
	m03->row(3) = T03.row(3);
}
void Kinematics::T06(double q1,double q2,double q3,double q4,double q5,double q6,Matrix4f *m06)
{
	q1 = q1 * D2R;	q2 = q2 * D2R;	q4 = q4 * D2R;	q5 = q5 * D2R;	q6 = q6 * D2R;

	Matrix4f ROT_X_90;
	ROT_X_90 << 1,0,0,0,0,0,-1,0,0,1,0,0,0,0,0,1;
	Matrix4f ROT_Z_th1;
	ROT_Z_th1 << cos(q1),-sin(q1),0,0,sin(q1),cos(q1),0,0,0,0,1,0,0,0,0,1;
	Matrix4f ROT_Z_90;
	ROT_Z_90 << 0,-1,0,0,1,0,0,0,0,0,1,0,0,0,0,1;
	Matrix4f T01;
	T01=ROT_X_90*ROT_Z_th1*ROT_Z_90;

	Matrix4f ROT_X_alp1;
	ROT_X_alp1 << 1,0,0,0,  0,cos(alp),-sin(alp),0,  0,sin(alp),cos(alp),0,  0,0,0,1;
	Matrix4f ROT_Z_th2;
	ROT_Z_th2 << cos(q2),-sin(q2),0,0,	sin(q2),cos(q2),0,0,	0,0,1,0,	0,0,0,1;
	Matrix4f T12;
	T12=ROT_X_alp1*ROT_Z_th2;

	Matrix4f ROT_X_alp2;
	ROT_X_alp2 << 1,0,0,0,  0,cos(beta),-sin(beta),0,  0,sin(beta),cos(beta),0,  0,0,0,1;
	Matrix4f TRANS_Z_d3;
	TRANS_Z_d3 << 1,0,0,0,	0,1,0,0,	0,0,1,radius-q3,	0,0,0,1;
	Matrix4f T23;
	T23=ROT_X_alp2*TRANS_Z_d3;

	Matrix4f ROT_Z_th4;
	ROT_Z_th4 << cos(q4),-sin(q4),0,0,sin(q4),cos(q4),0,0,0,0,1,0,0,0,0,1;
	Matrix4f T34;
	T34=ROT_Z_th4;

	Matrix4f ROT_Z_th5;
	ROT_Z_th5 << cos(q5),-sin(q5),0,0,sin(q5),cos(q5),0,0,0,0,1,0,0,0,0,1;	
	Matrix4f ROT_Z__90;
	ROT_Z__90 << 0,1,0,0,  -1,0,0,0,  0,0,1,0,  0,0,0,1;
	Matrix4f T45;
	T45=ROT_X_90*ROT_Z_th5*ROT_Z__90;
	
	Matrix4f ROT_X__90;
	ROT_X__90 << 1,0,0,0,  0,0,1,0,  0,-1,0,0,  0,0,0,1;
	Matrix4f ROT_Z_th6;
	ROT_Z_th6 << cos(q6),-sin(q6),0,0,sin(q6),cos(q6),0,0,0,0,1,0,0,0,0,1;	
	Matrix4f T56;
	T56=ROT_X__90*ROT_Z_th6;

	Matrix4f T06;
	T06 = T01*T12*T23*T34*T45*T56;

	m06->row(0) = T06.row(0);
	m06->row(1) = T06.row(1);
	m06->row(2) = T06.row(2);
	m06->row(3) = T06.row(3);
}

void Kinematics::T0e(double q1,double q2,double q3,double q4,double q5,double q6,Matrix4f *m0e)
{
	q1 = q1 * D2R;	q2 = q2 * D2R;	q4 = q4 * D2R;	q5 = q5 * D2R;	q6 = q6 * D2R;

	Matrix4f ROT_X_90;
	ROT_X_90 << 1,0,0,0,0,0,-1,0,0,1,0,0,0,0,0,1;
	Matrix4f ROT_Z_th1;
	ROT_Z_th1 << cos(q1),-sin(q1),0,0,sin(q1),cos(q1),0,0,0,0,1,0,0,0,0,1;
	Matrix4f ROT_Z_90;
	ROT_Z_90 << 0,-1,0,0,1,0,0,0,0,0,1,0,0,0,0,1;
	Matrix4f T01;
	T01=ROT_X_90*ROT_Z_th1*ROT_Z_90;

	Matrix4f ROT_X_alp1;
	ROT_X_alp1 << 1,0,0,0,  0,cos(alp),-sin(alp),0,  0,sin(alp),cos(alp),0,  0,0,0,1;
	Matrix4f ROT_Z_th2;
	ROT_Z_th2 << cos(q2),-sin(q2),0,0,	sin(q2),cos(q2),0,0,	0,0,1,0,	0,0,0,1;
	Matrix4f T12;
	T12=ROT_X_alp1*ROT_Z_th2;

	Matrix4f ROT_X_alp2;
	ROT_X_alp2 << 1,0,0,0,  0,cos(beta),-sin(beta),0,  0,sin(beta),cos(beta),0,  0,0,0,1;
	Matrix4f TRANS_Z_d3;
	TRANS_Z_d3 << 1,0,0,0,	0,1,0,0,	0,0,1,radius-q3,	0,0,0,1;
	Matrix4f T23;
	T23=ROT_X_alp2*TRANS_Z_d3;

	Matrix4f ROT_Z_th4;
	ROT_Z_th4 << cos(q4),-sin(q4),0,0,sin(q4),cos(q4),0,0,0,0,1,0,0,0,0,1;
	Matrix4f T34;
	T34=ROT_Z_th4;

	Matrix4f ROT_Z_th5;
	ROT_Z_th5 << cos(q5),-sin(q5),0,0,sin(q5),cos(q5),0,0,0,0,1,0,0,0,0,1;	
	Matrix4f ROT_Z__90;
	ROT_Z__90 << 0,1,0,0,  -1,0,0,0,  0,0,1,0,  0,0,0,1;
	Matrix4f T45;
	T45=ROT_X_90*ROT_Z_th5*ROT_Z__90;
	
	Matrix4f ROT_X__90;
	ROT_X__90 << 1,0,0,0,  0,0,1,0,  0,-1,0,0,  0,0,0,1;
	Matrix4f ROT_Z_th6;
	ROT_Z_th6 << cos(q6),-sin(q6),0,0,sin(q6),cos(q6),0,0,0,0,1,0,0,0,0,1;	
	Matrix4f T56;
	T56=ROT_X__90*ROT_Z_th6;

	Matrix4f TRANS_X_l6; 
	TRANS_X_l6 << 1,0,0,l6,  0,1,0,0,  0,0,1,0,  0,0,0,1;
	Matrix4f T6e;
	T6e=TRANS_X_l6;

	Matrix4f T0e;
	T0e = T01*T12*T23*T34*T45*T56*T6e;

	m0e->row(0) = T0e.row(0);
	m0e->row(1) = T0e.row(1);
	m0e->row(2) = T0e.row(2);
	m0e->row(3) = T0e.row(3);
}

void Kinematics::ik(double des_px,double des_py,double des_pz,double des_rz, double des_ry,double des_rx,struct IKJoint* InvK)
{
	//입력변수 순서 : x,y,z [mm] , rz,ry,rx [deg]

	//지역변수 이렇게 많이 사용해도 되는 지 애매함. 스레드에서 ik함수를 여러번 실행할경우 메모리 누수가 생기지 않을까.
	double r11,r12,r13,r21,r22,r23,r31,r32,r33,px,py,pz;
	double costh1,sinth1,costh2,sinth2,sinth4,costh4,sinth5,costh5,sinth6,costh6,th1d,th2d,th4d,th5d,th6d,a1,b1,b2,b3,b4,b5,b6;
	
	Ori1=des_rz;	Ori2=des_ry;	Ori3=des_rx;
	Posi1=des_px;	Posi2=des_py;	Posi3=des_pz;

/*
	Adj_Ori1=m_FineRotZ;
	Adj_Ori2=m_FineRotY;
	Adj_Ori3=m_FineRotX;	

	MatrixXd Adj_ZOri(4,4);
	MatrixXd Adj_YOri(4,4);
	MatrixXd Adj_XOri(4,4);

	Adj_ZOri(0,0)=cos(Adj_Ori1*M_PI/180);	Adj_ZOri(0,1)=-sin(Adj_Ori1*M_PI/180);	Adj_ZOri(0,2)=0;	Adj_ZOri(0,3)=0;
	Adj_ZOri(1,0)=sin(Adj_Ori1*M_PI/180); Adj_ZOri(1,1)=cos(Adj_Ori1*M_PI/180);		Adj_ZOri(1,2)=0;	Adj_ZOri(1,3)=0;
	Adj_ZOri(2,0)=0;				Adj_ZOri(2,1)=0;					Adj_ZOri(2,2)=1;	Adj_ZOri(2,3)=0;
	Adj_ZOri(3,0)=0;				Adj_ZOri(3,1)=0;					Adj_ZOri(3,2)=0;	Adj_ZOri(3,3)=1;

	Adj_YOri(0,0)=cos(Adj_Ori2*M_PI/180);	Adj_YOri(0,1)=0;		Adj_YOri(0,2)=sin(Adj_Ori2*M_PI/180);	Adj_YOri(0,3)=0;
	Adj_YOri(1,0)=0;				Adj_YOri(1,1)=1;		Adj_YOri(1,2)=0;				Adj_YOri(1,3)=0;
	Adj_YOri(2,0)=-sin(Adj_Ori2*M_PI/180);Adj_YOri(2,1)=0;		Adj_YOri(2,2)=cos(Adj_Ori2*M_PI/180);	Adj_YOri(2,3)=0;
	Adj_YOri(3,0)=0;				Adj_YOri(3,1)=0;		Adj_YOri(3,2)=0;				Adj_YOri(3,3)=1;	

	Adj_XOri(0,0)=1;	Adj_XOri(0,1)=0;					Adj_XOri(0,2)=0;					Adj_XOri(0,3)=0;
	Adj_XOri(1,0)=0;	Adj_XOri(1,1)=cos(Adj_Ori3*M_PI/180);		Adj_XOri(1,2)=-sin(Adj_Ori3*M_PI/180);	Adj_XOri(1,3)=0;
	Adj_XOri(2,0)=0;	Adj_XOri(2,1)=sin(Adj_Ori3*M_PI/180);		Adj_XOri(2,2)=cos(Adj_Ori3*M_PI/180);		Adj_XOri(2,3)=0;
	Adj_XOri(3,0)=0;	Adj_XOri(3,1)=0;					Adj_XOri(3,2)=0;					Adj_XOri(3,3)=1;

*/
		Matrix4f ZOri,YOri,XOri,Ori;
	ZOri << cos(Ori1*M_PI/180),-sin(Ori1*M_PI/180),0,0,sin(Ori1*M_PI/180),cos(Ori1*M_PI/180),0,0,0,0,1,0,0,0,0,1;		
	YOri << cos(Ori2*M_PI/180),0,sin(Ori2*M_PI/180),0,		0,1,0,0,		-sin(Ori2*M_PI/180),0,cos(Ori2*M_PI/180),0,		0,0,0,1;
	XOri << 1,0,0,0,	0,cos(Ori3*M_PI/180),-sin(Ori3*M_PI/180),0,		0,sin(Ori3*M_PI/180),cos(Ori3*M_PI/180),0,		0,0,0,1;
	Ori=ZOri*YOri*XOri;

	//MatrixXd Adjustment_Ori(4,4);
	//Adjustment_Ori=Adj_ZOri*Adj_YOri*Adj_XOri;
	//Ori=Ori*Adjustment_Ori;

    r11=Ori(0,0);	r12=Ori(0,1);	r13=Ori(0,2);		px=Posi1;
	r21=Ori(1,0);	r22=Ori(1,1);	r23=Ori(1,2);		py=Posi2;
	r31=Ori(2,0);	r32=Ori(2,1);r33=Ori(2,2);	    pz=Posi3;
	
	 Matrix4f END;
	 END << r11,r12,r13,px,		r21,r22,r23,py,		r31,r32,r33,pz,		0,0,0,1;
	//%***************************************************************
	if( sqrt(px*px+py*py+pz*pz) < radius + l6 )
	{
		a1	=	sqrt((px-l6*r11)*(px-l6*r11)+(py-l6*r21)*(py-l6*r21)+(pz-l6*r31)*(pz-l6*r31));
		d3	=	radius-a1;

		if( d3 < 0 )
		{
			if(timerCount==0)
			{cout <<"Target point is too far away.. " << endl;}
			else if(timerCount%10 == 0)
			{cout <<"Target point is too far away.. " << endl;}
			InvK->bIKFlag = FALSE;
			return;
		}

		else if( d3 > 150 )
		{
			if(timerCount==0)
			{cout <<"Target point is too close .. " << endl;}
			else if(timerCount%10 == 0)
			{cout <<"Target point is too close .. " << endl;}
			InvK->bIKFlag = FALSE;
			return;
		}
	
		else if( py-l6*r21 < -radius*cos((64.43-50)*D2R) )
		{
			InvK->bIKFlag = FALSE;
			return;
		}

		else if( py-l6*r21 > radius*cos((180-64.43-50)*D2R) )
		{
			InvK->bIKFlag = FALSE;
			return;
		}

		else if ((0 < d3) && (d3 < 150))
		{
			costh2=(a1*cos(alp)*cos(beta)-l6*r21+py)/(sin(alp)*sin(beta)*a1);  
			if ( abs(costh2) < 1)
			{InvK->bIKFlag = TRUE;}
			else 
			{
				InvK->bIKFlag = FALSE;
				th1d=60;
				th2d=60;
				d3=75;
				th4d=0;
				th5d=0;
				th6d=0;		
			}
		}
	}

	else
	{
		InvK->bIKFlag = FALSE;
	}

	////////^^^^^^^^^^제한 조건 설정^^^^^^^^^^^^////////////////////////

	if(InvK->bIKFlag)
	{
		sinth2=sqrt(1-costh2*costh2);
		th2=atan2(sinth2,costh2);  th2d=th2*R2D;
		b1=cos(beta)*sin(alp)+sin(beta)*cos(alp)*cos(th2);	b2=sin(beta)*sin(th2);
		b3=-b2; b4=b1;b5=(px-l6*r11)/a1;  b6=(pz-l6*r31)/a1;
		MatrixXd crammer1(2,2);
		crammer1 << b1, b2, b3, b4;
		MatrixXd crammer2(2,2);
		crammer2 << b5, b6, b3, b4;
		MatrixXd crammer3(2,2);
		crammer3 << b1, b2, b5, b6;
		costh1=crammer2.determinant()/crammer1.determinant();  sinth1=crammer3.determinant()/crammer1.determinant();
		th1=atan2(sinth1,costh1);  th1d=th1*R2D;	
		///////----------------------------------------------------------
		Matrix4f ROT_X_90,ROT_Z_th1,ROT_Y_alp1,ROT_Z_th2,ROT_Y_alp2,TRANS_Z_d3;
		ROT_X_90 << 1,0,0,0,	0,0,-1,0,	0,1,0,0,	0,0,0,1;
		ROT_Z_th1 << cos(th1),-sin(th1),0,0,	sin(th1),cos(th1),0,0,	0,0,1,0,	0,0,0,1;
		ROT_Y_alp1 << cos(alp),0,sin(alp),0,	0,1,0,0,	-sin(alp),0,cos(alp),0,	0,0,0,1;
		ROT_Z_th2 << cos(th2),-sin(th2),0,0,	sin(th2),cos(th2),0,0,	0,0,1,0,	0,0,0,1;
		ROT_Y_alp2 << cos(beta),0,sin(beta),0,	0,1,0,0,	-sin(beta),0,cos(beta),0,	0,0,0,1;
		TRANS_Z_d3 << 1,0,0,0,	0,1,0,0,	0,0,1,radius-d3,	0,0,0,1;

		Matrix4f T01,T12,T23,T36,T03,T03_inv,ROT_Z_90;
		ROT_Z_90<<0,-1,0,0,  1,0,0,0,  0,0,1,0,  0,0,0,1;
		T01=ROT_X_90*ROT_Z_th1*ROT_Y_alp1;	T12=ROT_Z_th2*ROT_Y_alp2;	T23=TRANS_Z_d3*ROT_Z_90;
		T03=T01*T12*T23;	T03_inv=T03.inverse();	T36=T03_inv*END;
		///////----------------------------------------------------------
		sinth5=T36(2,2);	costh5=sqrt(1-sinth5*sinth5);
		th5=atan2(sinth5,costh5); 	th5d=th5*R2D;
		sinth6=T36(2,1)/costh5;	costh6=T36(2,0)/-costh5;
		th6=atan2(sinth6,costh6);	th6d=th6*R2D;
		costh4=T36(0,2)/costh5;	sinth4=T36(1,2)/costh5;
		th4=atan2(sinth4,costh4);	th4d=th4*R2D;
	}
	else if(!InvK->bIKFlag)
	{
		th1d=34.2934;
		th2d=66.3287;
		d3=88.44;
		th4d=-44.5549;
		th5d=0;
		th6d=0;
		if(timerCount%100 == 0)
		{cout<<"Can't solve the inverse kinematics. So, Go to home. "<<endl;}
		
		
		// return;
		
	}
	InvK->InvJoint[0]=th1d;
	InvK->InvJoint[1]=th2d;
	InvK->InvJoint[2]=d3;
	InvK->InvJoint[3]=th4d;
	InvK->InvJoint[4]=th5d;
	InvK->InvJoint[5]=th6d;
}

void Kinematics::ik_adj(double des_px,double des_py,double des_pz,double des_rz, double des_ry,double des_rx,double adj_rz,double adj_ry,double adj_rx,struct IKJoint* InvK)
{
	
	//입력변수 순서 : x,y,z [mm] , rz,ry,rx [deg]

	//지역변수 이렇게 많이 사용해도 되는 지 애매함. 스레드에서 ik함수를 여러번 실행할경우 메모리 누수가 생기지 않을까.
	double r11,r12,r13,r21,r22,r23,r31,r32,r33,px,py,pz,Adj_Ori1,Adj_Ori2,Adj_Ori3;
	double costh1,sinth1,costh2,sinth2,sinth4,costh4,sinth5,costh5,sinth6,costh6,th1d,th2d,th4d,th5d,th6d,a1,b1,b2,b3,b4,b5,b6;
	
	Ori1=des_rz;	Ori2=des_ry;	Ori3=des_rx;
	Posi1=des_px;	Posi2=des_py;	Posi3=des_pz;


	Adj_Ori1=adj_rz;
	Adj_Ori2=adj_ry;
	Adj_Ori3=adj_rx;	

	Matrix4f Adj_ZOri(4,4),Adj_YOri(4,4),Adj_XOri(4,4);

	Adj_ZOri(0,0)=cos(Adj_Ori1*M_PI/180);	Adj_ZOri(0,1)=-sin(Adj_Ori1*M_PI/180);	Adj_ZOri(0,2)=0;	Adj_ZOri(0,3)=0;
	Adj_ZOri(1,0)=sin(Adj_Ori1*M_PI/180); Adj_ZOri(1,1)=cos(Adj_Ori1*M_PI/180);		Adj_ZOri(1,2)=0;	Adj_ZOri(1,3)=0;
	Adj_ZOri(2,0)=0;				Adj_ZOri(2,1)=0;					Adj_ZOri(2,2)=1;	Adj_ZOri(2,3)=0;
	Adj_ZOri(3,0)=0;				Adj_ZOri(3,1)=0;					Adj_ZOri(3,2)=0;	Adj_ZOri(3,3)=1;

	Adj_YOri(0,0)=cos(Adj_Ori2*M_PI/180);	Adj_YOri(0,1)=0;		Adj_YOri(0,2)=sin(Adj_Ori2*M_PI/180);	Adj_YOri(0,3)=0;
	Adj_YOri(1,0)=0;				Adj_YOri(1,1)=1;		Adj_YOri(1,2)=0;				Adj_YOri(1,3)=0;
	Adj_YOri(2,0)=-sin(Adj_Ori2*M_PI/180);Adj_YOri(2,1)=0;		Adj_YOri(2,2)=cos(Adj_Ori2*M_PI/180);	Adj_YOri(2,3)=0;
	Adj_YOri(3,0)=0;				Adj_YOri(3,1)=0;		Adj_YOri(3,2)=0;				Adj_YOri(3,3)=1;	

	Adj_XOri(0,0)=1;	Adj_XOri(0,1)=0;					Adj_XOri(0,2)=0;					Adj_XOri(0,3)=0;
	Adj_XOri(1,0)=0;	Adj_XOri(1,1)=cos(Adj_Ori3*M_PI/180);		Adj_XOri(1,2)=-sin(Adj_Ori3*M_PI/180);	Adj_XOri(1,3)=0;
	Adj_XOri(2,0)=0;	Adj_XOri(2,1)=sin(Adj_Ori3*M_PI/180);		Adj_XOri(2,2)=cos(Adj_Ori3*M_PI/180);		Adj_XOri(2,3)=0;
	Adj_XOri(3,0)=0;	Adj_XOri(3,1)=0;					Adj_XOri(3,2)=0;					Adj_XOri(3,3)=1;


		Matrix4f ZOri,YOri,XOri,Ori;
	ZOri << cos(Ori1*M_PI/180),-sin(Ori1*M_PI/180),0,0,sin(Ori1*M_PI/180),cos(Ori1*M_PI/180),0,0,0,0,1,0,0,0,0,1;		
	YOri << cos(Ori2*M_PI/180),0,sin(Ori2*M_PI/180),0,		0,1,0,0,		-sin(Ori2*M_PI/180),0,cos(Ori2*M_PI/180),0,		0,0,0,1;
	XOri << 1,0,0,0,	0,cos(Ori3*M_PI/180),-sin(Ori3*M_PI/180),0,		0,sin(Ori3*M_PI/180),cos(Ori3*M_PI/180),0,		0,0,0,1;
	Ori=ZOri*YOri*XOri;

	Matrix4f Adjustment_Ori(4,4);
	Adjustment_Ori=Adj_ZOri*Adj_YOri*Adj_XOri;
	Ori=Ori*Adjustment_Ori;

    r11=Ori(0,0);	r12=Ori(0,1);	r13=Ori(0,2);		px=Posi1;
	r21=Ori(1,0);	r22=Ori(1,1);	r23=Ori(1,2);		py=Posi2;
	r31=Ori(2,0);	r32=Ori(2,1);r33=Ori(2,2);	    pz=Posi3;
	
	 Matrix4f END;
	 END << r11,r12,r13,px,		r21,r22,r23,py,		r31,r32,r33,pz,		0,0,0,1;
	//%***************************************************************
	if( sqrt(px*px+py*py+pz*pz) < radius + l6 )
	{
		a1	=	sqrt((px-l6*r11)*(px-l6*r11)+(py-l6*r21)*(py-l6*r21)+(pz-l6*r31)*(pz-l6*r31));
		d3	=	radius-a1;

		if( d3 < 0 )
		{
			if(timerCount==0)
			{cout <<"Target point is too far away.. " << endl;}
			else if(timerCount%10 == 0)
			{cout <<"Target point is too far away.. " << endl;}
			InvK->bIKFlag = FALSE;
			return;
		}

		else if( d3 > 150 )
		{
			if(timerCount==0)
			{cout <<"Target point is too close .. " << endl;}
			else if(timerCount%10 == 0)
			{cout <<"Target point is too close .. " << endl;}
			InvK->bIKFlag = FALSE;
			return;
		}
	
		else if( py-l6*r21 < -radius*cos((64.43-50)*D2R) )
		{
			InvK->bIKFlag = FALSE;
			return;
		}

		else if( py-l6*r21 > radius*cos((180-64.43-50)*D2R) )
		{
			InvK->bIKFlag = FALSE;
			return;
		}

		else if ((0 < d3) && (d3 < 150))
		{
			costh2=(a1*cos(alp)*cos(beta)-l6*r21+py)/(sin(alp)*sin(beta)*a1);  
			if ( abs(costh2) < 1)
			{InvK->bIKFlag = TRUE;}
			else 
			{
				InvK->bIKFlag = FALSE;
				th1d=60;
				th2d=60;
				d3=75;
				th4d=0;
				th5d=0;
				th6d=0;		
			}
		}
	}

	else
	{
		InvK->bIKFlag = FALSE;
	}

	////////^^^^^^^^^^제한 조건 설정^^^^^^^^^^^^////////////////////////

	if(InvK->bIKFlag)
	{
		sinth2=sqrt(1-costh2*costh2);
		th2=atan2(sinth2,costh2);  th2d=th2*R2D;
		b1=cos(beta)*sin(alp)+sin(beta)*cos(alp)*cos(th2);	b2=sin(beta)*sin(th2);
		b3=-b2; b4=b1;b5=(px-l6*r11)/a1;  b6=(pz-l6*r31)/a1;
		MatrixXd crammer1(2,2);
		crammer1 << b1, b2, b3, b4;
		MatrixXd crammer2(2,2);
		crammer2 << b5, b6, b3, b4;
		MatrixXd crammer3(2,2);
		crammer3 << b1, b2, b5, b6;
		costh1=crammer2.determinant()/crammer1.determinant();  sinth1=crammer3.determinant()/crammer1.determinant();
		th1=atan2(sinth1,costh1);  th1d=th1*R2D;	
		///////----------------------------------------------------------
		Matrix4f ROT_X_90,ROT_Z_th1,ROT_Y_alp1,ROT_Z_th2,ROT_Y_alp2,TRANS_Z_d3;
		ROT_X_90 << 1,0,0,0,	0,0,-1,0,	0,1,0,0,	0,0,0,1;
		ROT_Z_th1 << cos(th1),-sin(th1),0,0,	sin(th1),cos(th1),0,0,	0,0,1,0,	0,0,0,1;
		ROT_Y_alp1 << cos(alp),0,sin(alp),0,	0,1,0,0,	-sin(alp),0,cos(alp),0,	0,0,0,1;
		ROT_Z_th2 << cos(th2),-sin(th2),0,0,	sin(th2),cos(th2),0,0,	0,0,1,0,	0,0,0,1;
		ROT_Y_alp2 << cos(beta),0,sin(beta),0,	0,1,0,0,	-sin(beta),0,cos(beta),0,	0,0,0,1;
		TRANS_Z_d3 << 1,0,0,0,	0,1,0,0,	0,0,1,radius-d3,	0,0,0,1;

		Matrix4f T01,T12,T23,T36,T03,T03_inv,ROT_Z_90;
		ROT_Z_90<<0,-1,0,0,  1,0,0,0,  0,0,1,0,  0,0,0,1;
		T01=ROT_X_90*ROT_Z_th1*ROT_Y_alp1;	T12=ROT_Z_th2*ROT_Y_alp2;	T23=TRANS_Z_d3*ROT_Z_90;
		T03=T01*T12*T23;	T03_inv=T03.inverse();	T36=T03_inv*END;
		///////----------------------------------------------------------
		sinth5=T36(2,2);	costh5=sqrt(1-sinth5*sinth5);
		th5=atan2(sinth5,costh5); 	th5d=th5*R2D;
		sinth6=T36(2,1)/costh5;	costh6=T36(2,0)/-costh5;
		th6=atan2(sinth6,costh6);	th6d=th6*R2D;
		costh4=T36(0,2)/costh5;	sinth4=T36(1,2)/costh5;
		th4=atan2(sinth4,costh4);	th4d=th4*R2D;
	}
	else if(!InvK->bIKFlag)
	{
		th1d=34.2934;
		th2d=66.3287;
		d3=88.44;
		th4d=-44.5549;
		th5d=0;
		th6d=0;
		if(timerCount%100 == 0)
		{cout<<"Can't solve the inverse kinematics. So, Go to home. "<<endl;}
		
		
		// return;
		
	}
	InvK->InvJoint[0]=th1d;
	InvK->InvJoint[1]=th2d;
	InvK->InvJoint[2]=d3;
	InvK->InvJoint[3]=th4d;
	InvK->InvJoint[4]=th5d;
	InvK->InvJoint[5]=th6d;
}

void Kinematics::ik(struct PoseOfBody* Pose, struct IKJoint* InvK)
{
	double r11,r12,r13,r21,r22,r23,r31,r32,r33,px,py,pz;
	double costh1,sinth1,costh2,sinth2,sinth4,costh4,sinth5,costh5,sinth6,costh6,th1d,th2d,th4d,th5d,th6d,a1,b1,b2,b3,b4,b5,b6;
	
	Ori1=Pose->rz;	Ori2=Pose->ry;	Ori3=Pose->rx;
	Posi1=Pose->x;	Posi2=Pose->y;	Posi3=Pose->z;

	Matrix4f ZOri,YOri,XOri,Ori;
	ZOri << cos(Ori1*M_PI/180),-sin(Ori1*M_PI/180),0,0,		sin(Ori1*M_PI/180),cos(Ori1*M_PI/180),0,0,		0,0,1,0,		0,0,0,1;		
	YOri << cos(Ori2*M_PI/180),0,sin(Ori2*M_PI/180),0,		0,1,0,0,		-sin(Ori2*M_PI/180),0,cos(Ori2*M_PI/180),0,		0,0,0,1;
	XOri << 1,0,0,0,	0,cos(Ori3*M_PI/180),-sin(Ori3*M_PI/180),0,		0,sin(Ori3*M_PI/180),cos(Ori3*M_PI/180),0,		0,0,0,1;
	Ori	=	ZOri*YOri*XOri;

    r11=Ori(0,0);	r12=Ori(0,1);	r13=Ori(0,2);		px=Posi1;
	r21=Ori(1,0);	r22=Ori(1,1);	r23=Ori(1,2);		py=Posi2;
	r31=Ori(2,0);	r32=Ori(2,1);	r33=Ori(2,2);	    pz=Posi3;

	 Matrix4f END;
	 END << r11,r12,r13,px,		r21,r22,r23,py,		r31,r32,r33,pz,		0,0,0,1;

	//%***************************************************************
	if( sqrt(px*px+py*py+pz*pz) < radius + l6 )
	{
		a1	=	sqrt((px-l6*r11)*(px-l6*r11)+(py-l6*r21)*(py-l6*r21)+(pz-l6*r31)*(pz-l6*r31));
		d3	=	radius-a1;

		if( d3 < 0 )
		{
			if(timerCount==0)
			{cout <<"Target point is too far away.. " << endl;}
			else if(timerCount%10 == 0)
			{cout <<"Target point is too far away.. " << endl;}
			InvK->bIKFlag = FALSE;
			return;
		}

		else if( d3 > 150 )
		{
			if(timerCount==0)
			{cout <<"Target point is too close .. " << endl;}
			else if(timerCount%10 == 0)
			{cout <<"Target point is too close .. " << endl;}
			InvK->bIKFlag = FALSE;
			return;
		}
	
		else if( py-l6*r21 < -radius*cos((64.43-50)*D2R) )
		{
			InvK->bIKFlag = FALSE;
			return;
		}

		else if( py-l6*r21 > radius*cos((180-64.43-50)*D2R) )
		{
			InvK->bIKFlag = FALSE;
			return;
		}

		else if ((0 < d3) && (d3 < 150))
		{
			costh2=(a1*cos(alp)*cos(beta)-l6*r21+py)/(sin(alp)*sin(beta)*a1);  
			if ( abs(costh2) < 1)
			{InvK->bIKFlag = TRUE;}
			else 
			{
				InvK->bIKFlag = FALSE;
				th1d=60;
				th2d=60;
				d3=75;
				th4d=0;
				th5d=0;
				th6d=0;		
			}
		}
	}

	else
	{
		InvK->bIKFlag = FALSE;
	}

	////////^^^^^^^^^^제한 조건 설정^^^^^^^^^^^^////////////////////////

	if(InvK->bIKFlag)
	{
		sinth2=sqrt(1-costh2*costh2);
		th2=atan2(sinth2,costh2);  th2d=th2*R2D;
		b1=cos(beta)*sin(alp)+sin(beta)*cos(alp)*cos(th2);	b2=sin(beta)*sin(th2);
		b3=-b2; b4=b1;b5=(px-l6*r11)/a1;  b6=(pz-l6*r31)/a1;
		MatrixXd crammer1(2,2);
		crammer1 << b1, b2, b3, b4;
		MatrixXd crammer2(2,2);
		crammer2 << b5, b6, b3, b4;
		MatrixXd crammer3(2,2);
		crammer3 << b1, b2, b5, b6;
		costh1=crammer2.determinant()/crammer1.determinant();  sinth1=crammer3.determinant()/crammer1.determinant();
		th1=atan2(sinth1,costh1);  th1d=th1*R2D;	
		///////----------------------------------------------------------
		Matrix4f ROT_X_90,ROT_Z_th1,ROT_Y_alp1,ROT_Z_th2,ROT_Y_alp2,TRANS_Z_d3;
		ROT_X_90 << 1,0,0,0,	0,0,-1,0,	0,1,0,0,	0,0,0,1;
		ROT_Z_th1 << cos(th1),-sin(th1),0,0,	sin(th1),cos(th1),0,0,	0,0,1,0,	0,0,0,1;
		ROT_Y_alp1 << cos(alp),0,sin(alp),0,	0,1,0,0,	-sin(alp),0,cos(alp),0,	0,0,0,1;
		ROT_Z_th2 << cos(th2),-sin(th2),0,0,	sin(th2),cos(th2),0,0,	0,0,1,0,	0,0,0,1;
		ROT_Y_alp2 << cos(beta),0,sin(beta),0,	0,1,0,0,	-sin(beta),0,cos(beta),0,	0,0,0,1;
		TRANS_Z_d3 << 1,0,0,0,	0,1,0,0,	0,0,1,radius-d3,	0,0,0,1;

		Matrix4f T01,T12,T23,T36,T03,T03_inv,ROT_Z_90;
		ROT_Z_90<<0,-1,0,0,  1,0,0,0,  0,0,1,0,  0,0,0,1;
		T01=ROT_X_90*ROT_Z_th1*ROT_Y_alp1;	T12=ROT_Z_th2*ROT_Y_alp2;	T23=TRANS_Z_d3*ROT_Z_90;
		T03=T01*T12*T23;	T03_inv=T03.inverse();	T36=T03_inv*END;
		///////----------------------------------------------------------
		sinth5=T36(2,2);	costh5=sqrt(1-sinth5*sinth5);
		th5=atan2(sinth5,costh5); 	th5d=th5*R2D;
		sinth6=T36(2,1)/costh5;	costh6=T36(2,0)/-costh5;
		th6=atan2(sinth6,costh6);	th6d=th6*R2D;
		costh4=T36(0,2)/costh5;	sinth4=T36(1,2)/costh5;
		th4=atan2(sinth4,costh4);	th4d=th4*R2D;
	}
	else if(!InvK->bIKFlag)
	{
		th1d=34.2934;
		th2d=66.3287;
		d3=88.44;
		th4d=-44.5549;
		th5d=0;
		th6d=0;
		if(timerCount%100 == 0)
		{cout<<"Can't solve the inverse kinematics. So, Go to home. "<<endl;}
	}
	InvK->InvJoint[0]=th1d;
	InvK->InvJoint[1]=th2d;
	InvK->InvJoint[2]=d3;
	InvK->InvJoint[3]=th4d;
	InvK->InvJoint[4]=th5d;
	InvK->InvJoint[5]=th6d;
}


void Kinematics::ik(double px,double py,double pz,struct IKJoint* InvK)
{
	double costh1,sinth1,costh2,sinth2,th1d,th2d,th4d,th5d,th6d,a1,b1,b2,b3,b4,b5,b6,b7,b8;
	
	double radius2;

	//%***************************************************************
	a1	=	sqrt((px*px) + (py*py) + (pz*pz));
	if ( radius - a1 > 0 )
	{	
		radius2 = radius -l6;
		d3	=	radius2 - a1 ;
		costh2 = ( cos(alp)*cos(beta) * (d3-radius2) - py ) / (sin(alp)*sin(beta)*(d3-radius2)); 
		
		if ( abs(costh2) < 1)
		{InvK->bIKFlag = TRUE;}
		else 
		{	InvK->bIKFlag = FALSE;
			cout << " A amplitude of cos(th2) is over 1. " << endl; 		}
	}
	else 
			InvK->bIKFlag = FALSE;
	////////^^^^^^^^^^제한 조건 설정^^^^^^^^^^^^////////////////////////
	if(InvK->bIKFlag)
	{
		sinth2=sqrt(1-costh2*costh2);
		th2=atan2(sinth2,costh2);  th2d=th2*R2D;

		b1=cos(beta)*sin(alp) + sin(beta)*cos(alp)*cos(th2);	b2=-sin(beta)*sin(th2);
		b3=-b2;													b4=b1;

		b5=px/(radius2-d3);	b6=b2;
		b7=pz/(radius2-d3);	b8=b4;
		
		MatrixXd crammer1(2,2);
		crammer1 << b1, b2, b3, b4;
		MatrixXd crammer2(2,2);
		crammer2 << b5, b6, b7, b8;
		MatrixXd crammer3(2,2);
		crammer3 << b1, b5, b3, b7;

		costh1=crammer2.determinant()/crammer1.determinant();  sinth1=crammer3.determinant()/crammer1.determinant();
		th1=atan2(sinth1,costh1);  th1d=th1*R2D;
		th4d = 0;		th5d = 0;		th6d = 0;
	}
	else if(!InvK->bIKFlag)
	{
		th1d=34.2934;
		th2d=66.3287;
		d3=88.44;
		th4d=-44.5549;
		th5d=0;
		th6d=0;
		if(timerCount%100 == 0)
		{cout<<"Can't solve the inverse kinematics. So, Go to home. "<<endl;}
	}
	InvK->InvJoint[0]=th1d;
	InvK->InvJoint[1]=th2d;
	InvK->InvJoint[2]=d3;
	InvK->InvJoint[3]=th4d;
	InvK->InvJoint[4]=th5d;
	InvK->InvJoint[5]=th6d;
}

void Kinematics::Translation(double x,double y,double z,Matrix4f *Trans)
{	
	Matrix4f TRANS_X; 
	TRANS_X << 1,0,0,x,  0,1,0,0,  0,0,1,0,  0,0,0,1;
	Matrix4f TRANS_Y;
	TRANS_Y << 1,0,0,0,  0,1,0,y,  0,0,1,0,  0,0,0,1;
	Matrix4f TRANS_Z;
	TRANS_Z << 1,0,0,0,  0,1,0,0,  0,0,1,z,  0,0,0,1;
	
	Matrix4f T;
	T=TRANS_X*TRANS_Y*TRANS_Z;
	Trans->row(0) = T.row(0);
	Trans->row(1) = T.row(1);
	Trans->row(2) = T.row(2);
	Trans->row(3) = T.row(3);
}
void Kinematics::Translation(double x,double y,double z,Vector3f *Trans)
{	
	Vector3f P;
	P << x,y,z;
	Trans->coeffRef(0) = P.coeff(0);
	Trans->coeffRef(1) = P.coeff(1);
	Trans->coeffRef(2) = P.coeff(2);	
}
void Kinematics::Rotation(double rz,double ry,double rx,Matrix4f *Rot)
{	
	rz = rz*M_PI/180;
	ry = ry*M_PI/180;
	rx = rx*M_PI/180;

	Matrix4f ROT_X; 
	ROT_X << 1,0,0,0,  0,cos(rx),-sin(rx),0,  0,sin(rx),cos(rx),0,  0,0,0,1;
	Matrix4f ROT_Y;
	ROT_Y << cos(ry),0,sin(ry),0,  0,1,0,0,  -sin(ry),0,cos(ry),0,  0,0,0,1;
	Matrix4f ROT_Z;
	ROT_Z << cos(rz),-sin(rz),0,0,  sin(rz),cos(rz),0,0,  0,0,1,0,  0,0,0,1;
	
	Matrix4f R;
	R=ROT_Z*ROT_Y*ROT_X;
	Rot->row(0) = R.row(0);
	Rot->row(1) = R.row(1);
	Rot->row(2) = R.row(2);
	Rot->row(3) = R.row(3);

}
void Kinematics::Rotation(double rz,double ry,double rx,Matrix3f *Rot)
{	
	rz = rz*M_PI/180;
	ry = ry*M_PI/180;
	rx = rx*M_PI/180;

	Matrix3f ROT_X; 
	ROT_X << 1,0,0,  0,cos(rx),-sin(rx),  0,sin(rx),cos(rx);
	Matrix3f ROT_Y;
	ROT_Y << cos(ry),0,sin(ry),  0,1,0,  -sin(ry),0,cos(ry);
	Matrix3f ROT_Z;
	ROT_Z << cos(rz),-sin(rz),0,  sin(rz),cos(rz),0,  0,0,1;
	
	Matrix3f R;
	R=ROT_Z*ROT_Y*ROT_X;
	Rot->row(0) = R.row(0);
	Rot->row(1) = R.row(1);
	Rot->row(2) = R.row(2);
}
// Reverse(&Transform) 동차변환의 역을 리턴함
void Kinematics::Reverse(Matrix4f *T,Matrix4f *revT)
{
	Matrix3f R12,R21;
	R12 = T->block(0,0,3,3);
	R21 = R12.transpose();

	Vector3f p12;
	p12 = T->block(0,3,3,1);

	Matrix4f tmpT;
	tmpT << R21 , -R21*p12 , 0, 0, 0, 1 ;

	revT->row(0) = tmpT.row(0);
	revT->row(1) = tmpT.row(1);
	revT->row(2) = tmpT.row(2);
	revT->row(3) = tmpT.row(3);
}
void Kinematics::GetTransform(double q0,double qx,double qy,double qz,double tx,double ty,double tz,Matrix4f *T)
{
	Matrix4f TransformationMatrix; 
	TransformationMatrix << 1 - 2*(qy*qy+qz*qz) , 2*(qx*qy - q0*qz), 2*(q0*qy + qx*qz), tx ,    		2*(q0*qz + qx*qy), 1 - 2*(qx*qx+qz*qz) , 2*(qy*qz - q0*qx), ty,    		2*(qx*qz - q0*qy), 2*(q0*qx + qy*qz), 1 - 2*(qx*qx+qy*qy), tz,		0,0,0,1;
		
	T->row(0) = TransformationMatrix.row(0);
	T->row(1) = TransformationMatrix.row(1);
	T->row(2) = TransformationMatrix.row(2);
	T->row(3) = TransformationMatrix.row(3);
}
void Kinematics::GetTransform(struct PoseOfBody* Pose,Matrix4f *T)
{
	Matrix4f m1;
	Kinematics *k = new Kinematics;
	k->Rotation(Pose->rz,Pose->ry,Pose->rx,&m1);
	m1.coeffRef(0,3) = Pose->x;	m1.coeffRef(1,3) = Pose->y;	m1.coeffRef(2,3) = Pose->z;
	//m1.coeffRef(0,3) = Pose->x; 삭제해야함
	T->row(0) = m1.row(0);
	T->row(1) = m1.row(1);
	T->row(2) = m1.row(2);
	T->row(3) = m1.row(3);
}
void Kinematics::GetEuler(Matrix4f *T,struct PoseOfBody* pose)
{	
	double r11,r12,r13,r21,r22,r23,r31,r32,r33,phi,theta,psi;

	r11=T->coeff(0,0);r12=T->coeff(0,1);r13=T->coeff(0,2);	pose->x=T->coeff(0,3);
	r21=T->coeff(1,0);r22=T->coeff(1,1);r23=T->coeff(1,2);	pose->y=T->coeff(1,3);
	r31=T->coeff(2,0);r32=T->coeff(2,1);r33=T->coeff(2,2);	pose->z=T->coeff(2,3);

	theta = atan2(-r31 , sqrt(r32*r32+r33*r33)); // y-axis

	// -90deg < theta(y-axis) < 90deg 
	if((-(M_PI/2)<theta) && (theta<M_PI/2))
	{
		phi = atan2(r21,r11); // z-axis
		psi = atan2(r32,r33); // x-axis
	}

	else
	{
		if(theta > 0)
		{
			theta = M_PI/2;			// y-axis
			phi = 0;				// z-axis
			psi = atan2(r12,r22);	// x-axis
		}
		else
		{
			theta = -M_PI/2;		// y-axis
			phi = 0;				// z-axis
			psi = -atan2(r12,r22);	// x-axis
		}
	}

	pose->rz = phi*180/M_PI;
	pose->ry = theta*180/M_PI;
	pose->rx = psi*180/M_PI;
}

void Kinematics::GetEuler(Matrix3f *T,struct OrienOfBody* pose)
{	
	double r11,r12,r13,r21,r22,r23,r31,r32,r33,phi,theta,psi;

	r11=T->coeff(0,0);r12=T->coeff(0,1);r13=T->coeff(0,2);	
	r21=T->coeff(1,0);r22=T->coeff(1,1);r23=T->coeff(1,2);	
	r31=T->coeff(2,0);r32=T->coeff(2,1);r33=T->coeff(2,2);	

	theta = atan2(-r31,sqrt(r32*r32+r33*r33));
	// -90deg < theta(y-axis) < 90deg 
	if((-(M_PI/2)<theta)&&(theta<M_PI/2))
	{
	phi = atan2(r21,r11);
	psi = atan2(r32,r33);
	}
	else
	{
		if(theta > 0)
		{
			theta = M_PI/2;
			phi = 0;
			psi = atan2(r12,r22);
		}
		else
		{
			theta = -M_PI/2;
			phi = 0;
			psi = -atan2(r12,r22);
		}
	}
	pose->rz = phi*180/M_PI;
	pose->ry = theta*180/M_PI;
	pose->rx = psi*180/M_PI;	
}

void Kinematics::CarteToSphe(double x,double y,double z,struct SphericalCoordinate* Sphe)
{
	Sphe->R=sqrt(x*x+y*y+z*z);
	// theta : latitude
	// phi : longitude
	double theta,phi,sinphi,cosphi;
	theta=atan((sqrt(x*x+y*y))/z);

	Sphe->thetad=theta*R2D;

	sinphi=y/sqrt(x*x+y*y);
	cosphi=x/sqrt(x*x+y*y);
	phi=atan2(sinphi,cosphi);
	if(phi<0)
		phi = 2*M_PI + phi;

	Sphe->phid=phi*R2D;

}

void Kinematics::SpheToCarte(double r,double phid,double thetad,struct PositionOfBody* pos)
{
	pos->x = r*sin(thetad*D2R)*cos(phid*D2R);
	pos->y = r*sin(thetad*D2R)*sin(phid*D2R);
	pos->z = r*cos(thetad*D2R);
}

void Kinematics::SpheToCarte(double r,double phid,double thetad,double rx,double ry,double rz,struct PositionOfBody* pos)
{
	double x,y,z;

	x = r*sin(thetad*D2R)*cos(phid*D2R);
	y = r*sin(thetad*D2R)*sin(phid*D2R);
	z = r*cos(thetad*D2R);
	
	pos->x = x + rx * l6;
	pos->y = y + ry * l6;
	pos->z = z + rz * l6;
}