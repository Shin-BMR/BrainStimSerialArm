#include "stdafx.h"
#include "namedPipe.h"
#include <iostream>

using namespace std;
#define PIPE_TIMEOUT (120*1000)
BOOL bPipe;

//네비게이션 프로그램에서 접속하기를 기다리는 스레드
UINT namedPipe::serverThread(LPVOID p)
{
	HANDLE hPipe;
	DWORD dwRead;
	CString strPipename = TEXT("\\\\.\\pipe\\Pipe_EmpData");
	CString NaviToRobot, strTarget;
	CString strTotalcount;    CString strNumber[4];
	CString strQx[4];    CString strQy[4];    CString strQz[4];    CString strQ0[4];
	CString strTx[4];    CString strTy[4];    CString strTz[4];
	
	CString strTargetQx;    CString strTargetQy;    CString strTargetQz;    CString strTargetQ0;
	CString strTargetTx;    CString strTargetTy;    CString strTargetTz;
	int i,n,Totalcount;
		
	while(1)
	{
		hPipe = CreateNamedPipe(strPipename,PIPE_ACCESS_DUPLEX,PIPE_TYPE_BYTE,255,0,0,PIPE_TIMEOUT,0);
		ConnectNamedPipe(hPipe,NULL);

		
		// ------- NDI 프로그램만(네비게이션없이) 사용할 경우 --------- //
		if(!bPipe)
		{
			cout << "Success Connection with NDI Polaris system." <<endl;
			bPipe = TRUE;
		}
		Marker* marker = new Marker();
		ReadFile( hPipe, marker, sizeof(Marker), &dwRead, NULL);
		for(i=0;i<4;i++){
		tx[i]=marker->px[i];
		ty[i]=marker->py[i];
	    tz[i]=marker->pz[i];
		q0[i]=marker->qw[i];
		qx[i]=marker->qx[i];
		qy[i]=marker->qy[i];
		qz[i]=marker->qz[i];
		lFlag[i]=marker->flag[i];
		}
		//cout << tx[0] << " " << tx[0] <<  endl;
		//cout<<lFlag[0]<<" " <<lFlag[1]<<" " <<lFlag[2]<<" " <<lFlag[3]<<" " <<endl<<endl;
		delete marker;
		//////////////////////////////////////////////////////////////////////////////
		
		/*
		// Navigation( IGSTK ) 프로그램 사용할 경우 //
		if(!bPipe)
		{
			cout << "Success Connection with navigation program. " <<endl;
			bPipe = TRUE;
		}
		NDIData* ndiData = new NDIData();
		ReadFile( hPipe, ndiData, sizeof(NDIData), &dwRead, NULL);
		
		bTargetFlag = ndiData->Targetting;
		strTarget = ndiData->m_strA;
		NaviToRobot = ndiData->m_strData;	
				
		//wcout << (const wchar_t*)NaviToRobot << endl;
		
		n = 0;
		strTotalcount = NaviToRobot.Mid( n,2 );		n = n + 2 ;
		Totalcount = _ttoi(strTotalcount); // 전체 마커 개수에 따라서 반복문 횟수 변경. CString -> int : _ttoi 사용			
		//InitializeCriticalSection( g_critical );
		//g_critical.Lock();
		for (i = 0 ; i < Totalcount ; i++) 
		{
				if(i < Totalcount-1)
				{
					strNumber[i] = NaviToRobot.Mid( n, 2 );		n = n + 2;
					strQ0[i] = NaviToRobot.Mid( n , 6 ); 
					if(strQ0[i] == "MISSIN")
					{
						bFlag[i] = false;
						n = n + 24; // MISSING00000031000001E9 (23개) MISSING 빼고 16개
						q0[i] = 0;
						qx[i] = 0;
						qy[i] = 0;
						qz[i] = 0;
						tx[i] = 0;
						ty[i] = 0;
						tz[i] = 0;
					}
					else
					{
						bFlag[i] = true;
						strQ0[i] = NaviToRobot.Mid( n,6 ); n = n + 6;
						strQx[i] = NaviToRobot.Mid( n,6 ); n = n + 6;
						strQy[i] = NaviToRobot.Mid( n,6 ); n = n + 6;
						strQz[i] = NaviToRobot.Mid( n,6 ); n = n + 6;
						strTx[i] = NaviToRobot.Mid( n,7 ); n = n + 7;
						strTy[i] = NaviToRobot.Mid( n,7 ); n = n + 7;
						strTz[i] = NaviToRobot.Mid( n,7 ); n = n + 7;
						n = n + 23; // CRC 부분
						q0[i] = _wtof(strQ0[i]) * 0.0001;
						qx[i] = _wtof(strQx[i]) * 0.0001;
						qy[i] = _wtof(strQy[i]) * 0.0001;
						qz[i] = _wtof(strQz[i]) * 0.0001;
						tx[i] = _wtof(strTx[i]) * 0.01;
						ty[i] = _wtof(strTy[i]) * 0.01;
						tz[i] = _wtof(strTz[i]) * 0.01;
					}
				}
				else if( i == Totalcount -1 )
				{
					strNumber[i] = NaviToRobot.Mid( n, 2 );		n = n + 2;
					strQ0[i] = NaviToRobot.Mid( n , 6 ); 
					if(strQ0[i] == "MISSIN")
					{
						bFlag[i] = false;
						n = n + 24 + 4; // MISSING00000031000001E9 (23개) MISSING 빼고 16개
						q0[i] = 0;
						qx[i] = 0;
						qy[i] = 0;
						qz[i] = 0;
						tx[i] = 0;
						ty[i] = 0;
						tz[i] = 0;
					}
					else
					{
						bFlag[i] = true;
						strQ0[i] = NaviToRobot.Mid( n,6 ); n = n + 6;
						strQx[i] = NaviToRobot.Mid( n,6 ); n = n + 6;
						strQy[i] = NaviToRobot.Mid( n,6 ); n = n + 6;
						strQz[i] = NaviToRobot.Mid( n,6 ); n = n + 6;
						strTx[i] = NaviToRobot.Mid( n,7 ); n = n + 7;
						strTy[i] = NaviToRobot.Mid( n,7 ); n = n + 7;
						strTz[i] = NaviToRobot.Mid( n,7 ); n = n + 7;
						n = n + 23 + 4; // CRC 부분
						q0[i] = _wtof(strQ0[i]) * 0.0001;
						qx[i] = _wtof(strQx[i]) * 0.0001;
						qy[i] = _wtof(strQy[i]) * 0.0001;
						qz[i] = _wtof(strQz[i]) * 0.0001;
						tx[i] = _wtof(strTx[i]) * 0.01;
						ty[i] = _wtof(strTy[i]) * 0.01;
						tz[i] = _wtof(strTz[i]) * 0.01;
					}
				} // if문 종료, 마지막 마커일 경우 0000이 추가되기 때문에 조금 달라짐.
			} // for문 종료, 마커 정보
		n = 0;
		if(bTargetFlag)
		{
			strTargetQ0 = strTarget.Mid(n,6);	n = n + 6;
			strTargetQx = strTarget.Mid(n,6);	n = n + 6;
			strTargetQy = strTarget.Mid(n,6);	n = n + 6;
			strTargetQz = strTarget.Mid(n,6);	n = n + 6;
			strTargetTx = strTarget.Mid(n,7);	n = n + 7;
			strTargetTy = strTarget.Mid(n,7);	n = n + 7;
			strTargetTz = strTarget.Mid(n,7);	n = n + 7;
			n = n + 6;

			Target_q0 = _wtof(strTargetQ0) * 0.0001;
			Target_qx = _wtof(strTargetQx) * 0.0001;
			Target_qy = _wtof(strTargetQy) * 0.0001;
			Target_qz = _wtof(strTargetQz) * 0.0001;
			Target_tx = _wtof(strTargetTx) * 0.01;
			Target_ty = _wtof(strTargetTy) * 0.01;
			Target_tz = _wtof(strTargetTz) * 0.01;
		} // 타겟 정보
		//g_critical.Unlock();
		//cout << "HeadMarker(x,y,z,rz,ry,rx) :" <<tx[3] << ", "<<ty[3] << ", "<<tz[3]<<endl;  
		//cout << "RBMarker(x,y,z,rz,ry,rx) :" <<tx[2] << ", "<<ty[2] << ", "<<tz[2]<<endl;  
		
		delete ndiData;
		//////////////////////////////////////////////////////////////////////////////
		*/


		CloseHandle(hPipe);
	}//무한루프

	return 0;
}
void namedPipe::Connect()
{
	bPipe = FALSE;
	AfxBeginThread(namedPipe::serverThread,this);
}
