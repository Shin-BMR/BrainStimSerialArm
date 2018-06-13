


//extern NDIData ndidata; 
//extern DataFromNavigation DataN;
//extern CCriticalSection g_critical;
typedef class NAMEDPIPE
{
public:
	
	static UINT serverThread(LPVOID p);
	static UINT rxThread(LPVOID p);
	//NAMEDPIPE();
	//~NAMEDPIPE();
	void Connect();
	//static char *sArr[10];	
}namedPipe;

