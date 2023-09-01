#pragma once
#include <algorithm>
#include <tuple>
#include <functional>
#include <map>
#include <vector>
#include "EvtPrinter\LogServer.h"
#include "TestExec\EvtATELib\EvtATE.h"
#include "EvtImageLib\CISImage\CISImageInterface.h"
#include "LedCal.h"
#include "LedCtr.h"
#include <numeric>
#include <math.h>


#include <mutex>
#include <atomic>
#include <set>
//#include"Algorithm.h"
#define MAX_SITES         16

#define PD_RS_Voltage         1.8
#define IIC_SLAVE_ID          0x9C

const static std::string g_fixedLetter = "0123456789ABCDEFGHIJKLMNOPQRSTUVWXYZ";




struct IICRegisterStru
{
	unsigned char slaveId;
	unsigned short address;
	unsigned short data;
};

struct IICRegisterBlockStru
{
	EVEREST::EvtUnionData* pRegisterDataArr;
	unsigned int registerNum;
};

struct IDDOffsetEachSite
{
	double AVDD;
	double IOVDD;
	double DVDD;
};

class HSC02A :
	public EvtTestProgramInterface
{

	enum MipiSpeed
	{
		MIPI_1126Mbps = 0,

	};

public:
	HSC02A();
	virtual ~HSC02A();
	virtual int PreInstall() override;
	virtual int PostInstall() override;
	virtual int PreInit() override;
	virtual int PostInit() override;
	virtual int StartOfSite(const unsigned int siteNum) override;
	virtual int EndOfSite(const unsigned int siteNum, bool goodExit) override;

	virtual int PrePatternBurstLoad(const std::string &burstName) override;
	virtual int PostPatternBurstLoad(const std::string &burstName) override;
	virtual int PostPatternBurstExec(const std::string &burstName) override;
	virtual int StartOfTest() override;
	virtual int EndOfTest() override;
	virtual int StartOfLot() override;
	virtual int EndOfLot() override;
	virtual int UserLoad() override;
	virtual int UserUnload() override;
	virtual void FinalTestResults(std::vector<DutFinalTestResult>* finalTestResults) override;

public:
	EVTEXEC_API void PowerShort(const std::string &testName, Test_Result &result);
	EVTEXEC_API void OS_VSS(const std::string &testName, Test_Result &result);
	EVTEXEC_API void OS_VDD(const std::string &testName, Test_Result &result);

 
	EVTEXEC_API void IIH(const std::string &testName, Test_Result &result);   
	EVTEXEC_API void IIL(const std::string &testName, Test_Result &result); 
	EVTEXEC_API void IDD_Active(const std::string &testName, Test_Result &result);
	EVTEXEC_API void IDD_StandBy(const std::string &testName, Test_Result &result);
	EVTEXEC_API void IDD_ShutDown(const std::string &testName, Test_Result &result);   
	EVTEXEC_API void IIC(const std::string &testName, Test_Result &result);  
	EVTEXEC_API void GetTestResult(const std::string &testName, Test_Result &result);
	EVTEXEC_API void Light(const std::string &testName, Test_Result &result);
	EVTEXEC_API void Dark_DPCOn(const std::string &testName, Test_Result &result);
	EVTEXEC_API void Dark_DPCOff(const std::string &testName, Test_Result &result);
	EVTEXEC_API void test_demo(const std::string &testName, Test_Result &result);
	EVTEXEC_API void Temp(const std::string &testName, Test_Result &result);
	EVTEXEC_API void FWC(const std::string &testName, Test_Result &result);
	EVTEXEC_API void DarkFPN(const std::string &testName, Test_Result &result);
	EVTEXEC_API void PRNU(const std::string &testName, Test_Result &result);
	EVTEXEC_API void CG(const std::string& testName, Test_Result& result);
	EVTEXEC_API void TempNoise(const std::string& testName, Test_Result& result);
	EVTEXEC_API void DarkCurrent(const std::string &testName, Test_Result &result);
public:
	//void WriteRegisterAndReadBack(const std::string &testItem, Test_Result &result);
	bool WriteRegister(std::string testItem, bool bEnableReadBack = false);
	bool WriteRegisterBlock(std::string testItem, bool bEnableReadBack = false);
	bool ReadRegister(std::string testItem);
	static bool ReadRegisterFromfile();
	bool SwitchMIPI2PMU(unsigned short SwitchData = 0x1F1F, bool bPMU = true);

public:
	static std::map<std::string, std::vector<IICRegisterStru>> m_RegisterMap;

public:
	//void SetDRP();
	void PowerOn();
	void PowerUpSequence();
	void PowerUpSequenceIDD();
	void PowerUpSequenceLight();
	void PowerOff();
	void PowerDownSequence();
	void PowerDownSequenceIDD();
	void PowerDownSequenceLight();
	void SetCondition(const std::string& testName);
	void DisConnectAllPins();
	void SetCLK();

private:
	void GetTestExecFilePath(std::string & filepath); 
	void ReadImageConfig();
	void ReadIDDOffset();
	void ReadLedTest();
    void NotifyFunc(const std::string& testName, unsigned int site);
    void NotifyFunc(const std::string& testName, const std::vector<unsigned int>& sites);


private:
	static std::map<std::string, IICRegisterBlockStru> m_RegisterBlockMap;
	std::string m_CurrentItemName;
	std::map<std::string, std::map<unsigned int, unsigned short>> m_LuxMap;
	//ledTest
	std::map<std::string, std::map<unsigned int, unsigned short>> m_LedTestLuxMap;
	//add in 2023/06/30
	std::map<std::string, std::map<unsigned int, unsigned short>> m_Diff_LuxMap;
	std::array<std::string, MAX_SITES> m_oldBinKey;
	std::array<char*, MAX_SITES> m_imageTestResults;


	array<vector<unsigned int>, MAX_SITES> mAll_Normal_BD;
	array<vector<unsigned int>, MAX_SITES> mAll_Normal_WD;
	array<vector<unsigned int>, MAX_SITES> mAll_Dark_WS;
private:
	LedCtr m_ledCtr;
	LedCal m_ledCal;

	double Light_Value = 520;
	double PRNU40_Value = 250;
	double PRNU80_Value = 520;

	BOOLEAN IicSpeed = 0;
	unsigned char m_LedCalFlag = 0;
	unsigned char m_FWC_flag = 0;
	unsigned char m_Gray = 0;

	unsigned char Led_test = 0;
	unsigned int led_min = 0;
	unsigned int led_max = 8000;
	unsigned int led_step = 100;
    unsigned int led_current_lux;

	unsigned int IicFreq = 400;
	unsigned int IicAddrWidthBytes = 2;
	unsigned int IicDataWidthBytes = 1;
	unsigned int PCLK_Reverse = 0;
	unsigned int SettleCount = 0;
	unsigned int ClkMode = 0;
	unsigned int LBVersion = 2;

	std::string m_strLotID{ "TestLot" };
	std::string m_strLotStartTime{ "TestStartTime" };
	std::string m_strLotPhase{ "FT1" };
	/* CISTool configuration */
	bool m_cistoolStartup{ false };
	bool m_cistoolShowCapImage{ false };
	bool m_cistoolAutoSave{ false };
	bool m_bSaveImage{ false };
	std::string m_strSavePath;

	std::array<IDDOffsetEachSite, 16> m_IDDOffsets;

	HANDLE m_hSync[EVT_MAX_SYSTEM_SITE];

	unsigned int DarkDPCOnTestNumber;
	unsigned int DarkDPCOffTestNumber;
	unsigned int LightTestNumber;
	unsigned int DarkFPNTestNumber;
	unsigned int PRNUTestNumber;
	unsigned int CGTestNumber;
	unsigned int FWCTestNumber;
	unsigned int TempNoiseTestNumber;
	unsigned int DarkCurrentTestNumber;
};

extern HSC02A g_TestProgram;