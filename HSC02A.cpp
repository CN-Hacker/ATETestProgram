#include "stdafx.h"
#include "HSC02A.h"
#include "LightMap.h"
#include "ImageResultDefine.h"
#include <iostream>
#include <string>
#include <fstream>
#include <boost\lexical_cast.hpp>
#include <boost\property_tree\xml_parser.hpp>
#include <boost\property_tree\ini_parser.hpp>
#include <boost\property_tree\json_parser.hpp>   //add 20210425
#include <boost\filesystem.hpp>
#include <boost/algorithm/string.hpp>
#include <boost/dll.hpp>
#include <algorithm>
#include <tuple>
#include <functional>
#include <thread>
#include <chrono>
#include <EvtUtility/LEDController/EvtBasePort.h>
#include <boost\timer/timer.hpp>

using namespace std;
using namespace EVEREST;
using namespace boost::property_tree;

HSC02A g_TestProgram;


#define MODULE_IIC_DUT            2
#define PCA9535_CMD_PORT_WRITE    0x02
#define PCA9535_CMD_CFG_WRITE     0x06
#define PCA9535_SLAVE_ADDR        0x48
#define INPUT_CLK_REQ			 24*1000
time_t tt;
tm t;

#define IIC_BLOCK_WRITE_MAX 1000

#define FPGA_BASE_ADDR_DVP(siteIndex) ((siteIndex==0) ? 0x160000 : 0xA0000 )

unsigned int g_siteIndex[MAX_SITES];
std::map<std::string, std::vector<IICRegisterStru>> HSC02A::m_RegisterMap;
std::map<std::string, IICRegisterBlockStru> HSC02A::m_RegisterBlockMap;
std::array<int, 16> imageCount;
std::array<int, 16> loopingCounter{};

bool bImagePassFlag[16];
bool bAllPssFlag[16];

//std::map<int, int> LedSiteMap
//{
//	{2,0},
//	{3,1},
//	{6,2},
//	{7,3},
//	{8,4},
//	{9,5},
//	{12,6},
//	{13,7},
//};

/******************debug 新版光源控制板********************************/
//class SerialLight : private SERIAL::EvtBasePort
//{
//public:
//	SerialLight(const std::string& com)
//		: SERIAL::EvtBasePort(com)
//	{
//		SetTimeout(serial::Timeout::simpleTimeout(1000));
//	}
//	~SerialLight()
//	{
//		Close();
//	}
//
//public:
//	size_t send(const std::vector<uint8_t>& sendCmd)
//	{
//		try
//		{
//			return m_pSerialPort->write(sendCmd);
//		}
//		catch (const std::exception& e)
//		{
//			std::cout << e.what() << std::endl;
//			return 0;
//		}
//
//	}
//
//	size_t set_brightness(unsigned int siteID, size_t brightness)
//	{
//#define LB(_val) (uint8_t)((_val) & 0xff)
//#define HB(_val) (uint8_t)(((_val) >> 8u) & 0xff)
//
//		std::vector<uint8_t> sendCmd{ 0xC0, 0xA5, 0x62, 0x00, 0x00, 0x00, 0x0A, 0xA0 };
//		sendCmd[3] = LB(siteID);
//		sendCmd[4] = HB(brightness);
//		sendCmd[5] = LB(brightness);
//
//		return send(sendCmd);
//	}
//
//	size_t read(std::vector<uint8_t>& sendCmd)
//	{
//		sendCmd.clear();
//		sendCmd.resize(16);
//		try
//		{
//			return m_pSerialPort->read(sendCmd);
//		}
//		catch (const std::exception& e)
//		{
//			std::cout << e.what() << std::endl;
//			return 0;
//		}
//	}
//};

namespace fs = boost::filesystem;

std::string GetCurDirPath()
{
	char szPath[MAX_PATH];
	GetModuleFileNameA(NULL, szPath, MAX_PATH);
	char drive[4];
	char subdir[MAX_PATH];
	char fn[MAX_PATH];
	char exten[MAX_PATH];
	_splitpath_s(szPath, drive, subdir, fn, exten);
	std::string strFilePathName = drive;
	strFilePathName += subdir;
	return strFilePathName;
}

std::string GetCurTime()
{
	char buf[256] = {};
	std::time_t time = std::time(nullptr);
	tm local;
	localtime_s(&local, &time);
	strftime(buf, 256, "%Y-%m-%d-%H-%M-%S", &local);
	return buf;
}

HSC02A::HSC02A() :m_ledCal(m_ledCtr)
{
	for (int i = 0; i < EVT_MAX_SYSTEM_SITE; i++)
		m_hSync[i] = CreateEventA(NULL, TRUE, TRUE, "");
}

HSC02A::~HSC02A()
{

}

static void LoadBoardPowerOn(unsigned int siteNum)
{
	ATE()->HWInterface()->WriteRegister(siteNum, 0x070000 + 0x04, 0x0000000F, 0ul);
	ATE()->HWInterface()->WriteRegister(siteNum, 0x070000, 0x0000000F, 0ul);
}

/**********************************V2 Loadboard 切换说明*************************************************************
V2 loadboard切换IIC选用小端是因为硬件原理图9535 GPIO0组8个控制PMU12-21，GPIO1组8个控制PMU22-31
写入data共2个byte，bit0控制PMU20/21,bit1控制PMU18/19，bit2控制PMU16/17,bit3控制PMU14/15，bit4控制PMU12/13，bit5-7空闲
bit8控制PMU30/31，bit9控制PMU28/29，bit10控制PMU26/27，bit11控制PMU24/25，bit12控制PMU22/23，bit13-15空闲
data对应bit值为1-PMU，0-MIPI，全切PMU时写值0x1F1F，9734MIPI为1 lane， 切到MIPI时data应写0x0707
**********************************V2 Loadboard 切换说明End**********************************************************/
bool HSC02A::SwitchMIPI2PMU(unsigned short SwitchData, bool bPMU)
{
	bool bRet = true;
	unsigned short data = 0;
	// MODULE 2
	EvtIICInterface* LB_IIC = ATE()->Bus()->IIC(MODULE_IIC_DUT);
	LB_IIC->Config(PCA9535_SLAVE_ADDR, false, 1, 1, 2, false);        //需确定大端小端
	if (bPMU)
	{
		//data = ((0x001F) | (0x1F00));
		data = SwitchData;
	}
	else
	{
		data = 0;
	}
	bRet &= LB_IIC->Write(PCA9535_CMD_PORT_WRITE, data);
	bRet &= LB_IIC->Write(PCA9535_CMD_CFG_WRITE, 0);

	return bRet;
}

/*add 20210513*/
void HSC02A::GetTestExecFilePath(std::string & filepath)
{
	filepath.clear();
	HANDLE hProcess = OpenProcess(PROCESS_QUERY_INFORMATION | PROCESS_VM_READ, FALSE, GetCurrentProcessId());
	if (hProcess)
	{
		static DWORD(WINAPI *func)(HANDLE, HMODULE, LPTSTR, DWORD) = nullptr;

		HMODULE psapi = nullptr;
		if (func == nullptr)
			if (psapi = LoadLibraryA("psapi.dll"))
				*(FARPROC*)&func = GetProcAddress(psapi, "GetModuleFileNameExA");

		if (func != nullptr)
		{
			char cName[280];
			if (func(hProcess, nullptr, (LPTSTR)cName, sizeof(cName)))
			{
				filepath = cName;
				auto last_sep = filepath.find_last_of('\\');
				filepath = filepath.substr(0, last_sep + 1);

				if (psapi)
					FreeLibrary(psapi);
			}
		}
	}

}

void HSC02A::ReadImageConfig()
try
{
	boost::property_tree::ptree pt;
	boost::property_tree::ini_parser::read_ini("Configure\\ImageConfig.ini", pt);

	auto optional_value = pt.get_optional<double>("AllSite.Light_Value");
	if (optional_value.has_value())
	{
		Light_Value = optional_value.get();
	}

	optional_value = pt.get_optional<double>("AllSite.PRNU40_Value");
	if (optional_value.has_value())
	{
		PRNU40_Value = optional_value.get();
	}

	optional_value = pt.get_optional<double>("AllSite.PRNU80_Value");
	if (optional_value.has_value())
	{
		PRNU80_Value = optional_value.get();
	}

	IicSpeed = pt.get<BOOLEAN>("AllSite.IicSpeed");
	m_LedCalFlag = pt.get<unsigned char>("AllSite.LedCalFlag");
	m_FWC_flag = pt.get<unsigned char>("AllSite.FWC_Flag");
	m_Gray = pt.get<unsigned char>("AllSite.Gray");
	IicFreq = pt.get<unsigned int>("AllSite.IicFreq");
	IicAddrWidthBytes = pt.get<unsigned int>("AllSite.IicAddrWidthBytes");
	IicDataWidthBytes = pt.get<unsigned int>("AllSite.IicDataWidthBytes");
	LBVersion = pt.get<unsigned int>("AllSite.LBVersion");
	if (LBVersion == 1 || LBVersion == 2)
	{
		EVTSYS(LEVEL_DEBUG, "Loadboard Version is V%d.\r\n", LBVersion);
	}
	else
	{
		EVTSYS(LEVEL_ERROR, "The LBVersion flag in ImageConfig.ini is illegal.Switch LBVersion to default value.\r\n");
		LBVersion = 2;
	}

	PCLK_Reverse = pt.get<unsigned int>("AllSite.PCLK_Reverse");
	PCLK_Reverse = PCLK_Reverse << 3;

	/* CISTool configuration */
	m_cistoolStartup = (bool)pt.get<unsigned int>("AllSite.CISToolStartup");
	m_cistoolShowCapImage = (bool)pt.get<unsigned int>("AllSite.CISToolShowCapturedImage");
	m_cistoolAutoSave = (bool)pt.get<unsigned int>("AllSite.CISToolAutoSave");
}
catch (std::exception &e)
{
	EVTSYS(LEVEL_ERROR, e.what());
}

void HSC02A::ReadLedTest()
try
{
    if (Led_test == 0)
    {
        boost::property_tree::ptree pt;
        boost::property_tree::ini_parser::read_ini("Configure\\LedTest.ini", pt);
        Led_test = pt.get<unsigned char>("Led_test");
        if (Led_test == 1)
        {
            led_min = pt.get<unsigned int>("led_min");
            led_max = pt.get<unsigned int>("led_max");
            led_step = pt.get<unsigned int>("led_step");
            led_current_lux = led_min;
            pt.put<unsigned char>("Led_test", 0);
            boost::property_tree::ini_parser::write_ini("Configure\\LedTest.ini", pt);
            EVTSYS(LEVEL_DEBUG, "Led Test Begin\r\n");
        }
    }

}
catch (std::exception &e)
{
	EVTSYS(LEVEL_ERROR, e.what());
}

void HSC02A::ReadIDDOffset()
try
{
	boost::property_tree::ptree pt;
	boost::property_tree::ini_parser::read_ini("Configure\\IDDOffset.ini", pt);

	for (int i = 0; i < 16; i++)
	{
		m_IDDOffsets[i].AVDD = pt.get<double>("Site" + std::to_string(i) + ".AVDD");
		m_IDDOffsets[i].DVDD = pt.get<double>("Site" + std::to_string(i) + ".DVDD");
		m_IDDOffsets[i].IOVDD = pt.get<double>("Site" + std::to_string(i) + ".IOVDD");
		
	}
}
catch (std::exception &e)
{
	EVTSYS(LEVEL_ERROR, e.what());
}

void HSC02A::SetCondition(const std::string& testName)
{
	std::string levelName, timingName;
	ATE()->Test(testName)->GetTestCondition(levelName, timingName);
	if (!timingName.empty())
		ATE()->Timing(timingName)->Execute();
	if (!levelName.empty())
		ATE()->DCLevels()->Block(levelName)->Execute();

	//IIC Pull Up  1-On  0-Off
	ATE()->DIB()->Signal("I2C_EN")->CBitsOn();   //IIC上拉   //mask 20210425
}

void HSC02A::SetCLK()
{
	
	ATE()->DCLevels()->Signal("EXCLK")->SetClkFreq(INPUT_CLK_REQ * 16);
	ATE()->DCLevels()->Signal("EXCLK")->SetClkDividend(16);
	ATE()->DCLevels()->Signal("EXCLK")->Assign(DIGITAL_CLK);
	

}

int HSC02A::PreInstall()
{
	return 0;
}

int HSC02A::PostInstall()
{
	/*if (!m_pLedController.ReadLedConfig(".\\Configure\\LedConfig.ini"))
	{
		EVTSYS(LEVEL_ERROR, "Read led com port failed from Configure\\LedConfig.ini file.\r\n");
	}*/
	return 0;
}

int HSC02A::PreInit()
{
	return 0;
}

int HSC02A::PostInit()
{
	//for (const auto & col : addcol)
	//{
	//	ATE()->Datalog()->SetDefaultDatalogCustomContent(CSV_Body::Headers, col.second, col.first);
	//}

	return 0;
}

int HSC02A::PrePatternBurstLoad(const std::string &burstName)
{
	return 0;
}

int HSC02A::PostPatternBurstLoad(const std::string &burstName)
{
	return 0;
}

int HSC02A::PostPatternBurstExec(const std::string &burstName)
{
	return 0;
}

int HSC02A::StartOfTest()
{
	std::vector<unsigned int> vRsrcID;
	std::vector<unsigned int> vSites;
	ATE()->Site()->GetActive(vSites);
	ReadLedTest();
	ReadImageConfig();
	ReadIDDOffset();
	//CISImage()->ReadDRPConfig(3);   //Read DRP0.ini,DRP1.ini,DRP2.ini 
	if (m_LedCalFlag)
	{
		EVTSYS(LEVEL_INFO, "LedCalFlag is true.\n");
		m_ledCal.SetCalEnable(true);
		if (!m_ledCal.ReadLedCalFile())
		{
			EVTSYS(LEVEL_ERROR, "Read led cal file fail.\n");
		}
	}
	else
	{
		m_ledCal.SetCalEnable(false);
		m_LuxMap["Light"] = m_ledCal.GetValue("Light");
		m_LuxMap["FWC"] = m_ledCal.GetValue("FWC");
		m_LuxMap["PRNU40"] = m_ledCal.GetValue("PRNU40");
		m_LuxMap["PRNU80"] = m_ledCal.GetValue("PRNU80");
		m_LuxMap["CG_PRNU40"] = m_ledCal.GetValue("CG_PRNU40");
		m_LuxMap["CG_PRNU80"] = m_ledCal.GetValue("CG_PRNU80");
	}
			//add in 2023/06/30
	for (auto siteNum : vSites)
	{
			m_Diff_LuxMap["PRNU"][siteNum] = m_LuxMap["PRNU80"][siteNum] - m_LuxMap["PRNU40"][siteNum];
	}
	if (LBVersion == 1)
	{
		//Switch PMU OR MIPI  0-MIPI  1-PMU  
		//V1 LoadBoard Version
		ATE()->DIB()->Signal("MIPISTPMU")->CBitsOn();
	}
	else
	{
		
		SwitchMIPI2PMU();//V2
	}
	//for (auto & i : vSites)
	//{
	//	ATE()->Site()->SetEOSEnable(i, false);   //mask 20210420
	//	CISImage()->DVPSetTime(i, 1000);	//使能DVP delay参数调整功能
	//}
	EVTSUM("Start of Test is called.\r\n");
	SetCLK();
	/*namespace fs = boost::filesystem;
	auto pathTestExec = fs::path(GetCurDirPath());
	auto pathSaveConfig = pathTestExec.append("preset").append("ImageSaveCfg.json");
	if (fs::exists(pathSaveConfig) && fs::is_regular_file(pathSaveConfig) && !fs::is_directory(pathSaveConfig))
	{
		try
		{
			using namespace boost::property_tree;
			boost::property_tree::ptree pt;
			boost::property_tree::read_json(pathSaveConfig.string(), pt);

			auto ptAll = pt.get_child("All");
			for (auto ptConfig : ptAll)
			{
				if (ptConfig.first == "SavePath")
				{
					m_strSavePath = ptConfig.second.get_value<std::string>();
				}

				if (ptConfig.first == "SaveRaw")
				{
					m_bSaveImage = ptConfig.second.get_value<bool>();
				}
			}
		}
		catch (std::exception &e)
		{
			EVTDATA("%s", e.what());
			return false;
		}
	}*/
	return 0;
}

int HSC02A::EndOfTest()
{
	
	ATE()->DCLevels()->Block("ShutDownVterm")->Execute();
	PowerDownSequence();
	
	ATE()->PinPMU()->FV("AllPMU",0V,50mA,true,-50mA,50mA);
	ATE()->PinPMU()->FV("AllUTP",0V,20mA,true,-20mA,20mA);
	
	DisConnectAllPins();

    if (Led_test == 1)
    {
        led_current_lux += led_step;
        if (led_current_lux > led_max)
        {
            Led_test = 0;
            EVTSYS(LEVEL_DEBUG, "Led Test End\r\n");
        }
    }
	/*add 20210425*/
	//std::vector<unsigned int> vSites;
	//ATE()->Site()->GetActive(vSites);

	//for (auto site : vSites)
	//{
	//	ATE()->HWInterface()->WriteRegister(site, 0x890000, 0, 0xBFC1CA00);   //RC不补偿-default 20210416
	//}
	/*add 20210425*/

	return 0;
}

int HSC02A::StartOfLot()
{

	const string productName = ATE()->Datalog()->GetProductID();   
	EVTSYS(LEVEL_INFO, "DeviceName is %s\r\n", productName.data());
	if (productName != "HSC02A")
	{
		EVTSYS(LEVEL_ERROR, "DeviceName is not HSC02A\r\n");
		return 1;
	}

	const string lotId = ATE()->Datalog()->GetLotID();
	EVTSYS(LEVEL_INFO, "LotID is %s\r\n", lotId.data());
	if (lotId.size() < 7)
	{
		EVTSYS(LEVEL_ERROR, "LotID error\r\n");
		return 1;
	}

	const string handlerID = ATE()->Datalog()->GetHandlerID();
	EVTSYS(LEVEL_INFO, "handlerID is %s\r\n", handlerID.data());
	if (handlerID.size() != 7)
	{
		EVTSYS(LEVEL_ERROR, "HandlerID is not FT-20XX\r\n");
		return 1;
	}

	const string TestPhase = ATE()->Datalog()->GetTestPhase();
	EVTSYS(LEVEL_INFO, "TestPhase is %s\r\n", TestPhase.data());




	m_strLotID = lotId;
	m_strLotStartTime = GetCurTime();
	m_strLotPhase = TestPhase;
	return 0;
}

int HSC02A::EndOfLot()
{
	return 0;
}

int HSC02A::StartOfSite(const unsigned int siteNum)
{
	//std::string ChipNo;
	//ChipNo = ATE()->Site()->GetDutID(siteNum);
	//ATE()->Site()->SetDutID(siteNum, ChipNo);
	bImagePassFlag[siteNum]=true;
	bAllPssFlag[siteNum] = false;

	ATE()->DCLevels()->Block("General")->Execute();
	//IIC Pull Up  1-On  0-Off
	ATE()->DIB()->Signal("I2C_EN")->CBitsOff();

	ATE()->Bin()->Set(65535, siteNum);
	m_oldBinKey[siteNum] = "65535";

	g_siteIndex[siteNum] = ATE()->SignalMap()->Signal("EXCLK")->GetSiteIndex(siteNum);

	CISImage()->SetSiteIndex(siteNum, g_siteIndex[siteNum]);
	//Add by GX,DVP PCLK Reverse
	unsigned long long offset = FPGA_BASE_ADDR_DVP(g_siteIndex[siteNum]);
	unsigned long mask = 0x00000008;
	ATE()->HWInterface()->WriteRegister(siteNum, offset, mask, PCLK_Reverse);  //value should be unsigned long
	m_ledCtr.SetCctLux(siteNum, 0);
	//modify in20211229
	loopingCounter[siteNum]++;
	if (m_imageTestResults[siteNum])
	{
		delete[] m_imageTestResults[siteNum];
		m_imageTestResults[siteNum] = nullptr;
	}
	//modify in20211229
	//imageCount.fill(0);

	return 0;
}

int HSC02A::EndOfSite(const unsigned int siteNum, bool goodExit)
{
	bool bRet = true;
	std::vector<unsigned int> vSites;
	ATE()->Site()->GetActive(vSites);

	unsigned int currentBinNum = ATE()->Bin()->GetCurrentSoftBinNum(siteNum);
	if (currentBinNum == 65535 && bAllPssFlag[siteNum] == true)
	{

		ATE()->Bin()->Set(1, siteNum);
		ATE()->Bin()->Increment(1, siteNum);
		
	}
	//m_ledCtr.SetCctLux(siteNum, 0);
	for (auto &site : vSites)
	{
		//light.set_brightness(site, m_LuxMap[testName][site]);
		if (LedSiteMap.count(site) == 0) continue;
		m_ledCtr.SetCctLux(LedSiteMap[site], 0);
	}

	return 0;
}

int HSC02A::UserLoad()
{
	EVTDATA("UserLoad is called\r\n");

	if (!HSC02A::ReadRegisterFromfile())
		return -1;

	vector<unsigned int> vQualifiedSites;
	ATE()->Site()->GetQualified(vQualifiedSites);
	for (auto & i : vQualifiedSites)
	{
		LoadBoardPowerOn(i);    //mask for debug

		//g_siteIndex[i] = ATE()->SignalMap()->Signal("EXCLK")->GetSiteIndex(i);

		//CISImage()->SetSiteIndex(i, g_siteIndex[i]);
	}

	return 0;
}

int HSC02A::UserUnload()
{
	return 0;
}

void HSC02A::FinalTestResults(std::vector<DutFinalTestResult>* finalTestResults)
{
	return;
}

void HSC02A::PowerOn()
{
	ATE()->DCLevels()->Block("PowerOn")->Execute();
}

void HSC02A::PowerUpSequence()
{
	std::vector<unsigned int> vSites;
	ATE()->Site()->GetActive(vSites);

	PowerOff();
	//ATE()->PinPMU()->Signal("AllPMU")->Disconnect();
	//for (auto site : vSites)
	//{
	//	ATE()->HWInterface()->WriteRegister(site, 0x890000, 0, 0xBFC1CA00);   //RC不补偿-default 20210416
	//}
	ATE()->DCLevels()->Block("General")->Execute();
	//IIC Pin Pullup  0-disconnect   1-connect
	ATE()->DIB()->Signal("I2C_EN")->CBitsOn();
	//ATE()->PinPMU()->FV("I2CID", PD_RS_Voltage, 200uA, true, -200uA, 200uA);
	ATE()->PinPMU()->FV("I2CID", 0V, 20mA, true, -20mA, 20mA);
	
	ATE()->DCLevels()->Signal("EXCLK")->SignalEnable(DSM_CLOCK, false);
	ATE()->PinPMU()->FV("RSTB", 0V, 20mA, true, -20mA, 20mA);
	ATE()->SleepMs(5);
	PowerOn();
	ATE()->DCLevels()->Signal("EXCLK")->SignalEnable(DSM_CLOCK, true);
	ATE()->SleepMs(10);
	ATE()->PinPMU()->FV("RSTB", PD_RS_Voltage, 20mA, true, -20mA, 20mA);
	ATE()->SleepMs(100);


	
}

void HSC02A::PowerUpSequenceIDD()
{
	std::vector<unsigned int> vSites;
	ATE()->Site()->GetActive(vSites);

	ATE()->DPS()->FV("AVDD", 0V, 25.6mA, true, -25.6mA, 25.6mA);
	ATE()->SleepUs(100);
	ATE()->DPS()->FV("DVDD", 0V, 25.6mA, true, -25.6mA, 25.6mA);
	ATE()->SleepUs(100);
	ATE()->DPS()->FV("IOVDD", 0V, 25.6mA, true, -25.6mA, 25.6mA);
	ATE()->SleepUs(100);
	//for (auto site : vSites)
	//{
	//	ATE()->HWInterface()->WriteRegister(site, 0x890000, 0, 0xBFC1CA00);   //RC不补偿-default 20210416
	//}
	ATE()->DCLevels()->Block("General")->Execute();
	//IIC Pin Pullup  0-disconnect   1-connect
	ATE()->DIB()->Signal("I2C_EN")->CBitsOn();
	//ATE()->PinPMU()->FV("I2CID", PD_RS_Voltage, 200uA, true, -200uA, 200uA);
	ATE()->PinPMU()->FV("I2CID", 0V, 20mA, true, -20mA, 20mA);
	ATE()->DCLevels()->Signal("EXCLK")->SignalEnable(DSM_CLOCK, false);
	ATE()->PinPMU()->FV("RSTB", 0V, 20mA, true, -20mA, 20mA);

	ATE()->DPS()->FV("AVDD", 2.8V, 25.6mA, true, -25.6mA, 25.6mA);
	ATE()->SleepUs(100);
	ATE()->DPS()->FV("DVDD", 1.8V, 25.6mA, true, -25.6mA, 25.6mA);
	ATE()->SleepUs(100);
	ATE()->DPS()->FV("IOVDD", 1.8V, 25.6mA, true, -25.6mA, 25.6mA);
	ATE()->SleepUs(100);

	ATE()->DCLevels()->Signal("EXCLK")->SignalEnable(DSM_CLOCK, true);
	ATE()->SleepMs(15);
	ATE()->PinPMU()->FV("RSTB", PD_RS_Voltage, 20mA, true, -20mA, 20mA);
	ATE()->SleepMs(100);


}

void HSC02A::PowerUpSequenceLight()
{
	std::vector<unsigned int> vSites;
	ATE()->Site()->GetActive(vSites);

	ATE()->DPS()->FV("AVDD", 0V, 25.6mA, true, -38.4mA, 38.4mA);
	ATE()->SleepUs(100);
	ATE()->DPS()->FV("DVDD", 0V, 25.6mA, true, -38.4mA, 38.4mA);
	ATE()->SleepUs(100);
	ATE()->DPS()->FV("IOVDD", 0V, 25.6mA, true, -38.4mA, 38.4mA);
	ATE()->SleepUs(100);
	//for (auto site : vSites)
	//{
	//	ATE()->HWInterface()->WriteRegister(site, 0x890000, 0, 0xBFC1CA00);   //RC不补偿-default 20210416
	//}
	ATE()->DCLevels()->Block("General")->Execute();
	//IIC Pin Pullup  0-disconnect   1-connect
	ATE()->DIB()->Signal("I2C_EN")->CBitsOn();
	//ATE()->PinPMU()->FV("I2CID", PD_RS_Voltage, 200uA, true, -200uA, 200uA);
	ATE()->PinPMU()->FV("I2CID", 0V, 20mA, true, -20mA, 20mA);
	ATE()->DCLevels()->Signal("EXCLK")->SignalEnable(DSM_CLOCK, false);
	ATE()->PinPMU()->FV("RSTB", 0V, 20mA, true, -20mA, 20mA);

	ATE()->DPS()->FV("AVDD", 2.8V, 25.6mA, true, -38.4mA, 38.4mA);
	ATE()->SleepUs(100);
	ATE()->DPS()->FV("DVDD", 1.8V, 25.6mA, true, -38.4mA, 38.4mA);
	ATE()->SleepUs(100);
	ATE()->DPS()->FV("IOVDD", 1.8V, 25.6mA, true, -38.4mA, 38.4mA);
	ATE()->SleepUs(100);

	ATE()->DCLevels()->Signal("EXCLK")->SignalEnable(DSM_CLOCK, true);
	ATE()->SleepMs(15);
	ATE()->PinPMU()->FV("RSTB", PD_RS_Voltage, 20mA, true, -20mA, 20mA);
	ATE()->SleepMs(100);


}

void HSC02A::PowerOff()
{
	ATE()->DCLevels()->Block("PowerOff")->Execute();
}

void HSC02A::PowerDownSequence()
{
	ATE()->DIB()->Signal("I2C_EN")->CBitsOff();
	ATE()->SleepMs(5);
	ATE()->DCLevels()->Signal("EXCLK")->SignalEnable(DSM_CLOCK, false);
	ATE()->SleepMs(5);
	ATE()->PinPMU()->FV("RSTB", 0V, 2mA, true, -2mA, 2mA);
	ATE()->SleepMs(10);
	PowerOff();
	ATE()->SleepMs(100);
	//ATE()->DPS()->FV("CVDD",0V,512mA,true,-100mA,100mA);
	ATE()->PinPMU()->FV("AllPMU",0V,2mA,true,-2mA,2mA);
	ATE()->PinPMU()->FV("AllUTP",0V,2mA,true,-2mA,2mA);
	ATE()->SleepMs(100);
	DisConnectAllPins();
}

void HSC02A::PowerDownSequenceIDD()
{
	ATE()->DIB()->Signal("I2C_EN")->CBitsOff();
	ATE()->SleepMs(5);
	ATE()->DCLevels()->Signal("EXCLK")->SignalEnable(DSM_CLOCK, false);
	ATE()->SleepMs(5);
	ATE()->PinPMU()->FV("RSTB", 0V, 2mA, true, -2mA, 2mA);
	ATE()->SleepMs(1);
	ATE()->DPS()->FV("AVDD", 0V, 25.6mA, true, -38.4mA, 38.4mA);
	ATE()->SleepUs(100);
	ATE()->DPS()->FV("DVDD", 0V, 25.6mA, true, -38.4mA, 38.4mA);
	ATE()->SleepUs(100);
	ATE()->DPS()->FV("IOVDD", 0V, 25.6mA, true, -38.4mA, 38.4mA);
	ATE()->SleepUs(100);
	ATE()->PinPMU()->FV("AllPMU",0V,2mA,true,-2mA,2mA);
	ATE()->PinPMU()->FV("AllUTP",0V,2mA,true,-2mA,2mA);
	ATE()->SleepMs(5);
	DisConnectAllPins();
}

void HSC02A::PowerDownSequenceLight()
{
	ATE()->DIB()->Signal("I2C_EN")->CBitsOff();
	ATE()->SleepMs(5);
	ATE()->DCLevels()->Signal("EXCLK")->SignalEnable(DSM_CLOCK, false);
	ATE()->SleepMs(5);
	ATE()->PinPMU()->FV("RSTB", 0V, 2mA, true, -2mA, 2mA);
	ATE()->SleepMs(1);
	ATE()->DPS()->FV("AVDD", 0V, 25.6mA, true, -38.4mA, 38.4mA);
	ATE()->SleepUs(100);
	ATE()->DPS()->FV("DVDD", 0V, 25.6mA, true, -38.4mA, 38.4mA);
	ATE()->SleepUs(100);
	ATE()->DPS()->FV("IOVDD", 0V, 25.6mA, true, -38.4mA, 38.4mA);
	ATE()->SleepUs(100);
	ATE()->PinPMU()->FV("AllPMU",0V,2mA,true,-2mA,2mA);
	ATE()->PinPMU()->FV("AllUTP",0V,2mA,true,-2mA,2mA);
	ATE()->SleepMs(5);
	//DisConnectAllPins();
}

bool HSC02A::ReadRegisterFromfile()
{
	m_RegisterMap.clear();
	vector<IICRegisterStru> tempVec;

	auto readFileFun = [&](std::string readFilePath)->bool {
		std::ifstream in(readFilePath, std::ios::in);
		tempVec.clear();
		if (in)
		{
			IICRegisterStru tempData;
			tempData.slaveId = 0;
			while (in >> hex >> tempData.address >> tempData.data)
			{
				tempVec.push_back(tempData);
			}
			in.close();
			return true;
		}
		else
		{
			in.close();
			return false;
		}
	};

	using boost::property_tree::ptree;
	std::string xmlFilePath = "Configure\\RegisterMap.xml";
	std::ifstream readFile;
	readFile.open(xmlFilePath, ios::in);
	if (readFile)
	{
		readFile.close();
		try {
			ptree pt;
			read_xml(xmlFilePath, pt);
			auto root = pt.get_child("Blocks");
			std::string productName, testItemName, filePath;
			for (auto &product : root)
			{
				productName = product.second.get<std::string>("<xmlattr>.name");
				for (auto &testItem : product.second)
				{
					if (testItem.first == "TestItem")
					{
						testItemName = testItem.second.get<std::string>("<xmlattr>.name");
						filePath = testItem.second.get<std::string>("Path");
						if (readFileFun(filePath))
						{
							m_RegisterMap[testItemName] = tempVec;
						}
					}
				}
			}
		}
		catch (std::exception &e)
		{
			std::cout << e.what() << std::endl;
		}
	}
	else
	{
		readFile.close();
		return false;
	}

	for (std::map<std::string, IICRegisterBlockStru>::iterator iter = m_RegisterBlockMap.begin(); iter != m_RegisterBlockMap.end(); iter++)
	{
		IICRegisterBlockStru& registerBlock = iter->second;
		if (registerBlock.pRegisterDataArr)
		{
			delete[] registerBlock.pRegisterDataArr;
			registerBlock.pRegisterDataArr = NULL;
		}
	}
	m_RegisterBlockMap.clear();
	for (std::map<std::string, std::vector<IICRegisterStru>>::iterator iter = m_RegisterMap.begin(); iter != m_RegisterMap.end(); iter++)
	{
		std::string testItem = iter->first;
		const std::vector<IICRegisterStru>& vIICRegisterStru = iter->second;
		IICRegisterBlockStru registerBlock = {};
		registerBlock.registerNum = vIICRegisterStru.size();
		if (registerBlock.registerNum > 0)
		{
			registerBlock.pRegisterDataArr = new EVEREST::EvtUnionData[registerBlock.registerNum];
			memset(registerBlock.pRegisterDataArr, 0, registerBlock.registerNum * sizeof(EVEREST::EvtUnionData));
			for (unsigned int i = 0; i < registerBlock.registerNum; i++)
			{
				EVEREST::EvtUnionData registerData = {};
				registerData.registerOffset = vIICRegisterStru[i].address;
				registerData.registerVal = vIICRegisterStru[i].data;
				memcpy_s(reinterpret_cast<char*>(registerBlock.pRegisterDataArr) + i * sizeof(EVEREST::EvtUnionData),
					sizeof(EVEREST::EvtUnionData), &registerData, sizeof(EVEREST::EvtUnionData));
			}
			m_RegisterBlockMap[testItem] = registerBlock;
		}
	}

	return true;
}

void HSC02A::DisConnectAllPins()
{
	//auto a = ATE();
	//auto dps = a->DPS();
	//auto signal = dps->Signal("AllDps");
	ATE()->DPS()->Signal("AllDPS")->Disconnect();
	ATE()->PinPMU()->Signal("AllPMU")->Disconnect();
	ATE()->PinPMU()->Signal("ALLUTP")->Disconnect();
}
bool HSC02A::ReadRegister(std::string testItem)
{
	EVTSYS(LEVEL_ERROR,"Site\tAddress\tData\n");
	EvtIICInterface* siteIIC = ATE()->Bus()->IIC("SCL", "SDA");
	siteIIC->Config(IIC_SLAVE_ID, false, IicFreq / 100, IicAddrWidthBytes, IicDataWidthBytes);
	if (m_RegisterMap.find(testItem) != m_RegisterMap.end())
	{
		if (!m_RegisterMap[testItem].empty())
		{
			unsigned long data = 0;
			unsigned int cnt = 0;
			vector<unsigned int> vErrorCount(16, 0);
			EVT_EACH_SITE_BEGIN
			unsigned int errorcnt = 0;
			ATE()->SleepMs(10);
			for (auto iter = m_RegisterMap[testItem].begin(); iter != m_RegisterMap[testItem].end(); iter++)
			{
				if (1)
				{
					
					siteIIC->Read(iter->address, data);
					//vErrorCount[activeSite]++;
					//EVTSYS(LEVEL_ERROR, "Site%d Index: %04d,Fail Address: 0x%04x, Src Data: 0x%04x, Dest Data: 0x%04x \r\n", activeSite, iter->address, iter->data, data);
					EVTSYS(LEVEL_ERROR, "%04d\t0x%04x\t0x%04x\n", activeSite, iter->address, data);
				}
			}
			EVT_EACH_SITE_END
		}
		else
		{
			EVTSYS(LEVEL_ERROR, "%s Register is empty.\n", testItem.c_str());
			return false;
		}
	}
	else
	{
		EVTSYS(LEVEL_MSG, "%s Register is not find.\n", testItem.c_str());
		return false;
	}
}
bool HSC02A::WriteRegister(std::string testItem, bool bEnableReadBack)
{
	bool bRet = true;
	vector<unsigned int> vSites;
	ATE()->Site()->GetActive(vSites);

	//for (auto siteNum : vSites)
	//{
	//	ATE()->Site()->SetEOSEnable(siteNum, true);
	//}
	EvtIICInterface* siteIIC = ATE()->Bus()->IIC("SCL", "SDA");
	siteIIC->Config(IIC_SLAVE_ID, false, IicFreq / 100, IicAddrWidthBytes, IicDataWidthBytes);
	if (m_RegisterMap.find(testItem) != m_RegisterMap.end())
	{
		if (!m_RegisterMap[testItem].empty())
		{
			unsigned long data = 0;
			unsigned int cnt = 0;
			vector<unsigned int> vErrorCount(16, 0);
			for (auto iter = m_RegisterMap[testItem].begin(); iter != m_RegisterMap[testItem].end(); iter++)
			{
				cnt++;
				if (!siteIIC->Write(iter->address, iter->data, 100))
				{
					bRet = false;
					EVTSYS(LEVEL_DEBUG, "write IIC cmd fail cnt=%d, addr=0x%x, data=0x%x\r\n", cnt, iter->address, iter->data);
					//break;
				}
				//
				//siteIIC->Read(iter->address, data);
				//if (data != iter->data)
				//{
				//	
				//	EVTSYS(LEVEL_ERROR, "Fail Address: 0x%04x, Src Data: 0x%04x, Dest Data: 0x%04x \r\n",iter->address, iter->data, data);
				//}

			}
			EVT_EACH_SITE_BEGIN
				unsigned int errorcnt = 0;
			ATE()->SleepMs(10);
			for (auto iter = m_RegisterMap[testItem].begin(); iter != m_RegisterMap[testItem].end(); iter++)
			{
				if (bEnableReadBack)
				{
					errorcnt++;
					siteIIC->Read(iter->address, data);
					if (data != iter->data)
					{
						vErrorCount[activeSite]++;
						EVTSYS(LEVEL_ERROR, "Site%d Index: %04d,Fail Address: 0x%04x, Src Data: 0x%04x, Dest Data: 0x%04x \r\n", activeSite, errorcnt, iter->address, iter->data, data);
						bRet = false;
					}
				}
			}
			//siteIIC->Read(iter->address, data);

			EVT_EACH_SITE_END
				EVT_EACH_SITE_BEGIN
				if (vErrorCount[activeSite] > 0)
				{
					EVTSYS(LEVEL_ERROR, "Site%d, %s there are total %d register write and read not same. \r\n", activeSite, testItem.c_str(), vErrorCount[activeSite]);
				}
			EVT_EACH_SITE_END
				//return bRet;

		}
		else
		{
			EVTSYS(LEVEL_ERROR, "%s Register is empty.\n", testItem.c_str());
			return false;
		}
	}
	else
	{
		EVTSYS(LEVEL_ERROR, "%s Register is not find.\n", testItem.c_str());
		return false;
	}
	//for (auto siteNum : vSites)
	//{
	//	ATE()->Site()->SetEOSEnable(siteNum, false);
	//}
	//ATE()->Site()->ClearEOS(vSites);
	return bRet;
}

bool HSC02A::WriteRegisterBlock(std::string testItem, bool bEnableReadBack)
{
	bool bRet = false;
	EvtIICInterface* siteIIC = ATE()->Bus()->IIC("SCL", "SDA");
	//siteIIC->Config(IIC_SLAVE_ID, false, IicFreq / 100, IicAddrWidthBytes, IicDataWidthBytes);
	if (!siteIIC)
	{
		EVTSYS(LEVEL_TRACE, "%s %s siteIIC is NULL.\n", __FUNCTION__, testItem.c_str());
		return false;
	}

	if (m_RegisterBlockMap.find(testItem) != m_RegisterBlockMap.end())
	{
		IICRegisterBlockStru& registerBlock = m_RegisterBlockMap[testItem];
		unsigned int uiRegisterArrNum = registerBlock.registerNum;
		EVEREST::EvtUnionData* pRegisterDataArr = registerBlock.pRegisterDataArr;
		if (uiRegisterArrNum > 0 && pRegisterDataArr != NULL)
		{
			for (unsigned int m = 0; m < 2; m++)
			{
				siteIIC->Config(IIC_SLAVE_ID, false, IicFreq / 100, IicAddrWidthBytes, IicDataWidthBytes);
				bRet = siteIIC->BlockWrite(pRegisterDataArr, uiRegisterArrNum, IicAddrWidthBytes, IicDataWidthBytes, 100, bEnableReadBack);
				if (bRet)
				{
					if (m != 0)
					{
						EVTSYS(LEVEL_TRACE, "%s %s BlockWrite success, retry = %d.\n", __FUNCTION__, testItem.c_str(), m);
					}

					if (bEnableReadBack)
					{
						for (unsigned int i = 0; i < uiRegisterArrNum; i++)
						{
							const EVEREST::EvtUnionData* pRegisterData = &pRegisterDataArr[i];
							unsigned long readData = 0;
							const unsigned long addr = static_cast<unsigned long>(pRegisterData->registerOffset);
							unsigned long writeData = static_cast<unsigned long>(pRegisterData->registerVal);

							//same addr may write more times
							//find last writeData in this addr
							for (unsigned int j = uiRegisterArrNum - 1; j > i; j--)
							{
								const EVEREST::EvtUnionData* pRegisterDataSearch = &pRegisterDataArr[j];
								const unsigned long addrSearch = static_cast<unsigned long>(pRegisterDataSearch->registerOffset);
								const unsigned long writeDataSearch = static_cast<unsigned long>(pRegisterDataSearch->registerVal);
								if (addr == addrSearch)
								{
									writeData = writeDataSearch;
									break;
								}
							}

							bRet = siteIIC->Read(addr, readData);
							if (bRet)
							{
								if (readData != writeData)
								{
									EVTSYS(LEVEL_TRACE, "Read Fail Address: 0x%04x, write Data: 0x%04x, read Data: 0x%04x \r\n", addr, writeData, readData);
								}
							}
							else
							{
								EVTSYS(LEVEL_TRACE, "Read Fail Address: 0x%04x, write Data: 0x%04x, read Data: 0x%04x \r\n", addr, writeData, readData);
								return false;
							}
						}
					}
					break;
				}
				else
				{
					EVTSYS(LEVEL_TRACE, "%s %s BlockWrite fail, retry = %d.\n", __FUNCTION__, testItem.c_str(), m);
					ATE()->SleepMs(5);
					continue;
				}
			}

			if (!bRet)
			{
				return false;
			}

		}
		else
		{
			EVTSYS(LEVEL_ERROR, "%s RegisterBlock is empty.\n", testItem.c_str());
			return false;
		}
	}
	else
	{
		EVTSYS(LEVEL_ERROR, "%s RegisterBlock is not find.\n", testItem.c_str());
		return false;
	}
	return bRet;
}

static void SaveImage(const char* pImageBuffer, int imageWidth, int imageHeight, int imageCount)
{
	char buf[256] = {};
	std::time_t time = std::time(nullptr);
	tm local;
	localtime_s(&local, &time);
	strftime(buf, 256, "%Y-%m-%d-%H-%M-%S", &local);

	char nameBuf[256];
	snprintf(nameBuf, 256, "D:\\image\\SamSung\\%s_%s.raw", "normol", buf);
	std::ofstream os(nameBuf, std::ios::binary | std::ios::out | std::ios::app | std::ios::ate);

	unsigned int imageSize = imageWidth * imageHeight * sizeof(unsigned short);
	const char *ResultBuf = pImageBuffer;
	for (int i = 0; i < imageCount; ++i)
	{
		os.write((const char*)(ResultBuf + imageSize * i), imageSize);
	}
	os.close();
}

void HSC02A::OS_VSS(const std::string &testName, Test_Result &result)
{
	
	PowerOff();


	//vector<double> SDA_Pin;
    //ATE()->PinPMU()->FVMV("SDA", 1V, 200uA, true, -200uA, 200uA,1mS,100,SDA_Pin);
    //ATE()->PinPMU()->FV("SDA", 0V, 200uA, true, -200uA, 200uA);
	vector<unsigned int> vSites;
	ATE()->Site()->GetActive(vSites);
	auto params = ATE()->Test(testName)->GetParamValue("DCMeasure");
	ATE()->DCMeasure(params)->Execute(DCM_EXEMODE::DCM_PIN_G, true);



	return;
}

void HSC02A::OS_VDD(const std::string &testName, Test_Result &result)
{

	PowerOff();

	vector<unsigned int> vSites;
	ATE()->Site()->GetActive(vSites);
	auto params = ATE()->Test(testName)->GetParamValue("DCMeasure");


	ATE()->DCMeasure(params)->Execute(DCM_EXEMODE::DCM_PIN_G, true);


	return;
}

void HSC02A::PowerShort(const std::string &testName, Test_Result &result)
{
	PowerOff();
	//ATE()->DIB()->Signal("HVDD_EN")->CBitsOn();
	//ATE()->DIB()->Signal("NVDD_EN")->CBitsOn();
	//ATE()->PinPMU()->FV("NVDD_HVDD", 0V, 409.6uA, true, -384uA, 384uA);
	ATE()->SleepMs(50);
	DisConnectAllPins();
	vector<unsigned int> vSites;
	ATE()->Site()->GetActive(vSites);
	unsigned int testNum = ATE()->Test(testName)->GetTestNumber();
	//unsigned short testNum = ATE()->Test(testName)->GetTestNumber();
	//auto params = ATE()->Test(testName)->GetParamValue("DCMeasure");

	//ATE()->DCMeasure(params)->Execute(DCM_EXEMODE::DCM_PIN_G, true);
	//ATE()->DIB()->Signal("HVDD_EN")->CBitsOff();
	//ATE()->DIB()->Signal("NVDD_EN")->CBitsOff();

	vector<double> AVDD;
	vector<double> DVDD;
	vector<double> IOVDD;
	vector<double> CVDD;


	ATE()->DPS()->FI("AVDD", -100uA, 256uA, true, -2V, 2V);
	ATE()->DPS()->FI("DVDD", -100uA, 256uA, true, -2V, 2V);
	ATE()->DPS()->FI("IOVDD", -100uA, 256uA, true, -2V, 2V);
	//ATE()->DPS()->FI("CVDD", -100uA, 256uA, true, -2V, 2V);
	ATE()->SleepMs(300);
	ATE()->DPS()->MV("AVDD", 50mS, 10mS, 10000, AVDD);
	ATE()->DPS()->MV("DVDD", 50mS, 10mS, 10000, DVDD);
	ATE()->DPS()->MV("IOVDD", 50mS, 10mS, 10000, IOVDD);
	//ATE()->DPS()->MV("CVDD", 50mS, 10mS, 10000, CVDD);
	ATE()->Datalog()->SetParametricResult(vSites, "AVDD", testNum, AVDD);
	ATE()->Datalog()->SetParametricResult(vSites, "DVDD", testNum+1, DVDD);
	ATE()->Datalog()->SetParametricResult(vSites, "IOVDD", testNum+2, IOVDD);
	//ATE()->Datalog()->SetParametricResult(vSites, "CVDD", testNum+3, CVDD);

	//ATE()->DPS()->FV("AllDPS",0V,500mA,true,-100mA,100mA);
	PowerOff();
	DisConnectAllPins();
	return;
}

void HSC02A::IIL(const std::string &testName, Test_Result &result)
{
	vector<unsigned int> vSites;
	ATE()->Site()->GetActive(vSites);
	unsigned short testNum = ATE()->Test(testName)->GetTestNumber();



	PowerOn();

	vector<double> SDA;
	vector<double> SCL;
	vector<double> EXCLK;

	vector<double> TEST;
	vector<double> LDOEN;
	vector<double> RSTB;
	vector<double> I2CID;

	vector<double> PCLK;
	vector<double> VCYN;
	vector<double> HREF;
	vector<double> D0;
	vector<double> D1;
	vector<double> D2;
	vector<double> D3;
	vector<double> D4;
	vector<double> D5;
	vector<double> D6;
	vector<double> D7;
	vector<double> D8;
	vector<double> D9;

	//vector<unsigned int> vSites;
	//ATE()->Site()->GetActive(vSites);
	//unsigned short testNum = ATE()->Test(testName)->GetTestNumber();
	//auto params = ATE()->Test(testName)->GetParamValue("DCMeasure");

	//ATE()->DCMeasure(params)->Execute(DCM_EXEMODE::DCM_PIN_G, true);
	ATE()->PinPMU()->FV("Leakage_PMU", PD_RS_Voltage, 2uA, true, -2.2uA, 2.2uA);
	ATE()->PinPMU()->FV("Leakage_UTP", PD_RS_Voltage, 4.096uA, true, -4.096uA, 4.096uA);
	ATE()->PinPMU()->FV("AllDVP", PD_RS_Voltage, 2uA, true, -2.2uA, 2.2uA);
	//PMU
	ATE()->PinPMU()->FVMI("SDA", 0V, 2uA, true, -2.2uA, 2.2uA, 1mS, 1000, SDA);
	ATE()->PinPMU()->FV("SDA", PD_RS_Voltage, 2uA, true, -2.2uA, 2.2uA);

	ATE()->PinPMU()->FVMI("SCL", 0V, 2uA, true, -2.2uA, 2.2uA, 1mS, 1000, SCL);
	ATE()->PinPMU()->FV("SCL", PD_RS_Voltage, 2uA, true, -2.2uA, 2.2uA);

	ATE()->PinPMU()->FVMI("EXCLK", 0V, 2uA, true, -2.2uA, 2.2uA, 1mS, 1000, EXCLK);
	ATE()->PinPMU()->FV("EXCLK", PD_RS_Voltage, 2uA, true, -2.2uA, 2.2uA);
	//UTP
	ATE()->PinPMU()->FVMI("TEST", 0V, 4.096uA, true, -4.096uA, 4.096uA,1mS,1000, TEST);
	ATE()->PinPMU()->FV("TEST", PD_RS_Voltage, 4.096uA, true, -4.096uA, 4.096uA);

	ATE()->PinPMU()->FVMI("LDOEN", 0V, 4.096uA, true, -4.096uA, 4.096uA, 1mS, 1000, LDOEN);
	ATE()->PinPMU()->FV("LDOEN", PD_RS_Voltage, 4.096uA, true, -4.096uA, 4.096uA);

	

	ATE()->PinPMU()->FVMI("I2CID", 0V, 4.096uA, true, -4.096uA, 4.096uA, 1mS, 1000, I2CID);
	ATE()->PinPMU()->FV("I2CID", PD_RS_Voltage, 4.096uA, true, -4.096uA, 4.096uA);

	ATE()->PinPMU()->FVMI("RSTB", 0V, 4.096uA, true, -4.096uA, 4.096uA, 1mS, 1000, RSTB);
	ATE()->PinPMU()->FV("RSTB", PD_RS_Voltage, 4.096uA, true, -4.096uA, 4.096uA);
	//DVP
	/*ATE()->PinPMU()->FVMI("PCLK", 0V, 20uA, true, -22uA, 22uA, 1mS, 1000, PCLK);
	ATE()->PinPMU()->FV("PCLK", PD_RS_Voltage, 20uA, true, -22uA, 22uA);

	ATE()->PinPMU()->FVMI("VCYN", 0V, 20uA, true, -22uA, 22uA, 1mS, 1000, VCYN);
	ATE()->PinPMU()->FV("VCYN", PD_RS_Voltage, 20uA, true, -22uA, 22uA);

	ATE()->PinPMU()->FVMI("HREF", 0V, 20uA, true, -22uA, 22uA, 1mS, 1000, HREF);
	ATE()->PinPMU()->FV("HREF", PD_RS_Voltage, 20uA, true, -22uA, 22uA);

	ATE()->PinPMU()->FVMI("D0", 0V, 20uA, true, -22uA, 22uA, 1mS, 1000, D0);
	ATE()->PinPMU()->FV("D0", PD_RS_Voltage, 20uA, true, -22uA, 22uA);

	ATE()->PinPMU()->FVMI("D1", 0V, 20uA, true, -22uA, 22uA, 1mS, 1000, D1);
	ATE()->PinPMU()->FV("D1", PD_RS_Voltage, 20uA, true, -22uA, 22uA);

	ATE()->PinPMU()->FVMI("D2", 0V, 20uA, true, -22uA, 22uA, 1mS, 1000, D2);
	ATE()->PinPMU()->FV("D2", PD_RS_Voltage, 20uA, true, -22uA, 22uA);

	ATE()->PinPMU()->FVMI("D3", 0V, 20uA, true, -22uA, 22uA, 1mS, 1000, D3);
	ATE()->PinPMU()->FV("D3", PD_RS_Voltage, 20uA, true, -22uA, 22uA);

	ATE()->PinPMU()->FVMI("D4", 0V, 20uA, true, -22uA, 22uA, 1mS, 1000, D4);
	ATE()->PinPMU()->FV("D4", PD_RS_Voltage, 20uA, true, -22uA, 22uA);

	ATE()->PinPMU()->FVMI("D5", 0V, 20uA, true, -22uA, 22uA, 1mS, 1000, D5);
	ATE()->PinPMU()->FV("D5", PD_RS_Voltage, 20uA, true, -22uA, 22uA);

	ATE()->PinPMU()->FVMI("D6", 0V, 20uA, true, -22uA, 22uA, 1mS, 1000, D6);
	ATE()->PinPMU()->FV("D6", PD_RS_Voltage, 20uA, true, -22uA, 22uA);

	ATE()->PinPMU()->FVMI("D7", 0V, 20uA, true, -22uA, 22uA, 1mS, 1000, D7);
	ATE()->PinPMU()->FV("D7", PD_RS_Voltage, 20uA, true, -22uA, 22uA);

	ATE()->PinPMU()->FVMI("D8", 0V, 20uA, true, -22uA, 22uA, 1mS, 1000, D8);
	ATE()->PinPMU()->FV("D8", PD_RS_Voltage, 20uA, true, -22uA, 22uA);

	ATE()->PinPMU()->FVMI("D9", 0V, 20uA, true, -22uA, 22uA, 1mS, 1000, D9);
	ATE()->PinPMU()->FV("D9", PD_RS_Voltage, 20uA, true, -22uA, 22uA);


	ATE()->PinPMU()->FVMI("RSTB", 0V, 40.96uA, true, -22uA, 22uA, 1mS, 1000, RSTB);
	ATE()->PinPMU()->FV("RSTB", PD_RS_Voltage, 40.96uA, true, -22uA, 22uA);*/

	//PMU
	EVT_EACH_SITE_BEGIN
	ATE()->Datalog()->SetParametricResult(activeSite, "SDA", testNum, SDA);
	ATE()->Datalog()->SetParametricResult(activeSite, "SCL", testNum+1, SCL);
	ATE()->Datalog()->SetParametricResult(activeSite, "EXCLK", testNum+2, EXCLK);
	//UTP								  
	ATE()->Datalog()->SetParametricResult(activeSite, "TEST", testNum + 3, TEST);
	ATE()->Datalog()->SetParametricResult(activeSite, "LDOEN", testNum + 4, LDOEN);
	ATE()->Datalog()->SetParametricResult(activeSite, "RSTB", testNum + 5, RSTB);
	ATE()->Datalog()->SetParametricResult(activeSite, "I2CID", testNum + 6, I2CID);
	EVT_EACH_SITE_END
	//DVP
	/*ATE()->Datalog()->SetParametricResult(vSites, "PCLK", testNum + 7, PCLK);
	ATE()->Datalog()->SetParametricResult(vSites, "VCYN", testNum + 8, VCYN);
	ATE()->Datalog()->SetParametricResult(vSites, "HREF", testNum + 9, HREF);
	ATE()->Datalog()->SetParametricResult(vSites, "D0", testNum + 10, D0);
	ATE()->Datalog()->SetParametricResult(vSites, "D1", testNum + 11, D1);
	ATE()->Datalog()->SetParametricResult(vSites, "D2", testNum + 12, D2);
	ATE()->Datalog()->SetParametricResult(vSites, "D3", testNum + 13, D3);
	ATE()->Datalog()->SetParametricResult(vSites, "D4", testNum + 14, D4);
	ATE()->Datalog()->SetParametricResult(vSites, "D5", testNum + 15, D5);
	ATE()->Datalog()->SetParametricResult(vSites, "D6", testNum + 16, D6);
	ATE()->Datalog()->SetParametricResult(vSites, "D7", testNum + 17, D7);
	ATE()->Datalog()->SetParametricResult(vSites, "D8", testNum + 18, D8);
	ATE()->Datalog()->SetParametricResult(vSites, "D9", testNum + 19, D9);*/



	ATE()->PinPMU()->FV("Leakage_PMU", 0V, 2uA, true, -2.2uA, 2.2uA);
	ATE()->PinPMU()->FV("Leakage_UTP", 0V, 4.096uA, true, -4.096uA, 4.096uA);
	//ATE()->PinPMU()->FV("AllDVP", 0V, 2uA, true, -2.2uA, 2.2uA);
	PowerOff();
	DisConnectAllPins();
	return;
}

void HSC02A::IIH(const std::string &testName, Test_Result &result)
{
	PowerOn();
	vector<unsigned int> vSites;
	ATE()->Site()->GetActive(vSites);
	unsigned short testNum = ATE()->Test(testName)->GetTestNumber();
	//auto params = ATE()->Test(testName)->GetParamValue("DCMeasure");
	//ATE()->DCMeasure(params)->Execute(DCM_EXEMODE::DCM_PIN_G, true);
	vector<double> SDA;
	vector<double> SCL;
	vector<double> EXCLK;

	vector<double> TEST;
	vector<double> LDOEN;
	vector<double> RSTB;
	vector<double> I2CID;

	vector<double> PCLK;
	vector<double> VCYN;
	vector<double> HREF;
	vector<double> D0;
	vector<double> D1;
	vector<double> D2;
	vector<double> D3;
	vector<double> D4;
	vector<double> D5;
	vector<double> D6;
	vector<double> D7;
	vector<double> D8;
	vector<double> D9;

	ATE()->PinPMU()->FV("Leakage_PMU", 0V, 2uA, true, -2.2uA, 2.2uA);
	ATE()->PinPMU()->FV("Leakage_UTP", 0V, 4.096uA, true, -4.096uA, 4.096uA);
	ATE()->PinPMU()->FV("AllDVP", 0V, 2uA, true, -2.2uA, 2.2uA);
	//PMU
	ATE()->PinPMU()->FVMI("SDA", PD_RS_Voltage, 2uA, true, -2.2uA, 2.2uA, 1mS, 1000, SDA);
	ATE()->PinPMU()->FV("SDA", 0V, 2uA, true, -2.2uA, 2.2uA);

	ATE()->PinPMU()->FVMI("SCL", PD_RS_Voltage, 2uA, true, -2.2uA, 2.2uA, 1mS, 1000, SCL);
	ATE()->PinPMU()->FV("SCL", 0V, 2uA, true, -2.2uA, 2.2uA);

	ATE()->PinPMU()->FVMI("EXCLK", PD_RS_Voltage, 2uA, true, -2.2uA, 2.2uA, 1mS, 1000, EXCLK);
	ATE()->PinPMU()->FV("EXCLK", 0V, 2uA, true, -2.2uA, 2.2uA);
	//UTP
	ATE()->PinPMU()->FVMI("TEST", PD_RS_Voltage, 4.096uA, true, -4.096uA, 4.096uA, 1mS, 1000, TEST);
	ATE()->PinPMU()->FV("TEST", 0V, 4.096uA, true, -4.096uA, 4.096uA);

	ATE()->PinPMU()->FVMI("LDOEN", PD_RS_Voltage, 4.096uA, true, -4.096uA, 4.096uA, 1mS, 1000, LDOEN);
	ATE()->PinPMU()->FV("LDOEN", 0V, 4.096uA, true, -4.096uA, 4.096uA);

	

	ATE()->PinPMU()->FVMI("I2CID", PD_RS_Voltage, 4.096uA, true, -4.096uA, 4.096uA, 1mS, 1000, I2CID);
	ATE()->PinPMU()->FV("I2CID", 0V, 4.096uA, true, -4.096uA, 4.096uA);

	ATE()->PinPMU()->FVMI("RSTB", PD_RS_Voltage, 4.096uA, true, -4.096uA, 4.096uA,1mS, 1000, RSTB);
	ATE()->PinPMU()->FV("RSTB", 0V, 4.096uA, true, -4.096uA, 4.096uA);
	//DVP
	/*ATE()->PinPMU()->FVMI("PCLK", PD_RS_Voltage, 20uA, true, -22uA, 22uA, 1mS, 1000, PCLK);
	ATE()->PinPMU()->FV("PCLK", 0V, 20uA, true, -22uA, 22uA);

	ATE()->PinPMU()->FVMI("VCYN", PD_RS_Voltage, 20uA, true, -22uA, 22uA, 1mS, 1000, VCYN);
	ATE()->PinPMU()->FV("VCYN", 0V, 20uA, true, -22uA, 22uA);

	ATE()->PinPMU()->FVMI("HREF", PD_RS_Voltage, 20uA, true, -22uA, 22uA, 1mS, 1000, HREF);
	ATE()->PinPMU()->FV("HREF", 0V, 20uA, true, -22uA, 22uA);

	ATE()->PinPMU()->FVMI("D0", PD_RS_Voltage, 20uA, true, -22uA, 22uA, 1mS, 1000, D0);
	ATE()->PinPMU()->FV("D0", 0V, 20uA, true, -22uA, 22uA);

	ATE()->PinPMU()->FVMI("D1", PD_RS_Voltage, 20uA, true, -22uA, 22uA, 1mS, 1000, D1);
	ATE()->PinPMU()->FV("D1", 0V, 20uA, true, -22uA, 22uA);

	ATE()->PinPMU()->FVMI("D2", PD_RS_Voltage, 20uA, true, -22uA, 22uA, 1mS, 1000, D2);
	ATE()->PinPMU()->FV("D2", 0V, 20uA, true, -22uA, 22uA);

	ATE()->PinPMU()->FVMI("D3", PD_RS_Voltage, 20uA, true, -22uA, 22uA, 1mS, 1000, D3);
	ATE()->PinPMU()->FV("D3", 0V, 20uA, true, -22uA, 22uA);

	ATE()->PinPMU()->FVMI("D4", PD_RS_Voltage, 20uA, true, -22uA, 22uA, 1mS, 1000, D4);
	ATE()->PinPMU()->FV("D4", 0V, 20uA, true, -22uA, 22uA);

	ATE()->PinPMU()->FVMI("D5", PD_RS_Voltage, 20uA, true, -22uA, 22uA, 1mS, 1000, D5);
	ATE()->PinPMU()->FV("D5", 0V, 20uA, true, -22uA, 22uA);

	ATE()->PinPMU()->FVMI("D6", PD_RS_Voltage, 20uA, true, -22uA, 22uA, 1mS, 1000, D6);
	ATE()->PinPMU()->FV("D6", 0V, 20uA, true, -22uA, 22uA);

	ATE()->PinPMU()->FVMI("D7", PD_RS_Voltage, 20uA, true, -22uA, 22uA, 1mS, 1000, D7);
	ATE()->PinPMU()->FV("D7", 0V, 20uA, true, -22uA, 22uA);

	ATE()->PinPMU()->FVMI("D8", PD_RS_Voltage, 20uA, true, -22uA, 22uA, 1mS, 1000, D8);
	ATE()->PinPMU()->FV("D8", 0V, 20uA, true, -22uA, 22uA);

	ATE()->PinPMU()->FVMI("D9", PD_RS_Voltage, 20uA, true, -22uA, 22uA, 1mS, 1000, D9);
	ATE()->PinPMU()->FV("D9", 0V, 20uA, true, -22uA, 22uA);

	ATE()->PinPMU()->FVMI("RSTB", PD_RS_Voltage, 40.96uA, true, -22uA, 22uA, 1mS, 1000, RSTB);
	ATE()->PinPMU()->FV("RSTB", 0V, 40.96uA, true, -22uA, 22uA);*/
	//PMU
	ATE()->Datalog()->SetParametricResult(vSites, "SDA", testNum, SDA);
	ATE()->Datalog()->SetParametricResult(vSites, "SCL", testNum + 1, SCL);
	ATE()->Datalog()->SetParametricResult(vSites, "EXCLK", testNum + 2, EXCLK);
	//UTP
	ATE()->Datalog()->SetParametricResult(vSites, "TEST", testNum + 3, TEST);
	ATE()->Datalog()->SetParametricResult(vSites, "LDOEN", testNum + 4, LDOEN);
	ATE()->Datalog()->SetParametricResult(vSites, "RSTB", testNum + 5, RSTB);
	ATE()->Datalog()->SetParametricResult(vSites, "I2CID", testNum + 6, I2CID);
	//DVP
	/*ATE()->Datalog()->SetParametricResult(vSites, "PCLK", testNum + 7, PCLK);
	ATE()->Datalog()->SetParametricResult(vSites, "VCYN", testNum + 8, VCYN);
	ATE()->Datalog()->SetParametricResult(vSites, "HREF", testNum + 9, HREF);
	ATE()->Datalog()->SetParametricResult(vSites, "D0", testNum + 10, D0);
	ATE()->Datalog()->SetParametricResult(vSites, "D1", testNum + 11, D1);
	ATE()->Datalog()->SetParametricResult(vSites, "D2", testNum + 12, D2);
	ATE()->Datalog()->SetParametricResult(vSites, "D3", testNum + 13, D3);
	ATE()->Datalog()->SetParametricResult(vSites, "D4", testNum + 14, D4);
	ATE()->Datalog()->SetParametricResult(vSites, "D5", testNum + 15, D5);
	ATE()->Datalog()->SetParametricResult(vSites, "D6", testNum + 16, D6);
	ATE()->Datalog()->SetParametricResult(vSites, "D7", testNum + 17, D7);
	ATE()->Datalog()->SetParametricResult(vSites, "D8", testNum + 18, D8);
	ATE()->Datalog()->SetParametricResult(vSites, "D9", testNum + 19, D9);*/



	ATE()->PinPMU()->FV("Leakage_PMU", 0V, 2uA, true, -2.2uA, 2.2uA);
	ATE()->PinPMU()->FV("Leakage_UTP", 0V, 4.096uA, true, -4.096uA, 4.096uA);
	//ATE()->PinPMU()->FV("AllDVP", 0V, 2uA, true, -2.2uA, 2.2uA);


	
	PowerOff();
	DisConnectAllPins();
	return;
}

void HSC02A::IDD_Active(const std::string &testName, Test_Result &result)
{
	bool bRet = true;
	std::vector<unsigned int> vSites;
	ATE()->Site()->GetActive(vSites);
	unsigned short testNum = ATE()->Test(testName)->GetTestNumber();
	DisConnectAllPins();
	PowerUpSequenceIDD();
	//ATE()->DCLevels()->Block("IDD")->Execute();
	ATE()->SleepMs(100);
	//ATE()->HWInterface()->WriteRegister(vSites[0], 0x890000, 0, 0x3FC2FFFF);
	//ATE()->HWInterface()->WriteRegister(vSites[0], 0x890000, 0, 0x3FC3FFFF);
	//ATE()->DPS()->Signal("DVDD")->ChangeIRange(25.6mA, 0, -38.4mA, 38.4mA, true);
	//ATE()->DPS()->Signal("IOVDD")->ChangeIRange(25.6mA, 0, -38.4mA, 38.4mA, true);

	std::vector<double> AVDD;
	std::vector<double> DVDD;
	std::vector<double> IOVDD;
	double SumIDD;

	if (IicSpeed)
	{
		bRet = HSC02A::WriteRegisterBlock("IDD_Active");
	}
	else
	{
		bRet = HSC02A::WriteRegister("IDD_Active");
	}

	////1.打光测试IDD
	//for (auto &site : vSites)
	//{
	//	if (LedSiteMap.count(site) == 0) continue;
 //       unsigned short luxValue = m_LuxMap["Light"][site];//获取Light图的Lux值
	//	m_ledCtr.SetCctLux(LedSiteMap[site], luxValue);//打Light图的光

	//}

	ATE()->SleepMs(500);





	//ATE()->HWInterface()->WriteRegister(vSites[0], 0x890000, 0, 0xBFC181C0);
	ATE()->DPS()->MI("IOVDD", 10mS, 0, 10000, IOVDD);
	ATE()->DPS()->MI("AVDD", 10mS, 0, 10000, AVDD);
	ATE()->DPS()->MI("DVDD", 10mS, 0, 10000, DVDD);

	//for (int i = 0; i < 1000; i++)
	//{
	//	ATE()->SleepUs(896);
	//	ATE()->DPS()->MI("IOVDD", 0, 0,1,IOVDD);
	//	EVTSYS(LEVEL_ERROR, "%f", IOVDD[i]);
	//}

	//ATE()->HWInterface()->WriteRegister(vSites[0], 0x890000, 0, 0xBFC1CA00);
	
	//	EVTSYS(LEVEL_ERROR, "%.9f AVDD\n", AVDD[0]);
	//	EVTSYS(LEVEL_ERROR, "%.9f DVDD\n", DVDD[0]);
	//	EVTSYS(LEVEL_ERROR, "%.9f IOVDD\n", IOVDD[0]);
	//}
	
	EVT_EACH_SITE_BEGIN
		//2.打光测试IDD时注释掉下面3句
		AVDD[activeIndex] = AVDD[activeIndex] + m_IDDOffsets[activeSite].AVDD;
		DVDD[activeIndex] = DVDD[activeIndex] + m_IDDOffsets[activeSite].DVDD;
		IOVDD[activeIndex] = IOVDD[activeIndex] + m_IDDOffsets[activeSite].IOVDD;
		
		SumIDD = AVDD[activeIndex] + DVDD[activeIndex] + IOVDD[activeIndex];

		ATE()->Datalog()->SetParametricResult(activeSite, "AVDD", testNum, AVDD[activeIndex]);
		ATE()->Datalog()->SetParametricResult(activeSite, "DVDD", testNum + 1, DVDD[activeIndex]);
		ATE()->Datalog()->SetParametricResult(activeSite, "IOVDD", testNum + 2, IOVDD[activeIndex]);
		ATE()->Datalog()->SetParametricResult(activeSite,testNum + 3, SumIDD);

	EVT_EACH_SITE_END

	PowerDownSequenceIDD();
	//ATE()->PinPMU()->FV("AllPMU",0V,20mA,true,-20mA,20mA);
	//DisConnectAllPins();

	//for (auto &site : vSites)
	//{
	//	//light.set_brightness(site, m_LuxMap[testName][site]);
	//	if (LedSiteMap.count(site) == 0) continue;
	//	m_ledCtr.SetCctLux(LedSiteMap[site], 0);
	//}


	/*3.打光测试IDD
	for (auto &site : vSites)
	{
		if (LedSiteMap.count(site) == 0) continue;
		m_ledCtr.SetCctLux(LedSiteMap[site], 0);
	}*/

	return;
}

void HSC02A::IDD_StandBy(const std::string &testName, Test_Result &result)
{
	std::vector<unsigned int> vSites;
	ATE()->Site()->GetActive(vSites);
	unsigned short testNum = ATE()->Test(testName)->GetTestNumber();


	PowerUpSequence();
	EvtIICInterface* siteIIC = ATE()->Bus()->IIC("SCL", "SDA");
	siteIIC->Config(IIC_SLAVE_ID, false, IicFreq / 100, IicAddrWidthBytes, IicDataWidthBytes);
	bool bRet = true;
	if (IicSpeed)
	{
		bRet = HSC02A::WriteRegisterBlock("IDD_StandBy");
	}
	else
	{
		bRet = HSC02A::WriteRegister("IDD_StandBy");
	}

	std::vector<double> AVDD;
	std::vector<double> DVDD;
	std::vector<double> IOVDD;



	ATE()->DPS()->MI("AVDD", 10mS, 0, 16384, AVDD);
	ATE()->DPS()->MI("DVDD", 10mS, 0, 16384, DVDD);
	ATE()->DPS()->MI("IOVDD", 10mS, 0, 16384, IOVDD);

	ATE()->Datalog()->SetParametricResult(vSites, "AVDD", testNum, AVDD);
	ATE()->Datalog()->SetParametricResult(vSites, "DVDD", testNum + 1, DVDD);
	ATE()->Datalog()->SetParametricResult(vSites, "IOVDD", testNum + 2, IOVDD);

	PowerDownSequence();
	return;
}

void HSC02A::IDD_ShutDown(const std::string &testName, Test_Result &result)
{
	std::vector<unsigned int> vSites;
	ATE()->Site()->GetActive(vSites);
	unsigned short testNum = ATE()->Test(testName)->GetTestNumber();


	PowerUpSequence();
	EvtIICInterface* siteIIC = ATE()->Bus()->IIC("SCL", "SDA");
	siteIIC->Config(IIC_SLAVE_ID, false, IicFreq / 100, IicAddrWidthBytes, IicDataWidthBytes);
	bool bRet = true;
	if (IicSpeed)
	{
		bRet = HSC02A::WriteRegisterBlock("IDD_ShutDown");
	}
	else
	{
		bRet = HSC02A::WriteRegister("IDD_ShutDown");
	}

	std::vector<double> AVDD;
	std::vector<double> DVDD;
	std::vector<double> IOVDD;



	ATE()->DPS()->MI("AVDD", 10mS, 0, 16384, AVDD);
	ATE()->DPS()->MI("DVDD", 10mS, 0, 16384, DVDD);
	ATE()->DPS()->MI("IOVDD", 10mS, 0, 16384, IOVDD);

	ATE()->Datalog()->SetParametricResult(vSites, "AVDD", testNum, AVDD);
	ATE()->Datalog()->SetParametricResult(vSites, "DVDD", testNum + 1, DVDD);
	ATE()->Datalog()->SetParametricResult(vSites, "IOVDD", testNum + 2, IOVDD);

	PowerDownSequence();
	return;
}

void HSC02A::IIC(const std::string &testName, Test_Result &result)
{
	std::vector<unsigned int> vSites;
	ATE()->Site()->GetActive(vSites);
	unsigned short testNum = ATE()->Test(testName)->GetTestNumber();
	unsigned long rdata[27] = { 0 };
	PowerUpSequence();
	//bool bRet = true;
	/*EvtIICInterface* siteIIC = ATE()->Bus()->IIC("SCL", "SDA");
	siteIIC->Config(IIC_SLAVE_ID, false, IicFreq / 100, IicAddrWidthBytes, IicDataWidthBytes);*/
	//siteIIC->Write(0x0000, 0xff, 1000);
	//siteIIC->Read(0x0000, rdata[0], 1000);

	//siteIIC->Write(0x0020, 0x00, 1000);
	//siteIIC->Read(0x0020, rdata[2], 1000);
	//ATE()->SleepMs(8);
	//WriteRegister("Light", true);


	EVT_EACH_SITE_BEGIN

	bool bRet = true;
	bRet &= WriteRegister("IIC_00", true);
	PowerDownSequence();
	ATE()->SleepMs(10);
	PowerUpSequence();
	bRet &= WriteRegister("IIC_FF", true);
	



	TEST_RESULT result = TEST_RESULT::RESULT_FAIL;
	if (bRet == true) result = TEST_RESULT::RESULT_PASS;
	ATE()->Datalog()->SetParametricResult(activeSite, testNum, result);

	EVT_EACH_SITE_END
	PowerDownSequence();
	return;
}

//void HSC02A::Dark_DPCOn(const std::string &testName, Test_Result &result)
//{
//
//	bool bRet = true;
//	std::vector<unsigned int> vSites;
//	ATE()->Site()->GetActive(vSites);
//	unsigned short testNum = ATE()->Test(testName)->GetTestNumber();
//	DarkDPCOnTestNumber = testNum;
//	PowerUpSequence();
//	//EVTSYS(LEVEL_ERROR, "Befor write register:\n");
//	//ReadRegister("Dark_DPCOn");
//	
//	//ATE()->HWInterface()->WriteRegister(vSites[0], 0x890000, 0, 0xBFC181C0);  //RC补偿
//
//	ATE()->DCLevels()->Signal("PCLK")->Assign(DVP_PCLK);
//	ATE()->DCLevels()->Signal("HREF")->Assign(DVP_DE);
//	ATE()->DCLevels()->Signal("VCYN")->Assign(DVP_VS);
//	ATE()->DCLevels()->Signal("D0")->Assign(DVP_D0);
//	ATE()->DCLevels()->Signal("D1")->Assign(DVP_D1);
//	ATE()->DCLevels()->Signal("D2")->Assign(DVP_D2);
//	ATE()->DCLevels()->Signal("D3")->Assign(DVP_D3);
//	ATE()->DCLevels()->Signal("D4")->Assign(DVP_D4);
//	ATE()->DCLevels()->Signal("D5")->Assign(DVP_D5);
//	ATE()->DCLevels()->Signal("D6")->Assign(DVP_D6);
//	ATE()->DCLevels()->Signal("D7")->Assign(DVP_D7);
//	ATE()->DCLevels()->Signal("D8")->Assign(DVP_D8);
//	ATE()->DCLevels()->Signal("D9")->Assign(DVP_D9);
//
//	
//	//ATE()->DCLevels()->Block("ShutDownVterm")->Execute();
//	if (IicSpeed)
//	{
//		bRet = HSC02A::WriteRegisterBlock("Dark_DPCOn");
//	}
//	else
//	{
//		bRet = HSC02A::WriteRegister("Dark_DPCOn");
//	}
//	//ATE()->DCLevels()->Block("ShutDownVterm")->Execute();
//	ATE()->SleepMs(500);
//	//EVT_EACH_SITE_BEGIN
//	//EVTSYS(LEVEL_ERROR, "After write register:\n");
//	//ReadRegister("Dark_DPCOn");
//
//	unsigned short imageWidth = 480, imageHeight = 270;
//	unsigned char imageAverage = 2;
//	EVT_IMG::EVT_IMG_DATAWIDTH dataWidth = EVT_IMG::DATA_10BIT;
//
//	ATE()->SleepMs(50);
//
//	//bool bImg = true;
//	//boost::timer t;
//	CISImage()->DVPImageAcquireConfig(result, testName.c_str(), dataWidth, imageAverage, true, imageWidth, imageHeight, 5000, true);
//
//	//EVTSYS(LEVEL_DEBUG, "The time of Capture NorTest %f\n", t.elapsed());
//	//ATE()->HWInterface()->WriteRegister(vSites[0], 0x890000, 0, 0xBFC1CA00);  //取消RC补偿
//	PowerDownSequence();
//	
//
//	return;
//}

void HSC02A::Dark_DPCOn(const std::string &testName, Test_Result &result)
{
	bool bRet = true;
	std::vector<unsigned int> vSites;
	ATE()->Site()->GetActive(vSites);
	unsigned short testNum = ATE()->Test(testName)->GetTestNumber();
	DarkDPCOffTestNumber = testNum;
	DisConnectAllPins();
	PowerUpSequence();
	ATE()->DCLevels()->Signal("PCLK")->Assign(DVP_PCLK);
	ATE()->DCLevels()->Signal("HREF")->Assign(DVP_DE);
	ATE()->DCLevels()->Signal("VCYN")->Assign(DVP_VS);
	ATE()->DCLevels()->Signal("D0")->Assign(DVP_D0);
	ATE()->DCLevels()->Signal("D1")->Assign(DVP_D1);
	ATE()->DCLevels()->Signal("D2")->Assign(DVP_D2);
	ATE()->DCLevels()->Signal("D3")->Assign(DVP_D3);
	ATE()->DCLevels()->Signal("D4")->Assign(DVP_D4);
	ATE()->DCLevels()->Signal("D5")->Assign(DVP_D5);
	ATE()->DCLevels()->Signal("D6")->Assign(DVP_D6);
	ATE()->DCLevels()->Signal("D7")->Assign(DVP_D7);
	ATE()->DCLevels()->Signal("D8")->Assign(DVP_D8);
	ATE()->DCLevels()->Signal("D9")->Assign(DVP_D9);

	//ATE()->DCLevels()->Block("ImageVterm")->Execute();
	if (IicSpeed)
	{
		bRet = HSC02A::WriteRegisterBlock("Dark_DPCOn");
	}
	else
	{
		bRet = HSC02A::WriteRegister("Dark_DPCOn");
	}

	ATE()->SleepMs(500);


	unsigned short imageWidth = 480, imageHeight = 270;
	unsigned char imageAverage = 2;
	EVT_IMG::EVT_IMG_DATAWIDTH dataWidth = EVT_IMG::DATA_10BIT;


	ATE()->SleepMs(50);
	CISImage()->DVPImageAcquireConfig(result, "test_demo", dataWidth, imageAverage, true, imageWidth, imageHeight, 5000, true);
	ATE()->SleepMs(50);
	CISImage()->DVPImageAcquireConfig(result, testName.c_str(), dataWidth, imageAverage, true, imageWidth, imageHeight, 5000, true);

	//EVTSYS(LEVEL_DEBUG, "The time of Capture NorTest %f\n", t.elapsed());
	
	PowerDownSequence();


	return;
}

void HSC02A::Dark_DPCOff(const std::string &testName, Test_Result &result)
{
	bool bRet = true;
	std::vector<unsigned int> vSites;
	ATE()->Site()->GetActive(vSites);
	unsigned short testNum = ATE()->Test(testName)->GetTestNumber();
	DarkDPCOffTestNumber = testNum;
	PowerUpSequence();

	ATE()->DCLevels()->Signal("PCLK")->Assign(DVP_PCLK);
	ATE()->DCLevels()->Signal("HREF")->Assign(DVP_DE);
	ATE()->DCLevels()->Signal("VCYN")->Assign(DVP_VS);
	ATE()->DCLevels()->Signal("D0")->Assign(DVP_D0);
	ATE()->DCLevels()->Signal("D1")->Assign(DVP_D1);
	ATE()->DCLevels()->Signal("D2")->Assign(DVP_D2);
	ATE()->DCLevels()->Signal("D3")->Assign(DVP_D3);
	ATE()->DCLevels()->Signal("D4")->Assign(DVP_D4);
	ATE()->DCLevels()->Signal("D5")->Assign(DVP_D5);
	ATE()->DCLevels()->Signal("D6")->Assign(DVP_D6);
	ATE()->DCLevels()->Signal("D7")->Assign(DVP_D7);
	ATE()->DCLevels()->Signal("D8")->Assign(DVP_D8);
	ATE()->DCLevels()->Signal("D9")->Assign(DVP_D9);

	//ATE()->DCLevels()->Block("ImageVterm")->Execute();
	if (IicSpeed)
	{
		bRet = HSC02A::WriteRegisterBlock("Dark_DPCOff");
	}
	else
	{
		bRet = HSC02A::WriteRegister("Dark_DPCOff");
	}

	ATE()->SleepMs(500);


	unsigned short imageWidth = 480, imageHeight = 270;
	unsigned char imageAverage = 2;
	EVT_IMG::EVT_IMG_DATAWIDTH dataWidth = EVT_IMG::DATA_10BIT;


	ATE()->SleepMs(50);

	//bool bImg = true;
	//boost::timer t;
	CISImage()->DVPImageAcquireConfig(result, "test_demo", dataWidth, imageAverage, true, imageWidth, imageHeight, 5000, true);
	ATE()->SleepMs(50);
	CISImage()->DVPImageAcquireConfig(result, testName.c_str(), dataWidth, imageAverage, true, imageWidth, imageHeight, 5000, true);


	//EVTSYS(LEVEL_DEBUG, "The time of Capture NorTest %f\n", t.elapsed());
	
	PowerDownSequence();


	return;
}

void HSC02A::TempNoise(const std::string &testName, Test_Result &result)
{
	bool bRet = true;
	std::vector<unsigned int> vSites;
	ATE()->Site()->GetActive(vSites);
	unsigned short testNum = ATE()->Test(testName)->GetTestNumber();
	TempNoiseTestNumber = testNum;
	PowerUpSequence();

	ATE()->DCLevels()->Signal("PCLK")->Assign(DVP_PCLK);
	ATE()->DCLevels()->Signal("HREF")->Assign(DVP_DE);
	ATE()->DCLevels()->Signal("VCYN")->Assign(DVP_VS);
	ATE()->DCLevels()->Signal("D0")->Assign(DVP_D0);
	ATE()->DCLevels()->Signal("D1")->Assign(DVP_D1);
	ATE()->DCLevels()->Signal("D2")->Assign(DVP_D2);
	ATE()->DCLevels()->Signal("D3")->Assign(DVP_D3);
	ATE()->DCLevels()->Signal("D4")->Assign(DVP_D4);
	ATE()->DCLevels()->Signal("D5")->Assign(DVP_D5);
	ATE()->DCLevels()->Signal("D6")->Assign(DVP_D6);
	ATE()->DCLevels()->Signal("D7")->Assign(DVP_D7);
	ATE()->DCLevels()->Signal("D8")->Assign(DVP_D8);
	ATE()->DCLevels()->Signal("D9")->Assign(DVP_D9);

	//ATE()->DCLevels()->Block("ImageVterm")->Execute();
	if (IicSpeed)
	{
		bRet = HSC02A::WriteRegisterBlock("Dark_TempNoise");
	}
	else
	{
		bRet = HSC02A::WriteRegister("Dark_TempNoise");
	}

	ATE()->SleepMs(500);


	unsigned short imageWidth = 480, imageHeight = 270;
	unsigned char imageAverage = 8;
	EVT_IMG::EVT_IMG_DATAWIDTH dataWidth = EVT_IMG::DATA_10BIT;


	ATE()->SleepMs(50);

	//bool bImg = true;
	//boost::timer t;
	CISImage()->DVPImageAcquireConfig(result, "test_demo", dataWidth, 1, true, imageWidth, imageHeight, 5000, true);
	ATE()->SleepMs(50);
	CISImage()->DVPImageAcquireConfig(result, testName.c_str(), dataWidth, imageAverage, true, imageWidth, imageHeight, 10000, true);

	
	PowerDownSequence();


	return;
}

void HSC02A::test_demo(const std::string &testName, Test_Result &result)
{
	bool bRet = true;
	std::vector<unsigned int> vSites;
	ATE()->Site()->GetActive(vSites);
	unsigned short testNum = ATE()->Test(testName)->GetTestNumber();
	DarkDPCOffTestNumber = testNum;

	PowerUpSequence();

	ATE()->DCLevels()->Signal("PCLK")->Assign(DVP_PCLK);
	ATE()->DCLevels()->Signal("HREF")->Assign(DVP_DE);
	ATE()->DCLevels()->Signal("VCYN")->Assign(DVP_VS);
	ATE()->DCLevels()->Signal("D0")->Assign(DVP_D0);
	ATE()->DCLevels()->Signal("D1")->Assign(DVP_D1);
	ATE()->DCLevels()->Signal("D2")->Assign(DVP_D2);
	ATE()->DCLevels()->Signal("D3")->Assign(DVP_D3);
	ATE()->DCLevels()->Signal("D4")->Assign(DVP_D4);
	ATE()->DCLevels()->Signal("D5")->Assign(DVP_D5);
	ATE()->DCLevels()->Signal("D6")->Assign(DVP_D6);
	ATE()->DCLevels()->Signal("D7")->Assign(DVP_D7);
	ATE()->DCLevels()->Signal("D8")->Assign(DVP_D8);
	ATE()->DCLevels()->Signal("D9")->Assign(DVP_D9);

	//ATE()->DCLevels()->Block("ImageVterm")->Execute();
	if (IicSpeed)
	{
		bRet = HSC02A::WriteRegisterBlock("Dark_DPCOff");
	}
	else
	{
		bRet = HSC02A::WriteRegister("Dark_DPCOff");
	}

	ATE()->SleepMs(500);


	unsigned short imageWidth = 480, imageHeight = 270;
	unsigned char imageAverage = 2;
	EVT_IMG::EVT_IMG_DATAWIDTH dataWidth = EVT_IMG::DATA_10BIT;

	ATE()->SleepMs(50);

	//bool bImg = true;
	//boost::timer t;
	CISImage()->DVPImageAcquireConfig(result, testName.c_str(), dataWidth, imageAverage, true, imageWidth, imageHeight, 5000, true);

	//EVTSYS(LEVEL_DEBUG, "The time of Capture NorTest %f\n", t.elapsed());
	PowerDownSequence();


	return;
}

void HSC02A::Light(const std::string &testName, Test_Result &result)
{
	bool bRet = true;
	std::vector<unsigned int> vSites;
	
	ATE()->Site()->GetActive(vSites);
	unsigned short testNum = ATE()->Test(testName)->GetTestNumber();
	LightTestNumber = testNum;
	//SerialLight light("COM3");

	//PowerUpSequence();
	DisConnectAllPins();
	PowerUpSequenceLight();

	//在Light项中测试AVDD/DVDD/IOVDD/CVDD电压值
	vector<double> iAVDD;
    vector<double> iDVDD;
    vector<double> iIOVDD;
	
	vector<double> vAVDD;
    vector<double> vDVDD;
    vector<double> vIOVDD;
	
	

   

	ATE()->DCLevels()->Signal("PCLK")->Assign(DVP_PCLK);
	ATE()->DCLevels()->Signal("HREF")->Assign(DVP_DE);
	ATE()->DCLevels()->Signal("VCYN")->Assign(DVP_VS);
	ATE()->DCLevels()->Signal("D0")->Assign(DVP_D0);
	ATE()->DCLevels()->Signal("D1")->Assign(DVP_D1);
	ATE()->DCLevels()->Signal("D2")->Assign(DVP_D2);
	ATE()->DCLevels()->Signal("D3")->Assign(DVP_D3);
	ATE()->DCLevels()->Signal("D4")->Assign(DVP_D4);
	ATE()->DCLevels()->Signal("D5")->Assign(DVP_D5);
	ATE()->DCLevels()->Signal("D6")->Assign(DVP_D6);
	ATE()->DCLevels()->Signal("D7")->Assign(DVP_D7);
	ATE()->DCLevels()->Signal("D8")->Assign(DVP_D8);
	ATE()->DCLevels()->Signal("D9")->Assign(DVP_D9);

	ATE()->DCLevels()->Block("ImageVterm")->Execute();


	if (IicSpeed)
	{
		bRet = HSC02A::WriteRegisterBlock("Light");
	}
	else
	{
		bRet = HSC02A::WriteRegister("Light");
	}
	//unsigned long rdata[10] = { 0 };
	//EvtIICInterface* siteIIC = ATE()->Bus()->IIC("SCL", "SDA");
	//siteIIC->Config(IIC_SLAVE_ID, false, IicFreq / 100, IicAddrWidthBytes, IicDataWidthBytes);
	//siteIIC->Write(0x0087, 0x61, 1000);
	ATE()->SleepMs(500);
	//ATE()->DIB()->Signal("I2C_EN")->CBitsOff();
	//ATE()->PinPMU()->Signal("HVDD")->Disconnect();
	//ATE()->PinPMU()->Signal("NVDD")->Disconnect();
	
	//ATE()->SleepMs(2000);


	unsigned short imageWidth = 480, imageHeight = 270;
	unsigned char imageAverage = 2;
	EVT_IMG::EVT_IMG_DATAWIDTH dataWidth = EVT_IMG::DATA_10BIT;

	if (m_LedCalFlag)
	{
		EVT_EACH_SITE_BEGIN
			bRet = m_ledCal.CalStartDVP(result, activeSite, dataWidth, 1, imageWidth, imageHeight, testName, Light_Value, 3000, true,m_Gray);	//Remember modify ImageConfig.ini value,dataLaneSpeedType值从哪里确定,image取1还是imagaverage
		if (bRet)
		{
			result.SetPassFailBySite(activeSite, RESULT_PASS);
		}
		else
		{
			result.SetPassFailBySite(activeSite, RESULT_FAIL);
		}

		m_ledCal.CalFinish(activeSite);

		if (!bRet)
		{
			EVTSYS(LEVEL_ERROR, "Site:%d LowLight call CalStartMipiComponent exception", activeSite);
		}
		EVT_EACH_SITE_END



		PowerDownSequenceLight();
		DisConnectAllPins();

			return;
	}


	//打光
	for (auto &site : vSites)
	{
		if (LedSiteMap.count(site) == 0) continue;
        unsigned short luxValue = m_LuxMap[testName][site];
        //if (Led_test)
        //{
        //    EVTSYS(LEVEL_DEBUG, "Led Test: Site %d, Set Lux %d\r\n", site, led_current_lux);
        //    luxValue = led_current_lux;
        //}

		m_ledCtr.SetCctLux(LedSiteMap[site], luxValue);
        //ATE()->Datalog()->SetParametricResult(site, 2600, luxValue);
	}

	ATE()->DPS()->MV("AVDD", 100mS, 10mS, 1024, vAVDD);
    ATE()->DPS()->MV("DVDD", 100mS, 10mS, 1024, vDVDD);
    ATE()->DPS()->MV("IOVDD", 100mS, 10mS, 1024, vIOVDD);
    
	ATE()->Datalog()->SetParametricResult(vSites, "AVDD", 2424 , vAVDD);
    ATE()->Datalog()->SetParametricResult(vSites, "DVDD", 2425 , vDVDD);
    ATE()->Datalog()->SetParametricResult(vSites, "IOVDD", 2426 , vIOVDD);


	ATE()->SleepMs(200);



	ATE()->DPS()->MI("AVDD", 100mS, 0, 8192, iAVDD);
	ATE()->DPS()->MI("DVDD", 100mS, 0, 8192, iDVDD);
	ATE()->DPS()->MI("IOVDD", 100mS, 0, 8192, iIOVDD);

	//ATE()->HWInterface()->WriteRegister(vSites[0], 0x890000, 0, 0xBFC181C0);  //RC补偿

	{
		//boost::timer t;
		CISImage()->DVPImageAcquireConfig(
			result,
			testName.c_str(),
			dataWidth,
			imageAverage,
			true,
			imageWidth,
			imageHeight,
			5000,
			true);
		//EVTSYS(LEVEL_DEBUG, "The time of Capture LowLight %f\n", t.elapsed());
	}

	//ATE()->HWInterface()->WriteRegister(vSites[0], 0x890000, 0, 0xBFC1CA00);


	//在Light项测试时测试AVDD/DVDD/IOVDD的IDD值
	//ATE()->DPS()->FV("AVDD",2.8V,25.6mA,true,-25.6mA,25.6mA);
	//ATE()->DPS()->FV("IOVDD",1.8V,25.6mA,true,-25.6mA,25.6mA);
	//ATE()->DPS()->FV("DVDD",1.8V,25.6mA,true,-25.6mA,25.6mA);

	//ATE()->SleepMs(50);


	
	EVT_EACH_SITE_BEGIN
	//iAVDD[activeIndex] = iAVDD[activeIndex] + m_IDDOffsets[activeSite].AVDD;
	//iDVDD[activeIndex] = iDVDD[activeIndex] + m_IDDOffsets[activeSite].DVDD;
	//iIOVDD[activeIndex] = iIOVDD[activeIndex] + m_IDDOffsets[activeSite].IOVDD;

	double iSum;
	
	iSum = iAVDD[activeIndex] + iDVDD[activeIndex] + iIOVDD[activeIndex];
	
	ATE()->Datalog()->SetParametricResult(activeSite, "AVDD", 2427, iAVDD[activeIndex]);
	ATE()->Datalog()->SetParametricResult(activeSite, "DVDD", 2428, iDVDD[activeIndex]);
	ATE()->Datalog()->SetParametricResult(activeSite, "IOVDD", 2429, iIOVDD[activeIndex]);
	ATE()->Datalog()->SetParametricResult(activeSite,2600, iSum);

	EVT_EACH_SITE_END

	PowerDownSequenceLight();
	DisConnectAllPins();
		//light.set_brightness(site, 0);
	//m_ledCtr.SetCctLux(vSites, 0);
	for (auto &site : vSites)
	{
		if (LedSiteMap.count(site) == 0) continue;
		m_ledCtr.SetCctLux(LedSiteMap[site], 0);
	}
	return;
}

void HSC02A::GetTestResult(const std::string &testName, Test_Result &result)
{
	if (m_LedCalFlag)
	{
		SET_ALL_ACTIVE_SITE_RESULT(RESULT_PASS);
		return;
	}

	vector<unsigned int> vSites;
	ATE()->Site()->GetActive(vSites);

	auto NotifyFunc = [&testName](unsigned int site)
	{
		vector<unsigned int> vRsrc;
		ATE()->Site()->GetSiteLocation(site, vRsrc);

		EVEREST::UsrMsg_ImageInfo usrMsg;
		usrMsg.dmaNotify.msgHeader.eventID = MakeUsrEventID(UsrEvent_Image, IMAGE_READY);
		usrMsg.dmaNotify.msgData0.param.ulLow = vRsrc[0];
		usrMsg.dmaNotify.msgData0.param.ulHigh = site;
		usrMsg.dmaNotify.msgData1.param.ulLow = 0;       // bits of per pixel
		usrMsg.dmaNotify.msgData1.param.ulHigh = 0;
		strcpy_s(usrMsg.dmaNotify.msgData2.strData, sizeof(usrMsg.dmaNotify.msgData2.strData), testName.c_str());
		usrMsg.imageAverage = 0;
		usrMsg.width = 0;
		usrMsg.height = 0;
		usrMsg.frameCount = 1;

	

		ATE()->HWInterface()->NotifyUserEvent(&usrMsg, sizeof(UsrMsg_ImageInfo), 0x1 << site, true);
	};
	std::vector<std::thread> notifyThreads;
	for (auto &site : vSites)
	{
		notifyThreads.emplace_back(std::thread(NotifyFunc, site));
	}
	std::for_each(notifyThreads.begin(), notifyThreads.end(), std::mem_fn(&std::thread::join));


	//unsigned short testNum = ATE()->Test(testName)->GetTestNumber();

	SET_ALL_ACTIVE_SITE_RESULT(RESULT_PASS);

	EVT_EACH_SITE_BEGIN

		std::vector<AddrInfo> vAddr;

	if (m_imageTestResults[activeSite]) {
		delete[] m_imageTestResults[activeSite];
		m_imageTestResults[activeSite] = nullptr;
	}


	if (ATE()->Site()->WaitTestResult(10000) == 0)
	{
		

		ATE()->HWInterface()->GetDMABuffer(activeSite, vAddr);

		unsigned char *ResultBuf = vAddr[0].bufAddr;
		ResultBuf += ((g_siteIndex[activeSite] == 0) ? 0 : 0x2000000);

		bool finalTestReuslt = true;
		//std::string md5str = md5((char*)(ResultBuf), len);
		int len = *(int*)ResultBuf;
		int offset = sizeof(int);

		ofstream ofs("2.txt", ios::trunc);
		ofs << offset << "," << len;
		ofs.close();

		while (offset < len)
		{
			ImageResult Result;
			memcpy(&Result, ResultBuf + offset, sizeof(ImageResult));

			ofs.open("3.txt", ios::trunc);
			ofs << Result.testName;
			ofs.close();

			if (strcmp(Result.testName, "Dark_DPCOn") == 0)
			{

				//unsigned short testNum = DarkDPCOnTestNumber;
				//getImgResult_Dark ret_s;
				unsigned int testNum = ATE()->Test("Dark_DPCOn")->GetTestNumber();

				DarkImgResultWithDPCOn& ret_s = *(DarkImgResultWithDPCOn*)Result.pData;
	
				//todo: set datalog
				bool bRet = true;
				//mean

				bRet &= ATE()->Datalog()->SetParametricResult(activeSite, testNum + 1, ret_s.hDeadLineSize);
				if (!bRet)
				{
					if (bImagePassFlag[activeSite] == true)
					{
						ATE()->Bin()->Set(101, activeSite);
						ATE()->Bin()->Increment(101, activeSite);
					}
					bImagePassFlag[activeSite] = false;
				}

				bRet &= ATE()->Datalog()->SetParametricResult(activeSite, testNum + 2, ret_s.vDeadLineSize);
				if (!bRet)
				{
					if (bImagePassFlag[activeSite] == true)
					{
						ATE()->Bin()->Set(102, activeSite);
						ATE()->Bin()->Increment(102, activeSite);
					}
					bImagePassFlag[activeSite] = false;
				}
				bRet &= ATE()->Datalog()->SetParametricResult(activeSite, testNum + 0, ret_s.badPixelCount);
				if (!bRet)
				{
					if (bImagePassFlag[activeSite] == true)
					{
						ATE()->Bin()->Set(111, activeSite);
						ATE()->Bin()->Increment(111, activeSite);
					}
					bImagePassFlag[activeSite] = false;
				}

			


                

                bRet &= ATE()->Datalog()->SetParametricResult(activeSite, testNum + 3, ret_s.mean);
                bRet &= ATE()->Datalog()->SetParametricResult(activeSite, testNum + 4, ret_s.std);

            
				bRet &= ATE()->Datalog()->SetParametricResult(activeSite, testNum + 10, ret_s.clusterCount);
                bRet &= ATE()->Datalog()->SetParametricResult(activeSite, testNum + 11, ret_s.cluster_Cluster5);
                bRet &= ATE()->Datalog()->SetParametricResult(activeSite, testNum + 12, ret_s.cluster_4T1);
                bRet &= ATE()->Datalog()->SetParametricResult(activeSite, testNum + 13, ret_s.cluster_4T2);
                bRet &= ATE()->Datalog()->SetParametricResult(activeSite, testNum + 14, ret_s.cluster_4T3);
                bRet &= ATE()->Datalog()->SetParametricResult(activeSite, testNum + 15, ret_s.cluster_4T4);
                bRet &= ATE()->Datalog()->SetParametricResult(activeSite, testNum + 16, ret_s.cluster_3X1);
                bRet &= ATE()->Datalog()->SetParametricResult(activeSite, testNum + 17, ret_s.cluster_1X3);
                bRet &= ATE()->Datalog()->SetParametricResult(activeSite, testNum + 18, ret_s.cluster_2X1);
                bRet &= ATE()->Datalog()->SetParametricResult(activeSite, testNum + 19, ret_s.cluster_1X2);
                bRet &= ATE()->Datalog()->SetParametricResult(activeSite, testNum + 20, ret_s.cluster_2X1X1);
                bRet &= ATE()->Datalog()->SetParametricResult(activeSite, testNum + 21, ret_s.cluster_1X2X1);
                bRet &= ATE()->Datalog()->SetParametricResult(activeSite, testNum + 22, ret_s.cluster_2X2);
                bRet &= ATE()->Datalog()->SetParametricResult(activeSite, testNum + 23, ret_s.cluster_2D1);
                bRet &= ATE()->Datalog()->SetParametricResult(activeSite, testNum + 24, ret_s.cluster_2D2);
                bRet &= ATE()->Datalog()->SetParametricResult(activeSite, testNum + 25, ret_s.cluster_2D3);
                bRet &= ATE()->Datalog()->SetParametricResult(activeSite, testNum + 26, ret_s.cluster_2D4);
                bRet &= ATE()->Datalog()->SetParametricResult(activeSite, testNum + 27, ret_s.cluster_1X1);
                bRet &= ATE()->Datalog()->SetParametricResult(activeSite, testNum + 28, ret_s.cluster_Big);

				finalTestReuslt &= bRet;

			}

			if (strcmp(Result.testName, "Dark_DPCOff") == 0)
			{

				//unsigned short testNum = DarkDPCOffTestNumber;
				unsigned int testNum = ATE()->Test("Dark_DPCOff")->GetTestNumber();
				DarkImgResultWithDPCOff& ret_s = *(DarkImgResultWithDPCOff*)Result.pData;

				//todo: set datalog
				bool bRet = true;
				//mean
				bRet &= ATE()->Datalog()->SetParametricResult(activeSite, testNum + 0, ret_s.badPixelCount1);
				bRet &= ATE()->Datalog()->SetParametricResult(activeSite, testNum + 1, ret_s.badPixelCount2);
				bRet &= ATE()->Datalog()->SetParametricResult(activeSite, testNum + 2, ret_s.mean);
				bRet &= ATE()->Datalog()->SetParametricResult(activeSite, testNum + 3, ret_s.std);


				finalTestReuslt &= bRet;

			}

			if (strcmp(Result.testName, "DarkFPN") == 0)
			{

				//unsigned short testNum = DarkFPNTestNumber;
				unsigned int testNum = ATE()->Test("DarkFPN")->GetTestNumber();
                DarkFPNResult& ret_s = *(DarkFPNResult*)Result.pData;

                bool bRet = true;
                bRet &= ATE()->Datalog()->SetParametricResult(activeSite, testNum + 0, ret_s.AFPN);
				bRet &= ATE()->Datalog()->SetParametricResult(activeSite, testNum + 1, ret_s.HFPN);
				bRet &= ATE()->Datalog()->SetParametricResult(activeSite, testNum + 2, ret_s.VFPN);
				finalTestReuslt &= bRet;

			}

			if (strcmp(Result.testName, "DarkCurrent") == 0)
			{
				unsigned int testNum = ATE()->Test("DarkCurrent")->GetTestNumber();
				DarkCurrentResult& ret_s = *(DarkCurrentResult*)Result.pData;
				bool bRet = true;
				//mean
				bRet &= ATE()->Datalog()->SetParametricResult(activeSite, testNum + 0, ret_s.darkcurrent);
				finalTestReuslt &= bRet;

			}

			if (strcmp(Result.testName, "TempNoise") == 0)
			{

				//unsigned short testNum = DarkDPCOffTestNumber;
				unsigned int testNum = ATE()->Test("TempNoise")->GetTestNumber();
				TempNoiseResult& ret_s = *(TempNoiseResult*)Result.pData;
				bool bRet = true;
				//mean
				bRet &= ATE()->Datalog()->SetParametricResult(activeSite, testNum + 0, ret_s.TempNoise);
				finalTestReuslt &= bRet;

			}

			if (strcmp(Result.testName, "Light") == 0)
			{

				unsigned int testNum = ATE()->Test("Light")->GetTestNumber();
				//unsigned short testNum = LightTestNumber;
				LightImgResult& ret_s = *(LightImgResult*)Result.pData;

				//todo: set datalog
				bool bRet = true;
				bool RRet = true;
				bool GrRet = true;
				bool GbRet = true;
				bool BRet = true;


				bool R_BRet = true;
				bool Gr_GbRet = true;
				bool R_GrRet = true;
				bool B_GbRet = true;
				//mean
				RRet &= ATE()->Datalog()->SetParametricResult(activeSite, testNum + 0, ret_s.hDeadLineCount_R);
				GrRet &= ATE()->Datalog()->SetParametricResult(activeSite, testNum + 1, ret_s.hDeadLineCount_Gr);
				GbRet &= ATE()->Datalog()->SetParametricResult(activeSite, testNum + 2, ret_s.hDeadLineCount_Gb);
				BRet &= ATE()->Datalog()->SetParametricResult(activeSite, testNum + 3, ret_s.hDeadLineCount_B);
				if ((RRet&&GrRet&&GbRet&&BRet) == false)
				{
					if (bImagePassFlag[activeSite] == true)
					{
						ATE()->Bin()->Set(301, activeSite);
						ATE()->Bin()->Increment(301, activeSite);
					}
					bImagePassFlag[activeSite] = false;
				}

				RRet &= ATE()->Datalog()->SetParametricResult(activeSite, testNum + 4, ret_s.vDeadLineCount_R);
				GrRet &= ATE()->Datalog()->SetParametricResult(activeSite, testNum + 5, ret_s.vDeadLineCount_Gr);
				GbRet &= ATE()->Datalog()->SetParametricResult(activeSite, testNum + 6, ret_s.vDeadLineCount_Gb);
				BRet &= ATE()->Datalog()->SetParametricResult(activeSite, testNum + 7, ret_s.vDeadLineCount_B);
				if ((RRet&&GrRet&&GbRet&&BRet) == false)
				{
					if (bImagePassFlag[activeSite] == true)
					{
						ATE()->Bin()->Set(302, activeSite);
						ATE()->Bin()->Increment(302, activeSite);
					}
					bImagePassFlag[activeSite] = false;
				}
				RRet &= ATE()->Datalog()->SetParametricResult(activeSite, testNum + 10, ret_s.blackPixelCount_R);
				GrRet &= ATE()->Datalog()->SetParametricResult(activeSite, testNum + 11, ret_s.blackPixelCount_Gr);
				GbRet &= ATE()->Datalog()->SetParametricResult(activeSite, testNum + 12, ret_s.blackPixelCount_Gb);
				BRet &= ATE()->Datalog()->SetParametricResult(activeSite, testNum + 13, ret_s.blackPixelCount_B);
				if ((RRet&&GrRet&&GbRet&&BRet) == false)
				{
					if (bImagePassFlag[activeSite] == true)
					{
						ATE()->Bin()->Set(311, activeSite);
						ATE()->Bin()->Increment(311, activeSite);
					}
					bImagePassFlag[activeSite] = false;
				}
				RRet &= ATE()->Datalog()->SetParametricResult(activeSite, testNum + 14, ret_s.whitePixelCount_R);
				GrRet &= ATE()->Datalog()->SetParametricResult(activeSite, testNum + 15, ret_s.whitePixelCount_Gr);
				GbRet &= ATE()->Datalog()->SetParametricResult(activeSite, testNum + 16, ret_s.whitePixelCount_Gb);
				BRet &= ATE()->Datalog()->SetParametricResult(activeSite, testNum + 17, ret_s.whitePixelCount_B);
				if ((RRet&&GrRet&&GbRet&&BRet) == false)
				{
					if (bImagePassFlag[activeSite] == true)
					{
						ATE()->Bin()->Set(321, activeSite);
						ATE()->Bin()->Increment(321, activeSite);
					}
					bImagePassFlag[activeSite] = false;
				}

				bRet &= ATE()->Datalog()->SetParametricResult(activeSite, testNum + 30, ret_s.mean);
				RRet &= ATE()->Datalog()->SetParametricResult(activeSite, testNum + 31, ret_s.mean_R);
				GrRet &= ATE()->Datalog()->SetParametricResult(activeSite, testNum + 32, ret_s.mean_Gr);
				GbRet &= ATE()->Datalog()->SetParametricResult(activeSite, testNum + 33, ret_s.mean_Gb);
				BRet &= ATE()->Datalog()->SetParametricResult(activeSite, testNum + 34, ret_s.mean_B);
				if ((bRet&&RRet&&GrRet&&GbRet&&BRet) == false)
				{
					if (bImagePassFlag[activeSite] == true)
					{
						ATE()->Bin()->Set(351, activeSite);
						ATE()->Bin()->Increment(351, activeSite);
					}
					bImagePassFlag[activeSite] = false;
				}

				RRet &= ATE()->Datalog()->SetParametricResult(activeSite, testNum + 40, ret_s.std_R);
				GrRet &= ATE()->Datalog()->SetParametricResult(activeSite, testNum + 41, ret_s.std_Gr);
				GbRet &= ATE()->Datalog()->SetParametricResult(activeSite, testNum + 42, ret_s.std_Gb);
				BRet &= ATE()->Datalog()->SetParametricResult(activeSite, testNum + 43, ret_s.std_B);
				if ((bRet&&RRet&&GrRet&&GbRet&&BRet) == false)
				{
					if (bImagePassFlag[activeSite] == true)
					{
						ATE()->Bin()->Set(352, activeSite);
						ATE()->Bin()->Increment(352, activeSite);
					}
					bImagePassFlag[activeSite] = false;
				}

				R_BRet &= ATE()->Datalog()->SetParametricResult(activeSite, testNum + 20, ret_s.radio_R_B);
				Gr_GbRet &= ATE()->Datalog()->SetParametricResult(activeSite, testNum + 21, ret_s.radio_Gr_Gb);
				R_GrRet &= ATE()->Datalog()->SetParametricResult(activeSite, testNum + 22, ret_s.radio_R_Gr);
				B_GbRet &= ATE()->Datalog()->SetParametricResult(activeSite, testNum + 23, ret_s.radio_B_Gb);
				if ((R_BRet&&Gr_GbRet&&R_GrRet&&B_GbRet) == false)
				{
					if (bImagePassFlag[activeSite] == true)
					{
						ATE()->Bin()->Set(361, activeSite);
						ATE()->Bin()->Increment(361, activeSite);
					}
					bImagePassFlag[activeSite] = false;
				}


				bRet &= ATE()->Datalog()->SetParametricResult(activeSite, testNum + 50, ret_s.shading_ROI1);
				bRet &= ATE()->Datalog()->SetParametricResult(activeSite, testNum + 51, ret_s.shading_ROI2);
				bRet &= ATE()->Datalog()->SetParametricResult(activeSite, testNum + 52, ret_s.shading_ROI3);
				bRet &= ATE()->Datalog()->SetParametricResult(activeSite, testNum + 53, ret_s.shading_ROI4);
				bRet &= ATE()->Datalog()->SetParametricResult(activeSite, testNum + 54, ret_s.shading_ROI6);
				bRet &= ATE()->Datalog()->SetParametricResult(activeSite, testNum + 55, ret_s.shading_ROI7);
				bRet &= ATE()->Datalog()->SetParametricResult(activeSite, testNum + 56, ret_s.shading_ROI8);
				bRet &= ATE()->Datalog()->SetParametricResult(activeSite, testNum + 57, ret_s.shading_ROI9);

				bRet &= ATE()->Datalog()->SetParametricResult(activeSite, testNum + 60, ret_s.balance_RB_ROI1);
				bRet &= ATE()->Datalog()->SetParametricResult(activeSite, testNum + 61, ret_s.balance_RB_ROI2);
				bRet &= ATE()->Datalog()->SetParametricResult(activeSite, testNum + 62, ret_s.balance_RB_ROI3);
				bRet &= ATE()->Datalog()->SetParametricResult(activeSite, testNum + 63, ret_s.balance_RB_ROI4);
				bRet &= ATE()->Datalog()->SetParametricResult(activeSite, testNum + 64, ret_s.balance_RB_ROI6);
				bRet &= ATE()->Datalog()->SetParametricResult(activeSite, testNum + 65, ret_s.balance_RB_ROI7);
				bRet &= ATE()->Datalog()->SetParametricResult(activeSite, testNum + 66, ret_s.balance_RB_ROI8);
				bRet &= ATE()->Datalog()->SetParametricResult(activeSite, testNum + 67, ret_s.balance_RB_ROI9);
				bRet &= ATE()->Datalog()->SetParametricResult(activeSite, testNum + 68, ret_s.balances_RB_Max);



				bRet &= ATE()->Datalog()->SetParametricResult(activeSite, testNum + 70, ret_s.balance_GrGb_ROI1);
				bRet &= ATE()->Datalog()->SetParametricResult(activeSite, testNum + 71, ret_s.balance_GrGb_ROI2);
				bRet &= ATE()->Datalog()->SetParametricResult(activeSite, testNum + 72, ret_s.balance_GrGb_ROI3);
				bRet &= ATE()->Datalog()->SetParametricResult(activeSite, testNum + 73, ret_s.balance_GrGb_ROI4);
				bRet &= ATE()->Datalog()->SetParametricResult(activeSite, testNum + 74, ret_s.balance_GrGb_ROI6);
				bRet &= ATE()->Datalog()->SetParametricResult(activeSite, testNum + 75, ret_s.balance_GrGb_ROI7);
				bRet &= ATE()->Datalog()->SetParametricResult(activeSite, testNum + 76, ret_s.balance_GrGb_ROI8);
				bRet &= ATE()->Datalog()->SetParametricResult(activeSite, testNum + 77, ret_s.balance_GrGb_ROI9);
				bRet &= ATE()->Datalog()->SetParametricResult(activeSite, testNum + 78, ret_s.balances_GrGb_Max);

				bRet = true;

				bool RG_Ratio = true;
				bool BG_Ratio = true;
				RG_Ratio &= ATE()->Datalog()->SetParametricResult(activeSite, testNum + 80, ret_s.RG_ROI19);
				RG_Ratio &= ATE()->Datalog()->SetParametricResult(activeSite, testNum + 81, ret_s.RG_ROI37);
				RG_Ratio &= ATE()->Datalog()->SetParametricResult(activeSite, testNum + 82, ret_s.RG_ROI28);
				RG_Ratio &= ATE()->Datalog()->SetParametricResult(activeSite, testNum + 83, ret_s.RG_ROI46);

				BG_Ratio &= ATE()->Datalog()->SetParametricResult(activeSite, testNum + 90, ret_s.BG_ROI19);
				BG_Ratio &= ATE()->Datalog()->SetParametricResult(activeSite, testNum + 91, ret_s.BG_ROI37);
				BG_Ratio &= ATE()->Datalog()->SetParametricResult(activeSite, testNum + 92, ret_s.BG_ROI28);
				BG_Ratio &= ATE()->Datalog()->SetParametricResult(activeSite, testNum + 93, ret_s.BG_ROI46);
				if ((RG_Ratio&&BG_Ratio) == false)
				{
					if (bImagePassFlag[activeSite] == true)
					{
						ATE()->Bin()->Set(383, activeSite);
						ATE()->Bin()->Increment(383, activeSite);
					}
					bImagePassFlag[activeSite] = false;
				}
				bool R_B_Ratio = true;
				bool Gr_Gb_Ratio = true;
				bool R_Gr_Ratio = true;
				bool B_Gb_Ratio = true;
				R_B_Ratio &= ATE()->Datalog()->SetParametricResult(activeSite, testNum + 100, ret_s.ratio_ROI5_R_B);
				Gr_Gb_Ratio &= ATE()->Datalog()->SetParametricResult(activeSite, testNum + 101, ret_s.ratio_ROI5_Gr_Gb);
				R_Gr_Ratio &= ATE()->Datalog()->SetParametricResult(activeSite, testNum + 102, ret_s.ratio_ROI5_R_Gr);
				B_Gb_Ratio &= ATE()->Datalog()->SetParametricResult(activeSite, testNum + 103, ret_s.ratio_ROI5_B_Gb);
				if ((RG_Ratio&&BG_Ratio) == false)
				{
					if (bImagePassFlag[activeSite] == true)
					{
						ATE()->Bin()->Set(362, activeSite);
						ATE()->Bin()->Increment(362, activeSite);
					}
					bImagePassFlag[activeSite] = false;
				}

				bRet &= ATE()->Datalog()->SetParametricResult(activeSite, testNum + 110, ret_s.blemish);
				if (!bRet)
				{
					if (bImagePassFlag[activeSite] == true)
					{
						ATE()->Bin()->Set(380, activeSite);
						ATE()->Bin()->Increment(380, activeSite);
					}
					bImagePassFlag[activeSite] = false;
				}

				bRet &= ATE()->Datalog()->SetParametricResult(activeSite, testNum + 120, ret_s.scratch);
				if (!bRet)
				{
					if (bImagePassFlag[activeSite] == true)
					{
						ATE()->Bin()->Set(390, activeSite);
						ATE()->Bin()->Increment(390, activeSite);
					}
					bImagePassFlag[activeSite] = false;
				}

				bRet &= ATE()->Datalog()->SetParametricResult(activeSite, testNum + 130, ret_s.clusterCount);
				bRet &= ATE()->Datalog()->SetParametricResult(activeSite, testNum + 131, ret_s.cluster_Cluster5);
				bRet &= ATE()->Datalog()->SetParametricResult(activeSite, testNum + 132, ret_s.cluster_4T1);
				bRet &= ATE()->Datalog()->SetParametricResult(activeSite, testNum + 133, ret_s.cluster_4T2);
				bRet &= ATE()->Datalog()->SetParametricResult(activeSite, testNum + 134, ret_s.cluster_4T3);
				bRet &= ATE()->Datalog()->SetParametricResult(activeSite, testNum + 135, ret_s.cluster_4T4);
				bRet &= ATE()->Datalog()->SetParametricResult(activeSite, testNum + 136, ret_s.cluster_3X1);
				bRet &= ATE()->Datalog()->SetParametricResult(activeSite, testNum + 137, ret_s.cluster_1X3);
				bRet &= ATE()->Datalog()->SetParametricResult(activeSite, testNum + 138, ret_s.cluster_2X1);
				bRet &= ATE()->Datalog()->SetParametricResult(activeSite, testNum + 139, ret_s.cluster_1X2);
				bRet &= ATE()->Datalog()->SetParametricResult(activeSite, testNum + 140, ret_s.cluster_2X1X1);
				bRet &= ATE()->Datalog()->SetParametricResult(activeSite, testNum + 141, ret_s.cluster_1X2X1);
				bRet &= ATE()->Datalog()->SetParametricResult(activeSite, testNum + 142, ret_s.cluster_2X2);
				bRet &= ATE()->Datalog()->SetParametricResult(activeSite, testNum + 143, ret_s.cluster_2D1);
				bRet &= ATE()->Datalog()->SetParametricResult(activeSite, testNum + 144, ret_s.cluster_2D2);
				bRet &= ATE()->Datalog()->SetParametricResult(activeSite, testNum + 145, ret_s.cluster_2D3);
				bRet &= ATE()->Datalog()->SetParametricResult(activeSite, testNum + 146, ret_s.cluster_2D4);
				bRet &= ATE()->Datalog()->SetParametricResult(activeSite, testNum + 147, ret_s.cluster_1X1);
				bRet &= ATE()->Datalog()->SetParametricResult(activeSite, testNum + 148, ret_s.cluster_Big);

				bRet = true;

				finalTestReuslt &= bRet;
				finalTestReuslt &= RRet ;
				finalTestReuslt &= GrRet;
				finalTestReuslt &= GbRet;
				finalTestReuslt &= BRet ;

				finalTestReuslt &= R_BRet ;
				finalTestReuslt &= Gr_GbRet;
				finalTestReuslt &= R_GrRet ;
				finalTestReuslt &= B_GbRet ;

				finalTestReuslt &= RG_Ratio;
				finalTestReuslt &= BG_Ratio;

				finalTestReuslt &= R_B_Ratio ;
				finalTestReuslt &= Gr_Gb_Ratio;
				finalTestReuslt &= R_Gr_Ratio ;
				finalTestReuslt &= B_Gb_Ratio ;

			}

			if (strcmp(Result.testName, "PRNU") == 0)
			{
				//unsigned short testNum = PRNUTestNumber;
				unsigned int testNum = ATE()->Test("PRNU")->GetTestNumber();
                PRNUResult& ret_s = *(PRNUResult*)Result.pData;

				//todo: set datalog
				bool bRet = true;

				bRet &= ATE()->Datalog()->SetParametricResult(activeSite, testNum + 0, ret_s.PRNU_R);
				bRet &= ATE()->Datalog()->SetParametricResult(activeSite, testNum + 1, ret_s.PRNU_Gr);
				bRet &= ATE()->Datalog()->SetParametricResult(activeSite, testNum + 2, ret_s.PRNU_Gb);
				bRet &= ATE()->Datalog()->SetParametricResult(activeSite, testNum + 3, ret_s.PRNU_B);


				//add in 2023/06/30         Sensitivity
				bRet &= ATE()->Datalog()->SetParametricResult(activeSite, testNum + 4, ret_s.Diff_R / (m_Diff_LuxMap["PRNU"][activeSite] * 1 / 30));
				bRet &= ATE()->Datalog()->SetParametricResult(activeSite, testNum + 5, ret_s.Diff_Gr / (m_Diff_LuxMap["PRNU"][activeSite] * 1 / 30));
				bRet &= ATE()->Datalog()->SetParametricResult(activeSite, testNum + 6, ret_s.Diff_Gb / (m_Diff_LuxMap["PRNU"][activeSite] * 1 / 30));
				bRet &= ATE()->Datalog()->SetParametricResult(activeSite, testNum + 7, ret_s.Diff_B / (m_Diff_LuxMap["PRNU"][activeSite] * 1 / 30));

				finalTestReuslt &= bRet;

			}

			

			if (strcmp(Result.testName, "CG") == 0)
			{
				
				unsigned int testNum = ATE()->Test("CG")->GetTestNumber();
				CGResult& ret_s = *(CGResult*)Result.pData;

				//todo: set datalog
				bool bRet = true;

				bRet &= ATE()->Datalog()->SetParametricResult(activeSite, testNum + 0, ret_s.CG_B);
				bRet &= ATE()->Datalog()->SetParametricResult(activeSite, testNum + 1, ret_s.CG_Gb);
				bRet &= ATE()->Datalog()->SetParametricResult(activeSite, testNum + 2, ret_s.CG_Gr);
				bRet &= ATE()->Datalog()->SetParametricResult(activeSite, testNum + 3, ret_s.CG_R);
				bRet &= ATE()->Datalog()->SetParametricResult(activeSite, testNum + 4, ret_s.CG);
				finalTestReuslt &= bRet;

				

			}


			if (strcmp(Result.testName, "FWC") == 0)
			{
				//unsigned short testNum = FWCTestNumber;
				unsigned int testNum = ATE()->Test("FWC")->GetTestNumber();
				FWCResult& ret_s = *(FWCResult*)Result.pData;

				//todo: set datalog
				bool bRet = true;

				bRet &= ATE()->Datalog()->SetParametricResult(activeSite, testNum + 0, ret_s.R_mean - 2.5 * ret_s.R_std);
				bRet &= ATE()->Datalog()->SetParametricResult(activeSite, testNum + 1, ret_s.Gr_mean - 2.5 * ret_s.Gr_std);
				bRet &= ATE()->Datalog()->SetParametricResult(activeSite, testNum + 2, ret_s.Gb_mean - 2.5 * ret_s.Gb_std);
				bRet &= ATE()->Datalog()->SetParametricResult(activeSite, testNum + 3, ret_s.B_mean - 2.5 * ret_s.B_std);

				finalTestReuslt &= bRet;

			}
			offset += sizeof(ImageResult);


		}

		unsigned int currentBinNum = ATE()->Bin()->GetCurrentSoftBinNum(activeSite);
		if ((currentBinNum == 65535) && (finalTestReuslt == true))
		{
			bAllPssFlag[activeSite] = true;
		}

	}

	else
	{
		EVTSYS(LEVEL_ERROR, "Site %d GetTestResult the WaitTestResult is timeout\r\n", activeSite);
		ATE()->Bin()->Set(65535, activeSite);
		ATE()->Bin()->Increment(65535, activeSite);
		SET_ALL_ACTIVE_SITE_RESULT(RESULT_FAIL);
	}
	EVT_EACH_SITE_END

		return;
}

void HSC02A::Temp(const std::string &testName, Test_Result &result)
{
	
	bool bRet = true;
	std::vector<unsigned int> vSites;
	ATE()->Site()->GetActive(vSites);
	unsigned short testNum = ATE()->Test(testName)->GetTestNumber();
	DisConnectAllPins();

	std::vector<double> IOVDD;

	ATE()->DPS()->FV("IOVDD", 4V, 25.6mA, true, -38.4mA, 38.4mA);

	ATE()->SleepMs(100);

	ATE()->DPS()->MI("IOVDD", 10mS, 0, 1000, IOVDD);

	ATE()->DPS()->FV("IOVDD", 0V, 25.6mA, true, -38.4mA, 38.4mA);
	DisConnectAllPins();

	ATE()->Datalog()->SetParametricResult(vSites, "IOVDD", testNum+2, IOVDD);
	return;


}

void HSC02A::FWC(const std::string &testName, Test_Result &result)
{
	
	bool bRet = true;
	std::vector<unsigned int> vSites;
	ATE()->Site()->GetActive(vSites);
	unsigned short testNum = ATE()->Test(testName)->GetTestNumber();
	FWCTestNumber = testNum;
	m_ledCal.ReadLedCalFile();
	
	PowerUpSequence();

	ATE()->DCLevels()->Signal("PCLK")->Assign(DVP_PCLK);
	ATE()->DCLevels()->Signal("HREF")->Assign(DVP_DE);
	ATE()->DCLevels()->Signal("VCYN")->Assign(DVP_VS);
	ATE()->DCLevels()->Signal("D0")->Assign(DVP_D0);
	ATE()->DCLevels()->Signal("D1")->Assign(DVP_D1);
	ATE()->DCLevels()->Signal("D2")->Assign(DVP_D2);
	ATE()->DCLevels()->Signal("D3")->Assign(DVP_D3);
	ATE()->DCLevels()->Signal("D4")->Assign(DVP_D4);
	ATE()->DCLevels()->Signal("D5")->Assign(DVP_D5);
	ATE()->DCLevels()->Signal("D6")->Assign(DVP_D6);
	ATE()->DCLevels()->Signal("D7")->Assign(DVP_D7);
	ATE()->DCLevels()->Signal("D8")->Assign(DVP_D8);
	ATE()->DCLevels()->Signal("D9")->Assign(DVP_D9);

	ATE()->DCLevels()->Block("ImageVterm")->Execute();

	if (IicSpeed)
	{
		bRet = HSC02A::WriteRegisterBlock("FWC");
	}
	else
	{
		bRet = HSC02A::WriteRegister("FWC");
	}

	ATE()->SleepMs(500);
	unsigned short imageWidth = 480, imageHeight = 270;
	unsigned char imageAverage = 2;
	EVT_IMG::EVT_IMG_DATAWIDTH dataWidth = EVT_IMG::DATA_10BIT;
	if (m_FWC_flag) 
	{
		EVT_EACH_SITE_BEGIN
			bRet = m_ledCal.StartFWC(result, activeSite, dataWidth, 1, imageWidth, imageHeight, testName, Light_Value, 5000, true, m_Gray);	//Remember modify ImageConfig.ini value,dataLaneSpeedType值从哪里确定,image取1还是imagaverage
		if (bRet)
		{
			result.SetPassFailBySite(activeSite, RESULT_PASS);
		}
		else
		{
			result.SetPassFailBySite(activeSite, RESULT_FAIL);
			EVTSYS(LEVEL_ERROR, "Site:%d FWC call FWCComponent exception", activeSite);
		}
		EVT_EACH_SITE_END

			return;
	}
	
	for (auto &site : vSites)
	{
		if (LedSiteMap.count(site) == 0) continue;
		unsigned short luxValue = m_LuxMap[testName][site];
		m_ledCtr.SetCctLux(LedSiteMap[site], luxValue);
	}


	ATE()->SleepMs(50);
	{
		CISImage()->DVPImageAcquireConfig(
			result,
			testName.c_str(),
			dataWidth,
			imageAverage,
			true,
			imageWidth,
			imageHeight,
			5000,
			true);

	}

	PowerDownSequence();
	for (auto &site : vSites)
	{

		if (LedSiteMap.count(site) == 0) continue;
		m_ledCtr.SetCctLux(LedSiteMap[site], 0);
	}
	return;
}

void HSC02A::DarkFPN(const std::string &testName, Test_Result &result)
{
	bool bRet = true;
	std::vector<unsigned int> vSites;
	ATE()->Site()->GetActive(vSites);
	unsigned short testNum = ATE()->Test(testName)->GetTestNumber();
	DarkFPNTestNumber = testNum;

	PowerUpSequence();


	ATE()->DCLevels()->Signal("PCLK")->Assign(DVP_PCLK);
	ATE()->DCLevels()->Signal("HREF")->Assign(DVP_DE);
	ATE()->DCLevels()->Signal("VCYN")->Assign(DVP_VS);
	ATE()->DCLevels()->Signal("D0")->Assign(DVP_D0);
	ATE()->DCLevels()->Signal("D1")->Assign(DVP_D1);
	ATE()->DCLevels()->Signal("D2")->Assign(DVP_D2);
	ATE()->DCLevels()->Signal("D3")->Assign(DVP_D3);
	ATE()->DCLevels()->Signal("D4")->Assign(DVP_D4);
	ATE()->DCLevels()->Signal("D5")->Assign(DVP_D5);
	ATE()->DCLevels()->Signal("D6")->Assign(DVP_D6);
	ATE()->DCLevels()->Signal("D7")->Assign(DVP_D7);
	ATE()->DCLevels()->Signal("D8")->Assign(DVP_D8);
	ATE()->DCLevels()->Signal("D9")->Assign(DVP_D9);

	ATE()->DCLevels()->Block("ImageVterm")->Execute();
	if (IicSpeed)
	{
		bRet = HSC02A::WriteRegisterBlock("DarkFPN");
	}
	else
	{
		bRet = HSC02A::WriteRegister("DarkFPN");
	}

	ATE()->SleepMs(500);


	unsigned short imageWidth = 480, imageHeight = 270;
	unsigned char imageAverage = 100;  
	EVT_IMG::EVT_IMG_DATAWIDTH dataWidth = EVT_IMG::DATA_10BIT;

	ATE()->SleepMs(500);  

	//CISImage()->DVPImageAcquireConfig(result, testName.c_str(), dataWidth, imageAverage, true, imageWidth, imageHeight, 7000, true);     //采集10张图
	CISImage()->DVPImageAcquireConfig(result, testName.c_str(), dataWidth, imageAverage, true, imageWidth, imageHeight, 650000, true);  //采集100张图

	PowerDownSequence();


	return;


}

void HSC02A::PRNU(const std::string &testName, Test_Result &result)
{

    bool bRet = true;
    std::vector<unsigned int> vSites;
    ATE()->Site()->GetActive(vSites);
    unsigned short testNum = ATE()->Test(testName)->GetTestNumber();
    PRNUTestNumber = testNum;

    PowerUpSequence();


    ATE()->DCLevels()->Signal("PCLK")->Assign(DVP_PCLK);
    ATE()->DCLevels()->Signal("HREF")->Assign(DVP_DE);
    ATE()->DCLevels()->Signal("VCYN")->Assign(DVP_VS);
    ATE()->DCLevels()->Signal("D0")->Assign(DVP_D0);
    ATE()->DCLevels()->Signal("D1")->Assign(DVP_D1);
    ATE()->DCLevels()->Signal("D2")->Assign(DVP_D2);
    ATE()->DCLevels()->Signal("D3")->Assign(DVP_D3);
    ATE()->DCLevels()->Signal("D4")->Assign(DVP_D4);
    ATE()->DCLevels()->Signal("D5")->Assign(DVP_D5);
    ATE()->DCLevels()->Signal("D6")->Assign(DVP_D6);
    ATE()->DCLevels()->Signal("D7")->Assign(DVP_D7);
    ATE()->DCLevels()->Signal("D8")->Assign(DVP_D8);
    ATE()->DCLevels()->Signal("D9")->Assign(DVP_D9);

	ATE()->DCLevels()->Block("ImageVterm")->Execute();
    if (IicSpeed)
    {
        bRet = HSC02A::WriteRegisterBlock("PRNU");
    }
    else
    {
        bRet = HSC02A::WriteRegister("PRNU");
    }

    ATE()->SleepMs(500);


    unsigned short imageWidth = 480, imageHeight = 270;
    unsigned char imageAverage = 2;
    EVT_IMG::EVT_IMG_DATAWIDTH dataWidth = EVT_IMG::DATA_10BIT;

    ATE()->SleepMs(50);

    // 需要对PRNU40或者PRNU80分别打灯
    // 采集图像,并且调用PRNU

	if (m_LedCalFlag)
	{
		EVT_EACH_SITE_BEGIN
			bRet = m_ledCal.CalStartDVP(result, activeSite, dataWidth, 1, imageWidth, imageHeight, "PRNU40", PRNU40_Value, 3000, true, m_Gray);	//Remember modify ImageConfig.ini value,dataLaneSpeedType值从哪里确定,image取1还是imagaverage
		if (bRet)
		{
			result.SetPassFailBySite(activeSite, RESULT_PASS);
		}
		else
		{
			result.SetPassFailBySite(activeSite, RESULT_FAIL);
		}

		m_ledCal.CalFinish(activeSite);

		if (!bRet)
		{
			EVTSYS(LEVEL_ERROR, "Site:%d LowLight call CalStartMipiComponent exception", activeSite);
		}
		EVT_EACH_SITE_END

			//return;
	}

	if (m_LedCalFlag)
	{
		EVT_EACH_SITE_BEGIN
			bRet = m_ledCal.CalStartDVP(result, activeSite, dataWidth, 1, imageWidth, imageHeight, "PRNU80", PRNU80_Value, 3000, true, m_Gray);	//Remember modify ImageConfig.ini value,dataLaneSpeedType值从哪里确定,image取1还是imagaverage
		if (bRet)
		{
			result.SetPassFailBySite(activeSite, RESULT_PASS);
		}
		else
		{
			result.SetPassFailBySite(activeSite, RESULT_FAIL);
		}

		m_ledCal.CalFinish(activeSite);

		if (!bRet)
		{
			EVTSYS(LEVEL_ERROR, "Site:%d LowLight call CalStartMipiComponent exception", activeSite);
		}
		EVT_EACH_SITE_END

			return;
	}


	for (auto &site : vSites)
	{

		if (LedSiteMap.count(site) == 0) continue;
		unsigned short luxValue40 = m_LuxMap["PRNU40"][site];
		m_ledCtr.SetCctLux(LedSiteMap[site], luxValue40);
	}
	ATE()->SleepMs(50);
    CISImage()->DVPImageAcquireConfig(result, "PRNU40", dataWidth, imageAverage, true, imageWidth, imageHeight, 5000, true);
	for (auto &site : vSites)
	{

		if (LedSiteMap.count(site) == 0) continue;
		unsigned short luxValue80 = m_LuxMap["PRNU80"][site];
		m_ledCtr.SetCctLux(LedSiteMap[site], luxValue80);
	}
	ATE()->SleepMs(50);
    CISImage()->DVPImageAcquireConfig(result, "PRNU80", dataWidth, imageAverage, true, imageWidth, imageHeight, 5000, true);
    NotifyFunc("PRNU", vSites);



	for (auto &site : vSites)
	{

		if (LedSiteMap.count(site) == 0) continue;
		m_ledCtr.SetCctLux(LedSiteMap[site], 0);
	}
    PowerDownSequence();

    return;

}

void HSC02A::DarkCurrent(const std::string &testName, Test_Result &result)
{

	bool bRet = true;
	std::vector<unsigned int> vSites;
	ATE()->Site()->GetActive(vSites);
	unsigned short testNum = ATE()->Test(testName)->GetTestNumber();
	DarkCurrentTestNumber = testNum;

	PowerUpSequence();

	ATE()->DCLevels()->Signal("PCLK")->Assign(DVP_PCLK);
	ATE()->DCLevels()->Signal("HREF")->Assign(DVP_DE);
	ATE()->DCLevels()->Signal("VCYN")->Assign(DVP_VS);
	ATE()->DCLevels()->Signal("D0")->Assign(DVP_D0);
	ATE()->DCLevels()->Signal("D1")->Assign(DVP_D1);
	ATE()->DCLevels()->Signal("D2")->Assign(DVP_D2);
	ATE()->DCLevels()->Signal("D3")->Assign(DVP_D3);
	ATE()->DCLevels()->Signal("D4")->Assign(DVP_D4);
	ATE()->DCLevels()->Signal("D5")->Assign(DVP_D5);
	ATE()->DCLevels()->Signal("D6")->Assign(DVP_D6);
	ATE()->DCLevels()->Signal("D7")->Assign(DVP_D7);
	ATE()->DCLevels()->Signal("D8")->Assign(DVP_D8);
	ATE()->DCLevels()->Signal("D9")->Assign(DVP_D9);

	//ATE()->DCLevels()->Block("ImageVterm")->Execute();

	//写入Dark_0.5s寄存器
	if (IicSpeed)
	{
		bRet = HSC02A::WriteRegisterBlock("Darkframe1");
	}
	else
	{
		bRet = HSC02A::WriteRegister("Darkframe1");
	}

	ATE()->SleepMs(500);

	unsigned short imageWidth = 480, imageHeight = 270;
	unsigned char imageAverage = 1;
	EVT_IMG::EVT_IMG_DATAWIDTH dataWidth = EVT_IMG::DATA_10BIT;

	CISImage()->DVPImageAcquireConfig(result, "test_demo", dataWidth, imageAverage, true, imageWidth, imageHeight, 5000, true);
	ATE()->SleepMs(50);

	CISImage()->DVPImageAcquireConfig(result, "DarkFrame1", dataWidth, imageAverage, true, imageWidth, imageHeight, 5000, true);

	ATE()->SleepMs(50);
	//写入Dark_0.0625s寄存器
	if (IicSpeed)
	{
		bRet = HSC02A::WriteRegisterBlock("Darkframe2");
	}
	else
	{
		bRet = HSC02A::WriteRegister("Darkframe2");
	}

	ATE()->SleepMs(500);
	
	CISImage()->DVPImageAcquireConfig(result, "test_demo", dataWidth, imageAverage, true, imageWidth, imageHeight, 5000, true);
	ATE()->SleepMs(50);

	CISImage()->DVPImageAcquireConfig(result, "DarkFrame2", dataWidth, imageAverage, true, imageWidth, imageHeight, 5000, true);
	NotifyFunc("DarkCurrent", vSites);

	PowerDownSequence();

	return;

}

void HSC02A::CG(const std::string& testName, Test_Result& result)
{

	bool bRet = true;
	std::vector<unsigned int> vSites;
	ATE()->Site()->GetActive(vSites);
	unsigned short testNum = ATE()->Test(testName)->GetTestNumber();
	CGTestNumber = testNum;

	PowerUpSequence();


	ATE()->DCLevels()->Signal("PCLK")->Assign(DVP_PCLK);
	ATE()->DCLevels()->Signal("HREF")->Assign(DVP_DE);
	ATE()->DCLevels()->Signal("VCYN")->Assign(DVP_VS);
	ATE()->DCLevels()->Signal("D0")->Assign(DVP_D0);
	ATE()->DCLevels()->Signal("D1")->Assign(DVP_D1);
	ATE()->DCLevels()->Signal("D2")->Assign(DVP_D2);
	ATE()->DCLevels()->Signal("D3")->Assign(DVP_D3);
	ATE()->DCLevels()->Signal("D4")->Assign(DVP_D4);
	ATE()->DCLevels()->Signal("D5")->Assign(DVP_D5);
	ATE()->DCLevels()->Signal("D6")->Assign(DVP_D6);
	ATE()->DCLevels()->Signal("D7")->Assign(DVP_D7);
	ATE()->DCLevels()->Signal("D8")->Assign(DVP_D8);
	ATE()->DCLevels()->Signal("D9")->Assign(DVP_D9);

	ATE()->DCLevels()->Block("ImageVterm")->Execute();
	if (IicSpeed)
	{
		bRet = HSC02A::WriteRegisterBlock("CG");
	}
	else
	{
		bRet = HSC02A::WriteRegister("CG");
	}

	ATE()->SleepMs(500);


	unsigned short imageWidth = 480, imageHeight = 270;
	unsigned char imageAverage = 2;
	EVT_IMG::EVT_IMG_DATAWIDTH dataWidth = EVT_IMG::DATA_10BIT;

	ATE()->SleepMs(50);

	// 需要对PRNU40或者PRNU80分别打灯
	// 采集图像,并且调用PRNU

	if (m_LedCalFlag)
	{
		EVT_EACH_SITE_BEGIN
			bRet = m_ledCal.CalStartDVP(result, activeSite, dataWidth, 1, imageWidth, imageHeight, "CG_PRNU40", PRNU40_Value, 3000, true, m_Gray);	//Remember modify ImageConfig.ini value,dataLaneSpeedType值从哪里确定,image取1还是imagaverage
		if (bRet)
		{
			result.SetPassFailBySite(activeSite, RESULT_PASS);
		}
		else
		{
			result.SetPassFailBySite(activeSite, RESULT_FAIL);
		}

		m_ledCal.CalFinish(activeSite);

		if (!bRet)
		{
			EVTSYS(LEVEL_ERROR, "Site:%d LowLight call CalStartMipiComponent exception", activeSite);
		}
		EVT_EACH_SITE_END

			//return;
	}

	if (m_LedCalFlag)
	{
		EVT_EACH_SITE_BEGIN
			bRet = m_ledCal.CalStartDVP(result, activeSite, dataWidth, 1, imageWidth, imageHeight, "CG_PRNU80", PRNU80_Value, 3000, true, m_Gray);	//Remember modify ImageConfig.ini value,dataLaneSpeedType值从哪里确定,image取1还是imagaverage
		if (bRet)
		{
			result.SetPassFailBySite(activeSite, RESULT_PASS);
		}
		else
		{
			result.SetPassFailBySite(activeSite, RESULT_FAIL);
		}

		m_ledCal.CalFinish(activeSite);

		if (!bRet)
		{
			EVTSYS(LEVEL_ERROR, "Site:%d LowLight call CalStartMipiComponent exception", activeSite);
		}
		EVT_EACH_SITE_END

			return;
	}



	for (auto& site : vSites)
	{

		if (LedSiteMap.count(site) == 0) continue;
		unsigned short luxValue40 = m_LuxMap["CG_PRNU40"][site];
		m_ledCtr.SetCctLux(LedSiteMap[site], luxValue40);
	}
	ATE()->SleepMs(500);
	CISImage()->DVPImageAcquireConfig(result, "CG_PRNU40", dataWidth, imageAverage, true, imageWidth, imageHeight, 5000, true);



	for (auto& site : vSites)
	{

		if (LedSiteMap.count(site) == 0) continue;
		unsigned short luxValue80 = m_LuxMap["CG_PRNU80"][site];
		m_ledCtr.SetCctLux(LedSiteMap[site], luxValue80);
	}
	ATE()->SleepMs(500);
	CISImage()->DVPImageAcquireConfig(result, "CG_PRNU80", dataWidth, imageAverage, true, imageWidth, imageHeight, 5000, true);
	NotifyFunc("CG", vSites);


	for (auto& site : vSites)
	{

		if (LedSiteMap.count(site) == 0) continue;
		m_ledCtr.SetCctLux(LedSiteMap[site], 0);
	}

	//if (IicSpeed)
	//{
	//	bRet = HSC02A::WriteRegisterBlock("Dark_DPCOff");
	//}
	//else
	//{
	//	bRet = HSC02A::WriteRegister("Dark_DPCOff");
	//}


	PowerDownSequence();

	return;

}

void HSC02A::NotifyFunc(const std::string& testName, unsigned int site)
{
    vector<unsigned int> vRsrc;
    ATE()->Site()->GetSiteLocation(site, vRsrc);

    EVEREST::UsrMsg_ImageInfo usrMsg;
    usrMsg.dmaNotify.msgHeader.eventID = MakeUsrEventID(UsrEvent_Image, IMAGE_READY);
    usrMsg.dmaNotify.msgData0.param.ulLow = vRsrc[0];
    usrMsg.dmaNotify.msgData0.param.ulHigh = site;
    usrMsg.dmaNotify.msgData1.param.ulLow = 0;       // bits of per pixel
    usrMsg.dmaNotify.msgData1.param.ulHigh = 0;
    strcpy_s(usrMsg.dmaNotify.msgData2.strData, sizeof(usrMsg.dmaNotify.msgData2.strData), testName.c_str());
    usrMsg.imageAverage = 0;
    usrMsg.width = 0;
    usrMsg.height = 0;
    usrMsg.frameCount = 1;

    ATE()->HWInterface()->NotifyUserEvent(&usrMsg, sizeof(UsrMsg_ImageInfo), 0x1 << site, true);
}

void HSC02A::NotifyFunc(const std::string& testName, const std::vector<unsigned int>& sites)
{
    for (auto site : sites)
    {
        NotifyFunc(testName, site);
    }
}