#pragma once
#include <string>
#include <map>
#include <mutex>
#include "EvtImageLib/ImageConversion/ImageCommon.h"
#include "LedCtr.h"
#include "EvtPrinter\LogServer.h"
#include "TestExec\EvtATELib\EvtTestInterface.h"

class LedCtr;
class LedCal
{
public:

	LedCal(LedCtr& controller);

	void SetCalEnable(bool flag);
	bool ReadLedCalFile();
	std::map<unsigned int, unsigned short> GetValue(std::string testName);

	bool CalReadDVPImage(
		std::string testName,
		unsigned int siteNum,
		unsigned short imageWidth,
		unsigned short imageHeight,
		unsigned char frameValidCnt,
		double &pixelAvg,
		bool gray);
//LedTest
	bool LedTestReadDVPImage(
		std::string testName,
		unsigned int siteNum,
		unsigned short imageWidth,
		unsigned short imageHeight,
		unsigned char frameValidCnt,
		double &RpixelAvg,
		double &GrpixelAvg,
		double &GbpixelAvg,
		double &BpixelAvg,
		bool gray
	);


	bool CalDVPImageAcquireConfig(
		Test_Result &result,	//add by GX
		std::string testName,	//add by GX
		EVT_IMG::EVT_IMG_DATAWIDTH dataWidth,
		double &pixelAvg,
		unsigned char imageAverage,
		unsigned short imageWidth,
		unsigned short imageHeight,
		unsigned short timeout,
		bool isOneSegmentTwoSite,
		bool gray);	//add by GX

	bool CalStartDVP(
		Test_Result &result,	//add by GX
		unsigned int siteNum,
		EVT_IMG::EVT_IMG_DATAWIDTH dataWidth,
		unsigned char imageAverage,	  //add by GX
		unsigned short imageWidth,
		unsigned short imageHeight,
		std::string testName,
		double tarVal,
		unsigned short timeout,		//add by GX
		bool isOneSegmentTwoSite,
		bool gray);  //add by GX

	bool StartFWC(
		Test_Result &result,
		unsigned int siteNum,
		EVT_IMG::EVT_IMG_DATAWIDTH dataWidth,
		unsigned char imageAverage,
		unsigned short imageWidth,
		unsigned short imageHeight,
		std::string testName,
		double tarVal,
		unsigned short timeout,
		bool isOneSegmentTwoSite,
		bool gray

	);

	bool CalDVP(
		Test_Result &result,	//add by GX
		std::string testName,
		unsigned int siteNum,
		unsigned short lo,
		unsigned short hi,
		double tarVal,
		unsigned short &mid,
		EVT_IMG::EVT_IMG_DATAWIDTH dataWidth,
		unsigned char imageAverage,	  //add by GX
		unsigned short imageWidth,
		unsigned short imageHeight,
		unsigned short timeout,			//add by GX
		bool isOneSegmentTwoSite);		//add by GX

	bool ImagePixelAverage(
		std::string testName,
		unsigned int siteNum,
		unsigned short imageWidth,
		unsigned short imageHeight,
		unsigned char frameValidCnt,
		double &pixelAvg);

	bool CalMipiImageAcquireComponent(
		Test_Result &result,	//add by GX
		std::string testName,   //add by GX
		EVT_IMG::EVT_IMG_DATAWIDTH dataWidth,
		double &pixelAvg,
		unsigned char clkLaneCnt,
		unsigned char dataLaneCnt,
		unsigned char dataLaneSpeedType,	//add by GX
		unsigned char imageAverage,
		unsigned short imageWidth,
		unsigned short imageHeight,
		unsigned short timeout,
		bool isOneSegmentTwoSite);	//add by GX

	bool CalStartMipiComponent(
		Test_Result &result,	//add by GX
		unsigned int siteNum,
		EVT_IMG::EVT_IMG_DATAWIDTH dataWidth,
		unsigned char clkLaneCnt,
		unsigned char dataLaneCnt,
		unsigned char dataLaneSpeedType,	//add by GX
		unsigned char imageAverage,	  //add by GX
		unsigned short imageWidth,
		unsigned short imageHeight,
		std::string testName,
		double tarVal,
		unsigned short timeout,		//add by GX
		bool isOneSegmentTwoSite);  //add by GX

	void CalFinish(unsigned int siteNum);
	void CalSetValue(std::string testName, unsigned int siteNum, unsigned short lux);
	//LedTest
	void LedTestSetValue(std::string testName, unsigned int siteNum, unsigned short lux);

public:
	LedCal();
	~LedCal();
	bool m_CalFlag = false;

	unsigned int m_LedMax = 0;
	unsigned int m_LedMin = 0;
	unsigned int m_FWCstep = 0;

	unsigned int m_SettleCount = 0;
	unsigned int m_ClkMode = 0;
	std::mutex m_mutex;

	LedCtr &m_ledCtrl;
	HMODULE m_pAlgModule = NULL;
};