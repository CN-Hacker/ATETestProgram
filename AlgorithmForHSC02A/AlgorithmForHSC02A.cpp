#include "stdafx.h"
#include "AlgorithmForHSC02A.h"
#include "HSC02_Image_Algorithm.h"
#include "ImageResultDefine.h"
#include <iostream>
#include <fstream>
#include <sstream>
#include <set>
#include <filesystem>
#include <string>
#include <conio.h>
#include <windows.h>
#include <string.h>
#include <boost/algorithm/string.hpp>
#include <boost/filesystem.hpp>
#include <boost/property_tree/ptree.hpp>
#include <boost/property_tree/ini_parser.hpp>
#include <direct.h>
#include "opencv2/opencv.hpp"

namespace fs = boost::filesystem;
using namespace std;
//#include "md5.h"//test

#define pixelWidth 480
#define pixelHeight 270

#define maxImgCount 5
unsigned short* g_pBuf = NULL;

//HMODULE  m_hLib = NULL;
unsigned int Cnt;
static char* gDMAAddr;

ImageResults gv_ImageResults;

bool bConvertBinaryCode = false;
std::vector<unsigned short> ImagePRNU40;
std::vector<unsigned short> ImagePRNU80;

std::vector<unsigned short> ImgDarkFrame1;
std::vector<unsigned short> ImgDarkFrame2;

std::vector<unsigned short> CGImagePRNU40;
std::vector<unsigned short> CGImagePRNU80;

void DarkWithDPCOnResultConvert(const ImgResult_Dark_DPCOn& algorithmResult, DarkImgResultWithDPCOn& testprogramResult)
{
    testprogramResult.badPixelCount = algorithmResult.BadPixelCount;
    testprogramResult.hDeadLineSize = algorithmResult.Horizontal_DeadLine_Length;
    testprogramResult.vDeadLineSize = algorithmResult.Vertical_DeadLine_Length;
    testprogramResult.mean = algorithmResult.Mean;
    testprogramResult.std = algorithmResult.Std;

    testprogramResult.clusterCount = algorithmResult.ClusterCount;
    testprogramResult.cluster_Cluster5 = algorithmResult.EachClustersSize[Cluster_Cluster5];
    testprogramResult.cluster_4T1 = algorithmResult.EachClustersSize[Cluster_4T1];
    testprogramResult.cluster_4T2 = algorithmResult.EachClustersSize[Cluster_4T2];
    testprogramResult.cluster_4T3 = algorithmResult.EachClustersSize[Cluster_4T3];
    testprogramResult.cluster_4T4 = algorithmResult.EachClustersSize[Cluster_4T4];
    testprogramResult.cluster_3X1 = algorithmResult.EachClustersSize[Cluster_3X1];
    testprogramResult.cluster_1X3 = algorithmResult.EachClustersSize[Cluster_1X3];
    testprogramResult.cluster_2X1 = algorithmResult.EachClustersSize[Cluster_2X1];
    testprogramResult.cluster_1X2 = algorithmResult.EachClustersSize[Cluster_1X2];
    testprogramResult.cluster_2X1X1 = algorithmResult.EachClustersSize[Cluster_2X1X1];
    testprogramResult.cluster_1X2X1 = algorithmResult.EachClustersSize[Cluster_1X2X1];
    testprogramResult.cluster_2X2 = algorithmResult.EachClustersSize[Cluster_2X2];
    testprogramResult.cluster_2D1 = algorithmResult.EachClustersSize[Cluster_2D1];
    testprogramResult.cluster_2D2 = algorithmResult.EachClustersSize[Cluster_2D2];
    testprogramResult.cluster_2D3 = algorithmResult.EachClustersSize[Cluster_2D3];
    testprogramResult.cluster_2D4 = algorithmResult.EachClustersSize[Cluster_2D4];
    testprogramResult.cluster_1X1 = algorithmResult.EachClustersSize[Cluster_1X1];
    testprogramResult.cluster_Big = algorithmResult.EachClustersSize[Cluster_Big];
}

void DarkWithDPCOffResultConvert(const ImgResult_Dark_DPCOff& algorithmResult, DarkImgResultWithDPCOff& testprogramResult)
{
    testprogramResult.badPixelCount1 = algorithmResult.BadPixel1_Count;
    testprogramResult.badPixelCount2 = algorithmResult.BadPixel2_Count;
    testprogramResult.mean = algorithmResult.Mean;
    testprogramResult.std = algorithmResult.Std;
}


void TempNoiseResultConvert(const ImgResult_Dark_TmpNoise& algorithmResult, TempNoiseResult& testprogramResult)
{
	testprogramResult.TempNoise=algorithmResult.TmpNoise;
}

void DarkFPNResultConvert(const ImgResult_DarkFPN& algorithmResult, DarkFPNResult& testprogramResult)
{
    testprogramResult.AFPN = algorithmResult.AFPN;
	testprogramResult.HFPN = algorithmResult.HFPN;
	testprogramResult.VFPN = algorithmResult.VFPN;
}

void DarkCurrentResultConvert(const ImgResult_DarkCurrent& algorithmResult, DarkCurrentResult& testprogramResult)
{
	testprogramResult.darkcurrent = algorithmResult.darkcurrent;

}

void PRNUResultConvert(const ImgResult_PRNU& algorithmResult, PRNUResult& testprogramResult)
{
    testprogramResult.PRNU40_R = algorithmResult.PRNU40_R;
    testprogramResult.PRNU80_R = algorithmResult.PRNU80_R;
    testprogramResult.PRNU_R = algorithmResult.PRNU_R;
    testprogramResult.PRNU40_Gr = algorithmResult.PRNU40_Gr;
    testprogramResult.PRNU80_Gr = algorithmResult.PRNU80_Gr;
    testprogramResult.PRNU_Gr = algorithmResult.PRNU_Gr;
    testprogramResult.PRNU40_Gb = algorithmResult.PRNU40_Gb;
    testprogramResult.PRNU80_Gb = algorithmResult.PRNU80_Gb;
    testprogramResult.PRNU_Gb = algorithmResult.PRNU_Gb;
    testprogramResult.PRNU40_B = algorithmResult.PRNU40_B;
    testprogramResult.PRNU80_B = algorithmResult.PRNU80_B;
    testprogramResult.PRNU_B = algorithmResult.PRNU_B;

	//add in 2023/06/30     Sensitivity
	testprogramResult.Diff_R = algorithmResult.Diff_R;
	testprogramResult.Diff_Gr = algorithmResult.Diff_Gr;
	testprogramResult.Diff_Gb = algorithmResult.Diff_Gb;
	testprogramResult.Diff_B = algorithmResult.Diff_B;
}

void CGResultConvert(const ImgResult_CG& algorithmResult, CGResult& testprogramResult)
{
	//testprogramResult.PRNU40_R = algorithmResult.PRNU40_R;
	testprogramResult.CG_B = algorithmResult.CG_B;
	testprogramResult.CG_R = algorithmResult.CG_R;
	testprogramResult.CG_Gb = algorithmResult.CG_Gb;
	testprogramResult.CG_Gr = algorithmResult.CG_Gr;
	testprogramResult.CG = algorithmResult.CG;
}


void LightResultConvert(const ImgResult_Light& algorithmResult, LightImgResult& testprogramResult)
{
    testprogramResult.hDeadLineCount_R = algorithmResult.R_HorizontalDeadLineCount;
    testprogramResult.hDeadLineCount_Gr = algorithmResult.Gr_HorizontalDeadLineCount;
    testprogramResult.hDeadLineCount_Gb = algorithmResult.Gb_HorizontalDeadLineCount;
    testprogramResult.hDeadLineCount_B = algorithmResult.B_HorizontalDeadLineCount;
    
    testprogramResult.vDeadLineCount_R = algorithmResult.R_VerticalDeadLineCount;
    testprogramResult.vDeadLineCount_Gr = algorithmResult.Gr_VerticalDeadLineCount;
    testprogramResult.vDeadLineCount_Gb = algorithmResult.Gb_VerticalDeadLineCount;
    testprogramResult.vDeadLineCount_B = algorithmResult.B_VerticalDeadLineCount;

    testprogramResult.blackPixelCount_R = algorithmResult.R_BlackPixelCount;
    testprogramResult.blackPixelCount_Gr = algorithmResult.Gr_BlackPixelCount;
    testprogramResult.blackPixelCount_Gb = algorithmResult.Gb_BlackPixelCount;
    testprogramResult.blackPixelCount_B = algorithmResult.B_BlackPixelCount;

    testprogramResult.whitePixelCount_R = algorithmResult.R_WhitePixelCount;
    testprogramResult.whitePixelCount_Gr = algorithmResult.Gr_WhitePixelCount;
    testprogramResult.whitePixelCount_Gb = algorithmResult.Gb_WhitePixelCount;
    testprogramResult.whitePixelCount_B = algorithmResult.B_WhitePixelCount;

    testprogramResult.radio_R_B = algorithmResult.R_B;
    testprogramResult.radio_Gr_Gb = algorithmResult.Gr_Gb;
    testprogramResult.radio_R_Gr = algorithmResult.R_Gr;
    testprogramResult.radio_B_Gb = algorithmResult.B_Gb;

    testprogramResult.mean = algorithmResult.Mean;
    testprogramResult.mean_R = algorithmResult.Mean_R;
    testprogramResult.mean_Gr = algorithmResult.Mean_Gr;
    testprogramResult.mean_Gb = algorithmResult.Mean_Gb;
    testprogramResult.mean_B = algorithmResult.Mean_B;

    testprogramResult.std_R = algorithmResult.Std_R;
    testprogramResult.std_Gr = algorithmResult.Std_Gr;
    testprogramResult.std_Gb = algorithmResult.Std_Gb;
    testprogramResult.std_B = algorithmResult.Std_B;

    testprogramResult.shading_ROI1 = algorithmResult.LUShadings[ROI_1];
    testprogramResult.shading_ROI2 = algorithmResult.LUShadings[ROI_2];
    testprogramResult.shading_ROI3 = algorithmResult.LUShadings[ROI_3];
    testprogramResult.shading_ROI4 = algorithmResult.LUShadings[ROI_4];
    testprogramResult.shading_ROI6 = algorithmResult.LUShadings[ROI_6];
    testprogramResult.shading_ROI7 = algorithmResult.LUShadings[ROI_7];
    testprogramResult.shading_ROI8 = algorithmResult.LUShadings[ROI_8];
    testprogramResult.shading_ROI9 = algorithmResult.LUShadings[ROI_9];

    testprogramResult.balance_RB_ROI1 = algorithmResult.CUBalances_RB[ROI_1];
    testprogramResult.balance_RB_ROI2 = algorithmResult.CUBalances_RB[ROI_2];
    testprogramResult.balance_RB_ROI3 = algorithmResult.CUBalances_RB[ROI_3];
    testprogramResult.balance_RB_ROI4 = algorithmResult.CUBalances_RB[ROI_4];
    testprogramResult.balance_RB_ROI6 = algorithmResult.CUBalances_RB[ROI_6];
    testprogramResult.balance_RB_ROI7 = algorithmResult.CUBalances_RB[ROI_7];
    testprogramResult.balance_RB_ROI8 = algorithmResult.CUBalances_RB[ROI_8];
    testprogramResult.balance_RB_ROI9 = algorithmResult.CUBalances_RB[ROI_9];
    testprogramResult.balances_RB_Max = algorithmResult.CUBalances_RB_Max;

    testprogramResult.balance_GrGb_ROI1 = algorithmResult.CUBalances_GrGb[ROI_1];
    testprogramResult.balance_GrGb_ROI2 = algorithmResult.CUBalances_GrGb[ROI_2];
    testprogramResult.balance_GrGb_ROI3 = algorithmResult.CUBalances_GrGb[ROI_3];
    testprogramResult.balance_GrGb_ROI4 = algorithmResult.CUBalances_GrGb[ROI_4];
    testprogramResult.balance_GrGb_ROI6 = algorithmResult.CUBalances_GrGb[ROI_6];
    testprogramResult.balance_GrGb_ROI7 = algorithmResult.CUBalances_GrGb[ROI_7];
    testprogramResult.balance_GrGb_ROI8 = algorithmResult.CUBalances_GrGb[ROI_8];
    testprogramResult.balance_GrGb_ROI9 = algorithmResult.CUBalances_GrGb[ROI_9];
    testprogramResult.balances_GrGb_Max = algorithmResult.CUBalances_GrGb_Max;

    testprogramResult.RG_ROI19 = algorithmResult.RG_ROI19;
    testprogramResult.RG_ROI37 = algorithmResult.RG_ROI37;
    testprogramResult.RG_ROI28 = algorithmResult.RG_ROI28;
    testprogramResult.RG_ROI46 = algorithmResult.RG_ROI46;

    testprogramResult.BG_ROI19 = algorithmResult.BG_ROI19;
    testprogramResult.BG_ROI37 = algorithmResult.BG_ROI37;
    testprogramResult.BG_ROI28 = algorithmResult.BG_ROI28;
    testprogramResult.BG_ROI46 = algorithmResult.BG_ROI46;

    testprogramResult.ratio_ROI5_R_B = algorithmResult.ROI5_R_B;
    testprogramResult.ratio_ROI5_Gr_Gb = algorithmResult.ROI5_Gr_Gb;
    testprogramResult.ratio_ROI5_R_Gr = algorithmResult.ROI5_R_Gr;
    testprogramResult.ratio_ROI5_B_Gb = algorithmResult.ROI5_B_Gb;

    testprogramResult.blemish = algorithmResult.BlemishCount;
    testprogramResult.scratch = algorithmResult.ScratchCount;

    testprogramResult.clusterCount = algorithmResult.ClusterCount;
	testprogramResult.cluster_Cluster5 = algorithmResult.EachClustersSize[Cluster_Cluster5];
	testprogramResult.cluster_4T1 = algorithmResult.EachClustersSize[Cluster_4T1];
	testprogramResult.cluster_4T2 = algorithmResult.EachClustersSize[Cluster_4T2];
	testprogramResult.cluster_4T3 = algorithmResult.EachClustersSize[Cluster_4T3];
	testprogramResult.cluster_4T4 = algorithmResult.EachClustersSize[Cluster_4T4];
	testprogramResult.cluster_3X1 = algorithmResult.EachClustersSize[Cluster_3X1];
	testprogramResult.cluster_1X3 = algorithmResult.EachClustersSize[Cluster_1X3];
	testprogramResult.cluster_2X1 = algorithmResult.EachClustersSize[Cluster_2X1];
	testprogramResult.cluster_1X2 = algorithmResult.EachClustersSize[Cluster_1X2];
	testprogramResult.cluster_2X1X1 = algorithmResult.EachClustersSize[Cluster_2X1X1];
	testprogramResult.cluster_1X2X1 = algorithmResult.EachClustersSize[Cluster_1X2X1];
	testprogramResult.cluster_2X2 = algorithmResult.EachClustersSize[Cluster_2X2];
	testprogramResult.cluster_2D1 = algorithmResult.EachClustersSize[Cluster_2D1];
	testprogramResult.cluster_2D2 = algorithmResult.EachClustersSize[Cluster_2D2];
	testprogramResult.cluster_2D3 = algorithmResult.EachClustersSize[Cluster_2D3];
	testprogramResult.cluster_2D4 = algorithmResult.EachClustersSize[Cluster_2D4];
	testprogramResult.cluster_1X1 = algorithmResult.EachClustersSize[Cluster_1X1];
	testprogramResult.cluster_Big = algorithmResult.EachClustersSize[Cluster_Big];
}

void FWCResultConvert(const ImageResult_FWC& algorithmResult, FWCResult& testprogramResult)
{
	testprogramResult.R_mean = algorithmResult.Mean_R;
	testprogramResult.Gr_mean = algorithmResult.Mean_Gr;
	testprogramResult.Gb_mean = algorithmResult.Mean_Gb;
	testprogramResult.B_mean = algorithmResult.Mean_B;

	testprogramResult.R_std = algorithmResult.Std_R;
	testprogramResult.Gr_std = algorithmResult.Std_Gr;
	testprogramResult.Gb_std = algorithmResult.Std_Gb;
	testprogramResult.B_std = algorithmResult.Std_B;

}

static std::string ConvertTime2Str(time_t time)
{
	char buf[128] = {};
	tm local;
	localtime_s(&local, &time);
	strftime(buf, 128, "%Y/%m/%d %H:%M:%S", &local);
	return buf;
}

EVT_ALG_RET Init(char *pAddr)
{
	gDMAAddr = pAddr;

	Cnt = 0;
	
	char iniName[MAX_PATH];
	GetCurrentDirectoryA(MAX_PATH, iniName);
	snprintf(iniName, MAX_PATH, "%s\\AlgConfig.ini", iniName);
	
    try
    {
        boost::property_tree::ptree pt;
        boost::property_tree::read_ini(iniName, pt);
        if (pt.get<int>("ConvertBinaryCode") == 1)
        {
            bConvertBinaryCode = true;
        }
    }
    catch (const boost::exception& e)
    {
        bConvertBinaryCode = false;
    }

	g_pBuf = new unsigned short[pixelWidth * pixelHeight * sizeof(unsigned short)*maxImgCount];	

	if (g_pBuf)
	{
		return ALG_SUCCESS;
	}
	else
	{
		return ALG_ERR_BAD_ALLOC;
	}
}

EVT_ALG_RET Deinit()
{
	
	if (g_pBuf)
	{
		delete[] g_pBuf;
	}

	
	return ALG_SUCCESS;
}

EVT_ALG_RET Dark_DPCOn(
	unsigned short *rawData,
	unsigned char *frameData,
	int frameWidth,
	int frameHeight,
	int frameCnt,
	EVT_IMG_FORMAT frameFormat,
	int pixelDepth,
	unsigned int siteNum,
	SaveImageCallBack saveCallBack,
	int argc,
	char** argv
)
{
	EVT_ALG_RET ret = ALG_SUCCESS;

	ImageResult result;
	strcpy_s(result.testName, "Dark_DPCOn");

	cv::Mat r_RawData;
	cv::Mat srcImage = cv::Mat(pixelHeight, pixelWidth, CV_16UC1, rawData);
	cv::Rect cvImage(0, 0, pixelWidth, pixelHeight - 80);
	srcImage(cvImage).copyTo(r_RawData);

	DarkImgResultWithDPCOn darkImageResult;
	try
	{
		ImgCondition_Dark_DPCOn ImageCondition;
		ImgResult_Dark_DPCOn algorithmDarkImageResult;
		DarkImageWithDPCOnTest((unsigned short*)r_RawData.data, pixelHeight - 80, pixelWidth, ImageCondition, &algorithmDarkImageResult);

		DarkWithDPCOnResultConvert(algorithmDarkImageResult, darkImageResult);

	}
	catch (cv::Exception& e)
	{
		time_t currentTime = time(0);
		tm ctm;
		localtime_s(&ctm, &currentTime);
		char stime[256];
		memset(stime, 0, 256);
		std::strftime(stime, 256, "%Y%m%d%H%M%S", &ctm);
		std::string failRawName = "FailImage";
		failRawName = failRawName + __FUNCDNAME__ + stime + ".raw";
		std::ofstream ofs(failRawName, ios::trunc | ios::binary);
		ofs.write((char*)rawData, pixelHeight * pixelWidth * 2);
		ofs.close();
	}

	memcpy(result.pData, &darkImageResult, sizeof(darkImageResult));
	gv_ImageResults.push_back(result);
    
	bool savingStatus = false;
	saveCallBack(savingStatus, false, 0, nullptr);
	return ret;
}

EVT_ALG_RET Dark_DPCOff(
	unsigned short *rawData,
	unsigned char *frameData,
	int frameWidth,
	int frameHeight,
	int frameCnt,
	EVT_IMG_FORMAT frameFormat,
	int pixelDepth,
	unsigned int siteNum,
	SaveImageCallBack saveCallBack,
	int argc,
	char** argv
)
{
	EVT_ALG_RET ret = ALG_SUCCESS;


	ImageResult result;
	strcpy_s(result.testName, "Dark_DPCOff");

	cv::Mat r_RawData;
	cv::Mat srcImage = cv::Mat(pixelHeight, pixelWidth, CV_16UC1, rawData);
	cv::Rect cvImage(0, 0, pixelWidth, pixelHeight - 80);
	srcImage(cvImage).copyTo(r_RawData);

	DarkImgResultWithDPCOff darkImageResult;
	try
	{
		ImgCondition_Dark_DPCOff ImageCondition;
		ImgResult_Dark_DPCOff algorithmDarkImageResult;
		DarkImageWithDPCOffTest((unsigned short*)r_RawData.data, pixelHeight - 80, pixelWidth, ImageCondition, &algorithmDarkImageResult);
		DarkWithDPCOffResultConvert(algorithmDarkImageResult, darkImageResult);
	}
	catch(cv::Exception& e)
	{
		time_t currentTime = time(0);
		tm ctm;
		localtime_s(&ctm, &currentTime);
		char stime[256];
		memset(stime, 0, 256);
		std::strftime(stime, 256, "%Y%m%d%H%M%S", &ctm);
		std::string failRawName = "FailImage";
		failRawName = failRawName + __FUNCDNAME__ + stime + ".raw";
		std::ofstream ofs(failRawName, ios::trunc | ios::binary);
		ofs.write((char*)rawData, pixelHeight * pixelWidth * 2);
		ofs.close();
	}


	memcpy(result.pData, &darkImageResult, sizeof(darkImageResult));
	gv_ImageResults.push_back(result);

	bool savingStatus = false;
	saveCallBack(savingStatus, false, 0, nullptr);
	return ret;
}

EVT_ALG_RET TempNoise(
	unsigned short *rawData,
	unsigned char *frameData,
	int frameWidth,
	int frameHeight,
	int frameCnt,
	EVT_IMG_FORMAT frameFormat,
	int pixelDepth,
	unsigned int siteNum,
	SaveImageCallBack saveCallBack,
	int argc,
	char** argv
)
{
	EVT_ALG_RET ret = ALG_SUCCESS;

	ImageResult result;
	strcpy_s(result.testName, "TempNoise");

	TempNoiseResult darkTmpNoiseResult;
	ImgCondition_Dark_TmpNoise TmpNoiseCondition;
	ImgResult_Dark_TmpNoise algorithmTempNoiseResult;
	DarkTmpNoise(rawData,frameHeight,frameWidth,TmpNoiseCondition,&algorithmTempNoiseResult);
	TempNoiseResultConvert(algorithmTempNoiseResult,darkTmpNoiseResult);

	memcpy(result.pData, &darkTmpNoiseResult, sizeof(darkTmpNoiseResult));
	gv_ImageResults.push_back(result);

	bool savingStatus = false;
	saveCallBack(savingStatus, false, 0, nullptr);
	return ret;
}

EVT_ALG_RET DarkFrame1(
	unsigned short *rawData,
	unsigned char *frameData,
	int frameWidth,
	int frameHeight,
	int frameCnt,
	EVT_IMG_FORMAT frameFormat,
	int pixelDepth,
	unsigned int siteNum,
	SaveImageCallBack saveCallBack,
	int argc,
	char** argv
)
{
	EVT_ALG_RET ret = ALG_SUCCESS;

	ImgDarkFrame1.clear();
	ImgDarkFrame1.resize(pixelHeight * pixelWidth * frameCnt, 0);
	::memcpy_s(&ImgDarkFrame1[0], ImgDarkFrame1.size() * 2, rawData, ImgDarkFrame1.size() * 2);

	bool savingStatus = false;
	saveCallBack(savingStatus, false, 0, nullptr);
	return ret;
}

EVT_ALG_RET DarkFrame2(
	unsigned short *rawData,
	unsigned char *frameData,
	int frameWidth,
	int frameHeight,
	int frameCnt,
	EVT_IMG_FORMAT frameFormat,
	int pixelDepth,
	unsigned int siteNum,
	SaveImageCallBack saveCallBack,
	int argc,
	char** argv
)
{
	EVT_ALG_RET ret = ALG_SUCCESS;

	ImgDarkFrame2.clear();
	ImgDarkFrame2.resize(pixelHeight * pixelWidth * frameCnt, 0);
	::memcpy_s(&ImgDarkFrame2[0], ImgDarkFrame2.size() * 2, rawData, ImgDarkFrame2.size() * 2);

	bool savingStatus = false;
	saveCallBack(savingStatus, false, 0, nullptr);
	return ret;
}

EVT_ALG_RET DarkCurrent(
	unsigned int rsrcID,
	unsigned int siteNum,
	int argc,
	char** argv
)
{
	if (ImgDarkFrame1.empty() || ImgDarkFrame2.empty())
	{
		return EVT_ALG_RET::ALG_ERR_BAD_INPUT;
	}

	if (ImgDarkFrame1.size() != ImgDarkFrame2.size())
	{
		return EVT_ALG_RET::ALG_ERR_BAD_INPUT;
	}

	ImgCondition_DarkCurrent condition;
	auto s = condition.FrameCount * pixelHeight * pixelWidth;
	if (s != ImgDarkFrame2.size())
	{
		return EVT_ALG_RET::ALG_ERR_BAD_INPUT;
	}

	ImgResult_DarkCurrent algorithmDarkCurrentResult;
	DarkCurrentResult DarkCurrentImageResult;

	DarkCurrentTest(&ImgDarkFrame1[0], &ImgDarkFrame2[0], pixelHeight, pixelWidth, condition, &algorithmDarkCurrentResult);
	DarkCurrentResultConvert(algorithmDarkCurrentResult, DarkCurrentImageResult);



	ImageResult result;
	strcpy_s(result.testName, "DarkCurrent");

	memcpy(result.pData, &DarkCurrentImageResult, sizeof(DarkCurrentImageResult));
	gv_ImageResults.push_back(result);

	ImgDarkFrame1.clear();
	ImgDarkFrame2.clear();
	return ALG_SUCCESS;
}

EVT_ALG_RET test_demo(
	unsigned short *rawData,
	unsigned char *frameData,
	int frameWidth,
	int frameHeight,
	int frameCnt,
	EVT_IMG_FORMAT frameFormat,
	int pixelDepth,
	unsigned int siteNum,
	SaveImageCallBack saveCallBack,
	int argc,
	char** argv
)
{
	EVT_ALG_RET ret = ALG_SUCCESS;


	////ImageResult result;
	////strcpy_s(result.testName, "Dark_DPCOff");

	////cv::Mat r_RawData;
	////cv::Mat srcImage = cv::Mat(pixelHeight, pixelWidth, CV_16UC1, rawData);
	////cv::Rect cvImage(0, 0, pixelWidth, pixelHeight - 80);
	////srcImage(cvImage).copyTo(r_RawData);

	//DarkImgResultWithDPCOff darkImageResult;
	//try
	//{
	//	ImgCondition_Dark_DPCOff ImageCondition;
	//	ImgResult_Dark_DPCOff algorithmDarkImageResult;
	//	DarkImageWithDPCOffTest((unsigned short*)r_RawData.data, pixelHeight - 80, pixelWidth, ImageCondition, &algorithmDarkImageResult);
	//	DarkWithDPCOffResultConvert(algorithmDarkImageResult, darkImageResult);
	//}
	//catch(cv::Exception& e)
	//{
	//	time_t currentTime = time(0);
	//	tm ctm;
	//	localtime_s(&ctm, &currentTime);
	//	char stime[256];
	//	memset(stime, 0, 256);
	//	std::strftime(stime, 256, "%Y%m%d%H%M%S", &ctm);
	//	std::string failRawName = "FailImage";
	//	failRawName = failRawName + __FUNCDNAME__ + stime + ".raw";
	//	std::ofstream ofs(failRawName, ios::trunc | ios::binary);
	//	ofs.write((char*)rawData, pixelHeight * pixelWidth * 2);
	//	ofs.close();
	//}


	/*memcpy(result.pData, &darkImageResult, sizeof(darkImageResult));
	gv_ImageResults.push_back(result);*/

	//bool savingStatus = false;
	//saveCallBack(savingStatus, false, 0, nullptr);
	return ret;
}

EVT_ALG_RET DarkFPN(
	unsigned short *rawData,
	unsigned char *frameData,
	int frameWidth,
	int frameHeight,
	int frameCnt,
	EVT_IMG_FORMAT frameFormat,
	int pixelDepth,
	unsigned int siteNum,
	SaveImageCallBack saveCallBack,
	int argc,
	char** argv
)
{
	EVT_ALG_RET ret = ALG_SUCCESS;


	ImageResult result;
	strcpy_s(result.testName, "DarkFPN");
    DarkFPNResult FPNImageResult;

    try
    {
        // 将100张完成的图片放到算法中计算,在Condition中设置ROI
        ImgCondition_DarkFPN condition; // frame count 100
        ImgResult_DarkFPN algorithmFPNImageResult;
        DarkFPNTest(rawData, pixelHeight, pixelWidth, condition, &algorithmFPNImageResult);
        DarkFPNResultConvert(algorithmFPNImageResult, FPNImageResult);
    }
    catch (cv::Exception& e)
    {
        time_t currentTime = time(0);
        tm ctm;
        localtime_s(&ctm, &currentTime);
        char stime[256];
        memset(stime, 0, 256);
        std::strftime(stime, 256, "%Y%m%d%H%M%S", &ctm);
        std::string failRawName = "FailImage";
        failRawName = failRawName + __FUNCDNAME__ + stime + ".raw";
        std::ofstream ofs(failRawName, ios::trunc | ios::binary);
        ofs.write((char*)rawData, pixelHeight * pixelWidth * 2);
        ofs.close();
    }

    memcpy(result.pData, &FPNImageResult, sizeof(FPNImageResult));
	gv_ImageResults.push_back(result);

	bool savingStatus = false;
	saveCallBack(savingStatus, false, 0, nullptr);
	return ret;
}

EVT_ALG_RET PRNU40(
	unsigned short *rawData,
	unsigned char *frameData,
	int frameWidth,
	int frameHeight,
	int frameCnt,
	EVT_IMG_FORMAT frameFormat,
	int pixelDepth,
	unsigned int siteNum,
	SaveImageCallBack saveCallBack,
	int argc,
	char** argv
)
{
	EVT_ALG_RET ret = ALG_SUCCESS;

    ImagePRNU40.clear();
    ImagePRNU40.resize(pixelHeight * pixelWidth * frameCnt, 0);
    ::memcpy_s(&ImagePRNU40[0], ImagePRNU40.size() * 2, rawData, ImagePRNU40.size() * 2);

	bool savingStatus = false;
	saveCallBack(savingStatus, false, 0, nullptr);
	return ret;
}

EVT_ALG_RET PRNU80(
	unsigned short *rawData,
	unsigned char *frameData,
	int frameWidth,
	int frameHeight,
	int frameCnt,
	EVT_IMG_FORMAT frameFormat,
	int pixelDepth,
	unsigned int siteNum,
	SaveImageCallBack saveCallBack,
	int argc,
	char** argv
)
{
	EVT_ALG_RET ret = ALG_SUCCESS;

    ImagePRNU80.clear();
    ImagePRNU80.resize(pixelHeight * pixelWidth * frameCnt, 0);
    ::memcpy_s(&ImagePRNU80[0], ImagePRNU80.size() * 2, rawData, ImagePRNU80.size() * 2);

	bool savingStatus = false;
	saveCallBack(savingStatus, false, 0, nullptr);
	return ret;
}

EVT_ALG_RET CG_PRNU40(
	unsigned short *rawData,
	unsigned char *frameData,
	int frameWidth,
	int frameHeight,
	int frameCnt,
	EVT_IMG_FORMAT frameFormat,
	int pixelDepth,
	unsigned int siteNum,
	SaveImageCallBack saveCallBack,
	int argc,
	char** argv
)
{
	EVT_ALG_RET ret = ALG_SUCCESS;

    CGImagePRNU40.clear();
    CGImagePRNU40.resize(pixelHeight * pixelWidth * frameCnt, 0);
    ::memcpy_s(&CGImagePRNU40[0], CGImagePRNU40.size() * 2, rawData, CGImagePRNU40.size() * 2);

	bool savingStatus = false;
	saveCallBack(savingStatus, false, 0, nullptr);
	return ret;
}

EVT_ALG_RET CG_PRNU80(
	unsigned short *rawData,
	unsigned char *frameData,
	int frameWidth,
	int frameHeight,
	int frameCnt,
	EVT_IMG_FORMAT frameFormat,
	int pixelDepth,
	unsigned int siteNum,
	SaveImageCallBack saveCallBack,
	int argc,
	char** argv
)
{
	EVT_ALG_RET ret = ALG_SUCCESS;

    CGImagePRNU80.clear();
    CGImagePRNU80.resize(pixelHeight * pixelWidth * frameCnt, 0);
    ::memcpy_s(&CGImagePRNU80[0], CGImagePRNU80.size() * 2, rawData, CGImagePRNU80.size() * 2);

	bool savingStatus = false;
	saveCallBack(savingStatus, false, 0, nullptr);
	return ret;
}

EVT_ALG_RET PRNU(
    unsigned int rsrcID,
    unsigned int siteNum,
    int argc,
    char** argv
)
{
    if (ImagePRNU40.empty() || ImagePRNU80.empty())
    {
        return EVT_ALG_RET::ALG_ERR_BAD_INPUT;
    }

    if (ImagePRNU40.size() != ImagePRNU80.size())
    {
        return EVT_ALG_RET::ALG_ERR_BAD_INPUT;
    }

    ImgCondition_PRNU condition;
	auto s = condition.FrameCount * pixelHeight * pixelWidth;
    if (s != ImagePRNU80.size())
    {
        return EVT_ALG_RET::ALG_ERR_BAD_INPUT;
    }

    ImgResult_PRNU algorithmPRNUResult;
    PRNUResult PRNUImageResult;
    try
    {
        TestRPNU(&ImagePRNU40[0], &ImagePRNU80[0], pixelHeight, pixelWidth, condition, &algorithmPRNUResult);
        PRNUResultConvert(algorithmPRNUResult, PRNUImageResult);
    }
    catch (cv::Exception& e)
    {
        time_t currentTime = time(0);
        tm ctm;
        localtime_s(&ctm, &currentTime);
        char stime[256];
        memset(stime, 0, 256);
        std::strftime(stime, 256, "%Y%m%d%H%M%S", &ctm);
        {
            std::string failRawName = "FailImage40";
            failRawName = failRawName + __FUNCDNAME__ + stime + ".raw";
            std::ofstream ofs(failRawName, ios::trunc | ios::binary);
            ofs.write((char*)&ImagePRNU40[0], ImagePRNU40.size() * 2);
            ofs.close();
        }
        {
            std::string failRawName = "FailImage80";
            failRawName = failRawName + __FUNCDNAME__ + stime + ".raw";
            std::ofstream ofs(failRawName, ios::trunc | ios::binary);
            ofs.write((char*)&ImagePRNU80[0], ImagePRNU80.size() * 2);
            ofs.close();
        }
    }

    ImageResult result;
    strcpy_s(result.testName, "PRNU");

    memcpy(result.pData, &PRNUImageResult, sizeof(PRNUImageResult));
    gv_ImageResults.push_back(result);

    ImagePRNU40.clear();
    ImagePRNU80.clear();
    return ALG_SUCCESS;
}

EVT_ALG_RET CG(
	unsigned int rsrcID,
	unsigned int siteNum,
	int argc,
	char** argv
)
{
	if (CGImagePRNU40.empty() || CGImagePRNU80.empty())
	{
		return EVT_ALG_RET::ALG_ERR_BAD_INPUT;
	}

	if (CGImagePRNU40.size() != CGImagePRNU80.size())
	{
		return EVT_ALG_RET::ALG_ERR_BAD_INPUT;
	}

	ImgCondition_CG condition;
	auto s = condition.FrameCount * pixelHeight * pixelWidth;
	if (s != CGImagePRNU80.size())
	{
		return EVT_ALG_RET::ALG_ERR_BAD_INPUT;
	}

	ofstream out("2H.txt", ios::trunc);
	out.close();

	ImgResult_CG algorithmCGResult;
	CGResult CGImageResult;
	try
	{
		TestCG(&CGImagePRNU40[0], &CGImagePRNU80[0], pixelHeight, pixelWidth, condition, &algorithmCGResult);
		CGResultConvert(algorithmCGResult, CGImageResult);
	}
	catch (cv::Exception& e)
	{
		time_t currentTime = time(0);
		tm ctm;
		localtime_s(&ctm, &currentTime);
		char stime[256];
		memset(stime, 0, 256);
		std::strftime(stime, 256, "%Y%m%d%H%M%S", &ctm);
		{
			std::string failRawName = "FailImage40";
			failRawName = failRawName + __FUNCDNAME__ + stime + ".raw";
			std::ofstream ofs(failRawName, ios::trunc | ios::binary);
			ofs.write((char*)&CGImagePRNU40[0], CGImagePRNU40.size() * 2);
			ofs.close();
		}
		{
			std::string failRawName = "FailImage80";
			failRawName = failRawName + __FUNCDNAME__ + stime + ".raw";
			std::ofstream ofs(failRawName, ios::trunc | ios::binary);
			ofs.write((char*)&CGImagePRNU80[0], CGImagePRNU80.size() * 2);
			ofs.close();
		}
	}

	ImageResult result;
	strcpy_s(result.testName, "CG");

	memcpy(result.pData, &CGImageResult, sizeof(CGImageResult));
	gv_ImageResults.push_back(result);

	CGImagePRNU40.clear();
	CGImagePRNU80.clear();
	return ALG_SUCCESS;
}

EVT_ALG_RET FWC(
	unsigned short *rawData,
	unsigned char *frameData,
	int frameWidth,
	int frameHeight,
	int frameCnt,
	EVT_IMG_FORMAT frameFormat,
	int pixelDepth,
	unsigned int siteNum,
	SaveImageCallBack saveCallBack,
	int argc,
	char** argv
)
{
	EVT_ALG_RET ret = ALG_SUCCESS;


	ImageResult result;
	strcpy_s(result.testName, "FWC");

	cv::Mat r_RawData;
	cv::Mat srcImage = cv::Mat(pixelHeight, pixelWidth, CV_16UC1, rawData);
	//cv::Rect cvImage(0, 0, pixelWidth, pixelHeight - 80);
	cv::Rect cvImage(214, 70, 50, 50);
	srcImage(cvImage).copyTo(r_RawData);

	FWCResult FWC_res;

	ImgCondition_Light imageCondition;
	
	ImageResult_FWC  Algori_FWC;
	FWCTest((unsigned short*)r_RawData.data, 50, 50, imageCondition, &Algori_FWC);

	FWCResultConvert(Algori_FWC, FWC_res);

	strcpy_s(result.testName, "FWC");
	
	memcpy(result.pData, &FWC_res, sizeof(FWC_res));
	gv_ImageResults.push_back(result);

	bool savingStatus = false;
	saveCallBack(savingStatus, false, 0, nullptr);
	return ret;
}

EVT_ALG_RET Light(
	unsigned short *rawData,
	unsigned char *frameData,
	int frameWidth,
	int frameHeight,
	int frameCnt,
	EVT_IMG_FORMAT frameFormat,
	int pixelDepth,
	unsigned int siteNum,
	SaveImageCallBack saveCallBack,
	int argc,
	char** argv
)
{
	EVT_ALG_RET ret = ALG_SUCCESS;

	ImageResult result;
	strcpy_s(result.testName, "Light");

	cv::Mat r_RawData;
	cv::Mat srcImage = cv::Mat(pixelHeight, pixelWidth, CV_16UC1, rawData);
	cv::Rect cvImage(0, 0, pixelWidth, pixelHeight - 80);
	//cv::Rect cvImage(214, 70, 50, 50);
	srcImage(cvImage).copyTo(r_RawData);

	LightImgResult lightImageResult;
	try
	{
		ImgCondition_Light imageCondition;
		ImgResult_Light algorithmLightImageResult;
		TestLightImage((unsigned short*)r_RawData.data, pixelHeight - 80, pixelWidth, imageCondition, &algorithmLightImageResult);
		//TestLightImage((unsigned short*)r_RawData.data, 50, 50, imageCondition, &algorithmLightImageResult);
		LightResultConvert(algorithmLightImageResult, lightImageResult);

	}
	catch (cv::Exception& e)
	{
		time_t currentTime = time(0);
		tm ctm;
		localtime_s(&ctm, &currentTime);
		char stime[256];
		memset(stime, 0, 256);
		std::strftime(stime, 256, "%Y%m%d%H%M%S", &ctm);
		std::string failRawName = "FailImage";
		failRawName = failRawName + __FUNCDNAME__ + stime + ".raw";
		std::ofstream ofs(failRawName, ios::trunc | ios::binary);
		ofs.write((char*)rawData, pixelHeight * pixelWidth * 2);
		ofs.close();
	}

	memcpy(result.pData, &lightImageResult, sizeof(lightImageResult));
	gv_ImageResults.push_back(result);

	bool savingStatus = false;
	saveCallBack(savingStatus, false, 0, nullptr);

	return ret;
}

EVT_ALG_RET  TGP_ColorBar(
	unsigned short *rawData,
	unsigned char *frameData,
	int frameWidth,
	int frameHeight,
	int frameCnt,
	EVT_IMG_FORMAT frameFormat,
	int pixelDepth,
	unsigned int siteNum,
	SaveImageCallBack saveCallBack,
	int argc,
	char** argv)
{
	EVT_ALG_RET ret = ALG_SUCCESS;
	bool bRet = true;
	if (NULL == g_pBuf)
	{
		return ALG_ERR_BAD_ALLOC;
	}

	unsigned int errorPixelCount = 0;

	std::string rawFilePath = "Image\\TGP_ColorBar\\TGP_ColorBar.raw";
	std::ifstream ReadRowFile(rawFilePath, std::ios::binary);

	ReadRowFile.read((char*)(g_pBuf), sizeof(unsigned short) * frameWidth * frameHeight);
	ReadRowFile.close();

	//int32_t *tempBuf = new int32_t[];

	for (unsigned int i = 0; i < static_cast<unsigned int>(frameHeight * frameWidth); i++)
	{
		if (g_pBuf[i] != rawData[i])
		{
			errorPixelCount++;
		}
	}

	if (errorPixelCount > 0)
	{
		std::string rawSavePath = "Image\\TGP_ColorBar\\Fail_TGP_ColorBar\\TGP_ColorBar_Site" + std::to_string(siteNum) + "_Cnt" + std::to_string(Cnt) + ".raw";
		fs::path rawPath(rawSavePath);
		fs::create_directories(rawPath.remove_filename());
		std::ofstream WriteRawFile(rawSavePath, std::ios::trunc | std::ios::binary);
		WriteRawFile.write((char*)(rawData), frameWidth * frameHeight * sizeof(unsigned short));
		WriteRawFile.close();
	}
	Cnt++;


	bool savingStatus = (errorPixelCount > 0) ? false : true;
	saveCallBack(savingStatus, false, 0, nullptr);

	if (errorPixelCount > 0)
		return ALG_ERR_GENERAL;
	else
		return ALG_SUCCESS;

}

EVT_ALG_RET GetTestResult(
	unsigned int rsrcID,
	unsigned int siteNum,
	int argc,
	char** argv
)
{
	//::Sleep(3000);
	unsigned char *buf = (unsigned char*)gDMAAddr;
	buf += (((siteNum & 0x1) == 0) ? 0 : 0x2000000);
	int offset = sizeof(int);
	for (auto &r : gv_ImageResults)
	{
		memcpy(buf + offset, &r, sizeof(r));
		offset += sizeof(r);
	}
	*(int*)buf = offset;

	gv_ImageResults.clear();
	return ALG_SUCCESS;
}

static inline void ImageSum(unsigned short* pImage, unsigned int imageSize, unsigned int frameCnt)
{
    for (int cnt = 1; cnt < frameCnt; cnt++)
    {
        for (int imageIndex = 0; imageIndex < imageSize; imageIndex++)
        {
            pImage[imageIndex] += pImage[imageSize*cnt + imageIndex];
        }
    }
}

static inline unsigned short G2B(unsigned short gray)
{
    gray &= 0x03FF;
    unsigned short binary = gray;
    while (gray >>= 1)
        binary ^= gray;
    return binary;
}

EVT_ALG_RET preprocess(
    unsigned char* pImageBuffer, 
    uint16_t* width, 
    uint16_t* height, 
    unsigned int* frameCount,
    uint8_t* average,
    const char* testName
)
{
    unsigned short* pUShortImageBuffer = (unsigned short*)pImageBuffer;
    int imageHeight = (*height);
    int imageWidth = (*width);
    int imageFrameCount = (*frameCount);
    int imageAverage = (*average);
    std::string strTestName(testName);

    if (bConvertBinaryCode == true)
    {
        // 格雷码转换
        size_t ushortBufferSize = imageWidth * imageHeight * imageFrameCount;
        for (int index = 0; index < ushortBufferSize; index++)
        {
			if (pUShortImageBuffer[index] == 1023)
				continue;
			else
				pUShortImageBuffer[index] = G2B(pUShortImageBuffer[index]);
        }
    }

    static const std::vector<std::string> DontAverageImageName{"DarkFPN", "PRNU40", "PRNU80","CG_PRNU40","CG_PRNU80","TempNoise"}; // 不做累加平均的图像
	bool bFind = false;
	for (int i = 0; i < DontAverageImageName.size(); i++)
	{
		if (DontAverageImageName.at(i) == strTestName)
		{
			bFind = true;
		}
	}

    if (!bFind)
    {
        // 做图像的累加
        ImageSum(pUShortImageBuffer, imageHeight * imageWidth, imageFrameCount);
        // 将平均丢给CISTool做
        *average = imageFrameCount;
        *frameCount = 1;
    }
    else
    {
        // CISTool不做平均
        *average = 1;
    }

    return ALG_SUCCESS;
}
