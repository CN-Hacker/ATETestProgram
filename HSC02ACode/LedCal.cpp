#include "stdafx.h"
#include <boost/algorithm/string.hpp>
#include <boost/filesystem.hpp>
#include <boost/property_tree/ptree.hpp>
#include <boost/property_tree/ini_parser.hpp>
#include "LedCal.h"
#include "LedCtr.h"
#include "FT60xLedController.h"
#include "TestExec\EvtATELib\EvtATE.h"
#include "HSC02A.h"
#include "CISImageInterface.h"
#include "LightMap.h"

#define CALLED_ROI


//unsigned int g_siteIndex[MAX_SITES];
void ExtractPixel(int rows, int cols,  unsigned short* rawdata, vector<unsigned short>& RBuf, vector<unsigned short>& GrBuf, vector<unsigned short>& GbBuf, vector<unsigned short>& BBuf)
{
	
	for (int row = 0; row < rows-80 ; ++row)
	{
		unsigned short* startRow = &rawdata[row * cols];
		
		for (int col = 0; col < cols ; ++col)
		{
			if ((row % 2) == 0)
			{
				if ((col % 2) == 0)
				{
					
					BBuf[(row / 2)*(cols / 2) + col / 2] = startRow[col];
				}
				else
				{
					GbBuf[(row / 2)*(cols / 2) + col / 2] = startRow[col];
				}
			}
			else
			{
				if ((col % 2) == 0)
				{
					GrBuf[(row / 2)*(cols / 2) + col / 2] = startRow[col];
				}
				else {
					RBuf[(row / 2)*(cols / 2) + col / 2] = startRow[col];
				}
			}
		}
	}
}

unsigned short Gray2Bcd_A(unsigned short gray)
{
	gray &= 0x03FF;
	unsigned short binary = gray;
	while (gray >>= 1)
		binary ^= gray;
	return binary;
}

LedCal::LedCal(LedCtr& controller)
	: m_ledCtrl(controller)
{
}


LedCal::~LedCal()
{
}

bool LedCal::ReadLedCalFile()
{
	try
	{
		boost::property_tree::ptree pt;
		boost::property_tree::ini_parser::read_ini("Configure\\ImageConfig.ini", pt);

		m_LedMax = pt.get<unsigned int>("AllSite.LedMax");
		m_LedMin = pt.get<unsigned int>("AllSite.LedMin");
		m_FWCstep = pt.get<unsigned int>("AllSite.FWCstep");
	}
	catch (std::exception &e)
	{
		EVTDATA("%s", e.what());
		return false;
	}
	return true;
}

void LedCal::SetCalEnable(bool flag)
{
	m_CalFlag = flag;
}

std::map<unsigned int, unsigned short> LedCal::GetValue(std::string testName)
{
	std::map<unsigned int, unsigned short> resMap;
	try
	{
		boost::property_tree::ptree pt;
		boost::property_tree::ini_parser::read_ini("Configure\\LedCal.ini", pt);
		for (unsigned int site = 0; site < MAX_SITES; site++)
		{
			std::string tag = testName + "." + std::to_string(site);
			resMap[site] = pt.get<unsigned short>(tag, 0);
		}
	}
	catch (std::exception &e)
	{
		EVTDATA("%s\r\n", e.what());
	}
	return resMap;
}


bool LedCal::CalReadDVPImage(
	std::string testName,
	unsigned int siteNum,
	unsigned short imageWidth,
	unsigned short imageHeight,
	unsigned char frameValidCnt,
	double &pixelAvg,
	bool gray)
{
	vector<unsigned int> vSites;
	ATE()->Site()->GetActive(vSites);
	bool bRet = true;
	do
	{
		std::vector<AddrInfo> addr;
		ATE()->HWInterface()->GetDMABuffer(vSites[0], addr);
		if (!addr.empty())
		{
			
			unsigned char *buf = (unsigned char*)(addr[0].bufAddr += vSites[0] & 0x1 ? 0x2000000 : 0);

			unsigned short *sumOfImage = reinterpret_cast<unsigned short*>(buf);
			if (gray == true)
			{
				for (int index = 0; index < imageWidth*imageHeight; index++)
				{
					if (sumOfImage[index]==1023)
						continue;
					else
						sumOfImage[index] = Gray2Bcd_A(sumOfImage[index]);
					
				}
			}

			/*ofstream outFile("D:/full.raw", ios::out | ios::binary);

			for (int i = 0; i < imageWidth * imageHeight; i++)
			{
				unsigned char b1 = sumOfImage[i] >> 8;
				unsigned char b2 = sumOfImage[i] &  0xff;
				
				outFile.write((char*)b2, 1);
				outFile.write((char*)b1, 1);
			}
			outFile.close();*/

			unsigned short m_imageWidth = imageWidth / 2;
			unsigned short m_imageHeight = (imageHeight-80) / 2;
			double Gr_Sum = 0;

#ifdef CALLED_ROI
			int sx = 214;
			int sy = 70;
			int roi_size = 50;
			unsigned short* roi_data = (unsigned short*)malloc(roi_size * roi_size * sizeof(unsigned short));
			
			int curr = 0;
			for (int i = sy; i < sy + roi_size; i++)
			{
				for (int j = sx; j < sx + roi_size; j++)
				{
					roi_data[curr] = sumOfImage[i * imageWidth + j];
					if ( (i & 1) && !(j&1))
					{
						Gr_Sum += roi_data[curr];
					}
					curr++;
				}
			}


			m_imageWidth = roi_size/2;
			m_imageHeight = roi_size/2;



			
			/*outFile.open("D:/roi.raw", ios::out | ios::binary);

			for (int i = 0; i < 50 * 50; i++)
			{
				unsigned char b1 = roi_data[i] >> 8;
				unsigned char b2 = roi_data[i] & 0xff;

				outFile.write((char*)b2, 1);
				outFile.write((char*)b1, 1);
			}
			outFile.close();*/

#else

			vector<unsigned short> g_RBuf(m_imageWidth*m_imageHeight, 0);
			vector<unsigned short> g_GrBuf(m_imageWidth*m_imageHeight, 0);
			vector<unsigned short> g_GbBuf(m_imageWidth*m_imageHeight, 0);
			vector<unsigned short> g_BBuf(m_imageWidth*m_imageHeight, 0);

			// ROI
			// /*
			

			
			// */

			ExtractPixel(imageHeight, imageWidth, sumOfImage, g_RBuf, g_GrBuf, g_GbBuf, g_BBuf);

			double R_Sum = 0;
			
			double Gb_Sum = 0;
			double B_Sum = 0;


			for (vector<unsigned short>::iterator it = g_GrBuf.begin(); it != g_GrBuf.end(); it++)
			{
				Gr_Sum += *it;
			}
			
#endif
			pixelAvg = Gr_Sum /( (m_imageWidth*m_imageHeight)*frameValidCnt);

		}
		else
			return false;
	} while (0);
	return bRet;
}

bool LedCal::LedTestReadDVPImage(
	std::string testName,
	unsigned int siteNum,
	unsigned short imageWidth,
	unsigned short imageHeight,
	unsigned char frameValidCnt,
	double &RpixelAvg,
	double &GrpixelAvg,
	double &GbpixelAvg,
	double &BpixelAvg,
	bool gray)
{
	vector<unsigned int> vSites;
	ATE()->Site()->GetActive(vSites);
	bool bRet = true;
	do
	{
		std::vector<AddrInfo> addr;
		ATE()->HWInterface()->GetDMABuffer(vSites[0], addr);
		if (!addr.empty())
		{

			unsigned char *buf = (unsigned char*)(addr[0].bufAddr += vSites[0] & 0x1 ? 0x2000000 : 0);

			unsigned short *sumOfImage = reinterpret_cast<unsigned short*>(buf);
			if (gray == true)
			{
				for (int index = 0; index < imageWidth*imageHeight; index++)
				{
					sumOfImage[index] = Gray2Bcd_A(sumOfImage[index]);
				}
			}
			unsigned short m_imageWidth = imageWidth / 2;
			unsigned short m_imageHeight = (imageHeight - 80) / 2;

			vector<unsigned short> g_RBuf(m_imageWidth*m_imageHeight, 0);
			vector<unsigned short> g_GrBuf(m_imageWidth*m_imageHeight, 0);
			vector<unsigned short> g_GbBuf(m_imageWidth*m_imageHeight, 0);
			vector<unsigned short> g_BBuf(m_imageWidth*m_imageHeight, 0);

			ExtractPixel(imageHeight, imageWidth, sumOfImage, g_RBuf, g_GrBuf, g_GbBuf, g_BBuf);
			unsigned long R_Sum = 0;
			unsigned long Gr_Sum = 0;
			unsigned long Gb_Sum = 0;
			unsigned long B_Sum = 0;
			for (vector<unsigned short>::iterator it = g_RBuf.begin(); it != g_RBuf.end(); it++)
			{
				R_Sum += *it;
			}

			for (vector<unsigned short>::iterator it = g_GrBuf.begin(); it != g_GrBuf.end(); it++)
			{
				Gr_Sum += *it;
			}

			for (vector<unsigned short>::iterator it = g_GbBuf.begin(); it != g_GbBuf.end(); it++)
			{
				Gb_Sum += *it;
			}

			for (vector<unsigned short>::iterator it = g_BBuf.begin(); it != g_BBuf.end(); it++)
			{
				B_Sum += *it;
			}
			RpixelAvg = R_Sum / ((m_imageWidth*m_imageHeight)*frameValidCnt);
			GrpixelAvg= Gr_Sum / ((m_imageWidth*m_imageHeight)*frameValidCnt);
			GbpixelAvg = Gb_Sum / ((m_imageWidth*m_imageHeight)*frameValidCnt);
			BpixelAvg = B_Sum / ((m_imageWidth*m_imageHeight)*frameValidCnt);
		}
		else
			return false;
	} while (0);
	return bRet;
}


bool LedCal::CalDVPImageAcquireConfig(
	Test_Result &result,	//add by GX
	std::string testName,	//add by GX
	EVT_IMG::EVT_IMG_DATAWIDTH dataWidth,
	double &pixelAvg,
	unsigned char imageAverage,
	unsigned short imageWidth,
	unsigned short imageHeight,
	unsigned short timeout,
	bool isOneSegmentTwoSite,
	bool gray)	//add by GX
{
	//add by GX
	bool bRet = true;
	vector<unsigned int> vSites;
	ATE()->Site()->GetActive(vSites);
	unsigned int site = vSites[0];
	//新增返回值，修复不出图也继续执行的问题
	bRet = CISImage()->DVPImageAcquireConfig(result, testName.c_str(), dataWidth, imageAverage, true, imageWidth, imageHeight, timeout, isOneSegmentTwoSite);
	if (!bRet)
	{
		EVTSYS(LEVEL_ERROR, "Site%d DVP Capture Image Fail.\r\n", site);
		return false;
	}

	bRet = LedCal::CalReadDVPImage(
		testName,
		site,
		imageWidth,
		imageHeight,
		imageAverage, 	//modify from frameValidCnt to imageAverage,need confirm modify by GX
		pixelAvg,
		gray);

	if (!bRet)
	{
		EVTSYS(LEVEL_ERROR, "Site%d Read DVP Image Fail.\r\n", site);
		return false;
	}

	return true;
}

bool LedCal::CalStartDVP(
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
	bool gray)	//add by GX
{
	if (!m_CalFlag)
		return true;

	if (LedSiteMap.count(siteNum) == 0)
	{
		EVTDATA("SiteNum=%d is error!\n", siteNum);
		return false;
	}

	bool isStep = false;
	bool status = true;
	EVTDATA("SiteNum = %d, test name = %s start!!!!!\n", siteNum, testName.c_str());

	if ((m_LedMax < m_LedMin) || (m_LedMax == 0))
	{
		EVTDATA("Capture image = %s fail, Invalid led lux value.\n", testName.c_str());
		return false;
	}

	{
		m_ledCtrl.SetCctLux(LedSiteMap[siteNum], m_LedMax);
		ATE()->SleepMs(100);
		double maxRes = 0;

		if (!CalDVPImageAcquireConfig(result, testName, dataWidth, maxRes,imageAverage, imageWidth, imageHeight, 2000, isOneSegmentTwoSite,gray))		//Modify by GX
		{
			EVTDATA("Capture image = %s fail, no image.\n", testName.c_str());
			return false;
		}
		else
		{
			EVTDATA("Site = %d, lux = %d, Gr = %f.\n", siteNum, m_LedMax, maxRes);
		}

		if (maxRes < tarVal)
		{
			EVTDATA("Capture image = %s fail, max result lower than target value.\n", testName.c_str());
			return false;
		}

		m_ledCtrl.SetCctLux(LedSiteMap[siteNum], m_LedMin);
		ATE()->SleepMs(100);
		double minRes = 0;


		if (!CalDVPImageAcquireConfig(result, testName, dataWidth, minRes, imageAverage, imageWidth, imageHeight, 2000, isOneSegmentTwoSite,gray))
		{
			EVTDATA("Capture image = %s fail, no image.\n", testName.c_str());
			return false;
		}
		else
		{
			EVTDATA("Site = %d, lux = %d, Gr = %f.\n", siteNum, m_LedMin, minRes);
		}

		if (minRes > tarVal)
		{
			EVTDATA("Capture image = %s fail, min result higher than target value.\n", testName.c_str());
			return false;
		}
	}

	std::map<int, int> average2Lux;

	int lowLux = m_LedMin;
	int highLux = m_LedMax;
	while (lowLux <= highLux)
	{
		int midLux = lowLux + (highLux - lowLux) / 2;
		m_ledCtrl.SetCctLux(LedSiteMap[siteNum], midLux);
		ATE()->SleepMs(400);

		double pixelAverage = 0.0;
		if (!CalDVPImageAcquireConfig(
			result,
			testName,
			dataWidth,
			pixelAverage,
			imageAverage,
			imageWidth,
			imageHeight,
			2000,
			isOneSegmentTwoSite,
			gray))		//Modify by GX
		{
			EVTDATA("Capture image = %s fail, no image.\n", testName.c_str());
			return false;
		}
		else
		{
			EVTDATA("Site = %d, lux = %d, Gr = %f.\n", siteNum, midLux, pixelAverage);
			average2Lux[std::abs(pixelAverage - tarVal)] = midLux;
		}
		if ((midLux == lowLux) && (midLux = highLux))
		{
			CalSetValue(testName, siteNum, midLux);
			EVTDATA("SiteNum = %d, LuxValue = %d, Capture image = %s Finish!!!!!\n", siteNum, midLux, testName.c_str());
			isStep = true;
			//return true;
			break;
		}

		if (pixelAverage - tarVal > 5)
			//highLux = midLux - 1;
			highLux = midLux ;
		else if (pixelAverage - tarVal < -5)
			//lowLux = midLux + 1;
			lowLux = midLux ;
		else
		{
			// calibration finish. midLux is the target.
			CalSetValue(testName, siteNum, midLux);
			EVTDATA("SiteNum = %d, LuxValue = %d, Capture image = %s Finish!!!!!\n", siteNum, midLux, testName.c_str());
			return true;
		}

	}
	
	if (isStep)
	{
		return true;
	}


	if (average2Lux.size())
	{
		CalSetValue(testName, siteNum, average2Lux.begin()->second);
		EVTDATA("SiteNum = %d, LuxValue = %d, Capture image = %s Finish!!!!!\n", siteNum, average2Lux.begin()->second, testName.c_str());
	}
	else
	{
		CalSetValue(testName, siteNum, lowLux);
		EVTDATA("SiteNum = %d, LuxValue = %d, Capture image = %s Finish!!!!!\n", siteNum, lowLux, testName.c_str());
	}

	
	return true;
}

bool LedCal::StartFWC(
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
	bool gray)	//add by GX
{
	//std::map<int, double> Lux_PixelValue;
	vector<int> Lux;
	vector<double>PixelValue;
	int lowLux = m_LedMin;
	int highLux = m_LedMax;
	int currentLux = lowLux;


	if (LedSiteMap.count(siteNum) == 0)
	{
		EVTDATA("SiteNum=%d is error!\n", siteNum);
		return false;
	}

	bool status = true;
	EVTDATA("SiteNum = %d, test name = %s start!!!!!\n", siteNum, testName.c_str());

	if ((m_LedMax < m_LedMin) || (m_LedMax == 0))
	{
		EVTDATA("Capture image = %s fail, Invalid led lux value.\n", testName.c_str());
		return false;
	}

	while (currentLux <= highLux)
	{
		
		m_ledCtrl.SetCctLux(LedSiteMap[siteNum], currentLux);
		ATE()->SleepMs(120);

		double pixelAverage = 0.0;
		if (!CalDVPImageAcquireConfig(
			result,
			testName,
			dataWidth,
			pixelAverage,
			1,
			imageWidth,
			imageHeight,
			2000,
			isOneSegmentTwoSite,
			gray))		
		{
			EVTDATA("Capture image = %s fail, no image.\n", testName.c_str());
			return false;
		}
		else
		{
			EVTDATA("Site = %d, lux = %d, Gr = %f.\n", siteNum, currentLux, pixelAverage);
			Lux.push_back(currentLux);
			PixelValue.push_back(pixelAverage);
		}
		currentLux = currentLux + m_FWCstep;
	}




	vector<double> k;
	vector<double> tend;
	for (int i = 0; i < Lux.size()-1; i++)
	{
		k.push_back((PixelValue[i + 1] - PixelValue[i]) / (Lux[i + 1] - Lux[i]));
	}
	for (int j = 0; j < k.size() - 1; j++)
	{
		tend.push_back((k[j]- k[j+1]));
	}

	int maxPosition = max_element(tend.begin(), tend.end()) - tend.begin();
	EVTDATA("Site=%d, FWC_lux=%d\n", siteNum, 2*Lux[maxPosition + 1]);
	CalSetValue(testName, siteNum, 2*Lux[maxPosition + 1]);
	return true;
}


void LedCal::CalFinish(unsigned int siteNum)
{
	if (m_CalFlag)
	{
		m_ledCtrl.SetCctLux(LedSiteMap[siteNum], 0);
	}
}

void LedCal::CalSetValue(std::string testName, unsigned int siteNum, unsigned short lux)
{
	std::unique_lock<std::mutex> lk(m_mutex);
	try
	{
		boost::property_tree::ptree pt;
		boost::property_tree::ini_parser::read_ini("Configure\\LedCal.ini", pt);
		std::string tag = testName + "." + std::to_string(siteNum);
		pt.put<unsigned short>(tag, lux);
		boost::property_tree::ini_parser::write_ini("Configure\\LedCal.ini", pt);
	}
	catch (std::exception &e)
	{
		EVTDATA("%s\r\n", e.what());
		return;
	}
	return;
}

void LedCal::LedTestSetValue(std::string testName, unsigned int siteNum, unsigned short lux)
{
	std::unique_lock<std::mutex> lk(m_mutex);
	try
	{
		boost::property_tree::ptree pt;
		boost::property_tree::ini_parser::read_ini("Configure\\LedTest.ini", pt);
		std::string tag = testName + "." + std::to_string(siteNum);
		pt.put<unsigned short>(tag, lux);
		boost::property_tree::ini_parser::write_ini("Configure\\LedTest.ini", pt);
	}
	catch (std::exception &e)
	{
		EVTDATA("%s\r\n", e.what());
		return;
	}
	return;
}