#include "stdafx.h"
#include "LedCtr.h"
#include"HSC02A.h"


LedCtr::LedCtr()
	: controller()
	, m_usbDevicesNum(0)
	, bInit(false)
{
	m_usbDevicesNum = controller.GetDeviceNumbers();
	if (m_usbDevicesNum == 0)
	{
		EVTDATA("No device found.\r\n");
		return;
	}
	else
	{
		if (!controller.Open())
		{
			controller.Close();
			return;
		}
	}

	bInit = true;
}

LedCtr::~LedCtr()
{
	if (bInit)
	{
		controller.Close();
	}
}

void LedCtr::Close()
{
	if (bInit)
	{
		std::array<unsigned char, 8> closeCmd{ 0xa5, 0x10, 0x21, 0xfc, 0x62, 0x00,0x00,0x00 };

		closeCmd[5] = 0x10;
		closeCmd[6] = HB(0u);
		closeCmd[7] = LB(0u);
		WriteUSB(closeCmd, 0);

		ATE()->SleepMs(5);

		closeCmd[5] = 0x11;
		closeCmd[6] = HB(0u);
		closeCmd[7] = LB(0u);
		WriteUSB(closeCmd, 1);
	}
}

void LedCtr::SetCctLux(unsigned int siteNum, unsigned short value)
{
	if (bInit)
	{
		//A5 10 21 FC 62      00       07 D0
		std::array<unsigned char, 8> sendData{0xa5, 0x10, 0x21, 0xfc, 0x62, 0x00,0x00,0x00};
		sendData[5] = siteNum % 8;
		sendData[6] = HB(value);
		sendData[7] = LB(value);;
		WriteUSB(sendData, siteNum / 8);
		//EVTSYS(LEVEL_ERROR, "SetCctLux for siteNum=%d led=0x%x\n", siteNum, value);
		Sleep(15);
	}
}

void LedCtr::SetCctLux(const std::vector<unsigned int> vSites, unsigned short value)
{
	if (bInit)
	{
		for (auto &site : vSites)
		{
			SetCctLux(site, value);
		}
	}
}

void LedCtr::ReadValue(unsigned int usbIndex, unsigned char* pData, size_t len)
{
	if (bInit)
	{
		// 光源有两个版本:
		// 大板子: 一次接16site，此时usbIndex都是0
		// 小板子: 分成两个，每个接8site，usbIndex 0或者1
		if (1 == m_usbDevicesNum)
		{
			// 这个if条件适配大板子的情况
			usbIndex = 0;
		}

		controller.Read(pData, len, usbIndex);
	}
}

void LedCtr::WriteUSB(const FT_LightCtrlCmd& cmd, uint32_t index)
{
	if (!bInit || (m_usbDevicesNum == 0))
		return;

	controller.Write((void*)cmd.data(), cmd.size(), index);
	//for (unsigned int usbIdx = 0; usbIdx < m_usbDevicesNum; ++usbIdx)
	//{
	//	
	//}
}
