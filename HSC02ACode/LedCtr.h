#pragma once
#include "FT60xLedController.h"
#include <vector>

class LedCtr
{
public:
	LedCtr();
	~LedCtr();

	void Close();
	void SetCctLux(unsigned int siteNum, unsigned short value);
	void SetCctLux(const std::vector<unsigned int> vSites, unsigned short value);
	void ReadValue(unsigned int usbIndex, unsigned char* pData, size_t len);
	// 一个电磁阀控制8site，因此多个active site只需要传入其中一个即可
	void CtrlPump(unsigned int site, bool isOpen);
private:
	void WriteUSB(const FT_LightCtrlCmd& cmd, uint32_t index);

private:
	Ft60xLedControl controller;
	unsigned int m_usbDevicesNum;
	bool bInit;
};
