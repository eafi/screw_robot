#pragma once
#include <MvCameraControl.h>
#include "Register.h"

class Hik
{
public:
	Hik(unsigned exposureTime);
	~Hik();
	cv::Mat get_one_frame();

private:
	Hik();
	int nRet = MV_OK;
	void *handle = NULL;
	unsigned int g_nPayloadSize = 0;

	bool PrintDevice(MV_CC_DEVICE_INFO *pstMVDevInfo);

	bool Conver2Mat(MV_FRAME_OUT_INFO_EX* pstImageInfo, unsigned char *pData,cv::Mat &_img);
};

