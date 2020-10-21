#include "Hik.h"
#include <Windows.h>
#include <process.h>
#include <conio.h>


Hik::Hik(unsigned exposureTime)
{

		MV_CC_DEVICE_INFO_LIST stDeviceList;
		memset(&stDeviceList, 0, sizeof(MV_CC_DEVICE_INFO_LIST));
		nRet = MV_CC_EnumDevices(MV_GIGE_DEVICE | MV_USB_DEVICE, &stDeviceList);

		if(MV_OK != nRet)
		{
			std::cout << "Enum Devices fail! " << std::endl;
		}

		if(stDeviceList.nDeviceNum > 0)
		{
			for(unsigned int i=0;i<stDeviceList.nDeviceNum;++i)
			{
				std::cout << "[device] :" << i << std::endl;
				MV_CC_DEVICE_INFO *pDeviceInfo = stDeviceList.pDeviceInfo[i];
				if (nullptr == pDeviceInfo)
					break;
				PrintDevice(pDeviceInfo);
			}
		}
		else
		{
			std::cout << "Find No Devices!" << std::endl;
		}

		nRet = MV_CC_CreateHandle(&handle, stDeviceList.pDeviceInfo[0]);
		if(MV_OK != nRet)
		{
			std::cout << "Create Handle fail!" << std::endl;
		}

		nRet = MV_CC_OpenDevice(handle);
		if(MV_OK != nRet)
		{
			std::cout << "Open device fail!" << std::endl;
		}

		nRet = MV_CC_SetEnumValue(handle, "TriggerMode", 0);
		if(MV_OK != nRet)
		{
			std::cout << "Set Trigger Mode fail" << std::endl;
		}

		MVCC_INTVALUE stParam;
		memset(&stParam, 0, sizeof(MVCC_INTVALUE));
		nRet = MV_CC_GetIntValue(handle, "PayloadSize", &stParam);
		if(MV_OK != nRet)
		{
			std::cout << "Get payloadsize fail!" << std::endl;
		}
		g_nPayloadSize = stParam.nCurValue;

		nRet = MV_CC_StartGrabbing(handle);

		if(MV_OK != nRet)
		{
			std::cout << "Start Grabbing fail" << std::endl;
		}

		//MV_CC_SetExposureAutoMode(handle, MV_EXPOSURE_AUTO_MODE_ONCE);
		MV_CC_SetExposureTime(handle, exposureTime);

	//	MV_CC_SetWidth(handle, 2592 * 0.5);
	//	MV_CC_SetHeight(handle, 2048 * 0.5);

}


Hik::~Hik()
{
	nRet = MV_CC_StopGrabbing(handle);
	if (MV_OK != nRet)
	{
		printf("Stop Grabbing fail! nRet [0x%x]\n", nRet);
	}

	// Close device
	nRet = MV_CC_CloseDevice(handle);
	if (MV_OK != nRet)
	{
		printf("ClosDevice fail! nRet [0x%x]\n", nRet);
	}

	// Destroy handle
	nRet = MV_CC_DestroyHandle(handle);
	if (MV_OK != nRet)
	{
		printf("Destroy Handle fail! nRet [0x%x]\n", nRet);
	}

}

cv::Mat Hik::get_one_frame()
{
	cv::Mat aFrame;
	MV_FRAME_OUT_INFO_EX stImageInfo = { 0, };
	memset(&stImageInfo, 0, sizeof(MV_FRAME_OUT_INFO_EX));
	unsigned char *pData = (unsigned char *)malloc(sizeof(unsigned char)*(g_nPayloadSize));
	if (pData == NULL)
	{
		std::cout << "Allocate memory fail." << std::endl;
	}

	nRet = MV_CC_GetOneFrameTimeout(handle, pData, g_nPayloadSize, &stImageInfo, 1000);

	if (nRet == MV_OK)
	{
		//std::cout << "Get one Frame" << std::endl;
	}
	else
	{
		std::cout << "No data" << std::endl;
		free(pData);
		pData = nullptr;
	}

	bool bConvertRet = false;
	bConvertRet = Conver2Mat(&stImageInfo, pData,aFrame);
	if (bConvertRet)
	{
	//	std::cout << "convert finished." << std::endl;
		free(pData);
		pData = nullptr;
	}
	return aFrame;

}

bool Hik::PrintDevice(MV_CC_DEVICE_INFO *pstMVDevInfo)
{
	if (NULL == pstMVDevInfo)
	{
		std::cout << "The Pointer of pstMVDevInfo is NULL\n";
		return false;
	}
	if (pstMVDevInfo->nTLayerType == MV_USB_DEVICE)
	{
		std::cout << "UserDefineName:" << pstMVDevInfo->SpecialInfo.stUsb3VInfo.chUserDefinedName << std::endl;
		std::cout << "Serial Number:" << pstMVDevInfo->SpecialInfo.stUsb3VInfo.chSerialNumber << std::endl;
	}
	else
	{
		std::cout << "Not support\n";
	}
	return true;
}


bool Hik::Conver2Mat(MV_FRAME_OUT_INFO_EX* pstImageInfo, unsigned char *pData,cv::Mat &_img)
{
	cv::Mat srcImage;
	if(pstImageInfo->enPixelType == PixelType_Gvsp_Mono8)
	{
		srcImage = cv::Mat(pstImageInfo->nHeight, pstImageInfo->nWidth, CV_8UC1, pData);
	}else
	{
		std::cout << "unsupported pixel format\n";
		return false;
	}

	if(nullptr == srcImage.data)
	{
		return false;
	}
	else
	{
		srcImage.copyTo(_img);
		return true;
	}
}


