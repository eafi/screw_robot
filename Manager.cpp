#include "Manager.h"

Manager::~Manager()
{
	delete ptrArduino;
	delete ptrCalibrator;
	delete ptrAnalyzer;
	delete ptrWhere;
	delete ptrHik;
}

void Manager::sys_run()
{
/*
 *		calibrator will select whether
 *		 calibrate camera[iCameraNum] or not by valuable 'bFileorCalib'.
 *		
 *		delete calibreator;2
 */
	std::vector<cv::Mat> mapVec;
	if (ptrCalibrator == nullptr)
	{
		ptrCalibrator = new Calibrator(6, 8);
		if (bFileorCalib)
		{
			mapVec = ptrCalibrator->intrinsic_cali(iCameraNum);
			if (mapVec.at(0).empty())
			{
				std::cout << "ERROR : intrinsic_cali returned wrong values." << std::endl;
				return;
			}
		}
		else
		{
			mapVec = ptrCalibrator->read_from_file();
		}
	}

	if(serial_init(iPortNum))
	{
		std::cout << "SYS_RUN : Connected port : " << iPortNum << std::endl;
	}else
	{
		std::cout << "SYS_RUN : Connecting failed in port : " << iPortNum << std::endl;
		//return;
	}

	/*
	 *	turn on the camera and start the main loop
	*/
	cv::VideoCapture capture(iCameraNum);
	if (ptrAnalyzer == nullptr)
		ptrAnalyzer = new Analyzer;

	POINTXYZ pos2(-48320,0, 0);
	POINTXYZ pos1(3200, 3200, 0);
//	posStack.push(pos2);
//	posStack.push(pos1);
//	for (int i = 0; i < 5; ++i)
//	{
//		POINTXYZ pos3(640, 640, 0);
//		posStack.push(pos3);
//	}
	std::string strPort = "\\\\.\\COM"+std::to_string(3);
	int i = 0;
	cv::Rect roi;
	cv::Point2d pcdPosInImg(.0, .0);
	Hik hik(50000);
	startTime = cv::getTickCount();
	//ptrWhere->setXYZ(160000, 16000, 0);
	while (1)
	{
		cv::Mat image, image0;
		image = hik.get_one_frame();
//		cv::remap(
//			image0,
//			image,
//			mapVec.at(0),
//			mapVec.at(1),
//			cv::INTER_LINEAR,
//			cv::BORDER_CONSTANT,
//			cv::Scalar()
//		);

		switch (machineState)
		{
		case RESET:
			init_machine(image);
	//		machineState = LOCATING;
			machineState = FIND;
			break;
		case BUSY:
		//	std::cout << "BUSY" << std::endl;
			break;
		case LOCATING:
			//locate_pcb();
			//ptrAnalyzer->analyze(image);
			std::cout << "LOCATING" << std::endl;
			break;
		case FIND:
			if (!roi.empty())
				find_pcb(image, roi, pcdPosInImg);
			break;
		}
			check_machine_state();
			if (canSend)
				send_pos();
//
//		
//
//		//POINTXYZ pos = ptrAnalyzer->analyze(image);
//	//	std::cout << "channels : " << image.channels() << std::endl;
//
//	//	ptrAnalyzer->pcb_locate(image);
//	//	ptrAnalyzer->cut_img_corner_edge(image, 0, 0);
//		//push_pos(pos);
//		//std::cout << "size of stack : " << posStack.size() << std::endl;
//
//
	//	roi = ptrWhere->update();
			roi = cv::Rect(0, 0, image.cols, image.rows);
	//	std::cout << "roi"<<roi<< std::endl;

		show_img(image, "IMG_VIEW",0.3);
	//	if(!roi.empty())
	//	cv::imshow("roi", image(roi));

	//	ptrWhere->setXYZ((poss[i++]));
		//if (i > 5) i = 0;
		if ((cv::waitKey(30) & 255) == 27) break;
	}
}

void Manager::check_machine_state()
{
	std::string strRead;
	if (get_msg(strRead, READ_BUF_SZ))
	{
		if (strRead == CHECK_STR)
	//	if(strRead.find(CHECK_STR))
		{
			//machineState = LOCATING;
			canSend = true;
			
		}
		else if(strRead == "")
		{
		//	machineState = FIND;
		}
	}
}


bool Manager::serial_init()
{
	return serial_init(iPortNum);
}

bool Manager::serial_init(unsigned portNum_)
{
	delete ptrArduino;
	std::string strPort = "\\\\.\\COM"+std::to_string(portNum_);
	ptrArduino = new SerialPort(strPort.c_str());
	return ptrArduino->isConnected();
}

bool Manager::get_msg(std::string& msg_,unsigned bufSize_)
{
	char cRead[MAX_DATA_LENGTH]={0,};
	if (ptrArduino->isConnected())
	{
		ptrArduino->readSerialPort(cRead, bufSize_);
		msg_ = cRead; 
		std::cout << "msg :\"" << msg_ <<"\""<< std::endl;
		return true;
	}
	return false;
}


bool Manager::send_msg(const std::string& msg_)
{
	std::cout << "Listening." << std::endl;
	char *c_string = new char[msg_.size() + 1];
	//copying the std::string to c string
	std::copy(msg_.begin(), msg_.end(), c_string);
	//Adding the delimiter
	c_string[msg_.size()] = '\n';
	ptrArduino->writeSerialPort(c_string, MAX_DATA_LENGTH);
	std::cout << "Send_MSG done." << std::endl;

	return true;
}

void Manager::push_pos(POINTXYZ pos)
{
	if (posEqual(pos, POINTXYZ(-1, -1, -1)))
		return;
	if(posStack.empty()||(!posStack.empty()&&!posEqual(posStack.top(),pos)))
	{
		posStack.push(pos);
	}
}

void Manager::send_pos()
{
	if (!posStack.empty())
	{
		std::cout << "stack size:" << posStack.size() << std::endl;
		ptrWhere->setXYZ(posStack.top());
		if (send_msg(ptrWhere->getXYZ_str()))
		{
			std::cout << "sending done!" << std::endl;
			canSend = false;
			posStack.pop();
		}
		else
		{
			std::cout << "sending failed." << std::endl;
		}
	}
}

/*
 *	1. clean position stack
 *	2. move machine to the origin
 */
void Manager::init_machine(const cv::Mat &img)
{
	while(!posStack.empty())
		posStack.pop();
//	posStack.push(POINTXYZ(0, 0, 0));

	ptrWhere->set_fovSize_scale(img.size(),0.38);

	ptrCalibrator->isExCali(true);
	if (!ptrCalibrator->isExCali())
	{
		ptrCalibrator->extrinsic_cali(img, cv::Size(4, 5), 13.3, 300.0);
		if (ptrCalibrator->isExCali())
		{
			//scale 确定，初始化where
			ptrWhere->set_fovSize_scale(img.size(), ptrCalibrator->get_scale());
		}
	}
//	machineState = FIND;
}

bool Manager::find_pcb(const cv::Mat &img,cv::Rect roi,cv::Point2d &pcdPos)
{
	while(!posStack.empty())
		posStack.pop();
	//posStack.push(POINTXYZ(ABS_X_HALF, ABS_Y_HALF,0));
	ptrAnalyzer->pcb_locate(img(roi),pcdPos,cv::Size(25,25),cv::RotatedRect());

	if(pcdPos != cv::Point2d(0.0,0.0))
	{
	//	std::cout << "pcdPos before:" << pcdPos << std::endl;
		pcdPos.x += roi.x;
		pcdPos.y += roi.y;
	//	std::cout << "pcdPos:" << pcdPos << std::endl;
		if(filter_points2stack(ptrWhere->cal_pos_loc2wor(pcdPos, img.size())))
		{
			//machineState = LOCATING;
		}
		return true;
	}return false;
}

bool Manager::filter_points2stack(const cv::Point2d& _p)
{
	if(optimizedCnt++ < OPTIMISE_CNT)
	{
		optimizedPos.x += _p.x;
		optimizedPos.y += _p.y;
	//	optimizedPos.z += _p.z;
		return false;
	}else
	{

		optimizedPos.x /= OPTIMISE_CNT;
		optimizedPos.y /= OPTIMISE_CNT;
	//	optimizedPos.z /= OPTIMISE_CNT;
		std::cout<<"optimised pos:" << optimizedPos << std::endl;
	//	cv::waitKey(0);
	//	std::cout << "after stack size:" << posStack.size() << std::endl;
		posStack.push(POINTXYZ(optimizedPos.x, optimizedPos.y, 0));
	//	std::cout << "after stack size:" << posStack.size() << std::endl;
	//	cv::waitKey(0);
		optimizedCnt = 0;
		optimizedPos = { 0,0,0 };
		return true;
	}
}


