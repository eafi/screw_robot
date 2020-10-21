/*
 *  Author : Eafi  *
 *  imeafi@gmail.com
 *  Date : 7/22/2019
 */
#pragma once
#include "Analyzer.h"
#include "SerialPort.h"
#include "Calibrator.h"
#include "Register.h"
#include "Where.h"
#include <string>
#include <stack>
#include "Hik.h"
#define CHECK_STR "check"
#define READ_BUF_SZ 255
#define OPTIMISE_CNT 10

class Manager
{
public:
	enum MACHINE_STATE
	{
		RESET,
		BUSY,
		LOCATING,
		FIND
	};
	Manager(unsigned cameraNum_ = 1,unsigned portNum_=3,bool fileorCalib=0)
	:iCameraNum(cameraNum_),iPortNum(portNum_),bFileorCalib(fileorCalib)
	{
		ptrWhere = new Where(ABS_X,ABS_Y, 100);
	}
	~Manager();
	void sys_run();
	
private:
	bool bFileorCalib;
	unsigned iCameraNum;
	unsigned machineState = 0;
	Analyzer *ptrAnalyzer;
	SerialPort *ptrArduino = nullptr;
	Calibrator *ptrCalibrator = nullptr;
	Where *ptrWhere;
	Hik *ptrHik;

	//string buffer from arduino
	unsigned iPortNum;
	bool serial_init();
	bool serial_init(unsigned portNum_);
	bool send_msg(const std::string& msg_);
	bool get_msg(std::string& msg_,unsigned bufSize_);
	
	void push_pos(POINTXYZ pos);
	void send_pos();
	void check_machine_state();
	std::stack<POINTXYZ> posStack;

	int startTime;
	int endTime;
	bool canSend=false;


	/*
	 * state functions
	 */
	void init_machine(const cv::Mat &img);

	bool find_pcb(const cv::Mat &img, cv::Rect roi,cv::Point2d &pcdPos);
	void locate_pcb();

	/*
	 *	优化坐标数据后入栈
	 */
	unsigned optimizedCnt = 0;
	cv::Point3d optimizedPos = { 0.0,0.0,0.0 };
	bool filter_points2stack(const cv::Point2d& p);
};

