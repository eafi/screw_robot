#pragma once
#include "Register.h"
class Analyzer
{
public:
	Analyzer();
	~Analyzer();

	void cut_img_corner_edge(cv::Mat &img,unsigned _corner,unsigned _edge);
	POINTXYZ analyze(const cv::Mat &img);


	void pcb_locate(const cv::Mat &img,cv::Point2d &pcbPos,cv::Size dilateKernelSize,cv::RotatedRect &pcbRotatedRect);
private:
	void split_img(const cv::Mat img, std::vector<cv::Mat> &vecImg, unsigned diameter,unsigned &colCnt,unsigned &rowCnt);
	cv::Mat filter_img(unsigned method_, const cv::Mat &img_);
	bool analyze_splited_img(const cv::Mat &img,cv::Point imgPos,cv::Vec3f &circle);
	cv::Mat intrinsic_matrix, distortion_coeffs;
	cv::Size image_size;

	/*
	 *	method_ 0 : x
	 *  method_ 1 : y
	 */

};

