#pragma once
#include "Register.h"
#include <vector>
class Calibrator
{
public:
	Calibrator(unsigned board_w,unsigned board_h)
		:uBoard_w(board_w),uBoard_h(board_h) {}
	~Calibrator();

	std::vector<cv::Mat> intrinsic_cali(unsigned cameraNum_);
	void extrinsic_cali(const cv::Mat &img_,cv::Size size_,double basis,double error);
	void calibrate(unsigned cameraNum_);
	std::vector<cv::Mat> read_from_file();
	double get_scale() { return scale; }
	void set_scale(double val) { scale = val; }

	bool isExCali() { return isExtrinsicCali; }
	void isExCali(bool flag) { isExtrinsicCali = flag; }

private:
	unsigned uBoard_w;
	unsigned uBoard_h;

	bool isExtrinsicCali = false;
	double scale=0.0;

	void find_rect(const std::vector<cv::Point2f>& points, cv::Point2f *rect);
	double cal_extrinsic_scale(double error, double millimeter,
		std::vector<cv::Point2f> &corners);
	double cal_extrinsic_scale(std::vector<cv::Point2f> &corners);
	double pixel_length(cv::Point2f p1,cv::Point2f p2)
	{
		return (p1.x - p2.x)*(p1.x - p2.x) + (p1.y - p2.y)*(p1.y - p2.y);
	}

};
