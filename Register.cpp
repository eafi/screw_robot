#include "Register.h"

bool posEqual(POINTXYZ p1,POINTXYZ p2)
{
	return (p1.x == p2.x&&p1.y == p2.y&&p1.z == p2.z);
}

void show_img(const cv::Mat& img,const std::string &winName,float scale)
{
	cv::Mat img2Show;
	cv::resize(img, img2Show, cv::Size(), scale,scale);
	cv::imshow(winName, img2Show);
}

