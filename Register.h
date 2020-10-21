#pragma once
#include <opencv2/opencv.hpp>
#include <iostream>
const unsigned ABS_X = 320000;
const unsigned ABS_Y = 320000;
const unsigned ABS_Z = 320000;
const unsigned ABS_X_HALF = ABS_X/2;
const unsigned ABS_Y_HALF = ABS_Y/2;
const unsigned ABS_Z_HALF = ABS_Z/2;

const unsigned PCB_MIN_AREA = 10000;

const unsigned XYZ_STR_PADDING = 6;

struct POINTXYZ
{
	POINTXYZ() {}
	POINTXYZ(unsigned _x, unsigned _y, unsigned _z) { x = _x; y = _y; z = _z; }
	unsigned x;
	unsigned y;
	unsigned z;
};

bool posEqual(POINTXYZ p1, POINTXYZ p2);

void show_img(const cv::Mat& img, const std::string &winName, float scale);
