#include "Where.h"


Where::~Where()
{
}

cv::Rect Where::update()
{
	//cv::imshow("Where is machine?", _worldMap);
	define_edge_corner();

	draw_in_scale();
	return return_intersection_rect();

}

cv::Rect Where::return_intersection_rect()
{
	double deskWidth = (ABS_X / _m2fScale) / _p2mScale;
	double deskHeight = (ABS_Y / _m2fScale) / _p2mScale;
	cv::Rect r1(0,0,deskWidth,deskHeight);
	cv::Rect r2(cv::Point(get_pixel_from_fre(pos.x - _fovInFrequency.x/2.0) ,
		get_pixel_from_fre(pos.y - _fovInFrequency.y/2.0)),
		cv::Point(get_pixel_from_fre(pos.x + _fovInFrequency.x/2.0),
		get_pixel_from_fre(pos.y + _fovInFrequency.y/2.0)));
	//cv::Rect r4(pos.x - _fovSize.width, pos.y - _fovSize.height, _fovSize.width, _fovSize.height);
	cv::Rect r3 = r1 & r2;
	r3.x += r1.x - r2.x;
	r3.y += r1.y - r2.y;
//	std::cout << "r2 : " << r2 << std::endl;
	//std::cout << "r4 : " << r4 << std::endl;
//	std::cout << "r3 : " << r3 << std::endl;
	return r3;

}

void Where::set_fovSize_scale(cv::Size fov_,double val) 
{
	_fovSize = fov_;
	_p2mScale = val;
	_fovSizeInMilli = cv::Point2d(_fovSize.width*val,_fovSize.height*val);
	_fovInFrequency = cv::Point2d(_fovSizeInMilli.x * _m2fScale, _fovSizeInMilli.y * _m2fScale);
}

cv::Point2d Where::cal_pos_loc2wor(cv::Point2d loc,cv::Size imgSize)
{
	loc.x -= (double)imgSize.width / 2.0;
	loc.y -= (double)imgSize.height / 2.0;

	loc.x = loc.x * _p2mScale * _m2fScale;
	loc.y = loc.y * _p2mScale * _m2fScale;

	cv::Point2d worldPos;
	worldPos.x = loc.x;
	worldPos.y = loc.y;
	std::cout << "worldPos in fre" << worldPos << std::endl;
	return worldPos;
}

/*
 *	利用当前坐标判断出machine的大体位置
 */
void Where::define_edge_corner()
{
	//left part of surface
	if(pos.x<ABS_X_HALF)
	{
		//top part of surface
		if(pos.y<ABS_Y_HALF)
		{
			_corner = LEFT_TOP;
		}else 
			_corner = LEFT_BOTTOM;
	}else
	{
		if (pos.y < ABS_Y_HALF)
		{
			_corner = RIGHT_TOP;
		}
		else
			_corner = RIGHT_BOTTOM;
	}
	find_nearest_border();
}


void Where::find_nearest_border()
{
	float k = (float)ABS_X / (float)ABS_Y;
	float k1 = (float)pos.x / (float)pos.y;
	float k2 =- (float)pos.x / (float)(ABS_Y - pos.y);
	if(k1 >= k)
	{
		if(k2 >= -k )
		{
			//left side
			_edge = EDGE::LEFT;
		}else
		{
			//down side
			_edge = EDGE::BOTTOM;
		}
	}else
	{
		if(k2 >= -k)
		{
			//top side
			_edge = EDGE::TOP;
		}else
		{
			//right side
			_edge = EDGE::RIGHT;
		}
	}

}

#define SCALE 1000
void Where::draw_in_scale()
{
	cv::Mat m(ABS_Y / SCALE, ABS_X / SCALE, CV_8UC1, cv::Scalar(255,255,255));

	std::cout << "fov size:" << _fovInFrequency << std::endl;
	cv::rectangle(m, cv::Point((pos.x - _fovInFrequency.x) / SCALE, (pos.y - _fovInFrequency.y) / SCALE),
		cv::Point((pos.x + _fovInFrequency.x) / SCALE, (pos.y + _fovInFrequency.y) / SCALE), cv::Scalar(0));

	//std::cout << "r3 : " << r3 << std::endl;
	cv::imshow("比例尺全局图", m);
}


