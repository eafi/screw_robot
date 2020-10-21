#pragma once
#include <string>
#include "Register.h"

class Where
{
public:
	enum CORNER
	{
		LEFT_TOP,
		LEFT_BOTTOM,
		RIGHT_TOP,
		RIGHT_BOTTOM
	};
	enum EDGE
	{
		LEFT,TOP,RIGHT,BOTTOM
	};
	Where(unsigned threshold_x,unsigned threshold_y,unsigned threshold_z)
		:_threshold_x(threshold_x),_threshold_y(threshold_y),_threshold_z(threshold_z)
	{
		setXYZ(0, 0, 0);
		_worldMap = cv::Mat(threshold_x/1000, threshold_y/1000, CV_8UC1,cv::Scalar(255));
	}
	~Where();

	void setXYZ(unsigned x_, unsigned y_, unsigned z_) { setX(x_); setY(y_); setZ(z_); }
	void setXYZ(POINTXYZ pos_) { pos = pos_; }
	void getXYZ(unsigned &x_, unsigned &y_, unsigned &z_) { x_ = pos.x; y_ = pos.y; z_ = pos.z; }
	int getX() { return pos.x; }
	int getY() { return pos.y; }
	int getZ() { return pos.z; }
	std::string getXYZ_str()
	{
//		return "(" + std::to_string(getX()) + ","
//			+ std::to_string(getY()) + ","
//			+ std::to_string(getZ()) + ")";
		return "(" + get_xyz_str_padding(getX()) +
			get_xyz_str_padding(getY()) +
			get_xyz_str_padding(getZ()) + ")";

	}
	void setX(unsigned x_) { if (x_ < _threshold_x) { _old_x = pos.x; pos.x = x_; } }
	void setY(unsigned y_) { if(y_ < _threshold_y) { _old_y = pos.y; pos.y = y_; }}
	void setZ(unsigned z_) { if(z_ < _threshold_z) { _old_z = pos.z; pos.z = z_; }}
	void backup() { pos.x = _old_x; pos.y = _old_y; pos.z = _old_z; }

	cv::Rect update();
	void set_fovSize_scale(cv::Size _fov,double val);

	/*
	 *	
	 */
	unsigned get_corner() { return _corner; }
	unsigned get_edge() { return _edge; }

	cv::Point2f posOriginal;
	/*
	 *	image's pixel pos to world frequency pos
	 */
	cv::Point2d cal_pos_loc2wor(cv::Point2d loc,cv::Size imgSize);
private:

	void define_edge_corner();
	void find_nearest_border();
	int get_pixel_from_fre(double val) { return (int)((val / _m2fScale) / _p2mScale); }
	POINTXYZ pos;	//record the position of machine in frequency scale.
	unsigned _old_x;
	unsigned _old_y;
	unsigned _old_z;
	unsigned _threshold_x=ABS_X;
	unsigned _threshold_y=ABS_Y;
	unsigned _threshold_z;

	double _p2mScale = 0.0; // pixel scale to millimeter scale.
	//double _m2fScale = 320.0; //millimeter scale to frequency scale.
	double _m2fScale = 100;
	cv::Size _fovSize; //filed of view of camera.
	cv::Point2d _fovInFrequency; // fov in frequency scale.
	cv::Point2d _fovSizeInMilli; // fov in millimeter scale.
	cv::Mat _worldMap;
	unsigned _corner;
	unsigned _edge;

	void draw_in_scale();
	cv::Rect return_intersection_rect();
	std::string get_xyz_str_padding(int val)
	{
		std::string str = std::to_string(abs(val));
		while(str.size() < XYZ_STR_PADDING)
		{
			str.insert(0, "0");
		}
		if (val >= 0)
			str.insert(0, "1");
		else str.insert(0, "0");
		return str;
	}
};

