#include "Calibrator.h"

Calibrator::~Calibrator()
{
}

std::vector<cv::Mat> Calibrator::intrinsic_cali(unsigned cameraNum_)
{
	unsigned board_n = uBoard_w * uBoard_h;
	unsigned n_boards = 15;
	float image_sf = 1;
	cv::Size board_sz = cv::Size(uBoard_w, uBoard_h);
	std::cout << "board_sz : " << board_sz << std::endl;
	cv::VideoCapture capture(cameraNum_);
	if(!capture.isOpened())
	{
		std::cout << "\n open camera failed.";
		return std::vector<cv::Mat>{cv::Mat(),cv::Mat()};
	}

	std::vector<std::vector<cv::Point2f>> image_points;
	std::vector<std::vector<cv::Point3f>> object_points;

	double last_captured_timestamp = 0;
	cv::Size image_size;

	while(image_points.size() < (size_t)n_boards)
	{
		cv::Mat image0, image;
		capture >> image0;
		image_size = image0.size();
		cv::resize(image0, image, cv::Size(), image_sf, image_sf, cv::INTER_LINEAR);

		std::vector<cv::Point2f> corners;
		bool found = cv::findChessboardCorners(image, board_sz, corners);
		std::cout << "found : " << found << std::endl;
		cv::drawChessboardCorners(image, board_sz, corners, found);

		double timestamp = (double)clock() / CLOCKS_PER_SEC;

		if(found && timestamp - last_captured_timestamp > 1)
		{
			last_captured_timestamp = timestamp;
			image ^= cv::Scalar::all(255);

			cv::Mat mcorners(corners);
			mcorners *= (1. / image_sf);
			image_points.push_back(corners);
			object_points.push_back(std::vector<cv::Point3f>());
			std::vector<cv::Point3f> &opts = object_points.back();
			opts.resize(board_n);
			for(int j=0;j<board_n;++j)
			{
				opts[j] = cv::Point3f((float)(j / uBoard_w), (float)(j%uBoard_w), 0.f);
			}
			std::cout << "Collected " << (int)image_points.size() <<
				" of " << n_boards << " needed chessboard images\n" << std::endl;
		}

		cv::imshow("Calibration", image);
	}


	cv::destroyWindow("Calibration");
	std::cout << "\n\n *** Calibrating the camera #" << cameraNum_ << "... ***"<<std::endl;

	cv::Mat intrinsic_matrix, distortion_coeffs;
	double err = cv::calibrateCamera(
		object_points,
		image_points,
		image_size,
		intrinsic_matrix,
		distortion_coeffs,
		cv::noArray(),
		cv::noArray(),
		cv::CALIB_ZERO_TANGENT_DIST | cv::CALIB_FIX_PRINCIPAL_POINT
	);

	cv::FileStorage fs("intrinsics.xml", cv::FileStorage::WRITE);
	fs << "camera_matrix" << intrinsic_matrix
		<< "distortion_coefficients" << distortion_coeffs
	<< "image_size" << image_size;
	fs.release();

	cv::Mat map1, map2;
	cv::initUndistortRectifyMap(
		intrinsic_matrix,
		distortion_coeffs,
		cv::Mat(),
		intrinsic_matrix,
		image_size,
		CV_16SC2,
		map1,
		map2
	);
	return std::vector<cv::Mat>{map1, map2};
}

void Calibrator::extrinsic_cali(const cv::Mat &img_,cv::Size size_,double basis,double error)
{
	std::vector<cv::Point2f> corners;
	cv::Mat image;
	img_.copyTo(image);
	bool found = cv::findChessboardCorners(image, size_, corners,cv::CALIB_CB_FAST_CHECK);

	std::cout << "found : " << found << std::endl;
	cv::Mat todraw(image.size(), CV_8UC3, cv::Scalar(255, 255, 255));
	//cv::drawChessboardCorners(img_, size_, corners, found);
	cv::drawChessboardCorners(img_, size_, corners, found);
	if(found)
	{
		cv::Point2f rect[4] = { cv::Point2f(0,0), };
		find_rect(corners, rect);

		double k1, k2;
		k1 = (rect[0].y - rect[1].y) / (rect[0].x - rect[1].x);
		k2 = (rect[0].y - rect[2].y) / (rect[0].x - rect[2].x);
		cv::line(img_, rect[0], cv::Point2f(rect[1].x + 100.0, k1*(rect[1].x + 100.0 - rect[0].x) + rect[0].y), cv::Scalar(0, 0, 255), 2);
		cv::line(img_, rect[0], cv::Point2f(rect[2].x + 100.0, k2*(rect[2].x + 100.0 - rect[0].x) + rect[0].y), cv::Scalar(0, 255, 255), 2);
		scale = cal_extrinsic_scale(error, basis*basis, corners);
		if (scale != 0.0)
		{
			std::cout << "scale : " << scale << std::endl;
			isExCali(true);
		}
	}
}

void Calibrator::calibrate(unsigned cameraNum_)
{
	intrinsic_cali(cameraNum_);
}

std::vector<cv::Mat> Calibrator::read_from_file()
{

	cv::Mat intrinsic_matrix, distortion_coeffs;
	cv::Size image_size;
	cv::FileStorage fs("intrinsics.xml", cv::FileStorage::READ);
	fs["camera_matrix"] >> intrinsic_matrix;
	fs["distortion_coefficients"] >> distortion_coeffs;
	fs["image_size"] >> image_size;
	std::cout << "\n intrinsic matrix : " << intrinsic_matrix << std::endl;
	std::cout << "\ndistortion matrix : " << distortion_coeffs << std::endl;
	std::cout << "\nimage_size" << image_size << std::endl;

	cv::Mat map1, map2;
	cv::initUndistortRectifyMap(
		intrinsic_matrix,
		distortion_coeffs,
		cv::Mat(),
		intrinsic_matrix,
		image_size,
		CV_16SC2,
		map1,
		map2
	);
	return std::vector<cv::Mat>{map1, map2};
}
void Calibrator::find_rect(const std::vector<cv::Point2f>& points,cv::Point2f *rect)
{
	std::vector<cv::Point2f> x_points = points;
	std::vector<cv::Point2f> y_points = points;
	std::sort(x_points.begin(), x_points.end(), [](cv::Point2f a, cv::Point2f b) {return a.x < b.x; });
	std::sort(y_points.begin(), y_points.end(), [](cv::Point2f a, cv::Point2f b) {return a.y < b.y; });

	if (!points.empty())
	{
//		std::cout << "lt" << x_points.at(0) << std::endl;
//		std::cout << "rb" << x_points.at(x_points.size() - 1) << std::endl;
//		std::cout << "rt" << y_points.at(0) << std::endl;
//		std::cout << "lb" << y_points.at(y_points.size() - 1) << std::endl;
		cv::Mat a(500, 500, CV_8UC1, cv::Scalar(0));
		unsigned char *p;
		int j = 0;
		for(auto &i:points)
		{
		//	p = a.ptr<unsigned char>((int)i.y);
			cv::putText(a, std::to_string(j++), i, cv::FONT_HERSHEY_PLAIN, 1, cv::Scalar(255));
	//		p[(int)i.x] = 255;
		//	std::cout << i << std::endl;
		}
		cv::imshow("a", a);
		rect[0]=(x_points.at(0)); //left top
		rect[1]=(y_points.at(0)); //right top
		rect[2]=(y_points.at(y_points.size()-1)); //left bottom
		rect[3]=(x_points.at(x_points.size()-1)); //right bottom
	}
}

double Calibrator::cal_extrinsic_scale(double error, 
	double millimeter,std::vector<cv::Point2f> &corners)
{
	double l0 = 0.0;
//	double l1 = 0.0;
	l0 = cal_extrinsic_scale(corners);

//	l1 = pixel_length(corners[12], corners[16]);
//	std::cout << "l4 :" << l1 << std::endl;

//	std::cout << "error : " << abs(l1 - l0) << std::endl;
//	if (abs(l1 - l0) < error)
	{
		std::cout << "sqrt l0" << sqrt(l0) << " .sqrt millimter" << sqrt(millimeter) << std::endl;
		return sqrt(millimeter)/sqrt(l0);
	}
    return 0.0;
}

double Calibrator::cal_extrinsic_scale(std::vector<cv::Point2f> &corners)
{
	double l0 = pixel_length(corners[3], corners[7]);
	double l1 = pixel_length(corners[7], corners[6]);
	double l2 = pixel_length(corners[2], corners[6]);
	double l3 = pixel_length(corners[2], corners[3]);

	std::cout << "l0 :" << l0 << std::endl;
	std::cout << "l1 :" << l1 << std::endl;
	std::cout << "l2 :" << l2 << std::endl;
	std::cout << "l3 :" << l3 << std::endl;
	double l4 = (l0 + l1 + l2 + l3)/4.0;

	return l4;
}


