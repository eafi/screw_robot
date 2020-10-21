#include "Analyzer.h"
#define SPLIT_PIXEL 300


Analyzer::Analyzer()
{

}


Analyzer::~Analyzer()
{
}

/*
 * Date : 8/7/2019
 * 设计1：将img拆分为多个小roi后套用霍夫圆
 * 或设计2：将拆分的多个小roi导入SVM预测
 */
POINTXYZ Analyzer::analyze(const cv::Mat& img)
{
	if(img.empty())
		return POINTXYZ(-1,-1,-1);
	cv::Mat image;
	img.copyTo(image);
	cv::RotatedRect rotatedRect(cv::Point2f(0,0),cv::Size2f(0,0),0);
	cv::Point2d pcbLocate;
	pcb_locate(image, pcbLocate, cv::Size(25, 25), rotatedRect);
	/*
	 * bounding rect
	 */
	cv::Rect boundngRect = rotatedRect.boundingRect();
	boundngRect = boundngRect & cv::Rect(0, 0, image.cols, image.rows);
	show_img(image(boundngRect), "boudingRect", 0.3);
	std::vector<cv::Mat> vecImg;
	//拆分图像
	unsigned colCnt=0;
	unsigned rowCnt=0;
	split_img(image(boundngRect), vecImg, 300,colCnt,rowCnt);

	cv::Point p(0,0);
	std::vector<cv::Vec3f> vecCircles;
	for(int row = 0;row<rowCnt;++row)
	{
		p.y = row * SPLIT_PIXEL+boundngRect.y;
		for(int col=0;col<colCnt;++col)
		{
			p.x = col * SPLIT_PIXEL+boundngRect.x;
			cv::Vec3f circle;
			bool circleCheck = analyze_splited_img(vecImg[col + row*colCnt], p, circle);
			if (circleCheck)
			{
				vecCircles.push_back(circle);
			}
		}
	}

	cv::Mat a(image.size(), CV_8UC1, cv::Scalar(0));
	for(auto &i:vecCircles)
	{
		cv::circle(a, cv::Point(i[0], i[1]), i[2], cv::Scalar(255));
	}
	show_img(a, "aaa", 0.3);
	show_img(vecImg[0], "SplitRect", 0.3);
}

void Analyzer::split_img(const cv::Mat img, std::vector<cv::Mat> &vecImg, unsigned diameter,unsigned &colCnt,unsigned &rowCnt)
{
	unsigned residualRow = img.rows % diameter;
	unsigned residualCol = img.cols % diameter;
	unsigned additionalRow = diameter - residualRow;
	unsigned additionalCol = diameter - residualCol;
	cv::Mat imgDividable(img.rows + additionalRow, img.cols + additionalCol,CV_8UC1,cv::Scalar(255));
	cv::Mat imgDividableRoi = imgDividable(cv::Rect(0, 0, img.cols, img.rows));
	img.copyTo(imgDividableRoi);
	show_img(imgDividable, "dividable", 0.3);
	colCnt = (imgDividable.cols / diameter);
	rowCnt = (imgDividable.rows / diameter);
	for(int i=0;i<rowCnt;++i)
	{
		for(int j=0;j<colCnt;++j)
		{
			cv::Mat aSplit = imgDividable(cv::Rect(j*diameter, i*diameter, diameter, diameter));
			vecImg.push_back(aSplit);
		}
	}

}



cv::Mat Analyzer::filter_img(unsigned method_, const cv::Mat& img_)
{
	return cv::Mat();
}

/*
 *  pcb_locate function
 *  将导入的img图像分析，取得PCB的位置坐标并返回
 * 
 */
void Analyzer::pcb_locate(const cv::Mat &img,cv::Point2d &pcbPos,cv::Size dilateKernelSize,cv::RotatedRect &pcbRotatedRect)
{
	cv::Mat img0;
	img.copyTo(img0);
	cv::Mat imgWithoutBg = cv::Mat(img0.size(),CV_8UC1,cv::Scalar(255)) - img0;
	//threshold
	cv::threshold(imgWithoutBg, imgWithoutBg,150,255,cv::THRESH_BINARY) ;

	//膨胀
	cv::Mat element = cv::getStructuringElement(cv::MORPH_RECT, dilateKernelSize);
	cv::dilate(imgWithoutBg, img0, element);

	/* connect components detect
	*  该步骤可能检测出很多物体，所需需要对物体进行分类
	*  在设计之初假设，传入图像内PCB为唯一对象，但为了避免错误判断PCB（比如，有噪点的原因导致分析出两个物体
	*  ）此时先假设PCB面积应该为检测出的所有物体面积中最大
	*/
	cv::Mat labels, stats, centroids;
	int numObj = cv::connectedComponentsWithStats(img0, labels, stats, centroids);

	if(numObj < 2)
	{
		std::cout << "no obj detected." << std::endl;
	}else
	{
		/*
		 * 找出PCB ,设PCB为图像中唯一物体，面积大于10000
		 */
		cv::Mat img1(labels.size(), CV_8UC1, cv::Scalar(0));
		int maxCnt = 1;
		double maxArea = 0.0, tmp;
		for (int i = 1; i < numObj; ++i)
		{
			tmp = stats.at<int>(i, cv::CC_STAT_AREA);
			if (tmp > maxArea)
			{
				maxCnt = i;
				maxArea = tmp;
			}
		}
		if(maxArea >= PCB_MIN_AREA)
		{
			img1 = labels == maxCnt;
			std::cout << "lables : " << labels.type() << std::endl;
			pcbPos = centroids.at<cv::Point2d>(maxCnt); //返回PCB的重心坐标，相对于当前图样的像素坐标
			std::string posStr;
			posStr = "(" + std::to_string(pcbPos.x) + "," + std::to_string(pcbPos.y) + ")";
			cv::putText(img1, posStr, cv::Point(pcbPos.x, pcbPos.y+130), 1, 5, cv::Scalar(255));
			std::cout << "PCB area :" << maxArea << std::endl;
			cv::Mat connectDraw(img1.size(),CV_8UC1,cv::Scalar(255));
			/*
			 * 产生包括矩形
			 */
			std::vector<cv::Point> points;
			unsigned *p;
			for(int i=0;i<img1.rows;++i)
			{
				p = labels.ptr<unsigned>(i);
				for(int j=0;j<img1.cols;++j)
				{
					if (p[j] == maxCnt)
						points.push_back(cv::Point(j,i));
				}
			}
			pcbRotatedRect = cv::minAreaRect(points);

			cv::Point2f vertex[4];
			pcbRotatedRect.points(vertex);
			for(int i=0;i<4;++i)
			{
				cv::line(connectDraw, vertex[i], vertex[(i + 1) % 4], cv::Scalar(0), 2, cv::LINE_AA);
			}
			show_img(connectDraw, "rotateRect_pcb", 0.3);
		}
		
		show_img(img1, "PCB", 0.3);
	//	cv::imshow("PCB", img1);
	}
}

bool Analyzer::analyze_splited_img(const cv::Mat& img, cv::Point imgPos, cv::Vec3f &circle)
{
	cv::medianBlur(img, img, 5);
	std::vector<cv::Vec3f> circles;
	cv::HoughCircles(img, circles, cv::HOUGH_GRADIENT, 1.5, 10);
	if (!circles.empty())
	{
		circle[0] = circles[0][0]+imgPos.x;
		circle[1] = circles[0][1]+imgPos.y;
		circle[2] = circles[0][2];
		return true;
	}
	else return false;
}


