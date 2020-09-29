#include <iostream>
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <ros/ros.h>
#include "sensor_msgs/PointCloud2.h"
#include <std_msgs/Float32MultiArray.h>
#include "sensor_msgs/Image.h"
#include <cv_bridge/cv_bridge.h>


using namespace cv;
using namespace std;


int color_count; 
static int radius_path = 40;
int confidence = 0;
Point2d trackPoint;
int test = 10;
float sigmax;
float sigmay;
Point2d colorBlock3;

ros::Publisher centerPointPub;


Point2d frameToCoordinate(int colortype,  Mat frame, int lowh, int lows, int lowv, int highh, int highs, int highv)
{
	Point2d xy;
	Mat imgHSV;
	vector<Mat> hsvSplit;
	cvtColor(frame, imgHSV, COLOR_BGR2HSV);
	//Convert the captured frame from BGR to HSV
	
	split(imgHSV, hsvSplit);
	equalizeHist(hsvSplit[2], hsvSplit[2]);
	merge(hsvSplit, imgHSV);
	Mat imgThresholded;
	inRange(imgHSV, Scalar(lowh, lows, lowv), Scalar(highh, highs, highv), imgThresholded); //Threshold the image

	//开操作 (去除一些噪点)
	Mat element = getStructuringElement(MORPH_RECT, Size(3, 3));
	morphologyEx(imgThresholded, imgThresholded, MORPH_OPEN, element);
	//闭操作
	Mat kernel = getStructuringElement(MORPH_ELLIPSE, Size(9, 9));
    morphologyEx(imgThresholded, imgThresholded, MORPH_CLOSE, kernel);
	imshow("red block", imgThresholded);
	

	vector<Vec3f> circles;

	HoughCircles(imgThresholded, circles, CV_HOUGH_GRADIENT, 
					3, //累加器分辨率
					20, //两园间最小距离
					160, // canny高阈值
					80, //最小通过数
					30, 300 );  //最小和最大半径
	test++;
	cout<<"------->circles.size() :"<<circles.size()<<endl;

	Point centexy(0,0);
	
	for (size_t i = 0; i < circles.size(); i++)
	{
		//提取出圆心坐标  
		Point center(round(circles[i][0]), round(circles[i][1]));
		//提取出圆半径  
		int radius = round(circles[i][2]);
		// circle(frame, center, 3, Scalar(0, 255, 0), -1, 4, 0);
		// circle(frame, center, radius, Scalar(0, 255, 0), 3, 4, 0);
		centexy.x += center.x;
		centexy.y += center.y;

	}
	if(circles.size()>0)
	{
		confidence = 1;
		centexy.x = centexy.x/circles.size();
		centexy.y = centexy.y/circles.size();
		// sigmax = 0;
		// sigmay = 0; 
		// for (size_t i = 0; i < circles.size(); i++)
		// {
		// 	sigmax += (round(circles[i][0])-centexy.x)*(round(circles[i][0])-centexy.x);
		// 	sigmay += (round(circles[i][1])-centexy.y)*(round(circles[i][1])-centexy.y);
		// }
		// sigmax = sqrt(sigmax)/circles.size();
		// sigmay = sqrt(sigmay)/circles.size();

		// Point finalcenter(0,0);
		// int finalcnt = 0;
		// for (size_t i = 0; i < circles.size(); i++)
		// {
		// 	if(abs(round(circles[i][0])-centexy.x)<3.4*sigmax && abs(round(circles[i][1])-centexy.y)<3.4*sigmay)
		// 	{
		// 		finalcenter.x += round(circles[i][0]);
		// 		finalcenter.y += round(circles[i][1]);
		// 		finalcnt++;
		// 	}
		// }
		// cout<<"------->finalcnt :"<<finalcnt<<endl;
		// if(finalcnt>0)
		// {
		// 	finalcenter.x = finalcenter.x/finalcnt;
		// 	finalcenter.y = finalcenter.y/finalcnt;
		// 	confidence = 1;
		// }
		// else{
		// 	confidence = -1;
		// }
		

		circle(frame, centexy, 3, Scalar(0, 0, 255), -1, 4, 0);
	}
	else
	{
		confidence = -1;
	}
	
	imshow("circle", frame);
	waitKey(1);
	return centexy;
}

void imageCallback(const sensor_msgs::Image::ConstPtr &imgae_msg)
{
	//red
	cv_bridge::CvImagePtr cv_ptr = cv_bridge::toCvCopy(imgae_msg, sensor_msgs::image_encodings::BGR8);
	Mat imgOriginal = cv_ptr -> image;

	colorBlock3 = frameToCoordinate(3, imgOriginal, 170, 0, 0, 180, 255, 255);
	
	std_msgs::Float32MultiArray msg;
	msg.data.push_back(colorBlock3.x);   // x
	msg.data.push_back(colorBlock3.y);   // y
	msg.data.push_back(confidence);   // bbox_w
	msg.data.push_back(confidence);   // bbox_h
	msg.data.push_back(1);   // confidence
	centerPointPub.publish(msg);
	
	
}


int main(int argc, char** argv)
{
    // auto cap = VideoCapture("/home/zhou/Desktop/7.mp4");
    // if(!cap.isOpened())
	// {
		
	// 	cout<<"no captured...";
	// 	return 0;
	// }
        
    // namedWindow("Thresholded Image",WINDOW_NORMAL);
    // resizeWindow("Thresholded Image",1080,960);
	ros::init(argc,argv,"color_tracker");
    ros::NodeHandle nh;
	ros::Time::init();

	


    //发布中心坐标
    
    centerPointPub = nh.advertise<std_msgs::Float32MultiArray>("color_tracker_point",1);

	//订阅图像
	ros::Subscriber sub = nh.subscribe("/camera/color/image_raw", 1, &imageCallback);

	
	ros::spin();


	// ros::Rate loop_rate(1000);
	
	// while(1)
	// 	{
			
	// 		Mat imgOriginal ;
			
	// 		cap>> imgOriginal;
	// 		if(imgOriginal.empty())
	// 		{
	// 			cout<<"over...";
	// 			break;
	// 		}
	// 		int width = imgOriginal.cols;
	// 		int height = imgOriginal.rows;


	// 		colorBlock3 = frameToCoordinate(3, imgOriginal, 170, 0, 0, 180, 255, 255);
	

	// 		auto k = waitKey(10) & 0xff;
	// 		if(k == 27)
	// 		{
	// 			break;
	// 		}
	// 		ros::spinOnce();
	// 		loop_rate.sleep();
	// 	}

	// cap.release();
	// destroyAllWindows();
	
}