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
float bbox = 0;
float confidence = 0;
Point2d trackPoint;
int test = 10;
float sigmax;
float sigmay;
Point2d colorBlock3;
float average_radius = 0;
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
	imshowframeToCoordinate"red block", imgThresholded);
	

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
	average_radius = 0;
	
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
		average_radius += radius;

	}
	if(circles.size()>0)
	{
		bbox = 1;
		confidence = 1;
		centexy.x = centexy.x/circles.size();
		centexy.y = centexy.y/circles.size();
		average_radius = average_radius/circles.size();
		circle(frame, centexy, 3, Scalar(0, 0, 255), -1, 4, 0);
	}
	else
	{
		bbox = 0;
		average_radius = 0;
		confidence = 0;
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
	Mat imgCor;
	flip(imgOriginal, imgCor, -1);

	colorBlock3 = frameToCoordinate(3, imgCor, 170, 150, 60, 181, 256, 256);
	
	std_msgs::Float32MultiArray msg;
	msg.data.push_back(colorBlock3.x);   // x
	msg.data.push_back(colorBlock3.y);   // y
	msg.data.push_back(average_radius);   // bbox_w
	msg.data.push_back(average_radius);   // bbox_h
	msg.data.push_back(confidence);   // confidence
	centerPointPub.publish(msg);
	cout << "centerxy: " << colorBlock3 << endl;
}


int main(int argc, char** argv)
{
	ros::init(argc,argv,"color_tracker");
    ros::NodeHandle nh;
	ros::Time::init();

    //发布中心坐标
    centerPointPub = nh.advertise<std_msgs::Float32MultiArray>("tracker/pos_image",1);
	//订阅图像
	ros::Subscriber sub = nh.subscribe("/camera/color/image_raw", 1, &imageCallback);
	ros::spin();
}