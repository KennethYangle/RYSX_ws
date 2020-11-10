#include <iostream>
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <ros/ros.h>
#include <std_msgs/Float32MultiArray.h>
#include "sensor_msgs/Image.h"
#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>
#include <geometry_msgs/PoseStamped.h>
#include <sensor_msgs/PointCloud2.h>
#include <fstream>
#include <opencv/cv.hpp>

#include "../YEAD/EllipseDetectorYaed.h"

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
Point3d colorBlock3;
float average_radius = 0;
ros::Publisher centerPointPub;

float pos_i[5]; // i_x, i_y, bbox_w, bbox_h, confidence
cv_bridge::CvImagePtr depth_ptr;
cv::Mat depth_pic;
cv::Point2i center_point(0, 0);
cv::Point2i left_point(0, 0);
cv::Point2i right_point(0, 0);

ros::Publisher pub;
ros::Publisher pub_left;
ros::Publisher pub_right;

geometry_msgs::PoseStamped depth;
geometry_msgs::PoseStamped depth_left;
geometry_msgs::PoseStamped depth_right;

int depth_w = 640;
int depth_h = 480;
cv::Point2i center_offset(10, 10);
bool depth_initialize = false;

Point3d frameToCoordinate(int colortype,  Mat frame, int lowh, int lows, int lowv, int highh, int highs, int highv)
{
	Point3d xy;
	Mat imgHSV;
	vector<Mat> hsvSplit;
	cvtColor(frame, imgHSV, COLOR_BGR2HSV);
	//Convert the captured frame from BGR to HSV
	
	split(imgHSV, hsvSplit);
	equalizeHist(hsvSplit[2], hsvSplit[2]);
	merge(hsvSplit, imgHSV);
	Mat imgThresholded;
	inRange(imgHSV, Scalar(lowh, lows, lowv), Scalar(highh, highs, highv), imgThresholded); //Threshold the image

	// //开操作 (去除一些噪点)
	// Mat element = getStructuringElement(MORPH_RECT, Size(3, 3));
	// morphologyEx(imgThresholded, imgThresholded, MORPH_OPEN, element);
	// //闭操作
	// Mat kernel = getStructuringElement(MORPH_ELLIPSE, Size(9, 9));
    // morphologyEx(imgThresholded, imgThresholded, MORPH_CLOSE, kernel);
	imshow("red block", imgThresholded);
	
    // ros::Time begin = ros::Time::now();
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
	circle(frame, Point(320,240), 3, Scalar(0, 255, 0), -1, 4, 0);
	imshow("circle", frame);
    // ros::Time end = ros::Time::now();
    // cout<< "------------------time cost :"<< end-begin <<endl;
	waitKey(1);
    Point3d return_centerxy;
    return_centerxy.x = centexy.x;
    return_centerxy.y = centexy.y;
    return_centerxy.z = average_radius;
	return return_centerxy;
}

//yead circle 
Point3d yeadCircleCalc(Mat frame, int lowh, int lows, int lowv, int highh, int highs, int highv)
{
    // ros::Time begin = ros::Time::now();
    Point3d xy(0,0,0);
	Mat imgHSV;
	vector<Mat> hsvSplit;
	cvtColor(frame, imgHSV, COLOR_BGR2HSV);
	//Convert the captured frame from BGR to HSV
    
	split(imgHSV, hsvSplit);
	equalizeHist(hsvSplit[2], hsvSplit[2]);
	merge(hsvSplit, imgHSV);
	Mat1b imgThresholded;
	inRange(imgHSV, Scalar(lowh, lows, lowv), Scalar(highh, highs, highv), imgThresholded); //Threshold the image
	// //开操作 (去除一些噪点)
	Mat element = getStructuringElement(MORPH_RECT, Size(5, 5));
	morphologyEx(imgThresholded, imgThresholded, MORPH_OPEN, element);

	imshow("red block", imgThresholded);
    
    // Parameters Settings (Sect. 4.2)
    int		iThLength = 6;
    float	fThObb = 4.0f;
    float	fThPos = 1.0f;
    float	fTaoCenters = 0.05f;
    int 	iNs = 16;
    float	fMaxCenterDistance = sqrt(float(640*640 + 480*480)) * fTaoCenters;
    float	fThScoreScore = 0.1f;
    // Gaussian filter parameters, in pre-processing
    Size	szPreProcessingGaussKernelSize = Size(5, 5);
    double	dPreProcessingGaussSigma = 1.0;

    float	fDistanceToEllipseContour = 0.1f;	// (Sect. 3.3.1 - Validation)
    float	fMinReliability = 0.2f;	// Const parameters to discard bad ellipses
    // Initialize Detector with selected parameters
    CEllipseDetectorYaed* yaed = new CEllipseDetectorYaed();
    yaed->SetParameters(szPreProcessingGaussKernelSize,
                        dPreProcessingGaussSigma,
                        fThPos,
                        fMaxCenterDistance,
                        iThLength,
                        fThObb,
                        fDistanceToEllipseContour,
                        fThScoreScore,
                        fMinReliability,
                        iNs
    );
    // Detect
    // cv::threshold(frame_rgb_l,frame_rgb_l,200,255,THRESH_BINARY);
    
    
    vector<Ellipse> ellsYaed;
    // Mat1b gray2 = frame_rgb_l.clone();
    yaed->Detect(imgThresholded, ellsYaed);
    cv::Mat3b output_frame;
    output_frame = frame.clone();
    xy = yaed->DrawDetectedEllipses(output_frame,ellsYaed,5);

    if (false) {
        

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
            circle(output_frame, centexy, 3, Scalar(0, 0, 255), -1, 4, 0);
        }
        else
        {
            bbox = 0;
            average_radius = 0;
            confidence = 0;
        }
        circle(output_frame, Point(320,240), 3, Scalar(0, 255, 0), -1, 4, 0);
        imshow("output_frame", output_frame);
        // ros::Time end = ros::Time::now();
        // cout<< "------------------time cost :"<< end-begin <<endl;
        waitKey(1);
        Point3d return_centerxy;
        return_centerxy.x = centexy.x;
        return_centerxy.y = centexy.y;
        return_centerxy.z = average_radius;
        return return_centerxy;
    }
    else
    {
        circle(output_frame, Point(320,240), 4, Scalar(255, 0, 0), -1, 5, 0);
        // imshow("Demo",frame);
        // imshow("frame_rgb_l",frame_rgb_l);
        imshow("output_frame",output_frame);
        
        int c = waitKey(1);
        if(c == (int)' ')
        {
            waitKey(0);
        }
        if(xy.x>0 && xy.y>0)
        {
            confidence = 1;
        }
        else{
            confidence = 0;
        }

        return xy;
    }
        
}


void imageCallback(const sensor_msgs::Image::ConstPtr &imgae_msg)
{

    // if (!depth_initialize) return; sss
    // ros::Time begin_image = ros::Time::now();
	//red
	cv_bridge::CvImagePtr cv_ptr = cv_bridge::toCvCopy(imgae_msg, sensor_msgs::image_encodings::BGR8);
	Mat imgOriginal = cv_ptr -> image;
	Mat imgCor;
	flip(imgOriginal, imgCor, -1);
    // ros::Time end_image1 = ros::Time::now();
    

	// calc center point
	// colorBlock3 = frameToCoordinate(3, imgCor, 170, 150, 10, 181, 256, 256);
    colorBlock3 = yeadCircleCalc(imgCor, 0, 0, 0, 181, 256, 46);
    
    
    // ros::Time end_image2 = ros::Time::now();
    // cout<< "------------------time cost2 :"<< end_image2-begin_image <<endl;
	// publish center point
	std_msgs::Float32MultiArray msg;
	msg.data.push_back(colorBlock3.x);   // x
	msg.data.push_back(colorBlock3.y);   // y
	msg.data.push_back(colorBlock3.z);   // bbox_w
	msg.data.push_back(colorBlock3.z);   // bbox_h
	msg.data.push_back(confidence);   // confidence
	centerPointPub.publish(msg);
	// cout << "centerxy: " << colorBlock3 << endl;

	
}
    


int main(int argc, char** argv)
{
	ros::init(argc,argv,"xuanting_vision");
    ros::NodeHandle nh;
	ros::Time::init();

    image_transport::ImageTransport it(nh);
	image_transport::Subscriber color_sub = it.subscribe("/camera_front/rgb/image_raw", 1, imageCallback);

    // pub = nh.advertise<geometry_msgs::PoseStamped>("tracker/depth", 1);
    // pub_left = nh.advertise<geometry_msgs::PoseStamped>("tracker/depth_left", 1);
    // pub_right = nh.advertise<geometry_msgs::PoseStamped>("tracker/depth_right", 1);
    centerPointPub = nh.advertise<std_msgs::Float32MultiArray>("tracker/pos_image",1);
	//订阅图像
	ros::spin();


}