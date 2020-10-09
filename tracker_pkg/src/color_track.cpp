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
					50, //两园间最小距离
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

void depth_Callback(const sensor_msgs::ImageConstPtr &depth_msg)
{
    cout << "depth_Callback begin" << endl;
    depth_ptr = cv_bridge::toCvCopy(depth_msg, sensor_msgs::image_encodings::TYPE_32FC1);
    cout << "depth_Callback end" << endl;
}

void imageCallback(const sensor_msgs::Image::ConstPtr &imgae_msg)
{
    cout << "imageCallback begin" << endl;
	//red
	cv_bridge::CvImagePtr cv_ptr = cv_bridge::toCvCopy(imgae_msg, sensor_msgs::image_encodings::BGR8);
	Mat imgOriginal = cv_ptr -> image;
	Mat imgCor;
	flip(imgOriginal, imgCor, -1);

	// calc center point
	colorBlock3 = frameToCoordinate(3, imgCor, 170, 150, 10, 181, 256, 256);

	// publish center point
	std_msgs::Float32MultiArray msg;
	msg.data.push_back(colorBlock3.x);   // x
	msg.data.push_back(colorBlock3.y);   // y
	msg.data.push_back(average_radius);   // bbox_w
	msg.data.push_back(average_radius);   // bbox_h
	msg.data.push_back(confidence);   // confidence
	centerPointPub.publish(msg);
	cout << "centerxy: " << colorBlock3 << endl;

	// center_point, left_point and right_point
	center_point.x = (int)colorBlock3.x;
    center_point.y = (int)colorBlock3.y;
    left_point.x = center_point.x - (average_radius/2);
    left_point.y = center_point.y;
    right_point.x = center_point.x + (average_radius/2);
    right_point.y = center_point.y;
	
	// calc depth
    depth.pose.position.x = depth.pose.position.y = depth.pose.position.z = -1;
    if (center_point.x <= 0 || center_point.x >= depth_w || center_point.y <= 0 || center_point.y >= depth_h) {
        depth.pose.position.x = depth.pose.position.y = depth.pose.position.z = -1;
    }
    else 
    {
        // cv::imshow("depth_view", cv_bridge::toCvShare(depth_msg, sensor_msgs::image_encodings::TYPE_32FC1)->image);
        double minval, maxval;
        cv::minMaxIdx(depth_ptr -> image, &minval, &maxval);
        cv::Mat adjMapOri, adjMap, hotMap;
        // expand your range to 0 and 255
        depth_ptr -> image.convertTo(adjMapOri, CV_8UC1, 255/(maxval-minval), -minval);
        flip(adjMapOri, adjMap, -1);
        applyColorMap(adjMap, hotMap, COLORMAP_HOT);
        // Rectangle: img, left-top, right-bottom, color, line-width, line-type, point-type 
        cv::rectangle(hotMap, center_point-center_offset, center_point+center_offset, Scalar(255,0,0),3,8,0);
        cv::rectangle(hotMap, left_point-center_offset, left_point+center_offset, Scalar(0,255,0),3,8,0);
        cv::rectangle(hotMap, right_point-center_offset, right_point+center_offset, Scalar(0,255,0),3,8,0);

        cout << "depth_ptr begin" << endl;
        depth_pic = depth_ptr->image;
        cout << "depth_ptr end" << endl;

        float depth_sum = 0;
        int cnt = 0;
        for (int i=center_point.x-center_offset.x; i<=center_point.x+center_offset.x; i++) {
            for (int j=center_point.y-center_offset.y; j<=center_point.y+center_offset.y; j++) {
                // border detection
                if (i<0 || i>=depth_w || j<0 || j>=depth_h)   continue;
                int a = depth_w - i;
                int b = depth_h - j;
                float dd = depth_pic.ptr<float>(b)[a];
                if (dd > 10 && dd < 8182) {
                    cnt++;
                    // cout << j << ", " << i << ": " << dd << endl;
                    depth_sum += dd;
                }
            }
        }
        if (cnt == 0) {
            depth.pose.position.x = depth.pose.position.y = depth.pose.position.z = -1;
        }
        else {
            depth.pose.position.x = depth.pose.position.y = depth.pose.position.z = depth_sum/1000.0/cnt;
        }


        float depth_sum_left = 0;
        int cnt_left = 0;
        for (int i=left_point.x-center_offset.x; i<=left_point.x+center_offset.x; i++) {
            for (int j=left_point.y-center_offset.y; j<=left_point.y+center_offset.y; j++) {
                // border detection
                if (i<0 || i>=depth_w || j<0 || j>=depth_h)   continue;
                int a = depth_w - i;
                int b = depth_h - j;
                float dd = depth_pic.ptr<float>(b)[a];
                if (dd > 10 && dd < 8182) {
                    cnt_left++;
                    // cout << j << ", " << i << ": " << dd << endl;
                    depth_sum_left += dd;
                }
            }
        }
        if (cnt == 0) {
            depth_left.pose.position.x = depth_left.pose.position.y = depth_left.pose.position.z = -1;
        }
        else {
            depth_left.pose.position.x = depth_left.pose.position.y = depth_left.pose.position.z = depth_sum_left/1000.0/cnt_left;
        }


        float depth_sum_right = 0;
        int cnt_right = 0;
        for (int i=right_point.x-center_offset.x; i<=right_point.x+center_offset.x; i++) {
            for (int j=right_point.y-center_offset.y; j<=right_point.y+center_offset.y; j++) {
                // border detection
                if (i<0 || i>=depth_w || j<0 || j>=depth_h)   continue;
                int a = depth_w - i;
                int b = depth_h - j;
                float dd = depth_pic.ptr<float>(b)[a];
                if (dd > 10 && dd < 8182) {
                    cnt_right++;
                    // cout << j << ", " << i << ": " << dd << endl;
                    depth_sum_right += dd;
                }
            }
        }
        if (cnt_right == 0) {
            depth_right.pose.position.x = depth_right.pose.position.y = depth_right.pose.position.z = -1;
        }
        else {
            depth_right.pose.position.x = depth_right.pose.position.y = depth_right.pose.position.z = depth_sum_right/1000.0/cnt_right;
        }
        cout << "111 begin" << endl;

        putText(hotMap, "depth: "+std::to_string(depth.pose.position.x), Point(100,100),
                FONT_HERSHEY_SIMPLEX, 0.7, Scalar(255,0,0), 2, CV_AA);
        putText(hotMap, "depth_left: "+std::to_string(depth_left.pose.position.x), Point(100,50),
                FONT_HERSHEY_SIMPLEX, 0.7, Scalar(255,0,0), 2, CV_AA);
        putText(hotMap, "depth_right: "+std::to_string(depth_right.pose.position.x), Point(100,150),
                FONT_HERSHEY_SIMPLEX, 0.7, Scalar(255,0,0), 2, CV_AA);

        imshow("DepthImage", hotMap);
        cv::waitKey(1);
    }
    
    pub.publish(depth);
    pub_left.publish(depth_left);
    pub_right.publish(depth_right);
    cout << "imageCallback end" << endl;
}


int main(int argc, char** argv)
{
	ros::init(argc,argv,"color_tracker");
    ros::NodeHandle nh;
	ros::Time::init();

    image_transport::ImageTransport it(nh);
    image_transport::Subscriber cloud_sub = it.subscribe("/camera/aligned_depth_to_color/image_raw", 1, depth_Callback);
	image_transport::Subscriber color_sub = it.subscribe("/camera/color/image_raw", 1, imageCallback);

    pub = nh.advertise<geometry_msgs::PoseStamped>("tracker/depth", 1);
    pub_left = nh.advertise<geometry_msgs::PoseStamped>("tracker/depth_left", 1);
    pub_right = nh.advertise<geometry_msgs::PoseStamped>("tracker/depth_right", 1);
    centerPointPub = nh.advertise<std_msgs::Float32MultiArray>("tracker/pos_image",1);
	//订阅图像
	ros::spin();
}