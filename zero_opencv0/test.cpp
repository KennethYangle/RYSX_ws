//#include <cv.h>
//#include <highgui.h>

#include <opencv2/core/utility.hpp>
#include <opencv2/opencv.hpp>

#include <sys/shm.h>
#include <iostream>
#include <sstream>
#include <stdlib.h>
#include <unistd.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <sys/time.h>
#include <fcntl.h>
#include <termios.h>
#include <errno.h>
#include <string.h>
#include <math.h>
#include <sys/time.h>
#include <iostream>
#include <algorithm>
#include <fstream>
#include <unistd.h>
#include <math.h>
#include <unistd.h>
#include <stdlib.h>
#include <getopt.h>
#include <fcntl.h>
#include <string.h>
#include <string>
#include <vector>
#include <iomanip>
#include <sstream>
#include <stdio.h>
#include <unistd.h>
#include <stdlib.h>
#include <getopt.h>
#include <string.h>
#include <pthread.h>
#include <math.h>
#include <fcntl.h>
#include <errno.h>
#include <termios.h>

#include "opencv2/core/utility.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include "opencv2/highgui/highgui.hpp"

#include "timer.h"

#include <stdio.h>
#include <string.h>
#include <ctype.h>

using namespace cv;
using namespace std;

#include <iostream>
#include <fstream>
#include <opencv2/core/core.hpp>

double calib_stage = 0;
std::vector<cv::Point> matches;
std::vector<cv::Point> d435_points;
std::vector<cv::Point> tracker_points;
cv::Point tracker_point;

void mouseHandler(int event, int x, int y, int flags, void *param)
{
    double K = 0;

    switch (event)
    {
        case EVENT_MOUSEMOVE:

            break;
        case EVENT_LBUTTONDOWN:

            break;
        case EVENT_LBUTTONUP:
            calib_stage+=0.5;
            matches.push_back(cv::Point(x,y));
            tracker_point.x = x;
            tracker_point.y = y;
            break;
        case EVENT_MBUTTONDOWN:
            break;
        case EVENT_MBUTTONUP:
            break;
    }
}

int main(int argc, char* argv[])
{
    int depth_w = 640;
    int depth_h = 480;
    int color_w = 640;
    int color_h = 480;

    cv::namedWindow("Demo");
    setMouseCallback("Demo", mouseHandler, NULL);

    cv::Mat d435_frame = cv::imread("../1.png");
    cv::Mat tracker_frame = cv::imread("../2.png");

    cv::Mat d435_frame_raw,tracker_frame_raw;
    cv::resize(d435_frame,d435_frame_raw,cv::Size(depth_w,depth_h));
    cv::resize(tracker_frame,tracker_frame_raw,cv::Size(color_w,color_h));

	for(;;)
	{
	    cv::Mat conb_frame = cv::Mat::zeros(depth_h,depth_w+color_w,CV_8UC3);
	    for(int y=0;y<d435_frame_raw.rows;y++)
        {
	        for(int x=0;x<d435_frame_raw.cols;x++)
            {
                conb_frame.data[y*conb_frame.cols*3+x*3+0] = d435_frame_raw.data[(d435_frame_raw.rows-1-y)*d435_frame_raw.cols*3+(d435_frame_raw.cols-1-x)*3+0];
                conb_frame.data[y*conb_frame.cols*3+x*3+1] = d435_frame_raw.data[(d435_frame_raw.rows-1-y)*d435_frame_raw.cols*3+(d435_frame_raw.cols-1-x)*3+1];
                conb_frame.data[y*conb_frame.cols*3+x*3+2] = d435_frame_raw.data[(d435_frame_raw.rows-1-y)*d435_frame_raw.cols*3+(d435_frame_raw.cols-1-x)*3+2];
            }
        }
        for(int y=0;y<tracker_frame_raw.rows;y++)
        {
            for(int x=0;x<tracker_frame_raw.cols;x++)
            {
                conb_frame.data[y*conb_frame.cols*3+(x+depth_w)*3+0] = tracker_frame_raw.data[y*tracker_frame_raw.cols*3+x*3+0];
                conb_frame.data[y*conb_frame.cols*3+(x+depth_w)*3+1] = tracker_frame_raw.data[y*tracker_frame_raw.cols*3+x*3+1];
                conb_frame.data[y*conb_frame.cols*3+(x+depth_w)*3+2] = tracker_frame_raw.data[y*tracker_frame_raw.cols*3+x*3+2];
            }
        }

        if(matches.size()<14)
        {
            for(int i=0;i<matches.size()-matches.size()%2;i+=2)
            {
                cv::line(conb_frame,matches[i],matches[i+1],cv::Scalar(0,0,255),1,CV_AA);
            }
            for(int i=0;i<matches.size();i++)
            {
                if(i%2==0)
                    cv::circle(conb_frame,matches[i],6,cv::Scalar(0,0,255),2,CV_AA);
                else
                    cv::circle(conb_frame,matches[i],6,cv::Scalar(0,255,0),2,CV_AA);
            }

            putText(conb_frame, "points selected : "+std::to_string((int)calib_stage), Point(100,100),
                    FONT_HERSHEY_SIMPLEX, 0.7, Scalar(0,0,222),2,CV_AA);

        }
        if(matches.size()==14)
        {
            // compute homography matrix
            d435_points.clear();
            tracker_points.clear();
            for(int i=0;i<matches.size()-matches.size()%2;i+=2)
            {
                d435_points.push_back(cv::Point(depth_w-1-matches[i].x,depth_h-1-matches[i].y));
                tracker_points.push_back(cv::Point(matches[i+1].x-depth_w,matches[i+1].y));
            }
            Mat inlier_mask;
            cv::Mat homography = findHomography(tracker_points,d435_points,RANSAC, 2.5f, inlier_mask);

            std::cout<<homography<<std::endl<<"--------------------------------------------"<<endl;

            /*
             * 保存矩阵
             */
            FileStorage fs("../homography.xml", FileStorage::WRITE);
            fs<<"homography"<<homography;
            fs.release();
        }
        if(matches.size()>14)
        {
            /*
             * 从文件中读取变换矩阵并且测试投影准确性
             */
            // 从文件中读取变换矩阵
            cv::Mat homography_from_file;
            FileStorage fs1("../homography.xml", FileStorage::READ);
            fs1["homography"]>>homography_from_file;
            fs1.release();
            // 测试投影准确性
            std::vector<Point2f> points_projection;
            points_projection.push_back(cv::Point(tracker_point.x-depth_w,tracker_point.y));
            std::vector<Point2f> points_back_projection;
            perspectiveTransform(points_projection,points_back_projection,homography_from_file);
            cv::circle(conb_frame,cv::Point(points_projection[0].x+depth_w,points_projection[0].y),10,cv::Scalar(0,255,0),1,CV_AA);
            cv::circle(conb_frame,cv::Point(depth_w-1-points_back_projection[0].x,depth_h-1-points_back_projection[0].y),10,cv::Scalar(0,0,255),1,CV_AA);
            cv::line(conb_frame,cv::Point(points_projection[0].x+depth_w,points_projection[0].y),cv::Point(depth_w-1-points_back_projection[0].x,depth_h-1-points_back_projection[0].y),cv::Scalar(0,0,255),5,CV_AA);

            cout << "nano: " << points_projection[0] << "  --->  " << "D435i: " << points_back_projection[0] << endl;
        }

        cv::imshow("Demo", conb_frame);
        int wt = 20;
//        if(matches.size()>=14)wt = 0;
        int c_ = cv::waitKey(wt);
		if(c_==27)break;
	}

    return 0;
}

