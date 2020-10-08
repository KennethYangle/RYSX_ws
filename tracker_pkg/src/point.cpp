#include "tracker_node/SocketClient.h"
#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <geometry_msgs/PoseStamped.h>
#include <std_msgs/Float32MultiArray.h>
#include <sensor_msgs/PointCloud2.h>
#include <iostream>
#include <fstream>
#include <opencv/cv.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
using namespace cv;
using namespace std;

float pos_i[5]; // i_x, i_y, bbox_w, bbox_h, confidence
cv_bridge::CvImagePtr depth_ptr;
cv::Mat depth_pic;
cv::Point2i center_point(0, 0);
cv::Point2i left_point(0, 0);
cv::Point2i right_point(0, 0);

cv::Mat homography_from_file;
ros::Publisher pub;
ros::Publisher pub_left;
ros::Publisher pub_right;

geometry_msgs::PoseStamped depth;
geometry_msgs::PoseStamped depth_left;
geometry_msgs::PoseStamped depth_right;

int depth_w = 640;
int depth_h = 480;
cv::Point2i center_offset(10, 10);

void pos_image_cb(const std_msgs::Float32MultiArray &input)
{
    center_point.x = input.data[0];
    center_point.y = input.data[1];
    left_point.x = center_point.x - (input.data[2]/2);
    left_point.y = center_point.y;
    right_point.x = center_point.x + (input.data[2]/2);
    right_point.y = center_point.y;
}

void depth_Callback(const sensor_msgs::ImageConstPtr &depth_msg)
{
    depth.pose.position.x = depth.pose.position.y = depth.pose.position.z = -1;
    if (center_point.x <= 0 || center_point.x >= depth_w || center_point.y <= 0 || center_point.y >= depth_h) {
        depth.pose.position.x = depth.pose.position.y = depth.pose.position.z = -1;
    }
    else 
    {
        // cv::imshow("depth_view", cv_bridge::toCvShare(depth_msg, sensor_msgs::image_encodings::TYPE_32FC1)->image);
        depth_ptr = cv_bridge::toCvCopy(depth_msg, sensor_msgs::image_encodings::TYPE_32FC1);
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

        depth_pic = depth_ptr->image;

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

        putText(hotMap, "depth: "+std::to_string(depth.pose.position.x), Point(100,100),
                FONT_HERSHEY_SIMPLEX, 0.7, Scalar(255,0,0), 2, CV_AA);
        putText(hotMap, "depth_left: "+std::to_string(depth_left.pose.position.x), Point(100,200),
                FONT_HERSHEY_SIMPLEX, 0.7, Scalar(255,0,0), 2, CV_AA);
        putText(hotMap, "depth_right: "+std::to_string(depth_right.pose.position.x), Point(100,300),
                FONT_HERSHEY_SIMPLEX, 0.7, Scalar(255,0,0), 2, CV_AA);

        imshow("DepthImage", hotMap);
        cv::waitKey(1);
    }
    
    pub.publish(depth);
    pub_left.publish(depth_left);
    pub_right.publish(depth_right);

    // output some info about the depth image in cv format
    // cout << "output some info about the depth image in cv format" << endl;
    // cout << "rows of the depth image = " << depth_pic.rows << endl;
    // cout << "cols of the depth image = " << depth_pic.cols << endl;
    // cout << "type of depth_pic's element = " << depth_pic.type() << endl;
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "calc_depth");
    ros::NodeHandle nh;
    image_transport::ImageTransport it(nh);
    image_transport::Subscriber cloud_sub = it.subscribe("/camera/aligned_depth_to_color/image_raw", 1, depth_Callback);
    ros::Subscriber image_sub = nh.subscribe("tracker/pos_image", 1, pos_image_cb);
    pub = nh.advertise<geometry_msgs::PoseStamped>("tracker/depth", 1);
    pub_left = nh.advertise<geometry_msgs::PoseStamped>("tracker/depth_left", 1);
    pub_right = nh.advertise<geometry_msgs::PoseStamped>("tracker/depth_right", 1);

    ros::spin();
    return 0;
}
