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
cv::Mat homography_from_file;
ros::Publisher pub;
geometry_msgs::PoseStamped depth;

void pos_image_cb(const std_msgs::Float32MultiArray &input)
{
    if (input.data[0] <= 0) {
        center_point.x = center_point.y = 0;
    }
    else {
        std::vector<Point2f> points_projection;
        points_projection.push_back(cv::Point(input.data[0], input.data[1]));
        std::vector<Point2f> points_back_projection;
        perspectiveTransform(points_projection, points_back_projection, homography_from_file);
        center_point.x = int(points_back_projection[0].x);
        center_point.y = int(points_back_projection[0].y);
    }
    cout << center_point << endl;
}

void depth_Callback(const sensor_msgs::ImageConstPtr &depth_msg)
{
    if (center_point.x <= 0) {
        depth.pose.position.x = depth.pose.position.y = depth.pose.position.z = -1;
    }
    else {
        try
        {
            depth_ptr = cv_bridge::toCvCopy(depth_msg, sensor_msgs::image_encodings::TYPE_32FC1);
        }
        catch (cv_bridge::Exception &e)
        {
            ROS_ERROR("Could not convert from '%s' to 'mono16'.", depth_msg->encoding.c_str());
        }
        depth_pic = depth_ptr->image;
        cv::imshow("depth_view", depth_pic);
        cv::circle(depth_pic, center_point, 10, cv::Scalar(0,255,0),1, CV_AA);
        cv::waitKey(1);

        float depth_sum = 0;
        for (int i=center_point.x-5; i<=center_point.x+5; i++) {
            for (int j=center_point.y-5; j<=center_point.y+5; j++) {
                depth_sum += depth_pic.ptr<float>(i)[j];
            }
        }
        depth.pose.position.x = depth.pose.position.y = depth.pose.position.z = depth_sum/121;
    }
    
    pub.publish(depth);

    // output some info about the depth image in cv format
    cout << "output some info about the depth image in cv format" << endl;
    cout << "rows of the depth image = " << depth_pic.rows << endl;
    cout << "cols of the depth image = " << depth_pic.cols << endl;
    cout << "type of depth_pic's element = " << depth_pic.type() << endl;
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "calc_depth");
    ros::NodeHandle nh;
    image_transport::ImageTransport it(nh);
    image_transport::Subscriber cloud_sub = it.subscribe("/camera/aligned_depth_to_color/image_raw", 1, depth_Callback);
    ros::Subscriber image_sub = nh.subscribe("tracker/pos_image", 1, pos_image_cb);
    pub = nh.advertise<geometry_msgs::PoseStamped>("tracker/depth", 1);

    FileStorage fs1("/home/t/RYSX_ws/src/zero_opencv0/homography.xml", FileStorage::READ);
    fs1["homography"] >> homography_from_file;
    cout << homography_from_file << endl;
    fs1.release();

    ros::spin();
    return 0;
}
