#include "tracker_node/SocketClient.h"
#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <std_msgs/Float32MultiArray.h>
#include <sensor_msgs/PointCloud2.h>

float pos_i[5];     // i_x, i_y, bbox_w, bbox_h, confidence
sensor_msgs::PointCloud2 output;

void cloud_cb (const sensor_msgs::PointCloud2ConstPtr& input)
{
    output = *input;
}

void pos_image_cb (const std_msgs::Float32MultiArray& input)
{
    pos_i[0] = input.data[0];
    pos_i[1] = input.data[1];
    pos_i[2] = input.data[2];
    pos_i[3] = input.data[3];
    pos_i[4] = input.data[4];
}

int main(int argc, char** argv) {
    // Initialize ROS
    ros::init(argc, argv, "point");
    ros::NodeHandle nh;
    // Create a ROS subscriber for the input point cloud
    ros::Subscriber cloud_sub = nh.subscribe("camera/depth/color/points", 1, cloud_cb);
    ros::Subscriber image_sub = nh.subscribe("tracker/pos_image", 1, pos_image_cb);
    ros::Publisher pub = nh.advertise<geometry_msgs::PoseStamped>("tracker/depth", 1);
    ros::Rate loop_rate(30);

    geometry_msgs::PoseStamped depth;
    while(ros::ok())
    {
        // TODO
        // depth.pose.position.x = 1.1;
        // depth.pose.position.y = 1.1;
        // depth.pose.position.z = 1.1;
        pub.publish(depth);
        loop_rate.sleep();
    }

    ros::spin();
    return 0;
}
