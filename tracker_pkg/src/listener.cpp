#include "tracker_node/SocketServer.h"
#include <ros/ros.h>
#include <std_msgs/Float32MultiArray.h>

int main(int argc, char** argv)
{
    ros::init(argc, argv, "listener_node");
    ros::NodeHandle nh;
    ros::Publisher pub = nh.advertise<std_msgs::Float32MultiArray>("tracker/pos_image", 1);
    // ros::Rate loop_rate(100.0);

    SocketServer SS("127.0.0.1",6667);

    while(ros::ok)
    {
        std_msgs::Float32MultiArray msg;
        SS.receive_msg();

        DataPack DP;
        memcpy(DP.pack_field, SS.buff, 40*sizeof(unsigned char));
        msg.data.push_back(DP.data_field[0]+DP.data_field[2]/2);   // x
        msg.data.push_back(DP.data_field[1]+DP.data_field[3]/2);   // y
        msg.data.push_back(DP.data_field[2]);   // bbox_w
        msg.data.push_back(DP.data_field[3]);   // bbox_h
        msg.data.push_back(DP.data_field[4]);   // confidence
        pub.publish(msg);
        // cout << msg << endl;

        // loop_rate.sleep();
        ros::Duration(0.001).sleep();
    }
    
    SS.release();
    return 0;
}
