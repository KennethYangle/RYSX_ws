#include "tracker_node/SocketServer.h"
#include <ros/ros.h>
#include <geometry_msgs/Vector3Stamped.h>

int main(int argc, char** argv)
{
    ros::init(argc, argv, "listener_node");
    ros::NodeHandle nh;
    geometry_msgs::Vector3Stamped msg;
    ros::Publisher pub = nh.advertise<geometry_msgs::Vector3Stamped>("tracker/pos_image", 1);
    // ros::Rate loop_rate(100.0);

    SocketServer SS("127.0.0.1",6667);

    while(ros::ok)
    {
        SS.receive_msg();

        DataPack DP;
        memcpy(DP.pack_field, SS.buff, 40*sizeof(unsigned char));
        msg.header.stamp = ros::Time::now();
        msg.vector.x = DP.data_field[0];
        msg.vector.y = DP.data_field[1];
        msg.vector.z = DP.data_field[2] * DP.data_field[3];
        pub.publish(msg);
        // cout << msg << endl;

        // loop_rate.sleep();
        ros::Duration(0.001).sleep();
    }
    
    SS.release();
    return 0;
}

