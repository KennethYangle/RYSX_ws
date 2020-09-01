#include "tracker_node/SocketClient.h"
#include <ros/ros.h>
#include <std_srvs/Empty.h>

bool handle_save(std_srvs::Empty::Request &req,  std_srvs::Empty::Response &res){
    //显示请求信息
    ROS_INFO("Received the save image request");
    //处理请求
    SocketClient SC("127.0.0.1", 6666);
    char user_command[10];
    memset(user_command, 0, 10*sizeof(char));
    user_command[0] = 'c';
    SC.send_msg(user_command);
    ros::Duration(2).sleep();
    SC.release();
    //返回true，正确处理了请求
    return true;
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "command_server");
    ros::NodeHandle n;
    ros::ServiceServer service = n.advertiseService("tracker/save_img", handle_save);
    ros::spin();

	return 0;
}
