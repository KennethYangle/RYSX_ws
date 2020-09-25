#!/usr/bin/env python
#coding=utf-8

import numpy as np
import rospy
import os, json
#导入自定义的数据类型
from geometry_msgs.msg import PoseStamped, TwistStamped, Vector3Stamped
from gazebo_msgs.srv import *
from gazebo_msgs.msg import ModelState

sphere_pos_x,sphere_pos_y,sphere_pos_z = 1,0,0.55
sphere_orientation_w,sphere_orientation_x,sphere_orientation_y,sphere_orientation_z = 0,0,0,0
def sphere_cb(msg):
    global sphere_pos_x,sphere_pos_y,sphere_pos_z,sphere_orientation_w,sphere_orientation_x,sphere_orientation_y,sphere_orientation_z
    sphere_pos_x,sphere_pos_y,sphere_pos_z = msg.pose.position.x,msg.pose.position.y,msg.pose.position.z
    sphere_orientation_w,sphere_orientation_x,sphere_orientation_y,sphere_orientation_z = msg.pose.orientation.w, msg.pose.orientation.x, msg.pose.orientation.y, msg.pose.orientation.z

def sphere_control(params):
    rospy.init_node('iris_fpv_cam')
    sphere_pub = rospy.Publisher('gazebo/set_model_state', ModelState, queue_size=10)
    rospy.Subscriber("mavros_ruying/local_position/pose", PoseStamped, sphere_cb)

    interval_rate = 50
    interval_time = 1.0 / interval_rate
    rate = rospy.Rate(interval_rate)

    pose_msg = ModelState()
    pose_msg.model_name = 'unit_sphere'
    while not rospy.is_shutdown():
        pose_msg.pose.position.x = sphere_pos_x + params["GPS_offset"][0]
        pose_msg.pose.position.y = sphere_pos_y + params["GPS_offset"][1]
        pose_msg.pose.position.z = sphere_pos_z + params["GPS_offset"][2]
        pose_msg.pose.orientation.w = sphere_orientation_w
        pose_msg.pose.orientation.x = sphere_orientation_x
        pose_msg.pose.orientation.y = sphere_orientation_y
        pose_msg.pose.orientation.z = sphere_orientation_z
        sphere_pub.publish(pose_msg)
        rate.sleep()


if __name__ == '__main__':
    setting_file = open(os.path.join(os.path.expanduser('~'),"RYSX_ws/src","settings.json"))
    setting = json.load(setting_file)
    sphere_control(setting["Simulation"])
