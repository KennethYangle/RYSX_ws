#!/usr/bin/env python
#coding=utf-8

import numpy as np
import rospy
#导入自定义的数据类型
from geometry_msgs.msg import PoseStamped, TwistStamped, Vector3Stamped
from mavros_msgs.msg import HomePosition

from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
import cv2
from gazebo_msgs.srv import *
from gazebo_msgs.msg import ModelState

sphere_pos_x,sphere_pos_y,sphere_pos_z = 0,0,0
sphere_orientation_w,sphere_orientation_x,sphere_orientation_y,sphere_orientation_z = 0,0,0,0
def sphere_cb(msg):
    global sphere_pos_x,sphere_pos_y,sphere_pos_z,sphere_orientation_w,sphere_orientation_x,sphere_orientation_y,sphere_orientation_z
    sphere_pos_x,sphere_pos_y,sphere_pos_z = msg.pose.position.x,msg.pose.position.y,msg.pose.position.z
    sphere_orientation_w,sphere_orientation_x,sphere_orientation_y,sphere_orientation_z = msg.pose.orientation.w, msg.pose.orientation.x, msg.pose.orientation.y, msg.pose.orientation.z



def sphere_control():
    sphere_pub = rospy.Publisher('gazebo/set_model_state', ModelState, queue_size=10)
    rospy.init_node('iris_fpv_cam')

    rospy.Subscriber("mavros_ruying/local_position/pose", PoseStamped, sphere_cb)

    interval_rate = 50
    interval_time = 1.0 / interval_rate
    rate = rospy.Rate(interval_rate)

    pose_msg = ModelState()
    pose_msg.model_name = 'unit_sphere'
    while not rospy.is_shutdown():
        pose_msg.pose.position.x = sphere_pos_x
        pose_msg.pose.position.y = sphere_pos_y
        pose_msg.pose.position.z = sphere_pos_z
        pose_msg.pose.orientation.w = sphere_orientation_w
        pose_msg.pose.orientation.x = sphere_orientation_x
        pose_msg.pose.orientation.y = sphere_orientation_y
        pose_msg.pose.orientation.z = sphere_orientation_z
        # pose_msg.pose.position.x += 4. / 30
        sphere_pub.publish(pose_msg)
        rate.sleep()
    # rospy.wait_for_service('/gazebo/set_model_state')
    # set_state_service = rospy.ServiceProxy('/gazebo/set_model_state', SetModelState)
    # objstate = SetModelStateRequest()





    # while not rospy.is_shutdown():
    #     rospy.wait_for_service('/gazebo/set_model_state')
    #     set_state_service = rospy.ServiceProxy('/gazebo/set_model_state', SetModelState)
    #     objstate = SetModelStateRequest()
    #     rospy.wait_for_service('/gazebo/set_model_state')
    #     set_state_service = rospy.ServiceProxy('/gazebo/set_model_state', SetModelState)
    #     objstate = SetModelStateRequest()
    #     # send sphere x,y,z
    #     objstate.model_state.model_name = "unit_sphere"
    #     objstate.model_state.pose.position.x = sphere_pos_x
    #     objstate.model_state.pose.position.y = sphere_pos_y
    #     objstate.model_state.pose.position.z = sphere_pos_z
    #     objstate.model_state.pose.orientation.w = sphere_orientation_w
    #     objstate.model_state.pose.orientation.x = sphere_orientation_x
    #     objstate.model_state.pose.orientation.y = sphere_orientation_y
    #     objstate.model_state.pose.orientation.z = sphere_orientation_z
    #     objstate.model_state.twist.linear.x = 0.0
    #     objstate.model_state.twist.linear.y = 0.0
    #     objstate.model_state.twist.linear.z = 0.0
    #     objstate.model_state.twist.angular.x = 0.0
    #     objstate.model_state.twist.angular.y = 0.0
    #     objstate.model_state.twist.angular.z = 0.0
    #     objstate.model_state.reference_frame = "world"
    #
    #     result = set_state_service(objstate)
    #     rate.sleep()




if __name__ == '__main__':
    # talker()
    sphere_control()
