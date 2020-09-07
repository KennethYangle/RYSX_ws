#!/usr/bin/env python
#coding=utf-8

import numpy as np
import rospy
#导入自定义的数据类型
from geometry_msgs.msg import PoseStamped, TwistStamped, Vector3Stamped
from gazebo_msgs.srv import *
from gazebo_msgs.msg import ModelStates

img_recongition = False
sphere_x,sphere_y,sphere_z = 0,0,0
mav_x,mav_y,mav_z = 0,0,0

def spin():
    rospy.spin()

def depth_cb(msg):
    global img_recongition
    if msg.vector.z == 0:
        img_recongition = False
    else:
        img_recongition = True
    # print(img_recongition)

def states_cb(msg):
    global sphere_x,sphere_y,sphere_z
    global mav_x, mav_y, mav_z
    # sphere_msg = ModelState()
    # sphere_msg.model_name = 'unit_sphere'
    sphere_x = msg.pose[0].position.x
    sphere_y = msg.pose[0].position.y
    sphere_z = msg.pose[0].position.z
    mav_x = msg.pose[3].position.x
    mav_y = msg.pose[3].position.y
    mav_z = msg.pose[3].position.z

    # print("sphere_pose_cb: {}, sphere_vel_cb: {}, sphere_pose_cb: {}".format(sphere_x, sphere_y, sphere_z))
    # print("mav_pose_cb: {}, mav_vel_cb: {}, mav_pose_cb: {}".format(mav_x, mav_y, mav_z))
    # print(sphere_x, sphere_y, sphere_z)
    # print(0, 0, 0)


# def uav_cb(msg):
#     global mav_x,mav_y,mav_z
#     # mav_msg = ModelState()
#     # mav_msg.model_name = 'iris_fpv_cam'
#     mav_x = msg.pose.position.x
#     mav_y = msg.pose.position.y
#     mav_z = msg.pose.position.z
#     # print("mav_pose_cb: {}, mav_vel_cb: {}, mav_pose_cb: {}".format(mav_x, mav_y, mav_z))
#     print(mav_x, mav_y, mav_z)

def listener_pose():
    sphere_depth_pub = rospy.Publisher("tracker/depth", PoseStamped, queue_size=10)  # send sphere postion
    # sphere_depth = PoseStamped()
    # rospy.init_node('iris_fpv_cam', anonymous=True)

    rospy.Subscriber('tracker/pos_image', Vector3Stamped, depth_cb)

    # sphere_msg = ModelState()
    # sphere_msg.model_name = 'unit_sphere'
    rospy.Subscriber('gazebo/model_states', ModelStates, states_cb)
    rospy.init_node('iris_fpv_cam', anonymous=True)

    sphere_depth = PoseStamped()
    #
    # mav_msg = ModelState()
    # mav_msg.model_name = 'iris_fpv_cam'
    # rospy.Subscriber('gazebo/model_states', mav_msg, uav_cb)

    print('ok')

    interval_rate = 50
    interval_time = 1.0 / interval_rate
    rate = rospy.Rate(interval_rate)

    while not rospy.is_shutdown():
        dx = (sphere_x - mav_x) ** 2
        dy = (sphere_y - mav_y) ** 2
        dz = (sphere_z - mav_z) ** 2
        dis_uavtosphere = (dx + dy + dz) ** 0.5

        if dis_uavtosphere < 5 and img_recongition == True:
            #sphere_depth.pose.position.w = dis_uavtosphere
            sphere_depth.pose.position.x = dis_uavtosphere
            sphere_depth.pose.position.y = dis_uavtosphere
            sphere_depth.pose.position.z = dis_uavtosphere
        else:
            #sphere_depth.pose.position.w = -1
            sphere_depth.pose.position.x = -1
            sphere_depth.pose.position.y = -1
            sphere_depth.pose.position.z = -1

        sphere_depth_pub.publish(sphere_depth)
        print("dis_uavtosphere: {}".format(sphere_depth))
        rate.sleep()

# while not rospy.is_shutdown():
    #     get_state_service = rospy.ServiceProxy('/gazebo/get_model_state', GetModelState)
    #     # sphere position
    #     model_sphere = GetModelStateRequest()
    #     model_sphere.model_name = 'unit_sphere'
    #     objstate_model_sphere = get_state_service(model_sphere)
    #     state_sphere = (objstate_model_sphere.pose.position.x, objstate_model_sphere.pose.position.y,
    #                     objstate_model_sphere.pose.position.z)
    #     # print(state_sphere)
    #
    #     model_mav = GetModelStateRequest()
    #     model_mav.model_name = 'iris_fpv_cam'
    #     objstate_model_mav = get_state_service(model_mav)
    #     state_mav = (objstate_model_mav.pose.position.x, objstate_model_mav.pose.position.y, objstate_model_mav.pose.position.z)
    #     # print(state_mav)
    #     dx = (state_sphere[0] - state_mav[0]) ** 2
    #     dy = (state_sphere[1] - state_mav[1]) ** 2
    #     dz = (state_sphere[1] - state_mav[1]) ** 2
    #     dis_uavtosphere = (dx + dy + dz) ** 0.5
    #
    #     interval_rate = 50
    #     interval_time = 1.0 / interval_rate
    #     rate = rospy.Rate(interval_rate)
    #
    #     if dis_uavtosphere < 5 and img_recongition == True:
    #         #sphere_depth.pose.position.w = dis_uavtosphere
    #         sphere_depth.pose.position.x = dis_uavtosphere
    #         sphere_depth.pose.position.y = dis_uavtosphere
    #         sphere_depth.pose.position.z = dis_uavtosphere
    #     else:
    #         #sphere_depth.pose.position.w = -1
    #         sphere_depth.pose.position.x = -1
    #         sphere_depth.pose.position.y = -1
    #         sphere_depth.pose.position.z = -1
    #
    #     # print(dis_uavtosphere)
    #
    #     sphere_depth_pub.publish(sphere_depth)
    #     rate.sleep()

if __name__ == '__main__':
    # rospy.init_node('offb_node', anonymous=True)
    # spin_thread = threading.Thread(target=spin)
    # spin_thread.start()
    listener_pose()