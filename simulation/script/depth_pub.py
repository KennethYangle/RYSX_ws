#!/usr/bin/env python
#coding=utf-8

import numpy as np
import rospy
#导入自定义的数据类型
from geometry_msgs.msg import PoseStamped, TwistStamped, Vector3Stamped
from gazebo_msgs.srv import *

img_recongition = False

def spin():
    rospy.spin()

def depth_cb(msg):
    global img_recongition
    if msg.vector.z == 0:
        img_recongition = False
    else:
        img_recongition = True
    print(img_recongition)


def listener_pose():
    sphere_depth_pub = rospy.Publisher("mavros_ruying/local_position/depth", PoseStamped, queue_size=10)  # send sphere postion
    sphere_depth = PoseStamped()
    rospy.init_node('iris_fpv_cam', anonymous=True)

    rospy.Subscriber('tracker/pos_image', Vector3Stamped, depth_cb)



    while not rospy.is_shutdown():
        get_state_service = rospy.ServiceProxy('/gazebo/get_model_state', GetModelState)
        # sphere position
        model_sphere = GetModelStateRequest()
        model_sphere.model_name = 'unit_sphere'
        objstate_model_sphere = get_state_service(model_sphere)
        state_sphere = (objstate_model_sphere.pose.position.x, objstate_model_sphere.pose.position.y,
                        objstate_model_sphere.pose.position.z)
        # print(state_sphere)

        model_mav = GetModelStateRequest()
        model_mav.model_name = 'iris_fpv_cam'
        objstate_model_mav = get_state_service(model_mav)
        state_mav = (objstate_model_mav.pose.position.x, objstate_model_mav.pose.position.y, objstate_model_mav.pose.position.z)
        # print(state_mav)
        dx = (state_sphere[0] - state_mav[0]) ** 2
        dy = (state_sphere[1] - state_mav[1]) ** 2
        dz = (state_sphere[1] - state_mav[1]) ** 2
        dis_uavtosphere = (dx + dy + dz) ** 0.5

        interval_rate = 50
        interval_time = 1.0 / interval_rate
        rate = rospy.Rate(interval_rate)

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

        # print(dis_uavtosphere)

        sphere_depth_pub.publish(sphere_depth)
        rate.sleep()



    # sphere_depth.pose.orientation.w = dis_uavtosphere
    # sphere_depth.pose.orientation.x = dis_uavtosphere
    # sphere_depth.pose.orientation.y = dis_uavtosphere
    # sphere_depth.pose.orientation.z = dis_uavtosphere


    #return sphere x,y,z     uav x,y,z       uavtosphere_distance
    # return (state_sphere[0],state_sphere[1],state_sphere[2],state_mav[0],state_mav[1],state_mav[2],dis_uavtosphere)

if __name__ == '__main__':
    # rospy.init_node('offb_node', anonymous=True)
    # spin_thread = threading.Thread(target=spin)
    # spin_thread.start()
    listener_pose()
