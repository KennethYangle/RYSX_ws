#!/usr/bin/env python
#coding=utf-8

import os
import json
import numpy as np
import sys
import rospy
#导入自定义的数据类型
from geometry_msgs.msg import PoseStamped, TwistStamped
from std_msgs.msg import Float32MultiArray
from mavros_msgs.msg import HomePosition
sys.path.append("/home/wsh/RYSX_ws/src/offboard_pkg/script")
from utils import Utils

# JMAVSIM or GAZEBO
SIM_MODE = "GAZEBO"

def talker():
    car_pos_pub = rospy.Publisher("mavros_ruying/local_position/pose", PoseStamped, queue_size=10)
    car_vel_pub = rospy.Publisher("mavros_ruying/local_position/velocity_local", TwistStamped, queue_size=10)
    pos_image_pub = rospy.Publisher("tracker/pos_image", Float32MultiArray, queue_size=10)
    car_home_pub = rospy.Publisher("mavros_ruying/home_position/home", HomePosition, queue_size=10)
    rospy.init_node('pub_node', anonymous=True)
    
    interval_rate = 50
    interval_time = 1.0 / interval_rate
    rate = rospy.Rate(interval_rate) 
    
    car_pos = PoseStamped()
    car_vel = TwistStamped()
    pos_image = Float32MultiArray()
    car_yaw = np.pi/3

    car_vel.twist.linear.y = 0
    car_vel.twist.angular.z = 0
    car_pos.pose.position.x = 1.2
    car_pos.pose.position.z = 2
    car_pos.pose.orientation.w = np.cos(car_yaw/2)
    car_pos.pose.orientation.x = 0
    car_pos.pose.orientation.y = 0
    car_pos.pose.orientation.z = np.sin(car_yaw/2)
    pos_image.data = [0, 0, 0, 0, 0]

    mav_home = HomePosition()
    mav_home.geo.latitude = 47.3977422
    mav_home.geo.longitude = 8.5455935
    mav_home.geo.altitude = 535.3059164018949
    car_home = HomePosition()
    car_home.geo = mav_home.geo
    # car_home.position.x = car_pos.pose.position.x
    # car_home.position.y = car_pos.pose.position.y
    # car_home.position.z = car_pos.pose.position.z
    car_home.orientation.w = car_pos.pose.orientation.w
    car_home.orientation.x = car_pos.pose.orientation.x
    car_home.orientation.y = car_pos.pose.orientation.y
    car_home.orientation.z = car_pos.pose.orientation.z
    dlt_home_pos = u.GeoToENU([mav_home.geo.latitude, mav_home.geo.longitude, mav_home.geo.altitude], [car_home.geo.latitude, car_home.geo.longitude, car_home.geo.altitude])
    print("dlt_home_pos: {}".format(dlt_home_pos))
    # car_pos.pose.position.x -= dlt_home_pos[1]
    # car_pos.pose.position.y -= dlt_home_pos[0]
    # car_pos.pose.position.z -= dlt_home_pos[2]

    cnt = 0
    while not rospy.is_shutdown():
        if cnt < 500:
            car_vel.twist.linear.x = 0
        elif cnt < 6000:
            car_vel.twist.linear.x = 1
        elif cnt < 6200:
            car_vel.twist.linear.x = 0
            car_yaw = -2*np.pi/3
        else:
            car_vel.twist.linear.x = -1
        #计算距离
        car_pos.pose.position.x += car_vel.twist.linear.x * interval_time
        car_pos.pose.position.y += car_vel.twist.linear.y * interval_time
        car_pos.pose.position.z += car_vel.twist.linear.z * interval_time
        car_yaw += car_vel.twist.angular.z * interval_time
        # car_yaw = 0.2*np.random.rand(1)[0]-0.1
        car_pos.pose.orientation.w = np.cos(car_yaw/2)
        car_pos.pose.orientation.x = 0
        car_pos.pose.orientation.y = 0
        car_pos.pose.orientation.z = np.sin(car_yaw/2)
        car_pos_pub.publish(car_pos)
        car_vel_pub.publish(car_vel)
        if SIM_MODE == "JMAVSIM":
            pos_image_pub.publish(pos_image)
        if cnt % 100 == 0:
            car_home_pub.publish(car_home)
        cnt += 1
        rate.sleep()

if __name__ == '__main__':
    setting_file = open(os.path.join(os.path.expanduser('~'),"RYSX_ws/src","settings.json"))
    setting = json.load(setting_file)
    u = Utils(setting["Utils"])

    talker()