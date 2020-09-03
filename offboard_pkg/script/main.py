#!/usr/bin/env python
#coding=utf-8

import rospy
import numpy as np
import math
import threading
from geometry_msgs.msg import *
from std_srvs.srv import Empty
from mavros_msgs.srv import CommandBool
from mavros_msgs.srv import SetMode
from mavros_msgs.msg import State, RCIn, HomePosition
from flow import StateMachine
from utils import Utils
from Queue import Queue

current_state = State()
ch5, ch6, ch7, ch8, ch9 = -1, -1, -1, -1, -1
is_initialize_1, is_initialize_2, is_initialize_3, is_initialize_4, is_initialize_5, is_initialize_6, is_initialize_7, is_initialize_8 = False, False, False, False, False, False, False, False
mav_pos = [0, 0, 0]
mav_vel = [0, 0, 0]
mav_yaw = 0
mav_R = np.zeros((3,3))
mav_home_pos = [0, 0, 0]
mav_home_yaw = 0
mav_home_geo = [0, 0, 0]
car_pos = [0, 0, 0]
car_vel = [0, 0, 0]
car_yaw = 0
car_home_pos = [0, 0, 0]
car_home_yaw = 0
car_home_geo = [0, 0, 0]
pos_i = [-1, -1, -1]
car_velocity = 2
state_name = "InitializeState"
command = TwistStamped()
q = Queue()
maxQ = 100
sumQ = 0.0
# follow_mode: 0, ll=follow_distance; 1, ll=norm(car_home, mav_home)
follow_mode = 0
follow_distance = 2
u = Utils()
home_dx, home_dy = 0, 0
FLIGHT_H = 3

def spin():
    rospy.spin()

def state_cb(msg):
    global current_state
    current_state = msg

def mav_pose_cb(msg):
    global mav_pos, mav_yaw, mav_R, is_initialize_1
    is_initialize_1 = True
    mav_pos = [msg.pose.position.x, msg.pose.position.y, msg.pose.position.z]
    q0, q1, q2, q3 = msg.pose.orientation.w, msg.pose.orientation.x, msg.pose.orientation.y, msg.pose.orientation.z
    mav_yaw = math.atan2(2*(q0*q3+q1*q2), 1-2*(q2*q2+q3*q3))
    mav_R = np.array([[q0**2+q1**2-q2**2-q3**2, 2*(q1*q2-q0*q3), 2*(q1*q3+q0*q2)],
                      [2*(q1*q2+q0*q3), q0**2-q1**2+q2**2-q3**2, 2*(q2*q3-q0*q1)],
                      [2*(q1*q3-q0*q2), 2*(q2*q3+q0*q1), q0**2-q1**2-q2**2+q3**2]])

def mav_vel_cb(msg):
    global mav_vel, is_initialize_2
    is_initialize_2 = True
    mav_vel = [msg.twist.linear.x, msg.twist.linear.y, msg.twist.linear.z]

def car_pose_cb(msg):
    global car_pos, car_yaw, q, sumQ, is_initialize_3
    is_initialize_3 = True
    car_pos = [msg.pose.position.x, msg.pose.position.y, msg.pose.position.z]
    q0, q1, q2, q3 = msg.pose.orientation.w, msg.pose.orientation.x, msg.pose.orientation.y, msg.pose.orientation.z
    yaw = math.atan2(2*(q0*q3+q1*q2), 1-2*(q2*q2+q3*q3))
    if q.qsize() < maxQ:
        sumQ += yaw
        q.put(yaw)
        car_yaw = sumQ / q.qsize()
    else:
        sumQ += yaw
        q.put(yaw)
        first_yaw = q.get()
        sumQ -= first_yaw
        car_yaw = sumQ / maxQ
    # car_yaw = yaw

def car_vel_cb(msg):
    global car_vel, is_initialize_4
    is_initialize_4 = True
    car_vel = [msg.twist.linear.x, msg.twist.linear.y, msg.twist.linear.z]

def rcin_cb(msg):
    global ch5, ch6, ch7, ch8, ch9, is_initialize_5
    is_initialize_5 = True
    last_ch5, last_ch6, last_ch7, last_ch8, last_ch9 = ch5, ch6, ch7, ch8, ch9
    chs = msg.channels
    ch5 = 2 if chs[4] < 1300 else 1 if chs[4] < 1700 else 0
    ch6 = 2 if chs[5] < 1300 else 1 if chs[5] < 1700 else 0
    ch7 = 2 if chs[6] < 1300 else 1 if chs[6] < 1700 else 0
    ch8 = 2 if chs[7] < 1300 else 1 if chs[7] < 1700 else 0
    ch9 = 2 if chs[8] < 1300 else 1 if chs[8] < 1700 else 0
    if ch5!=last_ch5 or ch6!=last_ch6 or ch7!=last_ch7 or ch8!=last_ch8 or ch9!=last_ch9:
        print("ch5: {}, ch6: {}, ch7: {}, ch8: {}, ch9: {}".format(ch5, ch6, ch7, ch8, ch9))

def pos_image_cb(msg):
    global is_initialize_6, pos_i
    is_initialize_6 = True
    pos_i = [msg.vector.x, msg.vector.y, msg.vector.z]

def mav_home_cb(msg):
    global is_initialize_7, mav_home_pos, mav_home_yaw, mav_home_geo, state_name
    is_initialize_7 = True
    if state_name == "InitializeState" or state_name == "IdleState":
        mav_home_pos = [msg.position.x, msg.position.y, msg.position.z]
        q0, q1, q2, q3 = msg.orientation.w, msg.orientation.x, msg.orientation.y, msg.orientation.z
        mav_home_yaw = math.atan2(2*(q0*q3+q1*q2), 1-2*(q2*q2+q3*q3))
        mav_home_geo = [msg.geo.latitude, msg.geo.longitude, msg.geo.altitude]

def car_home_cb(msg):
    global is_initialize_8, car_home_pos, car_home_yaw, car_home_geo, state_name
    is_initialize_8 = True
    if state_name == "InitializeState" or state_name == "IdleState":
        car_home_pos = [msg.position.x, msg.position.y, msg.position.z]
        q0, q1, q2, q3 = msg.orientation.w, msg.orientation.x, msg.orientation.y, msg.orientation.z
        car_home_yaw = math.atan2(2*(q0*q3+q1*q2), 1-2*(q2*q2+q3*q3))
        car_home_geo = [msg.geo.latitude, msg.geo.longitude, msg.geo.altitude]

def minAngleDiff(a, b):
    diff = a - b
    if diff < 0:
        diff += 2*np.pi
    if diff < np.pi:
        return diff
    else:
        return diff - 2*np.pi

def angleLimiting(a):
    if a > np.pi:
        return a - 2*np.pi
    if a < -np.pi:
        return a + 2*np.pi
    return a


if __name__=="__main__":
    rospy.init_node('offb_node', anonymous=True)
    spin_thread = threading.Thread(target = spin)
    spin_thread.start()

    rospy.Subscriber("mavros/state", State, state_cb)
    rospy.Subscriber("mavros/local_position/pose", PoseStamped, mav_pose_cb)
    rospy.Subscriber("mavros/local_position/velocity_local", TwistStamped, mav_vel_cb)
    rospy.Subscriber("mavros_ruying/local_position/pose", PoseStamped, car_pose_cb)
    rospy.Subscriber("mavros_ruying/local_position/velocity_local", TwistStamped, car_vel_cb)
    rospy.Subscriber("mavros/rc/in", RCIn, rcin_cb)
    rospy.Subscriber("tracker/pos_image", Vector3Stamped, pos_image_cb)
    rospy.Subscriber("mavros/home_position/home", HomePosition, mav_home_cb)
    rospy.Subscriber("mavros_ruying/home_position/home", HomePosition, car_home_cb)
    local_pos_pub = rospy.Publisher('mavros/setpoint_position/local', PoseStamped, queue_size=10)
    local_vel_pub = rospy.Publisher('mavros/setpoint_velocity/cmd_vel', TwistStamped, queue_size=10)
    print("Publisher and Subscriber Created")

    rospy.wait_for_service("tracker/save_img")
    save_client = rospy.ServiceProxy("tracker/save_img", Empty)
    rospy.wait_for_service("mavros/cmd/arming")
    arming_client = rospy.ServiceProxy('mavros/cmd/arming', CommandBool)
    rospy.wait_for_service("mavros/set_mode")
    set_mode_client = rospy.ServiceProxy('mavros/set_mode', SetMode)
    print("Clients Created")
    rate = rospy.Rate(20)
    
    # ensure the connection 
    while(not current_state.connected):
        print(current_state.connected)
        rate.sleep()
    
    # set position here. 
    # Usually send a few times before entering into the offboard mode
    print("Creating pose")
    pose = PoseStamped()
    pose.pose.position.x = 20
    pose.pose.position.y = 0
    pose.pose.position.z = 10
    pose.pose.orientation.x = 0
    pose.pose.orientation.y = 0
    pose.pose.orientation.z = math.sqrt(2) / 2
    pose.pose.orientation.w = math.sqrt(2) / 2

    for i in range(100):
        local_vel_pub.publish(command)
        rate.sleep()
        
    # switch into offboard
    print("Creating Objects for services")
    offb_set_mode = SetMode()
    offb_set_mode.custom_mode = "OFFBOARD"
    arm_cmd = CommandBool()
    arm_cmd.value = True

    # Instantiate StateMachine object
    geo_fence = [-1000000,-1000000, 2000000, 1000000]
    sm = StateMachine(geo_fence)

    last_request = rospy.Time.now()

    # start
    cnt = -1
    while not rospy.is_shutdown():
        cnt += 1
        state_name = sm.state_name
        if cnt % 10 == 0:
            print("state_name: {}".format(state_name))
        is_initialize_finish = is_initialize_1 and is_initialize_2 and is_initialize_3 and is_initialize_4 and is_initialize_5 and is_initialize_6 and is_initialize_7 and is_initialize_8
        if not is_initialize_finish:
            print("mav_pose_cb: {}, mav_vel_cb: {}, car_pose_cb: {}, car_vel_cb: {}, rcin_cb: {}, pos_image_cb: {}, mav_home_cb: {}, car_home_cb: {}".format(is_initialize_1, is_initialize_2, is_initialize_3, is_initialize_4, is_initialize_5, is_initialize_6, is_initialize_7, is_initialize_8))
        else:
            if cnt % 100 == 0:
                print("is_initialize all True")

        if ch9 == 0 or ch9 == 2:
            try: 
                save_client()
            except rospy.ServiceException, e:
                print("/tracker/save_img service call failed")
        if ch8 == 0:
            if current_state.mode == "OFFBOARD":
                resp1 = set_mode_client(0, "POSCTL")	# (uint8 base_mode, string custom_mode)
            if cnt % 10 == 0:
                print("Enter MANUAL mode")
            rate.sleep()
            continue
        delta_time = rospy.Time.now() - last_request
        if delta_time < rospy.Duration(5):
            local_vel_pub.publish(command)
            rate.sleep()
            continue
        else:
            if current_state.mode != "OFFBOARD":
                resp1 = set_mode_client( 0,offb_set_mode.custom_mode )
                if resp1.mode_sent:
                    print("Offboard enabled")
                last_request = rospy.Time.now()
            # elif (state_name == "InitializeState" or state_name == "IdleState") and (not current_state.armed):
            #     arm_client_1 = arming_client(arm_cmd.value)
            #     if arm_client_1.success:
            #         print("Vehicle armed")
            #     last_request = rospy.Time.now()
            # else:
            #     pass

        # dlt_home_pos = np.array(car_home_pos) - np.array(mav_home_pos)
        # home_dx, home_dy = u.GeoToDis(mav_home_geo, car_home_geo)
        # print("mav_home_geo: {}, car_home_geo: {}, home_dx: {}, home_dy: {}".format(mav_home_geo, car_home_geo, home_dx, home_dy))
        # ll = follow_distance if follow_mode == 0 else np.linalg.norm([home_dx, home_dy])
        # print("ll: {}".format(ll))
        dlt_home_yaw = minAngleDiff(car_home_yaw, mav_home_yaw)
        # dlt_pos = np.array(car_pos) - np.array(mav_pos) - dlt_home_pos
        car_yaw_cor = angleLimiting(car_yaw - dlt_home_yaw)
        mav_local = np.array(mav_pos) - np.array(mav_home_pos)
        dif_car_mav_pos = u.GeoToENU(mav_home_geo, car_home_geo)
        ll = follow_distance if follow_mode == 0 else np.linalg.norm([dif_car_mav_pos[0], dif_car_mav_pos[1]])
        car_local = np.array(car_pos) - np.array(car_home_pos)
        virtual_car_pos = np.array([-ll*np.cos(car_yaw_cor), -ll*np.sin(car_yaw_cor), 0])
        dlt_pos_raw = -mav_local + dif_car_mav_pos + car_local + virtual_car_pos
        dlt_pos = np.array([dlt_pos_raw[0], dlt_pos_raw[1], FLIGHT_H-(mav_pos[2]-mav_home_pos[2])])
        print("dlt_home_yaw: {}\ncar_yaw_cor: {}\nmav_pos: {}\nmav_home_pos: {}\nmav_local: {}\ncar_home_geo: {}\nmav_home_geo: {}\ndif_car_mav_pos: {}\nll: {}\ncar_pos: {}\ncar_home_pos: {}\ncar_local: {}\nvirtual_car_pos: {}\ndlt_pos: {}".format(dlt_home_yaw, car_yaw_cor, mav_pos, mav_home_pos, mav_local, car_home_geo, mav_home_geo, dif_car_mav_pos, ll, car_pos, car_home_pos, car_local, virtual_car_pos, dlt_pos))

        dlt_vel = np.array(car_vel) - np.array(mav_vel)
        dlt_yaw = minAngleDiff(car_yaw_cor, mav_yaw)
        keys = [ch5, ch6, ch7, ch8]
        pos_info = {"mav_pos": mav_pos, "mav_vel": mav_vel, "mav_yaw": mav_yaw, "mav_R": mav_R, "mav_home_pos": mav_home_pos, "mav_home_yaw": mav_home_yaw, "car_home_pos": car_home_pos, "rel_pos": dlt_pos, "rel_vel": dlt_vel, "rel_yaw": dlt_yaw}
        print("car_yaw_cor: {}, mav_yaw: {}".format(car_yaw_cor, mav_yaw))

        cmd = sm.update(keys, is_initialize_finish, pos_info, pos_i, car_velocity)
        print("cmd: {}\n".format(cmd))
        if cmd is not None:
            if cmd == "failed":
                command.twist.linear.x = 0
                command.twist.linear.y = 0
                command.twist.linear.z = 0
                command.twist.angular.z = 0
            else:
                command.twist.linear.x = cmd[0]
                command.twist.linear.y = cmd[1]
                command.twist.linear.z = u.SaftyZ(cmd[2])
                command.twist.angular.z = cmd[3]
        else:
            command.twist.linear.x = 0
            command.twist.linear.y = 0
            command.twist.linear.z = 0
            command.twist.angular.z = 0
        local_vel_pub.publish(command)
        print(command.twist)
        rate.sleep()
    rospy.spin()
