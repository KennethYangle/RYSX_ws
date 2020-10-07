#!/usr/bin/env python
#coding=utf-8

import numpy as np
import time
from utils import Utils

# 初始化状态
class InitializeState(object):
    def __init__(self, stateMachine):
        self.stateMachine = stateMachine
        self.state_name = "InitializeState"
    def waitForInitialize(self):
        time.sleep(1)
    def initializeFinished(self):
        self.stateMachine.setState(self.stateMachine.getIdleState())
    def takeoff(self, pos_info):
        print("cannot takeoff, waiting for initialize.")
    def approach(self, pos_info, pos_i, depth, car_velocity):
        print("Warning , waiting for initialize.")
    def go_home(self, pos_info):
        self.stateMachine.setState(self.stateMachine.getHomewardState())
    def flightInward(self, pos, geo_fence):
        print("Error! The starting position is outside the geographic fence. Closed mission.")
        return "failed"
    def failed(self):
        self.stateMachine.setState(self.stateMachine.getFailedState())


# 空闲状态
class IdleState(object):
    def __init__(self, stateMachine):
        self.stateMachine = stateMachine
        self.state_name = "IdleState"
    def initializeFinished(self):
        pass
    def takeoff(self, pos_info):
        self.stateMachine.setState(self.stateMachine.getTakeoffState())
    def approach(self, pos_info, pos_i, depth, car_velocity):
        # if pos_i > 1:
        #     self.stateMachine.setState(self.stateMachine.getDockingState())
        self.stateMachine.setState(self.stateMachine.getDockingState())
    def go_home(self, pos_info):
        self.stateMachine.setState(self.stateMachine.getHomewardState())
    def flightInward(self, pos, geo_fence):
        print("Error! The starting position is outside the geographic fence. Closed mission.")
        return "failed"
    def failed(self):
        self.stateMachine.setState(self.stateMachine.getFailedState())


# 起飞状态
class TakeoffState(object):
    def __init__(self, stateMachine):
        self.stateMachine = stateMachine
        self.state_name = "TakeoffState"
    def initializeFinished(self):
        pass
    def takeoff(self, pos_info):
        """
        return: [vel_cmd], is_takeoff_finish
        """
        # sendTakeoffCmdAsyn()
        if pos_info["mav_pos"][2] < 2:
            return [0,0,1,0]
        else:
            self.stateMachine.setState(self.stateMachine.getDockingState())
    def go_home(self, pos_info):
        self.stateMachine.setState(self.stateMachine.getHomewardState())
    def flightInward(self, pos, geo_fence):
        self.stateMachine.setState(self.stateMachine.getGeographicalFenceState())
        print("Warning! The takeoff point is at the geographic fence boundary.")
    def failed(self):
        self.stateMachine.setState(self.stateMachine.getFailedState())


# 对接状态
class DockingState(object):
    def __init__(self, stateMachine):
        self.stateMachine = stateMachine
        self.state_name = "DockingState"
    def initializeFinished(self):
        pass
    def takeoff(self, pos_info):
        pass
    def go_home(self, pos_info):
        self.stateMachine.setState(self.stateMachine.getHomewardState())
    def approach(self, pos_info, pos_i, depth, car_velocity):
        cmd = self.stateMachine.util.DockingControllerFusion(pos_info, pos_i, depth, car_velocity)
        return cmd
    def flightInward(self, pos, geo_fence):
        print("Warning! Out of the geographic fence during the docking.")
        self.stateMachine.setState(self.stateMachine.getGeographicalFenceState())
    def failed(self):
        self.stateMachine.setState(self.stateMachine.getFailedState())


# 返航状态
class HomewardState(object):
    def __init__(self, stateMachine):
        self.stateMachine = stateMachine
        self.state_name = "HomewardState"
        self.homeward = True
    def initializeFinished(self):
        pass
    def takeoff(self, pos_info):
        pass
    def go_home(self, pos_info):
        if self.homeward and np.linalg.norm(np.array(pos_info["mav_pos"]) - np.array(pos_info["mav_home_pos"]) - np.array([0, 0, 2])) >= 1:
            cmd = self.stateMachine.util.PostionController(pos_info)
            return cmd
        elif np.linalg.norm(np.array(pos_info["mav_vel"])) > 2:
            return [0,0,0,0]
        else:
            self.homeward = False
            return [0,0,-1,0]
    def flightInward(self, pos, geo_fence):
        self.stateMachine.setState(self.stateMachine.getGeographicalFenceState())
    def failed(self):
        self.stateMachine.setState(self.stateMachine.getFailedState())


# 地理围栏状态
class GeographicalFenceState(object):
    def __init__(self, stateMachine):
        self.stateMachine = stateMachine
        self.state_name = "GeographicalFenceState"
    def initializeFinished(self):
        pass
    def takeoff(self, pos_info):
        pass
    def go_home(self, pos_info):
        self.stateMachine.setState(self.stateMachine.getHomewardState())
    def flightInward(self, pos, geo_fence):
        cmd = np.array([0,0,0])
        if pos[0] < geo_fence[0]:
            cmd += np.array([20,0,0])
        elif pos[0] > geo_fence[2]:
            cmd += np.array([-20,0,0])
        else:
            pass
        if pos[1] < geo_fence[1]:
            cmd += np.array([0,20,0])
        elif pos[1] > geo_fence[3]:
            cmd += np.array([0,-20,0])
        else:
            pass
        return [cmd[0], cmd[1], cmd[2], 0]
    def failed(self):
        self.stateMachine.setState(self.stateMachine.getFailedState())


# 失败状态
class FailedState(object):
    def __init__(self, stateMachine):
        self.stateMachine = stateMachine
        self.state_name = "FailedState"
    def failed(self):
        print("mission failed!")
        return "failed"


# 状态机
class StateMachine(object):
    def __init__(self, geo_fence, setting):
        self.geo_fence = geo_fence
        self.setting = setting
        self.outrange_cnt = 0
        self.outrange_cnt2 = 0
        self.outrange_th = 100

        self.ch5 = -1
        self.ch6 = -1
        self.ch7 = -1
        self.ch8 = -1
        self.is_initialize_finish = False

        self.start_key = -1
        self.homeward_key = -1

        self.initialize_state = InitializeState(self)
        self.idle_state = IdleState(self)
        self.takeoff_state = TakeoffState(self)
        self.docking_state = DockingState(self)
        self.homeward_state = HomewardState(self)
        self.geographical_fence_state = GeographicalFenceState(self)
        self.failed_state = FailedState(self)

        self.state = self.initialize_state
        self.state_name = "InitializeState"
        self.last_state = None

        self.util = Utils(self.setting["Utils"])

    def reset(self):
        self.state = self.initialize_state
        self.state_name = "InitializeState"
        self.last_state = None
        self.util = Utils(self.setting["Utils"])

    def getInitializeState(self):
        return self.initialize_state
    def getIdleState(self):
        return self.idle_state
    def getTakeoffState(self):
        return self.takeoff_state
    def getDockingState(self):
        return self.docking_state
    def getHomewardState(self):
        return self.homeward_state
    def getGeographicalFenceState(self):
        return self.geographical_fence_state
    def getFailedState(self):
        return self.failed_state
    def setState(self, state):
        self.state = state
        self.state_name = state.state_name

    def update(self, keys, is_initialize_finish, pos_info, pos_i, depth, depth_left, depth_right, car_velocity):
        """
        - keys: 按键状态
        - is_initialize_finish: externally subscribed
        - pos_info: a dict contain absolute position and relative position, 
        {"mav_pos": mav_pos, "mav_vel": mav_vel, "mav_yaw": mav_yaw, "mav_home_pos": mav_home_pos, "car_home_pos": car_home_pos, "rel_pos": dlt_pos, "rel_vel": dlt_vel, "rel_yaw": dlt_yaw}
        - pos_i: i_x, i_y, bbox_w, bbox_h, confidence
        - car_velocity: use the velocity of car as base value
        """
        self.ch5, self.ch6, self.ch7, self.ch8 = keys
        self.start_key = self.ch6
        self.homeward_key = self.ch7
        self.offboard_key = self.ch8

        if not self.util.IsInFence(pos_info["mav_pos"], self.geo_fence):
            print(pos_info["mav_pos"])
            self.outrange_cnt += 1
            if self.outrange_cnt > self.outrange_th:
                self.outrange_cnt2 += 1
                if self.outrange_cnt2 > 10*self.outrange_th:
                    return self.state.failed()
                # 储存上一状态
                if self.state_name != "GeographicalFenceState":
                    self.last_state = self.state
                return self.state.flightInward(pos_info["mav_pos"], self.geo_fence)
        else:
            self.outrange_cnt = 0
            self.outrange_cnt2 = 0
            # 回到上一状态
            if self.state_name == "GeographicalFenceState":
                self.setState(self.last_state)

        if self.homeward_key >= 1:
            return self.state.go_home(pos_info)
        self.is_initialize_finish = is_initialize_finish

        if self.state_name == "InitializeState":
            if not self.is_initialize_finish:
                self.state.waitForInitialize()
            else:
                self.state.initializeFinished()

        if self.start_key >= 1:
            return self.state.approach(pos_info, pos_i, depth, car_velocity)
