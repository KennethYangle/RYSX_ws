import numpy as np

class Utils(object):
    def __init__(self, P=0.5, D=0.5, P_i=0.005):
        self.P = P
        self.D = D
        self.P_i = P_i
        self.P_h = 0.005
        self.WIDTH = 640
        self.HEIGHT = 480
        self.Ea = 6378137
        self.Eb = 6356725
        self.saftyz = 0.3

    def sat(self, a, maxv):
        n = np.linalg.norm(a)
        if n > maxv:
            return a/n*maxv
        else:
            return a

    def PostionController(self, pos_info):
        cmd_vel = self.sat(self.P*(np.array(pos_info["mav_home_pos"]) + np.array([0, 0, 2]) - np.array(pos_info["mav_pos"])), 3)
        cmd_yawrate = self.sat(self.P*(pos_info["mav_home_yaw"]-pos_info["mav_yaw"]), 2)
        return [cmd_vel[0], cmd_vel[1], cmd_vel[2], cmd_yawrate]

    def DockingController(self, pos_info, pos_i, car_velocity):
        if pos_info["rel_pos"][0] > 5:
            self.P = 2
        elif pos_info["rel_pos"][0] > 2:
            self.p = 1
        elif pos_info["rel_pos"][0] > 1:
            self.P = 0.5
        else:
            self.P = 0.1
        cmd_vel = self.sat(self.P*(np.array(pos_info["rel_pos"])-np.array([0,0,-2])) + self.D*np.array(pos_info["rel_vel"]), 3*car_velocity)
        cmd_yawrate = self.sat(self.P*pos_info["rel_yaw"], 2)
        if pos_i[0] > 0:
            v_zi = self.P_i * (pos_i[1] - self.HEIGHT/2)
            print("pos_i: {}".format(pos_i))
            return [cmd_vel[0], cmd_vel[1], v_zi, cmd_yawrate]
        else:
            return [cmd_vel[0], cmd_vel[1], cmd_vel[2], cmd_yawrate]
        # if pos_i[0] > 0:
        #     i_x = self.WIDTH - pos_i[0]
        #     i_y = self.HEIGHT - pos_i[1]
        #     v_zi = self.P_i * (i_y - self.HEIGHT/2)
        #     print("pos_i: {}".format([i_x, i_y]))
        #     v_yi = self.P_h * (i_x - self.WIDTH/2)
        #     vb = np.array([0, v_yi, v_zi])
        #     ve = pos_info["mav_R"].dot(vb)
        #     ve_list = [-ve[0], -ve[1], -ve[2], 0]
        #     return ve_list
        # else:
        #     return [0, 0, 0, 0]

    def IsInFence(self, pos, geo_fence):
        if pos[0] > geo_fence[0] and pos[0] < geo_fence[2] and pos[1] > geo_fence[1] and pos[1] < geo_fence[3]:
            return True
        else:
            return False

    """ 
    start: [latitude, longitude, altitude]
    return: [distance from start point to end point in the East, ... North, altitude].
    """
    def GeoToENU(self, start, end):
        Ec = self.Eb + (self.Ea - self.Eb) * (90.0 - start[0]) / 90.0
        Ed = Ec * np.cos(start[0] * np.pi / 180)
        dx = (end[1] - start[1]) * Ed * np.pi / 180
        dy = (end[0] - start[0]) * Ec * np.pi / 180
        dz = end[2] - start[2]
        return np.array([dx, dy, dz])

    # return a safty vz
    def SaftyZ(self, vz):
        if vz > self.saftyz:
            return self.saftyz
        elif vz < -self.saftyz:
            return -self.saftyz
        else:
            return vz