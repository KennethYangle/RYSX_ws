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
        self.FLIGHT_H = 3
        self.saftyz = 0.3
        self.USE_CAM_FOR_X = True
        self.GPS_SLIDING = False
        self.we_gps = np.array([[0.5, 0, 0], [0, 0.5, 0], [0, 0, 0.5]])
        self.we_realsense = 1.0
        self.wedt_realsense = 0.1
        self.Kp = np.array([[0.5, 0, 0], [0, 0.5, 0], [0, 0, 0.5]])
        self.Kp_nu_cam = np.array([[0.005, 0, 0], [0, 0.005, 0], [0, 0, 0.005]])
        self.cam_offset = np.array([0, 0, 0])
        self.Kp_yaw_cam = 0.00001
        self.rpos_est_k = np.array([0, 0, 0])
        self.cam_offset = np.array([0, 0, 0]) #cam distance to center
        self.rpos_init = False

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

    def DockingControllerFusion(self, pos_info, pos_i, depth, car_velocity):
        if pos_info["mav_pos"] == 0:
            return [0,0,0,0]

        rpos_est = pos_info["dlt_mav_car_gps_enu"]
        if not self.rpos_init:
            rpos_est_k = rpos_est
            self.rpos_init = True

        cam_is_ok = False
        realsense_is_ok = False
        dt = 0.05 # time interval
        track_quality = pos_i[2] #pos_i[2] is quality, range from 0 to 1
        if np.linalg.norm([rpos_est[0], rpos_est[1]]) < 5 and track_quality > 0.6:
            cam_is_ok = True
        if depth > 0:
            realsense_is_ok = True
        if self.GPS_SLIDING:
            rpos_est = rpos_est_k + we_gps.dot(pos_info["dlt_mav_car_gps_enu"] - rpos_est_k)
        else:
            rpos_est = pos_info["dlt_mav_car_gps_enu"]

        # if realsense_is_ok:
        #     # body: right-front-up
        #     rpos_est_body = pos_info["mav_R"].T.dot(rpos_est)
        #     rpos_est_body[1] = rpos_est_body[1] + track_quality*(self.we_realsense*(depth - rpos_est_body[1] + self.wedt_realsense*(depth - rpos_est_body[1]*dt)))
        #     rpos_est = pos_info["mav_R"].dot(rpos_est_body)
        
        ref_vel_enu = self.Kp.dot(rpos_est + pos_info["virtual_car_pos"]) + self.D*np.array(pos_info["rel_vel"])
        print("ref_vel_enu: {}".format(ref_vel_enu))
        if cam_is_ok:
            i_err = np.array([pos_i[0] - self.WIDTH/2, pos_i[1] - self.HEIGHT/2, 0])
            ref_vel_cam_cam = self.Kp_nu_cam.dot(i_err)
            ref_vel_cam_enu = pos_info["mav_R"].dot(pos_info["R_bc"].dot(ref_vel_cam_cam) + self.cam_offset)
            if not self.USE_CAM_FOR_X:
                ref_vel_cam_enu[0] = 0
            ref_vel_cam_enu[1] = 0
            print("ref_vel_cam_enu: {}".format(ref_vel_cam_enu))
            ref_vel_enu = ref_vel_enu + track_quality * ref_vel_cam_enu

        cmd_yawrate = self.sat(self.P*pos_info["rel_yaw"], 2)
        if cam_is_ok:
            yaw_cam = -i_err[0]
            cmd_yawrate = cmd_yawrate + self.Kp_yaw_cam*yaw_cam

        cmd_vel = self.sat(ref_vel_enu, 3*car_velocity)
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
        cmd_vel = self.sat(self.P*np.array(pos_info["rel_pos"]) + self.D*np.array(pos_info["rel_vel"]), 3*car_velocity)
        cmd_yawrate = self.sat(self.P*pos_info["rel_yaw"], 2)
        if pos_i[0] > 0:
            v_zi = self.P_i * (pos_i[1] - self.HEIGHT/2)
            print("pos_i: {}".format(pos_i))
            # return [cmd_vel[0], cmd_vel[1], v_zi, cmd_yawrate]
            return [cmd_vel[0], cmd_vel[1], cmd_vel[2], cmd_yawrate]
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