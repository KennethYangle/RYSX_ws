import numpy as np
from avoidance import Avoidance

class Utils(object):
    def __init__(self, params):
        self.P = params["P"]
        self.D = params["D"]
        self.P_i = params["P_i"]
        self.P_h = params["P_h"]
        self.WIDTH = params["WIDTH"]
        self.HEIGHT = params["HEIGHT"]
        self.Ea = params["Ea"]
        self.Eb = params["Eb"]
        self.saftyz = params["saftyz"]
        self.USE_CAM_FOR_X = params["USE_CAM_FOR_X"]
        self.USE_DEPTH_FOR_X = params["USE_DEPTH_FOR_X"]
        self.GPS_SLIDING = params["GPS_SLIDING"]
        self.we_gps = np.array(params["we_gps"])
        self.we_realsense = params["we_realsense"]
        self.wedt_realsense = params["wedt_realsense"]
        self.Kp = np.array(params["Kp"])
        self.Ki = np.array(params["Ki"])
        self.integral = np.array([0, 0, 0])
        self.Kp_nu_cam = np.array(params["Kp_nu_cam"])
        self.Ki_nu_cam = np.array(params["Ki_nu_cam"])
        self.integral_cam = np.array([0, 0, 0])
        self.Kp_yaw_cam = params["Kp_yaw_cam"]
        self.rpos_est_k = np.array([0, 0, 0])
        self.cam_offset = np.array(params["cam_offset"]) # cam distance to center on body coordinate in pixel uint, if z > 0, object appears at the bottom of the image
        self.rpos_init = False
        self.track_quality_k = 0
        self.CAM_GPS_COM = params["CAM_GPS_COM"]
        self.cam_gps_err_body = np.array([0, 0, 0])
        self.cam_gps_err_up = 0
        self.cam_gps_err_kp = np.array(params["cam_gps_err_kp"])
        self.USE_GPS = params["USE_GPS"]
        self.USE_REALSENSE = params["USE_REALSENSE"]
        self.USE_CAMERA = params["USE_CAMERA"]
        self.ref_vel_cam_body = np.array([0, 0, 0])
        self.rsense_gps2_err = 0
        self.rsense_gps2_err_kp = 0.5
        self.RSENSE_GPS_COM = params["RSENSE_GPS_COM"]
        self.I_saturation_limit = params["I_saturation_limit"]
        self.Kp_lr_depth = params["Kp_lr_depth"]


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

    def DockingControllerFusion(self, pos_info, pos_i, depth, depth_left, depth_right, car_velocity):
        # GPS+IMU failed
        if pos_info["mav_pos"] == 0:
            return [0,0,0,0]

        # GPS position error for mav and car.
        dlt_mav_car_gps_enu = pos_info["dlt_mav_car_gps_enu"] 
        if not self.rpos_init:
            self.rpos_est_k = dlt_mav_car_gps_enu
            self.rpos_init = True

        cam_is_ok = False
        realsense_is_ok = False
        dt = 0.05 # time interval
        track_quality = pos_i[4] # pos_i[4] is quality, range from 0 to 1
        print("track_quality: {}".format(track_quality))
        # lowpass filter
        # if track_quality > 0.75:
        #     track_quality = 1.0
        # elif track_quality < 0.2:
        #     track_quality = 0
        self.track_quality_k = self.track_quality_k + 0.1*(track_quality - self.track_quality_k)
        print("track_quality_k: {}".format(self.track_quality_k))

        # Use camera information when tracking close and having good tracking quality.
        if self.track_quality_k > 0.3:
            cam_is_ok = True
        # When RealSense is available, the return depth is greater than 0.
        if depth > 0:
            realsense_is_ok = True

        # difference between left depth with right depth
        depth_delta = 0
        if depth_left > 0 and depth_right > 0:
            #if depth_delta >0  roll to right
            depth_delta = depth_right - depth_left
        print("depth_delta: {}".format(depth_delta))
        
        # Is use cam compensate GPS.
        if self.CAM_GPS_COM:
            dlt_mav_car_gps_body = pos_info["mav_R"].T.dot(dlt_mav_car_gps_enu)
            dlt_mav_car_gps_body[0] = dlt_mav_car_gps_body[0] - self.cam_gps_err_body[0]
            # dlt_mav_car_gps_body[2] = dlt_mav_car_gps_body[2] - self.cam_gps_err_body[2]
            print("dlt_mav_car_gps_body: {}".format(dlt_mav_car_gps_body))
            dlt_mav_car_gps_enu = pos_info["mav_R"].dot(dlt_mav_car_gps_body)
            dlt_mav_car_gps_enu[2] = dlt_mav_car_gps_enu[2] - self.cam_gps_err_up
        if self.RSENSE_GPS_COM:
            dlt_mav_car_gps_body = pos_info["mav_R"].T.dot(dlt_mav_car_gps_enu)
            dlt_mav_car_gps_body[1] = dlt_mav_car_gps_body[1] - self.rsense_gps2_err
            dlt_mav_car_gps_enu = pos_info["mav_R"].dot(dlt_mav_car_gps_body)
        # IS use a slide window to smooth the position measurement.
        if self.GPS_SLIDING:
            self.rpos_est_k = self.rpos_est_k + self.we_gps.dot(dlt_mav_car_gps_enu - self.rpos_est_k)
        else:
            self.rpos_est_k = dlt_mav_car_gps_enu
        
        # Is depth used to calibrate position estimates.
        if not self.USE_REALSENSE:
            realsense_is_ok = False
        if realsense_is_ok:
            # body: right-front-up3rpos_est_body)
            rpos_est_body = pos_info["mav_R"].T.dot(self.rpos_est_k)
            print("rpos_est_body_raw: {}".format(rpos_est_body))
            rpos_est_body[1] = rpos_est_body[1] + self.track_quality_k*self.we_realsense*(depth - rpos_est_body[1])
            self.rpos_est_k = pos_info["mav_R"].dot(rpos_est_body)

            self.rsense_gps2_err = self.rsense_gps2_err + self.rsense_gps2_err_kp*(rpos_est_body[1] - depth)
            print("rsense_gps2_err_body: {}".format(self.rsense_gps2_err))
            
        print("rpos_est: {}".format(self.rpos_est_k))
        
        # obstacle avoidance component
        rpos_est_body = pos_info["mav_R"].T.dot(self.rpos_est_k)
        print("rpos_est_body: {}".format(rpos_est_body))
        avo = Avoidance(3*car_velocity)
        avo_cmd = avo.controller(rpos_est_body)
        print("avoidance: {}".format(avo_cmd))

        # calc position error betweent mav and virtual point
        dlt_pos = self.rpos_est_k + pos_info["virtual_car_pos"]
        print("dlt_pos: {}".format(dlt_pos))
        
        dlt_pos_body = pos_info["mav_R"].T.dot(dlt_pos)
        if np.linalg.norm(pos_info["car_vel"]) > 2:
            self.integral = self.integral + self.Ki.dot(dlt_pos_body)*dt
        # integral saturation
        self.SatIntegral(self.integral, self.I_saturation_limit, -self.I_saturation_limit)

        # PID controller
        P_component = self.Kp.dot(dlt_pos_body)
        I_component = self.integral
        D_component = self.D*np.array(pos_info["mav_R"].T.dot(pos_info["rel_vel"]))
        F_component = pos_info["mav_R"].T.dot(pos_info["car_vel"])
        ref_vel_body = P_component + I_component + D_component + F_component
        print("P_component: {}\nI_component: {}\nD_component: {}\nF_component: {}".format(P_component, I_component, D_component, F_component))
        ref_vel_enu = pos_info["mav_R"].dot(ref_vel_body)
        if not self.USE_GPS:
            ref_vel_enu = np.array([0, 0, 0])
        print("ref_vel_enu: {}".format(ref_vel_enu))
        
        # camera controller
        if not self.USE_CAMERA:
            cam_is_ok = False
            self.track_quality_k = 0
        if cam_is_ok:
            self.integral =np.array([0, 0, 0])
            i_err = np.array([pos_i[0] - self.WIDTH/2, pos_i[1] - self.HEIGHT/2, 0])
            i_err_body = pos_info["R_bc"].dot(i_err) - self.cam_offset
            i_err_body[1] = 0
            print("i_err_body: {}".format(i_err_body))
            i_err_enu = pos_info["mav_R"].dot(i_err_body)

            # use cam compensate GPS
            if np.linalg.norm(i_err_body) < self.HEIGHT/2:
                self.cam_gps_err_body = self.cam_gps_err_body + self.cam_gps_err_kp.dot(pos_info["mav_R"].T.dot(self.rpos_est_k))*dt
                self.cam_gps_err_up = self.cam_gps_err_up + self.cam_gps_err_kp[2][2]*self.rpos_est_k[2]*dt
                print("cam_gps_err_body: {}".format(self.cam_gps_err_body))
                print("cam_gps_err_up: {}".format(self.cam_gps_err_up))
            # PI
            self.integral_cam = self.integral_cam + self.Ki_nu_cam.dot(i_err_body)*dt
            self.SatIntegral(self.integral_cam, 0.5, -0.5)
            self.ref_vel_cam_body = (self.Kp_nu_cam.dot(i_err_body) + self.integral_cam) * np.linalg.norm(self.rpos_est_k)
            # Is use camera information to control the body's lateral speed.
            if not self.USE_CAM_FOR_X:
                self.ref_vel_cam_body[0] = 0
            if self.USE_DEPTH_FOR_X:
                self.ref_vel_cam_body[0] = self.sat(self.Kp_lr_depth * depth_delta, 0.5)
            self.ref_vel_cam_body[1] = 0
            print("ref_vel_cam_body: {}".format(self.ref_vel_cam_body))
            # ref_vel_cam_enu = pos_info["mav_R"].dot(ref_vel_cam_body + self.cam_offset)
            # print("ref_vel_cam_enu: {}".format(ref_vel_cam_enu))

            # ref_vel_enu = (1 - self.track_quality_k) * ref_vel_enu + self.track_quality_k * ref_vel_cam_enu
            ref_vel_body = pos_info["mav_R"].T.dot(ref_vel_enu)
            print("ref_vel_body_gps: {}".format(ref_vel_body))
            if self.USE_CAM_FOR_X or self.USE_DEPTH_FOR_X:
                ref_vel_body[0] = (1 - self.track_quality_k) * ref_vel_body[0] + self.track_quality_k * self.ref_vel_cam_body[0]
            ref_vel_body[2] = (1 - self.track_quality_k) * ref_vel_body[2] + self.track_quality_k * self.ref_vel_cam_body[2]
            print("ref_vel_body_cam: {}".format(ref_vel_body))
            ref_vel_enu = pos_info["mav_R"].dot(ref_vel_body)
        else:
            self.integral_cam = np.array([0, 0, 0])
            self.ref_vel_cam_body[1] = 0
            ref_vel_enu = (1 - self.track_quality_k) * ref_vel_enu + self.track_quality_k * pos_info["mav_R"].dot(self.ref_vel_cam_body)


        cmd_yawrate = self.sat(self.P*pos_info["rel_yaw"], 2)
        if cam_is_ok:
            yaw_cam = -i_err[0]
            print("yaw_cam: {}".format(yaw_cam))
            # cmd_yawrate = cmd_yawrate + self.Kp_yaw_cam*yaw_cam
            # cmd_yawrate = self.Kp_yaw_cam*yaw_cam

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

    def GeoToENU(self, start, end):
        """ 
        1. start: [latitude, longitude, altitude]
        2. return: [distance from start point to end point in the East, ... North, altitude].
        """
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

    def SatIntegral(self, a, up, down):
        for i in range(len(a)):
            if a[i] > up:
                a[i] = up
            elif a[i] < down:
                a[i] = down
        return a
