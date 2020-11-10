/**
 * @file offb_node.cpp
 * @brief offboard example node, written with mavros version 0.14.2, px4 flight
 * stack and tested in Gazebo SITL
 */
#include <vector>
#include <algorithm>
#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/State.h>
#include <mavros_msgs/SetMavFrame.h>

#include <geometry_msgs/TwistStamped.h>

#include <geometry_msgs/QuaternionStamped.h>
#include <geometry_msgs/Vector3Stamped.h>
#include <sensor_msgs/NavSatFix.h>
#include <std_msgs/Float32.h>
#include <std_msgs/UInt8.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>
#include <tf/tf.h>
#include <sensor_msgs/LaserScan.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>
#include <sensor_msgs/Joy.h>
#include <sensor_msgs/Imu.h>
// #include <sensor_msgs/LaserScan.h>  qaq

void state_cb(const mavros_msgs::State::ConstPtr& msg);
void scan_tf_callback(const sensor_msgs::LaserScan& msg);
void odometry_callback(const nav_msgs::Odometry &current_info);
void savedmap_callback(const nav_msgs::Odometry &saved_info);
void position_pid_control(geometry_msgs::Point current_set_point,geometry_msgs::Point current_local_point,float velocity_limit,float target_yaw, float dead_zone, uint8_t mode);
geometry_msgs::Vector3 toEulerAngle(geometry_msgs::Quaternion quat);
geometry_msgs::Point rotation_hehe(geometry_msgs::Vector3 angle);
geometry_msgs::Point limit_velocity(float vx, float vy,float maximum);


const float deg2rad = 3.1415926535798/180.0;
const float rad2deg = 180.0/3.1415926535798;

float bizhangvx = 0;
float bizhangvy = 0;

bool takeoff_result;

//navigation define
#define IFPLANNER 0
#define IFVINS 0
#define TAG 1
#define GPS 2
#define VINS 3
#define BODY 1
#define GROUND 2
#define NAVIGATION_MODE VINS //if use tag, then TAG
#define FRAME_MODE BODY //if earth frame, then GROUND
#define LAST_TAG 350 //the tag whose id is 350 is the last one, the drone will take the last photo there and then RTL
#define POS_I_LIMIT 4.0*deg2rad
//mynteye to flight controler imu distance
#define DELTA_X -0.1
#define DELTA_Y 0.0
#define DELTA_Z 0.03

#define DEAD_ZONE 0.00 //dead zone, unit: m
#define POS_I_DEAD_ZONE 0.04 //in dead zone, don't intergrate
#define YAW_DEAD_ZONE 3
#define HEIGHT_DEAD_ZONE 0.0001
#define TARGET_HEIGHT 1.2


float Radius = 8.0;
float test_velocity = 2.0;
float yaw_init = 0.0;
//for control
uint32_t fc_sec, fc_nsec, velocity_sec, velocity_nsec;
uint32_t tag_sec, tag_nsec;
uint32_t yaw_record_count = 0;
float yaw_average = 0;
double fc_time, fc_time_start, velocity_time = 0.0, velocity_time_last = 0.0, velocity_time_last_control =0.0; //flight controller time now
double tag_time, tag_time_last;
float error_yaw_last = 0.0, error_yaw_integrated = 0.0;
bool fc_time_valid = false, obtain_control_result = false;
bool saved_update_flag = false, current_update_flag = false, planned_update_flag = false;
double time_current_update = 0,time_current_update_last = 0,time_current_update_last2 = 0;

uint8_t flight_status = 255, display_mode  = 255, obtain_control_count = 0, tag_dist_count =0, tag_yaw_count = 0, tag_invalid_count = 0;
sensor_msgs::NavSatFix current_gps;
geometry_msgs::Quaternion current_atti,current_tag_atti;
geometry_msgs::Point current_local_pos, velocity_intgrated_position, tag_mission_reset;
geometry_msgs::Vector3 current_velocity, fc_attitude_rpy, tag_attitude_rpy;
geometry_msgs::Vector3 current_acceleration;
geometry_msgs::Pose tag_pose_pub;
sensor_msgs::LaserScan scan_msg;
float current_height;
uint32_t tag_id, tag_id_last;
//when detected tag ,then tag_valid is true, if tag always can't be detected, then false
bool tag_valid = false, tag_valid_flag = false;//judge if the tag data is valid
//next_tag: if or not go to next tag,   position_hold_flag: if or not hold above this tag
bool next_tag = true, position_hold_flag = false, tag_position_flag = false, gimbal_flag = false, RTL_flag = false;
bool photo_flag = false, only_once_flag = true;


const float I_VEL_LIMIT = 0.025; //the intergrate item limit of position control, limit the expected velocity
const float D_VEL_LIMIT = 0.015;
const float I_ANG_LIMIT = 0.5; //the intergrate item limit of velocity control, limit the expected angle
const float YAW_I_LIMIT = 2.0;
const float YAW_R_LIMIT = 0.015;
const float YAW_RATE_LIMIT = 10.0;
const float BIZHANG_LIMIT = 1.0;

bool dead_zone_flag = false;

//control parameters
const float P_pos = 0.50;
const float I_pos = 0.10 ;
const float D_pos = 0.00;
const float P_vel = 6.00;
const float I_vel = 1.0;
const float D_vel = 0.00;
const float P_yaw = 1.00;
const float I_yaw = 0.00;
const float D_yaw = 0.00;
const float P_z = 1.00;

//gimbal and camera controll commands
const float GIMBAL_ROLL     = 0.0;
const float GIMBAL_PITCH    = 0.0;
const float GIMBAL_YAW      = 90.0;
const uint8_t CAMERA_ACTION = 1; //1:take photos 2:videos 3:photos and videos
const uint8_t PHOTO_NUMBER  = 1; //the number of photos to take
const uint8_t VIDEO_TIME    = 4; //the video time to take

//how long time between controls
const float delta_t = 1;
 
mavros_msgs::State current_state;
ros::Subscriber state_sub;
ros::Subscriber odometrysub;
ros::Subscriber savedmapsub;
ros::Subscriber planWaypointsub;
ros::Subscriber scan_tf_sub;
ros::Publisher local_pos_pub, local_vel_pub;

ros::ServiceClient arming_client;
ros::ServiceClient set_mode_client;
ros::ServiceClient set_vechile_ned;

geometry_msgs::Point current_point;
geometry_msgs::Point saved_point;
geometry_msgs::Point planned_point;
geometry_msgs::Quaternion current_angle;
geometry_msgs::Quaternion saved_angle;
geometry_msgs::Quaternion planned_angle;
geometry_msgs::Vector3 curr_angle;
geometry_msgs::Vector3 save_angle;
geometry_msgs::Vector3 plan_angle;
float current_yaw_angle;
float saved_yaw_angle;
float planned_yaw_angle;
geometry_msgs::Point tag_position, convert_tag_position, error_pos_last, error_vel_last, error_pos_integrated, error_vel_integrated, attitude_expect, velocity_expected;
geometry_msgs::PoseStamped pose;
geometry_msgs::TwistStamped msgtwist;
geometry_msgs::Vector3 linear;
geometry_msgs::Vector3 angular;


template <typename T>
T limit(T a, T max)
{
  if(a > max)
    a = max;
  if(a < -max)
    a = -max;
  return a;
}

void state_cb(const mavros_msgs::State::ConstPtr& msg){
    current_state = *msg;
}

void scan_tf_callback(const sensor_msgs::LaserScan& msg){
    scan_msg = msg;
    // msg_pub.header.frame_id = "world";
    //定义一个tf广播器
	// static tf::TransformBroadcaster br;
	
	// tf::Transform transform;
	// transform.setOrigin(tf::Vector3(0, 0, 0.05));
	// tf::Quaternion q;
	// q.setRPY(0,0,0);
	// transform.setRotation(q);
    // br.sendTransform(tf::StampedTransform(transform, msg_pub.header.stamp, "body", "laser"));
    // std::cout<<"----------->"<<scan_msg.ranges[180]<<std::endl;

    
}


//vins subscriber 
void savedmap_callback(const nav_msgs::Odometry &saved_info)
{
  saved_point.x= saved_info.pose.pose.position.x;
  saved_point.y= saved_info.pose.pose.position.y;
  saved_point.z= saved_info.pose.pose.position.z;
  saved_angle.x= saved_info.pose.pose.orientation.x;
  saved_angle.y= saved_info.pose.pose.orientation.y;
  saved_angle.z= saved_info.pose.pose.orientation.z;
  saved_angle.w= saved_info.pose.pose.orientation.w;
  save_angle = toEulerAngle(saved_angle);
  geometry_msgs::Point rotation_hehe_data1 = rotation_hehe(save_angle);
  saved_point.x += rotation_hehe_data1.x;
  saved_point.y += rotation_hehe_data1.y;
  saved_point.z += rotation_hehe_data1.z;
  saved_yaw_angle = save_angle.z;
//   ROS_INFO("saved %f %f %f %f",saved_point.x,saved_point.y,saved_point.z,saved_yaw_angle*rad2deg);

  saved_update_flag = true;
}


void odometry_callback(const nav_msgs::Odometry &current_info)
{
  time_current_update = ros::Time::now().toSec();
  current_point.x= current_info.pose.pose.position.x;
  current_point.y= current_info.pose.pose.position.y;
  current_point.z= current_info.pose.pose.position.z;
  current_angle.x= current_info.pose.pose.orientation.x;
  current_angle.y= current_info.pose.pose.orientation.y;
  current_angle.z= current_info.pose.pose.orientation.z;
  current_angle.w= current_info.pose.pose.orientation.w;
  curr_angle = toEulerAngle(current_angle);
  geometry_msgs::Point rotation_hehe_data = rotation_hehe(curr_angle);
  current_point.x += rotation_hehe_data.x;
  current_point.y += rotation_hehe_data.y;
  current_point.z += rotation_hehe_data.z;
  
  current_yaw_angle = curr_angle.z;
//   ROS_INFO("current %f %f %f",current_point.x,current_point.y,current_point.z, current_yaw_angle*rad2deg);
//  ROS_INFO("angle   %f %f %f",curr_angle.x*rad2deg,curr_angle.y*rad2deg,curr_angle.z*rad2deg);
  current_update_flag = true;
}
 
int main(int argc, char **argv)
{
    ros::init(argc, argv, "off_board_test");
    ros::NodeHandle nh;
    


    //接收vins的当前位置和目标位置
    odometrysub     = nh.subscribe("vins_estimator/odometry",10,odometry_callback);
    savedmapsub     = nh.subscribe("vins_estimator/saved_keyframe",10,savedmap_callback);
 
    state_sub = nh.subscribe<mavros_msgs::State>
            ("mavros/state", 10, state_cb);
    scan_tf_sub = nh.subscribe("/scan", 10, &scan_tf_callback);


    local_vel_pub = nh.advertise<geometry_msgs::TwistStamped>
            ("/mavros/setpoint_velocity/cmd_vel", 10);
    local_pos_pub = nh.advertise<geometry_msgs::PoseStamped>
            ("mavros/setpoint_position/local", 10);
    arming_client = nh.serviceClient<mavros_msgs::CommandBool>
            ("mavros/cmd/arming");
    set_mode_client = nh.serviceClient<mavros_msgs::SetMode>
            ("mavros/set_mode");
    set_vechile_ned = nh.serviceClient<mavros_msgs::SetMavFrame>
            ("mavros/setpoint_velocity/mav_frame");


    //the setpoint publishing rate MUST be faster than 2Hz
    ros::Rate rate(10);
 
    // wait for FCU connection
    while(ros::ok() && current_state.connected){
        ros::spinOnce();
        rate.sleep();
    }

    geometry_msgs::PoseStamped pose;
    pose.pose.position.x = 0;
    pose.pose.position.y = 0;
    pose.pose.position.z = 1.0;
    pose.pose.orientation.x=0;
    pose.pose.orientation.y=0;
    pose.pose.orientation.z=-0.7;
    pose.pose.orientation.w=-0.7;

 
    mavros_msgs::SetMode offb_set_mode;
    offb_set_mode.request.custom_mode = "OFFBOARD";
 
    mavros_msgs::CommandBool arm_cmd;
    arm_cmd.request.value = true;

    mavros_msgs::SetMavFrame setmavframe;
    setmavframe.request.mav_frame = 8;
 
    ros::Time last_request = ros::Time::now();
    int time_wait = 0;
 
    while(ros::ok()){
        if( current_state.mode != "OFFBOARD" &&
            (ros::Time::now() - last_request > ros::Duration(5.0))){
            if( set_mode_client.call(offb_set_mode) &&
                offb_set_mode.response.mode_sent){
                ROS_INFO("Offboard enabled");
            }
            last_request = ros::Time::now();
        } else {
            if( !current_state.armed &&
                (ros::Time::now() - last_request > ros::Duration(5.0))){
                if( arming_client.call(arm_cmd) &&
                    arm_cmd.response.success){
                    ROS_INFO("Vehicle armed");
                }
                
                last_request = ros::Time::now();
            }
        }
        if( (ros::Time::now() - last_request > ros::Duration(5.0))){
        if(set_vechile_ned.call(setmavframe) && setmavframe.response.success)
        {
            std::cout<<"------------>success"<<std::endl;
        }
        last_request = ros::Time::now();
            }
        if (IFPLANNER){
            if (planned_update_flag){
                if(time_wait <0){
                    local_pos_pub.publish(pose);
                    ROS_INFO("waiting vins stable: %d", time_wait);
                    time_wait++;
                }
                else{
                    position_pid_control(planned_point,current_point,1,planned_yaw_angle,0,7);
                    linear.x = velocity_expected.x;
                    linear.y = velocity_expected.y;
                    linear.z = velocity_expected.z;
                    angular.x = 0;
                    angular.y = 0;
                    angular.z = attitude_expect.z;


                    msgtwist.header.stamp = ros::Time::now();
                    msgtwist.header.seq=1;
                    msgtwist.twist.linear=linear;
                    msgtwist.twist.angular=angular;
                    // ROS_INFO("planned send to vechile: %f %f %f %f",msgtwist.twist.linear.x,msgtwist.twist.linear.y,msgtwist.twist.linear.z,msgtwist.twist.angular.z*rad2deg);
                    local_vel_pub.publish(msgtwist);
                    planned_update_flag = false;
                }
            
            }
            else{
                // linear.x = 0;
                // linear.y = 0;
                // linear.z = 0;
                // angular.x = 0;
                // angular.y = 0;
                // angular.z = 0;
                // msgtwist.header.stamp = ros::Time::now();
                // msgtwist.header.seq=1;
                // msgtwist.twist.linear=linear;
                // msgtwist.twist.angular=angular;
                // local_vel_pub.publish(msgtwist);
                local_pos_pub.publish(pose);
                ROS_INFO("no planned message");
            }
            
 
        }
        else if (IFVINS){
            if(current_update_flag && saved_update_flag)
                {

                position_pid_control(saved_point,current_point,1,saved_yaw_angle,0,7);
                linear.x = velocity_expected.x;
                linear.y = velocity_expected.y;
                linear.z = velocity_expected.z;
                angular.x = 0;
                angular.y = 0;
                angular.z = attitude_expect.z;

                    // linear.x = 0;
                    // linear.y = 1;
                    // linear.z = 0;
                    // angular.x = 0;
                    // angular.y = 0;
                    // angular.z = 0;

                msgtwist.header.stamp = ros::Time::now();
                    msgtwist.header.seq=1;
                    msgtwist.twist.linear=linear;
                    msgtwist.twist.angular=angular;
                    // ROS_INFO("send to vechile: %f %f %f %f",msgtwist.twist.linear.x,msgtwist.twist.linear.y,msgtwist.twist.linear.z,msgtwist.twist.angular.z*rad2deg);
                local_vel_pub.publish(msgtwist);


                current_update_flag = false;
                saved_update_flag = false;
                }
                else if(ros::Time::now().toSec() - time_current_update >= 2)
                {
                    linear.x = 0;
                    linear.y = 0;
                    linear.z = 0;
                    angular.x = 0;
                    angular.y = 0;
                    angular.z = 0;
                    msgtwist.header.stamp = ros::Time::now();
                    msgtwist.header.seq=1;
                    msgtwist.twist.linear=linear;
                    msgtwist.twist.angular=angular;
                    local_vel_pub.publish(msgtwist);
                    ROS_INFO("the vins blowing");
                }

        }
        
        
 
        ros::spinOnce();
        rate.sleep();
    }
 
    return 0;
}

void position_pid_control(geometry_msgs::Point current_set_point,geometry_msgs::Point current_local_point,float velocity_limit,float target_yaw, float dead_zone, uint8_t mode)
{
    float vx = 0.0, vy = 0.0, vxp = 0.0, vyp = 0.0, vxi = 0.0, vyi = 0.0, vxd = 0.0, vyd = 0.0;
    float yaw_rate = 0.0, yaw_rate_p = 0.0, yaw_rate_i = 0.0, yaw_rate_d = 0.0, vz = 0;
    float roll = 0.0, pitch = 0.0;
    //position control  mode & 0x01
    if (1) {
        //calculate velocity, P control
        geometry_msgs::Point error_pos;
        error_pos.x = current_set_point.x - current_local_point.x;
        error_pos.y = current_set_point.y - current_local_point.y;
        vxp = P_pos * error_pos.x;
        vyp = P_pos * error_pos.y;
        vxd = D_pos * (error_pos.x - error_pos_last.x);
        vyd = D_pos * (error_pos.y - error_pos_last.y);
        vxd = limit(vxd, D_VEL_LIMIT);
        vyd = limit(vyd, D_VEL_LIMIT);
        /*if(abs(error_pos.x) < abs(error_pos_last.x) && (abs(error_pos.x) >= 1.5 * DEAD_ZONE))
        {
          vxd = 0.0;
        }
        if(abs(error_pos.y) < abs(error_pos_last.y) && (abs(error_pos.y) >= 1.5 * DEAD_ZONE))
        {
          vyd = 0.0;
        }*/
        if (abs(error_pos.x) >= POS_I_DEAD_ZONE) {
            error_pos_integrated.x += error_pos.x;
        } else {
            error_pos_integrated.x = 0.0;
        }
        if (abs(error_pos.y) >= POS_I_DEAD_ZONE) {
            error_pos_integrated.y += error_pos.y;
        } else {
            error_pos_integrated.y = 0.0;
        }
        if (I_pos > 0.0001) {
            error_pos_integrated.x = limit((float) error_pos_integrated.x, I_VEL_LIMIT / I_pos);
            error_pos_integrated.y = limit((float) error_pos_integrated.y, I_VEL_LIMIT / I_pos);
        }
        vxi = I_pos * error_pos_integrated.x;
        vyi = I_pos * error_pos_integrated.y;
        vx = vxp + vxi + vxd;
        vy = vyp + vyi + vyd;

        float x_offset = current_set_point.x - current_local_point.x;
        float y_offset = current_set_point.y - current_local_point.y;
        float distance = sqrt(x_offset * x_offset + y_offset * y_offset);
        if (distance <= dead_zone) {
            dead_zone_flag = true;
            error_pos_integrated.x = 0.0;
            error_pos_integrated.y = 0.0;
            vx = 0;
            vy = 0;
        } else {
            dead_zone_flag = false;
        }

        //limit the speed
        vx = limit_velocity(vx, vy, velocity_limit).x;
        vy = limit_velocity(vx, vy, velocity_limit).y;
        //ROS_INFO("vx_exp vy_exp %f %f",vx,vy);

        error_pos_last = error_pos;
    }
    //yaw control  mode & 0x02
    if (0) {
        //use the flight controller yaw in default
        float current_yaw = fc_attitude_rpy.z;
        //the attitude form tag is not correct when the drone is rotate, so we don't use it
        if (NAVIGATION_MODE == TAG) {
            current_yaw = tag_attitude_rpy.z;
        }
        if (NAVIGATION_MODE == VINS) {
            current_yaw = current_yaw_angle;
        }
        //Drone turns at the smallest angle
        float error_yaw = (target_yaw - current_yaw) * rad2deg;
        if (error_yaw < -180)
            error_yaw += 360;
        else if (error_yaw > 180)
            error_yaw -= 360;
        yaw_rate_p = P_yaw * error_yaw;
        yaw_rate_d = D_yaw * (error_yaw - error_yaw_last);
        error_yaw_integrated += error_yaw;
        if (I_yaw > 0.0001) {
            error_yaw_integrated = limit(error_yaw_integrated, YAW_I_LIMIT / I_yaw);
        }
        yaw_rate_i = I_yaw * error_yaw_integrated;
        if (abs(error_yaw) <= YAW_DEAD_ZONE) {
            yaw_rate_p = 0.0;
            yaw_rate_i = 0.0;
            yaw_rate_d = 0.0;
            error_yaw_integrated = 0.0;
        }
        yaw_rate = yaw_rate_p + yaw_rate_i + yaw_rate_d;
        if (abs(error_yaw) >= YAW_RATE_LIMIT) {
            yaw_rate = limit(yaw_rate, YAW_RATE_LIMIT);
        } else {
            yaw_rate = limit(yaw_rate, (float) (YAW_RATE_LIMIT * 0.4));
        }
        error_yaw_last = error_yaw;
    }

    //height control  mode & 0x04
    if (1) {
        //control the height around 1.5m, the height is from flight controller
        float deltaz = current_local_point.z - current_set_point.z;
        if (fabs(deltaz) >= HEIGHT_DEAD_ZONE) {
            vz = P_z * (current_set_point.z - current_local_point.z);
        } else {
            vz = 0.0;
        }
    }

    bizhangvx = 0;
    bizhangvy = 0;
    for(int i=0;i<360;i++){
        if(scan_msg.ranges[i]<0.5){
            bizhangvx += sin(i*deg2rad)/scan_msg.ranges[i];
            bizhangvy += cos(i*deg2rad)/scan_msg.ranges[i];
        }
    }
    if( fabs(bizhangvx)>0 || fabs(bizhangvy)>0){
        ROS_INFO("bizhang speed no limit : %f %f", bizhangvx, bizhangvy);
        if(fabs(bizhangvx)>fabs(bizhangvy)){
            velocity_expected.x = bizhangvx>0?0.25:-0.25;
            velocity_expected.y = bizhangvy>0?(fabs(bizhangvy)/fabs(bizhangvx))/4:-(fabs(bizhangvy)/fabs(bizhangvx))/4;
        }
        else {
            velocity_expected.y = bizhangvy>0?0.25:-0.25;
            velocity_expected.x = bizhangvx>0?(fabs(bizhangvx)/fabs(bizhangvy))/4:-(fabs(bizhangvx)/fabs(bizhangvy))/4;
        }
        // velocity_expected.x = limit(bizhangvx,BIZHANG_LIMIT);
        // velocity_expected.y = limit(bizhangvy,BIZHANG_LIMIT);
        // ROS_INFO("bizhang speed : %f %f", velocity_expected.x, velocity_expected.y);
        velocity_expected.z = 0;
        attitude_expect.z = 0;
    }

    else{
        velocity_expected.x = vx * cos(curr_angle.z) + vy * sin(curr_angle.z);
        velocity_expected.y = vy * cos(curr_angle.z) - vx * sin(curr_angle.z);
        velocity_expected.z = vz;
        attitude_expect.z = yaw_rate * deg2rad;
    }
    // velocity_expected.x = vx * cos(curr_angle.z) + vy * sin(curr_angle.z);
    // velocity_expected.y = vy * cos(curr_angle.z) - vx * sin(curr_angle.z);
    // velocity_expected.z = vz;
    // attitude_expect.z = yaw_rate * deg2rad;
}

geometry_msgs::Vector3 toEulerAngle(geometry_msgs::Quaternion quat)
{
    geometry_msgs::Vector3 ans;

    tf::Matrix3x3 R_FLU2ENU(tf::Quaternion(quat.x, quat.y, quat.z, quat.w));
    R_FLU2ENU.getRPY(ans.x, ans.y, ans.z);
    return ans;
}

geometry_msgs::Point rotation_hehe(geometry_msgs::Vector3 angle)
{
    geometry_msgs::Point delta_distance;
    float Rbe[3][3] = {0};
    angle.z -= yaw_init;
    Rbe[0][0] = cos(angle.y) * cos(angle.z);
    Rbe[0][1] = cos(angle.z) * sin(angle.y) * sin(angle.x) - sin(angle.z) * cos(angle.x);
    Rbe[0][2] = cos(angle.z) * sin(angle.y) * cos(angle.x) + sin(angle.z) * sin(angle.x);
    Rbe[1][0] = cos(angle.y) * sin(angle.z);
    Rbe[1][1] = sin(angle.z) * sin(angle.y) * sin(angle.x) + cos(angle.z) * cos(angle.x);
    Rbe[1][2] = sin(angle.z) * sin(angle.y) * cos(angle.x) - cos(angle.z) * sin(angle.x);
    Rbe[2][0] = -sin(angle.y);
    Rbe[2][1] = sin(angle.x) * cos(angle.y);
    Rbe[2][2] = cos(angle.x) * cos(angle.y);
    delta_distance.x = Rbe[0][0] * DELTA_X + Rbe[0][1] * DELTA_Y + Rbe[0][2] * DELTA_Z;
    delta_distance.y = Rbe[1][0] * DELTA_X + Rbe[1][1] * DELTA_Y + Rbe[1][2] * DELTA_Z;
    delta_distance.z = Rbe[2][0] * DELTA_X + Rbe[2][1] * DELTA_Y + Rbe[2][2] * DELTA_Z;
    return delta_distance;
}


geometry_msgs::Point limit_velocity(float vx, float vy,float maximum)
{
	geometry_msgs::Point vel;
	float velocity = sqrt(vx * vx + vy * vy);
	if(maximum <= 0)
	{
		vel.x = 0;
		vel.y = 0;
	}
	if(velocity <= maximum)
	{
		vel.x = vx;
		vel.y = vy;
	}
	//if velocity is bigger than maximum, then limit the vx and vy, and keep the direction meanwhile.
	else
	{
		//the velocity must not be zero when in this step
		vel.x = vx / velocity * maximum;
		vel.y = vy / velocity * maximum;
	}
	return vel;
}
