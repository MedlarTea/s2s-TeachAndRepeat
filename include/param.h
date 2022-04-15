//
// Created by jaguar on 25/12/2020.
//

#include <ros/ros.h>

#include <std_msgs/Header.h>
#include <std_msgs/Float32.h>
#include <sensor_msgs/JointState.h>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/Joy.h>
#include <geometry_msgs/Twist.h>

#include <opencv/cv.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <sensor_msgs/LaserScan.h>
#include <sensor_msgs/PointCloud2.h>

#include <vector>
#include <cmath>
#include <iostream>
#include <fstream>
#include <ctime>
#include <cfloat>
#include <iterator>
#include <sstream>
#include <string>

using namespace std;

class ParamSever
{

public:
    ros::NodeHandle nh;

    /* General Parameters */
    string FOLDER;
    bool GLOBAL_LOCALIZATION;
    bool USEGAZEBO;
    float TOPO_LINER_INTERVAL;
    float TOPO_ANGLE_INTERVAL;
    float X_THRESHOLD;
    float Y_THRESHOLD;
    float YAW_THRESHOLD;
    float MAX_LINEAR_VEL;
    float MIN_LINEAR_VEL;
    float MAX_ANGULAR_VEL;
    float MIN_ANGLE_VEL;
    float MAX_LINEAR_ACC;


    int STOP_BUTTON;
    int BEGIN_BUTTON;
    int PAUSE_BUTTON;
    // int ROT_ACC_BUTTON; // in the acceleration mode,  the robot will apply the 2x max angular velocity
    // int ROT_MAX_BUTTON; // in the acceleration Max mode, the robot will ignore the linear velocity (for rotation within origin)

    float SCAN_OFFSET_LINEAR_GAIN;
    float SCAN_OFFSET_ANGULAR_GAIN;

    /* PID Control */
    float PID_Kp_x, PID_Ki_x, PID_Kd_x;
    float PID_Kp_yaw, PID_Ki_yaw, PID_Kd_yaw;

    /* ROS Topics / Server */
    string LIDAR_TOPIC;
    string ODOM_TOPIC;
    string JOY_TOPIC;

    string SET_DIST_SERVER = "/ltr/set_distance";
    string TOTAL_DIST_TOPIC = "/ltr/total_dist";
    string LOCAL_DIST_TOPIC = "/ltr/local_dist";
    string LOCAL_ANGLE_TOPIC = "/ltr/total_angle";

    string ROBOT_VEL_TOPIC = "/cmd_vel";
    string VEL_CMD_TOPIC = "/cmd_vel";
    string MAP_TOPIC = "ltr/map_lidar";
    string ROBOT_POSE_TOPIC = "ltr/robot_pose";
    string ROBOT_POSE_GT_TOPIC = "ltr/robot_gt_pose";
    string ERROR_POSE_TOPIC = "ltr/error_pose";

    string JOY_VEL = "/bluetooth_teleop/cmd_vel";

    /* Map Info */

    ParamSever()
    {
        /* Navigation Parameters */
        // joystick axis
        nh.param<int>("begin_button", BEGIN_BUTTON, 2);  // Triangle
        nh.param<int>("stop_button", STOP_BUTTON, 1);  // Circle
        nh.param<int>("pause_button", PAUSE_BUTTON, 0);  // Cross

        // map parameters
        nh.param<bool>("global_localization", GLOBAL_LOCALIZATION, false);
        nh.param<bool>("use_gazebo", USEGAZEBO, false);
        nh.param<float>("topological_liner_interval", TOPO_LINER_INTERVAL, 0.4);
        nh.param<float>("topological_angle_interval", TOPO_ANGLE_INTERVAL, 0.08);  // radian
        nh.param<float>("x_threshold", X_THRESHOLD, 0.01);
        nh.param<float>("y_threshold", Y_THRESHOLD, 0.01);
        nh.param<float>("yaw_threshold", YAW_THRESHOLD, 0.02);

        nh.param<string>("map_folder", FOLDER, "/home/jing/dingo/myltr/map");

        // robot speed limits
        nh.param<float>("max_angular_velocity", MAX_ANGULAR_VEL, 1.4);
        nh.param<float>("min_angular_velocity", MIN_ANGLE_VEL, 0.01);
        nh.param<float>("max_linear_velocity", MAX_LINEAR_VEL, 2.0);
        nh.param<float>("min_linear_velocity", MIN_LINEAR_VEL, 0.01);
        nh.param<float>("max_linear_acceleration", MAX_LINEAR_ACC, 0.1);

        // visual navigation
        nh.param<float>("scan_offset_linear_gain", SCAN_OFFSET_LINEAR_GAIN, 0.1);
        nh.param<float>("scan_offset_angular_gain", SCAN_OFFSET_ANGULAR_GAIN, 0.3);
        nh.param<float>("PID_Kp_x", PID_Kp_x, 0.3);
        nh.param<float>("PID_Ki_x", PID_Ki_x, 0.01);
        nh.param<float>("PID_Kd_x", PID_Kd_x, 0);
        nh.param<float>("PID_Kp_yaw", PID_Kp_yaw, 0.3);
        nh.param<float>("PID_Ki_yaw", PID_Ki_yaw, 0);
        nh.param<float>("PID_Kd_yaw", PID_Kd_yaw, 0.05);

        nh.param<std::string>("topo_ltr/lidar_topic", LIDAR_TOPIC, "/front/scan");
        // nh.param<std::string>("topo_ltr/odom_topic", ODOM_TOPIC, "/dingo_velocity_controller/odom");
        nh.param<std::string>("topo_ltr/odom_topic", ODOM_TOPIC, "/odometry/filtered");  // Dad:base_link, Child:odom
        nh.param<std::string>("topo_ltr/joy_topic", JOY_TOPIC, "/bluetooth_teleop/joy");
        nh.param<std::string>("topo_ltr/vel_topic", VEL_CMD_TOPIC, "/cmd_vel");
    }
};
