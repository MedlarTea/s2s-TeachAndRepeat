//
// Created by jaguar on 26/12/2020.
//

#include "param.h"

#include <ltr/PathProfile.h>
#include <ltr/SetDistance.h>
#include <ltr/Navigation.h>

#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>

#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/registration/icp.h>
#include <pcl_conversions/pcl_conversions.h>

#include "pointmatcher/PointMatcher.h"
#include "pointmatcher/IO.h"
#include "pointmatcher_ros/point_cloud.h"

#include <tf_conversions/tf_eigen.h>
#include <laser_geometry/laser_geometry.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Pose.h>
#include "eigen_conversions/eigen_msg.h"

#include <Eigen/Eigen>
#include <Eigen/Geometry>
#include <Eigen/Core>

#include <chrono>
#include <assert.h>
#include <math.h>
#include <algorithm>
#include<mutex>

using namespace cv;
using namespace std;
using namespace std::chrono;
using namespace PointMatcherSupport;
typedef PointMatcher<float> PM;
typedef PM::DataPoints DP;
typedef PM::Parameters Parameters;
#define PI 3.14159
typedef enum
{
    PREPARING,
    PAUSED,
    NAVIGATING,
    INITIALIZATION,
    COMPLETED
} ENAVIGATINGState;

class Navigator : public ParamSever
{
private:
    // Used to change current pose
    std::mutex updateMutex;

    /* Service for set/reset distance */
    ros::ServiceClient dist_client;
    ltr::SetDistance dist_srv;

    /* Subscibers */
    ros::Subscriber dist_sub;
    ros::Subscriber lidar_sub;
    ros::Subscriber joy_sub;
    ros::Subscriber robot_pose_sub;

    /* publishers */
    ros::Publisher vel_cmd_pub;
    ros::Publisher map_pub;  // to see the stored key-scan
    ros::Publisher error_pose_pub;

    // used to change laser_scan to pointcloud2
    laser_geometry::LaserProjection projector_;

    /* map scan */
    vector<DP> scans_map;
    // vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> scans_map;
    vector<double> d_poses_map;
    vector<Eigen::Isometry3d> poses_map;
    int num_scan;

    /* run-time info */
    float linear_vel;
    float angular_vel;
    float dist_travelled;
    int map_scan_idx;
    DP current_scan, map_scan;
    pcl::PointCloud<pcl::PointXYZ>::Ptr p_current_scan, p_map_scan;
    Eigen::Isometry3d zero_odom = Eigen::Isometry3d::Identity();
    Eigen::Isometry3d zero_last_odom, last_odom, current_odom, next_odom, last_map_odom, next_map_odom, map_odom, delta_map_odom, current_relative_odom, goal_vector_odom;
    Eigen::Isometry3d map_odom_corrected, goal, difference;
    bool isUpdated, isFirst;

    // offset from scan matcher and odom matcher
    double last_v_offset=DBL_MAX;
    double v_offset;
    double last_yaw_offset=DBL_MAX;
    double last_x_offset=DBL_MAX;
    double last_y_offset=DBL_MAX;
    double yaw_offset, x_offset, y_offset;
    double yaw_offset_scan, x_offset_scan, y_offset_scan;
    double last_yaw_offset_scan=DBL_MAX;
    double last_x_offset_scan=DBL_MAX;
    double last_y_offset_scan=DBL_MAX;
    double yaw_offset_odom, x_offset_odom, y_offset_odom;
    double last_yaw_offset_odom=DBL_MAX;
    double last_x_offset_odom=DBL_MAX;
    double last_y_offset_odom=DBL_MAX;

    // error and output velocity
    float x_error_accumlation;
    float y_error_accumlation;
    float yaw_error_accumlation;
    float v_error_accumlation;
    geometry_msgs::Twist twist;

    /* Navigation state */
    ENAVIGATINGState state;

public:
    bool is_reverse = false;

public:
    Navigator(ros::NodeHandle *nh)
    : p_current_scan(new pcl::PointCloud<pcl::PointXYZ>()),
      p_map_scan(new pcl::PointCloud<pcl::PointXYZ>()),
      isUpdated(false),
      isFirst(true)
    {   
        /* initiate service */
        ROS_INFO("JOY_TOPIC        subscribe to %s", JOY_TOPIC.c_str());
        ROS_INFO("VEL_CMD_TOPIC    subscribe to %s", VEL_CMD_TOPIC.c_str());
        ROS_INFO("TOTAL_DIST_TOPIC subscribe to %s", TOTAL_DIST_TOPIC.c_str());
        ROS_INFO("LIDAR_TOPIC      subscribe to %s", LIDAR_TOPIC.c_str());
        ROS_INFO("ROBOT_POSE_TOPIC subscribe to %s", ROBOT_POSE_TOPIC.c_str());
        ROS_INFO("MAP_TOPIC        subscribe to %s", MAP_TOPIC.c_str());
        ROS_INFO("ERROR_POSE_TOPIC subscribe to %s", ERROR_POSE_TOPIC.c_str());
        ROS_INFO("MAP_FOLDER                    %s", FOLDER.c_str());
        ROS_INFO("GLOBAL_INIT              %d",   GLOBAL_LOCALIZATION);
        ROS_INFO("PID_Kp_x                 %.3f", PID_Kp_x);
        ROS_INFO("PID_Ki_x                 %.3f", PID_Ki_x);
        ROS_INFO("PID_Kd_x                 %.3f", PID_Kd_x);
        ROS_INFO("PID_Kp_yaw               %.3f", PID_Kp_yaw);
        ROS_INFO("PID_Ki_yaw               %.3f", PID_Ki_yaw);
        ROS_INFO("PID_Kd_yaw               %.3f", PID_Kd_yaw);
        ROS_INFO("SCAN_OFFSET_LINEAR_GAIN  %.3f", SCAN_OFFSET_LINEAR_GAIN);
        ROS_INFO("SCAN_OFFSET_ANGULAR_GAIN %.3f", SCAN_OFFSET_ANGULAR_GAIN);

        state = PREPARING;
        zero_odom = Eigen::Isometry3d::Identity();
        /* Initiate distance service client */
        dist_client = nh->serviceClient<ltr::SetDistance>(SET_DIST_SERVER);

        /* Subscribers */
        // dist_sub = nh->subscribe<std_msgs::Float32>(TOTAL_DIST_TOPIC, 1, boost::bind(&Navigator::distanceCallBack, this, _1));
        robot_pose_sub = nh->subscribe<geometry_msgs::PoseStamped>(ROBOT_POSE_TOPIC, 1, boost::bind(&Navigator::robotPoseCallBack, this, _1));
        joy_sub = nh->subscribe<sensor_msgs::Joy>(JOY_TOPIC, 1, boost::bind(&Navigator::joyCallBack, this, _1));
        lidar_sub = nh->subscribe(LIDAR_TOPIC, 1, &Navigator::lidarCallBack, this);

        /* Publishers */
        map_pub = nh->advertise<sensor_msgs::PointCloud2>(MAP_TOPIC, 1);
        vel_cmd_pub = nh->advertise<geometry_msgs::Twist>(VEL_CMD_TOPIC, 1);
        error_pose_pub = nh->advertise<geometry_msgs::PoseWithCovariance>(ERROR_POSE_TOPIC, 1);  // for PID control
    }

    /* Service */
    bool navigate(ltr::Navigation::Request& req, ltr::Navigation::Response& res);

    /* Subscribers callback */
    // void distanceCallBack(const std_msgs::Float32::ConstPtr &dist_msg);
    void robotPoseCallBack(const geometry_msgs::PoseStamped::ConstPtr &pose_msg);
    void joyCallBack(const sensor_msgs::Joy::ConstPtr &joy);
    void lidarCallBack(const sensor_msgs::LaserScanConstPtr &scan_msg);

    /* Some useful tools */
    void loadMap(const string map_name);
    // void correct_goal(Eigen::Matrix4f transform);
    void update_goal(Eigen::Matrix4f transform);
    float PID(float error, float last_error, float error_acc, float Kp, float Ki, float Kd, float max_velocity);
    double getYaw(const Eigen::Matrix4d &pose);
    double getYaw(const Eigen::Isometry3d &pose);
    bool getGoal(const double &delta_x, const double &delta_y, const double &delta_yaw);
    /* Use libpointmatcher to calculate the transformation */
    Eigen::Matrix4f ICP(const DP &ref, const DP &data, PM::TransformationParameters initTrans=PM::TransformationParameters::Identity(4,4));
    /* Use to filter the icp results */
    bool isGood(Eigen::Matrix4f transform);
    /* for interpolate poses */
    Eigen::Isometry3d interpolate_poses(Eigen::Isometry3d scan_map_pose, Eigen::Isometry3d odom_map_pose, float scan_linear_gain, float scan_angular_gain);

    template <class T>
    T f_min_max(T x, T min, T max) { return fmin(fmax(x, min), max); }
};



/* utility functions */
int binarySearch(const vector<float> &array, int i, int j, float val)
{   
    if (i == j)
        return i;

    if (j - i == 1)
        return fabs(val - array[i]) > fabs(val - array[j]) ? j : i;

    if (i < j)
    {
        int mid = i + (j - i) / 2;
        if (val == array[mid])
            return mid;
        else if (val > array[mid])
            return binarySearch(array, mid, j, val);
        else
            return binarySearch(array, i, mid, val);
    }
    return -1;
}

double getYaw(const Eigen::Isometry3d &pose)
{
    Eigen::Matrix3d rot = pose.linear();
    tf::Matrix3x3 rot_tf;
    tf::matrixEigenToTF(rot, rot_tf);
    double roll,pitch,yaw;
    rot_tf.getRPY(roll,pitch,yaw);
    return yaw;
}

/* class functions */
void Navigator::joyCallBack(const sensor_msgs::Joy::ConstPtr &joy)
{   
    // begin to navigate
    if (joy->buttons[BEGIN_BUTTON])
        state = NAVIGATING;
    // pause or stop
    if (joy->buttons[PAUSE_BUTTON])
        state = PAUSED;

    if (joy->buttons[STOP_BUTTON])
        state = COMPLETED;
}

bool Navigator::navigate(ltr::Navigation::Request& req, ltr::Navigation::Response& res)
{
    // load the map
    loadMap(req.map_name);
    // set the reverse mode or not
    is_reverse = req.reverse;
    res.status = false;

    map_scan_idx = is_reverse ? num_scan : 0;

    x_error_accumlation = y_error_accumlation = yaw_error_accumlation = 0; // Reseting the PID integral at the start of navigation

    /* reset distance using service*/
    dist_srv.request.distance = dist_travelled = 0;

    if (!dist_client.call(dist_srv))
        ROS_ERROR("Failed to call service SetDistance provided by odometry_monitor node!");

    // ROS_INFO("Navigation Initialized: reverse mode %i, current distance: %f, goal distance %f", is_reverse, dist_travelled, goal_dist);

    if(GLOBAL_LOCALIZATION)
        state = INITIALIZATION;
    else
        state = NAVIGATING;

    ros::Rate rate(50);

    while(!ros::isShuttingDown())
    {
        if(state == COMPLETED)
        {
            res.status = true;
            return true;
        }

        //rate.sleep();
        ros::spinOnce();
    }

    return false;
}

/* load map */
void Navigator::loadMap(const string map_name)
{
    // load path profile
    string path_file_name = FOLDER + "/" + map_name + ".yaml";
    ROS_INFO("Loading path profile from %s", path_file_name.c_str());
    FileStorage fsp(path_file_name, FileStorage::READ);

    scans_map.clear();
    poses_map.clear();

    if (fsp.isOpened())
    {
        fsp["map_pose"] >> d_poses_map;
        fsp.release();
    }

    num_scan = int(d_poses_map.size()/7);

    // load map scans
    for (int num = 0; num < num_scan; num++)
    {   
        string scan_file_name = FOLDER + "/" + map_name + std::to_string(num) + ".pcd";
        ROS_INFO("Loading map scans from %s", scan_file_name.c_str());
        scans_map.push_back(DP::load(scan_file_name));
    }
    // Load pose
    for(vector<double>::iterator it=d_poses_map.begin(); it!=(d_poses_map.end()); it=it+7)
    {
        Eigen::Isometry3d pose = Eigen::Isometry3d::Identity();
        Eigen::Vector3d translation(*it, *(it+1), *(it+2));
        Eigen::Quaterniond rotation(*(it+6), *(it+3), *(it+4), *(it+5));
        pose.rotate(rotation);
        pose.pretranslate(translation);
        poses_map.push_back(pose);
    }
    assert(poses_map.size() == scans_map.size());
    ROS_INFO("Done!"); 
    state = PREPARING;
}

// Monitor the transformation of curr_scan and next_scan, used to:
// 1. Judge whether to update goal
// 2. Correct the heading direction
void Navigator::lidarCallBack(const sensor_msgs::LaserScanConstPtr &scan_msg)
{   
    // lock, avoid robot pose changed
    unique_lock<mutex> lock(updateMutex);

    // laser_scan -> pointcloud2 (ROS->ROS)
    sensor_msgs::PointCloud2 pointcloud_msg; 
    projector_.projectLaser(*scan_msg, pointcloud_msg);
    if (state == NAVIGATING)
    {   
        auto start = high_resolution_clock::now();
        // Update scan
        map_scan = scans_map[map_scan_idx];
        // pointcloud2 -> pointmatcher_datapoint (ROS->PCL)
        current_scan = PointMatcher_ros::rosMsgToPointMatcherCloud<float>(pointcloud_msg);
        Eigen::Matrix4f transform = ICP(current_scan, map_scan, goal_vector_odom.matrix().cast<float>());  // T from i+1 to t (scan)
        auto stop = high_resolution_clock::now();
        auto duration = duration_cast<microseconds>(stop - start);
        cout << "Scan pose estimation: " << duration.count()/1000 << "ms" << endl;
        if(isGood(transform))
        {
            // correct_goal();
            update_goal(transform);
        }
        
    }
    if (state == INITIALIZATION)
    {
        map_scan = scans_map[map_scan_idx];
        current_scan = PointMatcher_ros::rosMsgToPointMatcherCloud<float>(pointcloud_msg);
        Eigen::Matrix4f transform = ICP(current_scan, map_scan);
        if(isGood(transform))
        {
            Eigen::Isometry3d _transform(transform.cast<double>());
            zero_odom = _transform;  // from fake_init to real_init
            state = NAVIGATING;
        }
    }
}

// Monitor the transformation of curr_odom and next_odom, used to:
// 1. Low-level and high-frequency control
void Navigator::robotPoseCallBack(const geometry_msgs::PoseStamped::ConstPtr &pose_msg)
{
    if (state == NAVIGATING)
    {
        geometry_msgs::Pose robot_pose = pose_msg->pose;
        tf::poseMsgToEigen(robot_pose, current_odom);
        current_odom = zero_odom*current_odom;  // 统一至repeat时候的地图
        if(map_scan_idx==0)
        {
            delta_map_odom = Eigen::Isometry3d::Identity();
            zero_last_odom = zero_odom*poses_map[map_scan_idx];
        }
        else
            delta_map_odom = poses_map[map_scan_idx-1].inverse()*poses_map[map_scan_idx];  // T from i+1 to i
        current_relative_odom = zero_last_odom.inverse()*current_odom;  // T from t to i
        goal_vector_odom = current_relative_odom.inverse()*delta_map_odom;  // T from i+1 to t 

        std::cout << "curr: " << current_relative_odom(0,3) << " " << current_relative_odom(1,3) << " " << getYaw(current_relative_odom) << std::endl;
        std::cout << "targ: " << delta_map_odom(0,3) << " " << delta_map_odom(1,3) << " " << getYaw(delta_map_odom) << std::endl;

        // publish to Guangcheng-controller
        // geometry_msgs::PoseWithCovariance error_pose_withcovariance;
        // geometry_msgs::Pose error_pose;
        // tf::poseEigenToMsg(difference, error_pose);
        // error_pose_withcovariance.pose = error_pose;
        // boost::array<double, 36> covariance;
        // error_pose_withcovariance.covariance = covariance;
        // error_pose_pub.publish(error_pose_withcovariance);
        x_offset_odom = goal_vector_odom(0,3);
        y_offset_odom = goal_vector_odom(1,3);
        v_offset = sqrt(x_offset_odom*x_offset_odom+y_offset_odom*y_offset_odom);  // for diff
        yaw_offset_odom = getYaw(goal_vector_odom);

        if(last_yaw_offset_odom==DBL_MAX && last_x_offset_odom==DBL_MAX && last_y_offset_odom==DBL_MAX)
        {
            last_yaw_offset_odom = yaw_offset_odom;
            last_x_offset_odom = x_offset_odom;
            last_y_offset_odom = y_offset_odom;
            last_v_offset = v_offset;  // for diff
        }
        x_error_accumlation+=x_offset_odom;
        y_error_accumlation+=y_offset_odom;
        yaw_error_accumlation+=yaw_offset_odom;
        v_error_accumlation+=v_offset;  // for diff
        /* Turn first, then move forward */
        if (abs(yaw_offset_odom)<=0.1)
        {   
            /* omni controller */
            twist.linear.x = PID(x_offset_odom, last_x_offset_odom, x_error_accumlation, PID_Kp_x, PID_Ki_x, PID_Kd_x, MAX_LINEAR_VEL);
            twist.linear.y = PID(y_offset_odom, last_y_offset_odom, y_error_accumlation, PID_Kp_x, PID_Ki_x, PID_Kd_x, MAX_LINEAR_VEL);
            /* diff controller */
            // twist.linear.x = PID(v_offset, last_v_offset, v_error_accumlation, PID_Kp_x, PID_Ki_x, PID_Kd_x, MAX_LINEAR_VEL);  // for diff

            twist.angular.z = PID(yaw_offset_odom, last_yaw_offset_odom, yaw_error_accumlation, PID_Kp_yaw, PID_Ki_yaw, PID_Kd_yaw, MAX_ANGULAR_VEL);
        }
        // if the angle distance is large, turn first
        else
        {
            twist.linear.x = 0;
            x_error_accumlation = 0;
            twist.linear.y = 0;
            y_error_accumlation = 0;
            twist.angular.z = PID(yaw_offset_odom, last_yaw_offset_odom, yaw_error_accumlation, PID_Kp_yaw, PID_Ki_yaw, PID_Kd_yaw, MAX_ANGULAR_VEL);
        }

        last_x_offset_odom = x_offset_odom;
        last_y_offset_odom = y_offset_odom;
        last_yaw_offset_odom = yaw_offset_odom;
        last_v_offset = v_offset;  // for diff
        std::cout << "Velocity" << std::endl;
        std::cout << "x: " << twist.linear.x << std::endl;
        std::cout << "y: " << twist.linear.y << std::endl;
        std::cout << "a: " << twist.angular.z << std::endl;
    }

    if (getGoal(x_offset_odom, y_offset_odom, yaw_offset_odom) && last_x_offset_odom!=DBL_MAX)
    {
        if(map_scan_idx==num_scan-1)
            state = COMPLETED;
        else
        {
            map_scan_idx++;
            isUpdated = false;
            x_error_accumlation = y_error_accumlation = yaw_error_accumlation = v_error_accumlation = 0;
        }
    }
    if (state==COMPLETED || state==PAUSED)
    {
        twist.linear.x = twist.linear.y = twist.linear.z = 0.0;
        twist.angular.z = twist.angular.y = twist.angular.x = 0.0;

        if(state==COMPLETED)
            ROS_INFO("Navigation Task Completed!");
        else
            ROS_INFO("Navigation Task Paused!");
    }
    vel_cmd_pub.publish(twist); 
}

float Navigator::PID(float error, float last_error, float error_acc, float Kp, float Ki, float Kd, float max_velocity)
{
    if (isnan(error))
        error = 0;
    float delta = error - last_error;
    float velocity = Kp * error + Kd * delta + Ki * error_acc;
    velocity = std::min(std::max(velocity, -max_velocity), max_velocity);
    
    return velocity;
}

double Navigator::getYaw(const Eigen::Matrix4d &pose)
{
    Eigen::Matrix3d rot = pose.block<3,3>(0,0);
    tf::Matrix3x3 rot_tf;
    tf::matrixEigenToTF(rot, rot_tf);
    double roll,pitch,yaw;
    rot_tf.getRPY(roll,pitch,yaw);
    return yaw;
}

double Navigator::getYaw(const Eigen::Isometry3d &pose)
{
    Eigen::Matrix3d rot = pose.linear();
    tf::Matrix3x3 rot_tf;
    tf::matrixEigenToTF(rot, rot_tf);
    double roll,pitch,yaw;
    rot_tf.getRPY(roll,pitch,yaw);
    return yaw;
}

bool Navigator::getGoal(const double &delta_x, const double &delta_y, const double &delta_yaw)
{
    if (abs(delta_x) < X_THRESHOLD && abs(delta_y) < Y_THRESHOLD && abs(delta_yaw) < YAW_THRESHOLD)
        return true;
    return false;
}

void Navigator::update_goal(Eigen::Matrix4f transform)
{   // transform is T from i+1 to t (from scan estimation)
    // update the goal
    if (abs(transform(0,3)) < X_THRESHOLD && abs(transform(1,3)) < Y_THRESHOLD && abs(getYaw(transform.cast<double>())) < YAW_THRESHOLD)
    {
        // Eigen::Isometry3d _transform(transform.cast<double>());
        zero_last_odom = current_odom;
        map_scan_idx++;
    }
    // correct the goal
    else
    {

    }
        
}

/*
initTrans-- initTransform rom data to ref
return Transform from data to ref
*/
Eigen::Matrix4f Navigator::ICP(const DP &ref, const DP &data, PM::TransformationParameters initTrans)
{   
    // Create the default ICP algorithm
    PM::ICP icp;
    icp.setDefault();
    std::shared_ptr<PM::Transformation> rigidTrans;
    rigidTrans = PM::get().REG(Transformation).create("RigidTransformation");
    if (!rigidTrans->checkParameters(initTrans)) {
        cerr << endl
            << "Initial transformation is not rigid, identiy will be used"
            << endl;
        initTrans = PM::TransformationParameters::Identity(4,4);
    }
    const DP initializedData = rigidTrans->compute(data, initTrans.inverse());
    PM::TransformationParameters T = icp(initializedData, ref);
    Eigen::Matrix4f transform = initTrans*T;
    return transform;
}

// Use to judge ICP results
bool Navigator::isGood(Eigen::Matrix4f transform)
{   
    // Judge by distance/angle prior
    float distance = sqrt(transform(0,3)*transform(0,3)+transform(1,3)*transform(1,3));
    if(distance > TOPO_LINER_INTERVAL || fabs(getYaw(transform.cast<double>())) > TOPO_ANGLE_INTERVAL)
        return false;
    // Judge by kalman filter
    return true;
}

// interpolate poses by interpolating quaternion
Eigen::Isometry3d Navigator::interpolate_poses(Eigen::Isometry3d scan_map_pose, Eigen::Isometry3d odom_map_pose, float scan_linear_gain, float scan_angular_gain)
{
    /* interpolate poses by interpolate quaternion */
    Eigen::Quaterniond scan_map_quaternion(scan_map_pose.rotation());
    Eigen::Quaterniond odom_map_quaternion(odom_map_pose.rotation());
    double goal_rotation_x = scan_angular_gain * scan_map_quaternion.x() + (1 - scan_angular_gain) * odom_map_quaternion.x();
    double goal_rotation_y = scan_angular_gain * scan_map_quaternion.y() + (1 - scan_angular_gain) * odom_map_quaternion.y();
    double goal_rotation_z = scan_angular_gain * scan_map_quaternion.z() + (1 - scan_angular_gain) * odom_map_quaternion.z();
    double goal_rotation_w = scan_angular_gain * scan_map_quaternion.w() + (1 - scan_angular_gain) * odom_map_quaternion.w();
    Eigen::Quaterniond goal_rotation(goal_rotation_w, goal_rotation_x, goal_rotation_y, goal_rotation_z);

    Eigen::Vector3d goal_position;
    for (size_t i=0;i<3;i++)
        goal_position[i] = scan_linear_gain * scan_map_pose.translation()[i] + (1 - scan_linear_gain) * odom_map_pose.translation()[i];
    // Only update rotation
    // goal_position = odom_map_pose.translation();
    
    Eigen::Isometry3d new_goal = Eigen::Isometry3d::Identity();
    new_goal.rotate(goal_rotation);
    new_goal.pretranslate(goal_position);
    return new_goal;
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "ltr_navigator");

    ROS_INFO("Navigation Server started.");

    ros::NodeHandle nh;
    Navigator nav(&nh);

    ros::ServiceServer ss = nh.advertiseService("ltr/navigator", &Navigator::navigate, &nav);
    ros::spin();

    return 0;
}
