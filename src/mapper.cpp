//
// Created by jaguar on 26/12/2020.
//

#include "param.h"

#include <ltr/PathProfile.h>
#include <ltr/SetDistance.h>
#include <ltr/Mapping.h>

#include <geometry_msgs/PoseStamped.h>
#include <laser_geometry/laser_geometry.h>

#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/registration/icp.h>
#include <pcl_conversions/pcl_conversions.h>

#include <nav_msgs/Odometry.h>
#include <tf_conversions/tf_eigen.h>
#include "eigen_conversions/eigen_msg.h"
using namespace std;
using namespace cv;

typedef enum
{
    PAUSED,
    MAPPING,
    COMPLETED
} EMappingState;

class Mapper : public ParamSever
{
private:
    /* Service for set/reset distance */
    ros::ServiceClient dist_client;
    ltr::SetDistance dist_srv;

    /* Subscibers and publishers */
    ros::Subscriber dist_sub;
    ros::Subscriber robot_pose_sub;
    ros::Subscriber robot_gt_pose_sub;
    ros::Subscriber lidar_sub;
    ros::Subscriber joy_sub;
    ros::Subscriber joy_vel_sub;

    /* Publisher */
    ros::Publisher vel_cmd_pub;

    // used to change laser_scan to pointcloud2
    laser_geometry::LaserProjection projector_;
    
    float linear_vel;
    float angular_vel;
    float last_linear_vel;
    float last_angular_vel;

    /* calculate by odom */
    float dist_travelled;
    float local_dist = 0;
    float local_angle = 0;

    /* pose record */
    Eigen::Isometry3d curr_pose, last_pose, curr_gt_pose, last_gt_pose, difference;    
    geometry_msgs::Pose robot_pose, robot_gt_pose;
    bool notFirst;

    /* map scans and path profile */
    vector<pcl::PointCloud<pcl::PointXYZ>> map_scans;
    vector<double> map_poses, map_gt_poses;
    vector<float> scans_dist;
    vector<float> event_dist;
    vector<float> event_linear_vel;
    vector<float> event_angular_vel;
    ros::Time last_event_time;

    /* Mapping state */
    EMappingState state;
    geometry_msgs::Twist twist;
    int scans_count=0;
    int event_count;

    // map name
    string map_name;

public:
    Mapper(ros::NodeHandle *nh):notFirst(false)
    {
        state=PAUSED;
        local_dist = local_angle = 0;
        /* Initiate distance service client */
        dist_client = nh->serviceClient<ltr::SetDistance>(SET_DIST_SERVER);
        // image_transport::ImageTransport img_trans(*nh);

        /* initiate service */
        ROS_INFO("JOY_TOPIC        subscribe to %s", JOY_TOPIC.c_str());
        ROS_INFO("JOY_VEL          subscribe to %s", JOY_VEL.c_str());
        ROS_INFO("TOTAL_DIST_TOPIC subscribe to %s", TOTAL_DIST_TOPIC.c_str());
        ROS_INFO("LIDAR_TOPIC      subscribe to %s", LIDAR_TOPIC.c_str());
        ROS_INFO("ROBOT_POSE_TOPIC subscribe to %s", ROBOT_POSE_TOPIC.c_str());
        ROS_INFO("USE_GAZEBO       %d", USEGAZEBO);
        // ROS_INFO("LOCAL_DIST, LOCAL_ANGLE       %f, %f", local_dist, local_angle);
        // ROS_INFO("subscribe to %s", ROBOT_VEL_TOPIC.c_str());

        dist_sub = nh->subscribe<std_msgs::Float32>(TOTAL_DIST_TOPIC, 1, boost::bind(&Mapper::distanceCallBack, this, _1));
        robot_pose_sub = nh->subscribe<geometry_msgs::PoseStamped>(ROBOT_POSE_TOPIC, 1, boost::bind(&Mapper::robotPoseCallBack, this, _1));
        if (USEGAZEBO)
            robot_gt_pose_sub = nh->subscribe<geometry_msgs::PoseStamped>(ROBOT_POSE_GT_TOPIC, 1, boost::bind(&Mapper::robotGtPoseCallBack, this, _1));

        joy_sub = nh->subscribe<sensor_msgs::Joy>(JOY_TOPIC, 1, boost::bind(&Mapper::joyCallBack, this, _1));
        lidar_sub = nh->subscribe(LIDAR_TOPIC, 1, &Mapper::lidarCallBack, this);
        // vel_cmd_pub = nh->advertise<geometry_msgs::Twist>(ROBOT_VEL_TOPIC, 1);
        joy_vel_sub = nh->subscribe(JOY_VEL, 1, &Mapper::joyVelCallBack, this);  // New!
    }

    void distanceCallBack(const std_msgs::Float32::ConstPtr &dist_msg);
    void robotPoseCallBack(const geometry_msgs::PoseStamped::ConstPtr &pose_msg);
    void robotGtPoseCallBack(const geometry_msgs::PoseStamped::ConstPtr &pose_msg);
    void joyCallBack(const sensor_msgs::Joy::ConstPtr &joy);
    void joyVelCallBack(const geometry_msgs::Twist &vel_msg);  // New!
    void lidarCallBack(const sensor_msgs::LaserScanConstPtr &scan_msg);
    bool mapping(ltr::Mapping::Request &req, ltr::Mapping::Response &res);
    void setMapName(string &file_name);
    void saveMap();

    template <class T>
    T f_min_max(T x, T min, T max) { return fmin(fmax(x, min), max); }
};

/* utility functions */
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

// set the name of the map
void Mapper::setMapName(string &file_name) { map_name = file_name; }

// distance currently travelled
void Mapper::distanceCallBack(const std_msgs::Float32::ConstPtr &dist_msg)
{ 
    dist_travelled = dist_msg->data;
}

void Mapper::robotPoseCallBack(const geometry_msgs::PoseStamped::ConstPtr &pose_msg)
{   
    if (state == MAPPING)
    {
        robot_pose = pose_msg->pose;
        tf::poseMsgToEigen(robot_pose, curr_pose);
        if(!notFirst)
        {
            last_pose = curr_pose;
            notFirst = true;
        }
        difference = last_pose.inverse()*curr_pose;
        local_dist = difference.translation().norm();
        local_angle = abs(getYaw(difference));
    }
}

void Mapper::robotGtPoseCallBack(const geometry_msgs::PoseStamped::ConstPtr &pose_msg)
{
    if (state == MAPPING)
    {
        robot_gt_pose = pose_msg->pose;
        tf::poseMsgToEigen(robot_gt_pose, curr_gt_pose);
    }
}

// joystick dcallback, for PAUSE and STOP
void Mapper::joyCallBack(const sensor_msgs::Joy::ConstPtr &joy)
{
    // pause or stop
    if (joy->buttons[PAUSE_BUTTON])
        state = PAUSED;

    if (joy->buttons[STOP_BUTTON])
        state = COMPLETED;
}

// Get velocity from joy's velocity output
// Maybe it gets leg-track's output as control velocity
void Mapper::joyVelCallBack(const geometry_msgs::Twist &vel_msg)
{
    linear_vel = vel_msg.linear.x;
    angular_vel = vel_msg.angular.z;
}

// lidar call back function
void Mapper::lidarCallBack(const sensor_msgs::LaserScanConstPtr &scan_msg)
{
    if (state == MAPPING)
    {
        // ROS_INFO("Scan %i is record at %f, %f, %f.", scans_count, dist_travelled, local_dist, local_angle);
        if (local_dist >= TOPO_LINER_INTERVAL || local_angle >= TOPO_ANGLE_INTERVAL)
        {   
            map_poses.insert(map_poses.end(), {robot_pose.position.x, robot_pose.position.y, robot_pose.position.z, robot_pose.orientation.x, robot_pose.orientation.y, robot_pose.orientation.z, robot_pose.orientation.w});
            if (USEGAZEBO)
                map_gt_poses.insert(map_gt_poses.end(), {robot_gt_pose.position.x, robot_gt_pose.position.y, robot_gt_pose.position.z, robot_gt_pose.orientation.x, robot_gt_pose.orientation.y, robot_gt_pose.orientation.z, robot_gt_pose.orientation.w});
            scans_dist.push_back(dist_travelled);
            // laser_scan -> pointcloud2 (ROS->ROS)
            sensor_msgs::PointCloud2 pointcloud_msg; 
            projector_.projectLaser(*scan_msg, pointcloud_msg);

            // pointcloud2 -> pointcloud_pcl (ROS->PCL)
            pcl::PointCloud<pcl::PointXYZ> pointcloud_pcl;
            pcl::fromROSMsg(pointcloud_msg, pointcloud_pcl);
            
            map_scans.push_back(pointcloud_pcl);
            ROS_INFO("Scan %i is record at %f, %f, %f.", scans_count, dist_travelled, local_dist, local_angle);
            std::cout << "From Eigen: " << curr_pose(0,3) << " " << curr_pose(1,3) << " " << getYaw(curr_pose) << std::endl;
            // std::cout << "From Odom : " << robot_pose.position.x << " " << robot_pose.position.y  << std::endl;

            local_dist = 0.;
            local_angle = 0.;
            last_pose = curr_pose; 
            scans_count++;
        }
    }
}

/* Mapping function */
bool Mapper::mapping(ltr::Mapping::Request &req, ltr::Mapping::Response &res)
{
    // set the name of the map
    setMapName(req.map_name);
    res.status = false;

    /* reset distance using service*/
    dist_srv.request.distance = dist_travelled = 0;
    local_dist = 0;
    local_angle = 0;

    if (!dist_client.call(dist_srv))
        ROS_ERROR("Failed to call service SetDistance provided by odometry_monitor node!");

    // map info
    map_scans.clear();
    map_poses.clear();
    map_gt_poses.clear();
    scans_dist.clear();
    event_dist.clear();
    event_linear_vel.clear();
    event_angular_vel.clear();

    state = MAPPING;
    scans_count = event_count = 0;
    linear_vel = 0.;
    angular_vel = 0.;

    ros::Rate rate(50);
    while (!ros::isShuttingDown())
    {
        /*on preempt request end mapping and save current map */
        if (state == COMPLETED)
        {
            ROS_INFO("Mapping completed, flushing map.");
            saveMap();
            res.status = true;
            return true;
        }

        //rate.sleep();
        ros::spinOnce();
    }
    return false;
}

void Mapper::saveMap()
{
    /*save the scans map as well*/
    for (int i = 0; i < map_scans.size(); i++)
    {   
        string scan_file_name = FOLDER + "/" + map_name + std::to_string(i) + ".pcd";  
        // ROS_INFO("Saving scans map to %s", scan_file_name.c_str());
        pcl::io::savePCDFileASCII (scan_file_name, map_scans[i]);
        // std::cerr << "Saved " << map_scans[i].size() << " data points to scan_file_name." << std::endl;
    }

    /*save the path profile as well*/
    string path_file_name = FOLDER + "/" + map_name + ".yaml";
    ROS_INFO("Saving path profile to %s", path_file_name.c_str());
    FileStorage pfs(path_file_name.c_str(), FileStorage::WRITE);
    write(pfs, "map_distance", vector<float>{dist_travelled});
    write(pfs, "scan_distance", scans_dist);
    write(pfs, "event_distance", event_dist);
    write(pfs, "linear_vel", event_linear_vel);
    write(pfs, "angular_vel", event_angular_vel);
    // if save the map keyframe poses
    write(pfs, "map_pose", map_poses);
    if (USEGAZEBO)
        write(pfs, "map_gt_poses", map_gt_poses);

    ROS_INFO("Done!");

    pfs.release();
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "ltr_mapper");

    ROS_INFO("Mapping Server started.");

    ros::NodeHandle nh;
    Mapper m(&nh);

    ros::ServiceServer ss = nh.advertiseService("ltr/mapper", &Mapper::mapping, &m);
    ros::spin();

    return 0;
}
