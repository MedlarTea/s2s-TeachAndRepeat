//
// Created by jaguar on 25/12/2020.
//

#include "param.h"
#include <sensor_msgs/JointState.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/PoseStamped.h>
#include <ltr/SetDistance.h>

#include "eigen_conversions/eigen_msg.h"
#include <Eigen/Dense>

#include <gazebo_msgs/GetModelState.h>
#include <gazebo_msgs/ModelStates.h>

#include <tf_conversions/tf_eigen.h>

class OdomMonitor : public ParamSever
{

private:
    /* Subscriber */
    ros::Subscriber set_dist_sub;
    ros::Subscriber odom_sub;

    /* Publisher */
    ros::Publisher total_dist_pub;
    ros::Publisher robot_pose_pub;
    ros::Publisher robot_pose_gt_pub;

    /* Service */
    ros::ServiceServer set_dist_srv;

    float total_dist = 0;
    std_msgs::Float32 total_dist_msg;


    Eigen::Affine3d init_pose_inverse = Eigen::Affine3d::Identity();
    Eigen::Affine3d init_gt_pose_inverse = Eigen::Affine3d::Identity();
    Eigen::Affine3d curr_pose, curr_gt_pose, last_pose,last_gt_pose, difference, curr_odom_pose, curr_odom_gt_pose;

public:
    OdomMonitor(ros::NodeHandle *nh)
    {
        /* initiate service */
        set_dist_srv = nh->advertiseService(SET_DIST_SERVER, &OdomMonitor::setDistance, this);

        /* Subscriber */
        odom_sub = nh->subscribe<nav_msgs::Odometry>(ODOM_TOPIC, 10, boost::bind(&OdomMonitor::odomCallBack, this, _1));

        /* Publisher */
        total_dist_pub = nh->advertise<std_msgs::Float32>(TOTAL_DIST_TOPIC, 1);
        robot_pose_pub = nh->advertise<geometry_msgs::PoseStamped>(ROBOT_POSE_TOPIC, 1);
        robot_pose_gt_pub = nh->advertise<geometry_msgs::PoseStamped>(ROBOT_POSE_GT_TOPIC, 1);
    }

    double getYaw(const Eigen::Isometry3d &pose);
    void setDistance(const std_msgs::Float32::ConstPtr &dist_msg);
    void odomCallBack(const nav_msgs::Odometry::ConstPtr &odom_msg);
    bool setDistance(ltr::SetDistance::Request &req, ltr::SetDistance::Response &res);
    // record gt pose from gazebo
    geometry_msgs::Pose gazeboPoseCallback();
};

double OdomMonitor::getYaw(const Eigen::Isometry3d &pose)
{
    Eigen::Matrix3d rot = pose.linear();
    tf::Matrix3x3 rot_tf;
    tf::matrixEigenToTF(rot, rot_tf);
    double roll,pitch,yaw;
    rot_tf.getRPY(roll,pitch,yaw);
    return yaw;
}

/* service for set/reset the distance */
// 因为odom有drift, 所以在开始建图之后才开始记录distance
bool OdomMonitor::setDistance(ltr::SetDistance::Request &req, ltr::SetDistance::Response &res)
{
    res.distance = req.distance;
    total_dist = req.distance;

    ROS_INFO("Setting current travelled distance to %.3f", (float)req.distance);

    total_dist_msg.data = total_dist;

    total_dist_pub.publish(total_dist_msg);
    // reset the init_pose for the map
    init_pose_inverse = curr_odom_pose.inverse();
    if (USEGAZEBO) {init_gt_pose_inverse = curr_odom_gt_pose.inverse();}

    return true;
}

void OdomMonitor::odomCallBack(const nav_msgs::Odometry::ConstPtr &odom_msg)
{   

    /* calculate the map poses */
    tf::poseMsgToEigen(odom_msg->pose.pose, curr_odom_pose);
    curr_pose = init_pose_inverse*curr_odom_pose;  // from base_link to odom
    
    // Publish the distance message
    total_dist+=curr_pose.translation().norm();
    total_dist_msg.data = total_dist;
    total_dist_pub.publish(total_dist_msg);

    // Publish 
    geometry_msgs::PoseStamped curr_pose_msg;
    curr_pose_msg.header.frame_id = "/odom";
    tf::poseEigenToMsg(curr_pose, curr_pose_msg.pose);
    robot_pose_pub.publish(curr_pose_msg);

    if (USEGAZEBO)
    {
        tf::poseMsgToEigen(gazeboPoseCallback(), curr_odom_gt_pose);
        curr_gt_pose = init_gt_pose_inverse*curr_odom_gt_pose;  // from base_link to odom

        // Publish 
        geometry_msgs::PoseStamped curr_gt_pose_msg;
        curr_gt_pose_msg.header.frame_id = "/base_link";
        tf::poseEigenToMsg(curr_pose, curr_gt_pose_msg.pose);
        robot_pose_gt_pub.publish(curr_gt_pose_msg);
    }
}

geometry_msgs::Pose OdomMonitor::gazeboPoseCallback()
{
  // service model,适用于这种瞬间触发的信息获取
  ros::NodeHandle n2;
  ros::ServiceClient client2 = n2.serviceClient<gazebo_msgs::GetModelState>("/gazebo/get_model_state");
  gazebo_msgs::GetModelState srv2;
  srv2.request.model_name = "dingo"; //指定要获取的机器人在gazebo中的名字；

  if (client2.call(srv2))
    {
      gazebo_msgs::GetModelStateResponse state = srv2.response;
      geometry_msgs::Pose gtPose = state.pose;
      return gtPose;
    }
  else
  {
    ROS_ERROR("Failed to call service /gazebo/get_model_state");
  }
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "ltr_odom_monitor");

    ros::NodeHandle nh;
    OdomMonitor om = OdomMonitor(&nh);

    ROS_INFO("Odometry Monitor Started.");

    ros::spin();

    return 0;
}
