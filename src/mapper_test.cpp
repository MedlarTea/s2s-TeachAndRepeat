//
// Created by jaguar on 26/12/2020.
//

#include "param.h"

#include <ltr/PathProfile.h>
#include <ltr/SetDistance.h>
#include <ltr/Mapping.h>

#include <tf_conversions/tf_eigen.h>

#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>

#include <geometry_msgs/PoseStamped.h>
#include <laser_geometry/laser_geometry.h>

#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/registration/icp.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/filters/voxel_grid.h>

#include "pointmatcher/PointMatcher.h"
// #include "pointmatcher/Bibliography.h"

#include "boost/filesystem.hpp"
#include <cassert>
#include <iostream>

#include <Eigen/Eigen>
#include <Eigen/Geometry>
#include <Eigen/Core>

#include <chrono>
#include <numeric>

using namespace std;
using namespace cv;
using namespace std::chrono;
using namespace PointMatcherSupport;
typedef PointMatcher<float> PM;
typedef PM::DataPoints DP;
typedef PM::Parameters Parameters;

#define PI 3.14159

double getYaw(const Eigen::Isometry3d &pose)
{
    Eigen::Matrix3d rot = pose.linear();
    tf::Matrix3x3 rot_tf;
    tf::matrixEigenToTF(rot, rot_tf);
    double roll,pitch,yaw;
    rot_tf.getRPY(roll,pitch,yaw);
    return yaw*360/PI;
}

double getYaw(const Eigen::Matrix4d &pose)
{
    Eigen::Matrix3d rot = pose.block<3,3>(0,0);
    tf::Matrix3x3 rot_tf;
    tf::matrixEigenToTF(rot, rot_tf);
    double roll,pitch,yaw;
    rot_tf.getRPY(roll,pitch,yaw);
    return yaw*360/PI;
}

void cal(const vector<vector<double>> &gt, const vector<vector<double>> &estim)
{
    vector<double> delta_x;
    vector<double> delta_y;
    vector<double> delta_yaw;
    for(int i=0;i<estim.size();i++)
    {
        delta_x.push_back(fabs(estim[i][0]-gt[i][0]));
        delta_y.push_back(fabs(estim[i][1]-gt[i][1]));
        delta_yaw.push_back(fabs(estim[i][2]-gt[i][2]));
    }
    double x_sum = accumulate(delta_x.begin(), delta_x.end(), 0.0);
    double y_sum = accumulate(delta_y.begin(), delta_y.end(), 0.0);
    double yaw_sum = accumulate(delta_yaw.begin(), delta_yaw.end(), 0.0);
    double x_mean = x_sum / delta_x.size();
    double y_mean = y_sum / delta_y.size();
    double yaw_mean = yaw_sum / delta_yaw.size();
    auto x_max = max_element(delta_x.begin(), delta_x.end());
    auto x_max_index = distance(delta_x.begin(), x_max);
    auto y_max = max_element(delta_y.begin(), delta_y.end());
    auto y_max_index = distance(delta_y.begin(), y_max);
    auto yaw_max = max_element(delta_yaw.begin(), delta_yaw.end());
    auto yaw_max_index = distance(delta_yaw.begin(), yaw_max);
    auto x_min = min_element(delta_x.begin(), delta_x.end());
    auto x_min_index = distance(delta_x.begin(), x_min);
    auto y_min = min_element(delta_y.begin(), delta_y.end());
    auto y_min_index = distance(delta_y.begin(), y_min);
    auto yaw_min = min_element(delta_yaw.begin(), delta_yaw.end());
    auto yaw_min_index = distance(delta_yaw.begin(), yaw_min);
    cout << "x_mean: " << x_mean << " "
    << "y_mean: " << y_mean << " "
    << "yaw_mean: " << yaw_mean << " " << endl;
    cout << "x_max: " <<  *x_max<< " "  << x_max_index << " "
    << "y_max: " << *y_max << " " << y_max_index << " "
    << "yaw_max: " << *yaw_max << " " << yaw_max_index << endl;
    cout << "x_min: " <<  *x_min<< " " << x_min_index << " "
    << "y_min: " << *y_min << " " << y_min_index << " "
    << "yaw_min: " << *yaw_min << " " << yaw_min_index << endl;
}
/**
 * Test sweep-sweep pose estimation
**/
int main(int argc, char **argv)
{
    typedef PointMatcher<float> PM;
    typedef PM::DataPoints DP;

    vector<vector<double>> gt_transform;  // gt transform
    vector<Eigen::Isometry3d> gt_pose_transform;  // gt transform

    vector<vector<double>> odom_transform;  // odom transform
    vector<Eigen::Isometry3d> odom_pose_transform;  // odom transform

    vector<vector<double>> icp_transform;  // icp transform
    vector<Eigen::Isometry3d> icp_pose_transform;  // icp transform
    vector<vector<double>> icp_init_transform;  // icp transform
    vector<Eigen::Isometry3d> icp_init_pose_transform;  // icp transform
    vector<double> icp_scores;
    vector<double> icp_init_scores;

    pcl::VoxelGrid<pcl::PointXYZ> downSizeFilter;
    downSizeFilter.setLeafSize(0.05f,0.05f,0.05f);

    vector<vector<double>> libpcl_transform;  // libpcl transform
    vector<vector<double>> libpcl_init_transform;  // libpcl transform with init

    // Loading the ground truth from gazebo, and calculate relative transformation
    string path_file_name = "/home/jing/dingo/myltr/map/race_high.yaml";
    string gazebo_file_name = "/home/jing/dingo/myltr/evo/gazebo.txt";
    string odom_file_name = "/home/jing/dingo/myltr/evo/odom.txt";
    string icp_file_name = "/home/jing/dingo/myltr/evo/icp.txt";
    string icp_init_file_name = "/home/jing/dingo/myltr/evo/icp_init.txt";
    ofstream f_gazebo, f_odom, f_icp, f_icp_init;
    f_gazebo.open(gazebo_file_name.c_str());
    f_odom.open(odom_file_name.c_str());
    f_icp.open(icp_file_name.c_str());
    f_icp_init.open(icp_init_file_name.c_str());



    FileStorage fsp(path_file_name, FileStorage::READ);

    vector<double> map_pose;
    vector<double> map_gt_poses;
    vector<vector<double>> gt_poses;
    

    if (fsp.isOpened())
    {
        fsp["map_pose"] >> map_pose;
        fsp["map_gt_poses"] >> map_gt_poses;
        fsp.release();
    }

    int num_scan = int(map_pose.size()/7);
    cout << "num_scan: " << num_scan << endl;

    auto start1 = high_resolution_clock::now();
    gt_poses.resize(num_scan);
    // Odom relative pose
    for(vector<double>::iterator it=map_pose.begin(); it!=(map_pose.end()-7); it=it+7)
    {
        Eigen::Isometry3d in_pose = Eigen::Isometry3d::Identity();
        Eigen::Vector3d in_translation(*it, *(it+1), *(it+2));
        Eigen::Quaterniond in_rotation(*(it+6), *(it+3), *(it+4), *(it+5));
        in_pose.rotate(in_rotation);
        in_pose.pretranslate(in_translation);

        Eigen::Isometry3d out_pose = Eigen::Isometry3d::Identity();
        Eigen::Vector3d out_translation(*(it+7), *(it+7+1), *(it+7+2));
        Eigen::Quaterniond out_rotation(*(it+7+6), *(it+7+3), *(it+7+4), *(it+7+5));
        out_pose.rotate(out_rotation);
        out_pose.pretranslate(out_translation);

        Eigen::Isometry3d transform = in_pose.inverse()*out_pose;
        odom_pose_transform.push_back(transform);
        vector<double> pose;
        pose.push_back(transform(0,3));
        pose.push_back(transform(1,3));
        pose.push_back(getYaw(transform));
        odom_transform.push_back(pose);
    }

    // Gazebo relative pose
    for(vector<double>::iterator it=map_gt_poses.begin(); it!=(map_gt_poses.end()-7); it=it+7)
    {
        Eigen::Isometry3d in_pose = Eigen::Isometry3d::Identity();
        Eigen::Vector3d in_translation(*it, *(it+1), *(it+2));
        Eigen::Quaterniond in_rotation(*(it+6), *(it+3), *(it+4), *(it+5));
        in_pose.rotate(in_rotation);
        in_pose.pretranslate(in_translation);

        Eigen::Isometry3d out_pose = Eigen::Isometry3d::Identity();
        Eigen::Vector3d out_translation(*(it+7), *(it+7+1), *(it+7+2));
        Eigen::Quaterniond out_rotation(*(it+7+6), *(it+7+3), *(it+7+4), *(it+7+5));
        out_pose.rotate(out_rotation);
        out_pose.pretranslate(out_translation);

        Eigen::Isometry3d transform = in_pose.inverse()*out_pose;
        gt_pose_transform.push_back(transform);
        vector<double> pose;
        pose.push_back(transform(0,3));
        pose.push_back(transform(1,3));
        pose.push_back(getYaw(transform));
        gt_transform.push_back(pose);
    }
    auto stop1 = high_resolution_clock::now();
    auto duration1 = duration_cast<microseconds>(stop1 - start1);
    std::cout << "odom, gt, Used: " << duration1.count()/1000 << "ms" << endl;

    // use ICP to calculate sweep-sweep pose
    auto start = high_resolution_clock::now();
    // Create the filtering object
    
    for(int i=0;i<(num_scan-1);i++)
    {
        string pcd_in_file = "/home/jing/dingo/myltr/map/race_high" + to_string(i) + ".pcd";
        string pcd_out_file = "/home/jing/dingo/myltr/map/race_high" + to_string(i+1) + ".pcd";
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_in (new pcl::PointCloud<pcl::PointXYZ>);
        // pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_in_filtered (new pcl::PointCloud<pcl::PointXYZ>);
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_out (new pcl::PointCloud<pcl::PointXYZ>);
        // pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_out_filtered (new pcl::PointCloud<pcl::PointXYZ>);
        if (pcl::io::loadPCDFile<pcl::PointXYZ> (pcd_in_file, *cloud_in) == -1) //* load the file
        {
            PCL_ERROR ("Couldn't read file %s\n", pcd_in_file);
        }
        if (pcl::io::loadPCDFile<pcl::PointXYZ> (pcd_out_file, *cloud_out) == -1) //* load the file
        {
            PCL_ERROR ("Couldn't read file %s\n", pcd_out_file);
        }
        // Downsize
        // downSizeFilter.setInputCloud(cloud_in);
        // downSizeFilter.filter(*cloud_in_filtered);
        // downSizeFilter.setInputCloud(cloud_out);
        // downSizeFilter.filter(*cloud_out_filtered);

        pcl::IterativeClosestPoint<pcl::PointXYZ, pcl::PointXYZ> icp;
        icp.setInputSource(cloud_out);
        icp.setInputTarget(cloud_in);

        icp.setMaxCorrespondenceDistance(0.07);
        
        
        pcl::PointCloud<pcl::PointXYZ> Final;
        icp.align(Final);
        Eigen::Matrix<double, 4, 4> transform;
        transform = icp.getFinalTransformation().cast<double>();
        Eigen::Isometry3d _transform(transform);
        vector<double> pose;
        pose.push_back(transform(0,3));
        pose.push_back(transform(1,3));
        pose.push_back(getYaw(transform));
        icp_scores.push_back(icp.getFitnessScore());
        icp_transform.push_back(pose);
        icp_pose_transform.push_back(_transform);
    }

    auto stop = high_resolution_clock::now();
    auto duration = duration_cast<microseconds>(stop - start);
    cout << "original icp, Used: " << duration.count()/1000 << "ms " << "Speed: " << duration.count()/1000/num_scan << "ms" << endl;

    // use initialization to reinforcement ICP
    start = high_resolution_clock::now();
    for(int i=0;i<(num_scan-1);i++)
    {
        string pcd_in_file = "/home/jing/dingo/myltr/map/race_high" + to_string(i) + ".pcd";
        string pcd_out_file = "/home/jing/dingo/myltr/map/race_high" + to_string(i+1) + ".pcd";
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_in (new pcl::PointCloud<pcl::PointXYZ>);
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_in_filtered (new pcl::PointCloud<pcl::PointXYZ>);
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_out (new pcl::PointCloud<pcl::PointXYZ>);
        // pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_out_filtered (new pcl::PointCloud<pcl::PointXYZ>);
        if (pcl::io::loadPCDFile<pcl::PointXYZ> (pcd_in_file, *cloud_in) == -1) //* load the file
        {
            PCL_ERROR ("Couldn't read file %s\n", pcd_in_file);
        }
        if (pcl::io::loadPCDFile<pcl::PointXYZ> (pcd_out_file, *cloud_out) == -1) //* load the file
        {
            PCL_ERROR ("Couldn't read file %s\n", pcd_out_file);
        }
        // Downsize
        // downSizeFilter.setInputCloud(cloud_in);
        // downSizeFilter.filter(*cloud_in_filtered);
        // downSizeFilter.setInputCloud(cloud_out);
        // downSizeFilter.filter(*cloud_out_filtered);

        // set init
        pcl::transformPointCloud(*cloud_in, *cloud_in_filtered, odom_pose_transform[i].matrix().inverse().cast<float>());
        pcl::IterativeClosestPoint<pcl::PointXYZ, pcl::PointXYZ> icp;
        icp.setInputSource(cloud_out);
        icp.setInputTarget(cloud_in_filtered);
        

        pcl::PointCloud<pcl::PointXYZ> Final;
        icp.align(Final);
        Eigen::Matrix<double, 4, 4> transform;
        transform = icp.getFinalTransformation().cast<double>();
        transform = odom_pose_transform[i].matrix()*transform;
        vector<double> pose;
        pose.push_back(transform(0,3));
        pose.push_back(transform(1,3));
        pose.push_back(getYaw(transform));
        icp_init_scores.push_back(icp.getFitnessScore());
        icp_init_transform.push_back(pose);
        Eigen::Isometry3d _transform(transform);
        icp_init_pose_transform.push_back(_transform);
    }
    stop = high_resolution_clock::now();
    duration = duration_cast<microseconds>(stop - start);
    cout << "rein icp, Used: " << duration.count()/1000 << "ms " << "Speed: " << duration.count()/1000/num_scan << "ms" << endl;

    // For evo ape
    // Eigen::Isometry3d odom_trans;
    // Eigen::Isometry3d gt_trans;
    // Eigen::Isometry3d icp_trans;
    // Eigen::Isometry3d icp_init_trans;
    // for(int i=0;i<num_scan-1;i++)
    // {
    //     if(i==0)
    //     {   
    //         odom_trans = odom_pose_transform[i];
    //         gt_trans = gt_pose_transform[i];
    //         icp_trans = icp_pose_transform[i];
    //         icp_init_trans = icp_init_pose_transform[i];
    //     }
    //     else
    //     {   
    //         odom_trans = odom_pose_transform[i].inverse() * odom_trans;
    //         gt_trans = gt_pose_transform[i].inverse() * gt_trans;
    //         icp_trans = icp_pose_transform[i].inverse() * icp_trans;
    //         icp_init_trans = icp_init_pose_transform[i].inverse() * icp_init_trans;
    //     }
    //     Eigen::Quaterniond q_odom(odom_trans.rotation());
    //     Eigen::Quaterniond q_gt(gt_trans.rotation());
    //     Eigen::Quaterniond q_icp(icp_trans.rotation());
    //     Eigen::Quaterniond q_icp_init(icp_init_trans.rotation());
    //     f_odom << i << " " << odom_trans.translation()[0] << " " << odom_trans.translation()[1] << " " << odom_trans.translation()[2] << " "
    //             << q_odom.x() << " " << q_odom.y() << " " << q_odom.z() << " " << q_odom.w() << endl;
    //     f_gazebo << i << " " << gt_trans.translation()[0] << " " << gt_trans.translation()[1] << " " << gt_trans.translation()[2] << " "
    //             << q_gt.x() << " " << q_gt.y() << " " << q_gt.z() << " " << q_gt.w() << endl;
    //     f_icp << i << " " << icp_trans.translation()[0] << " " << icp_trans.translation()[1] << " " << icp_trans.translation()[2] << " "
    //             << q_icp.x() << " " << q_icp.y() << " " << q_icp.z() << " " << q_icp.w() << endl;
    //     f_icp_init << i << " " << icp_init_trans.translation()[0] << " " << icp_init_trans.translation()[1] << " " << icp_init_trans.translation()[2] << " "
    //             << q_icp_init.x() << " " << q_icp_init.y() << " " << q_icp_init.z() << " " << q_icp_init.w() << endl;
    // }
    // f_odom.close();
    // f_gazebo.close();
    // f_icp.close();
    // f_icp_init.close();
    // For evo rpe
    Eigen::Isometry3d odom_trans;
    Eigen::Isometry3d gt_trans;
    Eigen::Isometry3d icp_trans;
    Eigen::Isometry3d icp_init_trans;
    for(int i=0;i<num_scan-1;i++)
    {
        Eigen::Quaterniond q_odom(odom_pose_transform[i].rotation());
        Eigen::Quaterniond q_gt(gt_pose_transform[i].rotation());
        Eigen::Quaterniond q_icp(icp_pose_transform[i].rotation());
        Eigen::Quaterniond q_icp_init(icp_init_pose_transform[i].rotation());
        f_odom << i << " " << odom_pose_transform[i].translation()[0] << " " << odom_pose_transform[i].translation()[1] << " " << odom_pose_transform[i].translation()[2] << " " << q_odom.x() << " " << q_odom.y() << " " << q_odom.z() << " " << q_odom.w() << endl;

        f_gazebo << i << " " << gt_pose_transform[i].translation()[0] << " " << gt_pose_transform[i].translation()[1] << " " << gt_pose_transform[i].translation()[2] << " "
                << q_gt.x() << " " << q_gt.y() << " " << q_gt.z() << " " << q_gt.w() << endl;
        f_icp << i << " " << icp_pose_transform[i].translation()[0] << " " << icp_pose_transform[i].translation()[1] << " " << icp_pose_transform[i].translation()[2] << " "
                << q_icp.x() << " " << q_icp.y() << " " << q_icp.z() << " " << q_icp.w() << endl;
        f_icp_init << i << " " << icp_init_pose_transform[i].translation()[0] << " " << icp_init_pose_transform[i].translation()[1] << " " << icp_init_pose_transform[i].translation()[2] << " "
                << q_icp_init.x() << " " << q_icp_init.y() << " " << q_icp_init.z() << " " << q_icp_init.w() << endl;
    }
    f_odom.close();
    f_gazebo.close();
    f_icp.close();
    f_icp_init.close();

    // use libpointmatcher
    start = high_resolution_clock::now();
    for(int i=0;i<(num_scan-1);i++)
    {
        string pcd_in_file = "/home/jing/dingo/myltr/map/race_high" + to_string(i) + ".pcd";
        string pcd_out_file = "/home/jing/dingo/myltr/map/race_high" + to_string(i+1) + ".pcd";
        const DP ref(DP::load(pcd_in_file));
	    const DP data(DP::load(pcd_out_file));
        // Create the default ICP algorithm
	    PM::ICP icp;
        // See the implementation of setDefault() to create a custom ICP algorithm
        // icp.loadFromYaml(is);
	    icp.setDefault();
        // Compute the transformation to express data in ref
	    PM::TransformationParameters transform = icp(data, ref);

        vector<double> pose;
        pose.push_back(transform(0,3));
        pose.push_back(transform(1,3));
        pose.push_back(getYaw(transform.cast<double>()));
        libpcl_transform.push_back(pose);
    }
    stop = high_resolution_clock::now();
    duration = duration_cast<microseconds>(stop - start);
    cout << "libpcl icp, Used: " << duration.count()/1000 << "ms " << "Speed: " << duration.count()/1000/num_scan << "ms" << endl;

    // use libpointmatcher with initialization
    start = high_resolution_clock::now();
    for(int i=0;i<(num_scan-1);i++)
    {
        string pcd_in_file = "/home/jing/dingo/myltr/map/race_high" + to_string(i) + ".pcd";
        string pcd_out_file = "/home/jing/dingo/myltr/map/race_high" + to_string(i+1) + ".pcd";
        const DP ref(DP::load(pcd_in_file));
	    const DP data(DP::load(pcd_out_file));
        // Create the default ICP algorithm
	    PM::ICP icp;
	    icp.setDefault();
        PM::TransformationParameters initTransformation = odom_pose_transform[i].matrix().inverse().cast<float>();  // from 2 to 1
        std::shared_ptr<PM::Transformation> rigidTrans;
	    rigidTrans = PM::get().REG(Transformation).create("RigidTransformation");
        if (!rigidTrans->checkParameters(initTransformation)) {
            cerr << endl
                << "Initial transformation is not rigid, identiy will be used"
                << endl;
            initTransformation = PM::TransformationParameters::Identity(4,4);
	    }
        const DP initializedData = rigidTrans->compute(data, initTransformation);
        PM::TransformationParameters T = icp(initializedData, ref);
        Eigen::Matrix4d transform = odom_pose_transform[i].matrix()*T.cast<double>();


        vector<double> pose;
        pose.push_back(transform(0,3));
        pose.push_back(transform(1,3));
        pose.push_back(getYaw(transform.cast<double>()));
        libpcl_init_transform.push_back(pose);
    }
    stop = high_resolution_clock::now();
    duration = duration_cast<microseconds>(stop - start);
    cout << "libpcl_init icp, Used: " << duration.count()/1000 << "ms " << "Speed: " << duration.count()/1000/num_scan << "ms" << endl;


    cout << "ODOM" << endl;
    cal(gt_transform, odom_transform);
    cout << "ICP" << endl;
    cal(gt_transform, icp_transform);
    cout << "REIN ICP" << endl;
    cal(gt_transform, icp_init_transform);
    cout << "LIB ICP" << endl;
    cal(gt_transform, libpcl_transform);
    cout << "LIB INIT ICP" << endl;
    cal(gt_transform, libpcl_init_transform);
    cout << "ICP MAX: " << icp_scores[24] << endl;
    cout << "ICP_INIT MAX: " << icp_init_scores[24] << endl;
    cout << "ICP MIN: " << icp_scores[76] << endl;
    cout << "ICP_INIT MIN: " << icp_init_scores[12] << endl;
    return 0;
}
