#ifndef BLOCK_SORTER_BLOCK_SORTER_H
#define BLOCK_SORTER_BLOCK_SORTER_H

#include <gripper_controller/gripper_controller.h>
#include <motion_planning/motion_planning_lib.h>
#include <block_finder/final_pcl_utils.h>
#include <pcl_utils/pcl_utils.h>

class BlockSorter
{
public:
    BlockSorter();
    void find_coke_can();
    void doBlockSort();
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr pclKinect_clr_ptr; //pointer for color version of pointcloud
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr plane_pts_ptr; //pointer for pointcloud of planar points found
    pcl::PointCloud<pcl::PointXYZ>::Ptr selected_pts_cloud_ptr; //ptr to selected pts from Rvis tool
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr downsampled_kinect_ptr; //ptr to hold filtered Kinect image

    pcl::PointCloud<pcl::PointXYZRGB>::Ptr rough_table_cloud_ptr; //ptr to table pts from Rvis tool    
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr can_pts_cloud_ptr;

    pcl::PointCloud<pcl::PointXYZ>::Ptr table_xyz_ptr;
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr final_table_cloud_ptr; //ptr to table pts from Rvis tool 

    ros::Publisher pubCloud;
    ros::Publisher pubPlane;
    ros::Publisher pubDnSamp;

    sensor_msgs::PointCloud2 tablePts; //create a ROS message
    ros::Publisher table;

    sensor_msgs::PointCloud2 canPts;
    ros::Publisher Can;    

    sensor_msgs::PointCloud2 ros_cloud, table_planar_cloud, ros_planar_cloud, downsampled_cloud; //here are ROS-compatible messages
    PclUtils pclUtils;

private:
    ros::NodeHandle nh;
    FinalPclUtils final_pcl_utils;
    MotionPlanning motion_planning;
    GripperController gripper_controller;
    Eigen::Vector3f goalPoint;
    Eigen::Vector3f goalOrientation;
    Eigen::Vector3d goalColor;
    pcl::PointCloud<pcl::PointXYZRGB> display_cloud; 
    sensor_msgs::PointCloud2 pcl2_display_cloud;
    tf::StampedTransform tf_sensor_frame_to_torso_frame;
    tf::TransformListener tf_listener;
    Eigen::Affine3f A_sensor_wrt_torso;
    Eigen::Vector3d red;
    Eigen::Vector3d blue;
    Eigen::Vector3d green;
};

#endif
