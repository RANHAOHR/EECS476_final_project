#include <block_sorter/block_sorter.h>
#include <geometry_msgs/PoseStamped.h>

#include <ros/ros.h> 
#include <stdlib.h>
#include <math.h>

#include <sensor_msgs/PointCloud2.h> 
#include <pcl_ros/point_cloud.h> //to convert between PCL and ROS
#include <pcl/ros/conversions.h>

#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
//#include <pcl/PCLPointCloud2.h> //PCL is migrating to PointCloud2 

#include <pcl/common/common_headers.h>
#include <pcl-1.7/pcl/point_cloud.h>
#include <pcl-1.7/pcl/PCLHeader.h>

//will use filter objects "passthrough" and "voxel_grid" in this example
#include <pcl/filters/passthrough.h>
#include <pcl/filters/voxel_grid.h> 

#include <pcl_utils/pcl_utils.h>  //a local library with some utility fncs
#include <iostream> 

using namespace std;

//this fnc is defined in a separate module, find_indices_of_plane_from_patch.cpp
extern void find_indices_of_plane_from_patch(pcl::PointCloud<pcl::PointXYZRGB>::Ptr input_cloud_ptr,
	pcl::PointCloud<pcl::PointXYZ>::Ptr patch_cloud_ptr, vector<int> &indices);

BlockSorter::BlockSorter():
    pclKinect_clr_ptr(new pcl::PointCloud<pcl::PointXYZRGB>),
    plane_pts_ptr(new pcl::PointCloud<pcl::PointXYZRGB>),
    selected_pts_cloud_ptr(new pcl::PointCloud<pcl::PointXYZ>),
    downsampled_kinect_ptr(new pcl::PointCloud<pcl::PointXYZRGB>),
    rough_table_cloud_ptr(new pcl::PointCloud<pcl::PointXYZRGB>),
    can_pts_cloud_ptr(new pcl::PointCloud<pcl::PointXYZRGB>),
    table_xyz_ptr(new pcl::PointCloud<pcl::PointXYZ>),
    final_table_cloud_ptr(new pcl::PointCloud<pcl::PointXYZRGB>), 
    final_pcl_utils(&nh),
    pclUtils(&nh),
    motion_planning(&nh),
    gripper_controller(&nh)
{   

    pubCloud = nh.advertise<sensor_msgs::PointCloud2> ("/pcd", 1);
    pubPlane = nh.advertise<sensor_msgs::PointCloud2> ("planar_pts", 1);
    pubDnSamp = nh.advertise<sensor_msgs::PointCloud2> ("downsampled_pcd", 1);
    table = nh.advertise<sensor_msgs::PointCloud2> ("/table_pts", 1);
    Can = nh.advertise<sensor_msgs::PointCloud2> ("/coke_can_pts", 1);  

}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "block_sorter");
    ros::NodeHandle nh;

    BlockSorter block_sorter;
    std::string keyboard_input;

    ROS_INFO("Type p to pause, c to continue, q to quit");
    while(ros::ok())
    {
            cin >> keyboard_input;
            if (keyboard_input == "p")
            {
                ROS_INFO("Block sorting is currently paused, press c to continue or q to quit");
                cin >> keyboard_input;
            }
            if (keyboard_input == "c")
            {
                ROS_INFO("Continuing block sorting...");
                block_sorter.doBlockSort();
            }
            if (keyboard_input == "q")
            {
                ROS_INFO("Exiting...");
                exit(1);
            }
            if (keyboard_input != "p" && keyboard_input != "c" && keyboard_input != "q")
            {
                ROS_INFO("Invalid keyboard input, try again");
            }
            ROS_INFO("Block sorting complete, press p to pause, c to sort another block, or q to quit");
    }
}

void BlockSorter::find_coke_can(){    

	    vector<int> indices;

        cout << "------------------------------------------------" << endl;
    	if (pclUtils.got_kinect_cloud_)
    	{
    		pclUtils.get_kinect_points(pclKinect_clr_ptr);
	    //will publish  pointClouds as ROS-compatible messages; create publishers; note topics for rviz viewing        
	        pcl::toROSMsg(*pclKinect_clr_ptr, ros_cloud); //convert from PCL cloud to ROS message this way

	    //use voxel filtering to downsample the original cloud:
	        cout << "starting voxel filtering" << endl;
	        pcl::VoxelGrid<pcl::PointXYZRGB> vox;
	        vox.setInputCloud(pclKinect_clr_ptr);

	        vox.setLeafSize(0.02f, 0.02f, 0.02f);
	        vox.filter(*downsampled_kinect_ptr);
	        cout << "done voxel filtering" << endl;//camera/depth_registered/points;

	        cout << "num bytes in original cloud data = " << pclKinect_clr_ptr->points.size() << endl;
	        cout << "num bytes in filtered cloud data = " << downsampled_kinect_ptr->points.size() << endl; // ->data.size()<<endl;    
	        pcl::toROSMsg(*downsampled_kinect_ptr, downsampled_cloud); //convert to ros message for publication and display

	    //*********************************************************************************************************************************//        
	        pclUtils.seek_rough_table_merry(pclKinect_clr_ptr, 0, rough_table_cloud_ptr);

	        pclUtils.from_RGB_to_XYZ(rough_table_cloud_ptr, table_xyz_ptr);
	        find_indices_of_plane_from_patch(downsampled_kinect_ptr, table_xyz_ptr, indices);    
	        pcl::copyPointCloud(*downsampled_kinect_ptr, indices, *plane_pts_ptr); //extract these pts into new cloud

	        pcl::toROSMsg(*plane_pts_ptr, table_planar_cloud); //rough table cloud

            final_table_cloud_ptr->points.clear();
	        pclUtils.find_final_table_merry(plane_pts_ptr, final_table_cloud_ptr);
            if (pclUtils.is_final_cloud)
            {
                pcl::toROSMsg(*final_table_cloud_ptr, tablePts); //final table cloud
            }
	        
            pubCloud.publish(ros_cloud); // will not need to keep republishing if display setting is persistent
            pubDnSamp.publish(downsampled_cloud); 

            pubPlane.publish(table_planar_cloud); // should be the rough table, to debug            
            table.publish(tablePts);  //final table
	        //the new cloud is a set of points from original cloud, coplanar with selected patch; display the result

	        pclUtils.seek_coke_can_cloud(pclKinect_clr_ptr, can_pts_cloud_ptr, goalPoint);

	        bool can_exist = pclUtils.is_coke_can(can_pts_cloud_ptr);
	        Eigen::Vector3f can_bottom;

	        if (can_exist)
	        {
	        	ROS_WARN("coke can detected !");
	            can_bottom = pclUtils.find_can_bottom(final_table_cloud_ptr); //find bottom in table
	            pcl::toROSMsg(*can_pts_cloud_ptr, canPts);
                Can.publish(canPts);
	            ROS_INFO("The x y z of the can bottom is: (%f, %f, %f)", can_bottom[0], can_bottom[1], can_bottom[2]);
	        }
	        else{
                ROS_WARN("NO coke can!");
                Can.publish(tablePts);

	        }
	        pclUtils.got_kinect_cloud_ = false;
	    }
	    else{
	    	ROS_WARN("NO Kinect callback !");
	    }
        
        //ros::spinOnce(); //pclUtils needs some spin cycles to invoke callbacks for new selected points
        ros::Duration(0.5).sleep();
}
void BlockSorter::doBlockSort()
{
    ROS_INFO("Moving to prepose");
    motion_planning.plan_move_to_pre_pose();
    motion_planning.rt_arm_execute_planned_path();

    find_coke_can();    
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    ROS_INFO("Converting goal pose to geometry_msgs");
    ROS_INFO_STREAM(goalPoint[0] << "," << goalPoint[1] << "," << goalPoint[2]);
    geometry_msgs::PoseStamped goalPose = final_pcl_utils.eigenToPose(goalPoint);
    int rtn_val = motion_planning.rt_arm_request_tool_pose_wrt_torso();
    geometry_msgs::PoseStamped rt_tool_pose = motion_planning.get_rt_tool_pose_stamped();
    rt_tool_pose.pose.position.x = goalPose.pose.position.x;
    rt_tool_pose.pose.position.y = goalPose.pose.position.y;
    rt_tool_pose.pose.position.z = goalPose.pose.position.z + .2;

    ROS_INFO("Going to goal pose");
    motion_planning.rt_arm_plan_path_current_to_goal_pose(rt_tool_pose); 
    motion_planning.rt_arm_execute_planned_path();

    gripper_controller.open();
    ros::spinOnce();

    ROS_INFO("Descending to block");
    rtn_val = motion_planning.rt_arm_request_tool_pose_wrt_torso();
    rt_tool_pose = motion_planning.get_rt_tool_pose_stamped();
    rt_tool_pose.pose.position.z -= .08;
    motion_planning.rt_arm_plan_path_current_to_goal_pose(rt_tool_pose); 
    motion_planning.rt_arm_execute_planned_path();
   
    gripper_controller.close();
    ros::spinOnce();
    ros::Duration(2.0).sleep();
    
    motion_planning.plan_move_to_pre_pose();

    gripper_controller.open();
    ros::spinOnce();
}
