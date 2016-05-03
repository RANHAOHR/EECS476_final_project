// wsn, April, 2016
// illustrates use of baxter_cart_move_as, action server called "cartMoveActionServer"

#include <ros/ros.h>
#include <stdlib.h>
#include <math.h>
#include <actionlib/client/simple_action_client.h>
#include <actionlib/server/simple_action_server.h>
#include <actionlib/client/terminal_state.h>
#include <iostream> 
#include <sensor_msgs/PointCloud2.h> 
#include <pcl_ros/point_cloud.h> //to convert between PCL and ROS
#include <pcl/ros/conversions.h>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <geometry_msgs/PoseStamped.h>

//#include <pcl/PCLPointCloud2.h> //PCL is migrating to PointCloud2 
#include <pcl/common/common_headers.h>
#include <pcl-1.7/pcl/point_cloud.h>
#include <pcl-1.7/pcl/PCLHeader.h>

//will use filter objects "passthrough" and "voxel_grid" in this example
#include <pcl/filters/passthrough.h>
#include <pcl/filters/voxel_grid.h> 

using namespace std;

// extern PclUtils *g_pcl_utils_ptr; 
#include <object_finder/object_finderAction.h>
#include "object_finder.cpp" // and another class def here
#include <pcl_utils/pcl_utils.h>  // whyyyyyyyyyyyyyyy

int main(int argc, char** argv) {
    ros::init(argc, argv, "object_finder_action_server_node"); // name this node 
    ros::NodeHandle nh; //standard ros node handle   
    PclUtils pclUtils(&nh); // hueheuheuehujhueihe
    ObjectFinder object_finder_as(&nh,&pclUtils); // create an instance of the class "ObjectFinder", containing an action server
    ROS_INFO("going into spin");
    while (ros::ok()) {
        ros::spinOnce(); //normally, can simply do: ros::spin();  
    }
    return 0;
}