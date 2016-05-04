// example_action_server: a simple action server
// Wyatt Newman

#include <ros/ros.h>
#include <actionlib/server/simple_action_server.h>
//the following #include refers to the "action" message defined for this package
// The action message can be found in: .../example_action_server/action/demo.action
// Automated header generation creates multiple headers for message I/O
// These are referred to by the root name (demo) and appended name (Action)
#include <pcl_utils/pcl_utils.h>
#include <object_finder/objectFinderAction.h>
#include <stdlib.h>
#include <math.h>
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


class ObjectFinder {
private:

    ros::NodeHandle nh_;  // we'll need a node handle; get one upon instantiation

    // this class will own a "SimpleActionServer" called "as_".
    // it will communicate using messages defined in example_action_server/action/demo.action
    // the type "demoAction" is auto-generated from our name "demo" and generic name "Action"
    actionlib::SimpleActionServer<object_finder::objectFinderAction> object_finder_as_;
    
    // here are some message types to communicate with our client(s)
    object_finder::objectFinderGoal goal_; // goal message, received from client
    object_finder::objectFinderResult result_; // put results here, to be sent back to the client when done w/ goal
    object_finder::objectFinderFeedback feedback_; // not used in this example; 
    // would need to use: as_.publishFeedback(feedback_); to send incremental feedback to the client

    PclUtils pclUtils_;
    //specialized function to find an upright Coke can on known height of horizontal surface;
    // returns true/false for found/not-found, and if found, fills in the object pose
    bool find_upright_coke_can(float surface_height,geometry_msgs::PoseStamped &object_pose);
    
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr pclKinect_clr_ptr; //pointer for color version of pointcloud
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr plane_pts_ptr; //pointer for pointcloud of planar points found
    pcl::PointCloud<pcl::PointXYZ>::Ptr selected_pts_cloud_ptr; //ptr to selected pts from Rvis tool
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr downsampled_kinect_ptr; //ptr to hold filtered Kinect image

    pcl::PointCloud<pcl::PointXYZRGB>::Ptr rough_table_cloud_ptr; //ptr to table pts from Rvis tool    
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr can_pts_cloud_ptr;

    pcl::PointCloud<pcl::PointXYZ>::Ptr table_xyz_ptr;
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr final_table_cloud_ptr; //ptr to table pts from Rvis tool 

    vector<int> indices;

    ros::Publisher pubCloud;
    ros::Publisher pubPlane;
    ros::Publisher pubDnSamp;

    sensor_msgs::PointCloud2 tablePts; //create a ROS message
    ros::Publisher table;

    sensor_msgs::PointCloud2 canPts;
    ros::Publisher Can;

    sensor_msgs::PointCloud2 ros_cloud, table_planar_cloud, ros_planar_cloud, downsampled_cloud; //here are ROS-compatible messages    
    //action callback fnc
    void find_indices_of_plane_from_patch(pcl::PointCloud<pcl::PointXYZRGB>::Ptr input_cloud_ptr,
        pcl::PointCloud<pcl::PointXYZ>::Ptr patch_cloud_ptr, vector<int> &indices);

public:
    ObjectFinder(); //define the body of the constructor outside of class definition

    ~ObjectFinder(void) {
    }
    // Action Interface
    void executeCB(const actionlib::SimpleActionServer<object_finder::objectFinderAction>::GoalConstPtr& goal);
};



ObjectFinder::ObjectFinder() :
   object_finder_as_(nh_, "objectFinderActionServer", boost::bind(&ObjectFinder::executeCB, this, _1),false),
   pclUtils_(&nh_),
   pclKinect_clr_ptr(new pcl::PointCloud<pcl::PointXYZRGB>),
   plane_pts_ptr(new pcl::PointCloud<pcl::PointXYZRGB>),
   selected_pts_cloud_ptr(new pcl::PointCloud<pcl::PointXYZ>),
   downsampled_kinect_ptr(new pcl::PointCloud<pcl::PointXYZRGB>),
   rough_table_cloud_ptr(new pcl::PointCloud<pcl::PointXYZRGB>),
   can_pts_cloud_ptr(new pcl::PointCloud<pcl::PointXYZRGB>),
   table_xyz_ptr(new pcl::PointCloud<pcl::PointXYZ>),
   final_table_cloud_ptr(new pcl::PointCloud<pcl::PointXYZRGB>) {
    ROS_INFO("in constructor of ObjectFinder...");
    // do any other desired initializations here...specific to your implementation
    pubCloud = nh_.advertise<sensor_msgs::PointCloud2> ("/pcd", 1);
    pubPlane = nh_.advertise<sensor_msgs::PointCloud2> ("planar_pts", 1);
    pubDnSamp = nh_.advertise<sensor_msgs::PointCloud2> ("downsampled_pcd", 1);
    table = nh_.advertise<sensor_msgs::PointCloud2> ("/table_pts", 1);
    Can = nh_.advertise<sensor_msgs::PointCloud2> ("/coke_can_pts", 1); 
    object_finder_as_.start(); //start the server running
}

//specialized function: DUMMY...JUST RETURN A HARD-CODED POSE; FIX THIS
bool ObjectFinder::find_upright_coke_can(float surface_height,geometry_msgs::PoseStamped &object_pose) {
	pclUtils_.get_kinect_points(pclKinect_clr_ptr);
    //will publish  pointClouds as ROS-compatible messages; create publishers; note topics for rviz viewing        
    pcl::toROSMsg(*pclKinect_clr_ptr, ros_cloud); //convert from PCL cloud to ROS message this way

    //use voxel filtering to downsample the original cloud:
    // cout << "starting voxel filtering" << endl;
    pcl::VoxelGrid<pcl::PointXYZRGB> vox;
    vox.setInputCloud(pclKinect_clr_ptr);

    vox.setLeafSize(0.02f, 0.02f, 0.02f);
    vox.filter(*downsampled_kinect_ptr);
    // cout << "done voxel filtering" << endl;//camera/depth_registered/points;

    // cout << "num bytes in original cloud data = " << pclKinect_clr_ptr->points.size() << endl;
    // cout << "num bytes in filtered cloud data = " << downsampled_kinect_ptr->points.size() << endl; // ->data.size()<<endl;    
    pcl::toROSMsg(*downsampled_kinect_ptr, downsampled_cloud); //convert to ros message for publication and display

//*********************************************************************************************************************************//        
    pclUtils_.seek_rough_table_merry(pclKinect_clr_ptr, 0, rough_table_cloud_ptr);

    pclUtils_.from_RGB_to_XYZ(rough_table_cloud_ptr, table_xyz_ptr);
    find_indices_of_plane_from_patch(downsampled_kinect_ptr, table_xyz_ptr, indices);    
    pcl::copyPointCloud(*downsampled_kinect_ptr, indices, *plane_pts_ptr); //extract these pts into new cloud

    pcl::toROSMsg(*plane_pts_ptr, table_planar_cloud); //rough table cloud

    final_table_cloud_ptr->points.clear();
    pclUtils_.find_final_table_merry(plane_pts_ptr, final_table_cloud_ptr);
    if (pclUtils_.is_final_cloud) {
        pcl::toROSMsg(*final_table_cloud_ptr, tablePts); //final table cloud
    }
    
    pubCloud.publish(ros_cloud); // will not need to keep republishing if display setting is persistent
    pubDnSamp.publish(downsampled_cloud); 

    pubPlane.publish(table_planar_cloud); // should be the rough table, to debug            
    table.publish(tablePts);  //final table
    //the new cloud is a set of points from original cloud, coplanar with selected patch; display the result

    pclUtils_.seek_coke_can_cloud(pclKinect_clr_ptr, can_pts_cloud_ptr);

    bool can_exist = pclUtils_.is_coke_can(can_pts_cloud_ptr);
    Eigen::Vector3f can_top;

    if (can_exist) {
        // ROS_WARN("coke can detected !");
        can_top = pclUtils_.find_can_bottom(final_table_cloud_ptr); //find bottom in table
        pcl::toROSMsg(*can_pts_cloud_ptr, canPts);
        Can.publish(canPts);
        can_top[0] = can_top[0] + 0.73; /////have to correct the tf!!!!
        can_top[1] = can_top[1] - 0.15;
        can_top[2] = can_top[2] - 0.9;

        ROS_INFO("The x y z of the can top is: (%f, %f, %f)", can_top[0], can_top[1], can_top[2]);
        object_pose.pose.position.x = can_top[0];
        object_pose.pose.position.y = can_top[1];
        object_pose.pose.position.z = can_top[2];
        return true;
        // find_result.object_pose = object_pose_;
        // find_result.found_object_code = object_finder::object_finderResult::OBJECT_FOUND;
    }
    else {
        ROS_WARN("NO coke can!");
        Can.publish(tablePts);
        return false;
        // find_result_.found_object_code = object_finder::object_finderResult::OBJECT_NOT_RECOGNIZED;
    }
    // pclUtils_.got_kinect_cloud_ = false;
    // object_finder_as_.setSucceeded(find_result_);
    // bool found_object=true;
    object_pose.header.frame_id = "torso";
    // object_pose.pose.position.x = 0.680;
    // object_pose.pose.position.y = -0.205;
    // object_pose.pose.position.z = surface_height;
    // object_pose.pose.orientation.x = 0;
    // object_pose.pose.orientation.y = 0;
    // object_pose.pose.orientation.z = 0;
    // object_pose.pose.orientation.w = 1;   
    // return found_object;
    
}

//executeCB implementation: this is a member method that will get registered with the action server
// argument type is very long.  Meaning:
// actionlib is the package for action servers
// SimpleActionServer is a templated class in this package (defined in the "actionlib" ROS package)
// <example_action_server::demoAction> customizes the simple action server to use our own "action" message 
// defined in our package, "example_action_server", in the subdirectory "action", called "demo.action"
// The name "demo" is prepended to other message types created automatically during compilation.
// e.g.,  "demoAction" is auto-generated from (our) base name "demo" and generic name "Action"
void ObjectFinder::executeCB(const actionlib::SimpleActionServer<object_finder::objectFinderAction>::GoalConstPtr& goal) {
    int object_id = goal->object_id;
    geometry_msgs::PoseStamped object_pose;
    bool known_surface_ht = goal->known_surface_ht;
    float surface_height;
    if (known_surface_ht) {
        surface_height=goal->surface_ht;
    }
    bool found_object=false;
    switch(object_id) {
        case object_finder::objectFinderGoal::COKE_CAN_UPRIGHT: 
              //specialized function to find an upright Coke can on a horizontal surface of known height:
               found_object = find_upright_coke_can(surface_height,object_pose); //special case for Coke can;
               if (found_object) {
                   ROS_INFO("found upright Coke can!");
                   result_.found_object_code = object_finder::objectFinderResult::OBJECT_FOUND;
                   result_.object_pose = object_pose;
                   object_finder_as_.setSucceeded(result_);
               }
               else {
                   ROS_WARN("could not find requested object");
                   result_.found_object_code = object_finder::objectFinderResult::OBJECT_NOT_FOUND;
                   object_finder_as_.setAborted(result_);
               }
               break;
        default:
             ROS_WARN("this object ID is not implemented");
             result_.found_object_code = object_finder::objectFinderResult::OBJECT_CODE_NOT_RECOGNIZED; 
             object_finder_as_.setAborted(result_);
            }
  
}

void ObjectFinder::find_indices_of_plane_from_patch(pcl::PointCloud<pcl::PointXYZRGB>::Ptr input_cloud_ptr,
        pcl::PointCloud<pcl::PointXYZ>::Ptr patch_cloud_ptr, vector<int> &indices) {
    float curvature;
    Eigen::Vector4f plane_parameters;
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr transformed_cloud_ptr(new pcl::PointCloud<pcl::PointXYZRGB>); //pointer for color version of pointcloud
    pcl::computePointNormal(*patch_cloud_ptr, plane_parameters, curvature); //pcl fnc to compute plane fit to point cloud
    // cout << "PCL: plane params of patch: " << plane_parameters.transpose() << endl;
    //next, define a coordinate frame on the plane fitted to the patch.
    // choose the z-axis of this frame to be the plane normal--but enforce that the normal
    // must point towards the camera
    Eigen::Affine3f A_plane_wrt_camera;
    // here, use a utility function in pclUtils to construct a frame on the computed plane
    A_plane_wrt_camera = pclUtils_.make_affine_from_plane_params(plane_parameters);
    // cout << "A_plane_wrt_camera rotation:" << endl;
    // cout << A_plane_wrt_camera.linear() << endl;
    // cout << "origin: " << A_plane_wrt_camera.translation().transpose() << endl;
    //next, transform all points in input_cloud into the plane frame.
    //the result of this is, points that are members of the plane of interest should have z-coordinates
    // nearly 0, and thus these points will be easy to find
    // cout << "transforming all points to plane coordinates..." << endl;
    //Transform each point in the given point cloud according to the given transformation. 
    //pcl fnc: pass in ptrs to input cloud, holder for transformed cloud, and desired transform
    //note that if A contains a description of the frame on the plane, we want to xform with inverse(A)
    pcl::transformPointCloud(*input_cloud_ptr, *transformed_cloud_ptr, A_plane_wrt_camera.inverse());
    //now we'll use some functions from the pcl filter library; 
    pcl::PassThrough<pcl::PointXYZRGB> pass; //create a pass-through object
    pass.setInputCloud(transformed_cloud_ptr); //set the cloud we want to operate on--pass via a pointer
    pass.setFilterFieldName("z"); // we will "filter" based on points that lie within some range of z-value
    pass.setFilterLimits(-0.02, 0.02); //here is the range: z value near zero, -0.02<z<0.02
    pass.filter(indices); //  this will return the indices of the points in   transformed_cloud_ptr that pass our test
    // cout << "number of points passing the filter = " << indices.size() << endl;
    //This fnc populates the reference arg "indices", so the calling fnc gets the list of interesting points
}


int main(int argc, char** argv) {
    ros::init(argc, argv, "object_finder_as"); // name this node 

    ROS_INFO("instantiating the object finder action server: ");

    ObjectFinder object_finder_as; // create an instance of the class "ObjectFinder"
    
    ROS_INFO("going into spin");
    // from here, all the work is done in the action server, with the interesting stuff done within "executeCB()"
    // you will see 5 new topics under example_action: cancel, feedback, goal, result, status
    while (ros::ok()) {
        ros::Duration(0.5).sleep();
        ros::spinOnce(); //normally, can simply do: ros::spin();  
        // for debug, induce a halt if we ever get our client/server communications out of sync
    }

    return 0;
}

