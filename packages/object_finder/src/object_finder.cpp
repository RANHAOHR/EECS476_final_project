class ObjectFinder {
private:
    ros::NodeHandle nh_;
    PclUtils pclUtils_;
    //messages to send/receive cartesian goals / results:
    object_finder::object_finderGoal find_goal_;
    object_finder::object_finderResult find_result_; 
    object_finder::object_finderFeedback find_fdbk_;    
    geometry_msgs::Pose object_pose_;
    int object_code_;
    
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr pclKinect_clr_ptr(new pcl::PointCloud<pcl::PointXYZRGB>); //pointer for color version of pointcloud
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr plane_pts_ptr(new pcl::PointCloud<pcl::PointXYZRGB>); //pointer for pointcloud of planar points found
    pcl::PointCloud<pcl::PointXYZ>::Ptr selected_pts_cloud_ptr(new pcl::PointCloud<pcl::PointXYZ>); //ptr to selected pts from Rvis tool
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr downsampled_kinect_ptr(new pcl::PointCloud<pcl::PointXYZRGB>); //ptr to hold filtered Kinect image

    pcl::PointCloud<pcl::PointXYZRGB>::Ptr rough_table_cloud_ptr(new pcl::PointCloud<pcl::PointXYZRGB>); //ptr to table pts from Rvis tool    
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr can_pts_cloud_ptr(new pcl::PointCloud<pcl::PointXYZRGB>);

    pcl::PointCloud<pcl::PointXYZ>::Ptr table_xyz_ptr(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr final_table_cloud_ptr(new pcl::PointCloud<pcl::PointXYZRGB>); //ptr to table pts from Rvis tool 

    vector<int> indices;
    
    ros::Publisher gripper_publisher;
    
    actionlib::SimpleActionServer<object_finder::object_finderAction> object_finder_as_;
    ros::Publisher pubCloud;
    ros::Publisher pubPlane;
    ros::Publisher pubDnSamp;
    sensor_msgs::PointCloud2 tablePts; //create a ROS message
    ros::Publisher table;

    sensor_msgs::PointCloud2 canPts;
    ros::Publisher Can;
    sensor_msgs::PointCloud2 ros_cloud, table_planar_cloud, ros_planar_cloud, downsampled_cloud; //here are ROS-compatible messages    
    //action callback fnc
    void executeCB(const actionlib::SimpleActionServer<object_finder::object_finderAction>::GoalConstPtr& goal);   
    void find_indices_of_plane_from_patch(pcl::PointCloud<pcl::PointXYZRGB>::Ptr input_cloud_ptr,
        pcl::PointCloud<pcl::PointXYZ>::Ptr patch_cloud_ptr, vector<int> &indices);
public:
    ObjectFinder(ros::NodeHandle* nodehandle); //define the body of the constructor outside of class definition
    ~ObjectFinder(void) {
    }
    //define some member methods here
};
  
//name this server; 
ObjectFinder::ObjectFinder(ros::NodeHandle* nodehandle, PclUtils* pclUtils): nh_(*nodehandle), pclUtils_(*pclUtils),
   object_finder_as_(nh_, "objectFinderActionServer", boost::bind(&ObjectFinder::executeCB, this, _1),false)
// in the above initialization, we name the server "waffleIron" NO WE DOn"T it's obviously objectFinderActionServer
// clients will need to refer to this name to connect with this server
{
    ROS_INFO("in constructor of ObjectFinder");
    pubCloud = nh_.advertise<sensor_msgs::PointCloud2> ("/pcd", 1);
    pubPlane = nh_.advertise<sensor_msgs::PointCloud2> ("planar_pts", 1);
    pubDnSamp = nh_.advertise<sensor_msgs::PointCloud2> ("downsampled_pcd", 1);
    table = nh_.advertise<sensor_msgs::PointCloud2> ("/table_pts", 1);
    Can = nh_.advertise<sensor_msgs::PointCloud2> ("/coke_can_pts", 1); 

    object_finderer_as_.start(); //start the server running
}

void ObjectGrabber::executeCB(const actionlib::SimpleActionServer<object_finder::object_finderAction>::GoalConstPtr& goal) {
    cout << "------------------------------------------------" << endl;
    if (pclUtils_.got_kinect_cloud_) {
        pclUtils_.get_kinect_points(pclKinect_clr_ptr);
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
        Eigen::Vector3f can_bottom;

        if (can_exist) {
            ROS_WARN("coke can detected !");
            can_bottom = pclUtils_.find_can_bottom(final_table_cloud_ptr); //find bottom in table
            pcl::toROSMsg(*can_pts_cloud_ptr, canPts);
            Can.publish(canPts);
            ROS_INFO("The x y z of the can bottom is: (%f, %f, %f)", can_bottom[0], can_bottom[1], can_bottom[2]);
            object_pose_.pose.position.x = can_bottom[0];
            object_pose_.pose.position.y = can_bottom[1];
            object_pose_.pose.position.z = can_bottom[2];
            find_result_->object_pose = object_pose_;
            find_result_->found_object_code = object_finder::objectFinderResult::OBJECT_FOUND;
        }
        else {
            ROS_WARN("NO coke can!");
            Can.publish(tablePts);
            find_result_->found_object_code = object_finder::objectFinderResult::OBJECT_NOT_RECOGNIZED;
        }
        pclUtils_.got_kinect_cloud_ = false;
    } else {
        ROS_WARN("NO Kinect callback !");
    }
}

void ObjectFinder::find_indices_of_plane_from_patch(pcl::PointCloud<pcl::PointXYZRGB>::Ptr input_cloud_ptr,
        pcl::PointCloud<pcl::PointXYZ>::Ptr patch_cloud_ptr, vector<int> &indices) {
    float curvature;
    Eigen::Vector4f plane_parameters;
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr transformed_cloud_ptr(new pcl::PointCloud<pcl::PointXYZRGB>); //pointer for color version of pointcloud
    pcl::computePointNormal(*patch_cloud_ptr, plane_parameters, curvature); //pcl fnc to compute plane fit to point cloud
    cout << "PCL: plane params of patch: " << plane_parameters.transpose() << endl;
    //next, define a coordinate frame on the plane fitted to the patch.
    // choose the z-axis of this frame to be the plane normal--but enforce that the normal
    // must point towards the camera
    Eigen::Affine3f A_plane_wrt_camera;
    // here, use a utility function in pclUtils to construct a frame on the computed plane
    A_plane_wrt_camera = g_pcl_utils_ptr->make_affine_from_plane_params(plane_parameters);
    cout << "A_plane_wrt_camera rotation:" << endl;
    cout << A_plane_wrt_camera.linear() << endl;
    cout << "origin: " << A_plane_wrt_camera.translation().transpose() << endl;
    //next, transform all points in input_cloud into the plane frame.
    //the result of this is, points that are members of the plane of interest should have z-coordinates
    // nearly 0, and thus these points will be easy to find
    cout << "transforming all points to plane coordinates..." << endl;
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
    cout << "number of points passing the filter = " << indices.size() << endl;
    //This fnc populates the reference arg "indices", so the calling fnc gets the list of interesting points
}
