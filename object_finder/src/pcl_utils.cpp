/*
 * pcl_utils: a  ROS library to illustrate use of PCL, including some handy utility functions
 *
 */

#include <pcl_utils/pcl_utils.h>
#include <pcl-1.7/pcl/PCLHeader.h>
/* uses initializer list for member vars */

PclUtils::PclUtils( ros::NodeHandle* nodehandle ) : nh_( *nodehandle ), pclKinect_ptr_( new PointCloud<pcl::PointXYZ>),
    pclKinect_clr_ptr_( new PointCloud<pcl::PointXYZRGB>),
    pclTransformed_ptr_( new PointCloud<pcl::PointXYZ>), pclSelectedPoints_ptr_( new PointCloud<pcl::PointXYZ>),
    pclSelectedPtsClr_ptr_( new PointCloud<pcl::PointXYZRGB>),
    pclTransformedSelectedPoints_ptr_( new PointCloud<pcl::PointXYZ>), pclGenPurposeCloud_ptr_( new PointCloud<pcl::PointXYZ>)
{
    initializeSubscribers();
    initializePublishers();
    got_kinect_cloud_   = false;
    got_selected_points_    = false;

    table_normal[0] = 0.0; /* /set in seek_rough_table_merry */
    table_normal[1] = 0.0;
    table_normal[2] = 0.0;

    table_origin[0] = 0.0;
    table_origin[1] = 0.0;
    table_origin[2] = 0.0;

    can_height = 0.125;

    is_final_cloud = false;
}


/* fnc to read a pcd file and put contents in pclKinect_ptr_: color version */
int PclUtils::read_clr_pcd_file( string fname )
{
    if ( pcl::io::loadPCDFile<pcl::PointXYZRGB> ( fname, *pclKinect_clr_ptr_ ) == -1 ) /* * load the file */
    {
        ROS_ERROR( "Couldn't read file \n" );
        return(-1);
    }
    std::cout   << "Loaded "
            << pclKinect_clr_ptr_->width * pclKinect_clr_ptr_->height
            << " data points from file " << fname << std::endl;
    return(0);
}


/* non-color version */
int PclUtils::read_pcd_file( string fname )
{
    if ( pcl::io::loadPCDFile<pcl::PointXYZ> ( fname, *pclKinect_ptr_ ) == -1 ) /* * load the file */
    {
        ROS_ERROR( "Couldn't read file \n" );
        return(-1);
    }
    std::cout   << "Loaded "
            << pclKinect_ptr_->width * pclKinect_ptr_->height
            << " data points from file " << fname << std::endl;
    return(0);
}


/*
 * given plane parameters of normal vec and distance to plane, construct and return an Eigen Affine object
 * suitable for transforming points to a frame defined on the plane
 */
Eigen::Affine3f PclUtils::make_affine_from_plane_params( Eigen::Vector3f plane_normal, double plane_dist )
{
    Eigen::Vector3f xvec, yvec, zvec;
    Eigen::Matrix3f R_transform;
    Eigen::Affine3f A_transform;
    Eigen::Vector3f plane_origin;
    /* define a frame on the plane, with zvec parallel to the plane normal */
    zvec = plane_normal;
    if ( zvec( 2 ) > 0 )
        zvec *= -1.0;                                           /* insist that plane normal points towards camera */
    /* this assumes that reference frame of points corresponds to camera w/ z axis pointing out from camera */
    xvec << 1, 0, 0;                                                /* this is arbitrary, but should be valid for images taken w/ zvec= optical axis */
    xvec            = xvec - zvec * (zvec.dot( xvec ) );    /* force definition of xvec to be orthogonal to plane normal */
    xvec            /= xvec.norm();                         /* want this to be unit length as well */
    yvec            = zvec.cross( xvec );
    R_transform.col( 0 )    = xvec;
    R_transform.col( 1 )    = yvec;
    R_transform.col( 2 )    = zvec;
    /*
     * cout<<"R_transform = :"<<endl;
     * cout<<R_transform<<endl;
     */
    if ( plane_dist > 0 )
        plane_dist *= -1.0;                             /* all planes are a negative distance from the camera, to be consistent w/ normal */
    A_transform.linear()        = R_transform;          /* directions of the x,y,z axes of the plane's frame, expressed w/rt camera frame */
    plane_origin            = zvec * plane_dist;    /* define the plane-frame origin here */
    A_transform.translation()   = plane_origin;
    return(A_transform);
}


/* same as above, really, but accept input is a 4-vector */
Eigen::Affine3f PclUtils::make_affine_from_plane_params( Eigen::Vector4f plane_parameters )
{
    Eigen::Vector3f plane_normal;
    double      plane_dist;
    plane_normal( 0 )   = plane_parameters( 0 );
    plane_normal( 1 )   = plane_parameters( 1 );
    plane_normal( 2 )   = plane_parameters( 2 );
    plane_dist      = plane_parameters( 3 );
    return(make_affine_from_plane_params( plane_normal, plane_dist ) );
}


void PclUtils::fit_points_to_plane( Eigen::MatrixXf points_mat, Eigen::Vector3f &plane_normal, double &plane_dist )
{
    /* ROS_INFO("starting identification of plane from data: "); */
    int npts = points_mat.cols();                   /* number of points = number of columns in matrix; check the size */
    ROS_INFO( "npts = %d", npts );
    /*
     * first compute the centroid of the data:
     * Eigen::Vector3f centroid; // make this member var, centroid_
     */
    centroid_ = Eigen::MatrixXf::Zero( 3, 1 );      /* see http://eigen.tuxfamily.org/dox/AsciiQuickReference.txt */

    /* centroid = compute_centroid(points_mat); */
    for ( int ipt = 0; ipt < npts; ipt++ )
    {
        centroid_ += points_mat.col( ipt );     /* add all the column vectors together */
    }
    centroid_ /= npts;                              /* divide by the number of points to get the centroid */
    cout << "centroid: " << centroid_.transpose() << endl;


    /* subtract this centroid from all points in points_mat: */
    Eigen::MatrixXf points_offset_mat = points_mat;
    for ( int ipt = 0; ipt < npts; ipt++ )
    {
        points_offset_mat.col( ipt ) = points_offset_mat.col( ipt ) - centroid_;
    }
    /* compute the covariance matrix w/rt x,y,z: */
    Eigen::Matrix3f CoVar;
    CoVar = points_offset_mat * (points_offset_mat.transpose() ); /* 3xN matrix times Nx3 matrix is 3x3 */
    /*
     * cout<<"covariance: "<<endl;
     * cout<<CoVar<<endl;
     */

    /*
     * here is a more complex object: a solver for eigenvalues/eigenvectors;
     * we will initialize it with our covariance matrix, which will induce computing eval/evec pairs
     */
    Eigen::EigenSolver<Eigen::Matrix3f> es3f( CoVar );

    Eigen::VectorXf evals; /* we'll extract the eigenvalues to here */
    /*
     * cout<<"size of evals: "<<es3d.eigenvalues().size()<<endl;
     * cout<<"rows,cols = "<<es3d.eigenvalues().rows()<<", "<<es3d.eigenvalues().cols()<<endl;
     * cout << "The eigenvalues of CoVar are:" << endl << es3d.eigenvalues().transpose() << endl;
     * cout<<"(these should be real numbers, and one of them should be zero)"<<endl;
     * cout << "The matrix of eigenvectors, V, is:" << endl;
     * cout<< es3d.eigenvectors() << endl << endl;
     * cout<< "(these should be real-valued vectors)"<<endl;
     * in general, the eigenvalues/eigenvectors can be complex numbers
     * however, since our matrix is self-adjoint (symmetric, positive semi-definite), we expect
     * real-valued evals/evecs;  we'll need to strip off the real parts of the solution
     */

    evals = es3f.eigenvalues().real();      /* grab just the real parts */
    /* cout<<"real parts of evals: "<<evals.transpose()<<endl; */

    /*
     * our solution should correspond to an e-val of zero, which will be the minimum eval
     *  (all other evals for the covariance matrix will be >0)
     * however, the solution does not order the evals, so we'll have to find the one of interest ourselves
     */

    double  min_lambda  = evals[0];     /* initialize the hunt for min eval */
    double  max_lambda  = evals[0];     /* and for max eval */
    /*
     * Eigen::Vector3cf complex_vec; // here is a 3x1 vector of double-precision, complex numbers
     * Eigen::Vector3f evec0, evec1, evec2; //, major_axis;
     * evec0 = es3f.eigenvectors().col(0).real();
     * evec1 = es3f.eigenvectors().col(1).real();
     * evec2 = es3f.eigenvectors().col(2).real();
     */


    /*
     * ((pt-centroid)*evec)*2 = evec'*points_offset_mat'*points_offset_mat*evec =
     * = evec'*CoVar*evec = evec'*lambda*evec = lambda
     * min lambda is ideally zero for evec= plane_normal, since points_offset_mat*plane_normal~= 0
     * max lambda is associated with direction of major axis
     */

    /* sort the evals: */

    /*
     * complex_vec = es3f.eigenvectors().col(0); // here's the first e-vec, corresponding to first e-val
     * cout<<"complex_vec: "<<endl;
     * cout<<complex_vec<<endl;
     */
    plane_normal    = es3f.eigenvectors().col( 0 ).real();  /* complex_vec.real(); //strip off the real part */
    major_axis_ = es3f.eigenvectors().col( 0 ).real();  /* starting assumptions */

    /*
     * cout<<"real part: "<<est_plane_normal.transpose()<<endl;
     * est_plane_normal = es3d.eigenvectors().col(0).real(); // evecs in columns
     */

    double  lambda_test;
    int i_normal    = 0;
    int i_major_axis    = 0;
    /* loop through "all" ("both", in this 3-D case) the rest of the solns, seeking min e-val */
    for ( int ivec = 1; ivec < 3; ivec++ )
    {
        lambda_test = evals[ivec];
        if ( lambda_test < min_lambda )
        {
            min_lambda  = lambda_test;
            i_normal    = ivec; /* this index is closer to index of min eval */
            plane_normal    = es3f.eigenvectors().col( i_normal ).real();
        }
        if ( lambda_test > max_lambda )
        {
            max_lambda  = lambda_test;
            i_major_axis    = ivec; /* this index is closer to index of min eval */
            major_axis_ = es3f.eigenvectors().col( i_major_axis ).real();
        }
    }
    /*
     * at this point, we have the minimum eval in "min_lambda", and the plane normal
     * (corresponding evec) in "est_plane_normal"/
     * these correspond to the ith entry of i_normal
     * cout<<"min eval is "<<min_lambda<<", corresponding to component "<<i_normal<<endl;
     * cout<<"corresponding evec (est plane normal): "<<plane_normal.transpose()<<endl;
     * cout<<"max eval is "<<max_lambda<<", corresponding to component "<<i_major_axis<<endl;
     * cout<<"corresponding evec (est major axis): "<<major_axis_.transpose()<<endl;
     */

    /*
     * what is the correct sign of the normal?  If the data is with respect to the camera frame,
     * then the camera optical axis is z axis, and thus any points reflected must be from a surface
     * with negative z component of surface normal
     */
    if ( plane_normal( 2 ) > 0 )
        plane_normal = -plane_normal;            /* negate, if necessary */

    /* cout<<"correct answer is: "<<normal_vec.transpose()<<endl; */
    plane_dist = plane_normal.dot( centroid_ );
    /*
     * cout<<"est plane distance from origin = "<<est_dist<<endl;
     * cout<<"correct answer is: "<<dist<<endl;
     * cout<<endl<<endl;
     */
}


/*
 * get pts from cloud, pack the points into an Eigen::MatrixXf, then use above
 * fit_points_to_plane fnc
 */

void PclUtils::fit_points_to_plane( pcl::PointCloud<pcl::PointXYZ>::Ptr input_cloud_ptr, Eigen::Vector3f &plane_normal, double &plane_dist )
{
    Eigen::MatrixXf points_mat;
    Eigen::Vector3f cloud_pt;
    /* populate points_mat from cloud data; */

    int npts = input_cloud_ptr->points.size();
    points_mat.resize( 3, npts );

    /* somewhat odd notation: getVector3fMap() reading OR WRITING points from/to a pointcloud, with conversions to/from Eigen */
    for ( int i = 0; i < npts; ++i )
    {
        cloud_pt        = input_cloud_ptr->points[i].getVector3fMap();
        points_mat.col( i ) = cloud_pt;
    }
    fit_points_to_plane( points_mat, plane_normal, plane_dist );
}


void PclUtils::fit_points_to_plane( pcl::PointCloud<pcl::PointXYZRGB>::Ptr input_cloud_ptr, Eigen::Vector3f &plane_normal, double &plane_dist )
{
    pcl::PointCloud<pcl::PointXYZ>::Ptr temp_cloud( new pcl::PointCloud<pcl::PointXYZ>); /* pointer for color version of pointcloud */
    from_RGB_to_XYZ( input_cloud_ptr, temp_cloud );
    fit_points_to_plane( temp_cloud, plane_normal, plane_dist );
}


/* compute and return the centroid of a pointCloud */
Eigen::Vector3f PclUtils::compute_centroid( pcl::PointCloud<pcl::PointXYZ>::Ptr input_cloud_ptr )
{
    Eigen::Vector3f centroid;
    Eigen::Vector3f cloud_pt;
    int     npts = input_cloud_ptr->points.size();
    centroid << 0, 0, 0;
    /* add all the points together: */

    for ( int ipt = 0; ipt < npts; ipt++ )
    {
        cloud_pt    = input_cloud_ptr->points[ipt].getVector3fMap();
        centroid    += cloud_pt;    /* add all the column vectors together */
    }
    centroid /= npts;                       /* divide by the number of points to get the centroid */
    return(centroid);
}


/* same thing, but arg is reference cloud instead of pointer: */
Eigen::Vector3f PclUtils::compute_centroid( pcl::PointCloud<pcl::PointXYZ> &input_cloud )
{
    Eigen::Vector3f centroid;
    Eigen::Vector3f cloud_pt;
    int     npts = input_cloud.points.size();
    centroid << 0, 0, 0;
    /* add all the points together: */

    for ( int ipt = 0; ipt < npts; ipt++ )
    {
        cloud_pt    = input_cloud.points[ipt].getVector3fMap();
        centroid    += cloud_pt;    /* add all the column vectors together */
    }
    centroid /= npts;                       /* divide by the number of points to get the centroid */
    return(centroid);
}


/* this fnc operates on transformed selected points */

void PclUtils::fit_xformed_selected_pts_to_plane( Eigen::Vector3f &plane_normal, double &plane_dist )
{
    fit_points_to_plane( pclTransformedSelectedPoints_ptr_, plane_normal, plane_dist );
    /* Eigen::Vector3f centroid; */
    cwru_msgs::PatchParams patch_params_msg;
    /*
     * compute the centroid; this is redundant w/ computation inside fit_points...oh well.
     * now the centroid computed by plane fit is stored in centroid_ member var
     * centroid = compute_centroid(pclTransformedSelectedPoints_ptr_);
     */

    patch_params_msg.offset = plane_dist;
    patch_params_msg.centroid.resize( 3 );
    patch_params_msg.normal_vec.resize( 3 );
    for ( int i = 0; i < 3; i++ )
    {
        patch_params_msg.normal_vec[i]  = plane_normal[i];
        patch_params_msg.centroid[i]    = centroid_[i];
    }
    patch_params_msg.frame_id = "torso";
    patch_publisher_.publish( patch_params_msg );
}


Eigen::Affine3f PclUtils::transformTFToEigen( const tf::Transform &t )
{
    Eigen::Affine3f e;
    /* treat the Eigen::Affine as a 4x4 matrix: */
    for ( int i = 0; i < 3; i++ )
    {
        e.matrix() ( i, 3 ) = t.getOrigin()[i];                 /* copy the origin from tf to Eigen */
        for ( int j = 0; j < 3; j++ )
        {
            e.matrix() ( i, j ) = t.getBasis()[i][j];       /* and copy 3x3 rotation matrix */
        }
    }
    /* Fill in identity in last row */
    for ( int col = 0; col < 3; col++ )
        e.matrix() ( 3, col ) = 0;
    e.matrix() ( 3, 3 ) = 1;
    return(e);
}


/**here is a function that transforms a cloud of points into an alternative frame;
 * it assumes use of pclKinect_ptr_ from kinect sensor as input, to pclTransformed_ptr_ , the cloud in output frame
 *
 * @param A [in] supply an Eigen::Affine3f, such that output_points = A*input_points
 */
void PclUtils::transform_kinect_cloud( Eigen::Affine3f A )
{
    transform_cloud( A, pclKinect_ptr_, pclTransformed_ptr_ );


    /*
     * pclTransformed_ptr_->header = pclKinect_ptr_->header;
     * pclTransformed_ptr_->is_dense = pclKinect_ptr_->is_dense;
     * pclTransformed_ptr_->width = pclKinect_ptr_->width;
     * pclTransformed_ptr_->height = pclKinect_ptr_->height;
     * int npts = pclKinect_ptr_->points.size();
     * cout << "transforming npts = " << npts << endl;
     * pclTransformed_ptr_->points.resize(npts);
     *
     * //somewhat odd notation: getVector3fMap() reading OR WRITING points from/to a pointcloud, with conversions to/from Eigen
     * for (int i = 0; i < npts; ++i) {
     *  pclTransformed_ptr_->points[i].getVector3fMap() = A * pclKinect_ptr_->points[i].getVector3fMap();
     * }
     * */
}


void PclUtils::transform_selected_points_cloud( Eigen::Affine3f A )
{
    transform_cloud( A, pclSelectedPoints_ptr_, pclTransformedSelectedPoints_ptr_ );
}


/*    void get_transformed_selected_points(pcl::PointCloud<pcl::PointXYZ> & outputCloud ); */

void PclUtils::get_transformed_selected_points( pcl::PointCloud<pcl::PointXYZ> & outputCloud )
{
    int npts = pclTransformedSelectedPoints_ptr_->points.size(); /* how many points to extract? */
    outputCloud.header  = pclTransformedSelectedPoints_ptr_->header;
    outputCloud.is_dense    = pclTransformedSelectedPoints_ptr_->is_dense;
    outputCloud.width   = npts;
    outputCloud.height  = 1;

    cout << "copying cloud w/ npts =" << npts << endl;
    outputCloud.points.resize( npts );
    for ( int i = 0; i < npts; ++i )
    {
        outputCloud.points[i].getVector3fMap() = pclTransformedSelectedPoints_ptr_->points[i].getVector3fMap();
    }
}


void PclUtils::get_copy_selected_points( pcl::PointCloud<pcl::PointXYZ>::Ptr &outputCloud )
{
    int npts = pclSelectedPoints_ptr_->points.size();               /* how many points to extract? */
    outputCloud->header     = pclSelectedPoints_ptr_->header;
    outputCloud->header.frame_id    = "camera_depth_optical_frame"; /* work-around for bug in publish selected pts tool */
    outputCloud->is_dense       = pclSelectedPoints_ptr_->is_dense;
    outputCloud->width      = npts;
    outputCloud->height     = 1;

    cout << "copying cloud w/ npts =" << npts << endl;
    outputCloud->points.resize( npts );
    for ( int i = 0; i < npts; ++i )
    {
        outputCloud->points[i].getVector3fMap() = pclSelectedPoints_ptr_->points[i].getVector3fMap();
    }
}


void PclUtils::get_kinect_points( pcl::PointCloud<pcl::PointXYZ> & outputCloud )
{
    int npts = pclKinect_ptr_->points.size(); /* how many points to extract? */
    outputCloud.header  = pclKinect_ptr_->header;
    outputCloud.is_dense    = pclKinect_ptr_->is_dense;
    outputCloud.width   = npts;
    outputCloud.height  = 1;

    cout << "copying cloud w/ npts =" << npts << endl;
    outputCloud.points.resize( npts );
    for ( int i = 0; i < npts; ++i )
    {
        outputCloud.points[i].getVector3fMap() = pclKinect_ptr_->points[i].getVector3fMap();
    }
}


void PclUtils::get_kinect_points( pcl::PointCloud<pcl::PointXYZRGB> & outputCloud )
{
    int npts = pclKinect_clr_ptr_->points.size(); /* how many points to extract? */
    outputCloud.header  = pclKinect_clr_ptr_->header;
    outputCloud.is_dense    = pclKinect_clr_ptr_->is_dense;
    outputCloud.width   = npts;
    outputCloud.height  = 1;

    cout << "get_kinect_points xyzrgb, copying cloud w/ npts =" << npts << endl;
    outputCloud.points.resize( npts );
    for ( int i = 0; i < npts; ++i )
    {
        outputCloud.points[i] = pclKinect_clr_ptr_->points[i];
    }
}


/* need this version for viewer */
void PclUtils::get_kinect_points( pcl::PointCloud<pcl::PointXYZRGB>::Ptr &outputCloudPtr )
{
    int npts = pclKinect_clr_ptr_->points.size(); /* how many points to extract? */
    /*
     * cout<<"need to copy "<<npts<<" points"<<endl;
     * cout<<"enter 1: ";
     * int ans;
     * cin>>ans;
     */
    outputCloudPtr->header      = pclKinect_clr_ptr_->header;
    outputCloudPtr->is_dense    = pclKinect_clr_ptr_->is_dense;
    cout << "setting width: " << endl;
    outputCloudPtr->width = npts;
    cout << "setting height" << endl;
    outputCloudPtr->height = 1;

    /*
     * cout << "ready to resize output cloud to npts = " << npts << endl;
     *    cout<<"enter 1: ";
     * cin>>ans;
     */
    outputCloudPtr->points.resize( npts );
    for ( int i = 0; i < npts; ++i )
    {
        outputCloudPtr->points[i].getVector3fMap() = pclKinect_clr_ptr_->points[i].getVector3fMap();

        outputCloudPtr->points[i].r = pclKinect_clr_ptr_->points[i].r;
        outputCloudPtr->points[i].g = pclKinect_clr_ptr_->points[i].g;
        outputCloudPtr->points[i].b = pclKinect_clr_ptr_->points[i].b;
    }
}


void PclUtils::get_kinect_points( pcl::PointCloud<pcl::PointXYZ>::Ptr &outputCloudPtr )
{
    int npts = pclKinect_ptr_->points.size(); /* how many points to extract? */
    outputCloudPtr->header      = pclKinect_ptr_->header;
    outputCloudPtr->is_dense    = pclKinect_ptr_->is_dense;
    outputCloudPtr->width       = npts;
    outputCloudPtr->height      = 1;

    cout << "copying cloud w/ npts =" << npts << endl;
    outputCloudPtr->points.resize( npts );
    for ( int i = 0; i < npts; ++i )
    {
        outputCloudPtr->points[i].getVector3fMap() = pclKinect_ptr_->points[i].getVector3fMap();
    }
}


/* same as above, but for general-purpose cloud */
void PclUtils::get_gen_purpose_cloud( pcl::PointCloud<pcl::PointXYZ> & outputCloud )
{
    int npts = pclGenPurposeCloud_ptr_->points.size(); /* how many points to extract? */
    outputCloud.header  = pclGenPurposeCloud_ptr_->header;
    outputCloud.is_dense    = pclGenPurposeCloud_ptr_->is_dense;
    outputCloud.width   = npts;
    outputCloud.height  = 1;

    cout << "copying cloud w/ npts =" << npts << endl;
    outputCloud.points.resize( npts );
    for ( int i = 0; i < npts; ++i )
    {
        outputCloud.points[i].getVector3fMap() = pclGenPurposeCloud_ptr_->points[i].getVector3fMap();
    }
}


/* makes a copy of the selected points from rviz tool; xyz only, no color (this version) */
void PclUtils::get_selected_points( pcl::PointCloud<pcl::PointXYZ>::Ptr &outputCloudPtr )
{
    int npts = pclSelectedPoints_ptr_->points.size(); /* how many points to extract? */
    outputCloudPtr->header      = pclSelectedPoints_ptr_->header;
    outputCloudPtr->is_dense    = pclSelectedPoints_ptr_->is_dense;
    outputCloudPtr->width       = npts;
    outputCloudPtr->height      = 1;

    cout << "copying cloud w/ npts =" << npts << endl;
    outputCloudPtr->points.resize( npts );
    for ( int i = 0; i < npts; ++i )
    {
        outputCloudPtr->points[i].getVector3fMap() = pclSelectedPoints_ptr_->points[i].getVector3fMap();
    }
}


/* this version for argument of reference variable to cloud, not pointer to cloud */
void PclUtils::get_selected_points( pcl::PointCloud<pcl::PointXYZ> &outputCloud )
{
    int npts = pclSelectedPoints_ptr_->points.size(); /* how many points to extract? */
    outputCloud.header  = pclSelectedPoints_ptr_->header;
    outputCloud.is_dense    = pclSelectedPoints_ptr_->is_dense;
    outputCloud.width   = npts;
    outputCloud.height  = 1;

    cout << "copying cloud w/ npts =" << npts << endl;
    outputCloud.points.resize( npts );
    for ( int i = 0; i < npts; ++i )
    {
        outputCloud.points[i].getVector3fMap() = pclSelectedPoints_ptr_->points[i].getVector3fMap();
    }
}


/*
 * void PclUtils::get_indices(vector<int> &indices,) {
 *    indices = indicies_;
 * }
 */

/*
 * here is an example utility function.  It operates on clouds that are member variables, and it puts its result
 * in the general-purpose cloud variable, which can be acquired by main(), if desired, using get_gen_purpose_cloud()
 */

/*
 * The operation illustrated here is not all that useful.  It uses transformed, selected points,
 * elevates the data by 5cm, and copies the result to the general-purpose cloud variable
 */
void PclUtils::example_pcl_operation()
{
    int npts = pclTransformedSelectedPoints_ptr_->points.size();                    /* number of points */
    copy_cloud( pclTransformedSelectedPoints_ptr_, pclGenPurposeCloud_ptr_ );       /* now have a copy of the selected points in gen-purpose object */
    Eigen::Vector3f offset;
    offset << 0, 0, 0.05;
    for ( int i = 0; i < npts; ++i )
    {
        pclGenPurposeCloud_ptr_->points[i].getVector3fMap() = pclGenPurposeCloud_ptr_->points[i].getVector3fMap() + offset;
    }
}


/*
 * This fnc populates and output cloud of type XYZRGB extracted from the full Kinect cloud (in Kinect frame)
 * provide a vector of indices and a holder for the output cloud, which gets populated
 */
void PclUtils::copy_indexed_pts_to_output_cloud( vector<int> &indices, PointCloud<pcl::PointXYZRGB> &outputCloud )
{
    int npts = indices.size(); /* how many points to extract? */
    outputCloud.header  = pclKinect_clr_ptr_->header;
    outputCloud.is_dense    = pclKinect_clr_ptr_->is_dense;
    outputCloud.width   = npts;
    outputCloud.height  = 1;
    int i_index;

    cout << "copying cloud w/ npts =" << npts << endl;
    outputCloud.points.resize( npts );
    for ( int i = 0; i < npts; ++i )
    {
        i_index                 = indices[i];
        outputCloud.points[i].getVector3fMap()  = pclKinect_clr_ptr_->points[i_index].getVector3fMap();
        outputCloud.points[i].r         = pclKinect_clr_ptr_->points[i_index].r;
        outputCloud.points[i].g         = pclKinect_clr_ptr_->points[i_index].g;
        outputCloud.points[i].b         = pclKinect_clr_ptr_->points[i_index].b;


        /*
         *  std::cout <<i_index
         *    << "    " << (int) pclKinect_clr_ptr_->points[i_index].r
         *    << " "    << (int) pclKinect_clr_ptr_->points[i_index].g
         *    << " "    << (int) pclKinect_clr_ptr_->points[i_index].b << std::endl;
         */
    }
}


/*
 * comb through kinect colors and compute average color
 * disregard color=0,0,0
 */
Eigen::Vector3d PclUtils::find_avg_color()
{
    Eigen::Vector3d avg_color;
    Eigen::Vector3d pt_color;
    Eigen::Vector3d ref_color;
    indices_.clear();
    ref_color << 147, 147, 147;
    int npts        = pclKinect_clr_ptr_->points.size();
    int npts_colored    = 0;
    for ( int i = 0; i < npts; i++ )
    {
        pt_color( 0 )   = (double) pclKinect_clr_ptr_->points[i].r;
        pt_color( 1 )   = (double) pclKinect_clr_ptr_->points[i].g;
        pt_color( 2 )   = (double) pclKinect_clr_ptr_->points[i].b;

        if ( (pt_color - ref_color).norm() > 1 )
        {
            avg_color += pt_color;
            npts_colored++;
            indices_.push_back( i ); /* save this points as "interesting" color */
        }
    }
    ROS_INFO( "found %d points with interesting color", npts_colored );
    avg_color /= npts_colored;
    ROS_INFO( "avg interesting color = %f, %f, %f", avg_color( 0 ), avg_color( 1 ), avg_color( 2 ) );
    return(avg_color);
}


Eigen::Vector3d PclUtils::find_avg_color_selected_pts( vector<int> &indices )
{
    Eigen::Vector3d avg_color;
    Eigen::Vector3d pt_color;
    /* Eigen::Vector3d ref_color; */

    int npts = indices.size();
    int index;

    for ( int i = 0; i < npts; i++ )
    {
        index       = indices[i];
        pt_color( 0 )   = (double) pclKinect_clr_ptr_->points[index].r;
        pt_color( 1 )   = (double) pclKinect_clr_ptr_->points[index].g;
        pt_color( 2 )   = (double) pclKinect_clr_ptr_->points[index].b;
        avg_color   += pt_color;
    }
    avg_color /= npts;
    ROS_INFO( "avg color = %f, %f, %f", avg_color( 0 ), avg_color( 1 ), avg_color( 2 ) );
    return(avg_color);
}


void PclUtils::find_indices_color_match( vector<int> &input_indices,
                     Eigen::Vector3d normalized_avg_color,
                     double color_match_thresh, vector<int> &output_indices )
{
    Eigen::Vector3d pt_color;

    int npts = input_indices.size();
    output_indices.clear();
    int index;
    int npts_matching = 0;

    for ( int i = 0; i < npts; i++ )
    {
        index       = input_indices[i];
        pt_color( 0 )   = (double) pclKinect_clr_ptr_->points[index].r;
        pt_color( 1 )   = (double) pclKinect_clr_ptr_->points[index].g;
        pt_color( 2 )   = (double) pclKinect_clr_ptr_->points[index].b;
        pt_color    = pt_color / pt_color.norm();   /* compute normalized color */
        if ( (normalized_avg_color - pt_color).norm() < color_match_thresh )
        {
            output_indices.push_back( index );      /* color match, so save this point index */
            npts_matching++;
        }
    }
    ROS_INFO( "found %d color-match points from indexed set", npts_matching );
}


/* special case of above for transformed Kinect pointcloud: */
void PclUtils::filter_cloud_z( double z_nom, double z_eps,
                   double radius, Eigen::Vector3f centroid, vector<int> &indices )
{
    filter_cloud_z( pclTransformed_ptr_, z_nom, z_eps, radius, centroid, indices );
}


/* operate on transformed Kinect pointcloud: */
void PclUtils::find_coplanar_pts_z_height( double plane_height, double z_eps, vector<int> &indices )
{
    filter_cloud_z( pclTransformed_ptr_, plane_height, z_eps, indices );
}


void PclUtils::filter_cloud_z( PointCloud<pcl::PointXYZ>::Ptr inputCloud, double z_nom, double z_eps, vector<int> &indices )
{
    int     npts = inputCloud->points.size();
    Eigen::Vector3f pt;
    indices.clear();
    double  dz;
    int ans;
    for ( int i = 0; i < npts; ++i )
    {
        pt = inputCloud->points[i].getVector3fMap();
        /* cout<<"pt: "<<pt.transpose()<<endl; */
        dz = pt[2] - z_nom;
        if ( fabs( dz ) < z_eps )
        {
            indices.push_back( i );
            /*
             * cout<<"dz = "<<dz<<"; saving this point...enter 1 to continue: ";
             * cin>>ans;
             */
        }
    }
    int n_extracted = indices.size();
    cout << " number of points in range = " << n_extracted << endl;
}


void PclUtils::filter_cloud_z( PointCloud<pcl::PointXYZRGB>::Ptr inputCloud, double z_nom, double z_eps, vector<int> &indices )
{
    int     npts = inputCloud->points.size();
    Eigen::Vector3f pt;
    indices.clear();
    double  dz;
    int ans;
    for ( int i = 0; i < npts; ++i )
    {
        pt = inputCloud->points[i].getVector3fMap();
        /* cout<<"pt: "<<pt.transpose()<<endl; */
        dz = pt[2] - z_nom;
        if ( fabs( dz ) < z_eps )
        {
            indices.push_back( i );
            /*
             * cout<<"dz = "<<dz<<"; saving this point...enter 1 to continue: ";
             * cin>>ans;
             */
        }
    }
    int n_extracted = indices.size();
    cout << " number of points in range = " << n_extracted << endl;
}


/* find points that are both (approx) coplanar at height z_nom AND within "radius" of "centroid" */
void PclUtils::filter_cloud_z( PointCloud<pcl::PointXYZ>::Ptr inputCloud, double z_nom, double z_eps,
                   double radius, Eigen::Vector3f centroid, vector<int> &indices )
{
    int     npts = inputCloud->points.size();
    Eigen::Vector3f pt;
    indices.clear();
    double  dz;
    int ans;
    for ( int i = 0; i < npts; ++i )
    {
        pt = inputCloud->points[i].getVector3fMap();
        /* cout<<"pt: "<<pt.transpose()<<endl; */
        dz = pt[2] - z_nom;
        if ( fabs( dz ) < z_eps )
        {
            /* passed z-test; do radius test: */
            if ( (pt - centroid).norm() < radius )
            {
                indices.push_back( i );
            }
            /*
             * cout<<"dz = "<<dz<<"; saving this point...enter 1 to continue: ";
             * cin>>ans;
             */
        }
    }
    int n_extracted = indices.size();
    cout << " number of points in range = " << n_extracted << endl;
}


void PclUtils::analyze_selected_points_color()
{
    int npts = pclTransformedSelectedPoints_ptr_->points.size(); /* number of points */
    /*
     * copy_cloud(pclTransformedSelectedPoints_ptr_,pclGenPurposeCloud_ptr_); //now have a copy of the selected points in gen-purpose object
     * Eigen::Vector3f offset;
     * offset<<0,0,0.05;
     */
    int npts_clr = pclSelectedPtsClr_ptr_->points.size();
    cout << "color pts size = " << npts_clr << endl;
    pcl::PointXYZRGB p;
    /* unpack rgb into r/g/b */
    uint32_t    rgb = *reinterpret_cast<int*>(&p.rgb);
    uint8_t     r, g, b;
    int     r_int;

    for ( int i = 0; i < npts; ++i )
    {
        p   = pclSelectedPtsClr_ptr_->points[i];
        r   = (rgb >> 16) & 0x0000ff;
        r_int   = (int) r;
        /*
         * g = (rgb >> 8)  & 0x0000ff;
         * b = (rgb)       & 0x0000ff;
         */
        cout << "r_int: " << r_int << endl;
        cout << "r1: " << r << endl;
        r = pclSelectedPtsClr_ptr_->points[i].r;
        cout << "r2 = " << r << endl;

        /*
         * cout<<" ipt, r,g,b = "<<i<<","<<pclSelectedPtsClr_ptr_->points[i].r<<", "<<
         *        pclSelectedPtsClr_ptr_->points[i].g<<", "<<pclSelectedPtsClr_ptr_->points[i].b<<endl;
         * pclGenPurposeCloud_ptr_->points[i].getVector3fMap() = pclGenPurposeCloud_ptr_->points[i].getVector3fMap()+offset;
         */
    }
    cout << "done combing through selected pts" << endl;
    got_kinect_cloud_ = false; /* get a new snapshot */
}


/*
 * generic function to copy an input cloud to an output cloud
 * provide pointers to the two clouds
 * output cloud will get resized
 */
void PclUtils::copy_cloud( PointCloud<pcl::PointXYZ>::Ptr inputCloud, PointCloud<pcl::PointXYZ>::Ptr outputCloud )
{
    int npts = inputCloud->points.size(); /* how many points to extract? */
    outputCloud->header = inputCloud->header;
    outputCloud->is_dense   = inputCloud->is_dense;
    outputCloud->width  = npts;
    outputCloud->height = 1;

    cout << "copying cloud w/ npts =" << npts << endl;
    outputCloud->points.resize( npts );
    for ( int i = 0; i < npts; ++i )
    {
        outputCloud->points[i].getVector3fMap() = inputCloud->points[i].getVector3fMap();
    }
}


/* given indices of interest, chosen points from input colored cloud to output colored cloud */
void PclUtils::copy_cloud_xyzrgb_indices( PointCloud<pcl::PointXYZRGB>::Ptr inputCloud, vector<int> &indices, PointCloud<pcl::PointXYZRGB>::Ptr outputCloud )
{
    int npts = indices.size(); /* how many points to extract? */
    outputCloud->header = inputCloud->header;
    outputCloud->is_dense   = inputCloud->is_dense;
    outputCloud->width  = npts;
    outputCloud->height = 1;

    cout << "copying cloud w/ npts =" << npts << endl;
    outputCloud->points.resize( npts );
    for ( int i = 0; i < npts; ++i )
    {
        /* outputCloud->points[i].getVector3fMap() = inputCloud->points[indices[i]].getVector3fMap(); */
        outputCloud->points[i] = inputCloud->points[indices[i]];
    }
}


/* need to fix this to put proper frame_id in header */

void PclUtils::transform_cloud( Eigen::Affine3f A, pcl::PointCloud<pcl::PointXYZ>::Ptr input_cloud_ptr,
                pcl::PointCloud<pcl::PointXYZ>::Ptr output_cloud_ptr )
{
    output_cloud_ptr->header    = input_cloud_ptr->header;
    output_cloud_ptr->is_dense  = input_cloud_ptr->is_dense;
    output_cloud_ptr->width     = input_cloud_ptr->width;
    output_cloud_ptr->height    = input_cloud_ptr->height;
    int npts = input_cloud_ptr->points.size();
    cout << "transforming npts = " << npts << endl;
    output_cloud_ptr->points.resize( npts );

    /* somewhat odd notation: getVector3fMap() reading OR WRITING points from/to a pointcloud, with conversions to/from Eigen */
    for ( int i = 0; i < npts; ++i )
    {
        output_cloud_ptr->points[i].getVector3fMap() = A * input_cloud_ptr->points[i].getVector3fMap();
    }
}


void PclUtils::transform_cloud( Eigen::Affine3f A, pcl::PointCloud<pcl::PointXYZRGB>::Ptr input_cloud_ptr,
                pcl::PointCloud<pcl::PointXYZRGB>::Ptr output_cloud_ptr )
{
    output_cloud_ptr->header    = input_cloud_ptr->header;
    output_cloud_ptr->is_dense  = input_cloud_ptr->is_dense;
    output_cloud_ptr->width     = input_cloud_ptr->width;
    output_cloud_ptr->height    = input_cloud_ptr->height;
    int npts = input_cloud_ptr->points.size();
    cout << "transforming npts = " << npts << endl;
    output_cloud_ptr->points.resize( npts );
    /* output_cloud_ptr->points.clear(); */

    /* somewhat odd notation: getVector3fMap() reading OR WRITING points from/to a pointcloud, with conversions to/from Eigen */
    float           xval;
    pcl::PointXYZRGB    pcl_pt;
    Eigen::Vector3f     pt1, pt2;
    for ( int i = 0; i < npts; ++i )
    {
        pt1 = input_cloud_ptr->points[i].getVector3fMap();
        /*
         * cout<<"pt1: "<<pt1.transpose()<<endl;
         * if (pt1(0)!= pt1(0)) { //option: could remove NaN's; odd syntax: will be true if NaN
         * ROS_WARN("pt %d: Nan",i);
         * }
         * else  {
         */
        pt2 = A * pt1; /* transform these coordinates */
        /* cout<<"pt2: "<<pt2.transpose()<<endl; */
        pcl_pt.x    = pt2( 0 );
        pcl_pt.y    = pt2( 1 );
        pcl_pt.z    = pt2( 2 );
        pcl_pt.rgb  = input_cloud_ptr->points[i].rgb;
        /* output_cloud_ptr->points.push_back(pcl_pt); // = A * input_cloud_ptr->points[i].getVector3fMap(); */
        output_cloud_ptr->points[i] = pcl_pt;
        /*
         * output_cloud_ptr->points[i].rgb = input_cloud_ptr->points[i].rgb;
         * }
         */
    }
    int npts_out = output_cloud_ptr->points.size();
    /*
     * output_cloud_ptr->width = npts_out;
     * output_cloud_ptr->height = 1;
     * ROS_INFO("transformed cloud w/ NaNs removed has %d points",npts_out);
     */
}


/*
 * member helper function to set up subscribers;
 * note odd syntax: &ExampleRosClass::subscriberCallback is a pointer to a member function of ExampleRosClass
 * "this" keyword is required, to refer to the current instance of ExampleRosClass
 */

void PclUtils::initializeSubscribers()
{
    ROS_INFO( "Initializing Subscribers" );

    pointcloud_subscriber_  = nh_.subscribe( "/kinect/depth/points", 1, &PclUtils::kinectCB, this );
    real_kinect_subscriber_ = nh_.subscribe( "/camera/depth_registered/points", 1, &PclUtils::kinectCB, this );
    /* add more subscribers here, as needed */

    /* subscribe to "selected_points", which is published by Rviz tool */
    selected_points_subscriber_ = nh_.subscribe<sensor_msgs::PointCloud2> ( "/selected_points", 1, &PclUtils::selectCB, this );
}


/* member helper function to set up publishers; */

void PclUtils::initializePublishers()
{
    ROS_INFO( "Initializing Publishers" );
    pointcloud_publisher_   = nh_.advertise<sensor_msgs::PointCloud2>( "cwru_pcl_pointcloud", 1, true );
    patch_publisher_    = nh_.advertise<cwru_msgs::PatchParams>( "pcl_patch_params", 1, true );
    /*
     * add more publishers, as needed
     * note: COULD make minimal_publisher_ a public member function, if want to use it within "main()"
     */
}


/**
 * callback fnc: receives transmissions of Kinect data; if got_kinect_cloud is false, copy current transmission to internal variable
 * @param cloud [in] messages received from Kinect
 */
void PclUtils::kinectCB( const sensor_msgs::PointCloud2ConstPtr & cloud )
{
    /*
     * cout<<"callback from kinect pointcloud pub"<<endl;
     * convert/copy the cloud only if desired
     */
    if ( !got_kinect_cloud_ )
    {
        pcl::fromROSMsg( *cloud, *pclKinect_ptr_ );
        pcl::fromROSMsg( *cloud, *pclKinect_clr_ptr_ );
        ROS_INFO( "kinectCB: got cloud with %d * %d points", (int) pclKinect_ptr_->width, (int) pclKinect_ptr_->height );
        got_kinect_cloud_ = true; /* cue to "main" that callback received and saved a pointcloud */
        /* check some colors: */
        int npts_clr = pclKinect_clr_ptr_->points.size();
        cout << "Kinect color pts size = " << npts_clr << endl;
        avg_color_ = find_avg_color();


        /*
         * for (size_t i = 0; i < pclKinect_clr_ptr_->points.size (); ++i)
         * std::cout << " " << (int) pclKinect_clr_ptr_->points[i].r
         *        << " "    << (int) pclKinect_clr_ptr_->points[i].g
         *        << " "    << (int) pclKinect_clr_ptr_->points[i].b << std::endl;
         *
         *
         *  //cout<<" ipt, r,g,b = "<<i<<","<<pclSelectedPtsClr_ptr_->points[i].r<<", "<<
         *  //        pclSelectedPtsClr_ptr_->points[i].g<<", "<<pclSelectedPtsClr_ptr_->points[i].b<<endl;
         *  //pclGenPurposeCloud_ptr_->points[i].getVector3fMap() = pclGenPurposeCloud_ptr_->points[i].getVector3fMap()+offset;
         *
         *  cout<<"done combing through selected pts"<<endl;
         *     */
    }
    /*
     * pcl::io::savePCDFileASCII ("snapshot.pcd", *g_pclKinect);
     * ROS_INFO("saved PCD image consisting of %d data points to snapshot.pcd",(int) g_pclKinect->points.size ());
     */
}


/* this callback wakes up when a new "selected Points" message arrives */

void PclUtils::selectCB( const sensor_msgs::PointCloud2ConstPtr & cloud )
{
    pcl::fromROSMsg( *cloud, *pclSelectedPoints_ptr_ );

    /*
     * looks like selected points does NOT include color of points
     * pcl::fromROSMsg(*cloud, *pclSelectedPtsClr_ptr_); //color version
     */

    ROS_INFO( "RECEIVED NEW PATCH w/  %d * %d points", pclSelectedPoints_ptr_->width, pclSelectedPoints_ptr_->height );
    /* ROS_INFO("frame_id = %s",pclSelectedPoints_ptr_->header.frame_id); */
    std::cout << "frame_id =" << pclSelectedPoints_ptr_->header.frame_id << endl;
    Eigen::Vector3f plane_normal;
    double      plane_dist;
    fit_points_to_plane( pclSelectedPoints_ptr_, plane_normal, plane_dist );
    ROS_INFO( "plane dist = %f", plane_dist );
    ROS_INFO( "plane normal = (%f, %f, %f)", plane_normal( 0 ), plane_normal( 1 ), plane_normal( 2 ) );
    patch_normal_   = plane_normal;
    patch_dist_ = plane_dist;


    /*
     * ROS_INFO("Color version has  %d * %d points", pclSelectedPtsClr_ptr_->width, pclSelectedPtsClr_ptr_->height);
     *
     * for (size_t i = 0; i < pclSelectedPtsClr_ptr_->points.size (); ++i) {
     * std::cout <<i<<": "
     *        << "    " << (int) pclSelectedPtsClr_ptr_->points[i].r
     *        << " "    << (int) pclSelectedPtsClr_ptr_->points[i].g
     *        << " "    << (int) pclSelectedPtsClr_ptr_->points[i].b << std::endl;
     * }
     * */
    ROS_INFO( "done w/ selected-points callback" );

    got_selected_points_ = true;
}


/* ************************************** the added func ************************************** */

void PclUtils::from_RGB_to_XYZ( pcl::PointCloud<pcl::PointXYZRGB>::Ptr rgbCloud, pcl::PointCloud<pcl::PointXYZ>::Ptr &xyzCloud )
{
    int input_size = rgbCloud->points.size();
    xyzCloud->header    = rgbCloud->header;
    xyzCloud->is_dense  = rgbCloud->is_dense;
    xyzCloud->points.resize( input_size );

    for ( int i = 0; i < input_size; ++i )
    {
        xyzCloud->points[i].getVector3fMap() = rgbCloud->points[i].getVector3fMap();
    }
}


void PclUtils::seek_rough_table_merry( pcl::PointCloud<pcl::PointXYZRGB>::Ptr input_cloud_ptr, pcl::PointCloud<pcl::PointXYZRGB>::Ptr &table_pts_cloud )
{
    table_pts_cloud->header.frame_id    = input_cloud_ptr->header.frame_id;
    table_pts_cloud->is_dense       = input_cloud_ptr->is_dense;

    /* now pick points from input */
    int input_size = input_cloud_ptr->points.size();
    ROS_INFO( "input has %d points", input_size );

    double  dist        = 0.0;
    double  pt_dx       = 0.0;
    double  pt_dy       = 0.0;
    double  pt_dz       = 0.0; /* camera_depth_frame */
    double  x       = 0.0;
    double  y       = 0.0;
    double  z       = 0.0;
    double  dot_product = 0.0;

    pcl::PointXYZRGB temp_point;

    for ( int i = 0; i < input_size; ++i )
    {
        pt_dx       = input_cloud_ptr->points[i].x - table_origin[0]; /* what about pts_3, still  = 0 */
        pt_dy       = input_cloud_ptr->points[i].y - table_origin[1];
        pt_dz       = input_cloud_ptr->points[i].z - table_origin[2];
        dot_product = pt_dx * table_normal[0] + pt_dy * table_normal[1] + pt_dz * table_normal[2];
        if ( dot_product < 0.0001 && dot_product > 0 )
        {
            /* ROS_INFO("has point that dot product = 0 !!!"); */

            dist = sqrt( pt_dx * pt_dx + pt_dy * pt_dy + pt_dz * pt_dz ); /* euclidean dist */
            if ( dist < 5 )
            {
                temp_point = input_cloud_ptr->points[i];
                table_pts_cloud->push_back( temp_point );
            }
        }
    }

    int table_pts_size = table_pts_cloud->points.size();
    ROS_INFO( "table has %d points", table_pts_size );

    Eigen::Vector3f plane_normal;
    double      plane_dist;
    /*
     * for (int i = 0; i < table_size; ++i)
     * {
     *     cout<<table_pts->points[i].getVector3fMap().transpose() <<endl;
     * }
     */
    fit_points_to_plane( table_pts_cloud, plane_normal, plane_dist );
    ROS_INFO( "plane dist = %f", plane_dist );
    ROS_INFO( "plane normal = (%f, %f, %f)", plane_normal( 0 ), plane_normal( 1 ), plane_normal( 2 ) );
    patch_normal_   = plane_normal;
    patch_dist_ = plane_dist;
}


void PclUtils::seek_rough_table_merry( pcl::PointCloud<pcl::PointXYZRGB>::Ptr input_cloud_ptr, double table_height, pcl::PointCloud<pcl::PointXYZRGB>::Ptr &table_pts_cloud )
{
    table_pts_cloud->points.clear();

    double  pts_1_x = -0.182946;
    double  pts_1_y = -0.235985;
    double  pts_1_z = 1.03673;

    double  pts_2_x = 0.0558143;
    double  pts_2_y = -0.23511;
    double  pts_2_z = 1.03458;

    double  pts_3_x = -0.0395804;   /* /actual origin in our table space */
    double  pts_3_y = -0.0460593;
    double  pts_3_z = 1.0051;

    double norm_z = -0.4;           /* preset the z of the norm of the table */

    double  dist        = 0.0;
    double  dot_product = 0.0;

    table_origin[0] = pts_3_x;
    table_origin[1] = pts_3_y;
    table_origin[2] = pts_3_z;

    std::vector<double> vec_1;                  /* vetors in the table */
    std::vector<double> vec_2;

    vec_1.resize( 3 );
    vec_2.resize( 3 );

    vec_1[0]    = pts_1_x - table_origin[0];    /* x */
    vec_1[1]    = pts_1_y - table_origin[1];    /* y */
    vec_1[2]    = pts_1_z - table_origin[2];    /* z */

    vec_2[0]    = pts_2_x - table_origin[0];
    vec_2[1]    = pts_2_y - table_origin[1];
    vec_2[2]    = pts_2_z - table_origin[2];

    /* compute norm_y */
    double  dz  = norm_z - table_origin[2];
    double  dy  = dz * (vec_2[2] * vec_1[0] - vec_1[2] * vec_2[0]);
    dy = dy / (vec_1[1] * vec_2[0] - vec_2[1] * vec_1[0]);
    double norm_y = dy + table_origin[1];
    /* compute norm_x */
    double dx = -dz * vec_1[2] - dy * vec_1[1];
    dx = dx / vec_1[0];
    double norm_x = dx + table_origin[0];

    double norm = dx * dx + dy * dy + dz * dz;

    table_normal[0] = dx;
    table_normal[1] = dy;
    table_normal[2] = dz;
    double theta;

    if ( table_height > 0 ) /* this table heigh is a relative height bwteen the basic table and another table */
    {
        for ( int i = 0; i < input_cloud_ptr->points.size(); ++i )
        {
            dx  = input_cloud_ptr->points[i].x - table_origin[0];
            dy  = input_cloud_ptr->points[i].y - table_origin[1];
            dz  = input_cloud_ptr->points[i].z - table_origin[2];
            dist    = sqrt( dx * dx + dy * dy + dz * dz );
            if ( fabs( dist - table_height ) > 0 && fabs( dist - table_height ) < 0.00001 ) /* /if the point is table height away */
            {
                dot_product = dx * table_normal[0] + dy * table_normal[1] + dz * table_normal[2];
                theta       = fabs( dot_product - dist * norm );
                if ( theta > 0 && theta < 0.001 )                                       /* right above */
                {
                    ROS_INFO( "find new table origin" );
                    table_origin[0] = input_cloud_ptr->points[i].x;
                    table_origin[1] = input_cloud_ptr->points[i].y;
                    table_origin[2] = input_cloud_ptr->points[i].z;
                    break;
                }
            }
        }
    }
    seek_rough_table_merry( input_cloud_ptr, table_pts_cloud );
}


void PclUtils::find_final_table_merry( pcl::PointCloud<pcl::PointXYZRGB>::Ptr input_cloud_ptr, pcl::PointCloud<pcl::PointXYZRGB>::Ptr &output_pts_cloud )
{
    output_pts_cloud->points.clear();

    double  dist    = 0.0;
    double  dx  = 0.0;
    double  dy  = 0.0;
    double  dz  = 0.0;

    pcl::PointXYZRGB temp_point;

    output_pts_cloud->header.frame_id   = input_cloud_ptr->header.frame_id;
    output_pts_cloud->is_dense      = input_cloud_ptr->is_dense;

    int input_size = input_cloud_ptr->points.size();
    for ( int i = 0; i < input_size; ++i )
    {
        dx  = input_cloud_ptr->points[i].x - table_origin[0];       /* what about pts_3, still  = 0 */
        dy  = input_cloud_ptr->points[i].y - table_origin[1];
        dz  = input_cloud_ptr->points[i].z - table_origin[2];
        dist    = sqrt( dx * dx + dy * dy + dz * dz );
        if ( dist < 1 )                                                 /* the range for finding the table */
        {
            temp_point = input_cloud_ptr->points[i];

            output_pts_cloud->push_back( temp_point );
        }
    }
    int outputsize = output_pts_cloud->points.size();

    if (outputsize == 0)
    {
        is_final_cloud = false;
        ROS_WARN(" No nice table found, please adjust your kinect perspective !");
    }
    else{
        ROS_INFO( "final_cloud has %d points", outputsize );
        is_final_cloud = true;
    }

}


void PclUtils::seek_coke_can_cloud( pcl::PointCloud<pcl::PointXYZRGB>::Ptr input_cloud_ptr, pcl::PointCloud<pcl::PointXYZRGB>::Ptr &coke_can_pts )
{
    coke_can_pts->points.clear();

    double  dx  = 0.0;
    double  dy  = 0.0;
    double  dz  = 0.0;

    double  in_table_frame_height   = 0.0;
    double  dist            = 0.0;
    double  height_diff     = 0.0;
    double  dist_diff       = 0.0;
    double  des_dis         = 0.0;

    double  dot_product = 0.0;
    pcl::PointXYZRGB    temp_point;

    coke_can_pts->header.frame_id   = input_cloud_ptr->header.frame_id;
    coke_can_pts->is_dense      = input_cloud_ptr->is_dense;

    Eigen::Vector3i can_color;
    can_color << 254, 254, 254;
    double  deltaR      = 0.0;
    double  deltaG      = 0.0;
    double  deltaB      = 0.0;
    double  delta_color = 0.0;

    Eigen::Vector3i point_color;

    int input_size = input_cloud_ptr->points.size();
    for ( int i = 0; i < input_size; ++i )
    {
        dx      = input_cloud_ptr->points[i].x - table_origin[0];       /* what about pts_3, still  = 0 */
        dy      = input_cloud_ptr->points[i].y - table_origin[1];
        dz      = input_cloud_ptr->points[i].z - table_origin[2];
        dot_product = dx * table_normal[0] + dy * table_normal[1] + dz * table_normal[2];

        in_table_frame_height   = dot_product / sqrt( table_normal[0] * table_normal[0] + table_normal[1] * table_normal[1] + table_normal[2] * table_normal[2] );
        height_diff     = fabs( in_table_frame_height - can_height );
        if ( height_diff < 0.05 && height_diff > 0 )                            /* if the point has the height of the can, so the point will be on the can height plane */
        {
            dist    = sqrt( dx * dx + dy * dy + dz * dz );
            des_dis = sqrt( 0.3 * 0.3 + can_height * can_height );
            /* dist_diff = fabs(des_dis - dist); */
            if ( dist < des_dis )                                           /* in certain range */
            {
                point_color = input_cloud_ptr->points[i].getRGBVector3i();
                deltaR      = abs( point_color[0] - can_color[0] ) / 255.0;
                deltaG      = abs( point_color[1] - can_color[1] ) / 255.0;
                deltaB      = abs( point_color[2] - can_color[2] ) / 255.0;
                delta_color = sqrt( deltaR * deltaR + deltaG * deltaG + deltaB * deltaB );
                if ( delta_color < 0.3 )                                /* pick the right color if necessary */
                {
                    temp_point = input_cloud_ptr->points[i];

                    coke_can_pts->push_back( temp_point );
                }
            }
        }
    }
    int cansize = coke_can_pts->points.size();
    ROS_WARN( "can cloud has %d points", cansize );

    /*
     * first compute the centroid of the data:
     * Eigen::Vector3f centroid; // make this member var, centroid_
     */
    pcl::PointCloud<pcl::PointXYZ>::Ptr can_xyz( new pcl::PointCloud<pcl::PointXYZ>);
    from_RGB_to_XYZ( coke_can_pts, can_xyz );
    Eigen::Vector3f centroid_ = compute_centroid( can_xyz ); /* see http://eigen.tuxfamily.org/dox/AsciiQuickReference.txt */

    cout << "can top centroid: " << centroid_.transpose() << endl;
}


bool PclUtils::is_coke_can( pcl::PointCloud<pcl::PointXYZRGB>::Ptr input_cloud_ptr )
{
    int can_size    = input_cloud_ptr->points.size();
    double  x       = 0.0;
    double  y       = 0.0;
    double  z       = 0.0;

    bool    no_x;
    bool    no_y;
    bool    no_z;

    for ( int i = 0; i < can_size; ++i )
    {       /* //if the number is nada */
        x   = input_cloud_ptr->points[i].x;
        y   = input_cloud_ptr->points[i].y;
        z   = input_cloud_ptr->points[i].z;
        if ( x != x )
        {
            no_x = true;
        }else { no_x = false; }

        if ( y != y )
        {
            no_y = true;
        }else { no_y = false; }

        if ( z != z )
        {
            no_z = true;
        }else { no_z = false; }

        if ( no_x || no_y || no_z )
        {
            return(false);
        }else { return(true); }
    }
}


Eigen::Vector3f PclUtils::find_can_bottom( pcl::PointCloud<pcl::PointXYZRGB>::Ptr table_cloud_ptr )
{
    double  norm_table  = sqrt( table_normal[0] * table_normal[0] + table_normal[1] * table_normal[1] + table_normal[2] * table_normal[2] );
    double  norm_coke   = 0.0;
    double  dot_product = 0.0;
    double  dist_diff   = 0.0;
    double  dist        = 0.0;
    double  diff        = 0.0;

    double  dx  = 0.0;
    double  dy  = 0.0;
    double  dz  = 0.0;

    Eigen::Vector3f can_bottom;

    int input_size = table_cloud_ptr->points.size();
    for ( int i = 0; i < input_size; ++i )
    {
        dx      = centroid_[0] - table_cloud_ptr->points[i].x;
        dy      = centroid_[1] - table_cloud_ptr->points[i].y;
        dz      = centroid_[2] - table_cloud_ptr->points[i].z;
        dist        = sqrt( dx * dx + dy * dy + dz * dz );
        dist_diff   = dist - can_height;
        if ( dist_diff > 0 && dist_diff < 0.001 )       /* /first want to find a point which is can_height away with the top */
        {
            dot_product = dx * table_normal[0] + dy * table_normal[1] + dz * table_normal[2];
            norm_coke   = sqrt( dx * dx + dy * dy + dz * dz );
            diff        = fabs( dot_product - (norm_table * norm_coke) );

            if ( diff >= 0 && diff <= 0.2 )         /* //see if the point is right in the bottom of the can */
            {
                //ROS_INFO( "find can bottom !!" );
                can_bottom[0]   = table_cloud_ptr->points[i].x;
                can_bottom[1]   = table_cloud_ptr->points[i].y;
                can_bottom[2]   = table_cloud_ptr->points[i].z;
                break;
            }
        }
    }
    return(can_bottom);
}


