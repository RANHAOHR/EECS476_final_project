#include <ros/ros.h> 
#include <gazebo_msgs/ModelStates.h>
#include <geometry_msgs/Pose.h>
#include <std_msgs/Float64.h>
#include <string.h>
#include <stdio.h>  
#include <math.h>
#include <random>


geometry_msgs::Pose g_mobot_pose; //this is the pose of the robot in the world, according to Gazebo
geometry_msgs::Pose g_noisy_mobot_pose; //added noise to x,y, and suppress orientation
geometry_msgs::Quaternion g_quat;
ros::Publisher g_pose_publisher; 
ros::Publisher g_yaw_publisher; 
ros::Publisher g_gps_publisher; 
std::normal_distribution<double> distribution(0.0,1.0); //args: mean, std_dev
std::default_random_engine generator;

double convertPlanarQuat2Phi(geometry_msgs::Quaternion quaternion) {
    double quat_z = quaternion.z;
    double quat_w = quaternion.w;
    double phi = 2.0 * atan2(quat_z, quat_w); // cheap conversion from quaternion to heading for planar motion
    return phi;
}


void model_state_CB(const gazebo_msgs::ModelStates& model_states) 
{ 
  int n_models = model_states.name.size();
  int imodel;
  //ROS_INFO("there are %d models in the transmission",n_models);
  bool found_name=false;
  for (imodel=0;imodel<n_models;imodel++) {
    std::string model_name(model_states.name[imodel]); 
    if (model_name.compare("mobot")==0) {
      ROS_INFO("found match: mobot is model %d",imodel);
      found_name=true;
      break;
    }
  }
  if(found_name) {
    g_mobot_pose= model_states.pose[imodel];
    g_pose_publisher.publish(g_mobot_pose);
    g_noisy_mobot_pose = g_mobot_pose;
    g_noisy_mobot_pose.orientation = g_quat;
    g_noisy_mobot_pose.position.x += distribution(generator);
    g_noisy_mobot_pose.position.y += distribution(generator);    
    g_gps_publisher.publish(g_noisy_mobot_pose); //publish noisy values
    double gazebo_yaw = convertPlanarQuat2Phi(g_mobot_pose.orientation);
    std_msgs::Float64 yaw_msg;
    yaw_msg.data = gazebo_yaw;
    g_yaw_publisher.publish(yaw_msg);
    }
  else
    {
      ROS_WARN("state of mobot model not found");
    }
} 



int main(int argc, char **argv) {
    ros::init(argc, argv, "gazebo_model_publisher");
    ros::NodeHandle nh;
    ROS_INFO("gazebo model state publisher");
    g_pose_publisher= nh.advertise<geometry_msgs::Pose>("gazebo_mobot_pose", 1); 
    g_yaw_publisher = nh.advertise<std_msgs::Float64>("gazebo_mobot_yaw", 1);
    g_gps_publisher = nh.advertise<geometry_msgs::Pose>("gazebo_mobot_noisy_pose", 1);
    ros::Subscriber state_sub = nh.subscribe("gazebo/model_states",1,model_state_CB); 
    //suppress the orientation output for noisy state; fill out a legal, constant quaternion
    g_quat.x=0;
    g_quat.y=0;
    g_quat.z=0;
    g_quat.w=1;
    ros::spin();
}
