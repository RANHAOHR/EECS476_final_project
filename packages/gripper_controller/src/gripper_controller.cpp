#include <ros/ros.h>
#include <std_msgs/Int16.h>
#include <std_msgs/Bool.h>
#include <math.h>
#include <gripper_controller/gripper_controller.h>

GripperController::GripperController(ros::NodeHandle* nodehandle):

nh_(*nodehandle) {
    initialize_motor();
}

// startup
void GripperController::initialize_motor() {

    int motor_id = 1; 
    char cmd_topic_name[50];
    char cmd_topic_toggle[50];
    sprintf(cmd_topic_name,"dynamixel_motor%d_cmd",motor_id);
    sprintf(cmd_topic_toggle,"dynamixel_motor%d_mode",motor_id);
    ROS_INFO("using command topic: %s",cmd_topic_name);
    dyn_pub_ = nh_.advertise<std_msgs::Int16>(cmd_topic_name, 1);
    torque_toggle_ = nh_.advertise<std_msgs::Bool>(cmd_topic_toggle, 1);
}

//close gripper command
void GripperController::close() {

	std_msgs::Int16 cmd_msg;
	std_msgs::Bool toggle_msg;

	toggle_msg.data = 1;
   	cmd_msg.data = 80;

   	torque_toggle_.publish(toggle_msg);
   	dyn_pub_.publish(cmd_msg);

   	ros::spinOnce();
   	ros::Duration(5).sleep();
   
    // std_msgs::Int16 int_ang;
    // int_ang.data = 3675.0;

    // dyn_pub_.publish(int_ang);

    // ros::Duration(0.5).sleep();
    
    ROS_INFO("closed");
}

void GripperController::open() {

   std_msgs::Int16 cmd_msg;
   std_msgs::Bool toggle_msg;   

   toggle_msg.data = 0;
   cmd_msg.data = 3000;

   torque_toggle_.publish(toggle_msg);
   dyn_pub_.publish(cmd_msg);

   ros::spinOnce();
   ros::Duration(5).sleep();
   
   // std_msgs::Int16 int_ang;
    // int_ang.data = 3000.0;

    // dyn_pub_.publish(int_ang);
   
    // ros::Duration(0.5).sleep();

    ROS_INFO("open");
}

void GripperController::midpoint() {

    // std_msgs::Int16 int_ang;
    // int_ang.data = 3400.0;

    // dyn_pub_.publish(int_ang);
   
    // ros::Duration(0.5).sleep();

    // ROS_INFO("midpoint");
}

