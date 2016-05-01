#include <ros/ros.h>
#include <std_msgs/Int16.h>
#include <gripper_controller/gripper_controller.h>
#include <math.h>

int input_command = 0;
double dt = 0.02;

//ros::Publisher my_publisher_object = nh.advertise<std_msgs::Int16>("gripper_cmd", 1);
//my_publisher_object.publish(input);
void manualCmdCB(const std_msgs::Int16& input_cmd) 
{ 

  if (input_cmd.data == 0) {
    ROS_INFO("received open command");
  } 
  else if (input_cmd.data == 1) {
    ROS_INFO("received close command");
  } 
  else if (input_cmd.data == 2) {
    ROS_INFO("going to midpoint");
  }
  else if (input_cmd.data == 3) {
    ROS_INFO("received quit command");
  }

  input_command = input_cmd.data;
} 

int main(int argc, char **argv) {
    
  ros::init(argc, argv, "gripper_controller"); 
  ros::NodeHandle nh; 

 	GripperController gripper_controller(&nh);
 	
 	ros::Subscriber manual_cmd_subscriber = nh.subscribe("gripper_cmd",1,manualCmdCB);     
 	ros::Rate naptime(1/dt);

 	std_msgs::Int16 action_msg;

 	while(ros::ok()) {
 		
 		action_msg.data = input_command;

  		if(action_msg.data == 3) 
  			break;		
	 	switch(action_msg.data) {
	 		case 0:
	 			gripper_controller.open();
	 			break;
	 		case 1:
	 			gripper_controller.close();
	 			break;
      case 2:
        gripper_controller.midpoint();
        break;
	 		default:
	 			break;
	 	}

	 	ros::spinOnce();
	 	naptime.sleep();
	}

 	ROS_INFO("doneski!");
}
