#ifndef GRIPPER_CONTROLLER_H_
#define GRIPPER_CONTROLLER_H_

#include <ros/ros.h>
#include <std_msgs/Int16.h>
#include <std_msgs/Bool.h>
#include <math.h>

class GripperController {

public:
	
	GripperController(ros::NodeHandle* nodehandle);
	
	void close();
	void open();
	void midpoint();

private:
	
	ros::NodeHandle nh_;
	ros::Publisher dyn_pub_;
	ros::Publisher torque_toggle_;

	void initialize_motor();
};

#endif
