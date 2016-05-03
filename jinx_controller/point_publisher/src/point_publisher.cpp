#include <pub_des_state/pub_des_state.h>
#include <traj_builder/traj_builder.h>
#include <iostream>
using namespace std;

int main(int argc, char **argv) {
	ros::init(argc, argv, "point_publisher");
	ros::NodeHandle nh;
	//instantiate a desired-state publisher object
	DesStatePublisher desStatePublisher(nh);
	ros::Rate looprate(1 / dt); //timer for fixed publication rate
	desStatePublisher.set_init_pose(0,0,0);
	desStatePublisher.append_path_queue(2.0,0.0,0);

	ros::Publisher backUp_publisher_ = nh.advertise<nav_msgs::Odometry>("/backup", 1, true);

	int counter = 0;
	while (ros::ok()) {

		if (counter == 300 || counter == 600) {
			nav_msgs::Odometry dummy_state;
			backUp_publisher_.publish(dummy_state);
			ROS_INFO("published dummy state!");
		}

		desStatePublisher.pub_next_state();

		if(desStatePublisher.get_estop_trigger())
		{
			ROS_INFO("estop is pressed");
		}

		ros::spinOnce();
		looprate.sleep();

		counter += 1;
	}

}
