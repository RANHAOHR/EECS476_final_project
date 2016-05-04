#include "pub_des_state.h"
#include <iostream>
using namespace std;

bool *alarm_p;
bool *estop;

void alarmCB(const std_msgs::Bool g_alarm) {

  if (g_alarm.data == true){
    *alarm_p = true;
  }
  else{*alarm_p = false;}
}
void estopCallback(const std_msgs::Bool estop_call) {
  if (estop_call.data == true){
    *estop = false;
  }
  else{*estop = true;}
}

int main(int argc, char **argv) {
  ros::init(argc, argv, "des_state_publisher");
  ros::NodeHandle nh;
  //instantiate a desired-state publisher object
  DesStatePublisher desStatePublisher(nh);
  alarm_p = &(desStatePublisher.alarm);
  estop = &(desStatePublisher.h_e_stop_);

  ros::Subscriber sub = nh.subscribe("/lidar_alarm", 1, alarmCB);
  ros::Subscriber estop_subscriber = nh.subscribe("/motors_enabled", 1, estopCallback);

  //dt is set in header file pub_des_state.h    
  ros::Rate looprate(1 / dt); //timer for fixed publication rate
  desStatePublisher.set_init_pose(0,0,0); //x=0, y=0, psi=0
  //put some points in the path queue--hard coded here
  //desStatePublisher.append_path_queue(5.0,0.0,0.0);
  //desStatePublisher.append_path_queue(0.0,0.0,0.0);
  
  // main loop; publish a desired state every iteration
  int set_botton = 0;
  while (ros::ok()) {
    desStatePublisher.pub_next_state();
    ros::spinOnce();
    looprate.sleep(); //sleep for defined sample period, then do loop again
  }
  return 0;
}

