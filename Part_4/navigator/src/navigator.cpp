// example_action_server: a simple action server
// Wyatt Newman

#include <ros/ros.h>
#include <actionlib/server/simple_action_server.h>
#include <navigator/navigatorAction.h>
#include <pub_des_state/pub_des_state.h>
#include <geometry_msgs/PoseStamped.h>
#include <traj_builder/traj_builder.h>

class Navigator {
private:

    ros::NodeHandle nh_;  // we'll need a node handle; get one upon instantiation
    actionlib::SimpleActionServer<navigator::navigatorAction> navigator_as_;
    ros::Duration looptimer; 

    // here are some message types to communicate with our client(s)
    navigator::navigatorGoal goal_; // goal message, received from client
    navigator::navigatorResult result_; // put results here, to be sent back to the client when done w/ goal
    navigator::navigatorFeedback feedback_; // not used in this example; 
    // would need to use: as_.publishFeedback(feedback_); to send incremental feedback to the client
    int t_;
    TrajBuilder trajBuilder_;
    ros::ServiceClient append_client_;
    ros::ServiceClient empty_client_;

    int navigate_home();
    int navigate_to_table();
    int navigate_to_pose(geometry_msgs::PoseStamped goal_pose);

public:
    Navigator(); //define the body of the constructor outside of class definition

    ~Navigator(void) {
    }
    // Action Interface
    void executeCB(const actionlib::SimpleActionServer<navigator::navigatorAction>::GoalConstPtr& goal);
};



Navigator::Navigator() :
   navigator_as_(nh_, "navigatorActionServer", boost::bind(&Navigator::executeCB, this, _1),false),
   looptimer(1)
 {
    ROS_INFO("in constructor of navigator...");
    // do any other desired initializations here...specific to your implementation
    append_client_ = nh_.serviceClient<mobot_pub_des_state::path>("/append_path_queue_service");
    empty_client_ = nh_.serviceClient<mobot_pub_des_state::emptyQueue>("/empty_queue_service");
    navigator_as_.start(); //start the server running
    t_ = 0;
}

//specialized function: DUMMY...JUST RETURN SUCCESS...fix this
//this SHOULD do the hard work of navigating to HOME
int Navigator::navigate_home() { 
    std::vector<geometry_msgs::PoseStamped> points;
    mobot_pub_des_state::path path_srv;
    points.push_back(trajBuilder_.xyPsi2PoseStamped(2.0,0.0,3.14159265358979323846264338));
    points.push_back(trajBuilder_.xyPsi2PoseStamped(2.0,0.0,-1.57));
    points.push_back(trajBuilder_.xyPsi2PoseStamped(0.0,0.0,0.0));
    path_srv.request.path.poses = points;
    append_client_.call(path_srv);
    t_ = 0;
    looptimer.sleep();
    mobot_pub_des_state::emptyQueue emptyQueueMsg;
    empty_client_.call(emptyQueueMsg);
    while (!emptyQueueMsg.response.empty) {
        if (t_ > 30) {
            return navigator::navigatorResult::FAILED_CANNOT_REACH_DES_POSE;
        }
        t_ += 1.0;
        looptimer.sleep();
        empty_client_.call(emptyQueueMsg); 
    }
    return navigator::navigatorResult::DESIRED_POSE_ACHIEVED; //just say we were successful
} 

int Navigator::navigate_to_table() { 
    std::vector<geometry_msgs::PoseStamped> points;
    mobot_pub_des_state::path path_srv;
    points.push_back(trajBuilder_.xyPsi2PoseStamped(3.0,0.0,0.0));
    path_srv.request.path.poses = points;
    append_client_.call(path_srv);
    t_ = 0;
    looptimer.sleep();
    mobot_pub_des_state::emptyQueue emptyQueueMsg;
    empty_client_.call(emptyQueueMsg);
    while (!emptyQueueMsg.response.empty) {
        if (t_ > 25) {
            return navigator::navigatorResult::FAILED_CANNOT_REACH_DES_POSE;
        }
        t_ += 1.0;
        looptimer.sleep();
        empty_client_.call(emptyQueueMsg); 
    }
    return navigator::navigatorResult::DESIRED_POSE_ACHIEVED; //just say we were successful
}

int Navigator::navigate_to_pose(geometry_msgs::PoseStamped goal_pose) {
    return navigator::navigatorResult::DESIRED_POSE_ACHIEVED;
}


void Navigator::executeCB(const actionlib::SimpleActionServer<navigator::navigatorAction>::GoalConstPtr& goal) {
    int destination_id = goal->location_code;
    geometry_msgs::PoseStamped destination_pose;
    int navigation_status;

    if (destination_id==navigator::navigatorGoal::COORDS) {
        destination_pose=goal->desired_pose;
    }
    
    switch(destination_id) {
        case navigator::navigatorGoal::HOME: 
              //specialized function to navigate to pre-defined HOME coords
               navigation_status = navigate_home(); 
               if (navigation_status==navigator::navigatorResult::DESIRED_POSE_ACHIEVED) {
                   ROS_INFO("reached home");
                   result_.return_code = navigator::navigatorResult::DESIRED_POSE_ACHIEVED;
                   navigator_as_.setSucceeded(result_);
               }
               else {
                   ROS_WARN("could not navigate home!");
                   result_.return_code = navigator::navigatorResult::FAILED_CANNOT_REACH_DES_POSE;
                   navigator_as_.setAborted(result_);
               }
               break;
        case navigator::navigatorGoal::TABLE: 
              //specialized function to navigate to pre-defined TABLE coords
               navigation_status = navigate_to_table(); 
               if (navigation_status==navigator::navigatorResult::DESIRED_POSE_ACHIEVED) {
                   ROS_INFO("reached table");
                   result_.return_code = navigator::navigatorResult::DESIRED_POSE_ACHIEVED;
                   navigator_as_.setSucceeded(result_);
               }
               else {
                   ROS_WARN("could not navigate to table!");
                   result_.return_code = navigator::navigatorResult::FAILED_CANNOT_REACH_DES_POSE;
                   navigator_as_.setAborted(result_);
               }
               break;
        case navigator::navigatorGoal::COORDS: 
              //more general function to navigate to specified pose:
              destination_pose=goal->desired_pose;
               navigation_status = navigate_to_pose(destination_pose); 
               if (navigation_status==navigator::navigatorResult::DESIRED_POSE_ACHIEVED) {
                   ROS_INFO("reached desired pose");
                   result_.return_code = navigator::navigatorResult::DESIRED_POSE_ACHIEVED;
                   navigator_as_.setSucceeded(result_);
               }
               else {
                   ROS_WARN("could not navigate to desired pose!");
                   result_.return_code = navigator::navigatorResult::FAILED_CANNOT_REACH_DES_POSE;
                   navigator_as_.setAborted(result_);
               }
               break;               
               
        default:
             ROS_WARN("this location ID is not implemented");
             result_.return_code = navigator::navigatorResult::DESTINATION_CODE_UNRECOGNIZED; 
             navigator_as_.setAborted(result_);
            }
  
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "navigator"); // name this node 

    ROS_INFO("instantiating the navigation action server: ");

    Navigator navigator_as; // create an instance of the class "ObjectFinder"

    ROS_INFO("going into spin");
    // from here, all the work is done in the action server, with the interesting stuff done within "executeCB()"
    // you will see 5 new topics under example_action: cancel, feedback, goal, result, status
    while (ros::ok()) {
        ros::spinOnce(); //normally, can simply do: ros::spin();  
        // for debug, induce a halt if we ever get our client/server communications out of sync
    }

    return 0;
}

