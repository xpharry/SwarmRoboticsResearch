#include <ros/ros.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>
#include <tf/transform_datatypes.h>
#include <string>

typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;
using namespace std;

int main(int argc, char** argv) {

    if (argc < 5) {
        ROS_ERROR("You must specify parameters.");
        return -1;
    }
    char *robot_id = argv[1];
    char *x = argv[2];
    char *y = argv[3];
    char *theta = argv[4];

    ros::init(argc, argv, "multi_navigation_send_goals");
    ros::NodeHandle nh;

    // Define the goal
    double goal_x = atof(x);
    double goal_y = atof(y);
    double goal_theta = atof(theta);

    // Create the string "robot_X/move_base"
    string move_base_str = "/robot_";
    move_base_str += robot_id;
    move_base_str += "/move_base";

    // create the action client
    MoveBaseClient ac(move_base_str, true);

    // Wait for the action server to become available
    ROS_INFO("Waiting for the move_base action server");
    ac.waitForServer(ros::Duration(5));

    ROS_INFO("Connected to move base server");

    // Send a goal to move_base
    move_base_msgs::MoveBaseGoal goal;
    goal.target_pose.header.frame_id = "map";
    goal.target_pose.header.stamp = ros::Time::now();

    goal.target_pose.pose.position.x = goal_x;
    goal.target_pose.pose.position.y = goal_y;
    // Convert the Euler angle to quaternion
    double radians = goal_theta * (M_PI/180);
    tf::Quaternion quaternion;
    quaternion = tf::createQuaternionFromYaw(radians);

    geometry_msgs::Quaternion qMsg;
    tf::quaternionTFToMsg(quaternion, qMsg);
    goal.target_pose.pose.orientation = qMsg;

    ROS_INFO("Sending goal to robot no. %s: x = %f, y = %f, theta = %f", robot_id, goal_x, goal_y, goal_theta);
    ac.sendGoal(goal);

    // Wait for the action to return
    ac.waitForResult();

    if (ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
        ROS_INFO("You have reached the goal!");
    else
        ROS_INFO("The base failed for some reason");

    return 0;
}
