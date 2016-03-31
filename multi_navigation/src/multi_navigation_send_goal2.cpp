#include <ros/ros.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>
#include <tf/transform_datatypes.h>

typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;
using namespace std;

int main(int argc, char** argv) {

    ros::init(argc, argv, "multi_navigation_send_goals_v2");
    ros::NodeHandle nh;

    //int robot_quantity = 6;

    // Define the goal
    double goal_x[] = {4.0, 5.0, 6.0, 7.0, 8.0, 9.0};
    double goal_y[] = {4.0, 5.0, 6.0, 7.0, 8.0, 9.0};
    double goal_theta[] = {0,0,0,0,0,0};

    int robot_id = 1;

    stringstream ss;
    ss << robot_id;
    string str = ss.str();

    // Create the string "robot_X/move_base"
    string move_base_str = "/robot_";
    move_base_str += str;
    move_base_str += "/move_base";

    // create the action client
    MoveBaseClient ac(move_base_str, true);

    // Wait for the action server to become available
    ROS_INFO("Waiting for the move_base action server");
    ac.waitForServer(ros::Duration(5));

    ROS_INFO("Connected to move base server");

    for(int i = 0; i < 2; i++) {

        // Send a goal to move_base
        move_base_msgs::MoveBaseGoal goal;
        goal.target_pose.header.frame_id = "map";
        goal.target_pose.header.stamp = ros::Time::now();

        goal.target_pose.pose.position.x = goal_x[i];
        goal.target_pose.pose.position.y = goal_y[i];
        // Convert the Euler angle to quaternion
        double radians = goal_theta[i] * (M_PI/180);
        tf::Quaternion quaternion;
        quaternion = tf::createQuaternionFromYaw(radians);

        geometry_msgs::Quaternion qMsg;
        tf::quaternionTFToMsg(quaternion, qMsg);
        goal.target_pose.pose.orientation = qMsg;

        ROS_INFO("Sending goal to robot no. %d: x = %f, y = %f, theta = %f",
            robot_id, goal_x[i], goal_y[i], goal_theta[i]);
        ac.sendGoal(goal);

        // Wait for the action to return
        ac.waitForResult();

        if (ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
            ROS_INFO("You have reached the goal!");
        else
            ROS_INFO("The base failed for some reason");

        ros::spinOnce();
        ros::Duration(0.5).sleep();
    }

    return 0;
}
