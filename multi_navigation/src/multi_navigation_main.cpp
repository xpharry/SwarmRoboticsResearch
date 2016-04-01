#include <ros/ros.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>
#include <tf/transform_datatypes.h>
#include <nav_msgs/Path.h>

typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;
using namespace std;

class MultiNavigation {
private:
    int robot_id_;
    string robot_str_;
    // create the action client
    MoveBaseClient *ac;
    string getRobotString(int robot_id);
public:
    MultiNavigation(int robot_id);
    void run(vector<double> x, vector<double> y, vector<double> theta);
    void run(nav_msgs::Path path);
};

MultiNavigation::MultiNavigation(int robot_id) {
    robot_id_ = robot_id;
    robot_str_ = getRobotString(robot_id);
    ac = new actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction>(robot_str_, true);
}

string MultiNavigation::getRobotString(int robot_id) {
    stringstream ss;
    ss << robot_id;
    string robot_id_str = ss.str();

    // Create the string "robot_X/move_base"
    robot_str_ = "/robot_";
    robot_str_ += robot_id_str;
    robot_str_ += "/move_base";

    return robot_str_;
}

void MultiNavigation::run(vector<double> x, vector<double> y, vector<double> theta) {

    int num = x.size();
    nav_msgs::Path path;
    path.poses.resize(num);

    for(int i = 0 ; i < num; i++) {
        path.poses[i].pose.position.x = x[i];
        path.poses[i].pose.position.y = y[i];

        // Convert the Euler angle to quaternion
        double radians = theta[i] * (M_PI/180);
        tf::Quaternion quaternion;
        quaternion = tf::createQuaternionFromYaw(radians);

        geometry_msgs::Quaternion qMsg;
        tf::quaternionTFToMsg(quaternion, qMsg);
        path.poses[i].pose.orientation = qMsg;
    }

    run(path);
}

void MultiNavigation::run(nav_msgs::Path path) {

    // Wait for the action server to become available
    ROS_INFO("Waiting for the move_base action server");
    ac->waitForServer(ros::Duration(5));

    ROS_INFO("Connected to move base server");

    for(int i = 0; i < path.poses.size(); i++) {
        // Send a goal to move_base
        move_base_msgs::MoveBaseGoal goal;
        goal.target_pose.header.frame_id = "map";
        goal.target_pose.header.stamp = ros::Time::now();

        goal.target_pose.pose.position.x = path.poses[i].pose.position.x;
        goal.target_pose.pose.position.y = path.poses[i].pose.position.y;
        goal.target_pose.pose.orientation = path.poses[i].pose.orientation;

        ac->sendGoal(goal);

        // Wait for the action to return
        ac->waitForResult();

        if (ac->getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
            ROS_INFO("You have reached the goal!");
        else
            ROS_INFO("The base failed for some reason");

        ros::spinOnce();
        ros::Duration(0.5).sleep();
    }
}

int main(int argc, char** argv) {

    ros::init(argc, argv, "multi_navigation_main");
    ros::NodeHandle nh;

    // Define the goal
    double goal_x[] = {4.0, 5.0, 6.0, 7.0, 8.0, 9.0};
    double goal_y[] = {4.0, 4.0, 4.0, 4.0, 4.0, 4.0};
    double goal_theta[] = {0,0,0,0,0,0};

    vector<double> x(goal_x, goal_x + sizeof(goal_x) / sizeof(double));
    vector<double> y(goal_y, goal_y + sizeof(goal_y) / sizeof(double));
    vector<double> theta(goal_theta, goal_theta + sizeof(goal_theta) / sizeof(double));

    int robot_id = 1;

    MultiNavigation multi_navigation(robot_id);

    multi_navigation.run(x,y,theta);

    return 0;
}
