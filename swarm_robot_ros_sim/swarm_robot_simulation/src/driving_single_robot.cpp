// this program is for the dispersion simulation of two wheel robot as a swarm robot

// communication including:
// subscribe to topic "swarm_robot_poses" to get current robot poses
// create an action client to "two_wheel_traj_action"

// impact avoiding is not implemented here, what's the better way to do that?

#include <ros/ros.h>
#include <math.h>
#include <swarm_robot_msgs/swarm_robot_poses.h>
#include <actionlib/client/simple_action_client.h>
#include <swarm_robot_action/swarm_robot_trajAction.h>

// global variables
std::vector<double> g_robot_x;
std::vector<double> g_robot_y;
std::vector<double> g_robot_angle;
bool g_robot_poses_cb_started = false;

// simulation control parameters
double spring_length = 0.7;  // spring length, may change from parameter server
const double upper_limit_ratio = 0.30;  // upper limit part relative to spring length
const double upper_limit = spring_length * (1 + upper_limit_ratio);
const double feedback_ratio = 0.382;  // smaller than 1 to make it stable, golden ratio ;)

// two wheel robot specification, really should get these values in another way
const double half_wheel_dist = 0.0177;
const double wheel_radius = 0.015;
double wheel_speed = 2.0;  // rad*s-1, for time cost of the action, may change also

// callback for message from topic "swarm_robot_poses"
void swarmRobotPosesCb(const swarm_robot_msgs::swarm_robot_poses& message_holder) {
    if (!g_robot_poses_cb_started)  // first time to be invoked
        g_robot_poses_cb_started = true;
    g_robot_x = message_holder.x;
    g_robot_y = message_holder.y;
    g_robot_angle = message_holder.angle;
}

int main(int argc, char **argv) {
    ros::init(argc, argv, "driving_single_robot");
    ros::NodeHandle nh;

    // get initialization message of robot swarm from parameter server
    std::string robot_model_name;
    int robot_quantity;
    bool get_name, get_quantity;
    get_name = nh.getParam("/robot_model_name", robot_model_name);
    get_quantity = nh.getParam("/robot_quantity", robot_quantity);
    if (!(get_name && get_quantity))
        return 0;  // return if fail to get parameter

    // get settings for this simulation from private parameter
    // parameter: spring_length
    bool get_spring_length = nh.getParam("spring_length", spring_length);
    if (get_spring_length)
        ROS_INFO_STREAM("using spring_length passed in: " << spring_length);
    else
        ROS_INFO_STREAM("using default spring_length: " << spring_length);
    // parameter: wheel_speed
    bool get_wheel_speed = nh.getParam("wheel_speed", wheel_speed);
    if (get_wheel_speed)
        ROS_INFO_STREAM("using wheel_speed passed in: " << wheel_speed);
    else
        ROS_INFO_STREAM("using default wheel_speed: " << wheel_speed);

    // initialize a subscriber to topic "swarm_robot_poses"
    ros::Subscriber swarm_robot_poses_subscriber = nh.subscribe("swarm_robot_poses", 1, swarmRobotPosesCb);

    // make sure topics "swarm_robot_poses" are active
    while (!g_robot_poses_cb_started) {
        ros::Duration(0.5).sleep();
        ros::spinOnce();
    }
    std::cout << "topic message from swarm_robot_poses is ready" << std::endl;

    // initialize an action client
    actionlib::SimpleActionClient<swarm_robot_action::swarm_robot_trajAction> action_client(
        "two_wheel_traj_action", true);
    swarm_robot_action::swarm_robot_trajGoal goal;  // instantiate a goal message

    // try to connect the client to action server
    bool server_exist = action_client.waitForServer(ros::Duration(5.0));
    ros::Duration(1.0).sleep();
    while (!server_exist) {
        ROS_WARN("could not connect to server; retrying");
        bool server_exist = action_client.waitForServer(ros::Duration(1.0));
        ros::Duration(1.0).sleep();
    }
    // if here, then connected to the server
    ROS_INFO("connected to action server");

    // the loop of optimizing robot position for dispersion
    int iteration_index = 0;
    while (ros::ok()) {
        iteration_index = iteration_index + 1;
        ROS_INFO_STREAM("");  // blank line
        ROS_INFO_STREAM("iteration index: " << iteration_index);  // iteration index

        ros::spinOnce();  // let robot positions update, will be used later

        // prepare the goal message
        goal.x.resize(robot_quantity);  // important here, otherwise runtime error
        goal.y.resize(robot_quantity);  // important here, otherwise runtime error
        for (int i=0; i<robot_quantity; i++) {
            goal.x[i] = g_robot_x[i] + 0.05;
            goal.y[i] = g_robot_y[i];
        }

        // send out goal
        action_client.sendGoal(goal);
        ros::Duration(1.0).sleep();
        
        // wait for expected duration plus some tolerance (2 seconds)
        bool finish_before_timeout = action_client.waitForResult(ros::Duration(10));
        if (!finish_before_timeout) {
            ROS_WARN("this action is not done...timeout");
            return 0;
        }
        else {
            ROS_INFO("this action is done.");
        }
    }
    return 0;
}

