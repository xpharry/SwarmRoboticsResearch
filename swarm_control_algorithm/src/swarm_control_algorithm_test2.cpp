#include <swarm_control_algorithm/swarm_control_algorithm.h>
#include <iostream>
#include <swarm_robot_msgs/swarm_robot_poses.h>
#include <actionlib/client/simple_action_client.h>
#include <swarm_robot_action/swarm_robot_trajAction.h>

using namespace std;

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
    ros::init(argc, argv, "final");
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

    // ********************************************************************************************************************************

    /* Initialization */

    // instantiate a desired-state publisher object
    SwarmControlAlgorithm swarm_control_algorithm;

    swarm_robot_action::swarm_robot_trajGoal goal;  // instantiate a goal message

    ROS_INFO("here 1");

    std::vector<double> x_vec;
    std::vector<double> y_vec;
    x_vec.resize(6);
    y_vec.resize(6);
    x_vec[0] = 0;   //1
    y_vec[0] = 0; 
    x_vec[1] = 1;   //2
    y_vec[1] = 0;
    x_vec[2] = 2;   //3
    y_vec[2] = 0;
    x_vec[3] = -1;   //4
    y_vec[3] = 0;
    x_vec[4] = -2;   //5
    y_vec[4] = 0;
    x_vec[5] = 0;   //6
    y_vec[5] = -1;

    ROS_INFO("here 2");
        
    swarm_control_algorithm.set_initial_position(x_vec, y_vec);

    // ****** send goal *******
    goal.x.resize(robot_quantity);  // important here, otherwise runtime error
    goal.y.resize(robot_quantity);  // important here, otherwise runtime error
    for (int irobot=0; irobot<robot_quantity; irobot++) {
        goal.x[irobot] = g_robot_x[irobot] + x_vec[irobot] / 100;
        goal.y[irobot] = g_robot_x[irobot] + y_vec[irobot] / 100; // - g_robot_y[irobot];
        ROS_INFO("******* x: %f, y: %f ********", goal.x[irobot], goal.y[irobot]);
    }

    // send out goal
    action_client.sendGoal(goal);
    // wait for expected duration plus some tolerance (2 seconds)
    bool finish_before_timeout = action_client.waitForResult(ros::Duration(10));
    if (!finish_before_timeout) {
        ROS_WARN("this action is not done...timeout");
        return 0;
    }
    else {
        ROS_INFO("this action is done.");
    }
    // send goal ends
/*
    swarm_control_algorithm.set_target_position(5.0, 5.0, 0.0);

    ROS_INFO("here 3");

    // obstacle positions
    geometry_msgs::PoseStamped temp;
    temp.pose.position.x = 0;
    temp.pose.position.y = 0;

    std::vector<geometry_msgs::PoseStamped> obst_posi;
    obst_posi.push_back(temp);

    std::vector< std::vector<geometry_msgs::PoseStamped> > obstacles;
    obstacles.resize(6);
    for(int i = 0; i < robot_quantity; i++) {
        obstacles[i].push_back(temp);
    }

    ROS_INFO("here 4");

    swarm_control_algorithm.ComputeConsumption(obstacles);

    ROS_INFO("here 5");

    swarm_control_algorithm.DecisionMaker();   //give corresponding targets

    ROS_INFO("here 6");

    // swarm
    swarm_control_algorithm.swarm_obstacles_state(obst_posi);

    ROS_INFO("here 7");

    std::vector< std::vector<nav_msgs::Odometry> > des_state;
    int max_size = 0;
    for(int i = 0; i < robot_quantity; i++) {
        ROS_INFO("------------ robot #%d ------------", i+1);
        des_state.push_back(swarm_control_algorithm.desired_path[i]);
        std::vector<nav_msgs::Odometry> path = swarm_control_algorithm.desired_path[i];
        if( path.size() > max_size ) max_size = path.size();
        for(int j = 0; j < path.size(); j++) {
            ROS_INFO("******* x: %f, y: %f ********", path[j].pose.pose.position.x, path[j].pose.pose.position.y);
        }
    }

    std::vector< std::vector<nav_msgs::Odometry> > des_state_by_step;
    des_state_by_step.resize(max_size);
    for(int i = 0; i < max_size; i++) {
        ROS_INFO("------------ path step #%d ------------", i+1);
        des_state_by_step[i].resize(robot_quantity);
        for(int j = 0; j < robot_quantity; j++) {
            if( i < swarm_control_algorithm.desired_path[j].size() ) {
                des_state_by_step[i][j] = swarm_control_algorithm.desired_path[j][i];
            } else {
                des_state_by_step[i][j] = des_state_by_step[i-1][j];
            }
            ROS_INFO("*******robot %d: x = %f, y = %f ********", j+1,
                des_state_by_step[i][j].pose.pose.position.x, des_state_by_step[i][j].pose.pose.position.y);
        }
    }    

    ROS_INFO("swarm_control_algorithm ends here ...");
    // main loop; publish a desired state every iteration
    // **************************************************************************

    // // initialize an action client
    // actionlib::SimpleActionClient<swarm_robot_action::swarm_robot_trajAction> action_client(
    //     "two_wheel_traj_action", true);

    // // try to connect the client to action server
    // bool server_exist = action_client.waitForServer(ros::Duration(5.0));
    // ros::Duration(1.0).sleep();
    // while (!server_exist) {
    //     ROS_WARN("could not connect to server; retrying");
    //     bool server_exist = action_client.waitForServer(ros::Duration(1.0));
    //     ros::Duration(1.0).sleep();
    // }
    // // if here, then connected to the server
    // ROS_INFO("connected to action server");

    // the loop of optimizing robot position for dispersion
   for(int iteration_index = 0; iteration_index < max_size; iteration_index++) {
        ROS_INFO_STREAM("");  // blank line
        ROS_INFO_STREAM("iteration index: " << iteration_index);  // iteration index

        ros::spinOnce();  // let robot positions update, will be used later

        // prepare the goal message
        // goal.x.resize(robot_quantity);  // important here, otherwise runtime error
        // goal.y.resize(robot_quantity);  // important here, otherwise runtime error
        for (int irobot=0; irobot<robot_quantity; irobot++) {
            goal.x[irobot] = des_state_by_step[iteration_index][irobot].pose.pose.position.x / 100;
            goal.y[irobot] = des_state_by_step[iteration_index][irobot].pose.pose.position.y / 100;
            ROS_INFO("******* x: %f, y: %f ********", goal.x[irobot], goal.y[irobot]);
        }

        // send out goal
        action_client.sendGoal(goal);
        ros::Duration(0.5).sleep();

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
*/
    return 0;
}

