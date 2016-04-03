#include <swarm_control_algorithm/swarm_control_algorithm.h>
#include <ros/ros.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>
#include <tf/transform_datatypes.h>
#include <nav_msgs/Path.h>
#include <boost/thread.hpp>


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
    nav_msgs::Path constructPath(std::vector<nav_msgs::Odometry>);
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

nav_msgs::Path MultiNavigation::constructPath(std::vector<nav_msgs::Odometry> path) {
    nav_msgs::Path new_path;
    new_path.poses.resize( path.size() );
    for(int i = 0; i < path.size(); i++) {
        new_path.poses[i].pose = path[i].pose.pose;

        // Convert the Euler angle to quaternion
        double radians = 45 * (M_PI/180);
        tf::Quaternion quaternion;
        quaternion = tf::createQuaternionFromYaw(radians);

        geometry_msgs::Quaternion qMsg;
        tf::quaternionTFToMsg(quaternion, qMsg);
        new_path.poses[i].pose.orientation = qMsg;
    }
    return new_path;
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

int main(int argc, char **argv) {
    ros::init(argc, argv, "multi_navigation_with_swarm_control_final");
    ros::NodeHandle nh;

    // get initialization message of robot swarm from parameter server
    int robot_quantity = 6;

    // ********************************************************************************************************************************

    /* Initialization */

    // instantiate a desired-state publisher object
    SwarmControlAlgorithm swarm_control_algorithm;

    ROS_INFO("here 1");

    std::vector<double> x_vec;
    std::vector<double> y_vec;
    x_vec.resize(6);
    y_vec.resize(6);
    x_vec[0] = 2;   //1
    y_vec[0] = 1; 
    x_vec[1] = 2;   //2
    y_vec[1] = 4;
    x_vec[2] = 2;   //3
    y_vec[2] = 6;
    x_vec[3] = 2;   //4
    y_vec[3] = 8;
    x_vec[4] = 2;   //5
    y_vec[4] = 10;
    x_vec[5] = 2;   //6
    y_vec[5] = 12;

    ROS_INFO("here 2");
        
    swarm_control_algorithm.set_initial_position(x_vec, y_vec);

    swarm_control_algorithm.set_target_position(30.0, 30.0, 0.0);

    ROS_INFO("here 3");

    // obstacle positions
    geometry_msgs::PoseStamped temp;
    temp.pose.position.x  = 10.269 ;
    temp.pose.position.y  =  10.001;

    std::vector<geometry_msgs::PoseStamped> obst_posi;
    obst_posi.push_back(temp);

    temp.pose.position.x  = 10.669 ;
    temp.pose.position.y  =  9.501; 
    obst_posi.push_back(temp);

        temp.pose.position.x  = 10.000;
    temp.pose.position.y  =  8.001; 
    obst_posi.push_back(temp);

    temp.pose.position.x  = 11.000;
    temp.pose.position.y  = 10.001; 
    obst_posi.push_back(temp);

    temp.pose.position.x  = 10.669 ;
    temp.pose.position.y  = 9.801  ; 
    obst_posi.push_back(temp);

    temp.pose.position.x  = 12.269;
    temp.pose.position.y  =  9.001; 
    obst_posi.push_back(temp);

    temp.pose.position.x  = 11.269 ;
    temp.pose.position.y  = 9.001; 
    obst_posi.push_back(temp);


    temp.pose.position.x  = 10.269 ;
    temp.pose.position.y  =  11.001 ; 
    obst_posi.push_back(temp);


    temp.pose.position.x  = 9.569 ;
    temp.pose.position.y  = 10.001 ; 
    obst_posi.push_back(temp);

    temp.pose.position.x  =  9.000 ;
    temp.pose.position.y  = 9.801; 
    obst_posi.push_back(temp);
////////////////////////////////////////////////////////////
    temp.pose.position.x  = 10.669 ;
    temp.pose.position.y  = 19.501; 
    obst_posi.push_back(temp);

    temp.pose.position.x  = 10.000;
    temp.pose.position.y  = 20.001; 
    obst_posi.push_back(temp);

    temp.pose.position.x  = 10.000 ;
    temp.pose.position.y  = 22.001; 
    obst_posi.push_back(temp);

    temp.pose.position.x  = 11.000 ;
    temp.pose.position.y  = 21.001 ; 
    obst_posi.push_back(temp);

    temp.pose.position.x  = 10.669 ;
    temp.pose.position.y  = 20.801; 
    obst_posi.push_back(temp);

    temp.pose.position.x  = 12.269 ;
    temp.pose.position.y  = 19.001; 
    obst_posi.push_back(temp);    

    temp.pose.position.x  = 11.269 ;
    temp.pose.position.y  = 20.001; 
    obst_posi.push_back(temp);


    temp.pose.position.x  = 10.269 ;
    temp.pose.position.y  = 20.501; 
    obst_posi.push_back(temp);


    temp.pose.position.x  = 9.569 ;
    temp.pose.position.y  = 20.001; 
    obst_posi.push_back(temp);

    temp.pose.position.x  =  9.000 ;
    temp.pose.position.y  = 21.801 ; 
    obst_posi.push_back(temp);
///////////////////////////////////////////
    temp.pose.position.x  = 20.269 ;
    temp.pose.position.y  = 10.001; 
    obst_posi.push_back(temp);

    temp.pose.position.x  = 18.669 ;
    temp.pose.position.y  = 10.501; 
    obst_posi.push_back(temp);

    temp.pose.position.x  = 19.000 ;
    temp.pose.position.y  = 10.001; 
    obst_posi.push_back(temp);

    temp.pose.position.x  = 20.000 ;
    temp.pose.position.y  = 10.501 ; 
    obst_posi.push_back(temp);

    temp.pose.position.x  = 19.669 ;
    temp.pose.position.y  = 10.801; 
    obst_posi.push_back(temp);

    temp.pose.position.x  = 21.269 ;
    temp.pose.position.y  = 10.001; 
    obst_posi.push_back(temp);


    temp.pose.position.x  = 21.269 ;
    temp.pose.position.y  = 9.001; 
    obst_posi.push_back(temp);


    temp.pose.position.x  = 20.269 ;
    temp.pose.position.y  = 11.001; 
    obst_posi.push_back(temp);

    temp.pose.position.x  =  19.569;
    temp.pose.position.y  =  10.801; 
    obst_posi.push_back(temp);

    temp.pose.position.x  =  18.000 ;
    temp.pose.position.y  =  10.801 ; 
    obst_posi.push_back(temp);  

    std::vector< std::vector<geometry_msgs::PoseStamped> > obstacles;
    obstacles.resize(6);
    for(int i = 0; i < robot_quantity; i++) {
        obstacles[i].push_back(temp);
    }

    swarm_control_algorithm.ComputeConsumption(obstacles);

    //swarm_control_algorithm.DecisionMaker();   //give corresponding targets
     swarm_control_algorithm.MarkovDecision();
    // swarm
    swarm_control_algorithm.swarm_obstacles_state(obst_posi);

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

    // std::vector< std::vector<nav_msgs::Odometry> > des_state_by_step;
    // des_state_by_step.resize(max_size);
    // for(int i = 0; i < max_size; i++) {
    //     ROS_INFO("------------ path step #%d ------------", i+1);
    //     des_state_by_step[i].resize(robot_quantity);
    //     for(int j = 0; j < robot_quantity; j++) {
    //         if( i < swarm_control_algorithm.desired_path[j].size() ) {
    //             des_state_by_step[i][j] = swarm_control_algorithm.desired_path[j][i];
    //         } else {
    //             des_state_by_step[i][j] = des_state_by_step[i-1][j];
    //         }
    //         ROS_INFO("*******robot %d: x = %f, y = %f ********", j+1,
    //             des_state_by_step[i][j].pose.pose.position.x, des_state_by_step[i][j].pose.pose.position.y);
    //     }
    // } 

    ROS_INFO("swarm_control_algorithm ends here ...");

    // ********************************************************************************************************************************

    ROS_INFO(" ");
    ROS_INFO("multi_navigation starts here ...");

    // int i = 1;
    // // for(int i = 0; i < robot_quantity; i++) {
    //     MultiNavigation multi_navigation(i);

    //     nav_msgs::Path path = multi_navigation.constructPath(des_state[i]);
    //     for(int j = 0; j < path.poses.size(); j++) {
    //         ROS_INFO("******* x: %f, y: %f ********", path.poses[j].pose.position.x, path.poses[j].pose.position.y);
    //     }

    //     multi_navigation.run(path);  
    // // }

    MultiNavigation multi_navigation(1);
    nav_msgs::Path path = multi_navigation.constructPath(des_state[1]);
    boost::thread thread(boost::bind(&MultiNavigation::run, &multi_navigation, path));

    ROS_INFO("waiting for thread ...");
    thread.join();     
    // ros::Duration(0.5).sleep();
    ROS_INFO("done!");

    return 0;
}

