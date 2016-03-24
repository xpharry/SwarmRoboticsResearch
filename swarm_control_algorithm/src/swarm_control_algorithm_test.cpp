#include "swarm_control_algorithm.cpp"


int main(int argc, char **argv) {
    ros::init(argc, argv, "final");
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
    x_vec[0] = 0;   //1
    y_vec[0] = 0; 
    x_vec[1] = 1;   //2
    y_vec[1] = 0;
    x_vec[2] = 2;   //3
    y_vec[2] = 0;
    x_vec[3] = 0;   //4
    y_vec[3] = 5;
    x_vec[4] = 2;   //5
    y_vec[4] = 5;
    x_vec[5] = 0;   //6
    y_vec[5] = 4;

    ROS_INFO("here 2");
        
    swarm_control_algorithm.set_initial_position(x_vec, y_vec);

    swarm_control_algorithm.set_target_position(10.0, 10.0, 0.0);

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
    for(int i = 0; i < robot_quantity; i++) {
        ROS_INFO("------------ robot #%d ------------", i+1);
        des_state.push_back(swarm_control_algorithm.desired_path[i]);
        std::vector<nav_msgs::Odometry> path = swarm_control_algorithm.desired_path[i];
        for(int j = 0; j < path.size(); j++) {
            ROS_INFO("******* x: %f, y: %f ********", path[j].pose.pose.position.x, path[j].pose.pose.position.y);
        }
    }

    ROS_INFO("swarm_control_algorithm ends here ...");

    // ********************************************************************************************************************************

    return 0;
}

