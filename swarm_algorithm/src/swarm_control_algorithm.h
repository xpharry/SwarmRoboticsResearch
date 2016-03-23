#ifndef swarm_control_algorithm_H_
#define swarm_control_algorithm_H_

#include <queue>
#include <std_msgs/Float64.h>
#include <std_msgs/Bool.h>
#include <std_srvs/Trigger.h>
#include "trajectory_plan.cpp"
#include <std_msgs/Float64.h>
#include <math.h>

using namespace std;

const double dt = 0.02; //send desired-state messages at fixed rate, e.g. 0.02 sec = 50Hz
//dynamic parameters: should be tuned for target system
const double accel_max = 0.5; //1m/sec^2
const double alpha_max = 0.2; // rad/sec^2
const double speed_max = 0.5; //1 m/sec
const double omega_max = 0.5; //1 rad/sec
const double path_move_tol = 0.01; // if path points are within 1cm, fuggidaboutit

const double dange_range_1 = M_PI/4;
const double dange_range_2 = M_PI/8;  //may need to  be tuned!!!!!!!!!

const int robot_quantity = 6;

class SwarmControlAlgorithm {
private:
    ros::NodeHandle nh_;

public: 
    double accel_max_; 
    double alpha_max_; 
    double speed_max_; 
    double omega_max_; 
    double path_move_tol_; 

    double range_1;
    double range_2;

    geometry_msgs::Twist halt_twist_;
    
    std::vector<geometry_msgs::PoseStamped> current_pose;

    ros::Publisher desired_state_publisher_1;
    ros::Publisher desired_state_publisher_2;
    ros::Publisher desired_state_publisher_3;
    ros::Publisher desired_state_publisher_4;
    ros::Publisher desired_state_publisher_5;
    ros::Publisher desired_state_publisher_6;

    ros::Publisher des_psi_publisher_1;
    ros::Publisher des_psi_publisher_2;
    ros::Publisher des_psi_publisher_3;
    ros::Publisher des_psi_publisher_4;
    ros::Publisher des_psi_publisher_5;
    ros::Publisher des_psi_publisher_6;
	
    TrajBuilder trajBuilder_; 

    void initializePublishers();

    std::vector<geometry_msgs::PoseStamped> target_pose;
    std::vector< std::vector<nav_msgs::Odometry> > des_state_vec;
    std::vector< std::vector<double> > swarm_consump;
  

// public:

    SwarmControlAlgorithm(ros::NodeHandle& nh);//constructor  

    std::vector<geometry_msgs::PoseStamped> vec_of_targets_pose;
    std::vector<int> vec_of_decision;

    void set_initial_position(std::vector<double> x_vec, std::vector<double> y_vec);
    void set_des_pose(double x,double y, double psi);

	void swarm_obstacles_state(std::vector<geometry_msgs::PoseStamped> obst_posi,
			geometry_msgs::PoseStamped robot_pose,
			geometry_msgs::PoseStamped target_posi,
			std::vector<nav_msgs::Odometry> &vec_of_states);
			
    double SingleConsumpt(geometry_msgs::PoseStamped target,
	                       geometry_msgs::PoseStamped robot,
	      std::vector<geometry_msgs::PoseStamped> obstacle);
		  
    void ComputeConsumption(std::vector< std::vector<geometry_msgs::PoseStamped>  > obstacles);
    void DecisionMaker();
	void build_point_and_go(geometry_msgs::PoseStamped start_pose, geometry_msgs::PoseStamped end_pose,
        std::vector<nav_msgs::Odometry> &vec_states);	
};

#endif
