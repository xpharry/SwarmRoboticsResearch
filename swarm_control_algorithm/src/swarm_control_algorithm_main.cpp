#include <swarm_control_algorithm/swarm_control_algorithm.h>
#include <iostream>

using namespace std;

int main(int argc, char **argv) {
  ros::init(argc, argv, "des_state_publisher");
  ros::NodeHandle nh;
  //instantiate a desired-state publisher object
  SwarmControlAlgorithm swarm_control_algorithm(nh);
  
  ros::Publisher geo_twist = nh.advertise <geometry_msgs::Twist> ("robot1/cmd_vel", 1, true);

  ros::Rate looprate(1 / dt); //timer for fixed publication rate
  //put some points in the path queue--hard coded here
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

  swarm_control_algorithm.set_initial_position(x_vec, y_vec);  ///psi defined to be 0 

  swarm_control_algorithm.set_target_position(10.0, 10.0, 0.0);
/*
need to know obst_positions
*/
geometry_msgs::PoseStamped temp;
temp.pose.position.x = 5;
temp.pose.position.y = 3;

  std::vector<geometry_msgs::PoseStamped> obst_posi;
  obst_posi.push_back(temp);

  std::vector< std::vector<geometry_msgs::PoseStamped> > obstacles;
  obstacles.resize(6);
  for(int i = 0; i < robot_quantity; i++) {
    obstacles[i].push_back(temp);
  }

  swarm_control_algorithm.ComputeConsumption(obstacles);

  swarm_control_algorithm.DecisionMaker();   //give corresponding targets

  ////swarm_1
  swarm_control_algorithm.swarm_obstacles_state(obst_posi, swarm_control_algorithm.current_pose[0], swarm_control_algorithm.target_pose[0],   ////PSO compute pose vector
    swarm_control_algorithm.desired_path[0]); //desired_path_1 

  swarm_control_algorithm.swarm_obstacles_state(obst_posi, swarm_control_algorithm.current_pose[1], swarm_control_algorithm.target_pose[1],
    swarm_control_algorithm.desired_path[1]); //desired_path_2 

  swarm_control_algorithm.swarm_obstacles_state(obst_posi, swarm_control_algorithm.current_pose[2], swarm_control_algorithm.target_pose[2],
    swarm_control_algorithm.desired_path[2]); //desired_path_3   
  
  swarm_control_algorithm.swarm_obstacles_state(obst_posi, swarm_control_algorithm.current_pose[3], swarm_control_algorithm.target_pose[3],
    swarm_control_algorithm.desired_path[3]); //desired_path_4   

  swarm_control_algorithm.swarm_obstacles_state(obst_posi, swarm_control_algorithm.current_pose[4], swarm_control_algorithm.target_pose[4],
    swarm_control_algorithm.desired_path[4]); //desired_path_5

  swarm_control_algorithm.swarm_obstacles_state(obst_posi, swarm_control_algorithm.current_pose[5], swarm_control_algorithm.target_pose[5],
    swarm_control_algorithm.desired_path[5]); //desired_path_6  
  // main loop; publish a desired state every iteration
    
  std::vector<nav_msgs::Odometry> vec_of_tran;
  geometry_msgs::PoseStamped new_temp;
  int des_num =  swarm_control_algorithm.desired_path[0].size();
  for(int i = 0; i < des_num; i++){
  	  new_temp.pose.position = swarm_control_algorithm.desired_path[0][i].pose.pose.position;
      double x = new_temp.pose.position.x;
      ROS_INFO("des x %d is: %f", i, x);

      double y = new_temp.pose.position.y;
      ROS_INFO("des y %d is: %f", i, y);
  }

  swarm_control_algorithm.build_point_and_go(swarm_control_algorithm.current_pose[0],new_temp,vec_of_tran);
  double position_x = vec_of_tran[10].pose.pose.position.x;
  double position_y = vec_of_tran[10].pose.pose.position.y;

  ROS_INFO("position x is %f",position_x);
  ROS_INFO("position y is %f", position_y);

  int num = vec_of_tran.size();

  for(int i = 0; i < num; i++){
  	  vec_of_tran[i].header.stamp = ros::Time::now();
	  geometry_msgs::Twist g_twist = vec_of_tran[i].twist.twist;
	  double x_speed = g_twist.linear.x;
	  double z_speed = g_twist.angular.z;
	//  ROS_INFO("x speed is %f",x_speed);
	//  ROS_INFO("z speed is %f",z_speed);
	  geo_twist.publish(g_twist);
	  looprate.sleep(); 
  }
 
    ros::spinOnce();
    //sleep for defined sample period, then do loop again

}

