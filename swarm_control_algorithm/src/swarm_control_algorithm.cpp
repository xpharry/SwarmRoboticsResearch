#include <swarm_control_algorithm/swarm_control_algorithm.h>


SwarmControlAlgorithm::SwarmControlAlgorithm() {
	range_1 = dange_range_1;
	range_2 = dange_range_2;

	current_pose.resize(robot_quantity);
    target_pose.resize(robot_quantity);
    desired_path.resize(robot_quantity);
    swarm_consump.resize(robot_quantity);
    for(int i = 0; i < robot_quantity; i++) {
    	swarm_consump[i].resize(robot_quantity);
    }
    vec_of_targets_pose.resize(6);
    vec_of_decision.resize(6);
}


/**
* Initialize initial positions
*
*/
void SwarmControlAlgorithm::set_initial_position(std::vector<double> x_vec, std::vector<double> y_vec) {
	for(int i = 0; i < robot_quantity; i++) {
		current_pose[i] = xyPsi2PoseStamped(x_vec[i], y_vec[i], 0);
	}
}


/**
* Initialize target positions
*
*/
void SwarmControlAlgorithm::set_target_position(double x, double y, double psi) {
	// build the pose for the robot leader
	target_pose[0] = xyPsi2PoseStamped(x, y, psi);

	// compute the poses for the remaining robots
	ComputeSubpositions();

	for(int i = 0; i < target_pose.size(); i++) {
		vec_of_targets_pose[i] = target_pose[i];
	}

}


//for planar motion, converts a quaternion to a scalar heading,
// where heading is measured CCW from x-axis
double SwarmControlAlgorithm::convertPlanarQuat2Psi(geometry_msgs::Quaternion quaternion) {
	double quat_z = quaternion.z;
	double quat_w = quaternion.w;
	double psi = 2.0 * atan2(quat_z, quat_w); // cheap conversion from quaternion to heading for planar motion
	return psi;
}


//given a heading for motion on a plane (as above), convert this to a quaternion
geometry_msgs::Quaternion SwarmControlAlgorithm::convertPlanarPsi2Quaternion(double psi) {
	geometry_msgs::Quaternion quaternion;
	quaternion.x = 0.0;
	quaternion.y = 0.0;
	quaternion.z = sin(psi / 2.0);
	quaternion.w = cos(psi / 2.0);
	return (quaternion);
}


//utility to fill a PoseStamped object from planar x,y,psi info
geometry_msgs::PoseStamped SwarmControlAlgorithm::xyPsi2PoseStamped(double x, double y, double psi) {
	geometry_msgs::PoseStamped poseStamped; // a pose object to populate
	poseStamped.pose.orientation = convertPlanarPsi2Quaternion(psi); // convert from heading to corresponding quaternion
	poseStamped.pose.position.x = x; // keep the robot on the ground!
	poseStamped.pose.position.y = y; // keep the robot on the ground!
	poseStamped.pose.position.z = 0.0; // keep the robot on the ground!
	return poseStamped;
}


/**
*  ?????
*
*/
void SwarmControlAlgorithm::swarm_obstacles_state(std::vector<geometry_msgs::PoseStamped> obst_posi) {
	for(int i = 0; i < robot_quantity; i++) {
    	swarm_obstacles_state(obst_posi, current_pose[i], target_pose[i], desired_path[i]); //desired_path_1 
    }
}


/**
*  ?????
*
*/
void SwarmControlAlgorithm::swarm_obstacles_state(std::vector<geometry_msgs::PoseStamped> obst_posi,
		geometry_msgs::PoseStamped robot_pose,
		geometry_msgs::PoseStamped target_posi,
		std::vector<nav_msgs::Odometry> &vec_of_states) {

	int Num = 150;
	double c1 = 0.7;
	double c2 = 0.7;
	int Miter = 20;
	double w = 0.75;

	//initialization
	double x_start = robot_pose.pose.position.x;
	double y_start = robot_pose.pose.position.y;

	srand((unsigned) time( NULL));

	std::vector<geometry_msgs::PoseStamped> particle_pose;
	std::vector<geometry_msgs::PoseStamped> velocity;
	std::vector<geometry_msgs::PoseStamped> local_pose;
	geometry_msgs::PoseStamped temp;
	geometry_msgs::PoseStamped vel;

	std::vector<double> local_best;

	double dx;
	double dy;
	double dist;
	double temp_evalu;

	int N = 999;
	
	for (int i = 0;i < Num; i++){
		temp.pose.position.x = x_start;
		temp.pose.position.y = y_start;

        vel.pose.position.x = rand() % (N + 1) / (float) (N + 1);
        vel.pose.position.y = rand() % (N + 1) / (float) (N + 1);

        //then everything should be same
		particle_pose.push_back(temp);
		local_pose.push_back(temp); //temprary local best


		temp_evalu = ComputeEvaluation(target_posi, obst_posi,
				temp);
		local_best.push_back(temp_evalu);
		velocity.push_back(vel);

	}

	int valid_num = particle_pose.size();

	geometry_msgs::PoseStamped global_best;

	global_best.pose.position.x = particle_pose[0].pose.position.x;
	global_best.pose.position.y = particle_pose[0].pose.position.y;


    nav_msgs::Odometry des_odom;
    //initialize global best
	double best_value;
	for (int i = 1; i < valid_num; i++) {
		best_value = ComputeEvaluation(target_posi, obst_posi,
				global_best);
		if (local_best[i] < best_value)
		global_best.pose.position.x = particle_pose[i].pose.position.x; ////////////
		global_best.pose.position.y = particle_pose[i].pose.position.y; ////////////
	}

	////main iteration
	vec_of_states.clear();
	for (int iter = 1; iter < Miter; iter++) {
		for (int n = 0; n < valid_num; n++) {
			velocity[n].pose.position.x = w * velocity[n].pose.position.x
					+ c1 * (rand() % (N + 1) / (float) (N + 1)) //need to be refinded
							* (local_pose[n].pose.position.x
									- particle_pose[n].pose.position.x)
					+ c2 * (rand() % (N + 1) / (float) (N + 1)) //need to be refinded
							* (global_best.pose.position.x
									- particle_pose[n].pose.position.x);
			velocity[n].pose.position.y = w * velocity[n].pose.position.y
					+ c1 * (rand() % (N + 1) / (float) (N + 1))
							* (local_pose[n].pose.position.y
									- particle_pose[n].pose.position.y)
					+ c2 * (rand() % (N + 1) / (float) (N + 1))
							* (global_best.pose.position.y
									- particle_pose[n].pose.position.y);
			particle_pose[n].pose.position.x = particle_pose[n].pose.position.x
					+ velocity[n].pose.position.x;
			particle_pose[n].pose.position.y = particle_pose[n].pose.position.y
					+ velocity[n].pose.position.y;

			temp_evalu = ComputeEvaluation(target_posi, obst_posi,
					particle_pose[n]);
			if (local_best[n] > temp_evalu) { /// update local best
				local_best[n] = temp_evalu;
				local_pose[n].pose.position.x =
						particle_pose[n].pose.position.x;
				local_pose[n].pose.position.y =
						particle_pose[n].pose.position.y;
			}

			best_value = ComputeEvaluation(target_posi, obst_posi,
					global_best);
			if (best_value > local_best[n]) {
				global_best.pose.position.x = local_pose[n].pose.position.x;
				global_best.pose.position.y = local_pose[n].pose.position.y;
			}
		}
		dx = global_best.pose.position.x - target_posi.pose.position.x;
		dy = global_best.pose.position.y - target_posi.pose.position.y;
		dist = sqrt(dx *dx + dy*dy);
		if( dist > 3){
			des_odom.pose.pose.position = global_best.pose.position;
			vec_of_states.push_back(des_odom);
		}  ///careful 
	}	
	des_odom.pose.pose.position = target_posi.pose.position;
	vec_of_states.push_back(des_odom);
}


/**
*  ?????
*
*/
void SwarmControlAlgorithm::ComputeConsumption(std::vector< std::vector<geometry_msgs::PoseStamped>  > obstacles) {
	//get all target position not acoordingly	
	for(int i = 1; i < obstacles.size(); i++) {
		for(int j = 1; j < robot_quantity; j++) {
			swarm_consump[i][j] = SingleConsumpt(target_pose[j], current_pose[i], obstacles[i]);
		}		
	}
}


/**
*  ?????
*
*/
double SwarmControlAlgorithm::SingleConsumpt(geometry_msgs::PoseStamped target,
	geometry_msgs::PoseStamped current,
	std::vector<geometry_msgs::PoseStamped> obstacle) {

	double target_dx = target.pose.position.x - current.pose.position.x;
	double target_dy = target.pose.position.y - current.pose.position.y;
	double dist = sqrt(target_dx * target_dx + target_dy * target_dy);

	double obst_dx;
	double obst_dy;
	double obst_dist;

	double dot_product;
	double mod;
	double dang_angle;
	///ROS_INFO
	double add_consumpt;
	double total_consumpt;
	int num = obstacle.size();

	for (int i = 0; i < num; i++) {
		obst_dx = obstacle[i].pose.position.x - current.pose.position.x;
		obst_dy = obstacle[i].pose.position.y - current.pose.position.y;
		obst_dist = sqrt(obst_dx * obst_dx + obst_dy * obst_dy);

		dot_product = target_dx * obst_dx + target_dy * obst_dy;
		mod = dist * obst_dist;
		dang_angle = dot_product / mod;

		if (fabs(dang_angle) <= range_1) {  ///symetric for both positive and negative
			add_consumpt = exp(-fabs(dang_angle)); //
		} else {
			add_consumpt = 0;
		}
		total_consumpt += add_consumpt;
	}

	return dist + total_consumpt + 1; // for convenience of proba
}


/**
*  ?????
*
*/
void SwarmControlAlgorithm::MarkovDecision() {

    int num2 = swarm_consump[1].size();
	for(int i = 1; i <num2; i++){
		swarm_consump[1][i] = 1/swarm_consump[1][i]; //reverse monotonicity
	}

	int num3 = swarm_consump[2].size();
	for(int i = 1; i <num3; i++){
		swarm_consump[2][i] = 1/swarm_consump[2][i]; //reverse monotonicity
	}

	int num4 = swarm_consump[3].size();
	for(int i = 1; i <num4; i++){
		swarm_consump[3][i] = 1/swarm_consump[3][i]; //reverse monotonicity		
	}

	int num5 = swarm_consump[4].size();
	for(int i = 1; i <num5; i++){
		swarm_consump[4][i] = 1/swarm_consump[4][i]; //reverse monotonicity		
	}	

	int num6 = swarm_consump[5].size();
	for(int i = 1; i <num6; i++){
		swarm_consump[5][i] = 1/swarm_consump[5][i]; //reverse monotonicity		
	}	
	
	double C_best = 0.0;
	double C;
	double post_prob;
    double c = 100.0;
	
    vec_of_decision[0] = 1;

    for(int i_2 = 1; i_2 <num2; i_2++) {
    	for(int i_3 = 1; i_3 <num3; i_3++) {
            if(i_2 != i_3){
                for(int i_4 = 1; i_4 <num4; i_4++) {
                    if(i_4 != i_3 && i_4 != i_2){
                        for(int i_5 = 1; i_5 <num5; i_5++){
                            if(i_5 != i_4 && i_5 != i_3 && i_5 != i_2) {
                               for(int i_6 = 1; i_6 <num6; i_6++){
                               	  if(i_6 != i_5 && i_6 != i_4 && i_6 != i_3 && i_6 != i_2) {
									//normalize
									
                                    post_prob = swarm_consump[1][i_2] * swarm_consump[2][i_3] *swarm_consump[3][i_4] *swarm_consump[4][i_5] *swarm_consump[5][i_6];
																		
									C = c * post_prob;

                                    if(C_best < C) {   // if not maxmum, replace                                    	

                                        C_best = C;
                                        
										vec_of_decision[1] = i_2+1;  ///index is from 0, but our target position is from 2
										vec_of_decision[2] = i_3+1;
										vec_of_decision[3] = i_4+1;
										vec_of_decision[4] = i_5+1;
										vec_of_decision[5] = i_6+1;

										target_pose[1] = vec_of_targets_pose[i_2];
										target_pose[2] = vec_of_targets_pose[i_3];
										target_pose[3] = vec_of_targets_pose[i_4];
										target_pose[4] = vec_of_targets_pose[i_5];
										target_pose[5] = vec_of_targets_pose[i_6];
                                    }
                               	  }
                               }
                            }
                        }
                    }
                }
            }
       }
    }

    int decision_index = 0;
	int decision_scale = vec_of_decision.size();
	ROS_INFO("**** decision_scale = %d", decision_scale);
	for (int i = 0; i < decision_scale; i ++) {
		decision_index = vec_of_decision[i];
		ROS_INFO("in order from 2 to 6, targets are: %d", decision_index);
	}	
}

void SwarmControlAlgorithm::DecisionMaker() {
	
	// normalize and change monoton
	// double delta = max - min;
	int num2 = swarm_consump[1].size();
	for(int i = 1; i <num2; i++){
		swarm_consump[1][i] = 1/swarm_consump[1][i]; //reverse monotonicity
	}

	int num3 = swarm_consump[2].size();
	for(int i = 1; i < num3; i++){
		swarm_consump[2][i] = 1/swarm_consump[2][i]; //reverse monotonicity
	}

	int num4 = swarm_consump[3].size();
	for(int i = 1; i <num4; i++){
		swarm_consump[3][i] = 1/swarm_consump[3][i]; //reverse monotonicity		
	}

	int num5 = swarm_consump[4].size();
	for(int i = 1; i <num5; i++){
		swarm_consump[4][i] = 1/swarm_consump[4][i]; //reverse monotonicity		
	}	

	int num6 = swarm_consump[5].size();
	for(int i = 1; i < num6; i++){
		swarm_consump[5][i] = 1/swarm_consump[5][i]; //reverse monotonicity		
	}	



    double norm;
    double proby_2;
    double proby_3;
    double proby_4;
    double proby_5;
    double proby_6;
    double Entropy;
    double BestEntropy = 1000;
    std::vector<double> vec_of_proby;
    vec_of_proby.resize(6); //as clear may let core dumpt
    vec_of_proby[0] = 1;
    vec_of_decision[0] = 1;
    
    for(int i_2 = 1; i_2 <num2; i_2++) {
    	for(int i_3 = 1; i_3 < num3; i_3++) {
            if(i_2 != i_3){
                for(int i_4 = 1; i_4 < num4; i_4++) {
                    if(i_4 != i_3 && i_4 != i_2){
                        for(int i_5 = 1; i_5 < num5; i_5++){
                            if(i_5 != i_4 && i_5 != i_3 && i_5 != i_2) {
                               for(int i_6 = 1; i_6 < num6; i_6++){
                               	  if(i_6 != i_5 && i_6 != i_4 && i_6 != i_3 && i_6 != i_2) {
                 

									//normalize
									proby_2 = swarm_consump[1][i_2];
									proby_3 = swarm_consump[2][i_3];
									proby_4 = swarm_consump[3][i_4];
									proby_5 = swarm_consump[4][i_5];
									proby_6 = swarm_consump[5][i_6];
									
									norm = proby_2+ proby_3 +proby_4 +proby_5 +proby_6;
									proby_2 = proby_2/norm;
									proby_3 = proby_3/norm;
									proby_4 = proby_4/norm;
									proby_5 = proby_5/norm;
									proby_6 = proby_6/norm;
									Entropy = -(proby_2 * log(proby_2) + proby_3 * log(proby_3) + proby_3 * log(proby_3)
										+ proby_4 * log(proby_4) + proby_5 * log(proby_5) + proby_6 * log(proby_6));


                                    if(Entropy < BestEntropy) {   // if not minimal, replace
                                    	
										BestEntropy = Entropy;
		
										vec_of_proby[1] = proby_2;
										vec_of_proby[2] = proby_3;
										vec_of_proby[3] = proby_4;
										vec_of_proby[4] = proby_5;
										vec_of_proby[5] = proby_6;
	
										vec_of_decision[1] = i_2+1;  
										vec_of_decision[2] = i_3+1;
										vec_of_decision[3] = i_4+1;
										vec_of_decision[4] = i_5+1;
										vec_of_decision[5] = i_6+1;

										target_pose[1] = vec_of_targets_pose[i_2];  
										target_pose[2] = vec_of_targets_pose[i_3];	
										target_pose[3] = vec_of_targets_pose[i_4];	
										target_pose[4] = vec_of_targets_pose[i_5];	
										target_pose[5] = vec_of_targets_pose[i_6];	
                                    }

                               	  }
                               }
                            }
                        }
                    }
                }
            }
       }
    }



	int decision_index = 0;
	int decision_scale = vec_of_decision.size();
	ROS_INFO("**** decision_scale = %d", decision_scale);
	for (int i = 0; i < decision_scale; i ++) {
		decision_index = vec_of_decision[i];
		ROS_INFO("in order from 2 to 6, targets are: %d", decision_index);
	}

	
}


/**
*  ?????
*
*/
double SwarmControlAlgorithm::ComputeEvaluation(geometry_msgs::PoseStamped target_posi, std::vector<geometry_msgs::PoseStamped> obst_posi, geometry_msgs::PoseStamped robot_posi) {
	   double w1 = 100;
	   double w2 = 50;

	   double tar_dx = robot_posi.pose.position.x - target_posi.pose.position.x;
	   double tar_dy = robot_posi.pose.position.y - target_posi.pose.position.y;
	   double tar_dist = sqrt(tar_dx * tar_dx + tar_dy * tar_dy);

	   double obs_dx;
	   double obs_dy;
	   double obs_dist;

	   double evaluation = w1 * tar_dist;
       int obst_num = obst_posi.size();
	   for(int i = 0; i< obst_num; i++){
		   obs_dx = robot_posi.pose.position.x - obst_posi[i].pose.position.x;
		   obs_dy = robot_posi.pose.position.y - obst_posi[i].pose.position.y;
		   obs_dist = sqrt(obs_dx * obs_dx + obs_dy * obs_dy);
		   evaluation += w2/obs_dist;
	   }
	   return evaluation;
}


/**
*  ?????
*
*/
void SwarmControlAlgorithm::ComputeSubpositions() {
	//please check if the coordinate is right
	// for(int i = 1; i < poses.size(); i++) {
	// 	// for robot #(i+1)
	// 	poses[i].pose.position.x = poses[0].pose.position.x - 1;
	// 	poses[i].pose.position.y = poses[0].pose.position.y + sqrt(3);		
	// }

	// for robot #2
	target_pose[1].pose.position.x = target_pose[0].pose.position.x - 2;
	target_pose[1].pose.position.y = target_pose[0].pose.position.y + 2*sqrt(3);

	// for robot #3
	target_pose[2].pose.position.x = target_pose[0].pose.position.x - 6;
	target_pose[2].pose.position.y = target_pose[0].pose.position.y + 2*sqrt(3);

	// for robot #4
	target_pose[3].pose.position.x = target_pose[0].pose.position.x - 2;
	target_pose[3].pose.position.y = target_pose[0].pose.position.y - 2*sqrt(3);

	// for robot #5
	target_pose[4].pose.position.x = target_pose[0].pose.position.x - 6;
	target_pose[4].pose.position.y = target_pose[0].pose.position.y - 2*sqrt(3);

	// for robot #6
	target_pose[5].pose.position.x = target_pose[0].pose.position.x - 8;
	target_pose[5].pose.position.y = target_pose[0].pose.position.y;
}