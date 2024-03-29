// Copyright (c) 2017 Franka Emika GmbH
// Use of this source code is governed by the Apache-2.0 license, see LICENSE
#include <iostream>

#include <franka/exception.h>
#include <franka/robot.h>
#include <vector>
#include <eigen3/Eigen/Dense>
#include <thread>
#include "ik.h"
#include <chrono>
/**
 * @example echo_robot_state.cpp
 * An example showing how to continuously read the robot state.
 */


namespace robotContext {
    franka::Robot *robot;
    franka::Gripper *gripper;
    franka::Model *model;

}




namespace global_variable{
  std::array< double, 16 > current_ee_pose; 
  bool flag_done_collecting; 
  std::vector<std::array< double, 16 >> collected_ee_poses;
}

std::vector<std::array< double, 16 >> record_pose_thread(int n_poses=3){
  int collected_poses = 0; 
  std::string my_string = "";
  
  while(collected_poses < n_poses){
    std::cout << "Press ENTER to collect current pose, anything else to quit data collection" << std::endl;
    std::getline(std::cin, my_string);
    if(my_string.length()==0){
      global_variable::collected_ee_poses.push_back(global_variable::current_ee_pose);
      collected_poses++;
    }
    else{
      std::cout << "Exiting data collection"<<std::endl; 
      global_variable::flag_done_collecting = true; 
      break;
    }
  }
  global_variable::flag_done_collecting = true; 


}

double distance(const Eigen::Vector3d &v1,const Eigen::Vector3d &v2){
  Eigen::Vector3d diff = v1 - v2;
  return diff.norm();
}


int main(int argc, char** argv) {
  global_variable::flag_done_collecting = false; 

  std::vector<std::array< double, 16 >> ee_poses; 
  if (argc != 2) {
    std::cerr << "Usage: " << argv[0] << " <robot-hostname>" << std::endl;
    return -1;
  }
  
  InverseKinematics ik_controller(1, IKType::M_P_PSEUDO_INVERSE);
  try {
    franka::Robot robot(argv[1]);
    robotContext::robot = &robot;
    franka::Model model = robot.loadModel();
    robotContext::model = &model;
    int choice{}; 

    std::thread t1(record_pose_thread, 3);

    // Step 1 Addition: Push centroid pose to collected poses to move robot to centroid
    std::array <double, 16> centroid =  {
                                        0.990848, 0.05153214, 0.09326474, 0.0,
                                        0.0585457, -0.99451956, -0.06095708, 0.0,
                                        0.09079327, 0.06359473, -0.99170611, 0.0,
                                        0.47447278, 0.05945557, 0.0886043, 1.0
                                        };
    global_variable::collected_ee_poses.push_back(centroid);

    try{
      robot.control([](const franka::RobotState& robot_state,
                      franka::Duration period) -> franka::Torques {

        franka::Torques output_torques = {0.0, 0.0 ,0.0, 0.0, 0.0, 0.0, 0.0}; 
        global_variable::current_ee_pose =  robot_state.O_T_EE;
        if (global_variable::flag_done_collecting)
          return franka::MotionFinished(output_torques);  
        else
          return output_torques;
      });
    }catch (franka::Exception const& e) {
      std::cout << e.what() << std::endl;
      return -1;
    }
    t1.join();

  std::cout << "Done collecting" << std::endl;

  // for (auto &ee_pose: ee_poses){
  for (auto &ee_pose: global_variable::collected_ee_poses){
    Eigen::Matrix4d pose = Eigen::Matrix4d::Map(ee_pose.data());
    std::cout<<pose<<"\n"; 
    double time = 0.0;
    robot.control([&time, &pose, &ik_controller](const franka::RobotState& robot_state,
                                         franka::Duration period) -> franka::JointVelocities {
      time += period.toSec();
      franka::JointVelocities output_velocities = ik_controller(robot_state, period, pose);
      Eigen::Map<const Eigen::Matrix<double, 7, 1>> output_eigen_velocities(robot_state.dq.data());
      Eigen::Vector3d current_position(robot_state.O_T_EE[12],robot_state.O_T_EE[13],robot_state.O_T_EE[14]); 
      Eigen::Vector3d desired_position(pose(0,3), pose(1,3), pose(2,3)) ;
      double dist = distance(current_position, desired_position);

      if (time >= 15.0 || (output_eigen_velocities.norm() < 0.0005 && dist < 0.0005) ) {
        output_velocities = {0.0, 0.0 ,0.0, 0.0, 0.0, 0.0, 0.0}; 
        return franka::MotionFinished(output_velocities);
      }
      
      return output_velocities;
    });
    std::this_thread::sleep_for(std::chrono::milliseconds(2000));
  }



  } catch (franka::Exception const& e) {
    std::cout << e.what() << std::endl;
    return -1;
  }

  return 0;
}
