// Copyright (c) 2017 Franka Emika GmbH
// Use of this source code is governed by the Apache-2.0 license, see LICENSE

//////////
// Step 5: generate voxel pose
// How to run: 
// 1. ensure filepath: /home/teachinglab_student/csc496/scratchpad/csc496/runners/generate_voxel_pose.cpp
// 2. run `cmake .. && make` in the build directory to create runner
//////////

#include <iostream>

#include <franka/exception.h>
#include <franka/robot.h>
#include <vector>
#include <eigen3/Eigen/Dense>
#include <thread>
#include "ik.h"
#include <chrono>
#include <Eigen/Dense>

#include "examples_common.h"
 

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

// Step 5: Collect voxel values and convert to physical space coordinates
void get_voxels(){
  int collected_poses = 0; 
  double x, y, z;
  // Eigen::Vector4d voxels;
  Eigen::Vector3d origin_baseframe;
  origin_baseframe << 0.645268, -0.143272, -0.00506412;
  Eigen::Matrix4d transformation;
  Eigen::Matrix4d current_pose;

  // Prompt user for voxel values
  std::cout << "Enter voxel values (decimals are valid)" << std::endl;
  std::cout << "Enter the value for x: ";
  std::cin >> x;
  std::cout << "Enter the value for y: ";
  std::cin >> y;
  std::cout << "Enter the value for z: ";
  std::cin >> z;
  std::cout << "Your input: x = " << x << ", y = " << y << ", z = " << z << std::endl;

  // Transform voxels to coordinates in eigen
  // voxels << x, y, z, 1;
  transformation = Eigen::Matrix4d::Identity();
  double k = 0.8 * 0.01; 
  // transformation << 0, -1, 0, origin_baseframe(0)+ -1 * x * k ,
  //                   1, 0, 0, origin_baseframe(1) + y * k,
  //                   0, 0, -1, origin_baseframe(2) + z * k,
  //                   0, 0, 0, 1;
  double x_baseframe = -y;
  double y_baseframe = x;
  transformation << 0, -1, 0, origin_baseframe(0)+ x_baseframe * k ,
                    1, 0, 0, origin_baseframe(1) + y_baseframe * k,
                    0, 0, -1, origin_baseframe(2) + z * k,
                    0, 0, 0, 1;
  // std::cout <<"transformation:\n" <<transformation << std::endl << std::endl;
  transformation.transposeInPlace();


  // store eigen matrix in c array
  std::array<double, 16> result_array;
  for (int i = 0; i < 4; ++i) {
      for (int j = 0; j < 4; ++j) {
          result_array[i * 4 + j] = transformation(i, j);
          std::cout << result_array[i * 4 + j] << "  ";
      }
    std::cout << std::endl;
  }
  
  // TODO: store resulting eigen matrix into current_ee_pose; 
  std::cout << std::endl;
  global_variable::collected_ee_poses.push_back(result_array);
  collected_poses++;
  
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

    //Step 5 addition: First move the robot to a suitable joint configuration
    franka::Robot robot(argv[1]);
    setDefaultBehavior(robot);
    std::array<double, 7> q_goal = {{0, 0, 0, -3 * M_PI_4, 0, 3 * M_PI_4 , M_PI_4}};
    MotionGenerator motion_generator(0.15, q_goal);
    std::cout << "WARNING: This example will move the robot! "
              << "Please make sure to have the user stop button at hand!" << std::endl
              << "The robot will now move to the homing position \nPress Enter to continue..." << std::endl;
    std::cin.ignore();
    robot.control(motion_generator);
    std::cout << "Finished moving to initial joint configuration." << std::endl;
  
  InverseKinematics ik_controller(1, IKType::M_P_PSEUDO_INVERSE);
  try {
    franka::Robot robot(argv[1]);
    robotContext::robot = &robot;
    franka::Model model = robot.loadModel();
    robotContext::model = &model;
    int choice{}; 

    // Prompt user for voxel values
    // std::thread t1(get_voxels);
    get_voxels();


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