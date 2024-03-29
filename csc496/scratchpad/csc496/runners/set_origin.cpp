// Copyright (c) 2017 Franka Emika GmbH
// Use of this source code is governed by the Apache-2.0 license, see LICENSE
#include <iostream>

#include "ik.h"
#include "examples_common.h"
#include <chrono>
#include <eigen3/Eigen/Dense>
#include <franka/exception.h>
#include <franka/robot.h>
#include <sstream>
#include <fstream>
#include <thread>
#include <vector>
std::string ORIGIN_FILE = "origin.txt";

/**
 * @example echo_robot_state.cpp
 * An example showing how to continuously read the robot state.
 */

namespace robotContext {
franka::Robot *robot;
franka::Gripper *gripper;
franka::Model *model;

} // namespace robotContext

namespace global_variable {
std::array<double, 16> current_ee_pose;
bool flag_done_collecting;
std::vector<std::array<double, 16>> collected_ee_poses;
} // namespace global_variable

void home_gripper() {
  if (robotContext::gripper) {
    robotContext::gripper->move(0.01, 0.08); // Example parameters, adjust accordingly
    // std::cout << "Gripping" << std::endl;
  }
}

void record_pose_thread(int n_poses = 3) {
  int collected_poses = 0;
  std::string my_string = "";

  while (collected_poses < n_poses) {
    std::cout << "Move probe so it is directly above the origin, then press ENTER to collect the location, anything else to quit"
              << std::endl;
    std::getline(std::cin, my_string);
    if (my_string.length() == 0) {
      global_variable::collected_ee_poses.push_back(
          global_variable::current_ee_pose);
      collected_poses++;
    } else {
      std::cout << "Exiting data collection" << std::endl;
      global_variable::flag_done_collecting = true;
      break;
    }
  }
  global_variable::flag_done_collecting = true;
}

double distance(const Eigen::Vector3d &v1, const Eigen::Vector3d &v2) {
  Eigen::Vector3d diff = v1 - v2;
  return diff.norm();
}

int main(int argc, char **argv) {
  std::vector<std::array<double, 16>> ee_poses;
  if (argc != 2) {
    std::cerr << "Usage: " << argv[0] << " <robot-hostname>" << std::endl;
    return -1;
  }
  global_variable::flag_done_collecting = false;
  franka::Robot robot(argv[1]);
  robotContext::robot = &robot;
robotContext::gripper = new franka::Gripper(argv[1]);


  // Home robot

  setDefaultBehavior(robot);
  std::array<double, 7> q_goal = {
      {0, 0, 0, -3 * M_PI_4, 0, 3 * M_PI_4, M_PI_4}};
  MotionGenerator motion_generator(0.15, q_goal);
  std::cout << "The robot will now move to the homing position, press Enter "
               "to continue..."
            << std::endl;
  std::cin.ignore();
  robot.control(motion_generator);
  home_gripper();



  InverseKinematics ik_controller(1, IKType::M_P_PSEUDO_INVERSE);
  try {
    franka::Robot robot(argv[1]);
    robotContext::robot = &robot;
    franka::Model model = robot.loadModel();
    robotContext::model = &model;
    int choice{};

    std::thread t1(record_pose_thread, 1);

    try {
      robot.control([](const franka::RobotState &robot_state,
                       franka::Duration period) -> franka::Torques {
        franka::Torques output_torques = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
        global_variable::current_ee_pose = robot_state.O_T_EE;
        if (global_variable::flag_done_collecting)
          return franka::MotionFinished(output_torques);
        else
          return output_torques;
      });
    } catch (franka::Exception const &e) {
      std::cout << e.what() << std::endl;
      return -1;
    }
    t1.join();

    Eigen::Matrix4d pose =
        Eigen::Matrix4d::Map(global_variable::current_ee_pose.data());
    // std::cout << pose << "\n";
    // std::cout << "Done collecting, writing to " << ORIGIN_FILE << std::endl;
    std::ofstream outFile;
    outFile.open(ORIGIN_FILE, std::ios::out);
    outFile << pose(12) << " " << pose(13) << " " << pose(14) << std::endl;
    outFile.close();

    // std::this_thread::sleep_for(std::chrono::milliseconds(2000));

  } catch (franka::Exception const &e) {
    std::cout << e.what() << std::endl;
    return -1;
  }

  return 0;
}
