// Copyright (c) 2017 Franka Emika GmbH
// Use of this source code is governed by the Apache-2.0 license, see LICENSE

#include <iostream>

#include "ik.h"
#include <Eigen/Dense>
#include <chrono>
#include <cmath>
#include <eigen3/Eigen/Dense>
#include <franka/exception.h>
#include <franka/robot.h>
#include <limits>
#include <thread>
#include <vector>

#include "examples_common.h"
double Z_BOARD = -0.001; // Board height constant

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

template <typename T>
T getInputWithinBounds(const T lowerBound, const T upperBound,
                       const std::string &prompt) {
  T inputValue;
  while (true) {
    std::cout << prompt;
    std::cin >> inputValue;
    if (std::cin.fail() || inputValue < lowerBound || inputValue > upperBound) {
      std::cin.clear(); // Clear the error flag
      std::cin.ignore(std::numeric_limits<std::streamsize>::max(),
                      '\n'); // Discard the input
      std::cout << "Input must be between " << lowerBound << " and "
                << upperBound << ". Please try again.\n";
    } else {
      break; // Input is within bounds, exit the loop
    }
  }
  return inputValue;
}

void open_gripper() {
  if (robotContext::gripper) {
    robotContext::gripper->move(0.06, 0.08);
  }
}

void close_gripper() {
  if (robotContext::gripper) {
    robotContext::gripper->grasp(0.01, 0.08, 30);
  }
}

// set target location for watching and gripping an object
void set_location(double x_raw, double y_raw) {
  double x_offset = 0.06, y_offset = 0, z_offset = 0.1;
  double x_desired = x_raw + x_offset;
  double y_desired = y_raw + y_offset;
  double z_high = Z_BOARD + z_offset;

  std::cout << "Watch object pose set as: (" << x_desired << ", " << y_desired
            << ", " << z_high << ")" << std::endl;

  std::array<double, 16> pose_watch_object = {
      0, 1, 0, 0, -1, 0, 0, 0, 0, 0, -1, 0, x_desired, y_desired, z_high, 1};
  global_variable::collected_ee_poses.push_back(pose_watch_object);

  std::cout << "Grip object pose set as: (" << x_desired << ", " << y_desired
            << ", " << Z_BOARD << ")" << std::endl;

  std::array<double, 16> pose_grip_object = {
      0, 1, 0, 0, -1, 0, 0, 0, 0, 0, -1, 0, x_desired, y_desired, Z_BOARD, 1};
  global_variable::collected_ee_poses.push_back(pose_grip_object);
  global_variable::flag_done_collecting = true;
}

double distance(const Eigen::Vector3d &v1, const Eigen::Vector3d &v2) {
  Eigen::Vector3d diff = v1 - v2;
  return diff.norm();
}

int main(int argc, char **argv) {
  // Ensure correct command line arguments are provided
  if (argc != 5) {
    std::cerr << "Usage: " << argv[0] << " <robot-hostname> <x> <y> <z>"
              << std::endl;
    return -1;
  }

  global_variable::flag_done_collecting = false;
  // Initialize the robot and gripper with the provided hostname
  franka::Robot robot(argv[1]);
  robotContext::gripper = new franka::Gripper(argv[1]);

  double x = 0.0, y = 0.0, z = 0.0;

  try {
    // Parse x, y, and z values from command line arguments
    x = std::stod(argv[2]);
    y = std::stod(argv[3]);
    z = std::stod(argv[4]);

  } catch (const std::invalid_argument &e) {
    std::cerr << "Invalid number: " << e.what() << std::endl;
    return 1;
  } catch (const std::out_of_range &e) {
    std::cerr << "Number out of range: " << e.what() << std::endl;
    return 1;
  }

  std::vector<std::array<double, 16>> ee_poses;

  // Home robot and prompt user to remove probe
  setDefaultBehavior(robot);
  std::array<double, 7> q_goal = {
      {0, 0, 0, -3 * M_PI_4, 0, 3 * M_PI_4, M_PI_4}};
  MotionGenerator motion_generator(0.15, q_goal);
  std::cout << "Please remove the probe. \nThe robot will then move to the "
               "homing position press Enter "
               "to continue..."
            << std::endl;
  std::cin.ignore();
  robot.control(motion_generator);

  InverseKinematics ik_controller(1, IKType::M_P_PSEUDO_INVERSE);
  int point = 1;
  char input;
  robotContext::robot = &robot;
  franka::Model model = robot.loadModel();
  robotContext::model = &model;

  // Prompt user for voxel values
  set_location(x, y);

  std::cout << "The robot will now grasp the object, press Enter to continue..."
            << std::endl;
  std::cin.ignore();

  // Open grippers before moving to the object
  open_gripper();

  // Move robot above object, then descend
  for (auto &ee_pose : global_variable::collected_ee_poses) {
    Eigen::Matrix4d pose = Eigen::Matrix4d::Map(ee_pose.data());
    double time = 0.0;
    robot.control([&time, &pose, &ik_controller](
                      const franka::RobotState &robot_state,
                      franka::Duration period) -> franka::JointVelocities {
      time += period.toSec();
      franka::JointVelocities output_velocities =
          ik_controller(robot_state, period, pose);
      Eigen::Map<const Eigen::Matrix<double, 7, 1>> output_eigen_velocities(
          robot_state.dq.data());
      Eigen::Vector3d current_position(robot_state.O_T_EE[12],
                                       robot_state.O_T_EE[13],
                                       robot_state.O_T_EE[14]);
      Eigen::Vector3d desired_position(pose(0, 3), pose(1, 3), pose(2, 3));
      double dist = distance(current_position, desired_position);

      if (time >= 15.0 ||
          (output_eigen_velocities.norm() < 0.0005 && dist < 0.0005)) {
        output_velocities = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
        return franka::MotionFinished(output_velocities);
      }

      return output_velocities;
    });
    std::this_thread::sleep_for(std::chrono::milliseconds(500));
  }
  // Grip object
  close_gripper();

  // Home robot again
  setDefaultBehavior(robot);
  q_goal = {{0, 0, 0, -3 * M_PI_4, 0, 3 * M_PI_4, M_PI_4}};

  std::cout << "The robot will now move to the homing position, press Enter "
               "to continue..."
            << std::endl;
  std::cin.ignore();
  robot.control(motion_generator);

  // Open the gripper to release any grasped objects
  std::cout
      << "Grippers will now open. Catch any objects as necessary. Press Enter "
         "to continue..."
      << std::endl;
  std::cin.ignore();
  open_gripper();

  return 0;
}
