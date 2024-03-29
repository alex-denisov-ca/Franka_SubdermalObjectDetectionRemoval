// Copyright (c) 2017 Franka Emika GmbH
// Use of this source code is governed by the Apache-2.0 license, see LICENSE



#include <iostream>

#include "examples_common.h"
#include "ik.h"
#include <Eigen/Dense>
#include <atomic>
#include <chrono>
#include <cmath>
#include <cstdlib>
#include <eigen3/Eigen/Dense>
#include <franka/exception.h>
#include <franka/robot.h>
#include <fstream>
#include <iomanip>
#include <ncurses.h>
#include <sstream>
#include <thread>
#include <unistd.h>
#include <vector>
std::string CAPTURE_FILE = "capture_location.txt";
double TRAY_WIDTH;
double TRAY_HEIGHT;

namespace robotContext {
franka::Robot *robot;
franka::Gripper *gripper;
franka::Model *model;

}

namespace global_variable {
std::array<double, 16> current_ee_pose;
bool flag_done_collecting;
std::vector<std::array<double, 16>> collected_ee_poses;
} 

std::atomic<bool> captureRequested(false);
std::atomic<bool> stopKeyPressThread(false);
std::atomic<bool> moveForward(false);
std::atomic<bool> moveBackward(false);
std::atomic<bool> moveLeft(false);
std::atomic<bool> moveRight(false);
std::atomic<bool> moveUp(false);
std::atomic<bool> moveDown(false);
std::atomic<bool> rotateClockwise(false);
std::atomic<bool> rotateCounterClockwise(false);
const double MIN_ROTATION = 0;      // Example min rotation in radians
const double MAX_ROTATION = M_PI_2; // Example max rotation in radians
const double ROTATION_STEP = (M_PI / 180.0) * 10;

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
    robotContext::gripper->grasp(0.06, 0.08,
                                 30); 
  }
}

void close_gripper() {
  if (robotContext::gripper) {
    robotContext::gripper->grasp(0.01, 0.08,
                                 30); 
  }
}


void listenForKeyPress() {
  initscr();             // Initialize ncurses mode
  cbreak();              // Disable line buffering
  noecho();              // Don't echo pressed keys to the console
  nodelay(stdscr, TRUE); // Non-blocking getch

  int ch;
  while (!stopKeyPressThread.load()) { // Check the flag
    ch = getch();                      // Get character

    switch (ch) {
    case ' ':
      captureRequested.store(true);
      break;
    case 's':
      moveForward.store(true);
      break;
    case 'w':
      moveBackward.store(true);
      break;
    case 'a':
      moveLeft.store(true);
      break;
    case 'd':
      moveRight.store(true);
      break;
    case 'r':
      moveUp.store(true);
      break;
    case 'f':
      moveDown.store(true);
      break;
    case 'g': // Grasp (close) the gripper
      close_gripper();
      break;
    case 't': // Release (open) the gripper
      open_gripper();
      break;
    case 'j': // rotate counter-clockwise
      rotateCounterClockwise.store(true);
      break;
    case 'k': // rotate clockwise
      rotateClockwise.store(true);
      break;
    case 27:
      stopKeyPressThread.store(true);
      break;
    default:
      break;
    }
    std::this_thread::sleep_for(
        std::chrono::milliseconds(100)); // Reduce CPU usage
  }

  endwin(); // End ncurses mode
}


void set_location(double x_OTEE, double y_OTEE, double z_OTEE) {
  double x_offset = 0, y_offset = 0, z_offset = 0.1;
  double x_desired = x_OTEE + x_offset;
  double y_desired = y_OTEE + y_offset;
  double z_desired = z_OTEE + z_offset;

  double k = 0.01;
  Eigen::Matrix4d transformation;
  Eigen::Matrix4d current_pose;

  std::cout << "Setting location to (" << x_desired << ", " << y_desired << ", "
            << z_desired << ")" << std::endl;

  // store eigen matrix in c array
  std::array<double, 16> result_array = {
    0, 1, 0, 0,
    -1, 0, 0, 0,
    0, 0, -1, 0,
    x_desired, y_desired, z_desired, 1
  };

  global_variable::current_ee_pose = result_array;
  global_variable::flag_done_collecting = true;
}

double distance(const Eigen::Vector3d &v1, const Eigen::Vector3d &v2) {
  Eigen::Vector3d diff = v1 - v2;
  return diff.norm();
}

int main(int argc, char **argv) {
  std::vector<std::array<double, 16>> ee_poses;
  if (argc != 5) {
    std::cerr << "Usage: " << argv[0] << " <robot-hostname> <x> <y> <z>" << std::endl;
    return -1;
  }
  global_variable::flag_done_collecting = false;

  franka::Robot robot(argv[1]);
  robotContext::gripper = new franka::Gripper(argv[1]);
  double x = 0.0, y=0.0, z=0.0;

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

  InverseKinematics ik_controller(1, IKType::M_P_PSEUDO_INVERSE);
  int point = 1;
  char input;
  robotContext::robot = &robot;
  franka::RobotState state;
  franka::Model model = robot.loadModel();
  robotContext::model = &model;
  int choice{};

  // Set sweeping origin
  set_location(x, y, z);

  Eigen::Matrix4d pose =
      Eigen::Matrix4d::Map(global_variable::current_ee_pose.data());
  double time = 0.0;
  int phase = 0;              // Current phase of the rectangle sweep
  double phase_start_time = 0.0;
  double phase_length = 3;
  double initial_y = pose(1, 3);
  bool isCaptured = false;


  std::cout<< "To capture the position of the object, move the robot with keyboard inputs. Here are the key mappings:\n"
  "w: forwards \n"
  "s: backwards \n"
  "a: left \n"
  "d: right \n"
  "r: up \n"
  "f: down \n"
  "SPACEBAR: capture object location (don't get too close, the height does not matter)\n"
  "ESC: quit object capturing\n"
  "\n Make sure probe is directly above object when capturing, press enter to continue ...\n"
            << std::endl;
  std::cin.ignore();

  std::thread keyPressThread(listenForKeyPress);

  robot.control([&time, &pose, &ik_controller, &phase,
                 &phase_start_time, &phase_length, &initial_y, &isCaptured](
                    const franka::RobotState &robot_state,
                    franka::Duration period) -> franka::JointVelocities {
    time += period.toSec();
    double move_step = 0.005;

    if (moveForward.load()) {
      pose(0, 3) += move_step; // Move forward
      moveForward.store(false);
    } else if (moveBackward.load()) {
      pose(0, 3) -= move_step; // Move backward
      moveBackward.store(false);
    } else if (moveLeft.load()) {
      pose(1, 3) -= move_step; // Move left
      moveLeft.store(false);
    } else if (moveRight.load()) {
      pose(1, 3) += move_step; // Move right
      moveRight.store(false);
    } else if (moveDown.load()) {
      pose(2, 3) -= move_step; // Move downs
      moveDown.store(false);
    } else if (moveUp.load()) {
      pose(2, 3) += move_step; // Move up
      moveUp.store(false);
    } else if (rotateCounterClockwise.load()) {
      Eigen::Matrix3d rotationMatrix =
          Eigen::AngleAxisd(ROTATION_STEP, Eigen::Vector3d::UnitZ())
              .toRotationMatrix();
      Eigen::Matrix4d poseMatrix = Eigen::Matrix4d::Map(pose.data());
      Eigen::Matrix3d currentRotation = poseMatrix.block<3, 3>(0, 0);
      Eigen::Matrix3d newRotation = rotationMatrix * currentRotation;
      poseMatrix.block<3, 3>(0, 0) = newRotation;
      Eigen::Map<Eigen::Matrix<double, 16, 1>>(pose.data()) =
          Eigen::Matrix<double, 16, 1>::Map(poseMatrix.data(), 16);
      rotateCounterClockwise.store(false);
    } else if (rotateClockwise.load()) {
      Eigen::Matrix3d rotationMatrix =
          Eigen::AngleAxisd(-ROTATION_STEP, Eigen::Vector3d::UnitZ())
              .toRotationMatrix();
      Eigen::Matrix4d poseMatrix = Eigen::Matrix4d::Map(pose.data());
      Eigen::Matrix3d currentRotation = poseMatrix.block<3, 3>(0, 0);
      Eigen::Matrix3d newRotation = rotationMatrix * currentRotation;
      poseMatrix.block<3, 3>(0, 0) = newRotation;
      Eigen::Map<Eigen::Matrix<double, 16, 1>>(pose.data()) =
          Eigen::Matrix<double, 16, 1>::Map(poseMatrix.data(), 16);
      rotateClockwise.store(false);
    }

    // Check if capture was requested
    if (captureRequested.load() && !isCaptured) {
      // Capture current XYZ coordinates
      Eigen::Vector3d current_position(robot_state.O_T_EE[12],
                                       robot_state.O_T_EE[13],
                                       robot_state.O_T_EE[14]);
      std::cout << "Captured Position: " << current_position.transpose()
                << std::endl;
      std::cout << "Time: " << time << std::endl;

      // Open file and append the captured position
      std::ofstream outFile;
      outFile.open(CAPTURE_FILE, std::ios::out); // Open in write mode
      outFile << pose(12) << " " << pose(13) << " " << pose(14) << std::endl;
      outFile.close();

      // Reset the request flag
      captureRequested.store(false);
      isCaptured = true;
    }

    franka::JointVelocities output_velocities =
        ik_controller(robot_state, period, pose);
    Eigen::Map<const Eigen::Matrix<double, 7, 1>> output_eigen_velocities(
        robot_state.dq.data());
    Eigen::Vector3d current_position(
        robot_state.O_T_EE[12], robot_state.O_T_EE[13], robot_state.O_T_EE[14]);
    Eigen::Vector3d desired_position(pose(0, 3), pose(1, 3), pose(2, 3));
    double dist = distance(current_position, desired_position);

    if (stopKeyPressThread.load()) {
      output_velocities = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
      return franka::MotionFinished(output_velocities);
    }
    return output_velocities;
  });

  stopKeyPressThread.store(true); // Signal the thread to stop
  keyPressThread.join();          // Wait for the thread to finish

  return 0;
}
