// Copyright (c) 2017 Franka Emika GmbH
// Use of this source code is governed by the Apache-2.0 license, see LICENSE

#include <iostream>

#include <franka/exception.h>
#include <franka/robot.h>
#include <vector>
#include <eigen3/Eigen/Dense>
#include <thread>
// Custom inverse kinematics and utilities
#include "ik.h"
#include <chrono>
#include <unistd.h>
#include <Eigen/Dense>
#include <cstdlib>
#include <fstream>
#include <iomanip>
#include <sstream>


#include "examples_common.h"

// ncurses for listening to keyboard inputs
#include <ncurses.h>
// Atomic operations for flag management
#include <atomic>
#include <thread>
std::string ROUGH_CAPTURE_FILE = "rough_capture_location.txt";
double TRAY_WIDTH = 0.12;

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

// Flags for capture request and keyboard thread management
std::atomic<bool> captureRequested(false);
std::atomic<bool> stopKeyPressThread(false);


void listenForKeyPress() {
    initscr(); // Initialize ncurses mode
    cbreak(); // Disable line buffering
    noecho(); // Don't echo pressed keys to the console
    nodelay(stdscr, TRUE); // Non-blocking getch
    
    int ch;
    while (!stopKeyPressThread.load()) { // Check the flag
        ch = getch(); // Get character
        if (ch == ' ') { // Space bar ASCII code
            captureRequested.store(true);
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(100)); // Reduce CPU usage
    }
    
    endwin(); // End ncurses mode
}

// Calculate and set the origin based on the given offset
void set_origin(double x_OTEE, double y_OTEE, double z_OTEE) {
  double x_offset = 0, y_offset = 0, z_offset = 0.03;
  double x_desired = x_OTEE + x_offset;
  double y_desired = y_OTEE + y_offset;
  double z_desired = z_OTEE + z_offset;

// Define the transformation matrix for setting the origin
  std::array<double, 16> result_array = {
    0, 1, 0, 0,
    -1, 0, 0, 0,
    0, 0, -1, 0,
    x_desired, y_desired, z_desired, 1
  };
   
  global_variable::current_ee_pose = result_array;
  global_variable::flag_done_collecting = true; 

}

double distance(const Eigen::Vector3d &v1,const Eigen::Vector3d &v2){
  Eigen::Vector3d diff = v1 - v2;
  return diff.norm();
}

// Main function to initialize the robot and execute the scanning sequence
int main(int argc, char** argv) {
  global_variable::flag_done_collecting = false; 
  std::vector<std::array< double, 16 >> ee_poses; 
  if (argc != 5) {
    std::cerr << "Usage: " << argv[0] << " <robot-hostname> <x> <y> <z>" << std::endl;
    return -1;
  }

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

// Init robot

franka::Robot robot(argv[1]);
robotContext::robot = &robot;
franka::RobotState state;
franka::Model model = robot.loadModel();
robotContext::model = &model;


    // Home robot
    setDefaultBehavior(robot);
    std::array<double, 7> q_goal = {{0, 0, 0, -3 * M_PI_4, 0, 3 * M_PI_4 , M_PI_4}};

    double speed_factor = 0.1;
    MotionGenerator motion_generator(speed_factor, q_goal); 
    std::cout << "The robot will now move to the homing position, press Enter to continue..." << std::endl;
    std::cin.ignore();
    robot.control(motion_generator);
  

    InverseKinematics ik_controller(1, IKType::M_P_PSEUDO_INVERSE);
    int point = 1;
    char input;
    int choice{};


    // Set sweeping origin
    set_origin(x, y, z);



    Eigen::Matrix4d pose = Eigen::Matrix4d::Map(global_variable::current_ee_pose.data());
    double time = 0.0;
    // Sweep dimensions
    const double width = 0.09; // meters
    const double height = 0.01; // meters
    int phase = 0; // Current phase of the zig-zag sweep
    double phase_start_time = 0.0;
    double phase_length = 3;
    double initial_y = pose(1,3);
    bool isDone = false;
    bool isCaptured = false;

  std::cout << "The autoscan sequence will now initiate. Press the space bar when the probe is directly above the object.\nPress enter to continue ...\n" << std::endl;
  std::cin.ignore();


  std::thread keyPressThread(listenForKeyPress);

  robot.control([&time, &pose, &ik_controller, &width, &height, &phase, &phase_start_time, &phase_length, &initial_y, &isDone , &isCaptured](const franka::RobotState& robot_state,
                                      franka::Duration period) -> franka::JointVelocities {
      time += period.toSec();

      // Update target position based on the current phase of the zig-zag sweeps
      if (!isDone && time - phase_start_time >= phase_length) { // Switch phase every <phase_length> seconds as an example
          phase = (phase + 1) % 4;
          phase_start_time = time;
          int slow_down_factor = 3;

          switch (phase) {
              case 0: pose(0,3) += width; phase_length *= slow_down_factor; break; // Move in +X
              case 1: pose(1,3) += height; phase_length /= slow_down_factor; break; // Move in +Y
              case 2: pose(0,3) -= width; phase_length *= slow_down_factor; break; // Move in -X
              case 3: pose(1,3) += height; phase_length /= slow_down_factor; break; // Move again in +Y, completing zig zag
          }
      }

      // Check if capture was requested
      if (captureRequested.load() && !isCaptured) {
          // Capture current XYZ coordinates
          Eigen::Vector3d current_position(robot_state.O_T_EE[12], robot_state.O_T_EE[13], robot_state.O_T_EE[14]);
          std::cout << "Captured Position: " << current_position.transpose() << std::endl;
          std::cout << "Time: " << time << std::endl;

          // Open file and append the captured position
          std::ofstream outFile;
          outFile.open(ROUGH_CAPTURE_FILE, std::ios::out); // Open in write mode
          outFile << pose(12) << " " << pose(13) << " " << pose(14) << std::endl;
          outFile.close();

            // Reset the request flag
            captureRequested.store(false);
          isCaptured = true;
      }

// Calculate joint velocities required to move to the target pose
      franka::JointVelocities output_velocities = ik_controller(robot_state, period, pose);
      Eigen::Map<const Eigen::Matrix<double, 7, 1>> output_eigen_velocities(robot_state.dq.data());
      Eigen::Vector3d current_position(robot_state.O_T_EE[12],robot_state.O_T_EE[13],robot_state.O_T_EE[14]); 
      Eigen::Vector3d desired_position(pose(0,3), pose(1,3), pose(2,3)) ;
      double dist = distance(current_position, desired_position);

      // If sweep reaches desired y position (if sweep is complete)
      if ((abs(pose(1,3) - initial_y) >= TRAY_WIDTH)) { 
        isDone = true;
      }

// Halt if the maximum time is exceeded or the robot has stopped moving
      if (time >= 60.0 || (output_eigen_velocities.norm() < 0.0005 && dist < 0.0005) ) {    
      output_velocities = {0.0, 0.0 ,0.0, 0.0, 0.0, 0.0, 0.0}; 
      return franka::MotionFinished(output_velocities);
      }
      return output_velocities;

  });

stopKeyPressThread.store(true); // Signal the thread to stop
keyPressThread.join(); // Wait for the thread to finish

  return 0;
}
