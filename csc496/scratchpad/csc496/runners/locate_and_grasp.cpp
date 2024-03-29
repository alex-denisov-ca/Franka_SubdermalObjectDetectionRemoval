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
#include <unistd.h>
#include <Eigen/Dense>
#include <cstdlib>
#include <fstream>
#include <iomanip>
#include <sstream>



#include "examples_common.h"

#include <ncurses.h>
#include <atomic>
#include <thread>

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
std::atomic<bool> captureRequested(false);
std::atomic<bool> stopKeyPressThread(false);
std::atomic<bool> moveForward(false);
std::atomic<bool> moveBackward(false);
std::atomic<bool> moveLeft(false);
std::atomic<bool> moveRight(false);
std::atomic<bool> moveUp(false);
std::atomic<bool> moveDown(false);


void open_gripper() {
  if(robotContext::gripper) {
                    robotContext::gripper->grasp(0.06, 0.08, 30); // Example parameters, adjust accordingly
                    std::cout << "Releasing" << std::endl;
  }
}

void close_gripper() {
  if(robotContext::gripper) {
                    robotContext::gripper->grasp(0.01, 0.08, 30); // Example parameters, adjust accordingly
                    std::cout << "Gripping" << std::endl;
                }
}

void listenForKeyPress() {
    initscr(); // Initialize ncurses mode
    cbreak(); // Disable line buffering
    noecho(); // Don't echo pressed keys to the console
    nodelay(stdscr, TRUE); // Non-blocking getch
    
    int ch;
    while (!stopKeyPressThread.load()) { // Check the flag
        ch = getch(); // Get character
        

        switch (ch) {
            case ' ': captureRequested.store(true); break;
            case 's': moveForward.store(true); break;
            case 'w': moveBackward.store(true); break;
            case 'a': moveLeft.store(true); break;
            case 'd': moveRight.store(true); break;
            case 'r': moveUp.store(true); break;
            case 'f': moveDown.store(true); break;
            case 'g': // Grasp (close) the gripper
                close_gripper();
                break;
            case 't': // Release (open) the gripper
                open_gripper();
                break;
            case 27: stopKeyPressThread.store(true); break;
            default: break;
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(100)); // Reduce CPU usage
    }
    
    endwin(); // End ncurses mode
}

/**
 * @example echo_robot_state.cpp
 * An example showing how to continuously read the robot state.
 */



void set_location(double x_OTEE, double y_OTEE, double z_OTEE){
  double x_offset=0, y_offset=0, z_offset = 0.1;
  double x_desired = x_OTEE + x_offset;
  double y_desired = y_OTEE + y_offset;
  double z_desired = z_OTEE + z_offset;


  double k = 0.01;
  Eigen::Matrix4d transformation;
  Eigen::Matrix4d current_pose;

  std::cout << "Setting location to (" << x_desired<< ", " <<
                                        y_desired<< ", " <<
                                        z_desired<< ")"<< std::endl;

  // Transform voxels to coordinates in eigen
  transformation = Eigen::Matrix4d::Identity();

  transformation << 0, -1, 0, x_desired,
                    1, 0, 0, y_desired,
                    0, 0, -1, z_desired,
                    0, 0, 0, 1;

  transformation.transposeInPlace();


  // store eigen matrix in c array
  std::array<double, 16> result_array;
  for (int i = 0; i < 4; ++i) {
      for (int j = 0; j < 4; ++j) {
          result_array[i * 4 + j] = transformation(i, j);
      }
  }
  
  // TODO: store resulting eigen matrix into current_ee_pose; 
  global_variable::current_ee_pose = result_array;
  global_variable::flag_done_collecting = true; 

}

double distance(const Eigen::Vector3d &v1,const Eigen::Vector3d &v2){
  Eigen::Vector3d diff = v1 - v2;
  return diff.norm();
}

// Function to generate a unique filename based on the current date and time
std::string generateFilename() {
    auto now = std::chrono::system_clock::now();
    auto in_time_t = std::chrono::system_clock::to_time_t(now);

    std::stringstream ss;
    ss << std::put_time(std::localtime(&in_time_t), "%Y-%m-%d_%H-%M-%S");
    return "capture_" + ss.str() + ".txt";
}


int main(int argc, char** argv) {
    global_variable::flag_done_collecting = false; 
    std::thread keyPressThread(listenForKeyPress);
    // Generate a unique filename for this run
    std::string filename = generateFilename();

    // NEW /////
    robotContext::gripper = new franka::Gripper(argv[1]);
    ///////////

  std::vector<std::array< double, 16 >> ee_poses; 
  if (argc != 2) {
    std::cerr << "Usage: " << argv[0] << " <robot-hostname>" << std::endl;
    return -1;
  }



// // Read the current state of the gripper
//         franka::GripperState gripper_state = robotContext::gripper->readOnce();

//         // Print the current width of the gripper
//         std::cout << "Current gripper width: " << gripper_state.width << " m" << std::endl;




    //Step 5 addition: First move the robot to a suitable joint configuration
    franka::Robot robot(argv[1]);
    setDefaultBehavior(robot);
    std::array<double, 7> q_goal = {{0, 0, 0, -3 * M_PI_4, 0, 3 * M_PI_4 , M_PI_4}};
    //SET SPEED HERE//////////////////////////////////////////////////////////////////////////
    double speed_factor = 0.1; //BETWEEN 0 AND 1
    MotionGenerator motion_generator(speed_factor, q_goal); //SPEED WAS 0.15
    std::cout << "WARNING: This example will move the robot! "
              << "Please make sure to have the user stop button at hand!" << std::endl
              << "The robot will now move to the homing position \nPress Enter to continue..." << std::endl;
    std::cin.ignore();
    robot.control(motion_generator);
    std::cout << "Finished moving to initial joint configuration." << std::endl;
  

    InverseKinematics ik_controller(1, IKType::M_P_PSEUDO_INVERSE);
    int point = 1;
    char input;
        // franka::Robot robot(argv[1]);
        robotContext::robot = &robot;
        franka::RobotState state;
        franka::Model model = robot.loadModel();
        robotContext::model = &model;
        int choice{};


    // Set sweeping origin

    set_location(0.521299,-0.0567737,0.0408605);



    Eigen::Matrix4d pose = Eigen::Matrix4d::Map(global_variable::current_ee_pose.data());
    double time = 0.0;
    // Rectangle dimensions
    const double width = 0.1; // meters
    const double height = 0.01; // meters
    int phase = 0; // Current phase of the rectangle sweep
    double phase_start_time = 0.0;
    double phase_length = 3;
    double initial_y = pose(1,3);
    bool isCaptured = false;

  std::cout << "Move the robot hand with keyboard inputs: \n"
  << "w, s, a, d moves the robot hand forwards, backwards, left, right \n"
  << "r, f moves the robot  hand up, down \n"
  << "g, t grasps and releases\n"
  << "ESC quits the program\n" << std::endl;

  std::cout << "Once you're happy with the scope position"
  << "pres the SPACE bar to capture its location"
  << std::endl;

  robot.control([&time, &pose, &ik_controller, &width, &height, &phase, &phase_start_time, &phase_length, &initial_y, &isCaptured, &filename](const franka::RobotState& robot_state,
                                      franka::Duration period) -> franka::JointVelocities {
      time += period.toSec();
      double move_step = 0.005;

      
      if (moveForward.load()) {
            pose(0,3) += move_step;  // Move forward
            moveForward.store(false);
        } else if (moveBackward.load()) {
            pose(0,3) -= move_step;  // Move backward
            moveBackward.store(false);
        } else if (moveLeft.load()) {
            pose(1,3) -= move_step;  // Move left
            moveLeft.store(false);
        } else if (moveRight.load()) {
            pose(1,3) += move_step;  // Move right
            moveRight.store(false);
        } else if (moveDown.load()) {
            pose(2,3) -= move_step;  // Move downs
            moveDown.store(false);
        } else if (moveUp.load()) {
            pose(2,3) += move_step;  // Move up
            moveUp.store(false);
        } 

      // Check if capture was requested
      if (captureRequested.load() && !isCaptured) {
          // Capture current XYZ coordinates
          Eigen::Vector3d current_position(robot_state.O_T_EE[12], robot_state.O_T_EE[13], robot_state.O_T_EE[14]);
          std::cout << "Captured Position: " << current_position.transpose() << std::endl;
          std::cout << "Time: " << time << std::endl;

          // Open file and append the captured position
          std::ofstream outFile;
          outFile.open(filename, std::ios_base::app); // Open in append mode
          outFile << current_position.transpose() << std::endl;
          outFile.close();

            // Reset the request flag
            captureRequested.store(false);
            isCaptured = true;
      }


      franka::JointVelocities output_velocities = ik_controller(robot_state, period, pose);
      Eigen::Map<const Eigen::Matrix<double, 7, 1>> output_eigen_velocities(robot_state.dq.data());
      Eigen::Vector3d current_position(robot_state.O_T_EE[12],robot_state.O_T_EE[13],robot_state.O_T_EE[14]); 
      Eigen::Vector3d desired_position(pose(0,3), pose(1,3), pose(2,3)) ;
      double dist = distance(current_position, desired_position);
      

    //   if (time >= 60.0 || (output_eigen_velocities.norm() < 0.0005 && dist < 0.0005) ) {    
        if (stopKeyPressThread.load()) {    
      output_velocities = {0.0, 0.0 ,0.0, 0.0, 0.0, 0.0, 0.0}; 
      return franka::MotionFinished(output_velocities);
      }
      return output_velocities;

  });

stopKeyPressThread.store(true); // Signal the thread to stop
keyPressThread.join(); // Wait for the thread to finish

  return 0;
}
