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

#include "examples_common.h"

#include <ncurses.h>
#include <atomic>
#include <thread>

std::atomic<bool> captureRequested(false);

void listenForKeyPress() {
    initscr(); // Initialize ncurses mode
    cbreak(); // Disable line buffering
    noecho(); // Don't echo pressed keys to the console
    nodelay(stdscr, TRUE); // Non-blocking getch
    
    int ch;
    while (true) {
        ch = getch(); // Get character
        if (ch == ' ') { // Space bar ASCII code
            captureRequested.store(true);
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(100)); // Reduce CPU usage
    }
    
    endwin(); // End ncurses mode
}

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
  double x, y;
  double z = 0.1;
  double k = 0.01;
  // Eigen::Vector4d voxels;
  Eigen::Vector3d origin_baseframe;
  origin_baseframe << 0.521299,-0.0567737,0.0408605;
  Eigen::Matrix4d transformation;
  Eigen::Matrix4d current_pose;

  // Prompt user for x, y voxel values
  std::cout << "Enter voxel values (decimals are valid)" << std::endl;
  std::cout << "Enter the value for x: ";
  std::cin >> x;
  std::cout << "Enter the value for y: ";
  std::cin >> y;
  std::cout << "Moving to (" << origin_baseframe(0)+ x * k << ", " <<
                 origin_baseframe(1) + y *k << ", " <<
                  origin_baseframe(2) + z << ")"<< std::endl;


  // Transform voxels to coordinates in eigen
  transformation = Eigen::Matrix4d::Identity();
  
  z = 0.1;
  double x_baseframe = -y;
  double y_baseframe = x;

  transformation << 0, -1, 0, origin_baseframe(0)+ x * k,
                    1, 0, 0, origin_baseframe(1) + y *k,
                    0, 0, -1, origin_baseframe(2) + z,
                    0, 0, 0, 1;
  // std::cout <<"transformation:\n" <<transformation << std::endl << std::endl;
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


int main(int argc, char** argv) {
  global_variable::flag_done_collecting = false; 
  std::thread keyPressThread(listenForKeyPress);

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
    int point = 1;
    char input;
        // franka::Robot robot(argv[1]);
        robotContext::robot = &robot;
        franka::RobotState state;
        franka::Model model = robot.loadModel();
        robotContext::model = &model;
        int choice{};


    // Prompt user for voxel values
    std::cout << "\n################\n" <<
                "To move to point " << point << std::endl;
    get_voxels();



    Eigen::Matrix4d pose = Eigen::Matrix4d::Map(global_variable::current_ee_pose.data());
    double time = 0.0;
    // Rectangle dimensions
    const double width = 0.1; // meters
    const double height = 0.01; // meters
    int phase = 0; // Current phase of the rectangle sweep
    double phase_start_time = 0.0;
    double initial_y = pose(1,3);



  // store eigen matrix in c array
    std::array<double, 16> result_array;
    for (int i = 0; i < 4; ++i) {
        robot.control([&time, &pose, &ik_controller, &width, &height, &phase, &phase_start_time, &initial_y](const franka::RobotState& robot_state,
                                            franka::Duration period) -> franka::JointVelocities {
            time += period.toSec();

            // Update target position based on the current phase of the rectangle sweep
            if (time - phase_start_time >= 1) { // Switch phase every X seconds as an example
                phase = (phase + 1) % 4;
                phase_start_time = time;

      

                switch (phase) {
                    case 0: pose(0,3) += width; break; // Move in +X
                    case 1: pose(1,3) += height; break; // Move in +Y
                    case 2: pose(0,3) -= width; break; // Move in -X
                    case 3: pose(1,3) += height; break; // Move back to start in -X
                }
            }

            // Check if capture was requested
        if (captureRequested.load()) {
            // Capture current XYZ coordinates
            Eigen::Vector3d current_position(robot_state.O_T_EE[12], robot_state.O_T_EE[13], robot_state.O_T_EE[14]);
            std::cout << "Captured Position: " << current_position.transpose() << std::endl;

            // Reset the request flag
            captureRequested.store(false);
        }


            franka::JointVelocities output_velocities = ik_controller(robot_state, period, pose);
            Eigen::Map<const Eigen::Matrix<double, 7, 1>> output_eigen_velocities(robot_state.dq.data());
            Eigen::Vector3d current_position(robot_state.O_T_EE[12],robot_state.O_T_EE[13],robot_state.O_T_EE[14]); 
            Eigen::Vector3d desired_position(pose(0,3), pose(1,3), pose(2,3)) ;
            double dist = distance(current_position, desired_position);

            //double curr_width =abs(pose(1,3) - initial_y);
            if ((abs(pose(1,3) - initial_y) >= 0.12) || time >= 30.0 || (output_eigen_velocities.norm() < 0.0005 && dist < 0.0005) ) {
            output_velocities = {0.0, 0.0 ,0.0, 0.0, 0.0, 0.0, 0.0}; 
            // for (int i =0; i < 1000; i++) {i;}
            // robotContext::robot->stop();
            return franka::MotionFinished(output_velocities);
            }
            return output_velocities;

        });


    }

  keyPressThread.join();

  return 0;
}