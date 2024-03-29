// Franka Emika Robot State Echo Example
// Demonstrates continuous robot state reading, using the Franka Emika robot API.
#include <iostream>
#include <chrono>
#include <eigen3/Eigen/Dense>
#include <franka/exception.h>
#include <franka/robot.h>
#include <sstream>
#include <fstream>
#include <thread>
#include <vector>

#include "ik.h" // Inverse Kinematics
#include "examples_common.h"

std::string const ORIGIN_FILE = "origin.txt"; // File to store the origin position.

// Namespace to hold the robot's operational context.
namespace robot_environment
{
    franka::Robot *robot = nullptr;
    franka::Gripper *gripper = nullptr;
    franka::Model *model = nullptr;
}

// Global variables to track the robot's end-effector pose and collection status.
namespace robot_state_tracking
{
    std::array<double, 16> current_end_effector_pose;
    bool is_collection_complete = false;
    std::vector<std::array<double, 16>> collected_poses;
}

// Initializes and homes the robot's gripper.
void initialize_gripper()
{
    if (robot_environment::gripper)
    {
        robot_environment::gripper->move(0.01, 0.08); // Parameters represent target width and speed.
    }
}

// Collects a predefined number of end-effector poses after user confirmation.
void collect_end_effector_poses(unsigned int number_of_poses = 3)
{
    unsigned int poses_collected = 0;
    std::string user_input;

    while (poses_collected < number_of_poses)
    {
        std::cout << "Position the probe directly above the origin, then press ENTER to save the location, or type anything else to exit." << std::endl;
        std::getline(std::cin, user_input);
        if (user_input.empty())
        {
            robot_state_tracking::collected_poses.push_back(robot_state_tracking::current_end_effector_pose);
            poses_collected++;
        }
        else
        {
            std::cout << "Data collection terminated." << std::endl;
            robot_state_tracking::is_collection_complete = true;
            break;
        }
    }
    robot_state_tracking::is_collection_complete = true;
}



// Main application entry point.
int main(int argc, char **argv)
{
    if (argc != 2)
    {
        std::cerr << "Usage: " << argv[0] << " <robot-hostname>" << std::endl;
        return -1;
    }

    try
    {
        robot_environment::robot = new franka::Robot(argv[1]);
        robot_environment::gripper = new franka::Gripper(argv[1]);
        robot_environment::model = &robot_environment::robot->loadModel();

        setDefaultBehavior(*robot_environment::robot);
        std::array<double, 7> home_position = {{0, 0, 0, -3 * M_PI_4, 0, 3 * M_PI_4, M_PI_4}};
        MotionGenerator move_to_home(0.15, home_position);

        std::cout << "Moving the robot to the home position. Press Enter to proceed." << std::endl;
        std::cin.ignore();
        robot_environment::robot->control(move_to_home);
        initialize_gripper();

        std::thread pose_collection_thread(collect_end_effector_poses, 1);
        try
        {
            robot.control([](const franka::RobotState &robot_state,
                             franka::Duration period) -> franka::Torques
                          {
        franka::Torques output_torques = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
        robot_state_tracking::current_ee_pose = robot_state.O_T_EE;
        if (robot_state_tracking::flag_done_collecting)
          return franka::MotionFinished(output_torques);
        else
          return output_torques; });
        }
        catch (franka::Exception const &e)
        {
            std::cout << e.what() << std::endl;
            return -1;
        }
        pose_collection_thread.join();

        Eigen::Matrix4d final_pose_matrix = Eigen::Matrix4d::Map std::ofstream outFile;
        outFile.open(ORIGIN_FILE, std::ios::out);
        outFile << final_pose_matrix(12) << " " << final_pose_matrix(13) << " " << final_pose_matrix(14) << std::endl;
    }
    catch (const franka::Exception &e)
    {
        std::cerr << e.what() << std::endl;
        return -1;
    }

    return 0;
}
