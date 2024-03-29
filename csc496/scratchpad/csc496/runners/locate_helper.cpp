#include "locate_helper.h"
#include <iostream>
#include <array>
#include <chrono>
#include <franka/exception.h>
#include <franka/gripper.h>
#include "robotContext.h"
#include "global_variable.h"

void open_gripper() {
    if(robotContext::gripper) {
        robotContext::gripper->move(0.08, 0.1); // Open the gripper to 8cm width at 0.1m/s speed
        std::cout << "Gripper opened." << std::endl;
    }
}

void close_gripper() {
    if(robotContext::gripper) {
        robotContext::gripper->grasp(0.02, 0.1, 60); // Grasp with a width of 2cm, speed 0.1m/s, and force 60N
        std::cout << "Gripper closed." << std::endl;
    }
}

double distance(const Eigen::Vector3d& v1, const Eigen::Vector3d& v2) {
    // Implementation unchanged
}

// Function to generate a unique filename based on the current date and time
std::string generateFilename(const std::string& title) {
    auto now = std::chrono::system_clock::now();
    auto in_time_t = std::chrono::system_clock::to_time_t(now);

    std::stringstream ss;
    ss << std::put_time(std::localtime(&in_time_t), "%Y-%m-%d_%H-%M-%S");
    return title + "_" + ss.str() + ".txt";
}