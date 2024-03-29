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
  bool flag_done_moving;
  std::vector<std::array< double, 16 >> collected_ee_poses;
  std::vector<Eigen::Vector3d> left_points;
  std::vector<Eigen::Vector3d> right_points;
  std::vector<Eigen::Vector3d> nogozones;

}
using namespace Eigen;
using namespace std;

pair<Matrix3d, Vector3d> R_and_t(const vector<Vector3d>& points_in_left, const vector<Vector3d>& points_in_right) {
    int num_points = points_in_left.size();
    int dim_points = points_in_left[0].size();

    MatrixXd left_mat(dim_points, num_points);
    MatrixXd right_mat(dim_points, num_points);

    for (int i = 0; i < num_points; ++i) {
        left_mat.col(i) = points_in_left[i];
        right_mat.col(i) = points_in_right[i];
    }

    Vector3d left_mean = left_mat.rowwise().mean();
    Vector3d right_mean = right_mat.rowwise().mean();
    MatrixXd left_M = left_mat.colwise() - left_mean;
    MatrixXd right_M = right_mat.colwise() - right_mean;

    Matrix3d M = left_M * right_M.transpose();
    JacobiSVD<MatrixXd> svd(M, ComputeFullU | ComputeFullV);
    Matrix3d R = svd.matrixV() * Matrix3d::Identity() * svd.matrixU().transpose();
    double det = R.determinant();
    Matrix3d V = svd.matrixV();
    if (det < 0)
        V.col(2) *= -1;
    R = V * Matrix3d::Identity() * svd.matrixU().transpose();
    Vector3d t = right_mean - R * left_mean;
    std::cout << "ROTATION MATRIX" << std::endl;
    std::cout << R << std::endl;
    std::cout << "T-vector" << std::endl;
    std::cout << t << std::endl;
    return make_pair(R, t);
}
std::vector<std::array< double, 16 >> record_pose_thread(int n_poses=3){
  int collected_poses = 0; 
  std::string my_string = "";
  
  while(collected_poses < n_poses){
    std::cout << "Enter the voxel coordinates of the point that you want to register: " << std::endl;
    std::string vox_x_str = "";
    std::string vox_y_str = "";
    std::string vox_z_str = "";
    std::cout << "x-coord: ";
    std::getline(std::cin, vox_x_str);
    std::cout << "y-coord: ";
    std::getline(std::cin, vox_y_str);
    std::cout << "z-coord: ";
    std::getline(std::cin, vox_z_str);
    double vox_x = stoi(vox_x_str)*0.008;
    double vox_y = stoi(vox_y_str)*0.008;
    double vox_z = stoi(vox_z_str)*0.0096;
    global_variable::left_points.push_back(Eigen::Vector3d(vox_x, vox_y, vox_z));
    /*std::cout << "LEFT POINTS" << std::endl;
    std::cout << global_variable::left_points[collected_poses] << std::endl;*/

    std::cout << "Press ENTER to collect the pose of the point, anything else to quit data collection" << std::endl;
    std::getline(std::cin, my_string);
    if(my_string.length()==0){
      //global_variable::collected_ee_poses.push_back(global_variable::current_ee_pose);
      global_variable::right_points.push_back(Eigen::Vector3d(global_variable::current_ee_pose[12], global_variable::current_ee_pose[13], global_variable::current_ee_pose[14]));
      
      std::cout << "RIGHT POINT" << std::endl;
      std::cout << global_variable::right_points[collected_poses] << std::endl;
      
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
Eigen::Vector3d voxel_to_base(Eigen::Vector3d voxel_position, Eigen::Matrix3d R_mat, Eigen::Vector3d t_vec) {
  Eigen::Vector3d result = R_mat*voxel_position + t_vec;
  /*std::cout << "***********Voxel position*************" <<std::endl;
  std::cout << voxel_position << std::endl;
  std::cout << "************Rotation Matrix************" <<std::endl;
  std::cout << R_mat <<  std::endl;
  std::cout << R_mat << std::endl;
  std::cout << "************Product Matrix************" <<std::endl;
  std::cout << R_mat*voxel_position << std::endl;
  std::cout << "*************Position vector***********" <<std::endl;
  std::cout << t_vec << std::endl;
  std::cout << "*************Result***********" <<std::endl;
  std::cout << result << std::endl;*/
  return result;
}
double distance(const Eigen::Vector3d &v1,const Eigen::Vector3d &v2){
  Eigen::Vector3d diff = v1 - v2;
  return diff.norm();
}
void move_to_entry_point(){
  std::string inp = "";
  std::cout << "Press ENTER to continue: ";
  std::getline(std::cin, inp);
  global_variable::flag_done_moving = true;
}
Eigen::Matrix3d rotation_matrix_to_point_to_vector(const Eigen::Vector3d& TP_minus_EP) {
    Eigen::Vector3d x;
    x << 1,0,0;
    Eigen::Vector3d z = TP_minus_EP.normalized();
    Eigen::Vector3d y = z.cross(x).normalized();
    x = y.cross(z).normalized();
    Eigen::Matrix3d rotation_matrix;
    rotation_matrix.col(0) = x;
    rotation_matrix.col(1) = y;
    rotation_matrix.col(2) = z;
    if (rotation_matrix.determinant() < 0) {
      y *= -1; // Flip the sign of y
      rotation_matrix.col(1) = y;
    }
    return rotation_matrix;
}
auto move_to_collected_points(franka::Robot& robot){
  InverseKinematics ik_controller(1, IKType::M_P_PSEUDO_INVERSE);
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
}
void create_blockage(int x1, int x2, int y1, int y2, int z) {
    for (int x = x1; x <= x2; ++x) {
        for (int y = y1; y <= y2; ++y) {
            global_variable::nogozones.push_back(Eigen::Vector3d(x*0.008, y*0.008, z*0.0096));
        }
    }
}
void remove_blockage(int x, int y, int z) {
    global_variable::nogozones.erase(std::remove_if(global_variable::nogozones.begin(), global_variable::nogozones.end(),
                                    [x, y, z](const Eigen::Vector3d& voxel) {
                                        return voxel[0] == x*0.008 && voxel[1] == y*0.008 && voxel[2] == z*0.0096;
                                    }), global_variable::nogozones.end());
}
void set_nogozones(){
    create_blockage(1, 4, 2, 9, 3);
    for (int z = 1; z <= 2; ++z) {
        create_blockage(1, 4, 0, 1, z);
        create_blockage(0, 1, 2, 5, z);
        create_blockage(-1, 0, 6, 9, z);
        create_blockage(1, 4, 8, 9, z);
        create_blockage(6, 7, 6, 9, z);
        create_blockage(4, 5, 2, 5, z);
    }

    remove_blockage(2, 6, 3);
}
std::pair<double, double> find_xy_given_z(const Eigen::Vector3d& v, const Eigen::Vector3d& p, int z) {
    double x0 = p[0], y0 = p[1], z0 = p[2];
    double v1 = v[0], v2 = v[1], v3 = v[2];

    double t = (z*0.0096 - z0) / v3;
    double x = x0 + t * v1;
    double y = y0 + t * v2;
    return {x, y};
}
int main(int argc, char** argv) {
  global_variable::flag_done_collecting = false; 

  //****************************************BP1 - Record points******************************************************
  std::cout << "****BP1 - Record points****" << std::endl;
  set_nogozones();
  for (Eigen::Vector3d zone: global_variable::nogozones){
    std::cout << zone.transpose() << std::endl;
  }
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
    std::thread t1(record_pose_thread, 4);
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

    //****************************************BP2 - Find R and t******************************************************


    std::cout << "*****BP2 - Find R and t****" << std::endl;

    pair<Matrix3d, Vector3d> result = R_and_t(global_variable::left_points, global_variable::right_points);
    Eigen::Matrix3d R_mat = result.first;
    Eigen::Vector3d t_vec = result.second;

    //****************************************BP3 - Get tumour coords*****************************************************

    std::cout << "***BP3 - Get tumour coords****" << std::endl;

    std::string vox_x_str = "";
    std::string vox_y_str = "";
    std::string vox_z_str = "";

    std::cout << "Enter the voxel space coordinates of the tumour: " << std::endl;
    
    std::cout << "x-coord: ";
    std::getline(std::cin, vox_x_str);
    std::cout << "y-coord: ";
    std::getline(std::cin, vox_y_str);
    std::cout << "z-coord: ";
    std::getline(std::cin, vox_z_str);
    double vox_x = stoi(vox_x_str);
    double vox_y = stoi(vox_y_str);
    double vox_z = stoi(vox_z_str);

    //**************************************BP4 - Convert to transformation matrix*******************************************
    std::cout << "******BP4 - Convert to transformation matrix*****" << std::endl;

    std::array<double, 3> voxel_pos = {vox_x, vox_y, vox_z};
    //Eigen::Vector3d voxel_pos_mat = Eigen::Vector3d::Map(voxel_pos.data());
    Eigen::Vector3d metric_voxel;
    metric_voxel << voxel_pos[0]*0.008, voxel_pos[1]*0.008, voxel_pos[2]*0.0096;
    std::array<double, 9> current_rotation =  {global_variable::current_ee_pose[0], global_variable::current_ee_pose[1], global_variable::current_ee_pose[2], global_variable::current_ee_pose[4], global_variable::current_ee_pose[5], global_variable::current_ee_pose[6], global_variable::current_ee_pose[8], global_variable::current_ee_pose[9], global_variable::current_ee_pose[10]};
    Eigen::Matrix3d current_rot_mat = Eigen::Matrix3d::Map(current_rotation.data());
    std::cout << "Current rotation matrix" << std::endl;
    std::cout << current_rot_mat << std::endl;

    Eigen::Vector3d TP = voxel_to_base(metric_voxel, R_mat, t_vec);
    Eigen::Matrix4d vox_to_base_trans = Eigen::Matrix4d::Identity();
    vox_to_base_trans.block<3,3>(0,0) << current_rot_mat;
    vox_to_base_trans.col(3).head(3) << TP;
    std::cout << TP << std::endl;

    std::cout << "Tumour Voxel to Base Transformation Matrix" <<std::endl;
    std::cout << vox_to_base_trans << std::endl;  

    //**************************************Set NoGoZones *******************************************
    // The nogozones are hardcoded in the function right now but they can be imported from a file as well.
    
   

    
    //**************************************BP5 - Move to entry point*******************************************
    std::cout << "******BP5 - Move to entry point*****" << std::endl;
  
    global_variable::flag_done_moving = false; 
    std::thread t2(move_to_entry_point);
    try{
      robot.control([](const franka::RobotState& robot_state,
                      franka::Duration period) -> franka::Torques {

        franka::Torques output_torques = {0.0, 0.0 ,0.0, 0.0, 0.0, 0.0, 0.0}; 
        global_variable::current_ee_pose =  robot_state.O_T_EE;
        if (global_variable::flag_done_moving)
          return franka::MotionFinished(output_torques);  
        else
          return output_torques;
      });
    }catch (franka::Exception const& e) {
      std::cout << e.what() << std::endl;
      return -1;
    }
    t2.join();


    // ******************DONE******************
    // Trying to collect the entry point (position), and compute the rotation matrix that makes the EE point towards 
    // to the tumour.  Once that is done just set the vox_to_base_trans rotation matrix to this  new rotation matrix.
    // Add the transformation to collected__ee__poses and boom, we're done.
    std::array<double, 9> entry_point =  {global_variable::current_ee_pose[12], global_variable::current_ee_pose[13], global_variable::current_ee_pose[14]};

    // Vector from base to EP
    Eigen::Vector3d EP = Eigen::Vector3d::Map(entry_point.data());
    std::cout << "Current position vector" << std::endl;
    std::cout << EP << std::endl;

    // UNIT vector from EP to TP
    Eigen::Vector3d TP_minus_EP = (TP-EP).normalized();
    
    // FIX these calculation (find the right formula and think of the axis that the vectors are respective to).
    Eigen::Matrix3d rotation = rotation_matrix_to_point_to_vector(TP_minus_EP);

    // This is correct assuming the calculations are correct.
    std::cout << rotation << std::endl;
    std::cout << rotation.determinant() << std::endl;


    vox_to_base_trans.col(3).head(3) << EP;
    vox_to_base_trans.block<3,3>(0,0) << rotation;

    std::array<double, 16> vox_to_base_arr;
    int x = 0;
    for (int i = 0; i < 4; ++i) {
        for (int j = 0; j < 4; ++j) {
            vox_to_base_arr[x] = vox_to_base_trans(j, i);
            x++;
        }
    }

    global_variable::collected_ee_poses.push_back(vox_to_base_arr);

    vox_to_base_trans.col(3).head(3) << TP;
    vox_to_base_trans.block<3,3>(0,0) << rotation;
    x = 0;
    for (int i = 0; i < 4; ++i) {
        for (int j = 0; j < 4; ++j) {
            vox_to_base_arr[x] = vox_to_base_trans(j, i);
            x++;
        }
    }

    global_variable::collected_ee_poses.push_back(vox_to_base_arr);


  std::cout << "Done collecting ************** Moving to tumour" << std::endl;

/** [COMPLETED]
 * Check why the axis is rotated on the z axis (5,4,0) ends up being (-4,5,0) for some reason
 *      - This works on Puneet's robot (I configure the rotation matrix to match the specifications)
 *        Maybe the one we were working on earlier had slightly incorrect rotation matrix
 * Figure out how to keep the robot running (freely) after collecting the points.
 *      - Done, use the move_to_entry_point function to keep going until it is able to find the 
 *        valid entry point (maybe add a while loop around the free motion threads to keep repeating)
 * 
 * TODO
 * Collect the current entry point(EP) and given the tumour point(TP), determine the transformation matrix from EP to TP
 * RESOURCES
 *      - https://calcworkshop.com/vectors-and-the-geometry-of-space/3d-vector/#:~:text=But%20how%20do%20we%20find,z%202%20%E2%88%92%20z%201%20%E2%9F%A9%20.
 *      - https://math.stackexchange.com/questions/180418/calculate-rotation-matrix-to-align-vector-a-to-vector-b-in-3d
*/

  // for (auto &ee_pose: ee_poses){
  

  move_to_collected_points(*robotContext::robot);


  } catch (franka::Exception const& e) {
    std::cout << e.what() << std::endl;
    return -1;
  }

  return 0;
}
