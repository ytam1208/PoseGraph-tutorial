#define _USE_MATH_DEFINES
#include "Graph_tuto/Solver.hpp"
#include "Graph_tuto/angle_manifold.h"
#include "Graph_tuto/pose_graph_2d_error_term.h"
#include "Graph_tuto/types.h"

#include "system.hpp"
#include "Graph_tuto/ICP.hpp"
#include "Graph_tuto/DataBase.hpp"
#include "Graph_tuto/Solver.hpp"

int main(int argc, char **argv)
{
  
	ros::init(argc, argv, "Graph_SLAM");
	ros::NodeHandle nh;
  google::InitGoogleLogging(argv[0]);
  Graph::Solver_ sv;

  std::string folder_path = "/home/cona/git/Graph-SLAM/Graph_slam/data_map/Map1/";
  std::string file_odo_name = folder_path + "map_file.txt";
  std::string file_laser_name = folder_path +"lm_data.dat";

  std::string ll_odo_file_name = folder_path + "experience_data/odom/odom0.txt";
  std::string ll_laser_file_name = folder_path + "experience_data/laser/laser0.txt";
  std::string output_origin = "poses_original.txt";
  std::string output_observation = "poses_observation.txt";
  std::string output_optimization = "poses_optimized.txt";
  

  DB data(file_odo_name, file_laser_name, ll_odo_file_name, ll_laser_file_name, output_origin, output_observation);
  sv(data);

  // std::cout << my_data.ref_vector << std::endl;
  // file_odo_name = "/home/cona/git/Graph-SLAM/Graph_slam/obervation.txt";
  // file_laser_name = "/home/cona/git/Graph-SLAM/Graph_slam/obervation_laser.txt";
  // CHECK(ceres::example::readPoses(file_odo_name, file_laser_name, &obervation)) << "Check filename or file path.";
  // CHECK(ceres::example::OutputPoses("poses_observation.txt", obervation)) << "Error outputting to poses_observation.txt";

  // CHECK(ceres::example::SolveRelativePose(Ground_truth, &measure)) << "Error Solve relative pose";

  // ceres::Problem problem;
  // measure = ceres::example::BuildOptimizationProblem(measure, Ground_truth, obervation, &problem);
  // CHECK(ceres::example::SolveOptimizationProblem(&problem)) << "The solve was not successful, exiting.";
  // CHECK(ceres::example::OutputPoses("poses_optimized.txt", measure)) << "Error outputting to poses_observation.txt";
  // ros::spin();
  return 0;
}

