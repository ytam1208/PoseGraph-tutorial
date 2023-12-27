// Ceres Solver - A fast non-linear least squares minimizer
// Copyright 2015 Google Inc. All rights reserved.
// http://ceres-solver.org/
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
//
// * Redistributions of source code must retain the above copyright notice,
//   this list of conditions and the following disclaimer.
// * Redistributions in binary form must reproduce the above copyright notice,
//   this list of conditions and the following disclaimer in the documentation
//   and/or other materials provided with the distribution.
// * Neither the name of Google Inc. nor the names of its contributors may be
//   used to endorse or promote products derived from this software without
//   specific prior written permission.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
// AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
// IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
// ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
// LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
// CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
// SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
// INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
// CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
// ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
// POSSIBILITY OF SUCH DAMAGE.
//
// Author: keir@google.com (Keir Mierle)
//
// Tutorial Author: shapelim@kaist.ac.kr (임형태)

// #define _USE_MATH_DEFINES
// #include "math.h"
// // #include "common/read_g2o.h"
// #include "Graph_tuto/angle_manifold.h"
// #include "Graph_tuto/pose_graph_2d_error_term.h"
// #include "Graph_tuto/types.h"
// #include "ceres/ceres.h"
// #include "glog/logging.h"

// #include <vector>
// #include <Eigen/Dense>

// using ceres::AutoDiffCostFunction;
// using ceres::CostFunction;
// using ceres::Problem;
// using ceres::Solver;
// using ceres::Solve;

// class node
// {
//   public:
//     int idx;
//     double x, y, th;  
//     node() : x(0.0), y(0.0), th(0.0) {}
//     ~node() {}
// };

// class Pose2dErrorTerm{
//   public:
//   Pose2dErrorTerm(double measured[])
//   :p_measured(measured[0], measured[1]), theta_measured(measured[2]) {}

//   template <typename T> bool operator()(const T* const a, const T* const b, T* residual) const {
//     const Eigen::Matrix<T, 2, 1> p_a(a[0], a[1]);
//     const Eigen::Matrix<T, 2, 1> p_b(b[0], b[1]);
    
//     Eigen::Matrix<T, 2, 2> R_a;
//     const T cos = ceres::cos(a[2]);
//     const T sin = ceres::sin(a[2]);
//     R_a << cos, -sin,
//            sin, cos;
    
//     const Eigen::Matrix<T, 2, 1> p_diff = R_a.transpose() * (p_b - p_a) - p_measured.cast<T>();

//     auto theta_diff = (b[2] - a[2]) - theta_measured;
    
//     residual[0] = p_diff(0);
//     residual[1] = p_diff(1);
//     residual[2] = theta_diff;
//     // // ##############중요##############
//     // 이 부분을 더 이상 쓰지 않으니까 코멘트해서 없애줌
//     // residual[3] = 3.000001 - a[0];
//     // residual[4] = 1.000001 - a[1];
//     // residual[5] = 0.523599000001 - a[2];
//     return true;
//   }
//   private:
//   const Eigen::Vector2d p_measured;

//   double theta_measured;
// };

// namespace ceres::example
// {

//   std::vector<node> BuildOptimizationProblem(std::vector<node> &measure, std::vector<node> &Ground, std::vector<node> &observation, ceres::Problem *problem)
//   {
//     CHECK(problem != nullptr);
//     if (measure.empty() || Ground.empty() || observation.empty()){
//       LOG(INFO) << "No measure, no problem to optimize.";
//     }
//     else{
//         double * a = (double *)malloc(3*sizeof(double));
//         double * b = (double *)malloc(3*sizeof(double));
//         double * ab_measured = (double *)malloc(3*sizeof(double));

//         int i = 0;

//         ceres::Manifold* angle_manifold = ceres::examples::AngleManifold::Create();
//         for(auto& observation_node: observation){
//             for(auto& Ground_node: Ground){
//                 if(observation_node.idx == Ground_node.idx){

//                     a[0] = Ground_node.x;
//                     a[1] = Ground_node.y;
//                     a[2] = Ground_node.th;

//                     b[0] = observation_node.x;
//                     b[1] = observation_node.y;
//                     b[2] = observation_node.th;

//                     ab_measured[0] = measure[i].x;
//                     ab_measured[1] = measure[i].y;
//                     ab_measured[2] = measure[i].th;

//                     CostFunction* cost_function =
//                         new AutoDiffCostFunction<Pose2dErrorTerm, 3, 3, 3>(new Pose2dErrorTerm(ab_measured));

//                     problem->AddResidualBlock(cost_function, NULL, a, b);
                    
//                     problem->SetManifold(&Ground_node.th, angle_manifold);
//                     problem->SetManifold(&observation_node.th, angle_manifold);
//                 }
//             }
//             i++;
//         }

//         Solver::Summary summary;
//         Solver::Options options;
//         options.max_num_iterations = 100;
//         options.linear_solver_type = ceres::SPARSE_NORMAL_CHOLESKY;
//         options.minimizer_progress_to_stdout = false;
        
//         Solve(options, problem, &summary);

//         measure[i].x = b[0];
//         measure[i].y = b[1];
//         measure[i].th = b[2];
//     }

//     return measure;
//   }
// //   std::vector<node> BuildOptimizationProblem(std::vector<node> &measure, std::vector<node> &Ground, std::vector<node> &observation, ceres::Problem *problem)
// //   {
// //     CHECK(problem != nullptr);
// //     if (measure.empty() || Ground.empty() || observation.empty()){
// //       LOG(INFO) << "No measure, no problem to optimize.";
// //     }
// //     else{
// //         ceres::LossFunction* loss_function = nullptr;
// //         ceres::Manifold* angle_manifold = AngleManifold::Create();

// //         int i = 0;
// //         for(const auto& observation_node: observation){
// //             for(const auto& Ground_node: Ground){
// //                 if(observation_node.idx == Ground_node.idx){
                    
// //                     const Eigen::Matrix3d sqrt_information; 
// //                         // = constraint.information.llt().matrixL();

// //                     ceres::CostFunction* cost_function = PoseGraph2dErrorTerm::Create(
// //                         measure[i].x, measure[i].y, measure[i].th, sqrt_information);

// //                     problem->AddResidualBlock(cost_function,
// //                                             loss_function,
// //                                             Ground[i].x,
// //                                             Ground[i].y,
// //                                             Ground[i].th,
// //                                             Ground[i].x,
// //                                             Ground[i].y,
// //                                             Ground[i].th);
// //                     problem->SetManifold(Ground[i].th, angle_manifold);
// //                     problem->SetManifold(Ground[i].th, angle_manifold);
// //                     problem->SetParameterBlockConstant(&pose_start_iter->second.x);
// //                     problem->SetParameterBlockConstant(&pose_start_iter->second.y);
// //                     problem->SetParameterBlockConstant(&pose_start_iter->second.yaw_radians);
// //                 }
// //             }
// //             i++;
// //         }
// //     }

// //     return measure;
// //   }
//   bool SolveOptimizationProblem(ceres::Problem *problem)
//   {
//     CHECK(problem != nullptr);

//     ceres::Solver::Options options;
//     options.max_num_iterations = 100;
//     options.linear_solver_type = ceres::SPARSE_NORMAL_CHOLESKY;

//     ceres::Solver::Summary summary;
//     ceres::Solve(options, problem, &summary);

//     std::cout << summary.FullReport() << '\n';

//     return summary.IsSolutionUsable();
//   }

//   bool SolveRelativePose(std::vector<node>& reference, std::vector<node>& obervations, std::vector<node>* measure_pose)
//   {
//     // if(obervations != nullptr || reference != nullptr) return false;
//     if(obervations.empty() || reference.empty()){
//         std::cout << "No obervations [" << obervations.empty() << "], No reference [" << reference.empty() << "]" << std::endl;
//         return false;
//     } 
    
//     node relative_node;
//     Eigen::MatrixXd A(3, 3);
//     Eigen::MatrixXd Relative_pose(3, 3);
//     Eigen::MatrixXd C(3, 3);

//     for(const auto& oberv_node : obervations){
//       for(const auto& ref_node : reference){
//         if(oberv_node.idx == ref_node.idx){
//             A(0,0) = cos(ref_node.th);
//             A(0,1) = -sin(ref_node.th);
//             A(0,2) = ref_node.x;
//             A(1,0) = sin(ref_node.th);
//             A(1,1) = cos(ref_node.th);
//             A(1,2) = ref_node.y;
//             A(2,0) = 0.0f;
//             A(2,1) = 0.0f;
//             A(2,2) = 1.0f;

//             C(0,0) = cos(oberv_node.th);
//             C(0,1) = -sin(oberv_node.th);
//             C(0,2) = oberv_node.x;
//             C(1,0) = sin(oberv_node.th);
//             C(1,1) = cos(oberv_node.th);
//             C(1,2) = oberv_node.y;
//             C(2,0) = 0.0f;
//             C(2,1) = 0.0f;
//             C(2,2) = 1.0f;          

//             A = A.inverse();
//             Relative_pose = C * A;

//             relative_node.idx = ref_node.idx;
//             relative_node.x = Relative_pose(0,2);
//             relative_node.y = Relative_pose(0,2);
//             relative_node.th = atan2(Relative_pose(1,0), Relative_pose(0,0)) * (180.0 / M_PI); //atan2 = sin / cos...[c = 1, a = cos, b = sin]
//             measure_pose->push_back(relative_node);
//         }
//       }
//     }
//     return true;
//   }



//   bool readPoses(std::string &filename, std::vector<node> *poses)
//   {
//     std::ifstream infile(filename.c_str(), std::ios_base::in | std::ios_base::binary);
//     if(!infile.is_open())
//     {
//       std::cout << "Cannot open file [Path = " << filename.c_str() << "]" << std::endl;
//       return false;
//     }

//     int find_comma = 0;
//     std::string comma = ",";
//     std::string word, keyword;
//     std::vector<std::string> line;
//     node read_node;

//     while(infile.good()){
//       getline(infile, word);
//       if(word == "" || word == "end") break;
//       while((find_comma = word.find(comma)) != std::string::npos){
//         keyword = word.substr(0, find_comma);
//         word.erase(0, find_comma + comma.length());
//         line.push_back(keyword);
//       }
//       read_node.idx = std::stoi(line[0]);
//       read_node.x = std::stof(line[1]);
//       read_node.y = std::stof(line[2]);
//       if(std::stof(line[3]) < 0.0)  read_node.th = std::stof(line[3]) + 360.0;
//       else read_node.th = std::stof(line[3]);

//       poses->push_back(read_node);
//       std::vector<std::string>().swap(line);
//     }

//     return true;
//   }

//   bool OutputPoses(const std::string &filename, const std::vector<node> &poses)
//   {
//     std::fstream outfile;
//     outfile.open(filename.c_str(), std::istream::out);
//     if (!outfile)
//     {
//       std::cerr << "Error opening the file: " << filename << '\n';
//       return false;
//     }
//     for (const auto &pair : poses)
//     {
//       outfile << pair.idx << " " << pair.x << " " << pair.y << ' '
//               << pair.th << '\n';
//     }
//     return true;
//   }

// }

// std::vector<node> Ground_truth;
// std::vector<node> obervation;
// std::vector<node> measure;

// int main(int argc, char **argv)
// {
//     google::InitGoogleLogging(argv[0]);

//     std::string file_name = "/home/cona/git/Graph-SLAM/Graph_slam/original.txt";
//     CHECK(ceres::example::readPoses(file_name, &Ground_truth)) << "Check filename or file path.";
//     CHECK(ceres::example::OutputPoses("poses_original.txt", Ground_truth)) << "The solve was not successful, exiting.";

//     file_name = "/home/cona/git/Graph-SLAM/Graph_slam/obervation.txt";
//     CHECK(ceres::example::readPoses(file_name, &obervation)) << "Check filename or file path.";
//     CHECK(ceres::example::OutputPoses("poses_observation.txt", obervation)) << "Error outputting to poses_observation.txt";

//     CHECK(ceres::example::SolveRelativePose(Ground_truth, obervation, &measure)) << "Error Solve relative pose";
    

//     ceres::Problem problem;
//     measure = ceres::example::BuildOptimizationProblem(measure, Ground_truth, obervation, &problem);
//     // CHECK(ceres::example::SolveOptimizationProblem(&problem)) << "The solve was not successful, exiting.";
//     CHECK(ceres::example::OutputPoses("poses_optimized.txt", measure)) << "Error outputting to poses_observation.txt";

//     return 0;
// }




    // for(const auto& node : Ground_truth)
    //     std::cout << "Node[" << node.idx << "] " << node.x << ", " << node.y << ", " << node.th << std::endl; 

    // for(const auto& node : obervation)
    //     std::cout << "Node[" << node.idx << "] " << node.x << ", " << node.y << ", " << node.th << std::endl; 
