#define _USE_MATH_DEFINES

#include "system.hpp"
#include "Graph_tuto/angle_manifold.h"
#include "Graph_tuto/pose_graph_2d_error_term.h"
#include "Graph_tuto/types.h"
#include "ceres/ceres.h"
#include "glog/logging.h"
#include "Graph_tuto/ICP.hpp"
#include "Graph_tuto/DataBase.hpp"

using ceres::AutoDiffCostFunction;
using ceres::CostFunction;
using ceres::Problem;
using ceres::Solver;
using ceres::Solve;


class Pose2dErrorTerm{
  public:
  Pose2dErrorTerm(double measured[])
  :p_measured(measured[0], measured[1]), theta_measured(measured[2]) {}

  template <typename T> bool operator()(const T* const a, const T* const b, T* residual) const {
    const Eigen::Matrix<T, 2, 1> p_a(a[0], a[1]);
    const Eigen::Matrix<T, 2, 1> p_b(b[0], b[1]);
    
    Eigen::Matrix<T, 2, 2> R_a;
    const T cos = ceres::cos(a[2]);
    const T sin = ceres::sin(a[2]);
    R_a << cos, -sin,
           sin, cos;
    
    const Eigen::Matrix<T, 2, 1> p_diff = R_a.transpose() * (p_b - p_a) - p_measured.cast<T>();

    auto theta_diff = (b[2] - a[2]) - theta_measured;
    
    residual[0] = p_diff(0);
    residual[1] = p_diff(1);
    residual[2] = theta_diff;
    // // ##############중요##############
    // 이 부분을 더 이상 쓰지 않으니까 코멘트해서 없애줌
    // residual[3] = 3.000001 - a[0];
    // residual[4] = 1.000001 - a[1];
    // residual[5] = 0.523599000001 - a[2];
    return true;
  }
  private:
  const Eigen::Vector2d p_measured;

  double theta_measured;
};

namespace ceres::example
{
  std::vector<node> BuildOptimizationProblem(std::vector<node> &measure, std::vector<node> &Ground, std::vector<node> &observation, ceres::Problem *problem)
  {
    CHECK(problem != nullptr);
    if (measure.empty() || Ground.empty() || observation.empty()){
      LOG(INFO) << "No measure, no problem to optimize.";
    }
    else{
        double * a = (double *)malloc(3*sizeof(double));
        double * b = (double *)malloc(3*sizeof(double));
        double * ab_measured = (double *)malloc(3*sizeof(double));

        int i = 0;

        ceres::Manifold* angle_manifold = ceres::examples::AngleManifold::Create();
        for(auto& observation_node: observation){
            for(auto& Ground_node: Ground){
                if(observation_node.idx == Ground_node.idx){

                    a[0] = Ground_node.x;
                    a[1] = Ground_node.y;
                    a[2] = Ground_node.th;

                    b[0] = observation_node.x;
                    b[1] = observation_node.y;
                    b[2] = observation_node.th;

                    ab_measured[0] = measure[i].x;
                    ab_measured[1] = measure[i].y;
                    ab_measured[2] = measure[i].th;

                    CostFunction* cost_function =
                        new AutoDiffCostFunction<Pose2dErrorTerm, 3, 3, 3>(new Pose2dErrorTerm(ab_measured));

                    problem->AddResidualBlock(cost_function, NULL, a, b);
                    
                    problem->SetManifold(&Ground_node.th, angle_manifold);
                    problem->SetManifold(&observation_node.th, angle_manifold);
                }
            }
            i++;
        }

        Solver::Summary summary;
        Solver::Options options;
        options.max_num_iterations = 100;
        options.linear_solver_type = ceres::SPARSE_NORMAL_CHOLESKY;
        options.minimizer_progress_to_stdout = false;
        
        Solve(options, problem, &summary);

        measure[i].x = b[0];
        measure[i].y = b[1];
        measure[i].th = b[2];
    }

    return measure;
  }
//   std::vector<node> BuildOptimizationProblem(std::vector<node> &measure, std::vector<node> &Ground, std::vector<node> &observation, ceres::Problem *problem)
//   {
//     CHECK(problem != nullptr);
//     if (measure.empty() || Ground.empty() || observation.empty()){
//       LOG(INFO) << "No measure, no problem to optimize.";
//     }
//     else{
//         ceres::LossFunction* loss_function = nullptr;
//         ceres::Manifold* angle_manifold = AngleManifold::Create();

//         int i = 0;
//         for(const auto& observation_node: observation){
//             for(const auto& Ground_node: Ground){
//                 if(observation_node.idx == Ground_node.idx){
                    
//                     const Eigen::Matrix3d sqrt_information; 
//                         // = constraint.information.llt().matrixL();

//                     ceres::CostFunction* cost_function = PoseGraph2dErrorTerm::Create(
//                         measure[i].x, measure[i].y, measure[i].th, sqrt_information);

//                     problem->AddResidualBlock(cost_function,
//                                             loss_function,
//                                             Ground[i].x,
//                                             Ground[i].y,
//                                             Ground[i].th,
//                                             Ground[i].x,
//                                             Ground[i].y,
//                                             Ground[i].th);
//                     problem->SetManifold(Ground[i].th, angle_manifold);
//                     problem->SetManifold(Ground[i].th, angle_manifold);
//                     problem->SetParameterBlockConstant(&pose_start_iter->second.x);
//                     problem->SetParameterBlockConstant(&pose_start_iter->second.y);
//                     problem->SetParameterBlockConstant(&pose_start_iter->second.yaw_radians);
//                 }
//             }
//             i++;
//         }
//     }

//     return measure;
//   }
  bool SolveOptimizationProblem(ceres::Problem *problem)
  {
    CHECK(problem != nullptr);

    ceres::Solver::Options options;
    options.max_num_iterations = 100;
    options.linear_solver_type = ceres::SPARSE_NORMAL_CHOLESKY;

    ceres::Solver::Summary summary;
    ceres::Solve(options, problem, &summary);

    std::cout << summary.FullReport() << '\n';

    return summary.IsSolutionUsable();
  }

  bool SolveRelativePose(std::vector<node>& reference, std::vector<node>* measure_pose)
  {
    // if(obervations != nullptr || reference != nullptr) return false;
    Eigen::MatrixXd Relative_pose(3, 3);

    if(reference.empty()){
        std::cout << "No reference [" << reference.empty() << "]" << std::endl;
        return false;
    } 

    dock_ICP get_relative_icp;
    float R[4] = { 1.0f, 0.0f, 0.0f, 1.0f };
    float T[2] = { 0.0f, 0.0f };
    CvMat r = cvMat(2, 2, CV_32F, R);
    CvMat t = cvMat(2, 1, CV_32F, T);	

    float err;
    int v_size = reference.size();
    for(int i = 0; i < v_size; i++){
        std::vector<cv::Point2f> current, next;
        current.resize(reference[i].laser_DB.size());
        std::copy(reference[i].laser_DB.begin(), reference[i].laser_DB.end(), current.begin());
        if(i != v_size-1){
            next.resize(reference[i+1].laser_DB.size());
            std::copy(reference[i+1].laser_DB.begin(), reference[i+1].laser_DB.end(), next.begin());
        }
        else{
            next.resize(reference[0].laser_DB.size());
            std::copy(reference[0].laser_DB.begin(), reference[0].laser_DB.end(), next.begin());            
        }
        err = get_relative_icp.icp(&current[0], current.size(), &next[0], next.size(), &r, &t, cvTermCriteria(CV_TERMCRIT_ITER, 100, 0.1));
        
        // Relative_pose(0,0) = get_relative_icp.test_r1.data.fl[0];   //cos
        // Relative_pose(0,1) = get_relative_icp.test_r1.data.fl[1];   //sin
        // Relative_pose(0,2) = get_relative_icp.test_t1.data.fl[0];   //tx
        // Relative_pose(1,0) = get_relative_icp.test_r1.data.fl[2];   //sin
        // Relative_pose(1,1) = get_relative_icp.test_r1.data.fl[3];   //cos
        // Relative_pose(1,2) = get_relative_icp.test_t1.data.fl[1];   //ty
        // Relative_pose(2,0) = 0.0f;
        // Relative_pose(2,1) = 0.0f;
        // Relative_pose(2,2) = 1.0f;

        node relative_node;
        relative_node.idx = reference[i].idx;
        relative_node.x = get_relative_icp.test_t1.data.fl[0];
        relative_node.y = get_relative_icp.test_t1.data.fl[1];
        relative_node.th = atan2(get_relative_icp.test_r1.data.fl[1], get_relative_icp.test_r1.data.fl[0]) * (180.0 / M_PI);    //degree = atan2(sin / cos)
        measure_pose->push_back(relative_node);
    }
    
    return true;
  }



  bool readPoses(std::string &filename_odom, std::string &filename_laser, std::vector<node> *poses)
  {
    std::ifstream infile_odom(filename_odom.c_str(), std::ios_base::in | std::ios_base::binary);
    if(!infile_odom.is_open())  return false;

    int find_comma = 0;
    std::string comma = ",";
    std::string word, keyword;
    std::vector<std::string> line;
    node read_node;

    while(infile_odom.good()){
      getline(infile_odom, word);
      if(word == "" || word == "end") break;
      while((find_comma = word.find(comma)) != std::string::npos){
        keyword = word.substr(0, find_comma);
        word.erase(0, find_comma + comma.length());
        line.push_back(keyword);
      }
      read_node.idx = std::stoi(line[0]);
      read_node.x = std::stof(line[1]);
      read_node.y = std::stof(line[2]);
      if(std::stof(line[3]) < 0.0)  read_node.th = std::stof(line[3]) + 360.0;
      else read_node.th = std::stof(line[3]);

      poses->push_back(read_node);
      std::vector<std::string>().swap(line);
    }
    int v_size = poses->size();

    std::ifstream infile_laser(filename_laser.c_str(), std::ios_base::in | std::ios_base::binary);
    if(!infile_laser.is_open()) return false;
    
    while(infile_laser.good()){
        getline(infile_laser, word);
        if(word == "" || word == "end") break;
        while((find_comma = word.find(comma)) != std::string::npos){
            keyword = word.substr(0, find_comma);
            word.erase(0, find_comma + comma.length());
            line.push_back(keyword);
        }        
        int laser_idx = std::stoi(line[0]);
        cv::Point2d read_laser_pt;
        read_laser_pt.x = std::stof(line[1]);
        read_laser_pt.y = std::stof(line[2]);
        line.clear();
        for(int j = 0; j < v_size; j++)
            if((*poses)[j].idx == laser_idx)
                (*poses)[j].laser_DB.push_back(read_laser_pt);
    }

    return true;
  }
}

std::vector<node> Ground_truth;
std::vector<node> obervation;
std::vector<node> measure;

int main(int argc, char **argv)
{
	ros::init(argc, argv, "Graph_SLAM");
	ros::NodeHandle nh;
    google::InitGoogleLogging(argv[0]);
    std::string folder_path = "/home/cona/git/Graph-SLAM/Graph_slam/data_map/Map1/";
    std::string file_odo_name = folder_path + "map_file.txt";
    std::string file_laser_name = folder_path +"lm_data.dat";

    std::string ll_odo_file_name = folder_path + "experience_data/odom/odom0.txt";
    std::string ll_laser_file_name = folder_path + "experience_data/laser/laser0.txt";
    std::string output_origin = "poses_original.txt";
    DB data(file_odo_name, file_laser_name, ll_odo_file_name, ll_laser_file_name, output_origin);

    // file_odo_name = "/home/cona/git/Graph-SLAM/Graph_slam/obervation.txt";
    // file_laser_name = "/home/cona/git/Graph-SLAM/Graph_slam/obervation_laser.txt";
    // CHECK(ceres::example::readPoses(file_odo_name, file_laser_name, &obervation)) << "Check filename or file path.";
    // CHECK(ceres::example::OutputPoses("poses_observation.txt", obervation)) << "Error outputting to poses_observation.txt";

    // CHECK(ceres::example::SolveRelativePose(Ground_truth, &measure)) << "Error Solve relative pose";
    

    // ceres::Problem problem;
    // measure = ceres::example::BuildOptimizationProblem(measure, Ground_truth, obervation, &problem);
    // CHECK(ceres::example::SolveOptimizationProblem(&problem)) << "The solve was not successful, exiting.";
    // CHECK(ceres::example::OutputPoses("poses_optimized.txt", measure)) << "Error outputting to poses_observation.txt";
    ros::spin();
    return 0;
}




    // for(const auto& node : Ground_truth)
    //     std::cout << "Node[" << node.idx << "] " << node.x << ", " << node.y << ", " << node.th << std::endl; 

    // for(const auto& node : obervation)
    //     std::cout << "Node[" << node.idx << "] " << node.x << ", " << node.y << ", " << node.th << std::endl; 
