#ifndef SOLVER_HPP
#define SOLVER_HPP

#include "system.hpp"
#include "Graph_tuto/angle_manifold.h"
#include "Graph_tuto/pose_graph_2d_error_term.h"

#include "ceres/ceres.h"
#include "common/read_g2o.h"
#include "gflags/gflags.h"
#include "glog/logging.h"
#include "Graph_tuto/types.h"

#include "Graph_tuto/DataBase.hpp"
#include "Graph_tuto/ICP.hpp"

using ceres::AutoDiffCostFunction;
using ceres::CostFunction;
using ceres::Problem;
using ceres::Solver;
using ceres::Solve;

namespace Graph
{
    class Solver_
    {
        public:
            DB my_data;
            Graph::CHECK_function<bool, std::string> Check;
            ceres::Problem problem;

            bool operator()(DB input_data)
            {
                if(input_data.ref_vector.size() == 0){
                    ROS_ERROR("No data");
                    return false;
                }
                my_data = std::move(input_data);
                Check(SolveRelativePose(), "SolveRelativePose");
                Check(BuildOptimizationProblem(), "BuildOptimization");
                Check(SolveOptimizationProblem(), "SolveOptimization");
                
                return true;
            }
            bool SolveRelativePose();
            bool BuildOptimizationProblem();
            bool SolveOptimizationProblem();
            
            Solver_(){}
            ~Solver_(){}
    };

    class PoseGraph2dErrorTerm {
    public:
    PoseGraph2dErrorTerm(double x_ab,
                        double y_ab,
                        double yaw_ab_radians,
                        const Eigen::Matrix3d& sqrt_information)
        : p_ab_(x_ab, y_ab),
            yaw_ab_radians_(yaw_ab_radians),
            sqrt_information_(sqrt_information) {}

    template <typename T>
    bool operator()(const T* const x_a,
                    const T* const y_a,
                    const T* const yaw_a,
                    const T* const x_b,
                    const T* const y_b,
                    const T* const yaw_b,
                    T* residuals_ptr) const {
        const Eigen::Matrix<T, 2, 1> p_a(*x_a, *y_a);
        const Eigen::Matrix<T, 2, 1> p_b(*x_b, *y_b);

        Eigen::Map<Eigen::Matrix<T, 3, 1>> residuals_map(residuals_ptr);

        residuals_map.template head<2>() =
            ceres::examples::RotationMatrix2D(*yaw_a).transpose() * (p_b - p_a) - p_ab_.cast<T>();
        residuals_map(2) = ceres::examples::NormalizeAngle(
            (*yaw_b - *yaw_a) - static_cast<T>(yaw_ab_radians_));

        // Scale the residuals by the square root information matrix to account for
        // the measurement uncertainty.
        residuals_map = sqrt_information_.template cast<T>() * residuals_map;

        return true;
    }

    static ceres::CostFunction* Create(double x_ab,
                                        double y_ab,
                                        double yaw_ab_radians,
                                        const Eigen::Matrix3d& sqrt_information) {
        return (new ceres::
                    AutoDiffCostFunction<PoseGraph2dErrorTerm, 3, 1, 1, 1, 1, 1, 1>(
                        new PoseGraph2dErrorTerm(
                            x_ab, y_ab, yaw_ab_radians, sqrt_information)));
    }

    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    private:
    // The position of B relative to A in the A frame.
    const Eigen::Vector2d p_ab_;
    // The orientation of frame B relative to frame A.
    const double yaw_ab_radians_;
    // The inverse square root of the measurement covariance matrix.
    const Eigen::Matrix3d sqrt_information_;
    };
}

#endif
