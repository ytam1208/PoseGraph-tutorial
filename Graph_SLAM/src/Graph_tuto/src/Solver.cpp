#include "Graph_tuto/Solver.hpp"

bool Graph::Solver_::SolveRelativePose()
{
    dock_ICP get_relative_icp;
    float R[4] = { 1.0f, 0.0f, 0.0f, 1.0f };
    float T[2] = { 0.0f, 0.0f };
    CvMat r = cvMat(2, 2, CV_32F, R);
    CvMat t = cvMat(2, 1, CV_32F, T);	

    float err;
    int v_size = my_data.ref_vector.size();    
    std::vector<cv::Point2f> current, next;
    node relative_pose;

    for(int i = 0; i < v_size; i++){
        current.resize(my_data.ref_vector[i].laser_DB.size());
        std::copy(my_data.ref_vector[i].laser_DB.begin(), my_data.ref_vector[i].laser_DB.end(), current.begin());
        if(i != v_size-1){
            next.resize(my_data.ref_vector[i+1].laser_DB.size());
            std::copy(my_data.ref_vector[i+1].laser_DB.begin(), my_data.ref_vector[i+1].laser_DB.end(), next.begin());
        }
        else{
            next.resize(my_data.ref_vector[0].laser_DB.size());
            std::copy(my_data.ref_vector[0].laser_DB.begin(), my_data.ref_vector[0].laser_DB.end(), next.begin());
        }
        if(current.size() > 0 && next.size() > 0){
            err = get_relative_icp.icp(&current[0], current.size(), &next[0], next.size(), &r, &t, cvTermCriteria(CV_TERMCRIT_ITER, 100, 0.1));
            relative_pose.idx = my_data.ref_vector[i].idx;
            relative_pose.x = get_relative_icp.test_t1.data.fl[0];
            relative_pose.y = get_relative_icp.test_t1.data.fl[1];
            relative_pose.th = atan2(get_relative_icp.test_r1.data.fl[1], get_relative_icp.test_r1.data.fl[0]) * (180.0 / M_PI);
            my_data.measure_vector.push_back(relative_pose);
        }
        else    
            return false;

        next.clear();
        current.clear();
    }
    // for(auto iter : my_data.measure_vector)
    //     std::cout << "idx = " << iter.idx << " " << iter.x << ", " << iter.y << ", "  << iter.th << std::endl;

    return true;
}

bool Graph::Solver_::BuildOptimizationProblem()
{
    ceres::LossFunction* loss_function = nullptr;
    ceres::Manifold* angle_manifold = ceres::examples::AngleManifold::Create();

    const Eigen::Matrix3d information = Eigen::Matrix3d::Identity();  
    int ref_size = my_data.ref_vector.size();
    for(const auto& Relative_node : my_data.measure_vector){
        const Eigen::Matrix3d sqrt_information = information.llt().matrixL();
        ceres::CostFunction* cost_function = Graph::PoseGraph2dErrorTerm::Create(Relative_node.x, Relative_node.y, Relative_node.th, sqrt_information);
        for(int i = 0; i < ref_size; i++){
            if(my_data.ref_vector[i].idx == my_data.observation[i].idx){
                problem.AddResidualBlock(cost_function,
                                        loss_function,
                                        &my_data.ref_vector[i].x,
                                        &my_data.ref_vector[i].y,
                                        &my_data.ref_vector[i].th,
                                        &my_data.observation[i].x,
                                        &my_data.observation[i].y,
                                        &my_data.observation[i].th);

                problem.SetManifold(&my_data.ref_vector[i].th, angle_manifold);
                problem.SetManifold(&my_data.observation[i].th, angle_manifold);
            }
        }
        auto iter = my_data.ref_vector.begin();
        problem.SetParameterBlockConstant(&iter->x);
        problem.SetParameterBlockConstant(&iter->y);
        problem.SetParameterBlockConstant(&iter->th);
    }

    return true;
}

bool Graph::Solver_::SolveOptimizationProblem()
{
    ceres::Solver::Options options;
    options.max_num_iterations = 100;
    options.linear_solver_type = ceres::SPARSE_NORMAL_CHOLESKY;

    ceres::Solver::Summary summary;
    ceres::Solve(options, &problem, &summary);

    std::cout << summary.FullReport() << '\n';
    Check(my_data.OutputPoses("poses_optimized_ref.txt", my_data.ref_vector), "OutputPoses");
    Check(my_data.OutputPoses("poses_optimized_obs.txt", my_data.observation), "OutputPoses");

    return true;
}