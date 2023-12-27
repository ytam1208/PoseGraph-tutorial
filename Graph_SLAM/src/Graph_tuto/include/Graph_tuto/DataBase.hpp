#ifndef DATABASE_HPP
#define DATABASE_HPP
#include "system.hpp"

class node
{
  public:
        int idx;
        double x, y, th;  
        std::vector<cv::Point2f> laser_DB;
        node() : idx(0), x(0.0), y(0.0), th(0.0) {}
        ~node() {}
};

class DB
{
    public:
        std::vector<node> observation;
        std::vector<node> ref_vector;
        std::vector<node> measure_vector;
        Graph::CHECK_function<bool, std::string> Check;
    public:
        bool load_odom_data(std::string& input_file_path);
        bool load_lm_data(std::string& input_file_path);
        bool load_Life_long_data(std::string& input_odom_file, std::string& input_laser_file);
        bool OutputPoses(const std::string& filename, const std::vector<node>& node_vector);
        bool SolveRelativePose(std::vector<node>& reference, std::vector<node>* measure_pose);

    DB(){}
    DB(std::string& input_odom_DB, std::string& input_l_DB, std::string& life_long_odom_DB, std::string& life_long_laser_DB, std::string& out_origin, std::string& out_obser){
        Check(load_odom_data(input_odom_DB), "load_odom_data");
        Check(load_lm_data(input_l_DB), "load_lm_data");
        Check(load_Life_long_data(life_long_odom_DB, life_long_laser_DB), "load_Life_long_data");     
        Check(OutputPoses(out_origin, ref_vector), "OutputPoses");
        Check(OutputPoses(out_obser, observation), "OutputPoses");
    }
    ~DB(){}
};

#endif
