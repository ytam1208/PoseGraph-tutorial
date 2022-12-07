#include "system.hpp"

namespace Graph{
    template<typename T1, typename T2>
    class CHECK_function
    {
        public:
            bool operator()(const T1 check_flag, const T2 function_name)
            {
                if(!check_flag){
                    ROS_ERROR("Runing Error[%d] -> Function name = [%s]", check_flag, function_name.c_str());
                    ros::shutdown();
                    return -1;
                }
                return true;
            }
            CHECK_function(){}
            ~CHECK_function(){}
    };
}

class node
{
  public:
        int idx;
        double x, y, th;  
        std::vector<cv::Point2f> laser_DB;
        node() : x(0.0), y(0.0), th(0.0) {}
        ~node() {}
};

class DB
{
    private:
        std::vector<node> observation;
    public:
        std::vector<node> ref_vector;
        Graph::CHECK_function<bool, std::string> Check;
    public:
        bool load_odom_data(std::string& input_file_path);
        bool load_lm_data(std::string& input_file_path);
        bool load_Life_long_data(std::string& input_odom_file, std::string& input_laser_file);
        bool OutputPoses(const std::string& filename, const std::vector<node>& node_vector);

    DB(){}
    DB(std::string& input_odom_DB, std::string& input_l_DB, std::string& life_long_odom_DB, std::string& life_long_laser_DB, std::string& out_origin){
        Check(load_odom_data(input_odom_DB), "load_odom_data");
        Check(load_lm_data(input_l_DB), "load_lm_data");
        Check(load_Life_long_data(life_long_odom_DB, life_long_laser_DB), "load_Life_long_data");     
        Check(OutputPoses(out_origin, ref_vector), "OutputPoses");
    }
    ~DB(){}
};

