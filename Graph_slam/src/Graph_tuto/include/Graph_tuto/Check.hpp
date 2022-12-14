
#ifndef CHECK_HPP
#define CHECK_HPP

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

#endif
