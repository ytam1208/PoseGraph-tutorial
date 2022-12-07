#include "Graph_tuto/DataBase.hpp"

bool DB::load_lm_data(std::string& input_file_path)
{
    std::ifstream DBloader(input_file_path, std::ios_base::in | std::ios_base::binary);
    if(!DBloader.is_open()) return false;

    int DB_size = 0;
    double _odom[3];
    DBloader.read((char*)&DB_size, sizeof(int));

    node read_node;
    std::vector<node> DB_node_laser_v;
    int loop_size;
    for(int i = 0; i < DB_size; i++){
        DBloader.read((char*)&read_node.idx, sizeof(int));
        DBloader.read((char*)_odom, sizeof(double) * 3);
        read_node.x = _odom[0];
        read_node.y = _odom[1];
        read_node.th = _odom[2];

        int row, col;
        DBloader.read((char*)&col, sizeof(int));
        DBloader.read((char*)&row, sizeof(int));
        if(col == 0 && row == 0)    return false;

        cv::Point2f laser_pt(0.0f, 0.0f);
        cv::Mat Points = cv::Mat(row, col, CV_32FC1);
        DBloader.read((char*)Points.data, sizeof(float)*col*row); 
        loop_size = Points.rows;
        for(int rows = 0; rows < loop_size; rows++){
            laser_pt.x = Points.at<float>(rows, 0);
            laser_pt.y = Points.at<float>(rows, 1);
            read_node.laser_DB.push_back(laser_pt);
        }
        DB_node_laser_v.push_back(read_node);
    }  

    for(auto ref_node : ref_vector){
        for(auto& DB_node : DB_node_laser_v){
            if(DB_node.idx == ref_node.idx){
                if(ref_node.laser_DB.size())    ref_node.laser_DB.clear();
                ref_node.laser_DB.shrink_to_fit();  // or...std::vector<cv::Point2f>().swap(ref_node.laser_DB);
                ref_node.laser_DB.resize(DB_node.laser_DB.size());
                std::copy(DB_node.laser_DB.begin(), DB_node.laser_DB.end(), ref_node.laser_DB.begin());
            }
        }
    }
    
    std::vector<cv::Point2f>().swap(read_node.laser_DB);
    std::vector<node>().swap(DB_node_laser_v);
    return true;
}

bool DB::load_odom_data(std::string& input_file_path)
{
    std::ifstream DBloader(input_file_path, std::ios_base::in | std::ios_base::binary);
    if(!DBloader.is_open()) return false;

    int find_comma = 0;
    std::string comma = ",";
    std::string word, keyword;
    std::vector<std::string> line;

    node ref_node;
    while(ros::ok()){
        getline(DBloader, word);
        if(word == "" || word == "end") break;
        
        while((find_comma = word.find(comma)) != std::string::npos){
            keyword = word.substr(0, find_comma);
            word.erase(0, find_comma + comma.length());
            line.push_back(keyword);
        }
        ref_node.idx = std::stoi(line[0]);
        ref_node.x = std::stof(line[1]);
        ref_node.y = std::stof(line[2]);
        ref_node.th = std::stof(line[3]);

        if(ref_node.th < 0.0)
            ref_node.th = ref_node.th + 360.0;

        std::string way_point = line[4];
        line.clear();
        ref_vector.push_back(ref_node);
    }

    DBloader.close();
    return true;
}

bool DB::load_Life_long_data(std::string &filename_odom, std::string &filename_laser)
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

      observation.push_back(read_node);
      std::vector<std::string>().swap(line);
    }
    int v_size = observation.size();

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
            if(observation[j].idx == laser_idx)
                observation[j].laser_DB.push_back(read_laser_pt);
    }
    return true;
}

bool DB::OutputPoses(const std::string& filename, const std::vector<node>& node_vector)
{
    // std::string filename = "poses_original.txt";
    std::fstream outfile;
    outfile.open(filename, std::istream::out);
    std::string data;
    if(outfile.is_open()){
        for(auto &node : node_vector)
            outfile << node.idx << " " << node.x << " " << node.y << " " << node.th << '\n';
        outfile.close();
    }
    else{
      std::cerr << "Error opening the file: " << filename << '\n';
      return false;
    }
    return true;
}

