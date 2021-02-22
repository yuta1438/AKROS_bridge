// ノードからPIDゲインを読み込めないか？
// 結局はrosparamから読み込むことになりそう！

// 基本的にはstd::mapと同じ構造なので不明な場合はそれを参考に！
#include <ros/ros.h>

int main(int argc, char** argv){
    ros::init(argc, argv, "check_rosparam");
    ros::NodeHandle nh;
    
    /*std::string name;
    nh.getParam("motor_list/Hip/model", name);
    ROS_INFO("%s", name.c_str());*/

    XmlRpc::XmlRpcValue params;
    nh.getParam("/motor_list", params);


    // どうやったらnameの部分を書かずにキーの名前を取得できるか？
    // iterator->firstで取得できる！！！
    for(auto params_iterator = params.begin(); params_iterator != params.end(); ++params_iterator){
        ROS_INFO("%-8s, %d", (static_cast<std::string>(params_iterator->first)).c_str(), static_cast<int>(params_iterator->second["can_id"]));
        
        // どうやったらjointLimit: []の中身を取り出せるか？
        if(params_iterator->second["joint_limit"].valid()){
            // これで取れるっちゃ取れる...
            /*
            std::vector<double> limit;
            nh.getParam("/motor_list/Hip/joint_limit", limit);
            for(auto& e : limit){
                std::cout << e << std::endl;
            }*/

            // iterator経由で取得する方法が分からなかったのでNodeHandle経由で取得するようにする．
            std::vector<double> limit;
            std::string string1 = "/motor_list/";
            std::string string2 = "/joint_limit";
            std::string full_path = string1 + static_cast<std::string>(params_iterator->first) + string2;
            nh.getParam(full_path, limit);
            for(auto& e : limit){
                std::cout << e << std::endl;
            }
        }
    }
    
    return 0;
}