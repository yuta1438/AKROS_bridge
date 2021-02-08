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
        ROS_INFO("%s, %d", (static_cast<std::string>(params_iterator->first)).c_str(), static_cast<int>(params_iterator->second["can_id"]));
    }
    
    return 0;
}