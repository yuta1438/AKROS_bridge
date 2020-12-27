// デジタル値とアナログ値をそれぞれ変換するノード
// 指令値(motor_cmd) -> CAN指令値（can_cmd）
// CAN応答値(can_reply) -> 応答値（jointstate）
#include <ros/ros.h>
#include <mutex>
//#include <AKROS_bridge_converter/general_settings.h>
#include <AKROS_bridge_converter/converter.h>

#include <AKROS_bridge_msgs/Initialize.h>
#include <AKROS_bridge_msgs/Initialize_can.h>
#include <AKROS_bridge_msgs/Initialize_lock.h>

#include <AKROS_bridge_msgs/motor_can_cmd.h>
#include <AKROS_bridge_msgs/motor_can_reply.h>
#include <AKROS_bridge_msgs/motor_cmd.h>
#include <AKROS_bridge_msgs/motor_reply.h>

#define ROS_Hz      100

// トピック通信関係
ros::Publisher can_pub; // Nucleoに整数指令値を送る
ros::Publisher reply_pub;   // ROSに実数応答値を送る
ros::Subscriber can_sub;    // Nucleoから整数応答値を受け取る
ros::Subscriber cmd_sub;    // ROSから実数指令値を受け取る

AKROS_bridge_msgs::motor_can_cmd can_cmd;
AKROS_bridge_msgs::motor_reply reply;

// サービス通信関係
ros::ServiceClient initialize_can_client;
ros::ServiceServer initialize_server;
AKROS_bridge_msgs::Initialize_can initialize_can_srv;
AKROS_bridge_msgs::Initialize_lock initialize_lock_srv;

std::mutex m;   // 排他処理

uint8_t motor_num = 0;
bool initialize_flag = false;


// motor_cmdを受け取ってcan_msgに変換
void convert_Cb(const AKROS_bridge_msgs::motor_cmd::ConstPtr& cmd_){
    std::lock_guard<std::mutex> lock(m);

    for(uint8_t i=0; i<cmd_->motor.size(); i++){
        pack_cmd(cmd_->motor[i], can_cmd.motor[i]);
    }
}

// can_replyを受け取って変換
void reply_Cb(const AKROS_bridge_msgs::motor_can_reply::ConstPtr& can_reply_){
    std::lock_guard<std::mutex> lock(m);

    for(uint8_t i=0; i<can_reply_->motor.size(); i++){
        unpack_reply(can_reply_->motor[i], reply.motor[i]);
    }
}


// request = モータ個数
// response = モータのデータ
// デジタル値を実数値に変換する必要あり！
// 2重のサービス通信の中間
// モータ一個につき一回呼び出される
bool initialize_Cb(const AKROS_bridge_msgs::Initialize::Request &req_, AKROS_bridge_msgs::Initialize::Response &res_){
    // モータ個数分だけメモリ確保
    motor_num++;

    initialize_can_srv.request.CAN_ID = req_.CAN_ID;
    initialize_can_client.call(initialize_can_srv);

    // Nucleoから帰ってきたデジタル応答値を実数応答値に変換してresponseとしてROSに返す
    unpack_reply(initialize_can_srv.response.can_reply, res_.reply);
}


// initialize終了
// モータの数を確定
void initialize_lock_Cb(const AKROS_bridge_msgs::Initialize_lock::Request& req_, AKROS_bridge_msgs::Initialize_lock::Response& res_){
    
    initialize_flag = true;
}

int main(int argc, char** argv){
    ros::init(argc, argv, "motor_can_converter");
    ros::NodeHandle nh;
    ros::Rate loop_rate(ROS_Hz);

    // initialize_lockがかかるまで待機
    while(!initialize_flag){
    }

    // ROSメッセージに関する初期設定
    can_cmd.motor.resize(motor_num);
    reply.motor.resize(motor_num);
    

    // ROSトピック設定
    can_pub = nh.advertise<AKROS_bridge_msgs::motor_can_cmd>("can_cmd_pub", 1);
    reply_pub = nh.advertise<AKROS_bridge_msgs::motor_reply>("motor_reply_pub", 1);

    can_sub = nh.subscribe<AKROS_bridge_msgs::motor_can_reply>("can_reply_sub", 1, reply_Cb);
    cmd_sub = nh.subscribe<AKROS_bridge_msgs::motor_cmd>("motor_cmd_sub", 1, convert_Cb);

    // ROSサービス設定
    initialize_can_client = nh.serviceClient<AKROS_bridge_msgs::Initialize_can>("initialize_can");
    initialize_server = nh.advertiseService("initialize", initialize_Cb);


    ros::AsyncSpinner spinner(0);

    ROS_INFO("Converter Ready !");
    spinner.start();

    while(ros::ok()){
        can_pub.publish(can_cmd);
        reply_pub.publish(reply);
        loop_rate.sleep();
    }
    
    spinner.stop();
    return 0;
}