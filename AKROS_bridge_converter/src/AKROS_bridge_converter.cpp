// デジタル値とアナログ値をそれぞれ変換するノード
// 指令値(motor_cmd) -> CAN指令値（can_cmd）
// CAN応答値(can_reply) -> 応答値（jointstate）

#include <ros/ros.h>
#include <mutex>
#include <AKROS_bridge_converter/converter.h>

#include <AKROS_bridge_msgs/motor_can_cmd.h>
#include <AKROS_bridge_msgs/motor_can_reply.h>
#include <AKROS_bridge_msgs/motor_cmd.h>
#include <AKROS_bridge_msgs/motor_reply.h>

#define ROS_Hz      100

// トピック通信関係
ros::Publisher can_pub;     // Nucleoに整数指令値を送る
ros::Publisher reply_pub;   // ROSに実数応答値を送る
ros::Subscriber can_sub;    // Nucleoから整数応答値を受け取る
ros::Subscriber cmd_sub;    // ROSから実数指令値を受け取る

AKROS_bridge_msgs::motor_can_cmd can_cmd;
AKROS_bridge_msgs::motor_reply reply;

std::mutex m;   // 排他処理

uint8_t motor_num = 0;
bool initialize_flag = false;


// motor_cmdを受け取ってcan_msgに変換
void convert_Cb(const AKROS_bridge_msgs::motor_cmd::ConstPtr& cmd_){
    std::lock_guard<std::mutex> lock(m);

    if(motor_num != cmd_->motor.size()){
        motor_num = cmd_->motor.size();
        can_cmd.motor.resize(motor_num);
    }
    for(uint8_t i=0; i<motor_num; i++){
        pack_cmd(cmd_->motor[i], can_cmd.motor[i]);
    }
}

// can_replyを受け取って変換
void reply_Cb(const AKROS_bridge_msgs::motor_can_reply::ConstPtr& can_reply_){
    std::lock_guard<std::mutex> lock(m);
    if(motor_num != can_reply_->motor.size()){
        motor_num = can_reply_->motor.size();
        reply.motor.resize(motor_num);
    }
    
    for(uint8_t i=0; i<motor_num; i++){
        unpack_reply(can_reply_->motor[i], reply.motor[i]);
    }
}

int main(int argc, char** argv){
    ros::init(argc, argv, "motor_can_converter");
    ros::NodeHandle nh;
    ros::Rate loop_rate(ROS_Hz);

    // ROSメッセージに関する初期設定
    can_cmd.motor.resize(motor_num);
    reply.motor.resize(motor_num);

    // ROSトピック設定
    can_pub = nh.advertise<AKROS_bridge_msgs::motor_can_cmd>("can_cmd", 1);
    reply_pub = nh.advertise<AKROS_bridge_msgs::motor_reply>("motor_reply", 1);

    can_sub = nh.subscribe<AKROS_bridge_msgs::motor_can_reply>("can_reply", 1, reply_Cb);
    cmd_sub = nh.subscribe<AKROS_bridge_msgs::motor_cmd>("motor_cmd", 1, convert_Cb);

    ros::AsyncSpinner spinner(0);

    ROS_INFO("Converter Ready !");
    spinner.start();

    while(ros::ok()){
        can_pub.publish(can_cmd);
        reply_pub.publish(reply);
        loop_rate.sleep();
    }
    
    spinner.stop();
    ros::waitForShutdown();
    return 0;
}