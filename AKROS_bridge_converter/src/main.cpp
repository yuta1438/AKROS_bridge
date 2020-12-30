// デジタル値とアナログ値をそれぞれ変換するノード
// 指令値(motor_cmd) -> CAN指令値（can_cmd）
// CAN応答値(can_reply) -> 応答値（jointstate）

// Service通信の中継の機能も持たせる．
// 下記のserviceをmotor_config_srvとしてNucleoにcall
// /enter_control_mode
// /exit_control_mode
// /set_position_zero
// /servo_off
// 
#include <ros/ros.h>
#include <AKROS_bridge_converter/converter.h>

#define ROS_Hz      100


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

    can_sub = nh.subscribe<AKROS_bridge_msgs::motor_can_reply>("can_reply", 1, convert_reply_Cb);
    cmd_sub = nh.subscribe<AKROS_bridge_msgs::motor_cmd>("motor_cmd", 1, convert_cmd_Cb);

    ros::AsyncSpinner spinner(0);

    ROS_INFO("Converter Ready !");
    spinner.start();

    while(ros::ok()){
        loop_rate.sleep();
    }
    
    spinner.stop();
    ros::waitForShutdown();
    return 0;
}