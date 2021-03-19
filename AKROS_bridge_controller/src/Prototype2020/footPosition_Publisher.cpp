// footposition_publisher
// ロボット(Prototype2020)の脚先位置計算．/motor_replyの値を読み取って行う．
#include <ros/ros.h>
#include <AKROS_bridge_msgs/motor_reply.h>
#include <AKROS_bridge_controller/Prototype2020_BaseController.h>
#include <geometry_msgs/Point32.h>
#include <AKROS_bridge_controller/Prototype2020.h>

geometry_msgs::Point32 foot_position;
ros::Publisher foot_position_pub;

ros::Subscriber motor_reply_sub;

// 
void motor_reply_Cb(const AKROS_bridge_msgs::motor_reply::ConstPtr& reply_){
    Eigen::Vector2d q;
    for(int i=0; i<reply_->motor.size(); i++){
        // HipとKneeの情報だけ欲しい
        if(reply_->motor[i].CAN_ID != 3){
            if(reply_->motor[i].CAN_ID == 1){
                q[0] = reply_->motor[i].position;
            }else if(reply_->motor[i].CAN_ID == 2){
                q[1] = reply_->motor[i].position;
            }
        }
    }

    foot_position.x = -l1*sin(q[0]) - l2*sin(q[0]+q[1]);
    foot_position.z = -l1*cos(q[0]) - l2*cos(q[0]+q[1]);
    foot_position_pub.publish(foot_position);
}

int main(int argc, char** argv){
    ros::init(argc, argv, "FootPosition_publisher");
    ros::NodeHandle nh;
    ros::AsyncSpinner spinner(0);
    nh.advertise<geometry_msgs::Point32>("foot_position", 1);

    spinner.start();
    while(ros::ok()){}
    spinner.stop();
    return 0;
}