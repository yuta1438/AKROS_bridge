// overflow_modifier
// converterから帰ってくるモータの応答値が限界値を超えると反対側の限界値に移動してしまうので，それを修正するノード
#include <ros/ros.h>
#include <AKROS_bridge_msgs/motor_reply.h>
#include <AKROS_bridge_converter/motors/AK10-9.h>
#include <AKROS_bridge_converter/motors/AK80-6.h>
#include <AKROS_bridge_converter/motors/AK10-9_old.h>
#include <AKROS_bridge_converter/motors/AK80-6_old.h>
#include <stdint.h>
#include <vector>

// motor_replyの一時保管場所
typedef struct motor_reply_status_{
    uint8_t can_id;
    float P_MAX, P_MIN;
    float V_MAX, V_MIN;
    float T_MAX, T_MIN;

    float position;
    float velocity;
    float effort;

    float old_position;
    float old_velocity;
    float old_effort;

    int position_overflow_count = 0;
    int velocity_overflow_count = 0;
    int effort_overflow_count = 0;

}motor_reply_status;

std::vector<motor_reply_status> motor;
AKROS_bridge_msgs::motor_reply modified_reply;
ros::Publisher pub;
ros::Subscriber sub;


// 前回と今回の値を比較してオーバーフローしたかどうかを確認
// 上限突破 ... (P_MAX/2 < old_value <= P_MAX) && (P_MIN <= new__value < P_MIN/2)
// 下限突破 ... (P_MIN <= old_value < P_MIN/2) && (P_MAX/2 < new__value <= P_MAX)
void overflow_check(const AKROS_bridge_msgs::motor_reply& reply_){
    for(int i=0; i<reply_.motor.size(); i++){
        motor[i].can_id   = reply_.motor[i].CAN_ID;
        motor[i].position = reply_.motor[i].position;
        motor[i].velocity = reply_.motor[i].velocity;
        motor[i].effort   = reply_.motor[i].effort;

        // Position
        // 上限を突破したか？
        if(((motor[i].old_position > motor[i].P_MAX/2) && (motor[i].old_position <= motor[i].P_MAX)) && \
           ((motor[i].position >= motor[i].P_MIN) && (motor[i].position < motor[i].P_MIN/2))){
            motor[i].position_overflow_count++;
            std::cout << motor[i].position_overflow_count << std::endl;
        }
        // 下限を突破したか？
        else if(((motor[i].old_position >= motor[i].P_MIN) && (motor[i].old_position < motor[i].P_MIN/2)) && \
                ((motor[i].position > motor[i].P_MAX/2) && (motor[i].position <= motor[i].P_MAX))){
            motor[i].position_overflow_count--;
            std::cout << motor[i].position_overflow_count << std::endl;
        }


        // Velocity
        // 上限を突破したか？
        if(((motor[i].old_velocity > motor[i].V_MAX/2) && (motor[i].old_velocity <= motor[i].V_MAX)) && \
           ((motor[i].velocity >= motor[i].V_MIN) && (motor[i].velocity < motor[i].V_MIN/2))){
            motor[i].velocity_overflow_count++;
            std::cout << motor[i].velocity_overflow_count << std::endl;
        }
        // 下限を突破したか？
        else if(((motor[i].old_velocity >= motor[i].V_MIN) && (motor[i].old_velocity < motor[i].V_MIN/2)) && \
                ((motor[i].velocity > motor[i].V_MAX/2) && (motor[i].velocity <= motor[i].V_MAX))){
            motor[i].velocity_overflow_count--;
            std::cout << motor[i].velocity_overflow_count << std::endl;
        }


        // Effort
        // 上限を突破したか？
        if(((motor[i].old_effort > motor[i].T_MAX/2) && (motor[i].old_effort <= motor[i].T_MAX)) && \
           ((motor[i].effort >= motor[i].T_MIN) && (motor[i].effort < motor[i].T_MIN/2))){
            motor[i].effort_overflow_count++;
            std::cout << motor[i].effort_overflow_count << std::endl;
        }
        // 下限を突破したか？
        else if(((motor[i].old_effort >= motor[i].T_MIN) && (motor[i].old_effort < motor[i].T_MIN/2)) && \
                ((motor[i].effort > motor[i].T_MAX/2) && (motor[i].effort <= motor[i].T_MAX))){
            motor[i].effort_overflow_count--;
            std::cout << motor[i].effort_overflow_count << std::endl;
        }
        


        // 情報の更新
        motor[i].old_position = motor[i].position;
        motor[i].old_velocity = motor[i].velocity;
        motor[i].old_effort   = motor[i].effort;

        // pack modified_reply
        modified_reply.motor[i].position = motor[i].position + motor[i].P_MAX * motor[i].position_overflow_count;
        modified_reply.motor[i].velocity = motor[i].velocity + motor[i].V_MAX * motor[i].velocity_overflow_count;
        modified_reply.motor[i].effort   = motor[i].effort   + motor[i].T_MAX * motor[i].effort_overflow_count;
    }

    pub.publish(modified_reply);
}


int main(int argc, char** argv){
    ros::init(argc, argv, "motor_reply_modifier");
    ros::NodeHandle nh;
    ros::AsyncSpinner spinner(0);
    XmlRpc::XmlRpcValue params;
    nh.getParam("/motor_list", params);

    // rosparamからモータの情報を読み取り，初期設定を行う
    for(auto params_iterator = params.begin(); params_iterator!=params.end(); params_iterator++){
        motor_reply_status m;
        std::string model_name = static_cast<std::string>(params_iterator->second["model"]);
        m.can_id = static_cast<int>(params_iterator->second["can_id"]);
        if(model_name == AK10_9::model_name){
            m.P_MAX = AK10_9::P_MAX;
            m.V_MAX = AK10_9::V_MAX;
            m.T_MAX = AK10_9::T_MAX;
            m.P_MIN = AK10_9::P_MIN;
            m.V_MIN = AK10_9::V_MIN;
            m.T_MIN = AK10_9::T_MIN;
        }
        else if(model_name == AK80_6::model_name){
            m.P_MAX = AK80_6::P_MAX;
            m.V_MAX = AK80_6::V_MAX;
            m.T_MAX = AK80_6::T_MAX;
            m.P_MIN = AK80_6::P_MIN;
            m.V_MIN = AK80_6::V_MIN;
            m.T_MIN = AK80_6::T_MIN;
        }
        else if(model_name == AK10_9_OLD::model_name){
            m.P_MAX = AK10_9_OLD::P_MAX;
            m.V_MAX = AK10_9_OLD::V_MAX;
            m.T_MAX = AK10_9_OLD::T_MAX;
            m.P_MIN = AK10_9_OLD::P_MIN;
            m.V_MIN = AK10_9_OLD::V_MIN;
            m.T_MIN = AK10_9_OLD::T_MIN;
        }
        else if(model_name == AK80_6_OLD::model_name){
            m.P_MAX = AK80_6_OLD::P_MAX;
            m.V_MAX = AK80_6_OLD::V_MAX;
            m.T_MAX = AK80_6_OLD::T_MAX;
            m.P_MIN = AK80_6_OLD::P_MIN;
            m.V_MIN = AK80_6_OLD::V_MIN;
            m.T_MIN = AK80_6_OLD::T_MIN;
        }
        motor.push_back(m);
    }

    modified_reply.motor.resize(motor.size());

    pub = nh.advertise<AKROS_bridge_msgs::motor_reply>("modified_motor_reply",1);
    sub = nh.subscribe("motor_reply", 1, overflow_check);

    spinner.start();
    while(ros::ok()){};
    spinner.stop();
}