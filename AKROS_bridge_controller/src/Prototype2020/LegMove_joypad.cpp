// Prototype2020用プログラム
// 脚先位置をdualshock4の左ジョイスティックで制御．

// ジョイスティックを急激に動かすとロボットも急に動いてしまうので，リミットをかけて対処する
// 制御周期ごとの修正量にフィルタをかける．
// 最高速度をどのくらいにするか？
// 0.08mを最短で0.5秒で動くように設定．
// 制御周期は100Hzなので，上限は0.0016[m/回]？

// before run this, run "rosrun joy joy_node" to enable joypad.

#include <AKROS_bridge_controller/Prototype2020_BaseController.h>
#include <sensor_msgs/Joy.h>
#include <geometry_msgs/Point32.h>
#include <mutex>

class LegMove_joypad_Controller : public Prototype2020_BaseController{
private:
    const double settingTime = 2.0;
    geometry_msgs::Point32 joy_cmd;
    sensor_msgs::Joy latestJoyMsg;
    std::mutex joy_mutex;

    const double radius_x = 0.08;    // x軸方向の最大移動値
    const double radius_z = 0.08;    // y軸方向の最大移動値
    const double max_wheel_speed = 1.0;

    const double max_delta_x = 0.005;  // 制御周期ごとの最大x方向移動量
    const double max_delta_z = 0.005;  // 制御周期ごとの最大z方向移動量

    ros::Publisher joy_cmd_pub;
    ros::Subscriber joy_sub;
    bool first_operation_done = false;

    Eigen::VectorXd q_initialize;   // 基準姿勢を取るときの脚の角度ベクトル
    Eigen::Vector2d p_initialize;   // 基準となる脚先位置
    Eigen::Vector2d p_target;       // joypadによる操作量
    Eigen::Vector2d p_target_old;   // 前回の操作量
    Eigen::Vector2d p_delta;        // 
    Eigen::Vector2d pref;       // 脚先位置の目標値

    const double q_init_deg[2] = {45.0f, -90.0f};

    void joy_callback(const sensor_msgs::Joy& joy_msg){
        if(!first_operation_done){
            first_operation_done = true;
        }
        std::lock_guard<std::mutex> lock(joy_mutex);
        latestJoyMsg = joy_msg;
    }

public:
    LegMove_joypad_Controller(void){
        joy_cmd_pub = nh.advertise<geometry_msgs::Point32>("joy_cmd", 1);
        joy_sub = nh.subscribe("joy", 1, &LegMove_joypad_Controller::joy_callback, this);
        q_initialize.resize(LEG_JOINTNUM);
        q_initialize << deg2rad(q_init_deg[0]), deg2rad(q_init_deg[1]);
        solve_sagittal_FK(q_initialize.head<2>(), p_initialize);    // 中心となる脚先位置の計算
        qref = q_init;
    }

    virtual void loop(const ros::TimerEvent& e) override {
        double current_time = getTime();

        if(phase == 0){
            if(!initializeFlag){
                joint_Interpolator.clear();
                joint_Interpolator.appendSample(current_time, q_init.head<2>());
                joint_Interpolator.appendSample(current_time+settingTime, q_initialize);
                joint_Interpolator.update();
                initializeFlag = true;
            }

            qref.head<2>() = joint_Interpolator.interpolate(current_time);

            if(current_time > joint_Interpolator.domainUpper()){
                initializeFlag = false;
                
                phase = 1;
                pref[0] = p_initialize[0];
                pref[1] = p_initialize[1];
                p_target = Eigen::Vector2d::Zero();
                p_target_old = Eigen::Vector2d::Zero();
                timer_start();
            }
        }

        // joypadを動かしたら次のフェーズを行うようにする
        else if(phase == 1){
            ROS_INFO("Move controller to start controller!");
            while(!first_operation_done);   // Segment Faultを防ぐためにコントローラを一度操作してから起動するようにする

            ROS_INFO("Enter key to start controlling ...");
            char buf;
            std::cin >> buf;    // 待ち
            phase = 2;
        }

        // joypadから送信されてくる指令に対して目標脚先位置を計算
        // IKを解き，モータへ送信
        else if(phase == 2){
            Eigen::VectorXd q_buf(2);
            
            {
                std::lock_guard<std::mutex> lock(joy_mutex);
                
                p_target[0] = -radius_x * latestJoyMsg.axes[0];
                p_target[1] = radius_z * latestJoyMsg.axes[1];
            }
            
            // filter p_target ---
            p_delta = p_target - p_target_old;

            // x軸方向の移動速度制限
            if(p_delta[0] > max_delta_x){
                p_delta[0] = max_delta_x;
            }else if(p_delta[0] < -max_delta_x){
                p_delta[0] = -max_delta_x;
            }

            // z軸方向の移動速度制限
            if(p_delta[1] > max_delta_z){
                p_delta[1] = max_delta_z;
            }else if(p_delta[1] < -max_delta_z){
                p_delta[1] = -max_delta_z;
            }

            p_target = p_target_old + p_delta;
            p_target_old = p_target;
            // ----------

            pref = p_initialize + p_target;
            solve_sagittal_IK(pref, q_buf);
            qref.head<2>() = q_buf;
            qref[WHEEL] = -(qref[HIP] - q_initialize[HIP]);    // 車輪が地面に対して動かないように制御

            joy_cmd.x = pref[0];
            joy_cmd.z = pref[1];
            joy_cmd_pub.publish(joy_cmd);
        }
        sendCommand();
    }
};

int main(int argc, char** argv){
    ros::init(argc, argv, "Legmove_joypad_controller");
    ros::AsyncSpinner spinner(0);
    Prototype2020_BaseController *controller = new LegMove_joypad_Controller;
    spinner.start();
    ros::waitForShutdown();
    return 0;
}
