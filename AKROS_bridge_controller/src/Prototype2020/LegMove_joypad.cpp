// Prototype2020用プログラム
// 脚先位置をdualshock4の左ジョイスティックで制御．

// before run this, run "rosrun joy joy_node" to enable joypad.

#include <AKROS_bridge_controller/Prototype2020_BaseController.h>
#include <sensor_msgs/Joy.h>
#include <geometry_msgs/Point32.h>

class LegMove_joypad_Controller : public Prototype2020_BaseController{
private:
    const double settingTime = 2.0;
    geometry_msgs::Point32 joy_cmd;

    const double radius_x = 0.05;    // x軸方向の最大移動値
    const double radius_z = 0.05;    // y軸方向の最大移動値

    ros::Publisher joy_cmd_pub;
    ros::Subscriber joy_sub;

    Eigen::VectorXd q_initialize;     // 基準姿勢を取るときの脚の角度ベクトル
    Eigen::Vector2d p_initialize;     // 基準となる脚先位置
    Eigen::Vector2d delta_p;    // joypadによる操作量
    Eigen::Vector2d pref;       // 脚先位置の目標値

    const double q_init_deg[2] = {45.0f, -90.0f};

    void joy_callback(const sensor_msgs::Joy& joy_msg){
        delta_p << radius_x * -joy_msg.axes[0], radius_z * joy_msg.axes[1];
        pref = p_initialize + delta_p;
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
                ROS_INFO("Enter key to start controlling ...");
                char buf;
                std::cin >> buf;    // 待ち
                phase = 1;
                timer_start();
            }
        }

        // joypadから送信されてくる指令に対して目標脚先位置を計算
        // IKを解き，モータへ送信
        else if(phase == 1){
            // ros::spinOnce(); // subscriberのcallback関数を実行

            Eigen::VectorXd q_buf(2);
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
    Prototype2020_BaseController *controller = new LegMove_joypad_Controller;
    ros::spin();
    return 0;
}
