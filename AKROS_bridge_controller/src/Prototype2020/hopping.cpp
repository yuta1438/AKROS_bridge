// 単脚用プログラム
// 跳躍させるプログラム

#include <AKROS_bridge_controller/Prototype2020_BaseController.h>
#include <geometry_msgs/Point32.h>
class Hopping_Controller : public Prototype2020_BaseController{
private:
    const double settingTime = 2.0;  // 初期位置に移動する時間
    const double movingTime = 0.08;  // 伸展時間
    double temp_time;
    
    const double q_initialize_deg[2] = {60.0, -120.0};  // 初期関節角度

    Eigen::VectorXd q_initialize;  // 関節角度ベクトル定義
    Eigen::Vector2d p_initialize;  // 位置ベクトル定義

    geometry_msgs::Point32 foot_position;  // 脚先位置のメッセージ定義
    ros::Publisher foot_position_pub;  // 脚先位置のPublisherを定義
public:
    Hopping_Controller(void){
        foot_position_pub = nh.advertise<geometry_msgs::Point32>("target_foot_position", 1);

        q_initialize.resize(LEG_JOINTNUM);  // q_initializeの要素数の変更
        q_initialize << deg2rad(q_initialize_deg[0]), deg2rad(q_initialize_deg[1]);  // q_initializeに代入
    
        solve_sagittal_FK(q_initialize, p_initialize);  // 順運動学(q初期角度 → p初期脚先位置)

        qref = q_init;  // q_initをqrefに代入
    }

    virtual void loop(const ros::TimerEvent& e) override {
        double current_time = getTime();  // 現在時刻取得

        // 初期姿勢へ
        if(phase == 0){
            if(!initializeFlag){
                joint_Interpolator.clear();
                joint_Interpolator.appendSample(current_time, q_init.head<2>());  // 現在の関節角度
                joint_Interpolator.appendSample(current_time+settingTime, q_initialize);  // 初期関節角度
                joint_Interpolator.update();  // 補間
                initializeFlag = true;
            }

            qref.head<2>() = joint_Interpolator.interpolate(current_time);  // 

            if(current_time > joint_Interpolator.domainUpper()){  // 初期関節角度になった後であれば
                initializeFlag = false;
                ROS_INFO("Enter key to start moving ...");
                char buf;
                std::cin >> buf;  // 待ち
                phase = 1;
                timer_start();
            }
        }

        // 跳躍
        else if(phase == 1){
            if(initializeFlag == false){
                Eigen::Vector2d delta_p1(0.00, -0.16);  // 跳躍後の脚先位置

                leg_Interpolator.clear();
                leg_Interpolator.appendSample(current_time, p_initialize);  // 初期位置
                leg_Interpolator.appendSample(current_time+(movingTime), p_initialize + delta_p1);  // 最終位置
                leg_Interpolator.update();  // 補間
                initializeFlag = true;
            }

            Eigen::VectorXd q_buff;
            q_buff.resize(LEG_JOINTNUM);
            solve_sagittal_IK(leg_Interpolator.interpolate(current_time), q_buff);  // 逆運動学(現在時刻の脚先位置 → 位置関節角度)
            qref.head<LEG_JOINTNUM>() = q_buff;
			qref[WHEEL] = -qref[WHEEL];
            // debug
            // -----
            Eigen::Vector2d buff = leg_Interpolator.interpolate(current_time);
            foot_position.x = buff[0];
            foot_position.z = buff[1];
            foot_position_pub.publish(foot_position);
            // -----

            if(current_time > leg_Interpolator.domainUpper()){  // 跳躍後であれば
                initializeFlag = false;
                phase = 2;
            }
        }

        else if(phase == 2){
            if(!initializeFlag){
                temp_time = getTime();
                initializeFlag = true;
            }
            robot_cmd.motor[0].Kp = 50.0;
            robot_cmd.motor[1].Kp = 50.0;
        
            if(current_time > temp_time + 1.0){
                phase = 3;
            }
        }

        else if(phase == 3){
            stopController();
        }
        
        sendCommand();
    }
};


int main(int argc, char** argv){
    ros::init(argc, argv, "Hopping_Controller");
    Prototype2020_BaseController *controller = new Hopping_Controller;
    ros::spin();
    return 0;
}
