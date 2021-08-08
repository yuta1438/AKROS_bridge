// 単脚用プログラム
// 車輪走行しながら跳躍させるプログラム

#include <AKROS_bridge_controller/Prototype2020_BaseController.h>
#include <geometry_msgs/Point32.h>
class Hopping_wheel_Controller : public Prototype2020_BaseController{
private:
    const double marginTime  = 2.0;            // 待機時間[s]
    const double settingTime = 2.0;            // initialPose遷移時間[s]
    const double wheel_accelerateTime = 1.0;   // 車輪移動時間[s]
    const double wheel_movingTime = 2.0;       // 車輪移動時間[s]
    const double wheel_movingDistance = 0.5;   // 車輪目標移動距離[m]
    const double wheel_maxVelocity = 6.0;      // 車輪目標移動距離[m]
    const double leg_movingTime  = 0.5;        // 跳躍伸展時間[s]
    const double controller_frequency = 100.0; // 制御周期[Hz]

    
    const double q_initialize_deg[2] = {60.0, -120.0};  // 初期関節角度

    Eigen::VectorXd q_initialize;    // 関節角度ベクトル定義
    Eigen::VectorXd wheel_vel;  // 車輪の速度ベクトル定義  
    Eigen::Vector2d p_initialize;    // 位置ベクトル定義

    geometry_msgs::Point32 foot_position;  // 脚先位置のメッセージ定義
    ros::Publisher foot_position_pub;      // 脚先位置のPublisherを定義
public:
    Hopping_wheel_Controller(void){
        foot_position_pub = nh.advertise<geometry_msgs::Point32>("target_foot_position", 1);

        q_initialize.resize(JOINTNUM);    // q_initializeの要素数の変更
        wheel_vel.resize(1);  // wheel_velocityの要素数の変更
        q_initialize << deg2rad(q_initialize_deg[0]), deg2rad(q_initialize_deg[1]), 0.0;  // q_initializeに代入
    
        solve_sagittal_FK(q_initialize.head<2>(), p_initialize);  // 順運動学(q初期角度 → p初期脚先位置)

        qref = q_init;  // q_initをqrefに代入
    }

    virtual void loop(const ros::TimerEvent& e) override {
        double current_time = getTime();  // 現在時刻取得

        // 初期姿勢へ
        if(phase == 0){
            if(!initializeFlag){
                // set Wheel_motor to Velocity-control mode
                robot_cmd.motor[2].Kp = 0.0;
                robot_cmd.motor[2].Kd = 3.0;

                joint_Interpolator.clear();
                joint_Interpolator.appendSample(current_time, q_init);  // 現在の関節角度
                joint_Interpolator.appendSample(current_time+settingTime, q_initialize);  // 初期関節角度
                joint_Interpolator.update();  // 補間
                initializeFlag = true;
            }

            qref = joint_Interpolator.interpolate(current_time);

            if(current_time > joint_Interpolator.domainUpper()){  // 初期関節角度になった後であれば
                initializeFlag = false;
                ROS_INFO("wheel_movingDistance = %.2f[m], wheel_movingTime = %.2f[s]", wheel_movingDistance, wheel_movingTime);
                ROS_INFO("leg_movingTime = %.2f[s]", leg_movingTime);
                ROS_INFO("Enter key to start wheel_locomotion & hopping");                
                char buf;
                std::cin >> buf;  // 待ち
                phase = 1;
                timer_start();
            }
        }

        // 車輪走行して目標速度まで補間
        else if(phase == 1){
            
            /*
            if(initializeFlag == false){
                ROS_INFO("Start controller!");
                joint_Interpolator.clear();
                joint_Interpolator.appendSample(current_time, q_initialize);
                // ROS_INFO("wheel = %.2f[rad]", q_initialize[WHEEL]);
                q_initialize[WHEEL] = wheel_movingDistance/wheel_D;
                // ROS_INFO("wheel delta = %.2f[rad]", q_initialize[WHEEL]);
                joint_Interpolator.appendSample(current_time+wheel_accelerateTime, q_initialize);
                q_initialize[WHEEL] = 2*wheel_movingDistance/wheel_D;
                joint_Interpolator.appendSample(current_time+wheel_movingTime, q_initialize);
                joint_Interpolator.update();
                initializeFlag = true;
            }
            */

            
            if(initializeFlag == false){
                ROS_INFO("Start controller!");
                joint_Interpolator.clear();
                wheel_vel << 0.0;
                joint_Interpolator.appendSample(current_time, wheel_vel);
                wheel_vel << wheel_maxVelocity;
                joint_Interpolator.appendSample(current_time+wheel_accelerateTime, wheel_vel);
                joint_Interpolator.appendSample(current_time+wheel_movingTime, wheel_vel);
                joint_Interpolator.update();
                initializeFlag = true;
            }
            
            // wheel_velocity = joint_Interpolator.interpolate(current_time);
            // cmd.motor[WHEEL].velocity = joint_Interpolator.interpolate(current_time);
            Eigen::VectorXd wheel_buff;
            wheel_buff.resize(1); 
            wheel_buff = joint_Interpolator.interpolate(current_time);
            robot_cmd.motor[WHEEL].velocity = wheel_buff[0];

            // qref = joint_Interpolator.interpolate(current_time);

            if(current_time > joint_Interpolator.domainUpper()){ // 車輪移動時間[s] - 0.5[s]後であれば
                initializeFlag = false;
                phase = 2;
            }
        }

        // 跳躍 + 停止
        else if(phase == 2){
            Eigen::Vector2d delta_p1(0.00, -0.10);              // 跳躍後の脚先位置
            if(initializeFlag == false){
                joint_Interpolator.clear();
                joint_Interpolator.appendSample(current_time, wheel_vel);
                joint_Interpolator.appendSample(current_time+leg_movingTime, wheel_vel);
                joint_Interpolator.update();

                leg_Interpolator.clear();
                leg_Interpolator.appendSample(current_time, p_initialize);  // 初期位置
                leg_Interpolator.appendSample(current_time+leg_movingTime, p_initialize + delta_p1);  // 最終位置
                leg_Interpolator.update();  // 補間
                initializeFlag = true;
            }

            Eigen::VectorXd q_buff;
            q_buff.resize(LEG_JOINTNUM);
            solve_sagittal_IK(leg_Interpolator.interpolate(current_time), q_buff);  // 逆運動学(現在時刻の脚先位置 → 位置関節角度)
            qref.head<LEG_JOINTNUM>() = q_buff;
            // qref[WHEEL] = -qref[HIP];
            // debug
            // -----
            Eigen::Vector2d buff = leg_Interpolator.interpolate(current_time);
            foot_position.x = buff[0];
            foot_position.z = buff[1];
            foot_position_pub.publish(foot_position);
            // -----

            Eigen::VectorXd wheel_buff;
            wheel_buff.resize(1); 
            wheel_buff = joint_Interpolator.interpolate(current_time);
            robot_cmd.motor[WHEEL].velocity = wheel_buff[0] + (-(qref[HIP] - qref_old[HIP]) * controller_frequency);

            if(current_time > joint_Interpolator.domainUpper()){  // 車輪走行後であれば
                initializeFlag = false;
                phase = 3;
            }
        }

        else if(phase == 3){
            if(initializeFlag == false){
                joint_Interpolator.clear();
                wheel_vel << wheel_maxVelocity;
                joint_Interpolator.appendSample(current_time, wheel_vel);
                wheel_vel << 0.0;
                joint_Interpolator.appendSample(current_time+wheel_accelerateTime, wheel_vel);
                joint_Interpolator.update();
                initializeFlag = true;
            }
            Eigen::VectorXd wheel_buff;
            wheel_buff.resize(1); 
            wheel_buff = joint_Interpolator.interpolate(current_time);
            robot_cmd.motor[WHEEL].velocity = wheel_buff[0];

            if(current_time > joint_Interpolator.domainUpper()){
                initializeFlag = false;
                phase = 4;
                robot_cmd.motor[WHEEL].velocity = 0.0;
                ROS_INFO("Stop controller");
                stopController();
            }
        }
        sendCommand();
    }
};

int main(int argc, char** argv){
    ros::init(argc, argv, "Hoppping_wheel_Controller");
    Prototype2020_BaseController *controller = new Hopping_wheel_Controller;
    ros::spin();
    return 0;
}