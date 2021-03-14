// 単脚用プログラム
// 矢状平面内でx軸方向にオフセットをもたせた屈伸運動(Bending & Stretching)

#include <AKROS_bridge_controller/Prototype2020_BaseController.h>
#include <geometry_msgs/Pose2D.h>
class Bending_Stretching_offset_Controller : public Prototype2020_BaseController{
private:
    // 屈伸開始時の脚先位置
    const double offset_x = 0.2;
    const double offset_z = -0.3;

    const double marginTime = 2.0;
    const double settingTime = 3.0;
    const double movingTime = 30.0;

    const double wave_frequency = 0.5;          // 脚先正弦波指令の周波数[Hz]
    const double wave_amplitude = 0.1;         // 正弦波振幅[m]
    const double omega = 2*M_PI*wave_frequency; // 正弦波の角振動数[rad/s]

    Eigen::VectorXd q_initialize;
    Eigen::Vector2d pref;
    Eigen::Vector2d p_init;
    Eigen::Vector2d p_center;   // 正弦波の中心点
    Eigen::Vector2d p_top;      // 正弦波の最高点
    Eigen::Vector2d p_bottom;   // 正弦波の最下点(= offset点)

    geometry_msgs::Pose2D leg_pos;
    ros::Publisher leg_pos_pub;

public:
    Bending_Stretching_offset_Controller(void){
        // ROS_INFO("Child Controller Constructor");
        q_initialize.resize(LEG_JOINTNUM);
        p_bottom << offset_x, offset_z;   // 振動中心点

        leg_pos_pub = nh.advertise<geometry_msgs::Pose2D>("leg_pos", 1);

        // 振動してる際に脚先可動範囲を超えないかどうかチェック
        // もし超えたらロボットを動かさずにコントローラを終了
        Eigen::VectorXd q_temp;
        q_temp.resize(JOINTNUM);

        p_center = p_bottom;
        p_top = p_bottom;
        p_center[1] += wave_amplitude;
        p_top[1] = p_center[1] + wave_amplitude;
        if(!solve_sagittal_IK(p_top, q_temp) || !solve_sagittal_IK(p_bottom, q_temp)){
            ROS_ERROR("I cannot reach this trajectory !");
            stopController();
        }

        qref = q_init;
    }


    virtual void loop(const ros::TimerEvent& e) override {
        double current_time = getTime();

        // 基準の姿勢に
        if(phase == 0){
            if(initializeFlag == false){
                // ROS_INFO("phase 0");
                solve_sagittal_IK(p_bottom, q_initialize);
                joint_Interpolator.clear();
                joint_Interpolator.appendSample(current_time, q_init.head<LEG_JOINTNUM>());
                joint_Interpolator.appendSample(current_time+settingTime, q_initialize);
                joint_Interpolator.update();
                initializeFlag = true;
            }
            
            qref.head<LEG_JOINTNUM>() = joint_Interpolator.interpolate(current_time);
            qref[WHEEL] = q_init[WHEEL];

            if(current_time > joint_Interpolator.domainUpper()){
                initializeFlag = false;
                ROS_INFO("Enter key to start moving ...");
                char buf;
                std::cin >> buf;    // 待ち
                timer_start();
                phase = 1;
            }
        }

        // 屈伸運動
        else if(phase == 1){
            if(initializeFlag == false){    // 各phaseの最初の一回だけ実行
                ROS_INFO("phase 2 : start flexion");
                pref[0] = p_bottom[0];
                initializeFlag = true;
                q_init = qref;
            }
            pref[1] = p_center[1] - wave_amplitude * cos(omega * (current_time));
            
            Eigen::VectorXd q_buff;
            q_buff.resize(LEG_JOINTNUM);
            solve_sagittal_IK(pref, q_buff);
            qref.head<2>() = q_buff;
            qref[WHEEL] = q_init[WHEEL] - (qref[HIP] - q_init[HIP]);

            Eigen::Vector2d p_buff;
            solve_sagittal_FK(qref.head<LEG_JOINTNUM>(), p_buff);
            leg_pos.x = p_buff[0];
            leg_pos.y = p_buff[1];

            leg_pos_pub.publish(leg_pos);

            if(current_time > movingTime){
                initializeFlag = false;
                ROS_INFO("controller finished !");
                stopController();
            }
        }
        sendCommand();
    }
};


int main(int argc, char** argv){
    ros::init(argc, argv, "bending_stretching_offset_Controller");
    Prototype2020_BaseController *controller = new Bending_Stretching_offset_Controller;
    ros::spin();
    return 0;
}