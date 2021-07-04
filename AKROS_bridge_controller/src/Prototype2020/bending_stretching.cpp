// 単脚用プログラム
// 矢状平面内での屈伸運動(Bending & Stretching)
// 石沢から振幅を指定できるようにしたいとの要望あり
// ただし，initialPoseを基準とする！

#include <AKROS_bridge_controller/Prototype2020_BaseController.h>
#include <geometry_msgs/Point32.h>

class Bending_Stretching_Controller : public Prototype2020_BaseController{
private:
    const double marginTime = 2.0;
    const double settingTime = 3.0;
    const double movingTime = 15.0;

    // const double wave_frequency = 1.0;       // 脚先正弦波指令の周波数[Hz]
    // const double amplitude = 0.1;           // 正弦波振幅[m]
    double wave_frequency;
    double amplitude;
    double omega;

    const double q_extention_deg[2] = {15.0f, -30.0f};   // 一番Kneeを伸ばすポーズ
    const double q_flexion_deg[2] = {60.0f, -120.0f};    // 一番Kneeを曲げるポーズ

    Eigen::Vector2d pref, p_center;
    Eigen::VectorXd q_extension;
    Eigen::Vector2d p_extension;

    geometry_msgs::Point32 foot_position;
    ros::Publisher foot_position_pub;

public:
    // コンストラクタ
    Bending_Stretching_Controller(double A_, double f_){
        amplitude = A_;
        wave_frequency = f_;
        foot_position_pub = nh.advertise<geometry_msgs::Point32>("Target_foot_position", 1);
        omega = 2*M_PI*wave_frequency;
        q_extension.resize(JOINTNUM);
        // 各種計算
        q_extension << deg2rad(q_extention_deg[0]), deg2rad(q_extention_deg[1]), 0.0;
        solve_sagittal_FK(q_extension.head<2>(), p_extension);
        p_center << p_extension[0], p_extension[1]+amplitude;   // 振動中心点
    }

    virtual void loop(const ros::TimerEvent& e) override {
        double current_time = getTime();

        // 
        if(phase == 0){
            if(initializeFlag == false){
                ROS_INFO("preparing to bending & stretching ...");
                joint_Interpolator.clear();
                joint_Interpolator.appendSample(current_time, q_init);
                joint_Interpolator.appendSample(current_time+settingTime, q_extension);
                joint_Interpolator.update();
                initializeFlag = true;
            }

            qref = joint_Interpolator.interpolate(current_time);

            if(current_time > joint_Interpolator.domainUpper()){
                initializeFlag = false;
                ROS_INFO("Enter key to start bending and stretching");
                ROS_INFO("(Freq = %.2f[Hz], Amp = %.2f[m])", wave_frequency, amplitude);
                char buf;
                std::cin >> buf;    // 待ち
                timer_start();
                phase = 1;
            }
        }

        else if(phase == 1){
            if(initializeFlag == false){
                ROS_INFO("phase 2 : start flexion");
                pref[0] = p_center[0];
                foot_position.y = 0.0;
                initializeFlag = true;
            }
            pref[1] = p_center[1] - amplitude * cos(omega * (current_time));
            
            Eigen::VectorXd q_buf(2);
            solve_sagittal_IK(pref, q_buf);
            qref.head<2>() = q_buf;
            qref[WHEEL] = -(qref[HIP] - q_initialPose[HIP]);    // 車輪が地面に対して動かないように制御

            // 目標脚先位置をPublish
            foot_position.x = pref[0];
            foot_position.z = pref[1];
            foot_position_pub.publish(foot_position);


            if(current_time > movingTime){
                initializeFlag = false;
                ROS_INFO("Enter key to return to initial Pose ...");
                char buf;
                std::cin >> buf;    // 待ち
                phase = 2;
            }
        }

        else if(phase == 2){
            if(initializeFlag == false){
                ROS_INFO("phase 2 : finish and move to initial pose");
                joint_Interpolator.clear();
                joint_Interpolator.appendSample(current_time, qref);
                joint_Interpolator.appendSample(current_time + 3.0, q_extension);
                joint_Interpolator.update();
                initializeFlag = true;
            }
            
            qref = joint_Interpolator.interpolate(current_time);
            qref[WHEEL] = -qref[HIP];

            if(current_time > joint_Interpolator.domainUpper()){
                ROS_INFO("controller finished !");
                stopController();
            }
        }
        sendCommand();
    }
};



int main(int argc, char** argv){
    ros::init(argc, argv, "bending_stretching_Controller");

    if(argc != 3){
        ROS_ERROR("Please enter amplitude and frequency of wave !");
    }

    double A = atof(argv[1]);
    double f = atof(argv[2]);
    Prototype2020_BaseController *controller = new Bending_Stretching_Controller(A, f);
    ros::spin();
    return 0;
}
