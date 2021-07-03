// @author Hayato Ota
// 屈伸動作を行いながら車輪移動で前後に動く動作
// 屈伸動作は「Prototype2020_bending_stretching」と同じ
// 車輪への位置指令は緩やかな正弦波指令とする．
// 屈伸時の車輪への影響を忘れずに！

#include <AKROS_bridge_controller/Prototype2020_BaseController.h>

class Bending_Stretching_wheel : public Prototype2020_BaseController{
private:
    const double marginTime = 2.0;
    const double settingTime = 2.0;
    const double movingTime = 15.0;

    double leg_frequency;
    double leg_amplitude;
    double leg_omega;

    const double q_extension_deg[2] = {15.0f, -30.0f};
    const double q_flexion_def[2] = {60.0f, -120.0f};

    Eigen::Vector2d pref, p_center;
    Eigen::VectorXd q_extension;
    Eigen::Vector2d p_extension;

    // wheel locomotion settings
    double wheel_frequency;
    double wheel_amplitude;
    double wheel_omega;

public:
    Bending_Stretching_wheel(double leg_A_, double leg_f_, double wheel_A_, double wheel_f_)
     : leg_amplitude(leg_A_), leg_frequency(leg_f_), wheel_amplitude(wheel_A_), wheel_frequency(wheel_f_)
    {
        leg_omega = 2*M_PI*leg_f_;
        wheel_omega = 2*M_PI*wheel_frequency;

        q_extension.resize(JOINTNUM);

        q_extension << deg2rad(q_extension_deg[0]), deg2rad(q_extension_deg[1]), 0.0;
        solve_sagittal_FK(q_extension.head<2>(), p_extension);
        p_center << p_extension[0], p_extension[1]+leg_amplitude;
    }

    virtual void loop(const ros::TimerEvent& e) override {
        double current_time = getTime();

        if(phase == 0){
            if(!initializeFlag){
                ROS_INFO("Preparing to start bending_stretching & wheel_locomotion ...");
                joint_Interpolator.clear();
                joint_Interpolator.appendSample(current_time, q_init);
                joint_Interpolator.appendSample(current_time+settingTime, q_extension);
                joint_Interpolator.update();
                initializeFlag = true;
            }
            qref = joint_Interpolator.interpolate(current_time);

            if(current_time > joint_Interpolator.domainUpper()){
                initializeFlag = false;
                ROS_INFO("Enter key to start bending and stretching and wheel locomotion");
                ROS_INFO("leg_amp = %.2f[m], leg_freq = %.2f[Hz]", leg_amplitude, leg_frequency);
                ROS_INFO("wheel_amp = %.2f[m], wheel_freq = %.2f[Hz]", wheel_amplitude, wheel_frequency);
                char buf;
                std::cin >> buf;
                timer_start();
                phase = 1;
            }
        }

        else if(phase == 1){
            
        }
    }
};