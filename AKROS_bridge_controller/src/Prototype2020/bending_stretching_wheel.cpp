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
    const double movingTime = 30.0;

    const double q_extension_deg[2] = {15.0f, -30.0f};
    const double q_flexion_def[2] = {60.0f, -120.0f};

    // legged locomotion settings
    double leg_frequency = 0.5;
    double leg_amplitude = 0.1;
    double leg_omega;
    Eigen::Vector2d pref, p_center;
    Eigen::VectorXd q_extension;
    Eigen::Vector2d p_extension;

    
    // wheel locomotion settings
    double wheel_pos_ref;
    double wheel_frequency = 1 / 15.0f;
    double wheel_amplitude = 1.0 / (wheel_D/2.0);
    double wheel_omega;

public:
    Bending_Stretching_wheel(void){
        leg_omega = 2*M_PI*leg_frequency;
        wheel_omega = 2*M_PI*wheel_frequency;

        q_extension.resize(JOINTNUM);

        q_extension << deg2rad(q_extension_deg[0]), deg2rad(q_extension_deg[1]), 0.0;
        solve_sagittal_FK(q_extension.head<2>(), p_extension);
        p_center << p_extension[0], p_extension[1]+leg_amplitude;
    }

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
                
                // set Wheel_motor to Velocity-control mode
                robot_cmd.motor[2].Kp = 0.0;
                robot_cmd.motor[2].Kd = 3.0;
                
                joint_Interpolator.clear();
                joint_Interpolator.appendSample(current_time, q_init);
                joint_Interpolator.appendSample(current_time+settingTime, q_extension);
                joint_Interpolator.update();
                initializeFlag = true;
            }
            qref = joint_Interpolator.interpolate(current_time);

            if(current_time > joint_Interpolator.domainUpper()){
                initializeFlag = false;
                ROS_INFO("leg_amp = %.2f[m], leg_freq = %.2f[Hz]", leg_amplitude, leg_frequency);
                ROS_INFO("wheel_amp = %.2f[m], wheel_freq = %.2f[Hz]", wheel_amplitude, wheel_frequency);
                ROS_INFO("Enter key to start bending and stretching and wheel locomotion");
                char buf;
                std::cin >> buf;
                timer_start();
                phase = 1;
            }
        }

        else if(phase == 1){
            if(!initializeFlag){
                ROS_INFO("Start controller!");
                pref[0] = p_center[0];
                wheel_pos_ref = 0.0;
                initializeFlag = true;
            }

            pref[1] = p_center[1] - leg_amplitude*cos(leg_omega*current_time);
            wheel_pos_ref = wheel_amplitude - wheel_amplitude * cos(wheel_omega*current_time);

            Eigen::VectorXd q_buf(2);
            solve_sagittal_IK(pref, q_buf);
            qref.head<2>() = q_buf;
            qref[WHEEL] = wheel_pos_ref - (qref[HIP] - q_extension[HIP]);

            if(current_time > movingTime){
                initializeFlag = false;
                ROS_INFO("Enter key to initial Pose");
                char buf;
                std::cin >> buf;
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
            // qref[WHEEL] = -qref[HIP];

            if(current_time > joint_Interpolator.domainUpper()){
                ROS_INFO("controller finished !");
                stopController();
            }
        }

        sendCommand();
    }
};


int main(int argc, char** argv){
    ros::init(argc, argv, "bending_stretching_wheel_controller");

    // use default settings
    if(argc == 1){
        Prototype2020_BaseController *controller = new Bending_Stretching_wheel;
    }
    // enter custom settings
    else if(argc == 5){
        double leg_A_ = atof(argv[1]);
        double leg_f_ = atof(argv[2]);
        double wheel_A_ = atof(argv[3]);
        double wheel_f_ = atof(argv[4]);

        Prototype2020_BaseController *controller = new Bending_Stretching_wheel(leg_A_, leg_f_, wheel_A_, wheel_f_);
    }
    else{
        ROS_ERROR("please enter ");
        ROS_ERROR("1: leg_amplitude [m]");
        ROS_ERROR("2: leg_frequency [Hz]");
        ROS_ERROR("3: wheel_amplitude [m]");
        ROS_ERROR("4: wheel_frequency [Hz]");
    }

    ros::spin();
    return 0;
}