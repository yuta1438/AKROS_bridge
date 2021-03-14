// 単脚用プログラム
// 任意の関節角度に動かすプログラム
#include <AKROS_bridge_controller/Prototype2020_BaseController.h>

class jointsMove_Controller : public Prototype2020_BaseController{
private:
    const double settingTime = 3.0;
    const double q_target_deg[3] = {30.0, -60.0, 0.0};
    Eigen::VectorXd q_target;
public:
    jointsMove_Controller(void){
        q_target.resize(JOINTNUM);
        q_target << deg2rad(q_target_deg[0]), deg2rad(q_target_deg[1]), deg2rad(q_target_deg[2]);
        ROS_INFO("Transition to target position ...");
    }

    virtual void loop(const ros::TimerEvent& e) override{
        double current_time = getTime();

        if(phase == 0){
            if(initializeFlag == false){
                joint_Interpolator.clear();
                joint_Interpolator.appendSample(current_time, q_init);
                joint_Interpolator.appendSample(current_time+settingTime, q_target);
                joint_Interpolator.update();
                initializeFlag = true;
            }
            qref = joint_Interpolator.interpolate(current_time);

            if(current_time > joint_Interpolator.domainUpper()){
                initializeFlag = false;
                phase = 1;
                stopController();
            }
        }
        sendCommand();
    }
};


int main(int argc, char** argv){
    ros::init(argc, argv, "jointsMove_Controller");
    Prototype2020_BaseController *controller = new jointsMove_Controller;
    ros::spin();
    return 0;
}