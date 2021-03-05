// 単脚用プログラム
// q = [15.0, -30.0]T[deg] (initialPose) にするコントローラ
// 何かしらの動作をさせる前にこのコントローラでinitialPoseにすること！

#include <AKROS_bridge_controller/Prototype2020_BaseController.h>

class initialPose_Controller : public Prototype2020_BaseController{
private:
    const double settingTime = 3.0;
    Eigen::VectorXd q_target;
public:
    initialPose_Controller(void){
        ROS_INFO("initialPose_Controller_constructor");
        q_target.resize(JOINTNUM);
        for(int i=0; i<JOINTNUM; i++){
            q_target[i] = deg2rad(initialPose[i]);
        }
    };

    virtual void loop(const ros::TimerEvent& e) override{
        double current_time = getTime();

        if(phase == 0){
            if(initializeFlag == false){
                ROS_INFO("phase 0");
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
                ros::shutdown();
            }
        }

        sendCommand();
    }
};




int main(int argc, char** argv){
    ros::init(argc, argv, "initialPose_Controller");
    Prototype2020_BaseController *controller = new initialPose_Controller;
    ros::spin();
    return 0;
}