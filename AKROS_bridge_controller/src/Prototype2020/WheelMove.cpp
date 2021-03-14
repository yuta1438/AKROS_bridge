// 単脚用プログラム
// 車輪による移動
#include <AKROS_bridge_controller/Prototype2020_BaseController.h>

class wheelMove_Controller : public Prototype2020_BaseController{
private:
    const double marginTime = 1.0;       // 待機時間[s]
    const double settingTime = 2.0;      // initialPose遷移時間[s]
    const double movingTime = 1.0;       // 移動時間[s]
    const double movingDistance = 1.0;   // 目標移動距離[m]

public:
    wheelMove_Controller(void){
        ROS_INFO("wheel Move controller start ...");
    }

    virtual void loop(const ros::TimerEvent& e) override {
        double current_time = getTime();

        // initialPoseに遷移
        if(phase == 0){
            if(!initializeFlag){
                joint_Interpolator.clear();
                joint_Interpolator.appendSample(current_time, q_init);
                joint_Interpolator.appendSample(current_time+settingTime, q_initialPose);
                joint_Interpolator.update();
                initializeFlag = true;
            }

            qref = joint_Interpolator.interpolate(current_time);

            if(current_time > joint_Interpolator.domainUpper()){
                initializeFlag = false;
                ROS_INFO("Enter key to start moving ...");
                char buf;
                std::cin >> buf;    // 待ち
                timer_start();
                phase = 1;
            }
        }

        // 車輪移動を開始
        else if(phase == 1){
            if(initializeFlag == false){
                joint_Interpolator.clear();
                joint_Interpolator.appendSample(current_time, q_initialPose);
                q_initialPose[WHEEL] = 2*movingDistance/wheel_D;
                joint_Interpolator.appendSample(current_time+movingTime, q_initialPose);
                joint_Interpolator.update();

                robot_cmd.motor[WHEEL].Kp = 0.0;
                initializeFlag = true;
            }

            qref = joint_Interpolator.interpolate(current_time);
            
            if(current_time > joint_Interpolator.domainUpper()){
                initializeFlag = false;
                robot_cmd.motor[WHEEL].velocity = 0.0;
                phase = 2;
                stopController();
            }
        }
        sendCommand();
    }
};


int main(int argc, char** argv){
    ros::init(argc, argv, "wheelMove_Controller");
    Prototype2020_BaseController *controller = new wheelMove_Controller;
    ros::spin();
    return 0;
}