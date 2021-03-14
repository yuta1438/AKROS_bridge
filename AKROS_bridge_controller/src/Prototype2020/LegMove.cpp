// 単脚用プログラム
// 矢状面内で脚を段差の上に動かすプログラム
// このコントローラを行う前にロボットをinitialPoseにすること！

#include <AKROS_bridge_controller/Prototype2020_BaseController.h>
#include <geometry_msgs/Pose2D.h>
class LegMove_Controller : public Prototype2020_BaseController{
private:
    const double marginTime = 1.0;
    const double settingTime = 2.0;
    const double movingTime1 = 0.3;
    const double movingTime2 = 0.2;
    const double q_initialize_deg[2] = {10.0, -20.0};

    Eigen::VectorXd q_initialize;
    Eigen::Vector2d p_initialize;

    geometry_msgs::Pose2D leg_pos;
    ros::Publisher leg_pos_pub;
public:
    LegMove_Controller(void){
        leg_pos_pub = nh.advertise<geometry_msgs::Pose2D>("leg_pos", 1);

        q_initialize.resize(LEG_JOINTNUM);
        q_initialize << deg2rad(q_initialize_deg[0]), deg2rad(q_initialize_deg[1]);
    
        solve_sagittal_FK(q_initialize, p_initialize);

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
                ROS_INFO("Enter key to start moving ...");
                char buf;
                std::cin >> buf;    // 待ち
                phase = 1;
                timer_start();
            }
        }

        else if(phase == 1){
            if(initializeFlag == false){
                Eigen::Vector2d delta_p1(0.045, 0.24);
                Eigen::Vector2d delta_p2(0.13, 0.23);

                leg_Interpolator.clear();
                leg_Interpolator.appendSample(current_time, p_initialize);
                leg_Interpolator.appendSample(current_time+movingTime1, p_initialize + delta_p1);
                leg_Interpolator.appendSample(current_time+movingTime1+movingTime2, p_initialize + delta_p2);
                leg_Interpolator.update();
                initializeFlag = true;
            }

            Eigen::VectorXd q_buff;
            q_buff.resize(LEG_JOINTNUM);
            solve_sagittal_IK(leg_Interpolator.interpolate(current_time), q_buff);
            qref.head<LEG_JOINTNUM>() = q_buff;

            // debug
            // -----
            Eigen::Vector2d buff = leg_Interpolator.interpolate(current_time);
            leg_pos.x = buff[0];
            leg_pos.y = buff[1];
            leg_pos_pub.publish(leg_pos);
            // -----

            if(current_time > leg_Interpolator.domainUpper()){
                initializeFlag = false;
                phase = 2;
                stopController();
            }
        }
        sendCommand();
    }
};


int main(int argc, char** argv){
    ros::init(argc, argv, "Legmove_Controller");
    Prototype2020_BaseController *controller = new LegMove_Controller;
    ros::spin();
    return 0;
}