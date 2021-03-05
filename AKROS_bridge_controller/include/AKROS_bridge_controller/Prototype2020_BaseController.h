#ifndef PROTOTYPE2020_BASECONTROLLER_H_
#define PROTOTYPE2020_BASECONTROLLER_H_

#include <ros/ros.h>
#include <Eigen/Core>
#include <Eigen/Dense>
#include <AKROS_bridge_controller/Interpolator.h>
#include <AKROS_bridge_msgs/motor_cmd.h>
#include <AKROS_bridge_msgs/currentState.h>

#define deg2rad(deg) (((deg) / 360) * 2 * M_PI)
#define rad2deg(rad) (((rad) / 2 / M_PI) * 360)


class Prototype2020_BaseController{
private:    // このクラスでしか見られない
    const double controller_frequency = 100.0;
    std::string topic_name = "motor_cmd";
    ros::Time t_start;

    ros::ServiceClient currentState_client;
    AKROS_bridge_msgs::currentState currentState_srv;

protected:  // このクラスと派生クラスからしか見れない
    // Node parameters
    ros::NodeHandle nh;
    ros::Timer timer;
    ros::Publisher pub;
    AKROS_bridge_msgs::motor_cmd robot_cmd;

    cnoid::Interpolator<Eigen::VectorXd> joint_Interpolator;
    cnoid::Interpolator<Eigen::Vector2d> leg_Interpolator;
    
    Eigen::Vector2d leg_pos;
    Eigen::VectorXd qref, qref_old, q_init;

    // Robot Parameters
    const double l1 = 0.20;  // 大腿脚長
    const double l2 = 0.20;  // 下腿脚長
    const double wheel_D = 0.1;  // 車輪直径

    const double initialPose[3] = {15.0f, -30.0f, 0.0};
    bool initializeFlag = false;
    int phase = 0;

    enum robot_joint{
        HIP,
        KNEE,
        WHEEL,
        JOINTNUM
    };

public:
    Prototype2020_BaseController(void);
    void timer_start(void);
    void read_State(void);
    double getTime(void);
    void stopController(void);

    void sendCommand(void);

    Eigen::Vector2d solve_sagittal_FK(Eigen::Vector2d q);
    Eigen::Vector2d solve_sagittal_IK(Eigen::Vector2d p, bool &error);

    virtual void loop(const ros::TimerEvent& e) = 0;
    virtual ~Prototype2020_BaseController(){};
};

#endif