// Prototype2020を動かす際にはこのクラスを継承すること！
// 今までは異なるコントローラごとに記述していたが，同じ変数，関数を使っており記述するのが面倒だった
// なので，よく使用する変数，関数をこのクラスにまとめてそのコントローラ特有のものはそちらに記述する...というスタイルを取ることになった！

#ifndef PROTOTYPE2020_BASECONTROLLER_H_
#define PROTOTYPE2020_BASECONTROLLER_H_

#include <ros/ros.h>
#include <Eigen/Core>
#include <Eigen/Dense>
#include <AKROS_bridge_controller/Interpolator.h>
#include <AKROS_bridge_msgs/motor_cmd.h>
#include <AKROS_bridge_msgs/currentState.h>

#ifdef USE_TIMESTAMP
#include <AKROS_bridge_msgs/motor_cmd_timestamped.h>
#endif

// Macros
#define deg2rad(deg) (((deg) / 360) * 2 * M_PI)
#define rad2deg(rad) (((rad) / 2 / M_PI) * 360)
#define RANGE_CHECK(x,min,max) (x<min  ? false : x<max ? true : false)


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

    cnoid::Interpolator<Eigen::VectorXd> joint_Interpolator;    // 関節空間での補間器
    cnoid::Interpolator<Eigen::Vector2d> leg_Interpolator;      // 作業空間での補間器
    
    Eigen::Vector2d leg_pos;    // 脚先目標位置
    Eigen::VectorXd qref;       // 目標関節角ベクトル
    Eigen::VectorXd qref_old;   // 1ステップ前のqref
    Eigen::VectorXd q_init;     // 初期関節角ベクトル

    // Robot Parameters
    const double l1 = 0.20;  // 大腿脚長
    const double l2 = 0.20;  // 下腿脚長
    const double wheel_D = 0.1;  // 車輪直径

    const double Hip_limit_deg[2] = {0.0, 0.0};
    const double Knee_limit[2] = {deg2rad(-135.0), deg2rad(-15.0)};

    // 脚長制限
    const double Lmin = 0.153;
    const double Lmax = 0.396;

    // 初期姿勢(initialPose)
    const double initialPose_deg[3] = {15.0f, -30.0f, 0.0};
    Eigen::VectorXd q_initialPose;

    bool initializeFlag = false;
    int phase = 0;

    enum robot_joint{
        HIP,        // = 0
        KNEE,       // = 1
        WHEEL,      // = 2
        JOINTNUM,   // = 3
        LEG_JOINTNUM = 2
    };

public:
    Prototype2020_BaseController(void);
    void timer_start(void);
    void read_State(Eigen::VectorXd&);
    double getTime(void);

    void stopController(void);
    void sendCommand(void);

    bool solve_sagittal_FK(const Eigen::VectorXd& q, Eigen::Vector2d& p);
    bool solve_sagittal_IK(const Eigen::Vector2d& p, Eigen::VectorXd& q);

    virtual void loop(const ros::TimerEvent& e) = 0;
    virtual ~Prototype2020_BaseController(){};
};

#endif