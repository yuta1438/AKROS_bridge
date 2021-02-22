#ifndef PROTOTYPE2020_H_
#define PROTOTYPE2020_H_
#include <ros/ros.h>
#include <Eigen/Core>
#include <Eigen/Dense>

#define HIP     0
#define KNEE    1
#define WHEEL   2

#define deg2rad(deg) (((deg) / 360) * 2 * M_PI)
#define rad2deg(rad) (((rad) / 2 / M_PI) * 360)

// Robot Parameters
static const double l1 = 0.20;  // 大腿脚長
static const double l2 = 0.20;  // 下腿脚長
static const double wheel_D = 0.1;  // 車輪直径

static const double initialPose[3] = {15.0f, -30.0f, 0.0};

// 矢状平面での順運動学
Eigen::Vector2d solve_sagittal_FK(Eigen::Vector2d q){
    Eigen::Vector2d p;
    p[0] = -l1*sin(q[0]) - l2*sin(q[0]+q[1]);
    p[1] = -l1*cos(q[0]) - l2*cos(q[0]+q[1]);
    return p;
}


// 矢状平面での逆運動学
// 引数には脚先位置p = [x, z]Tを渡すこと！
Eigen::Vector2d solve_sagittal_IK(Eigen::Vector2d p){
    Eigen::Vector2d q;
    double L = sqrt(p[0]*p[0] + p[1]*p[1]);
    q = Eigen::Vector2d::Zero();

    q[0] = -atan2(p[0], -p[1]) + acos((pow(L, 2) + pow(l1, 2) - pow(l2, 2)) / (2 * L * l1));
    q[1] = -M_PI + acos((pow(l1, 2.0) + pow(l2, 2.0) - pow(L, 2)) / (2 * l1 * l2));

    return q;
}

// 矢状平面での逆運動学
// エラー確認付き.　特異姿勢となってしまう場合はerrorが真となる;
// 引数には脚先位置p = [x, z]Tを渡すこと！
Eigen::Vector2d solve_sagittal_IK(Eigen::Vector2d p, bool &error){
    Eigen::Vector2d q;
    double L = sqrt(p[0]*p[0] + p[1]*p[1]);

    error = (L>(l1+l2)) ? true : false;

    q = Eigen::Vector2d::Zero();

    q[0] = -atan2(p[0], -p[1]) + acos((pow(L, 2) + pow(l1, 2) - pow(l2, 2)) / (2 * L * l1));
    q[1] = -M_PI + acos((pow(l1, 2.0) + pow(l2, 2.0) - pow(L, 2)) / (2 * l1 * l2));

    return q;
}
#endif