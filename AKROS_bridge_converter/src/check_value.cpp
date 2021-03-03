// 変換関数の確認用プログラム

#include <ros/ros.h>
#include <iostream>
#include <AKROS_bridge_converter/basic_op.h>
#include <AKROS_bridge_converter/motor_status.h>

int main(int argc, char** argv){
    float Hip_offset = 70.7;    // [deg]
    float Knee_offset = 30.0;   // [deg]

    float Hip_offset_rad = deg2rad(Hip_offset);
    float Knee_offset_rad = deg2rad(Knee_offset);

    std::cout << "Hip offset [rad]: " << Hip_offset_rad << std::endl;
    std::cout << "Knee offset [rad]: " << Knee_offset_rad << std::endl;

    std::cout << convertOffset(Hip_offset_rad, AK10_9_P_MIN, AK10_9_P_MAX, POSITION_BIT_NUM) << std::endl;
    std::cout << convertOffset(Knee_offset_rad, AK10_9_P_MIN, AK10_9_P_MAX, POSITION_BIT_NUM) << std::endl;

    return 0;
}