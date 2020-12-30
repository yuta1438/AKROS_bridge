#include <iostream>
#include "include/basic_op/basic_op.h"
#include "include/general_settings.h"

using namespace std;

uint16_t pos = 32767;
uint16_t vel = 2047;
uint16_t eff = 2047;

float pos_float = uint_to_float(pos, P_MIN, P_MAX, POSITION_BIT_NUM);
float vel_float = uint_to_float(vel, V_MIN, V_MAX, VELOCITY_BIT_NUM);
float eff_float = uint_to_float(eff, T_MIN, T_MAX, EFFORT_BIT_NUM);

int main(void){
    // int -> float
    cout << "position(float): " << pos_float << endl;
    cout << "velocity(float): " << vel_float << endl;
    cout << "effort(float)  : " << eff_float << endl;

    cout << endl;

    // float -> int
    cout << "position(int): " << float_to_uint(pos_float, P_MIN, P_MAX, POSITION_BIT_NUM) << endl;
    cout << "velocity(int): " << float_to_uint(vel_float, V_MIN, V_MAX, VELOCITY_BIT_NUM) << endl;
    cout << "effort(int)  : " << float_to_uint(eff_float, T_MIN, T_MAX, EFFORT_BIT_NUM) << endl;
}