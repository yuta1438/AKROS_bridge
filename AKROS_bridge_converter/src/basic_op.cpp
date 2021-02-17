#include <AKROS_bridge_converter/basic_op.h>

float fmaxf(float x, float y){
    /// Returns maximum of x, y ///
    return (((x)>(y))?(x):(y));
}

float fminf(float x, float y){
    /// Returns minimum of x, y ///
    return (((x)<(y))?(x):(y));
}

// float -> uint
int float_to_uint(float x, float x_min, float x_max, int bits){
    /// Converts a float to an unsigned int, given range and number of bits ///
    float span = x_max - x_min;
    float offset = x_min;
    return (int) ((x-offset)*((float)((1<<bits)-1))/span);
}
    
// uint -> float
float uint_to_float(int x_int, float x_min, float x_max, int bits){
    /// converts unsigned int to float, given range and number of bits ///
    float span = x_max - x_min;
    float offset = x_min;
    return ((float)x_int)*span/((float)((1<<bits)-1)) + offset;
}


// rad -> deg
float rad2deg(float rad){
    return rad*(180.0f / M_PI);
}

// deg -> rad
float deg2rad(float deg){
    return deg*(M_PI / 180.0f);
}

// bがtrueなら符号を変える
float signChange(float value, bool b){
    return b ? -value : value;
}

// 関節のオフセット角度をデジタル値に変換
int convertOffset(float offset, float x_min, float x_max, int bits){
    float span = x_max - x_min;
    return (int)((offset / span)*((float)((1<<bits)-1)));
}