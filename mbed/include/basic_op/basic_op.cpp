#include "basic_op.h"

// Return maximum of x, y
float fmaxf(float x, float y){
    return (((x)>(y))?(x):(y));
}

// Return minimum of x, y
float fminf(float x, float y){
    return (((x)<(y))?(x):(y));
}

// float(Analog value) -> uint(Digital Value)
int float_to_uint(float x, float x_min, float x_max, int bits){
    // Converts a float to an unsigned int, given range and number of bits
    float span = x_max - x_min;
    float offset = x_min;
    return (int) ((x-offset)*((float)((1<<bits)-1))/span);
}
    
// uint(Digital Value) -> float(Analog Value)
float uint_to_float(int x_int, float x_min, float x_max, int bits){
    // converts unsigned int to float, given range and number of bits
    float span = x_max - x_min;
    float offset = x_min;
    return ((float)x_int)*span/((float)((1<<bits)-1)) + offset;
}