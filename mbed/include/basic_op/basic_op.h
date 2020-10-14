#ifndef BASIC_OP_H_
#define BASIC_OP_H_

float fmaxf(float x, float y);
float fminf(float x, float y);
int float_to_uint(float x, float x_min, float x_max, int bits);
float uint_to_float(int x_int, float x_min, float x_max, int bits);

#endif