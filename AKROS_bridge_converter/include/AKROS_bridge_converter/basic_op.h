#ifndef BASIC_OP_H_
#define BASIC_OP_H_

#include <math.h>

float fmaxf(float x, float y);
float fminf(float x, float y);
int float_to_uint(float x, float x_min, float x_max, int bits);
float uint_to_float(int x_int, float x_min, float x_max, int bits);

float rad2deg(float rad);
float deg2rad(float deg);

// 実数値offsetをデジタル値に変換するための関数
int convertOffset(float offset, float x_min, float x_max, int bits);

#endif