#include "math-tools.h"

// 获取输入数值的符号（即正负号）
int getSign(float input) {
    if (input > 0) return 1;
    else if (input < 0) return -1;
    else return 0;
}

// 角度转弧度
float deg2rad(float deg) {
    return deg / 180.0 * M_PI;
}

// 弧度转角度
float rad2deg(float rad) {
    return rad / M_PI * 180.0;
}

// 计算角度偏差值
float calDeltaAng(float delta_ang) {
    while (fabs(delta_ang) > 180) {
        if (delta_ang > 0) delta_ang -= 360;
        else delta_ang += 360;
    }
    return delta_ang;
}