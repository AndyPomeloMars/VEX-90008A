#include "timer.h"
#include "robot-config.h"

MyTimer::MyTimer() {
    startTime = Brain.Timer.value(); // 将 V5 Brain 的当前系统时间（单位：秒）记录为计时器的起始时间
}

MyTimer::MyTimer(float init) {
    startTime = Brain.Timer.value() + init / 1000; // 将起始时间设置为 当前系统时间 加上一个偏移量。(单位：ms)
}

void MyTimer::reset() {
    startTime = Brain.Timer.value(); // 将 V5 Brain 的当前系统时间重新赋值给 startTime 变量
}

int MyTimer::getTime() const {
  return floor((Brain.Timer.value() - startTime) * 1000); // 计算出从起始时间到现在的秒数差
}