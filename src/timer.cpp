#include "timer.h"
#include "robot-config.h"

MyTimer::MyTimer() {
    startTime = Brain.Timer.value();
}

MyTimer::MyTimer(float init) {
    startTime = Brain.Timer.value() + init / 1000;
}

void MyTimer::reset() {
    startTime = Brain.Timer.value();
}

int MyTimer::getTime() const {
  return floor((Brain.Timer.value() - startTime) * 1000);
}