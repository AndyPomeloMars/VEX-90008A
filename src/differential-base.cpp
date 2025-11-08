#include "differential-base.h"

#include "PID.h"
#include "math-tools.h"
#include "timer.h"
#include "parameters.h"
#include "robot-config.h"
#include "vex.h"

// 宏定义：计算单侧轮组行驶的距离 (单位：毫米 mm)
// 1. (Motor_BaseLF.position(deg) + ... ) / 3.0: 计算单侧三个电机编码器的平均旋转角度。
// 2. * 69.85 * M_PI: 乘以轮子周长 (直径 * PI)，将旋转圈数转换为线性距离。
// 3. / 360.0: 将角度转换为圈数。
// 4. * 0.75: 一个修正系数，用于校准因轮子打滑、负载等原因造成的实际误差。
#define left_pos                                             \
  (Motor_BaseLF.position(deg) + Motor_BaseLM.position(deg) + \
   Motor_BaseLB.position(deg)) /                             \
      3.0 * 69.85 * M_PI / 360.0 * 0.75
#define right_pos                                            \
  (Motor_BaseRF.position(deg) + Motor_BaseRM.position(deg) + \
   Motor_BaseRB.position(deg)) /                             \
      3.0 * 69.85 * M_PI / 360.0 * 0.75

// 宏定义：获取IMU惯性传感器的旋转角度
// IMU.rotation() 是原始读数，除以一个校准常数 IMU_10 再乘以 3600 可能是为了将其转换为特定单位或进行校准。
#define heading IMU.rotation() / IMU_10 * 3600;

// 静态全局变量，用于存储上一次重置时的位置读数，以便计算相对位移
static float left_pos_last = 0, right_pos_last = 0;

// 传感器与状态获取
// 获取左侧轮组自上次重置以来的行驶距离 (mm)。
float getLeftPos() { return left_pos - left_pos_last; }
// 获取右侧轮组自上次重置以来的行驶距离 (mm)。
float getRightPos() { return right_pos - right_pos_last; }
// 获取机器人中心自上次重置以来的前进距离 (mm)，即左右两侧的平均值。
float getForwardPos() { return (getLeftPos() + getRightPos()) / 2; }
// 将左侧轮组的当前位置记为新的零点。
void resetLeftPos() { left_pos_last = left_pos; }
// 将右侧轮组的当前位置记为新的零点。
void resetRightPos() { right_pos_last = right_pos; }
// 将两侧轮组的当前位置都记为新的零点。
void resetForwardPos() {
  resetLeftPos();
  resetRightPos();
}

// 获取机器人当前的绝对朝向角度 (单位：度)。
// 顺时针为正，逆时针为负，会累加超过360度。
float getHeading() { return heading; }

// 底层电机控制
// 以指定的功率百分比驱动左侧所有电机
void moveLeft(float _input) {
  if (fabs(_input) > 100) _input = getSign(_input) * 100;
  // 使用电压控制模式，将百分比(-100~100)乘以127，转换为毫伏(mV)单位的电压值(近似-12700mV~12700mV)
  Motor_BaseLF.spin(directionType::fwd, (int)127 * _input, voltageUnits::mV);
  Motor_BaseLM.spin(directionType::fwd, (int)127 * _input, voltageUnits::mV);
  Motor_BaseLB.spin(directionType::fwd, (int)127 * _input, voltageUnits::mV);
}
// 以指定的速度百分比驱动左侧所有电机 (使用电机内置的速度单位)
void moveLeftVel(float _input) {
  if (fabs(_input) > 100) _input = getSign(_input) * 100;
  Motor_BaseLF.spin(directionType::fwd, (int)_input, velocityUnits::pct);
  Motor_BaseLM.spin(directionType::fwd, (int)_input, velocityUnits::pct);
  Motor_BaseLB.spin(directionType::fwd, (int)_input, velocityUnits::pct);
}
// 锁定左侧所有电机 (hold模式，电机将抵抗外力保持位置)
void lockLeft(void) {
  Motor_BaseLF.stop(vex::brakeType::hold);
  Motor_BaseLM.stop(vex::brakeType::hold);
  Motor_BaseLB.stop(vex::brakeType::hold);
}
// 解锁左侧所有电机 (coast模式，电机停止后可自由转动)
void unlockLeft(void) {
  Motor_BaseLF.stop(vex::brakeType::coast);
  Motor_BaseLM.stop(vex::brakeType::coast);
  Motor_BaseLB.stop(vex::brakeType::coast);
}

// 以指定的功率百分比驱动右侧所有电机。
void moveRight(float _input) {
  if (fabs(_input) > 100) _input = getSign(_input) * 100;
  Motor_BaseRF.spin(directionType::fwd, (int)127 * _input, voltageUnits::mV);
  Motor_BaseRM.spin(directionType::fwd, (int)127 * _input, voltageUnits::mV);
  Motor_BaseRB.spin(directionType::fwd, (int)127 * _input, voltageUnits::mV);
}
// 以指定的速度百分比驱动右侧所有电机 (使用电机内置的速度单位)。
void moveRightVel(float _input) {
  if (fabs(_input) > 100) _input = getSign(_input) * 100;
  Motor_BaseRF.spin(directionType::fwd, (int)_input, velocityUnits::pct);
  Motor_BaseRM.spin(directionType::fwd, (int)_input, velocityUnits::pct);
  Motor_BaseRB.spin(directionType::fwd, (int)_input, velocityUnits::pct);
}
// 锁定右侧所有电机 (hold模式)。
void lockRight(void) {
  Motor_BaseRF.stop(vex::brakeType::hold);
  Motor_BaseRM.stop(vex::brakeType::hold);
  Motor_BaseRB.stop(vex::brakeType::hold);
}
// 解锁右侧所有电机 (coast模式)。
void unlockRight(void) {
  Motor_BaseRF.stop(vex::brakeType::coast);
  Motor_BaseRM.stop(vex::brakeType::coast);
  Motor_BaseRB.stop(vex::brakeType::coast);
}

// 高层组合控制
// 让底盘以指定功率前进或后退 (两侧轮子同速同向)
void moveForward(float _input) {
  moveLeft(_input);
  moveRight(_input);
}
// 让底盘以指定功率顺时针旋转 (两侧轮子同速反向)
void moveClockwise(float _input) {
  moveLeft(_input);
  moveRight(-_input);
}
// 锁定整个底盘 (刹车模式)。
void lockBase(void) {
  lockLeft();
  lockRight();
}
// 解锁整个底盘 (滑行模式)。
void unlockBase(void) {
  unlockLeft();
  unlockRight();
}

// 使用街机模式（单摇杆或双摇杆组合）控制底盘移动
// turn_sensitivity 转向灵敏度系数，用于调整转向的快慢
void arcadeControl(int forward_axis, int turn_axis, float turn_sensitivity = 0.5) {
  // 计算左右两侧轮子的最终速度
  // 前进/后退是基础速度，转向是在这个基础上进行加减
  int left_power = forward_axis + (turn_axis * turn_sensitivity);
  int right_power = forward_axis - (turn_axis * turn_sensitivity);

  // 调用底层函数来驱动电机
  moveLeft(left_power);
  moveRight(right_power);
}

// 自动程序动作模块
// 前进/后退动作
// 在指定时间内，让前进功率从初始值线性增加/减少到最终值 (软启动/停止)
void softStartTimerForward(float _power_init, float _power_final, int _duration) {
  auto timer = MyTimer();
  float step = (_power_final - _power_init) / _duration;
  float error = 0, feedback = 0;
  float direction = getHeading();
  while (timer.getTime() < _duration) {
    error = getHeading() - direction; // 计算航向误差
    feedback = error * 0.0;
    feedback = fabs(feedback) > 20 ? 20 * getSign(feedback) : feedback;
    float power = _power_init + timer.getTime() * step;
    moveRight(power + feedback);
    moveLeft(power - feedback);
    this_thread::sleep_for(5);
  }
  unlockBase();
}
// 以固定功率前进指定时间，并进行简单的航向保持。
void timerForward(float _power, int _duration) {
  auto timer = MyTimer();
  float error = 0, feedback = 0;
  float direction = getHeading();
  while (timer.getTime() < _duration) {
    error = getHeading() - direction;
    feedback = error * 2.0; // P控制器修正航向
    // 限制修正量的最大值，防止过度转向
    feedback = fabs(feedback) > fabs(_power) * 0.15 ? fabs(_power) * 0.15 * getSign(feedback) : feedback;
    moveRight(_power + feedback);
    moveLeft(_power - feedback);
    this_thread::sleep_for(5);
  }
  unlockBase();
}
// 以固定功率前进指定时间，并转向指定的相对角度
void timerForward(float _power, int _duration, float _heading) {
  auto timer = MyTimer();
  float error = 0, feedback = 0;
  float direction = getHeading();
  while (timer.getTime() < _duration) {
    error = getHeading() - direction - _heading;
    feedback = error * 2.0;
    feedback = fabs(feedback) > fabs(_power) * 0.20 ? fabs(_power) * 0.20 * getSign(feedback) : feedback;
    moveRight(_power + feedback);
    moveLeft(_power - feedback);
    this_thread::sleep_for(5);
  }
  unlockBase();
}
// 前进，直到达到指定时间或指定距离，以先到者为准，同时保持航向
void timerForward(float _power, int _duration, float _heading, float _distance) {
  auto timer = MyTimer();
  float error = 0, feedback = 0;
  float direction = getHeading();
  float distance = getForwardPos();
  while (timer.getTime() < _duration && fabs(getForwardPos() - distance) < fabs(_distance)) {
    error = getHeading() - direction - _heading;
    feedback = error * 2.0;
    feedback = fabs(feedback) > fabs(_power) * 0.20 ? fabs(_power) * 0.20 * getSign(feedback) : feedback;
    moveRight(_power + feedback);
    moveLeft(_power - feedback);
    this_thread::sleep_for(5);
  }
  unlockBase();
}

// (基于编码器) 前进或后退指定的相对距离
void posForwardRel(float _power, float _target) {
  float start = getForwardPos();
  float error = 0, feedback = 0;
  float direction = getHeading();
  float power = getSign(_target) * fabs(_power);
  while (fabs(getForwardPos() - start) < fabs(_target)) {
    error = getHeading() - direction;
    feedback = error * 3.0;
    feedback = fabs(feedback) > fabs(_power) * 0.1 ? fabs(_power) * 0.1 * getSign(feedback) : feedback;
    moveRight(power + feedback);
    moveLeft(power - feedback);
    this_thread::sleep_for(5);
  }
  unlockBase();
}
// (基于编码器) 前进或后退指定的相对距离，并转向指定的相对角度
void posForwardRel(float _power, float _target, float _heading) {
  float start = getForwardPos();
  float error = 0, feedback = 0;
  float direction = getHeading();
  float power = getSign(_target) * fabs(_power);
  while (fabs(getForwardPos() - start) < fabs(_target)) {
    error = getHeading() - direction - _heading;
    feedback = error * 3.0;
    feedback = fabs(feedback) > fabs(_power) * 0.2 ? fabs(_power) * 0.2 * getSign(feedback) : feedback;
    moveRight(power + feedback);
    moveLeft(power - feedback);
    this_thread::sleep_for(5);
  }
  unlockBase();
}
// (基于编码器) 前进或后退到指定的绝对位置
void posForwardAbs(float _power, float _target) {
  resetForwardPos();
  float target_rel = _target - getForwardPos();
  posForwardRel(_power, target_rel);
}
// (基于编码器) 前进或后退到指定的绝对位置，并转向指定的相对角度
void posForwardAbs(float _power, float _target, float _heading) {
  float target_rel = _target - getForwardPos();
  posForwardRel(_power, target_rel, _heading);
}

// (基于PID) 前进或后退指定的相对距离
void pidForwardRel(float _target) {
  pidForwardAbs(getForwardPos() + _target);
}
// (基于PID) 前进或后退到指定的绝对位置，使用默认PID系数
void pidForwardAbs(float _target) {
  auto pid = PID();
  pid.setCoefficient(BASE_FORWARD_PID[0], BASE_FORWARD_PID[1], BASE_FORWARD_PID[2]);
  pid.setTarget(_target);
  pid.setIMax(20);
  pid.setIRange(5);
  pid.setErrorTolerance(3);  // 设定误差小于多少的时候可以跳出循环
  pid.setDTolerance(30);  // 设定速度小于多少的时候车可以跳出循环
  pid.setJumpTime(20);
  while (!pid.targetArrived()) {
    pid.update(getForwardPos());
    moveRight(pid.getOutput());
    moveLeft(pid.getOutput());
    this_thread::sleep_for(5);
  }
  unlockBase();
}
// (基于PID) 前进或后退到指定的绝对位置，并转向指定的相对角度
void pidForwardAbs(float _target, float _heading) {
  auto pid = PID();
  MyTimer timer;
  float error = 0, feedback = 0;
  float direction = getHeading();
  pid.setCoefficient(BASE_FORWARD_PID[0], BASE_FORWARD_PID[1], BASE_FORWARD_PID[2]);
  pid.setTarget(_target);
  pid.setIMax(20);
  pid.setIRange(5);
  pid.setErrorTolerance(3);  // 设定误差小于多少的时候可以跳出循环
  pid.setDTolerance(30);  // 设定速度小于多少的时候车可以跳出循环
  pid.setJumpTime(20);
  while (!pid.targetArrived() && timer.getTime() < 2000) {
    error = getHeading() - direction - _heading;
    feedback = error * 2.0;
    feedback = fabs(feedback) > 20 ? 20 * getSign(feedback) : feedback;
    pid.update(getForwardPos());
    moveRight(pid.getOutput() + feedback);
    moveLeft(pid.getOutput() - feedback);
    this_thread::sleep_for(5);
  }
  unlockBase();
}
// (基于PID) 前进或后退到指定的绝对位置，使用自定义PID系数
void pidForwardAbs(float _target, float _kp, float _ki, float _kd) {
  auto pid = PID();
  MyTimer timer;
  pid.setCoefficient(_kp, _ki, _kd);
  pid.setTarget(_target);
  pid.setIMax(5);
  pid.setIRange(3);
  pid.setErrorTolerance(3);  // 设定误差小于多少的时候可以跳出循环
  pid.setDTolerance(10);  // 设定速度小于多少的时候车可以跳出循环
  pid.setJumpTime(50);
  timer.reset();
  while (!pid.targetArrived() && timer.getTime() < 2000) {
    pid.update(getForwardPos());
    moveRight(pid.getOutput());
    moveLeft(pid.getOutput());
    this_thread::sleep_for(5);
  }
  unlockBase();
}

// 转向动作
// 在指定时间内，让转向功率从初始值线性增加/减少到最终值
void softStartTimerRotate(float _power_init, float _power_final, int _duration) {
  auto timer = MyTimer();
  float step = (_power_final - _power_init) / _duration;
  while (timer.getTime() < _duration) {
    moveClockwise(_power_init + timer.getTime() * step);
    this_thread::sleep_for(5);
  }
  resetForwardPos();
  unlockBase();
}
// 以固定功率转向指定的时间
void timerRotate(float _power, int _duration) {
  moveClockwise(_power);
  this_thread::sleep_for(_duration);
  resetForwardPos();
  unlockBase();
}
// (基于IMU) 转向指定的相对角度
void angleRotateRel(float _power, float _target) {
  angleRotateAbs(_power, getHeading() + _target);
}
// (基于IMU) 转向指定的绝对角度 (简易版，无PID)
void angleRotateAbs(float _power, float _target) {
  while (fabs(_target - getHeading()) > 180) {
    if (_target - getHeading() > 0)
      _target -= 360;
    else
      _target += 360;
  }
  float start = getHeading();
  float power = getSign(_target - getHeading()) * fabs(_power);
  float target = fabs(_target - getHeading());
  moveClockwise(power);
  while (fabs(getHeading() - start) < target)
    this_thread::sleep_for(5);
  resetForwardPos();
  unlockBase();
}

// (基于PID) 转向指定的相对角度
void pidRotateRel(float _target) {
  pidRotateAbs(getHeading() + _target);
}
// (基于PID) 转向指定的绝对角度，使用默认PID系数
void pidRotateAbs(float _target) {
  auto pid = PID();
  MyTimer timer;
  while (fabs(_target - getHeading()) > 180) {
    if (_target - getHeading() > 0)
      _target -= 360;
    else
      _target += 360;
  }
  pid.setTarget(_target);
  pid.setIMax(15);
  pid.setIRange(10);
  pid.setErrorTolerance(1);
  pid.setDTolerance(20);
  pid.setJumpTime(40);
  timer.reset();
  while (!pid.targetArrived() && timer.getTime() < 1000) {
    pid.setCoefficient(BASE_ROTATE_PID[0], BASE_ROTATE_PID[1], BASE_ROTATE_PID[2]);
    pid.update(getHeading());
    moveClockwise(pid.getOutput());
    this_thread::sleep_for(5);
  }
  resetForwardPos();
  unlockBase();
}
// (基于PID) 转向指定的绝对角度，使用自定义误差容忍度
void pidRotateAbs(float _target, float _error_tolerance) {
  auto pid = PID();
  MyTimer timer;
  while (fabs(_target - getHeading()) > 180) {
    if (_target - getHeading() > 0)
      _target -= 360;
    else
      _target += 360;
  }
  pid.setTarget(_target);
  pid.setIMax(15);
  pid.setIRange(10);
  pid.setErrorTolerance(_error_tolerance);
  pid.setDTolerance(20);
  pid.setJumpTime(40);
  timer.reset();
  while (!pid.targetArrived() && timer.getTime() < 1000) {
    pid.setCoefficient(BASE_ROTATE_PID[0], BASE_ROTATE_PID[1], BASE_ROTATE_PID[2]);
    pid.update(getHeading());
    moveClockwise(pid.getOutput());
    this_thread::sleep_for(5);
  }
  resetForwardPos();
  unlockBase();
}
// (基于PID) 转向指定的绝对角度，使用自定义PID系数
void pidRotateAbs(float _target, float _kp, float _ki, float _kd) {
  auto pid = PID();
  MyTimer timer;
  while (fabs(_target - getHeading()) > 180) {
    if (_target - getHeading() > 0)
      _target -= 360;
    else
      _target += 360;
  }
  pid.setTarget(_target);
  pid.setIMax(15);
  pid.setIRange(10);
  pid.setErrorTolerance(1);
  pid.setDTolerance(20);
  pid.setJumpTime(40);
  timer.reset();
  while (!pid.targetArrived() && timer.getTime() < 1000) {
    pid.setCoefficient(_kp, _ki, _kd);
    pid.update(getHeading());
    moveClockwise(pid.getOutput());
    this_thread::sleep_for(5);
  }
  resetForwardPos();
  unlockBase();
}

// 曲线动作
// 让左右轮以不同速度行驶指定时间，从而实现走弧线
void timerCurve(float left_power, float right_power, float _duration, bool _mirror_flag) {
  if(_mirror_flag) {
    moveLeft(left_power);
    moveRight(right_power);
  }
  else {
    moveLeft(right_power);
    moveRight(left_power);
  }
  this_thread::sleep_for(_duration);
  unlockBase();
}