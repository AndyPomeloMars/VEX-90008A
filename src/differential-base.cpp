#include "differential-base.h"

#include "PID.h"
#include "math-tools.h"
#include "timer.h"
#include "parameters.h"
#include "robot-config.h"
#include "vex.h"

#define left_pos                                             \
  (Motor_BaseLF.position(deg) + Motor_BaseLM.position(deg) + \
   Motor_BaseLB.position(deg)) /                             \
      3.0 * 69.85 * M_PI / 360.0 * 0.75
#define right_pos                                            \
  (Motor_BaseRF.position(deg) + Motor_BaseRM.position(deg) + \
   Motor_BaseRB.position(deg)) /                             \
      3.0 * 69.85 * M_PI / 360.0 * 0.75
#define heading IMU.rotation() / IMU_10 * 3600;
static float left_pos_last = 0, right_pos_last = 0;

/**
 * return the position of left side of base (mm from last reset position)
 * according to encoder value
 * @return return mm
 */
float getLeftPos() { return left_pos - left_pos_last; }
/**
 * return the position of right side of base (mm from last reset position)
 * according to encoder value
 * @return return mm
 */
float getRightPos() { return right_pos - right_pos_last; }
/**
 * return the vertical position of base (mm from last reset position) according
 * to encoder value
 * @return return mm
 */
float getForwardPos() { return (getLeftPos() + getRightPos()) / 2; }
/**
 * reset encoder on the left side of the base
 */
void resetLeftPos() { left_pos_last = left_pos; }
/**
 * reset encoder on the right side of the base
 */
void resetRightPos() { right_pos_last = right_pos; }
/**
 * reset encoders on both side of the base
 */
void resetForwardPos() {
  resetLeftPos();
  resetRightPos();
}
/**
 * return the heading angle of robot in deg (+360 after a full clockwise turn,
 * -360 after a counter-clockwise turn)
 * @return float with 1 decimal point
 */
float getHeading() { return heading; }
/**
 * powers all motors on left side of base with duty cycle _input%
 * @param _input ranges from -100 : 100
 */
void moveLeft(float _input) {
  if (fabs(_input) > 100) _input = getSign(_input) * 100;
  Motor_BaseLF.spin(directionType::fwd, (int)127 * _input, voltageUnits::mV);
  Motor_BaseLM.spin(directionType::fwd, (int)127 * _input, voltageUnits::mV);
  Motor_BaseLB.spin(directionType::fwd, (int)127 * _input, voltageUnits::mV);
}
/**
 * move left motors with speed feedback control
 * @param _input ranges from -100 : 100
 */
void moveLeftVel(float _input) {
  if (fabs(_input) > 100) _input = getSign(_input) * 100;
  Motor_BaseLF.spin(directionType::fwd, (int)_input, velocityUnits::pct);
  Motor_BaseLM.spin(directionType::fwd, (int)_input, velocityUnits::pct);
  Motor_BaseLB.spin(directionType::fwd, (int)_input, velocityUnits::pct);
}
/**
 * locks all motors on left side of base
 */
void lockLeft(void) {
  Motor_BaseLF.stop(vex::brakeType::hold);
  Motor_BaseLM.stop(vex::brakeType::hold);
  Motor_BaseLB.stop(vex::brakeType::hold);
}
/**
 * unlocks all motors on left side of base
 */
void unlockLeft(void) {
  Motor_BaseLF.stop(vex::brakeType::coast);
  Motor_BaseLM.stop(vex::brakeType::coast);
  Motor_BaseLB.stop(vex::brakeType::coast);
}
/**
 * powers all motors on right side of base with duty cycle _input%
 * @param _input ranges from -100 : 100
 */
void moveRight(float _input) {
  if (fabs(_input) > 100) _input = getSign(_input) * 100;
  Motor_BaseRF.spin(directionType::fwd, (int)127 * _input, voltageUnits::mV);
  Motor_BaseRM.spin(directionType::fwd, (int)127 * _input, voltageUnits::mV);
  Motor_BaseRB.spin(directionType::fwd, (int)127 * _input, voltageUnits::mV);
}
/**
 * move right motors with speed feedback control
 * @param _input ranges from -100 : 100
 */
void moveRightVel(float _input) {
  // _input ranges from -100 : 100
  // powers all motors on right side of base with duty cycle _input%
  if (fabs(_input) > 100) _input = getSign(_input) * 100;
  Motor_BaseRF.spin(directionType::fwd, (int)_input, velocityUnits::pct);
  Motor_BaseRM.spin(directionType::fwd, (int)_input, velocityUnits::pct);
  Motor_BaseRB.spin(directionType::fwd, (int)_input, velocityUnits::pct);
}
/**
 * locks all motors on right side of base
 */
void lockRight(void) {
  Motor_BaseRF.stop(vex::brakeType::hold);
  Motor_BaseRM.stop(vex::brakeType::hold);
  Motor_BaseRB.stop(vex::brakeType::hold);
}
/**
 * unlocks all motors on right side of base
 */
void unlockRight(void) {
  Motor_BaseRF.stop(vex::brakeType::coast);
  Motor_BaseRM.stop(vex::brakeType::coast);
  Motor_BaseRB.stop(vex::brakeType::coast);
}
/**
 * move forward with _input% power
 * @param _input ranges from -100 : 100
 */
void moveForward(float _input) {
  moveLeft(_input);
  moveRight(_input);
}
/**
 * rotate clockwise with _input% power
 * @param _input ranges from -100 : 100
 */
void moveClockwise(float _input) {
  moveLeft(_input);
  moveRight(-_input);
}
/**
 * stop the base with hold mode
 */
void lockBase(void) {
  lockLeft();
  lockRight();
}
/**
 * stop the base with coast mode
 */
void unlockBase(void) {
  unlockLeft();
  unlockRight();
}
/**
 * move forward with voltage change linearly
 */
void softStartTimerForward(float _power_init, float _power_final,
                           int _duration) {
  auto timer = MyTimer();
  float step = (_power_final - _power_init) / _duration;
  float error = 0, feedback = 0;
  float direction = getHeading();
  while (timer.getTime() < _duration) {
    error = getHeading() - direction;
    feedback = error * 0.0;
    feedback = fabs(feedback) > 20 ? 20 * getSign(feedback) : feedback;
    float power = _power_init + timer.getTime() * step;
    moveRight(power + feedback);
    moveLeft(power - feedback);
    this_thread::sleep_for(5);
  }
  unlockBase();
}
/**
 * forward with certain voltage in a period
 */
void timerForward(float _power, int _duration) {
  auto timer = MyTimer();
  float error = 0, feedback = 0;
  float direction = getHeading();
  while (timer.getTime() < _duration) {
    error = getHeading() - direction;
    feedback = error * 2.0;
    feedback = fabs(feedback) > fabs(_power) * 0.15
                   ? fabs(_power) * 0.15 * getSign(feedback)
                   : feedback;
    moveRight(_power + feedback);
    moveLeft(_power - feedback);
    this_thread::sleep_for(5);
  }
  unlockBase();
}
/**
 * forward with timer
 * @param _heading relative to current heading, if want a absolute heading degrees, for example 90 degrees, use "90 - getHeading()"
 */
void timerForward(float _power, int _duration, float _heading) {
  auto timer = MyTimer();
  float error = 0, feedback = 0;
  float direction = getHeading();
  while (timer.getTime() < _duration) {
    error = getHeading() - direction - _heading;
    feedback = error * 2.0;
    feedback = fabs(feedback) > fabs(_power) * 0.20
                   ? fabs(_power) * 0.20 * getSign(feedback)
                   : feedback;
    moveRight(_power + feedback);
    moveLeft(_power - feedback);
    this_thread::sleep_for(5);
  }
  unlockBase();
}
/**
 * forward with timer and distance, with heading control
 * @param _heading relative to current heading, if want a absolute heading degrees, for example 90 degrees, use "90 - getHeading()"
 * @param _distance distance relative to current position
 */
void timerForward(float _power, int _duration, float _heading,
                  float _distance) {
  auto timer = MyTimer();
  float error = 0, feedback = 0;
  float direction = getHeading();
  float distance = getForwardPos();
  while (timer.getTime() < _duration &&
         fabs(getForwardPos() - distance) < fabs(_distance)) {
    error = getHeading() - direction - _heading;
    feedback = error * 2.0;
    feedback = fabs(feedback) > fabs(_power) * 0.20
                   ? fabs(_power) * 0.20 * getSign(feedback)
                   : feedback;
    moveRight(_power + feedback);
    moveLeft(_power - feedback);
    this_thread::sleep_for(5);
  }
  unlockBase();
}

void posForwardRel(float _power, float _target) {
  float start = getForwardPos();
  float error = 0, feedback = 0;
  float direction = getHeading();
  float power = getSign(_target) * fabs(_power);
  while (fabs(getForwardPos() - start) < fabs(_target)) {
    error = getHeading() - direction;
    feedback = error * 3.0;
    feedback = fabs(feedback) > fabs(_power) * 0.1
                   ? fabs(_power) * 0.1 * getSign(feedback)
                   : feedback;
    moveRight(power + feedback);
    moveLeft(power - feedback);
    this_thread::sleep_for(5);
  }
  unlockBase();
}

void posForwardRel(float _power, float _target, float _heading) {
  float start = getForwardPos();
  float error = 0, feedback = 0;
  float direction = getHeading();
  float power = getSign(_target) * fabs(_power);
  while (fabs(getForwardPos() - start) < fabs(_target)) {
    error = getHeading() - direction - _heading;
    feedback = error * 3.0;
    feedback = fabs(feedback) > fabs(_power) * 0.2
                   ? fabs(_power) * 0.2 * getSign(feedback)
                   : feedback;
    moveRight(power + feedback);
    moveLeft(power - feedback);
    this_thread::sleep_for(5);
  }
  unlockBase();
}

void posForwardAbs(float _power, float _target) {
  resetForwardPos();
  float target_rel = _target - getForwardPos();
  posForwardRel(_power, target_rel);
}

void posForwardAbs(float _power, float _target, float _heading) {
  float target_rel = _target - getForwardPos();
  posForwardRel(_power, target_rel, _heading);
}

void pidForwardRel(float _target) { pidForwardAbs(getForwardPos() + _target); }
void pidForwardAbs(float _target) {
  auto pid = PID();
  pid.setCoefficient(BASE_FORWARD_PID[0], BASE_FORWARD_PID[1],
                     BASE_FORWARD_PID[2]);
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

void pidForwardAbs(float _target, float _heading) {
  auto pid = PID();
  MyTimer timer;
  float error = 0, feedback = 0;
  float direction = getHeading();
  pid.setCoefficient(BASE_FORWARD_PID[0], BASE_FORWARD_PID[1],
                     BASE_FORWARD_PID[2]);
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

void softStartTimerRotate(float _power_init, float _power_final,
                          int _duration) {
  auto timer = MyTimer();
  float step = (_power_final - _power_init) / _duration;
  while (timer.getTime() < _duration) {
    moveClockwise(_power_init + timer.getTime() * step);
    this_thread::sleep_for(5);
  }
  resetForwardPos();
  unlockBase();
}

void timerRotate(float _power, int _duration) {
  moveClockwise(_power);
  this_thread::sleep_for(_duration);
  resetForwardPos();
  unlockBase();
}

void angleRotateRel(float _power, float _target) {
  angleRotateAbs(_power, getHeading() + _target);
}

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
  while (fabs(getHeading() - start) < target) {
    this_thread::sleep_for(5);
  }
  resetForwardPos();
  unlockBase();
}

void pidRotateRel(float _target) { pidRotateAbs(getHeading() + _target); }

void pidRotateAbs(float _target) {
  auto pid = PID();
  MyTimer timer;
  // data transfer to prevend from huge angle rotation
  while (fabs(_target - getHeading()) > 180) {
    if (_target - getHeading() > 0)
      _target -= 360;
    else
      _target += 360;
  }
  pid.setTarget(_target);
  pid.setIMax(15);
  pid.setIRange(10);  // use if sentance to define the I coeff
  pid.setErrorTolerance(1);
  pid.setDTolerance(20);
  pid.setJumpTime(40);
  timer.reset();
  while (!pid.targetArrived() && timer.getTime() < 1000) {
    pid.setCoefficient(BASE_ROTATE_PID[0], BASE_ROTATE_PID[1],
                       BASE_ROTATE_PID[2]);
    pid.update(getHeading());
    moveClockwise(pid.getOutput());
    this_thread::sleep_for(5);
  }
  resetForwardPos();
  unlockBase();
}
void pidRotateAbs(float _target, float _error_tolerance) {
  auto pid = PID();
  MyTimer timer;
  // data transfer to prevend from huge angle rotation
  while (fabs(_target - getHeading()) > 180) {
    if (_target - getHeading() > 0)
      _target -= 360;
    else
      _target += 360;
  }
  pid.setTarget(_target);
  pid.setIMax(15);
  pid.setIRange(10);  // use if sentance to define the I coeff
  pid.setErrorTolerance(_error_tolerance);
  pid.setDTolerance(20);
  pid.setJumpTime(40);
  timer.reset();
  while (!pid.targetArrived() && timer.getTime() < 1000) {
    pid.setCoefficient(BASE_ROTATE_PID[0], BASE_ROTATE_PID[1],
                       BASE_ROTATE_PID[2]);
    pid.update(getHeading());
    moveClockwise(pid.getOutput());
    this_thread::sleep_for(5);
  }
  resetForwardPos();
  unlockBase();
}
void pidRotateAbs(float _target, float _kp, float _ki, float _kd) {
  auto pid = PID();
  MyTimer timer;
  // data transfer to prevend from huge angle rotation
  while (fabs(_target - getHeading()) > 180) {
    if (_target - getHeading() > 0)
      _target -= 360;
    else
      _target += 360;
  }
  pid.setTarget(_target);
  pid.setIMax(15);
  pid.setIRange(10);  // use if sentance to define the I coeff
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
/**
 * move with different speed of each wheel
 */
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
/**
 * this is shit
 */
void PIDPosCurveRel(float left_target, float right_target, float tolerance) {
  // move curved to _target position
  // stops base when finishing
  float _target = (left_target + right_target) / 2;
  float ratio = left_target / right_target;
  auto pid = PID();
  float k = 1;
  pid.setCoefficient(1.05, 0.05, 1.5);
  pid.setTarget(_target);
  pid.setIMax(30);
  pid.setIRange(20);
  pid.setErrorTolerance(tolerance);
  pid.setDTolerance(5);
  pid.setJumpTime(50);
  while (!pid.targetArrived()) {
    float leftPos_err =
        (getForwardPos() / _target) * left_target - getLeftPos();
    float rightPos_err =
        (getForwardPos() / _target) * right_target - getRightPos();
    pid.update(getForwardPos());
    float PIDoutput = pid.getOutput();
    if (fabs(PIDoutput) > 90) PIDoutput = getSign(PIDoutput) * 90;
    if (ratio > 1) {
      moveLeft(PIDoutput + k * leftPos_err);
      moveRight(PIDoutput / ratio + k * rightPos_err);
    } else {
      moveLeft(pid.getOutput() * ratio + k * leftPos_err);
      moveRight(pid.getOutput() + k * rightPos_err);
    }
    this_thread::sleep_for(5);
  }
  resetForwardPos();
  unlockBase();
}
/**
 * this is shit also
 */
void PIDPosCurveAbs(float left_target, float right_target, float tolerance) {
  PIDPosCurveRel(getLeftPos() + left_target, getRightPos() + right_target,
                 tolerance);
}

void arcadeControl(int forward_axis, int turn_axis, float turn_sensitivity = 0.5) {
  // 计算左右两侧轮子的最终速度
  // 前进/后退是基础速度，转向是在这个基础上进行加减
  int left_power = forward_axis + (turn_axis * turn_sensitivity);
  int right_power = forward_axis - (turn_axis * turn_sensitivity);

  // 调用底层函数来驱动电机
  moveLeft(left_power);
  moveRight(right_power);
}