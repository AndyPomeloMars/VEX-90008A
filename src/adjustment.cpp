#include "adjusment.h"
#include "differential-base.h"
#include "robot-config.h"
#include "change-1DOF.h"
#include "controller.h"
#include "vex.h"

#define resetHeading() IMU.resetRotation()

void Adjustment::updateInput(float _input) {
  input = _input;
}

void Adjustment::updateOutput(float _output) {
  output = _output;
}

void Adjustment::init(char* _str) {
  my_ofstream.open(_str);
}

void Adjustment::stop() {
  my_ofstream.close();
}

void Adjustment::write() {
  my_ofstream << input << ", " << output << std::endl;
}

void Adjustment::startTimer() {
  mytimer.reset();
}

void Adjustment::stopTimer() {
  std::cout << "autonomous time: " << mytimer.getTime() / 1000.0 << " sec" << std::endl;
}

void tuning_forward_p() {
  Adjustment my_adjustment;
  PID my_pid;
  MyTimer my_timer;
  float kp = 0.2;
  float input, output;
  for (int i = 0; i < 10; i++) {
    kp += 0.02;
    char filename[14];
    sprintf(filename, "forward-p-%1.2f.txt", kp);
    my_adjustment.init(filename);
    resetForwardPos();
    my_pid.setTarget(600);
    my_pid.setCoefficient(kp, 0, 0);
    my_pid.setErrorTolerance(0.1);
    my_pid.setDTolerance(1);
    my_pid.setJumpTime(100);
    std::cout << kp << std::endl;
    my_timer.reset();
    // my_base.softStartTimerRotate(30, 100, 600);
    while (!my_pid.targetArrived() && my_timer.getTime() < 2000) {
      input = getForwardPos();
      output = my_pid.getOutput();
      my_pid.update(input);
      moveForward(output);
      my_adjustment.updateInput(input);
      my_adjustment.updateOutput(output);
      my_adjustment.write();
      vex::this_thread::sleep_for(20);
    }
    my_adjustment.stop();
    pidRotateAbs(180);
  }
}

void tuning_forward_i() {}

void tuning_forward_d() {
  Adjustment my_adjustment;
  PID my_pid;
  MyTimer my_timer;
  float kp = 0.34, kd = 0.5;
  float input, output;
  for (int i = 0; i < 25; i++) {
    kd += 0.02;
    char filename[18];
    sprintf(filename, "forward-d-%1.2f-%1.2f.txt", kp, kd);
    my_adjustment.init(filename);
    resetForwardPos();
    my_pid.setTarget(600);
    my_pid.setCoefficient(kp, 0, kd);
    my_pid.setErrorTolerance(0.1);
    my_pid.setDTolerance(1);
    my_pid.setJumpTime(100);
    std::cout << kd << std::endl;
    my_timer.reset();
    // my_base.softStartTimerRotate(30, 100, 600);
    while (!my_pid.targetArrived() && my_timer.getTime() < 2000) {
      input = getForwardPos();
      output = my_pid.getOutput();
      my_pid.update(input);
      moveForward(output);
      my_adjustment.updateInput(input);
      my_adjustment.updateOutput(output);
      my_adjustment.write();
      vex::this_thread::sleep_for(20);
    }
    my_adjustment.stop();
    pidRotateAbs(180);
  }
}

void tuning_rotate_p() {
  Adjustment my_adjustment;
  PID my_pid;
  MyTimer my_timer;
  float kp = 0.8;
  float input, output;
  for (int i = 0; i < 21; i++) {
    kp += 0.05;
    char filename[13];
    sprintf(filename, "rotate-p-%1.2f.txt", kp);
    my_adjustment.init(filename);
    my_pid.setCoefficient(kp, 0, 0);
    my_pid.setTarget(180);
    my_pid.setErrorTolerance(0.1);
    my_pid.setDTolerance(1);
    my_pid.setJumpTime(100);
    std::cout << kp << std::endl;
    my_timer.reset();
    // my_base.softStartTimerRotate(30, 100, 600);
    while (!my_pid.targetArrived() && my_timer.getTime() < 2500) {
      input = getHeading();
      output = my_pid.getOutput();
      my_pid.update(input);
      moveClockwise(output);
      my_adjustment.updateInput(input);
      my_adjustment.updateOutput(output);
      my_adjustment.write();
      vex::this_thread::sleep_for(20);
    }
    my_adjustment.stop();
    resetHeading();
  }
}

void tuning_rotate_i() {}

void tuning_rotate_d() {
  Adjustment my_adjustment;
  PID my_pid;
  MyTimer my_timer;
  float kp = 1.00, kd = 3.0;
  float input, output;
  for (int i = 0; i < 14; i++) {
    kd += 0.1;
    char filename[20];
    sprintf(filename, "rotate-d-%1.2f-%1.2f.txt", kp, kd);
    my_adjustment.init(filename);
    my_pid.setCoefficient(kp, 0, kd);
    my_pid.setTarget(180);
    my_pid.setErrorTolerance(0.1);
    my_pid.setDTolerance(1);
    my_pid.setJumpTime(100);
    std::cout << kd << std::endl;
    my_timer.reset();
    // my_base.softStartTimerRotate(30, 100, 600);
    while (!my_pid.targetArrived() && my_timer.getTime() < 2500) {
      input = getHeading();
      output = my_pid.getOutput();
      my_pid.update(input);
      moveClockwise(output);
      my_adjustment.updateInput(input);
      my_adjustment.updateOutput(output);
      my_adjustment.write();
      vex::this_thread::sleep_for(20);
    }
    my_adjustment.stop();
    resetHeading();
  }
}

void fastTuningRotatePD() {
    static float kp = 2.6, kd = 18;
    const float pid_tuning_unit = 0.2;
    if(X && !last_X) {
      kp+=pid_tuning_unit;
      std::cout << kp << ", " << kd << std::endl;
    }
    if(Y && !last_Y) {
      kp-=pid_tuning_unit;
      std::cout << kp << ", " << kd << std::endl;
    }
    if(A && !last_A) {
      kd+=pid_tuning_unit;
      std::cout << kp << ", " << kd << std::endl;
    }
    if(B && !last_B) {
      kd-=pid_tuning_unit;
      std::cout << kp << ", " << kd << std::endl;
    }
    if (DOWN && !last_DOWN) {
      IMU.resetRotation();
      softStartTimerRotate(30, 100, 100);
      pidRotateAbs(120, kp, 0, kd);
    }
}
void fastTuningRotateI() {
    static float kp = 2.8, ki = 0, kd = 18;
    const float pid_tuning_unit = 0.05;
    if(X && !last_X) {
      ki+=pid_tuning_unit;
      std::cout << "ki: " << ki << std::endl;
    }
    if(Y && !last_Y) {
      ki-=pid_tuning_unit;
      std::cout << "ki: " << ki << std::endl;
    }
    if (DOWN && !last_DOWN) {
      IMU.resetRotation();
      softStartTimerRotate(30, 100, 100);
      pidRotateAbs(30, kp, ki, kd);
      IMU.resetRotation();
      softStartTimerRotate(30, 100, 100);
      pidRotateAbs(90, kp, ki, kd);
      IMU.resetRotation();
      softStartTimerRotate(30, 100, 100);
      pidRotateAbs(120, kp, ki, kd);
      IMU.resetRotation();
      softStartTimerRotate(30, 100, 100);
      pidRotateAbs(30, kp, ki, kd);
      IMU.resetRotation();
      softStartTimerRotate(30, 100, 100);
      pidRotateAbs(90, kp, ki, kd);
    }
}
void fastTuningForwardPD() {
    static float kp = 2.5, kd = 25;
    const float pid_tuning_unit = 0.1;
    if(X && !last_X) {
      kp+=pid_tuning_unit;
      std::cout << kp << ", " << kd << std::endl;
    }
    if(Y && !last_Y) {
      kp-=pid_tuning_unit;
      std::cout << kp << ", " << kd << std::endl;
    }
    if(A && !last_A) {
      kd+=pid_tuning_unit;
      std::cout << kp << ", " << kd << std::endl;
    }
    if(B && !last_B) {
      kd-=pid_tuning_unit;
      std::cout << kp << ", " << kd << std::endl;
    }
    if (DOWN && !last_DOWN) {
      resetForwardPos();
      softStartTimerForward(30, 100, 100);
      posForwardAbs(100, 1000);
      pidForwardAbs(1200, kp, 0, kd);
    }
}
void fastTuningForwardI() {
    static float kp = 2.6, ki = 0, kd = 23.5;
    const float pid_tuning_unit = 0.05;
    if(X && !last_X) {
      ki+=pid_tuning_unit;
      std::cout << "ki: " << ki << std::endl;
    }
    if(Y && !last_Y) {
      ki-=pid_tuning_unit;
      std::cout << "ki: " << ki << std::endl;
    }
    if (DOWN && !last_DOWN) {
      resetForwardPos();
      softStartTimerForward(30, 100, 100);
      posForwardAbs(100, 1000);
      pidForwardAbs(1200, kp, ki, kd);
      std::cout << "position: " << getForwardPos() << std::endl;
      IMU.resetRotation();
      softStartTimerRotate(30, 100, 100);
      pidRotateAbs(180);
      resetForwardPos();
      softStartTimerForward(30, 100, 100);
      posForwardAbs(100, 1000);
      pidForwardAbs(1200, kp, ki, kd);
      std::cout << "position: " << getForwardPos() << std::endl;
    }
}



void tuning_robot() {
  // tuning_forward_p();
  // tuning_forward_i();
  // tuning_forward_d();
  // tuning_rotate_p();
  // tuning_rotate_i();
  // tuning_rotate_d();
}