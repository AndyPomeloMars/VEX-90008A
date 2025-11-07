#ifndef ROBOT_CONFIG_H_
#define ROBOT_CONFIG_H_
#include "vex.h"
using namespace vex;

extern brain Brain;

// VEXcode devices
extern controller Controller;

extern motor Motor_BaseLF;
extern motor Motor_BaseLM;
extern motor Motor_BaseLB;
extern motor Motor_BaseRF;
extern motor Motor_BaseRM;
extern motor Motor_BaseRB;
extern motor Motor_Intaker1;
extern motor Motor_Intaker2;
extern motor Motor_Change;
extern pneumatics Piston_Up;
extern pneumatics Piston_Down;
extern pneumatics Piston_Front;
extern inertial IMU;
extern optical Color_Sensor;
extern distance Switch_Sensor;
/**
 * Used to initialize code/tasks/devices added using tools in VEXcode Pro.
 *
 * This should be called at the start of your int main function.
 */
void vexcodeInit(void);
#endif