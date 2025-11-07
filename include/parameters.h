#ifndef PARAMETERS_H_
#define PARAMETERS_H_

#include "vex.h"

#define RED_COLOR_HUE 5
#define BLUE_COLOR_HUE 215
#define COLOR_HUE_OFFSET 30
#define weiyu_robot2

const float MOVEMENT_LOWER_LIMIT = 10;
const float JOYSTICK_DEADZONE = 30;
const float ADS = 0.4;

#define CHASSIS_GEAR_RATIO 39 / 54
const float WHEEL_DIAMETER = 3.25;
const float BASE_WIDTH_2 = 156;

#ifdef ROBOT_YAHOO
#define IMU_10 3609.0
const float BASE_FORWARD_PID[3] = {2.6, 0.05, 23.5};
const float BASE_ROTATE_PID[3] = {2.8, 0.15, 18.7};
const float Change_ROTATE_PID[3] = {3.8, 0.0, 15};


//const float Change_READY_ANG = 65;
//const float Change_SCORING_ANG = 240;
#endif

#endif



#ifdef weiyu_robot2
const float Change_READY_ANG = 70;
const float Change_SCORING_ANG = 243;
#endif