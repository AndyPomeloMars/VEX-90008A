#include "robot-config.h"

using namespace vex;

brain Brain;
controller Controller = controller(primary);

// 电机定义
// 定义底盘左侧的三个电机（前、中、后）
// 参数: (端口号, 齿轮比, 是否反转)
// PORT11: 连接到V5大脑的11号端口
// ratio6_1: 使用6:1的齿轮组（红色齿轮盒，高扭矩）
// true: 设置电机方向为反转。通常底盘一侧的电机需要反转，以保证两侧同步前进
motor Motor_BaseLF = motor(PORT1, ratio6_1, true);
motor Motor_BaseLM = motor(PORT2, ratio6_1, true);
motor Motor_BaseLB = motor(PORT3, ratio6_1, true);
// 定义底盘右侧的三个电机（前、中、后）
// false: 设置电机方向为正转（默认）
motor Motor_BaseRF = motor(PORT4, ratio6_1, false);
motor Motor_BaseRM = motor(PORT5, ratio6_1, false);
motor Motor_BaseRB = motor(PORT6, ratio6_1, false);

// 定义进球机构的两个电机
// ratio18_1: 使用18:1的齿轮组（绿色齿轮盒，标准速度）
motor Motor_Intaker1 = motor(PORT7, ratio18_1, false);
motor Motor_Intaker2 = motor(PORT8, ratio18_1, false);

// 定义传送/翻转机构的电机
motor Motor_Change = motor(PORT9, ratio18_1, false);

// 气动/气缸定义
// 定义用于控制气缸的电磁阀
// 参数: (连接的三线端口)
// Brain.ThreeWirePort.H: 连接到大脑的三线端口H
pneumatics Piston_Up = pneumatics(Brain.ThreeWirePort.H); // 上升气缸
pneumatics Piston_Down = pneumatics(Brain.ThreeWirePort.G); // 下降气缸
pneumatics Piston_Front = pneumatics(Brain.ThreeWirePort.F); // 前部气缸

// 传感器定义
// 定义惯性测量单元 (IMU)
// PORT21: 连接到21号端口
inertial IMU = inertial(PORT21);
// 定义光学/颜色传感器
// PORT6: 连接到6号端口
optical Color_Sensor = optical(PORT6);
// 定义距离传感器（也可作为非接触式开关使用）
// PORT4: 连接到4号端口
distance Switch_Sensor = distance(PORT4);

bool RemoteControlCodeEnabled = true;

void vexcodeInit(void) {
  Controller.Screen.setCursor(5, 1);
  Controller.Screen.print("%19s", "IMU Calibrating...");
  Color_Sensor.setLightPower(100);
  Color_Sensor.integrationTime(20);
  IMU.startCalibration();
  while (IMU.isCalibrating())
    this_thread::sleep_for(5);
  Controller.Screen.setCursor(5, 1);
  Controller.Screen.print("%19s", "IMU Ready!");
}