#include "vex.h"
#include "change-1DOF.h"
#include "basic-functions.h"
#include "controller.h"
#include "differential-base.h"
#include "math-tools.h"
#include "timer.h"
#include "parameters.h"
#include "robot-config.h"
#include "adjusment.h"
#include "autonomous.h"
using namespace vex;

competition Competition;

// 0 代表左侧场地，1 代表右侧场地
static int auton_strategy = 0;

// 自动赛选择函数
// 该函数在比赛的自动阶段被调用。它会根据全局变量 auton_strategy 的值，来决定执行哪一套预先编写好的自动程序
void autonomous(void) {
  switch (auton_strategy) {
    case 0:
      auton_left();
      break;
    case 1:
      auton_right();
      break;
  }
}

// void drivercontrol(void) {
//   // 初始化局部变量
//   int intake_offset = 0;
//   int change_state = 0;
//   int piston_up_state = 0;

//   // 定义档位速度
//   const int GEAR_SPEEDS[] = {40, 70, 100};
//   const int NUM_GEARS = 3;
  
//   // 添加状态变量
//   bool is_reversed = false;
//   int speed_gear = 1; // 默认中速档

//   // 在手柄屏幕上初始化显示信息
//   Controller.Screen.setCursor(3, 1);
//   Controller.Screen.print("Drive: NORMAL  ");
//   Controller.Screen.setCursor(4, 1);
//   Controller.Screen.print("Speed: MEDIUM(70%%)");

//   while (true) {
//     /*------------- 1. 底盘移动控制 -------------*/
//     const float turn_sensitivity = 0.7;

//     int forward_power = std::abs(A3) < JOYSTICK_DEADZONE ? 0 : A3;
//     int left_turn_power = std::abs(A4) < JOYSTICK_DEADZONE ? 0 : A4;
//     int right_turn_power = std::abs(A1) < JOYSTICK_DEADZONE ? 0 : A1;

//     if (is_reversed) {
//       forward_power = -forward_power;
//     }

//     int total_turn_power = left_turn_power + right_turn_power;
//     int left_final_power = forward_power + (total_turn_power * turn_sensitivity);
//     int right_final_power = forward_power - (total_turn_power * turn_sensitivity);

//     // [新] 应用档位速度限制逻辑 (使用 if 语句)
//     // 1. 从数组中获取当前档位的最大速度
//     int max_speed = GEAR_SPEEDS[speed_gear];
    
//     // 2. 使用 if 语句限制左轮的速度
//     if (left_final_power > max_speed) {
//       left_final_power = max_speed;
//     } else if (left_final_power < -max_speed) {
//       left_final_power = -max_speed;
//     }

//     // 3. 使用 if 语句限制右轮的速度
//     if (right_final_power > max_speed) {
//       right_final_power = max_speed;
//     } else if (right_final_power < -max_speed) {
//       right_final_power = -max_speed;
//     }

//     // 驱动电机
//     if (forward_power != 0 || total_turn_power != 0) {
//       moveLeft(left_final_power);
//       moveRight(right_final_power);
//     } else {
//       unlockBase();
//     }

//     /*------------- 2. 进球/得分机构控制 (保持不变) -------------*/
//     // ... (这部分代码与之前完全相同)
//     intake_offset += 6;
//     intake_offset = intake_offset % 360;
//     if (R1) { spinIntaker1(100); spinIntaker2(100); }
//     else if (R2) { spinIntaker1(-100); spinIntaker2(-100); }
//     else if (L1) { piston_up_state = 1; spinIntaker1(100); spinIntaker2(-100); spinChange(100); }
//     else if (L2) { spinIntaker1(100); spinIntaker2(-100); spinChange(-100); }
//     else { spinIntaker1(0); spinIntaker2(0); spinChange(0); }
    
//     /*------------- 3. 气缸和其他按键控制 -------------*/
//     // ... (这部分代码与之前完全相同)
//     if (A && !last_A && !LEFT) { setPistonFront(false); }
//     if (B && !last_B && !LEFT) { setPistonFront(true); }

//     if (Y && !last_Y) {
//       is_reversed = !is_reversed;
//       Controller.Screen.setCursor(3, 1);
//       if (is_reversed) { Controller.Screen.print("Drive: REVERSED"); }
//       else { Controller.Screen.print("Drive: NORMAL  "); }
//     }

//     bool gear_changed = false;
//     if (UP && !last_UP && speed_gear < NUM_GEARS - 1) {
//       speed_gear++;
//       gear_changed = true;
//     }
//     else if (DOWN && !last_DOWN && speed_gear > 0) {
//       speed_gear--;
//       gear_changed = true;
//     }

//     if (gear_changed) {
//       Controller.Screen.setCursor(4, 1);
//       switch (speed_gear) {
//         case 0: Controller.Screen.print("Speed: LOW   (%d%%)", GEAR_SPEEDS[0]); break;
//         case 1: Controller.Screen.print("Speed: MEDIUM(%d%%)", GEAR_SPEEDS[1]); break;
//         case 2: Controller.Screen.print("Speed: HIGH  (%d%%)", GEAR_SPEEDS[2]); break;
//       }
//     }

//     if (X && !last_X && !LEFT) {
//       piston_up_state++;
//       this_thread::sleep_for(100);
//     }
//     if (!(piston_up_state % 2) && !LEFT) { setPistonUp(false); }
//     else if (piston_up_state % 2 && !LEFT) { setPistonUp(true); }
    
//     /*------------- 4. 系统/调试功能 (保持不变) -------------*/
//     // ... (这部分代码与之前完全相同)
//     if (LEFT && RIGHT) { autonomous(); }
//     if (LEFT && A) {
//       IMU.startCalibration();
//       while (IMU.isCalibrating()) { this_thread::sleep_for(5); }
//       Controller.Screen.setCursor(5, 1);
//       Controller.Screen.print("%19s", "IMU Ready!");
//     }
//     if (LEFT && A1 > 80) {
//       this_thread::sleep_for(300);
//       auton_strategy = (auton_strategy + 1) % 2;
//       Controller.Screen.setCursor(5, 1);
//       if (auton_strategy == 0) { Controller.Screen.print("%12s", "left"); }
//       else { Controller.Screen.print("%12s", "right"); }
//     }
//     Brain.Screen.setCursor(1, 1);
//     Brain.Screen.print("IMU: %4f, POS: %4f", getHeading(), getForwardPos());
//     Brain.Screen.setCursor(2, 1);
//     Brain.Screen.print("change-state: %2f", change_state);
//     Brain.Screen.setCursor(10, 1);
//     this_thread::sleep_for(10);
//   }
// }

void drivercontrol(void) {
  // 初始化局部变量
  int intake_offset = 0;
  int change_state = 0;
  int piston_up_state = 0;

  // 定义档位速度
  const int GEAR_SPEEDS[] = {40, 70, 100};
  const int NUM_GEARS = 3;
  
  // 添加状态变量
  bool is_reversed = false;
  int speed_gear = 1; // 默认中速档

  // 在手柄屏幕上初始化显示信息
  Controller.Screen.setCursor(3, 1);
  Controller.Screen.print("Drive: NORMAL  ");
  Controller.Screen.setCursor(4, 1);
  Controller.Screen.print("Speed: MEDIUM(70%%)");

  while (true) {
    /*------------- 1. 底盘移动控制 (已修正) -------------*/
    const float turn_sensitivity = 0.7; // 您可以调整这个值来改变转向灵敏度

    // 步骤 1: 读取摇杆轴的值
    // forward_power 来自左摇杆的垂直轴 (A3)
    // turn_power 来自右摇杆的水平轴 (A1)
    int forward_power = std::abs(A3) < JOYSTICK_DEADZONE ? 0 : A3;
    int turn_power = std::abs(A1) < JOYSTICK_DEADZONE ? 0 : A1;

    // 步骤 2: 应用行驶方向反转逻辑
    if (is_reversed) {
      forward_power = -forward_power;
    }

    // 步骤 3: [关键修正] 使用正确的街机控制公式计算最终动力
    int left_final_power = forward_power + (turn_power * turn_sensitivity);
    int right_final_power = forward_power - (turn_power * turn_sensitivity);

    // 步骤 4: 应用档位速度限制
    int max_speed = GEAR_SPEEDS[speed_gear];
    if (left_final_power > max_speed) { left_final_power = max_speed; }
    else if (left_final_power < -max_speed) { left_final_power = -max_speed; }
    if (right_final_power > max_speed) { right_final_power = max_speed; }
    else if (right_final_power < -max_speed) { right_final_power = -max_speed; }

    // 步骤 5: [关键修正] 使用简洁的逻辑驱动电机
    if (forward_power != 0 || turn_power != 0) {
      moveLeft(left_final_power);
      moveRight(right_final_power);
    } else {
      unlockBase(); // 如果没有摇杆输入，则让底盘自由滑动
    }

    /*------------- 2. 进球/得分机构控制 (保持不变) -------------*/
    // ... (这部分代码与之前完全相同)
    intake_offset += 6;
    intake_offset = intake_offset % 360;
    if (R1) { spinIntaker1(100); spinIntaker2(100); }
    else if (R2) { spinIntaker1(-100); spinIntaker2(-100); }
    else if (L1) { piston_up_state = 1; spinIntaker1(100); spinIntaker2(-100); spinChange(100); }
    else if (L2) { spinIntaker1(100); spinIntaker2(-100); spinChange(-100); }
    else { spinIntaker1(0); spinIntaker2(0); spinChange(0); }
    
    /*------------- 3. 气缸和其他按键控制 (保持不变) -------------*/
    // ... (这部分代码与之前完全相同)
    if (A && !last_A && !LEFT) { setPistonFront(false); }
    if (B && !last_B && !LEFT) { setPistonFront(true); }

    if (Y && !last_Y) {
      is_reversed = !is_reversed;
      Controller.Screen.setCursor(3, 1);
      if (is_reversed) { Controller.Screen.print("Drive: REVERSED"); }
      else { Controller.Screen.print("Drive: NORMAL  "); }
    }

    bool gear_changed = false;
    if (UP && !last_UP && speed_gear < NUM_GEARS - 1) {
      speed_gear++;
      gear_changed = true;
    }
    else if (DOWN && !last_DOWN && speed_gear > 0) {
      speed_gear--;
      gear_changed = true;
    }

    if (gear_changed) {
      Controller.Screen.setCursor(4, 1);
      switch (speed_gear) {
        case 0: Controller.Screen.print("Speed: LOW   (%d%%)", GEAR_SPEEDS[0]); break;
        case 1: Controller.Screen.print("Speed: MEDIUM(%d%%)", GEAR_SPEEDS[1]); break;
        case 2: Controller.Screen.print("Speed: HIGH  (%d%%)", GEAR_SPEEDS[2]); break;
      }
    }

    if (X && !last_X && !LEFT) {
      piston_up_state++;
      this_thread::sleep_for(100);
    }
    if (!(piston_up_state % 2) && !LEFT) { setPistonUp(false); }
    else if (piston_up_state % 2 && !LEFT) { setPistonUp(true); }
    
    /*------------- 4. 系统/调试功能 (保持不变) -------------*/
    // ... (这部分代码与之前完全相同)
    if (LEFT && RIGHT) { autonomous(); }
    if (LEFT && A) {
      IMU.startCalibration();
      while (IMU.isCalibrating()) { this_thread::sleep_for(5); }
      Controller.Screen.setCursor(5, 1);
      Controller.Screen.print("%19s", "IMU Ready!");
    }
    if (LEFT && A1 > 80) {
      this_thread::sleep_for(300);
      auton_strategy = (auton_strategy + 1) % 2;
      Controller.Screen.setCursor(5, 1);
      if (auton_strategy == 0) { Controller.Screen.print("%12s", "left"); }
      else { Controller.Screen.print("%12s", "right"); }
    }
    Brain.Screen.setCursor(1, 1);
    Brain.Screen.print("IMU: %4f, POS: %4f", getHeading(), getForwardPos());
    Brain.Screen.setCursor(2, 1);
    Brain.Screen.print("change-state: %2f", change_state);
    Brain.Screen.setCursor(10, 1);
    this_thread::sleep_for(10);
  }
}

int main() {
  // 将 autonomous 函数注册为自动阶段要执行的函数
  Competition.autonomous(autonomous);
  // 将 drivercontrol 函数注册为手动控制阶段要执行的函数
  Competition.drivercontrol(drivercontrol);

  // 执行VEXcode项目的一些必要的后台初始化
  vexcodeInit();

  // 创建并启动一个新线程，用于执行 defineController 函数
  // 这通常用于处理一些需要并行运行的任务，如更新手柄屏幕或复杂的传感器数据处理，以免阻塞主循环
  thread ThreadController(defineController);

  while (true) {
    this_thread::sleep_for(100); // 线程休眠
  }
}