#include "autonomous.h"
#include "Change-1DOF.h"
#include "basic-functions.h"
#include "differential-base.h"
#include "timer.h"
#include "robot-config.h"

// 创建一个静态的计时器实例，专门用于自动阶段计时
static auto auton_timer = MyTimer();

void auton_init(void) {
  resetForwardPos(); // 初始化/重置底盘的前进位置编码器读数
  auton_timer.reset(); // 重置自动阶段的计时器
}

// 自动阶段结束，进入手动阶段前的准备函数
// 用于将所有电机和气缸恢复到安全或初始状态，并打印自动阶段的耗时。
void auton_pre_usercontrol(void) {
  // 停止所有可能在自动赛中运行的电机
  spinIntaker1(0); // 停止进球电机1
  spinChange(0); // 停止传送/翻转电机

  // 将所有气缸收回
  setPistonUp(false);
  setPistonFront(false);
  setPistonDown(false);

  // 在V5大脑屏幕上打印自动程序总耗时
  Brain.Screen.setCursor(8, 1);
  Brain.Screen.print("AutonTimer: %2.2fsec", auton_timer.getTime() / 1000.0);
}

// 技能赛自动程序 (目前为空)
// 这是为VEX机器人技能挑战赛准备的60秒自动程序。
void auton_skill(void) {
  auton_init();
  auton_pre_usercontrol();
}

// 左侧场地自动程序
// 这是一个完整的15秒自动赛流程，设计用于从场地左侧开始。
void auton_left() {
  auton_init(); // 初始化位置和计时器
  this_thread::sleep_for(50);

  softStartTimerForward(0, 40, 150);
  posForwardAbs(40, 125);
  pidRotateAbs(-45);
  
  softStartTimerForward(0, 40, 150);
  spinIntaker1(100);
  posForwardAbs(40, 400);
  this_thread::sleep_for(200);
  
  // --- 结束 ---
  auton_pre_usercontrol(); // 恢复机器人状态
  return;
}

void auton_right() {
  auton_init();
  this_thread::sleep_for(50);

  softStartTimerForward(0, 40, 150);
  posForwardAbs(40, 170);
  pidRotateAbs(45);
  
  softStartTimerForward(0, 30, 150);
  spinIntaker1(100);
  posForwardAbs(30, 275);
  this_thread::sleep_for(500);

  pidRotateAbs(135);
  softStartTimerForward(0, 40, 150);
  spinIntaker1(100);
  posForwardAbs(30, 645); // 655

  setPistonFront(true);
  this_thread::sleep_for(300);
  pidRotateAbs(-180);
  softStartTimerForward(0, 40, 150);
  spinIntaker1(100);
  timerForward(40, 280);

  for (int i = 0; i < 3; i++) {
    timerForward(-20, 175);          // 后退
    this_thread::sleep_for(50);
    timerForward(30, 200);           // 前进
    this_thread::sleep_for(50);
  }

  // this_thread::sleep_for(2000);

  setPistonFront(false);

  softStartTimerForward(0, 40, 150);
  softStartTimerForward(0, 40, 150);
  timerForward(-40, 750);

  spinIntaker1(100); 
  spinIntaker2(100);
  spinChange(100); 
  this_thread::sleep_for(2000);

  posForwardAbs(30, 150);

  pidRotateAbs(-225);
  posForwardAbs(30, -240); 
  pidRotateAbs(-180);
  timerForward(-30, 800); 
  
  // --- 结束 ---
  auton_pre_usercontrol(); // 调用函数恢复机器人所有部件到初始状态
  return;
}

// 联盟赛自动程序 (目前为空)
// 可能用于与联盟伙伴配合的特殊自动程序。
void auton_alliance() {
  MyTimer timer;
  timer.reset();
}
