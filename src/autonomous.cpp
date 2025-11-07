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
  
  // --- 第一步：移动并转向 ---
  softStartTimerForward(0, 40, 150); // 使用150毫秒从0加速到40%功率前进
  posForwardAbs(40, 30);             // 以40%功率前进，直到位置编码器读数达到30
  pidRotateAbs(90);                  // 使用PID控制，将车头精确转向至场地绝对角度90度方向
  
  // --- 第二步：收集场地上的三个球 ---
  softStartTimerForward(0, 40, 150); // 再次软启动前进
  spinIntaker1(100);                 // 开启进球滚轮
  posForwardAbs(40, 300);            // 以40%功率快速前进，接近三个球
  posForwardAbs(30, 600);            // 减慢到30%功率，稳定地吸入三个球
  this_thread::sleep_for(200);       // 等待0.2秒，确保球完全进入
  setPistonFront(true);              // 伸出前方活塞，抱住并固定吸入的球
  this_thread::sleep_for(200);       // 等待0.2秒，确保活塞动作完成
  spinIntaker1(0);                   // 停止进球滚轮

  // --- 第三步：移动到X形桥并得分 ---
  posForwardAbs(40, -60);            // 以40%功率后退，为转向留出空间
  pidRotateAbs(-45);                 // 转向至-45度，对准X形桥的中层得分区
 
  setPistonUp(true);                 // 伸出上方活塞，打开球的限位装置
  this_thread::sleep_for(100);       // 等待0.1秒
  setPistonUp(false);                // 收回上方活塞
  softStartTimerForward(0, 40, 150); // 软启动后退
  posForwardAbs(40, -100);           // 后退到-100位置
  timerForward(-20, 800);            // 以-20%功率（后退）行驶800毫秒，轻轻靠到X桥上

  spinChange(-50);                   // 启动传送/翻转机构，以-50%功率吐出一个球到中层
  this_thread::sleep_for(200);       // 等待0.2秒，确保球已吐出
  spinChange(0);                     // 停止传送机构

  // --- 第四步：移动到导入桶并取球 ---
  pidRotateAbs(-45);                 // 再次校准角度至-45度
  softStartTimerForward(0, 40, 150); // 软启动前进
  posForwardAbs(40, 900);            // 前进到导入桶附近
  pidRotateAbs(-90);                 // 转向至-90度，正对导入桶
  
  spinIntaker1(100);                 // 开启进球滚轮，准备接球

  // --- 第五步：处理导入桶的球并移动到高桥 ---
  posForwardAbs(40, 30);             // 稍微前进靠近导入桶
  timerForward(100, 500);            // 以100%功率前进500毫秒，触发导入桶
  this_thread::sleep_for(1000);      // 等待1秒，让导入桶的球落下并被吸入
 
  timerForward(-30, 200);            // 短暂后退
  pidRotateAbs(-90);                 // 校准角度
  posForwardAbs(40, -300);           // 后退到高桥附近

  // --- 第六步：在高桥上得分 ---
  timerForward(-30, 200);            // 再次后退微调位置

  spinChange(-20);                   // 传送机构反转一下，调整球的位置
  spinIntaker1(-50);                 // 进球滚轮反转，调整球的位置
  this_thread::sleep_for(100);       // 等待0.1秒
  setPistonUp(true);                 // 伸出上方活塞，准备得分
  this_thread::sleep_for(100);       // 等待0.1秒
  spinIntaker1(100);                 // 进球滚轮和传送机构正转，将所有球送上高桥
  spinChange(100);
  timerForward(-30, 200);            // 后退以确保机器人不接触桥
  this_thread::sleep_for(2000);      // 等待2秒，确保所有球都已得分
  
  // --- 结束 ---
  auton_pre_usercontrol(); // 恢复机器人状态
  return;
}

void auton_right() {
  auton_init(); // 调用初始化函数，重置位置编码器和计时器
  this_thread::sleep_for(50);
  
  // --- 第一步：移动并转向 ---
  softStartTimerForward(0, 40, 150); // 使用150毫秒从0加速到40%功率前进 (软启动)
  posForwardAbs(40, 30);             // 以40%功率前进，直到位置编码器读数达到30
  pidRotateAbs(-90);                 // 使用PID控制，将车头精确转向至场地绝对角度-90度方向
  
  
  // --- 第二步：收集场地上的三个球 ---
  softStartTimerForward(0, 40, 150); // 再次软启动前进
  spinIntaker1(100);                 // 开启进球滚轮，准备吸球
  posForwardAbs(40, 300);            // 以40%功率快速前进，接近三个球
  posForwardAbs(30, 600);            // 减慢到30%功率，稳定地吸入三个球
  this_thread::sleep_for(200);       // 等待0.2秒，确保球体完全进入机器人内部
  setPistonFront(true);              // 伸出前方活塞，抱住并固定吸入的球
  this_thread::sleep_for(200);       // 等待0.2秒，确保活塞动作完成
  spinIntaker1(0);                   // 停止进球滚轮

  // --- 第三步：移动到X形桥并得分 ---
  posForwardAbs(40, -60);            // 以40%功率后退，为转向留出足够空间
  pidRotateAbs(45);                  // 转向至45度，对准X形桥的中层得分区
 
  softStartTimerForward(0, 40, 150); // 软启动后退
  posForwardAbs(40, -100);           // 后退到-100位置
  timerForward(-20, 400);            // 以-20%功率（后退）行驶400毫秒，轻轻靠到X桥上以校准位置

  // --- 第四步：在X桥中层得分 ---
  setPistonDown(true);               // 伸出下方活塞（与左侧程序使用上方活塞不同）
  spinChange(-100);                  // 启动传送机构，以-100%功率吐出一个球
  spinIntaker1(-100);                // 进球滚轮同时反转，辅助吐球
  this_thread::sleep_for(400);       // 等待0.4秒，确保球已成功吐出
  setPistonDown(false);              // 收回下方活塞
  spinChange(0);                     // 停止传送机构
  spinIntaker1(0);                   // 停止进球滚轮

  // --- 第五步：移动到导入桶 ---
  pidRotateAbs(45);                  // 再次校准角度至45度
  setPistonUp(true);                 // 伸出上方活塞，打开球的限位装置
  softStartTimerForward(0, 40, 150); // 软启动前进
  setPistonUp(false);                // 立即收回上方活塞，完成一个快速的开闭动作
  posForwardAbs(40, 960);            // 以40%功率长距离前进，直到导入桶附近
  pidRotateAbs(90);                  // 转向至90度，正对导入桶

  
  // --- 第六步：处理导入桶，获取新球 ---
  spinIntaker1(100);                 // 开启进球滚轮，准备接球
 
  posForwardAbs(40, 60);             // 稍微前进靠近导入桶
  timerForward(40, 300);             // 以40%功率前进300毫秒，触发导入桶
  // 使用循环，通过前后小幅度快速移动来“摇晃”导入桶，确保所有球都掉落出来
  for (int i = 0; i < 3; i++) {
    timerForward(-20, 250);          // 后退
    this_thread::sleep_for(50);
    timerForward(30, 350);           // 前进
    this_thread::sleep_for(50);
  }

  // --- 第七步：移动到高桥 ---
  timerForward(-30, 200);            // 短暂后退，脱离导入桶
  pidRotateAbs(90);                  // 再次校准角度
  posForwardAbs(40, -300);           // 后退到高桥附近

  // --- 第八步：在高桥上得分 ---
  timerForward(-30, 200);            // 再次后退微调位置

  spinIntaker1(-50);                 // 进球滚轮和传送机构短暂反转，调整球的位置
  spinChange(-100);
  setPistonUp(true);                 // 伸出上方活塞，准备将球抬升至高桥
  this_thread::sleep_for(100);       // 等待0.1秒
  spinIntaker1(100);                 // 进球滚-轮和传送机构正转，将所有球送上高桥
  spinChange(100);
  timerForward(-30, 200);            // 最后一次后退，确保机器人不接触桥体
  this_thread::sleep_for(2000);      // 长时间等待2秒，确保所有球都已稳定得分

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
