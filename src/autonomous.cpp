#include "autonomous.h"
#include "Change-1DOF.h"
#include "basic-functions.h"
#include "differential-base.h"
#include "timer.h"
#include "robot-config.h"

class Autonomous {
  public:
    // 构造函数
    Autonomous() {}

    void pid_left() {
      init();
      this_thread::sleep_for(50);
      
      spinIntaker1(100);//进球滚轮开始转动
      pidForwardAbs(40, 100);//快速接近3q球

      pidRotateAbs(-45);//转动车头指向45度方向
    
      pidForwardAbs(30, 260);//慢速吸3球
      this_thread::sleep_for(200);//等待0.2s,吸球
      setPistonFront(true);//前方活塞伸出，抱球
      this_thread::sleep_for(150);//等待0.2s,吸球
      spinIntaker1(0);//进球滚轮停止转动

    
      pidRotateAbs(-135);//对准导入桶
      pidForwardAbs(40, -320);//后退到中桥         这个距离就是x到中桥的距离
      timerForward(-10,200);//后退压到桥
      spinChange(-100);
      this_thread::sleep_for(100);
      spinChange(0);

      spinIntaker1(100);//进球滚轮开始转动
      pidRotateAbs(-130);//对准导入桶
      //softStartTimerForward(0, 40, 150 );//软起动0-40，
      pidForwardAbs(40, 870);//前进到导入桶          这个距离就是x到导入桶的距离
      pidRotateAbs(-180);//对准导入桶
      //pidForwardAbs(40, 60);
      //timerForward(40,1400);

      //pidForwardAbs(40, -300);//后退到高桥
      pidForwardAbs(40, -300);//后退到高桥
      timerForward(-30,400);
      spinIntaker1(100);
      spinIntaker2(100);
      spinChange(100);
      this_thread::sleep_for(1500);

      pidForwardAbs(30, 150);
      pidRotateAbs(-225);
      pidForwardAbs(30, -175);
      pidRotateAbs(-180);
      pidForwardAbs(30, -200);
      pidForwardAbs(100, -100);

      pre_usercontrol();
      return;
    }

    void pid_right() {
      init();
      this_thread::sleep_for(50);

      pidForwardAbs(30, 165);
      pidRotateAbs(45);
      this_thread::sleep_for(100);

      spinIntaker1(100);
      pidForwardAbs(30, 300);
      setPistonFront(true);
      this_thread::sleep_for(750);

      pidRotateAbs(135);
      spinIntaker1(100);
      pidForwardAbs(30, 520);

      pidRotateAbs(-180);
      timerForward(-30, 800);

      spinIntaker1(100);
      spinIntaker2(100);
      spinChange(100); 
      this_thread::sleep_for(2500);

      pidForwardAbs(30, 150);

      pidRotateAbs(-225);
      pidForwardAbs(30, -235); 
      pidRotateAbs(-180);
      timerForward(-30, 800); 
      
      // --- 结束 ---
      pre_usercontrol(); // 调用函数恢复机器人所有部件到初始状态
      return;
    }

  private:
    // 自动阶段的计时器实例
    MyTimer auton_timer;

    void init(void) {
      resetForwardPos(); // 初始化/重置底盘的前进位置编码器读数
      auton_timer.reset(); // 重置自动阶段的计时器
    }

    // 自动阶段结束，进入手动阶段前的准备函数
    // 用于将所有电机和气缸恢复到安全或初始状态，并打印自动阶段的耗时。
    void pre_usercontrol(void) {
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
};