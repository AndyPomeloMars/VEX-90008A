#include "controller.h"

// 用于存储当前循环的手柄状态
// t: 存储当前时间戳
// A1-A4: 存储四个摇杆轴的模拟值 (-100 到 100)
// L1-DOWN: 存储所有按键的数字状态 (1 表示按下, 0 表示未按下)
int t, A1, A2, A3, A4, L1, L2, R1, R2, X, Y, A, B, LEFT, RIGHT, UP, DOWN;

// 用于存储上一个循环的手柄按键状态
// 通过比较当前状态和上一个状态，可以判断出按键是“刚刚被按下”还是“一直被按住”
int last_L1, last_L2, last_R1, last_R2, last_X, last_Y, last_A, last_B, last_LEFT, last_RIGHT, last_UP, last_DOWN;
    
void defineController() {
  while (true) {
    // 保存上一次循环的状态
    // 在读取新的状态之前，先把当前的状态变量值存入 "last" 系列变量中
    last_L1 = L1;
    last_L2 = L2;
    last_R1 = R1;
    last_R2 = R2;
    last_X = X;
    last_Y = Y;
    last_A = A;
    last_B = B;
    last_LEFT = LEFT;
    last_RIGHT = RIGHT;
    last_UP = UP;
    last_DOWN = DOWN;

    // 读取并更新当前的状态
    // 获取当前程序的运行时间（以毫秒为单位）
    t = Brain.timer(vex::timeUnits::msec);

    // 读取四个摇杆轴的位置，单位是百分比 (-100 to 100)
    A1 = Controller.Axis1.position(vex::percentUnits::pct); // 右摇杆 水平
    A2 = Controller.Axis2.position(vex::percentUnits::pct); // 右摇杆 垂直
    A3 = Controller.Axis3.position(vex::percentUnits::pct); // 左摇杆 垂直
    A4 = Controller.Axis4.position(vex::percentUnits::pct); // 左摇杆 水平

    // 读取所有按键的按压状态，.pressing() 会在按键被按住时返回 true (1)，否则返回 false (0)
    L1 = Controller.ButtonL1.pressing();
    L2 = Controller.ButtonL2.pressing();
    R1 = Controller.ButtonR1.pressing();
    R2 = Controller.ButtonR2.pressing();
    X = Controller.ButtonX.pressing();
    Y = Controller.ButtonY.pressing();
    A = Controller.ButtonA.pressing();
    B = Controller.ButtonB.pressing();
    LEFT = Controller.ButtonLeft.pressing();
    RIGHT = Controller.ButtonRight.pressing();
    UP = Controller.ButtonUp.pressing();
    DOWN = Controller.ButtonDown.pressing();
    
    this_thread::sleep_for(10);
  }
}