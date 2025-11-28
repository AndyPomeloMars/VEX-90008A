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
Autonomous Auton; // 创建一个Autonomous类的实例

static int auton_strategy = 1; // 0 代表左侧场地，1 代表右侧场地
const float turn_sensitivity = 0.5; // 转向灵敏度
const int GEAR_SPEEDS[] = {10, 40, 70, 100}; // 定义四个档位各自对应的最大速度百分比
const int NUM_GEARS = 4; // 定义档位的总数
const int NUM_CONTROL_MODES = 2; // 定义总共有两种控制模式

// 自动赛选择函数
// 该函数在比赛的自动阶段被调用。它会根据全局变量 auton_strategy 的值，来决定执行哪一套预先编写好的自动程序
void autonomous(void) {
  switch (auton_strategy) {
    case 0:
      Auton.pid_left();
      break;
    case 1:
      Auton.pid_right();
      break;
  }
}

// 手动操作部分
void drivercontrol(void) {
  // 初始化局部变量
  int intake_offset = 0; // 进球机构偏移量，可能用于协同动作或视觉效果
  int change_state = 0; // 翻转/传送机构的状态变量
  int piston_up_state = 0; // 上升气缸的状态变量，用于实现开关式控制
  
  // 添加状态变量
  bool is_reversed = false; // 记录行驶方向是否反转，false为正常，true为反转
  int speed_gear = 2; // 记录当前速度档位，默认从2号档位（70%速度）开始

  // 控制模式状态变量，0为分离街机, 1为全功能街机)
  int control_mode = 0; 

  // 摇杆曲线
  int stick_curve = 3; // 记录当前摇杆曲线，默认为三次函数

  // 锁头
  float locked_heading = 0.0;

  while (true) {
    // 在每次循环开始时，声明左右轮的最终动力变量
    int left_final_power, right_final_power = 0;

    // 手柄摇杆轴对应说明:
    // left_stick_vec 来自左摇杆的垂直轴 (A3)
    // left_stick_hor 来自左摇杆的水平轴 (A4)
    // right_stick_hor 来自右摇杆的水平轴 (A1)

    // 使用 switch 语句根据当前的 control_mode 来选择不同的操控逻辑
    switch (control_mode) {
      case 0: { // 模式 0: 分离式街机 (左摇杆前进, 右摇杆转向)
        int left_stick_vec = std::abs(A3) < JOYSTICK_DEADZONE ? 0 : A3; // 读取左摇杆垂直值作为前进动力
        int right_stick_hor = std::abs(A1) < JOYSTICK_DEADZONE ? 0 : A1; // 读取右摇杆水平值作为转向动力
        if (is_reversed) // 如果处于反向模式
          left_stick_vec = -left_stick_vec; // 则将前进动力反向
          
        // 使用标准街机公式计算左右轮动力
        // left_final_power = left_stick_vec + (right_stick_hor * turn_sensitivity);
        // right_final_power = left_stick_vec - (right_stick_hor * turn_sensitivity);

        // 应用摇杆曲线
        // 1. 将转向值归一化到 [-1, 1]
        float normalized_turn = right_stick_hor / 100.0;
        // 2. 应用摇杆曲线
        float curved_turn = pow(normalized_turn, stick_curve);
        // 3. 将结果转换回 [-100, 100] 的范围
        int final_turn_power = curved_turn * 100;

        // 使用 final_turn_power 来计算 left_final_power 和 right_final_power
        left_final_power = left_stick_vec + (final_turn_power * turn_sensitivity);
        right_final_power = left_stick_vec - (final_turn_power * turn_sensitivity);

        break; // 结束模式0的处理
      }
      case 1: { // 模式 1: 全功能街机 (左摇杆前进 + 转向, 右摇杆也转向)
        int left_stick_vec = std::abs(A3) < JOYSTICK_DEADZONE ? 0 : A3; // 读取左摇杆垂直值作为前进动力
        int left_stick_hor = std::abs(A4) < JOYSTICK_DEADZONE ? 0 : A4; // 读取左摇杆水平值作为转向动力
        int right_stick_hor = std::abs(A1) < JOYSTICK_DEADZONE ? 0 : A1; // 读取右摇杆水平值也作为转向动力
        int total_turn = left_stick_hor + right_stick_hor; // 将两个摇杆的转向输入相加
        if (is_reversed) // 如果处于反向模式
          left_stick_vec = -left_stick_vec; // 则将前进动力反向

        // 使用合并后的总转向动力来计算左右轮动力
        // left_final_power = left_stick_vec + (total_turn * turn_sensitivity);
        // right_final_power = left_stick_vec - (total_turn * turn_sensitivity);

        // 应用摇杆曲线
        // 1. 将转向值归一化到 [-1, 1]
        float normalized_turn = total_turn / 100.0;
        // 2. 应用摇杆曲线
        float curved_turn = pow(normalized_turn, stick_curve);
        // 3. 将结果转换回 [-100, 100] 的范围
        int final_turn_power = curved_turn * 100;

        // 使用 final_turn_power 来计算 left_final_power 和 right_final_power
        left_final_power = left_stick_vec + (final_turn_power * turn_sensitivity);
        right_final_power = left_stick_vec - (final_turn_power * turn_sensitivity);

        break; // 结束模式1的处理
      }
    }

    // 档位速度限制
    int max_speed = GEAR_SPEEDS[speed_gear]; // 从数组中获取当前档位对应的最大速度
    // 使用 if-else if 结构将左轮的最终动力限制在 [-max_speed, max_speed] 区间内
    if (left_final_power > max_speed) left_final_power = max_speed;
    else if (left_final_power < -max_speed) left_final_power = -max_speed;
    // 使用 if-else if 结构将右轮的最终动力限制在 [-max_speed, max_speed] 区间内
    if (right_final_power > max_speed) right_final_power = max_speed;
    else if (right_final_power < -max_speed) right_final_power = -max_speed;

    // 驱动电机
    if (left_final_power != 0 || right_final_power != 0) { // 如果有任何有效的动力输入
      moveLeft(left_final_power); // 驱动左轮
      moveRight(right_final_power); // 驱动右轮
    }
    else // 如果没有任何摇杆输入
      unlockBase(); // 则让底盘自由滑动

    // 进球 / 得分机构控制
    intake_offset += 6;
    intake_offset = intake_offset % 360;
    if (R1) { // R1键：吸球
      spinIntaker1(100); 
    }
    else if (R2) { // R2键：吐球
      spinIntaker1(-100); 
      spinIntaker2(-100); 
    }
    else if (L1) { // L1键：上层得分动作
      spinIntaker1(100); 
      spinIntaker2(100); 
      spinChange(100); 
    }
    else if (L2) { // L2键：中层得分动作
      spinIntaker1(100); 
      spinIntaker2(100); 
      spinChange(-100); 
    }
    else { // 无按键：停止所有相关电机
      spinIntaker1(0); 
      spinIntaker2(0); 
      spinChange(0); 
    }
    
    // 气缸控制
    if (X && !last_X && !LEFT) { // X键：切换上方气缸状态
      piston_up_state++;
      this_thread::sleep_for(100);
    }
    // 根据 piston_up_state 的奇偶性来控制气缸伸出或缩回
    if (!(piston_up_state % 2) && !LEFT) 
      setPistonUp(false);
    else if (piston_up_state % 2 && !LEFT) 
      setPistonUp(true);

    // 其他按键控制
    if (A && !last_A && !LEFT) // A键：控制前方气缸
      setPistonFront(false);
    if (B && !last_B && !LEFT) // B键：控制前方气缸
      setPistonFront(true);
    
    // Y键：切换行驶方向
    if (Y && !last_Y) {
      is_reversed = !is_reversed; // 翻转方向状态
      Controller.rumble("."); // 一个短促的点震动
      Controller.Screen.setCursor(2, 1); // 设置光标位置
      if (is_reversed) 
        Controller.Screen.print("Drive: REVERSED"); // 显示反向模式
      else 
        Controller.Screen.print("Drive: NORMAL  "); // 显示正常模式
    }

    // UP/DOWN键：切换速度档位
    bool gear_changed = false; // 标志位，用于判断档位是否变化
    if (UP && !last_UP && speed_gear < NUM_GEARS - 1) { // UP键：升档，并防止超过最高档
      speed_gear++;
      gear_changed = true;
    }
    else if (DOWN && !last_DOWN && speed_gear > 0) { // DOWN键：降档，并防止低于最低档
      speed_gear--;
      gear_changed = true;
    }

    // 如果档位发生了变化，则更新手柄屏幕
    if (gear_changed) {
      Controller.rumble("."); // 一个短促的点震动
      Controller.Screen.setCursor(2, 1);
      switch (speed_gear) {
        case 0: Controller.Screen.print("Speed: LOW   (%d%%)", GEAR_SPEEDS[0]); break;
        case 1: Controller.Screen.print("Speed: MEDIUM(%d%%)", GEAR_SPEEDS[1]); break;
        case 2: Controller.Screen.print("Speed: HIGH  (%d%%)", GEAR_SPEEDS[2]); break;
        case 3: Controller.Screen.print("Speed: MAX   (%d%%)", GEAR_SPEEDS[3]); break;
      }
    }

    // 控制模式切换
    if (RIGHT && !last_RIGHT) { // RIGHT键：切换控制模式
      control_mode++; // 模式编号加1
      control_mode = control_mode % NUM_CONTROL_MODES; // 使用取模运算实现循环切换
      Controller.rumble("."); // 一个短促的点震动
      Controller.Screen.setCursor(2, 1); // 设置光标
      switch (control_mode) { // 根据新模式更新屏幕显示
        case 0: Controller.Screen.print("Mode: SPLIT ARCADE"); break;
        case 1: Controller.Screen.print("Mode: FULL ARCADE "); break;
      }
    }
    
    // 系统/调试功能
    if (LEFT && RIGHT){ // 同时按 LEFT 和 RIGHT：测试自动程序
      Controller.rumble("--"); // 长震动
      Controller.Screen.setCursor(2, 1);
      Controller.Screen.print("%19s", "Test Autonomous");
      autonomous();
    }

    if (LEFT && A) { // 同时按 LEFT 和 A：校准IMU
      Controller.Screen.setCursor(2, 1);
      Controller.Screen.print("%19s", "Set IMU!");
      IMU.startCalibration();
      while (IMU.isCalibrating()) 
        this_thread::sleep_for(5);
      Controller.rumble("--"); // 长震动
      Controller.Screen.setCursor(2, 1);
      Controller.Screen.print("%19s", "IMU Ready!");
    }

    if (LEFT && A1 > 80) { // 同时按 LEFT 和推动右摇杆：选择自动赛策略
      this_thread::sleep_for(300);
      auton_strategy = (auton_strategy + 1) % 2;
      Controller.rumble("--"); // 长震动
      Controller.Screen.setCursor(2, 1);
      if (auton_strategy == 0) 
        Controller.Screen.print("%12s", "left");
      else 
        Controller.Screen.print("%12s", "right");
    }
    
    // 在V5大脑屏幕上打印调试信息
    Brain.Screen.setCursor(1, 1);
    Brain.Screen.print("IMU: %4f, POS: %4f", getHeading(), getForwardPos());
    Brain.Screen.setCursor(2, 1);
    Brain.Screen.print("change-state: %2f", change_state);
    Brain.Screen.setCursor(10, 1);
    this_thread::sleep_for(10); // 线程休眠10毫秒，防止CPU占用过高
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

  while (true)
    this_thread::sleep_for(100); // 线程休眠
}