#include "basic-functions.h"
#include "math-tools.h"
#include "timer.h"
#include "parameters.h"
#include "queue"
#include "robot-config.h"

// 控制进球电机1的转动
// 根据输入的占空比（百分比）来设置电机的转速和方向。
// 速度百分比，范围从 -100 到 100。正值正转，负值反转。
void spinIntaker1(float _input) {
  // 输入值限幅：确保输入值的绝对值不会超过100。如果超过，则根据其原始符号设置为100或-100。
  _input = fabs(_input) > 100 ? getSign(_input) * 100 : _input;

  // 判断输入值是否为0
  if (!_input)
    Motor_Intaker1.stop(coast); // 如果输入值为0，则停止电机，并设置刹车模式为 coast (惯性滑行)
  else
    // 如果输入值不为0，则驱动电机转动。
    // 使用电压控制模式：将输入的百分比(-100到100)乘以127，转换为毫伏(mV)单位的电压值。
    // V5电机的最大电压为12V (12000mV)，这里的127*100 = 12700mV 是一个接近满电压的设定。
    // 电压的正负号决定了电机的转动方向。
    Motor_Intaker1.spin(directionType::fwd, (int)127 * _input, voltageUnits::mV);
}

// 控制进球电机2的转动（同理）
void spinIntaker2(float _input) {
  _input = fabs(_input) > 100 ? getSign(_input) * 100 : _input;
  if (!_input) {
    Motor_Intaker2.stop(coast);
  } else {
    Motor_Intaker2.spin(directionType::fwd, (int)127 * _input, voltageUnits::mV);
  }
}

// 控制上升气缸的状态
// _input 布尔值，true 表示伸出，false 表示缩回。
void setPistonUp(bool _input) {
  if (_input)
    Piston_Up.open();
  else
    Piston_Up.close();
}

// 控制下降气缸的状态
// _input 布尔值，true 表示伸出，false 表示缩回。
void setPistonDown(bool _input) {
  if (_input)
    Piston_Down.open();
  else
    Piston_Down.close();
}

// 控制前部气缸的状态
// _input 布尔值，true 表示伸出，false 表示缩回。
void setPistonFront(bool _input) {
  if (_input)
    Piston_Front.open();
  else
    Piston_Front.close();
}
