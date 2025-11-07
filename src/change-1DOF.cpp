#include "change-1DOF.h"
#include "PID.h"
#include "math-tools.h"
#include "parameters.h"
#include "robot-config.h"


// 控制传送/翻转机构（名为Change）的电机转动
// 根据输入的占空比（百分比）来设置电机的转速和方向。
// "1DOF" 通常指单自由度（1 Degree of Freedom），意味着这个机构只有一个可动的关节或轴。
// 速度百分比，范围从 -100 到 100。正值代表一个方向，负值代表相反方向。
void spinChange(float _input) {
  // 输入值限幅
  // 检查输入值的绝对值是否大于100。
  // 如果是，则使用 getSign(_input) 获取其原始符号（1 或 -1），然后乘以100，将其强制设置为 100 或 -100。
  // 如果不是，则保持原值不变。
  // 这一步能防止传入无效的参数导致程序错误。
  _input = fabs(_input) > 100 ? getSign(_input) * 100 : _input;

  // 判断电机动作
  // 检查处理后的输入值是否为 0
  if (!_input)
    // 如果输入值为 0，则停止电机。
    //刹车模式设置为 coast，表示电机在停止后会自由滑行，而不是立即刹车。
    Motor_Change.stop(coast);
  else
    // 如果输入值不为 0，则驱动电机转动。
    // 使用电压控制模式来驱动电机：
    // V5 电机的最大电压约为 12V (12000mV)。
    // 这里将输入的百分比(-100 到 100) 乘以 127，得到一个近似的毫伏(mV)值。
    // 电压值的正负号决定了电机的转动方向。
    Motor_Change.spin(directionType::fwd, (int)127 * _input, voltageUnits::mV);
}