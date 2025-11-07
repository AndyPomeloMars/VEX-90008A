#include "PID.h"
#include "math-tools.h"

PID::PID(): firstTime(true), arrived(false), IMax(20), IRange(50), jumpTime(50) {
  myTimer.reset(); // 重置计时器，用于后续判断是否达到稳定状态
}

void PID::setFirstTime() {
    firstTime = true;
}

void PID::setCoefficient(float _kp, float _ki, float _kd) {
  kp = _kp; // P系数：决定了控制器对当前误差的反应速度
  ki = _ki; // I系数：用于消除系统的稳态误差
  kd = _kd; // D系数：用于预测误差变化趋势，抑制过冲和振荡
}

void PID::setTarget(float _target) {
    target = _target;
}

// 设置积分项的最大绝对值（积分限幅）
// 这是防止积分饱和的重要参数。当积分累积过大时，会限制其最大值
void PID::setIMax(float _IMax) {
    IMax = _IMax;
}

// 设置积分有效范围
// 这是一种积分分离的策略，当误差很大（P项很大）时，暂时不进行积分，可以防止系统初期因巨大误差导致的积分饱和，加快响应速度。
void PID::setIRange(float _IRange) {
    IRange = _IRange;
}

// 设置可接受的误差容忍度
// 当当前误差的绝对值小于此值时，认为可能已接近目标
void PID::setErrorTolerance(float _errorTol) {
    errorTol = _errorTol;
}

// 设置可接受的微分项容忍度
// 当微分项的绝对值小于此值时，认为系统变化趋于稳定
void PID::setDTolerance(float _DTol) {
    DTol = _DTol;
}

// 设置可接受的误差容忍度
// 系统必须在误差和微分容忍度范围内持续此段时间，才认为已稳定到达目标
void PID::setJumpTime(float _jumpTime) {
    jumpTime = _jumpTime;
}

bool PID::targetArrived() {
    return arrived;
}

float PID::getOutput() {
    return output;
}

void PID::update(float input) {
  errorCrt = target - input; // 当前误差 = 目标值 - 当前测量值

  // 处理首次运行的情况
  if (firstTime) {
    firstTime = false;
    errorPrev = errorCrt;
    errorInt = 0;
  }

  // 计算比例项(P)
  P = kp * errorCrt; // P项输出与当前误差成正比，用于快速响应误差

  // 计算微分项(D)
  errorDev = errorCrt - errorPrev;  // 误差变化量 = 当前误差 - 上一次误差
  errorPrev = errorCrt;
  D = kd * errorDev; // D项输出与误差变化率成正比，用于预测趋势，增加系统阻尼，防止超调

  // 计算积分项(I)，并进行抗饱和处理
  // 积分分离：只有当P项（即误差）在一定范围内时，才累积积分
  if (fabs(P) >= IRange)
    errorInt = 0; // 如果误差过大，清除积分累加，防止在远离目标时积分过度累积
  else {
    errorInt += errorCrt;  // 在误差范围内，累积误差
    // 积分限幅：限制积分累加值的最大值
    if (fabs(errorInt) * ki > IMax)
        errorInt = getSign(errorInt) * IMax / ki; // 如果积分项的贡献超过了IMax，则将其限制在IMax对应的值
  }

  // 积分清零策略：
  // 1. 当误差符号与积分累加符号相反时（说明系统已越过目标点），清零积分，防止过冲。
  // 2. 当误差已经非常小（在容忍度内）时，也清零积分，因为此时不再需要积分来消除稳态误差。
  if (getSign(errorInt) != getSign(errorCrt) || (fabs(errorCrt) <= errorTol))
    errorInt = 0;

  // 计算最终的积分项贡献值
  I = ki * errorInt;

  // 判断是否到达目标并保持稳定
  // 条件：当前误差足够小并且系统状态足够稳定
  if (fabs(errorCrt) <= errorTol && fabs(D) <= DTol)
    // 如果满足稳定条件，检查稳定持续时间是否足够长
    if (myTimer.getTime() >= jumpTime)
        arrived = true; // 如果稳定时间达标，则设置“已到达”标志
  else
    myTimer.reset(); // 如果系统不稳定（误差或变化率超出容忍度），则重置计时器

  output = P + I + D;
}