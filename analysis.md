### 1. `main.cpp` - 主程序文件

这是整个项目的入口和核心。它负责组织程序的整体流程，包括自动阶段、手动阶段以及它们之间的切换。

*   **`#include "vex.h"` 等**: 包含了所有需要的头文件，将各个模块的功能引入到主程序中。
*   **`using namespace vex;`**: 使用 `vex` 命名空间，这样就不需要在每个 VEX 库函数前写 `vex::`。
*   **`competition Competition;`**: 创建一个 VEX 竞赛控制器实例，用于管理自动和手动控制阶段。
*   **`auton_strategy`**: 一个静态整型变量，用于存储和切换自动赛策略（例如，选择左边路线还是右边路线）。
*   **`auto1(void)` 函数**: 这是自动赛的主函数。它使用一个 `switch` 语句，根据 `auton_strategy` 变量的值来调用不同的自动程序（`auton_left()` 或 `auton_right()`）。
*   **`usercontrol(void)` 函数**: 这是手动控制阶段的函数，包含一个无限 `while(true)` 循环，不断地读取手柄输入并控制机器人。
    *   **底盘移动控制 (`Base Movement Control`)**: 读取手柄摇杆 `A1` 和 `A3` 的值，并设置了死区 (`JOYSTICK_DEADZONE`) 以防止摇杆漂移。通过计算 `A3 + A1` 和 `A3 - A1` 来实现坦克式或街机式的混合控制，分别驱动左右两侧的轮子。
    *   **进球和飞轮控制 (`Intake Control`)**: 根据手柄上的 `R1`, `R2`, `L1`, `L2` 按钮来控制进球、吐球以及上下层得分等组合动作。
    *   **气缸控制 (`Mobile Goal UP Control` 等)**:
        *   `A` 和 `B` 按钮控制前方气缸 (`PistonFront`) 的伸出和缩回。
        *   `X` 按钮用来切换 `piston_up_state` 的状态，从而控制上方气缸 (`PistonUp`)。
    *   **特殊功能和调试**:
        *   当 `LEFT` 和 `RIGHT` 键同时按下时，会执行自动赛程序 `auto1()`，这通常用于调试。
        *   当 `LEFT` 和 `A` 键同时按下时，会启动 IMU（惯性测量单元）的校准程序。
        *   当 `LEFT` 键按下，同时拨动摇杆 `A1` 时，可以切换自动赛策略 `auton_strategy`。
    *   **屏幕显示**: 在 `while` 循环的末尾，代码会将 IMU 的读数、机器人位置等信息打印到 V5 Brain 的屏幕上，便于实时监控和调试。
*   **`main()` 函数**: 程序的入口点。
    *   `Competition.autonomous(auto1);`: 将 `auto1` 函数注册为自动赛阶段要执行的函数。
    *   `Competition.drivercontrol(usercontrol);`: 将 `usercontrol` 函数注册为手动赛阶段要执行的函数。
    *   `vexcodeInit();`: 调用 VEX 官方的初始化函数。
    *   `thread ThreadController(defineController);`: 创建一个新线程来专门运行 `defineController` 函数，该函数负责不断刷新手柄的输入状态。

### 2. `adjusment.cpp` - PID 参数调试文件

这个文件主要用于调试和优化机器人的 PID 控制参数。PID 是一种经典的控制算法，用于让机器人精确地移动到目标位置或旋转到目标角度。

*   **`Adjustment` 类**: 一个辅助类，用于将调试过程中的数据（如输入值和输出值）写入到 SD 卡的文件中，方便后续用电脑进行分析。
*   **`tuning_forward_p()` / `tuning_forward_d()` / `tuning_rotate_p()` / `tuning_rotate_d()`**: 这些函数通过循环，系统地、自动地测试一系列不同的 P、I、D 参数值。每次测试都会执行一个标准的动作（如前进600mm或旋转180度），并将结果记录下来。开发者通过分析这些记录文件，可以找到最优的参数组合。
*   **`fastTuningRotatePD()` / `fastTuningForwardPD()` 等**: 这些是快速手动调参函数。开发者可以在手动模式下，通过手柄上的 X, Y, A, B 按钮实时增加或减小 `kp` 和 `kd` 的值，并通过 `DOWN` 按钮立即执行一个动作来观察效果。这比自动测试更直观、更快捷。

### 3. `controller.cpp` - 手柄控制器输入处理

这个文件专门负责处理手柄的输入。

*   **全局变量**: 定义了一系列全局变量，如 `A1`, `A2`, `L1`, `R1`, `X`, `last_X` 等，用于存储手柄所有摇杆和按钮的当前状态以及上一时刻的状态。
*   **`defineController()` 函数**: 这个函数在一个独立的线程中运行。它以很高的频率（每10毫秒一次）循环执行，不断地读取手柄的输入，并更新上面定义的全局变量。
    *   **为什么要这样做？**: 将手柄输入处理放在独立线程中，可以确保程序在任何地方都能立即获取到最新的手柄状态，而不需要在每个需要的地方都调用一次读取函数。同时，通过比较当前状态（如 `X`）和上一时刻的状态（如 `last_X`），可以轻松地检测出按钮是“被按下了” (`X && !last_X`) 还是“被持续按住”，这对于很多控制逻辑至关重要。

### 4. `autonomous.cpp` - 自动赛程序文件

这个文件包含了机器人在15秒自动阶段执行的所有预设程序。

*   **`auton_init()`**: 自动赛开始时的初始化函数，用于重置编码器读数和计时器。
*   **`auton_pre_usercontrol()`**: 自动赛结束、进入手动赛之前执行的函数，用于停止所有电机、收回气缸，并将自动赛耗时打印到屏幕。
*   **`auton_left()` 和 `auton_right()`**: 两个主要的自动赛程序，分别对应从场地左侧或右侧出发的策略。这些函数是机器人动作指令的序列，通过调用 `differential-base.cpp` 中定义的各种移动和旋转函数（如 `posForwardAbs`, `pidRotateAbs`）以及其他模块的函数（如 `spinIntaker1`, `setPistonFront`）来完成一系列复杂任务，例如：前进吸球、转向、将球放置到指定位置等。每一行代码都精确地控制着机器人的一个动作。

### 5. `basic-functions.cpp` - 基础功能函数

这个文件提供了一些简单的、用于控制机器人上各个小部件（电机、气缸）的封装函数。

*   **`spinIntaker1(float _input)`**: 控制进球电机1的转动。输入范围是 -100 到 100，代表功率百分比。
*   **`spinIntaker2(float _input)`**: 控制进球电机2。
*   **`setPistonUp(bool _input)` / `setPistonDown(bool _input)` / `setPistonFront(bool _input)`**: 分别控制上、下、前三个气缸的伸出 (`true`) 和缩回 (`false`)。
    *   **封装的好处**: 将底层的硬件控制代码（如 `Motor_Intaker1.spin(...)`）包装成一个简单的函数，使得在 `main.cpp` 或 `autonomous.cpp` 中调用时，代码更简洁、更易读。

### 6. `change-1DOF.cpp` - 特定机构控制

这个文件用于控制一个被称为 "Change" 的单自由度（1DOF）机构。

*   **`spinChange(float _input)`**: 控制 `Motor_Change` 电机的转动，输入同样是 -100 到 100 的功率百分比。这可能是用于传递或发射游戏元素的飞轮或其他类似装置。

### 7. `differential-base.cpp` - 差速底盘控制

这是整个项目中最核心的模块之一，负责机器人的所有移动和旋转功能。

*   **位置和角度获取 (`get...` 系列函数)**:
    *   `getLeftPos()`, `getRightPos()`: 读取左右两侧电机编码器的值，并计算出两侧轮子走过的距离。
    *   `getForwardPos()`: 通过平均左右两侧的距离，得到机器人前进的直线距离。
    *   `getHeading()`: 读取 IMU 的值，得到机器人当前朝向的角度。
    *   `reset...Pos()`: 重置编码器的计数值，用于设定新的起点。
*   **底层电机控制 (`move...`, `lock...`, `unlock...` 系列)**:
    *   `moveLeft(float _input)`, `moveRight(float _input)`: 直接控制左右两侧所有电机以指定的功率转动。
    *   `lockLeft()`, `lockRight()`: 将电机设置为 `hold` 模式，使其产生制动力，锁住底盘。
    *   `unlockLeft()`, `unlockRight()`: 将电机设置为 `coast` 模式，使其自由转动。
*   **高级移动控制 (封装了 PID 和计时器)**:
    *   **直线移动**:
        *   `timerForward()`: 在指定时间内以指定功率前进，并带有简单的陀螺仪校正。
        *   `posForwardAbs()` / `posForwardRel()`: 以指定功率前进指定的绝对距离或相对距离。
        *   `pidForwardAbs()` / `pidForwardRel()`: **使用 PID 控制**精确地前进到指定的绝对或相对位置，这是自动赛中最常用的精确移动函数。
    *   **旋转**:
        *   `timerRotate()`: 在指定时间内以指定功率旋转。
        *   `angleRotateAbs()` / `angleRotateRel()`: 以指定功率旋转指定的绝对角度或相对角度。
        *   `pidRotateAbs()` / `pidRotateRel()`: **使用 PID 控制**精确地旋转到指定的绝对或相对角度，同样是自动赛中保证精度的关键。

### 8. `math-tools.cpp` - 数学工具

提供了一些项目中会用到的简单数学函数。

*   **`getSign(float _input)`**: 返回一个数的符号（正数为1，负数为-1，零为0）。
*   **`deg2rad(float deg)` / `rad2deg(float rad)`**: 角度和弧度的转换。
*   **`calAbsDeltaAng(float _delta_ang)`**: 计算角度差，并将其处理到 -180 到 180 度之间，避免机器人“绕远路”旋转（例如，从350度转到10度，应该只转20度，而不是-340度）。

### 9. `timer.cpp` - 自定义计时器

实现了一个简单的计时器类 `MyTimer`。

*   **`MyTimer()`**: 构造函数，在创建对象时记录当前时间。
*   **`reset()`**: 重置计时器，重新开始计时。
*   **`getTime()`**: 返回从计时器创建或上次重置以来经过的毫秒数。这个类在 PID 控制（计算时间差）和自动程序（等待、延时）中非常有用。

### 10. `PID.cpp` - PID 控制器实现

这是一个通用的 PID 控制器类的实现。

*   **`PID()`**: 构造函数，初始化内部变量。
*   **`setCoefficient()`, `setTarget()`, `setIMax()` 等**: 设置 PID 控制器的各种参数，如 Kp, Ki, Kd 值、目标值、积分上限等。
*   **`update(float input)`**: PID 控制器的核心。每次调用时，传入当前的测量值（如机器人位置或角度），它会根据 P（比例）、I（积分）、D（微分）三项计算出一个输出值（如电机功率），用于驱动机器人去逼近目标。
*   **`targetArrived()`**: 判断是否已经达到目标。它不仅检查误差是否足够小，还检查速度是否也足够小，以防止机器人过冲。

### 11. `robot-config.cpp` - 机器人硬件配置

这个文件负责定义和初始化机器人所有的硬件设备。

*   **`brain Brain;`**: 创建 V5 主脑的实例。
*   **`controller Controller = controller(primary);`**: 创建手柄实例。
*   **`motor Motor_... = motor(PORT, ...);`**: 创建所有电机的实例，并指定它们连接的端口号、齿轮比和是否反转。
*   **`pneumatics Piston_... = pneumatics(...);`**: 创建所有气缸（气动装置）的实例，并指定它们连接的三线端口。
*   **`inertial IMU = inertial(PORT);`**: 创建 IMU（惯性传感器）实例。
*   **`vexcodeInit(void)`**: VEX 官方的初始化函数，这里被用来在程序开始时校准 IMU，并向手柄屏幕上打印提示信息。