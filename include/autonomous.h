#ifndef AUTONOMOUS_H
#define AUTONOMOUS_H

#include "vex.h"
#include "timer.h"

class Autonomous {
    public:
      Autonomous();
      void pid_left(void);
      void pid_right(void);

    private:
      MyTimer auton_timer;
      void init(void);
      void pre_usercontrol(void);
};

#endif