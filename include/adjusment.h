#ifndef ADJUSTMENT_H_
#define ADJUSTMENT_H_
#include "PID.h"
#include "fstream"
#include "iostream"
#include "stdio.h"
#include "string"

class Adjustment {
 private:
  float input, output;
  std::ofstream my_ofstream;
  PID pid;
  MyTimer mytimer, globaltimer;

 public:
  void updateInput(float);
  void updateOutput(float);
  void init(char*);
  void stop(void);
  void write(void);
  void startTimer(void);
  void stopTimer(void);
};
#endif

void tuning_forward_p(void);
void tuning_forward_i(void);
void tuning_forward_d(void);
void tuning_rotate_p(void);
void tuning_rotate_i(void);
void tuning_rotate_d(void);
void fastTuningRotatePD();
void fastTuningRotateI();
void fastTuningForwardPD();
void fastTuningForwardI();

void tuning_robot();