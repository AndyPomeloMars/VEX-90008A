#ifndef AUTONOMOUS_H_
#define AUTONOMOUS_H_

void auton_init(void);
void auton_pre_usercontrol(void);

void auton_skill(void);
void auton_left();
void auton_right();
void auton_plus_a(bool _mirror_flag, bool is_elimination=0);
void auton_minus_a(bool _mirror_flag);
void auton_alliance();

#endif