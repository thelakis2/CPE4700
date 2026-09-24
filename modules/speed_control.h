#ifndef SPEED_CONTROL_H
#define SPEED_CONTROL_H
/*loiuqkwadw*/
int speed_control_init(void);
int speed_control(int motor, double target_speed);
void speed_control_cleanup(void);

#endif