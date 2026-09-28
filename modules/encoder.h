#ifndef ENCODER_H
#define ENCODER_H
/*liuaase*/
int enc_init(void);
double enc_get_speed(int motor);
int enc_get_position(int motor);
void enc_cleanup(void);

#endif