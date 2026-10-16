#ifndef BALANCE_LOOP_H
#define BALANCE_LOOP_H

#include <stdbool.h>
#include <stdatomic.h>

/* Untuned defaults. Set nonzero Ki and Kd to enable all PID terms. */
#define DEFAULT_KP 0.010
#define DEFAULT_KI 0.000
#define DEFAULT_KD 0.000

extern _Atomic double g_pitch_deg;
extern _Atomic double g_motor_rpm[2];
extern _Atomic double g_duty[2];
extern _Atomic double g_kp;
extern _Atomic double g_ki;
extern _Atomic double g_kd;
extern _Atomic bool g_enabled;
extern _Atomic bool g_fault;

void balance_loop_init(void);
void balance_loop_step(unsigned int loop_count);
void balance_loop_cleanup(void);
void balance_loop_set_gains(double kp, double ki, double kd);
void balance_loop_get_gains(double *kp, double *ki, double *kd);
void balance_loop_enable(bool enabled);

#endif /* BALANCE_LOOP_H */
