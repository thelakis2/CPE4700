#define _POSIX_C_SOURCE 200809L
#include <stdbool.h>
#include <math.h>
#include <time.h>
#include "pwm.h"
#include "encoder.h"
#include "speed_control.h"
#ifndef SPEED_CONTROL_KP
#define SPEED_CONTROL_KP 0.002
#endif
#ifndef SPEED_CONTROL_KI
#define SPEED_CONTROL_KI 0.001
#endif
typedef struct { int encoder, sign; double kp, ki, integral; struct timespec previous; bool timed; } state_t;
static state_t states[2];
static bool ready;
static bool valid(int m) { return m==0 || m==1; }
static double clip(double x,double l) { return fmax(-l,fmin(l,x)); }
void speed_control_reset(int m) { if(valid(m)) { states[m].integral=0; states[m].timed=false; } }
void speed_control_init(void) {
    for(int m=0;m<2;m++) { states[m]=(state_t){.encoder=m,.sign=1,.kp=SPEED_CONTROL_KP,.ki=SPEED_CONTROL_KI}; pwm_set_duty(m,0); }
    ready=true;
}
int speed_control_set_encoder(int m,int e,int s) {
    if(!ready || !valid(m) || !valid(e) || (s!=1 && s!=-1)) return -1;
    states[m].encoder=e; states[m].sign=s; speed_control_reset(m); return 0;
}
int speed_control_set_gains(int m,double p,double i) {
    if(!ready || !valid(m) || !isfinite(p) || !isfinite(i) || p<0 || i<0) return -1;
    states[m].kp=p; states[m].ki=i; speed_control_reset(m); return 0;
}
double speed_control_compute(int m,double target,double measured,double dt,double limit) {
    if(!ready || !valid(m)) return 0;
    if(!isfinite(target)||!isfinite(measured)||!isfinite(limit)||limit<=0||limit>0.8) { speed_control_reset(m); return 0; }
    if(!isfinite(dt)||dt<=0||dt>0.020) dt=0;
    state_t *s=&states[m]; double error=target-measured, p=s->kp*error;
    double inc=s->ki*error*dt, candidate=clip(s->integral+inc,limit), u=p+candidate;
    if((u<=limit && u>=-limit)||(u>limit && inc<0)||(u< -limit && inc>0)) s->integral=candidate;
    return clip(p+s->integral,limit);
}
void speed_control(int m,double target) {
    if(!ready || !valid(m)) return;
    state_t *s=&states[m]; struct timespec now;
    if(clock_gettime(CLOCK_MONOTONIC,&now)) { pwm_set_duty(m,0); speed_control_reset(m); return; }
    double dt=s->timed ? (now.tv_sec-s->previous.tv_sec)+(now.tv_nsec-s->previous.tv_nsec)/1e9 : 0.001;
    s->previous=now; s->timed=true;
    double u=speed_control_compute(m,target,s->sign*encoder_get_speed(s->encoder),dt,0.80);
    pwm_set_duty(m,(float)u);
}
void speed_control_cleanup(void) {
    if(!ready) return;
    for(int m=0;m<2;m++) { pwm_set_duty(m,0); speed_control_reset(m); }
    ready=false;
}
