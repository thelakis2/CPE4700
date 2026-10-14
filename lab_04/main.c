#define _POSIX_C_SOURCE 200809L
#include <errno.h>
#include <signal.h>
#include <stdbool.h>
#include <stdatomic.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <time.h>
#include "pwm.h"
#include "encoder.h"
#include "speed_control.h"

atomic_bool running = ATOMIC_VAR_INIT(true);
static volatile sig_atomic_t stop_requested;
#define PERIOD_NS 1000000L
#define MAX_SAMPLES 170000
#define DEADBAND_STEP 0.02
#define DEADBAND_MAX_STEPS 40 /* Existing PWM module clamps at +/-0.80. */
#define REST_SECONDS 1.0
#define TRIAL_SECONDS 1.0
#define MOTION_COUNTS 3L

typedef struct {
    double time_ms, command, speed;
    long position;
    int direction, step;
    bool trial;
} sample_t;
static sample_t samples[MAX_SAMPLES];
static size_t sample_count;

static void interrupt_handler(int sig) { (void)sig; stop_requested = 1; }
static double seconds_between(struct timespec a, struct timespec b)
{
    return (double)(b.tv_sec-a.tv_sec)+(double)(b.tv_nsec-a.tv_nsec)/1e9;
}
static int get_time(struct timespec *t)
{
    if (clock_gettime(CLOCK_MONOTONIC,t) == 0) return 0;
    perror("clock_gettime"); return -1;
}
static void advance(struct timespec *t)
{
    t->tv_nsec += PERIOD_NS;
    if (t->tv_nsec >= 1000000000L) { t->tv_nsec -= 1000000000L; ++t->tv_sec; }
}
static int wait_next(struct timespec *deadline)
{
    struct timespec now;
    advance(deadline);
    if (get_time(&now)) return -1;
    while (seconds_between(now,*deadline) <= 0.0) advance(deadline);
    int rc;
    do { rc=clock_nanosleep(CLOCK_MONOTONIC,TIMER_ABSTIME,deadline,NULL); }
    while (rc==EINTR && !stop_requested);
    if (rc && rc!=EINTR) { fprintf(stderr,"Sleep: %s\n",strerror(rc)); return -1; }
    return 0;
}
static int parse_id(const char *s)
{
    if (!strcmp(s,"0")) return 0;
    if (!strcmp(s,"1")) return 1;
    return -1;
}
static double triangle(double t)
{
    double phase=t-2.0*(int)(t/2.0);
    return phase<1.0 ? 100.0-200.0*phase : -100.0+200.0*(phase-1.0);
}
static double step_command(double t)
{
    if (t<1.0) return 0.0;
    if (t<2.0) return 0.60;
    if (t<4.0) return 0.0;
    if (t<5.0) return -0.60;
    if (t<6.0) return 0.60;
    return 0.0;
}
static int write_log(FILE *fp, int mode)
{
    const char *header=mode==1 ? "time_ms,target_speed,measured_speed\n" :
        mode==2 ? "time_ms,duty_cycle,speed,position,direction,duty_step,phase\n" :
                  "time_ms,duty_cycle,speed,position\n";
    if (fputs(header,fp)==EOF) return -1;
    for (size_t i=0;i<sample_count;++i) {
        sample_t *s=&samples[i]; int rc;
        if (mode==1) rc=fprintf(fp,"%.3f,%.6f,%.6f\n",s->time_ms,s->command,s->speed);
        else if (mode==2) rc=fprintf(fp,"%.3f,%.2f,%.6f,%ld,%d,%d,%s\n",
            s->time_ms,s->command,s->speed,s->position,s->direction,s->step,
            s->trial ? "trial" : "rest");
        else rc=fprintf(fp,"%.3f,%.6f,%.6f,%ld\n",s->time_ms,s->command,s->speed,s->position);
        if (rc<0) return -1;
    }
    return fflush(fp)==0 ? 0 : -1;
}
int main(int argc,char **argv)
{
    if (argc<2 || argc>4 || (strcmp(argv[1],"step") && strcmp(argv[1],"speed") && strcmp(argv[1],"deadband"))) {
        fprintf(stderr,"Usage: %s step|speed|deadband [motor:0|1] [encoder:0|1]\n",argv[0]);
        return EXIT_FAILURE;
    }
    int mode=!strcmp(argv[1],"speed") ? 1 : !strcmp(argv[1],"deadband") ? 2 : 0;
    int motor=argc>=3 ? parse_id(argv[2]) : 0;
    int encoder=argc>=4 ? parse_id(argv[3]) : motor;
    if (motor<0 || encoder<0) { fprintf(stderr,"IDs must be 0 or 1.\n");return EXIT_FAILURE; }
    struct sigaction sa={0};sa.sa_handler=interrupt_handler;sigemptyset(&sa.sa_mask);
    if (sigaction(SIGINT,&sa,NULL) || sigaction(SIGTERM,&sa,NULL)) { perror("sigaction");return EXIT_FAILURE; }
    const char *filename=mode==1 ? "speed_control.csv" : mode==2 ? "deadband.csv" : "step_response.csv";
    FILE *fp=fopen(filename,"w");
    if (!fp) { perror(filename);return EXIT_FAILURE; }
    bool pwm_ready=false,encoder_ready=false,controller_ready=false;
    int result=EXIT_FAILURE;
    bool found[2]={false,false},completed[2]={false,false};
    double deadband[2]={0.0,0.0};
    struct timespec start,now,deadline,phase_start;
    int dir_index=0,duty_step=1;
    bool trial=false;
    long baseline=0,last_count=0,motion=0;
    double last_duty=0.0;

    if (pwm_init()!=0) { fprintf(stderr,"PWM init failed.\n");goto cleanup; }
    pwm_ready=true;pwm_set_duty(0,0);pwm_set_duty(1,0);
    if (encoder_init()!=0) { fprintf(stderr,"Encoder init failed.\n");goto cleanup; }
    encoder_ready=true;
    if (mode==1) {
        speed_control_init();controller_ready=true;
        if (speed_control_set_encoder(motor,encoder,1)!=0) goto cleanup;
    }
    printf("%s test: motor %d, encoder %d. Keep wheels lifted. Ctrl+C stops.\n",argv[1],motor,encoder);
    if (mode==2) printf("Deadband: 0.02 steps up to +/-0.80; 1 s rest and 1 s trial per duty; motion threshold %ld counts.\n",MOTION_COUNTS);
    if (get_time(&start)) goto cleanup;
    deadline=start;phase_start=start;
    while (!stop_requested) {
        if (get_time(&now)) goto cleanup;
        double t=seconds_between(start,now),command=0.0;
        long position=encoder_get_count(encoder);
        double measured=encoder_get_speed(encoder);
        if (mode==2) {
            double phase_time=seconds_between(phase_start,now);
            if (!trial && phase_time>=REST_SECONDS) {
                trial=true;phase_start=now;baseline=position;last_count=position;motion=0;
            } else if (trial) {
                /* Accumulate count changes, not stale encoder speed. */
                motion+=labs(position-last_count);last_count=position;
                if (phase_time>=TRIAL_SECONDS) {
                    bool moved=motion>=MOTION_COUNTS && labs(position-baseline)>=MOTION_COUNTS;
                    int direction=dir_index==0 ? 1 : -1;
                    if (moved) {
                        found[dir_index]=true;
                        deadband[dir_index]=direction*duty_step*DEADBAND_STEP;
                        printf("%s deadband: %+.2f (%ld net counts).\n",
                            dir_index==0 ? "Positive" : "Negative",deadband[dir_index],position-baseline);
                    }
                    if (moved || duty_step==DEADBAND_MAX_STEPS) {
                        completed[dir_index]=true;
                        if (!moved) printf("No motion detected in %s direction through %+.2f. Check motor/encoder pairing.\n",
                            dir_index==0 ? "positive" : "negative",direction*0.80);
                        if (dir_index==1) break;
                        dir_index=1;duty_step=1;
                    } else ++duty_step;
                    trial=false;phase_start=now;
                }
            }
            command=trial ? (dir_index==0 ? 1 : -1)*duty_step*DEADBAND_STEP : 0.0;
        } else {
            if (t >= (mode==1 ? 6.0 : 7.0)) break;
            command=mode==1 ? triangle(t) : step_command(t);
        }
        if (mode==1) speed_control(motor,command);
        else if (command!=last_duty) { pwm_set_duty(motor,(float)command);last_duty=command; }
        if (sample_count>=MAX_SAMPLES) { fprintf(stderr,"Log buffer full.\n");goto cleanup; }
        samples[sample_count++]=(sample_t){t*1000.0,command,measured,position,
            mode==2 ? (dir_index==0 ? 1 : -1) : 0,duty_step,trial};
        if (wait_next(&deadline)) goto cleanup;
    }
    result=stop_requested ? EXIT_FAILURE : EXIT_SUCCESS;
cleanup:
    if (controller_ready) { speed_control(motor,0);speed_control_cleanup(); }
    if (pwm_ready) { pwm_set_duty(0,0);pwm_set_duty(1,0); }
    atomic_store(&running,false);
    if (encoder_ready) encoder_cleanup();
    if (pwm_ready) pwm_cleanup();
    if (write_log(fp,mode)) { perror("CSV write");result=EXIT_FAILURE; }
    if (fclose(fp)) { perror("CSV close");result=EXIT_FAILURE; }
    if (mode==2) {
        FILE *summary=fopen("deadband_summary.csv","w");
        if (!summary) { perror("deadband_summary.csv");result=EXIT_FAILURE; }
        else {
            int rc=fprintf(summary,"direction,deadband_duty,status\n");
            for (int i=0;i<2;++i) {
                const char *name=i==0 ? "positive" : "negative";
                if (found[i]) { if (fprintf(summary,"%s,%+.2f,detected\n",name,deadband[i])<0) rc=-1; }
                else if (fprintf(summary,"%s,,%s\n",name,completed[i] ? "not_detected_up_to_0.80" : "incomplete")<0) rc=-1;
            }
            if (fclose(summary)) rc=-1;
            if (rc<0) { perror("Summary write");result=EXIT_FAILURE; }
        }
    }
    printf("%zu samples saved to %s.\n",sample_count,filename);
    return result;
}
