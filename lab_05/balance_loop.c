#define _POSIX_C_SOURCE 200809L

#include <math.h>
#include <pthread.h>
#include <stdio.h>
#include <time.h>

#include "balance_loop.h"
#include "encoder.h"
#include "imu.h"
#include "pwm.h"

#ifndef MOTOR0_SIGN
#define MOTOR0_SIGN 1.0
#endif
#ifndef MOTOR1_SIGN
#define MOTOR1_SIGN 1.0
#endif
#ifndef COUNTS_PER_REV
#define COUNTS_PER_REV 700.0
#endif
#ifndef TARGET_PITCH_DEG
#define TARGET_PITCH_DEG 180.0
#endif

#define MAX_DUTY             0.75
#define TILT_LIMIT_DEG       30.0
#define NOMINAL_DT_SECONDS   0.001
#define MAX_DT_SECONDS       0.020
#define RPM_WINDOW_SECONDS   0.050
#define DERIVATIVE_FILTER_TAU 0.020

_Atomic double g_pitch_deg;
_Atomic double g_motor_rpm[2];
_Atomic double g_duty[2];
_Atomic double g_kp;
_Atomic double g_ki;
_Atomic double g_kd;
_Atomic bool g_enabled;
_Atomic bool g_fault;

typedef struct {
    double kp;
    double ki;
    double kd;
    unsigned long revision;
} gains_t;

typedef struct {
    double integral_duty;
    double filtered_derivative;
    double previous_pitch;
    bool derivative_valid;
    bool timing_valid;
    struct timespec previous_time;
    struct timespec rpm_start;
    long previous_counts[2];
    unsigned long applied_revision;
} controller_state_t;

static pthread_mutex_t gain_mutex = PTHREAD_MUTEX_INITIALIZER;
static gains_t gains = { DEFAULT_KP, DEFAULT_KI, DEFAULT_KD, 0 };
static controller_state_t state;

static double elapsed_seconds(struct timespec start, struct timespec end)
{
    return (double)(end.tv_sec - start.tv_sec)
         + (double)(end.tv_nsec - start.tv_nsec) / 1e9;
}

static double clamp_duty(double duty)
{
    if (duty > MAX_DUTY) {
        return MAX_DUTY;
    }
    if (duty < -MAX_DUTY) {
        return -MAX_DUTY;
    }
    return duty;
}

static gains_t read_gains(void)
{
    pthread_mutex_lock(&gain_mutex);
    gains_t snapshot = gains;
    pthread_mutex_unlock(&gain_mutex);
    return snapshot;
}

static void reset_pid_state(void)
{
    state.integral_duty = 0.0;
    state.filtered_derivative = 0.0;
    state.derivative_valid = false;
}

static void command_motors(double command)
{
    const double motor_signs[2] = { MOTOR0_SIGN, MOTOR1_SIGN };

    for (int motor = 0; motor < 2; ++motor) {
        double duty = clamp_duty(command * motor_signs[motor]);
        pwm_set_duty(motor, (float)duty);
        atomic_store(&g_duty[motor], duty);
    }
}

static void report_fault(const char *reason, double pitch, double dt)
{
    if (!atomic_load(&g_fault)) {
        fprintf(stderr, "BALANCE FAULT: %s; pitch=%.3f deg, dt=%.3f ms\n",
                reason, pitch, dt * 1000.0);
    }
    atomic_store(&g_fault, true);
    atomic_store(&g_enabled, false);
    reset_pid_state();
    state.timing_valid = false;
    command_motors(0.0);
}

static void update_rpm(struct timespec now)
{
    double duration = elapsed_seconds(state.rpm_start, now);
    if (duration < RPM_WINDOW_SECONDS) {
        return;
    }

    for (int motor = 0; motor < 2; ++motor) {
        long count = encoder_get_count(motor);
        double count_change = (double)(count - state.previous_counts[motor]);
        double rpm = count_change * 60.0 / (COUNTS_PER_REV * duration);

        atomic_store(&g_motor_rpm[motor], rpm);
        state.previous_counts[motor] = count;
    }
    state.rpm_start = now;
}

void balance_loop_set_gains(double kp, double ki, double kd)
{
    if (!isfinite(kp) || !isfinite(ki) || !isfinite(kd) ||
        kp < 0.0 || ki < 0.0 || kd < 0.0) {
        return;
    }

    pthread_mutex_lock(&gain_mutex);
    if (gains.kp != kp || gains.ki != ki || gains.kd != kd) {
        gains.kp = kp;
        gains.ki = ki;
        gains.kd = kd;
        ++gains.revision;
    }
    atomic_store(&g_kp, kp);
    atomic_store(&g_ki, ki);
    atomic_store(&g_kd, kd);
    pthread_mutex_unlock(&gain_mutex);
}

void balance_loop_get_gains(double *kp, double *ki, double *kd)
{
    gains_t current = read_gains();
    *kp = current.kp;
    *ki = current.ki;
    *kd = current.kd;
}

void balance_loop_enable(bool enabled)
{
    if (enabled) {
        atomic_store(&g_fault, false);
    }
    atomic_store(&g_enabled, enabled);
}

void balance_loop_init(void)
{
    state = (controller_state_t){0};
    atomic_store(&g_enabled, false);
    atomic_store(&g_fault, false);
    balance_loop_set_gains(DEFAULT_KP, DEFAULT_KI, DEFAULT_KD);
    state.applied_revision = read_gains().revision;

    for (int motor = 0; motor < 2; ++motor) {
        state.previous_counts[motor] = encoder_get_count(motor);
        atomic_store(&g_motor_rpm[motor], 0.0);
    }
    if (clock_gettime(CLOCK_MONOTONIC, &state.rpm_start) != 0) {
        report_fault("clock initialization failed", NAN, 0.0);
        return;
    }
    command_motors(0.0);
}

void balance_loop_step(unsigned int loop_count)
{
    (void)loop_count;
    struct timespec now;

    if (clock_gettime(CLOCK_MONOTONIC, &now) != 0) {
        report_fault("clock read failed", NAN, 0.0);
        return;
    }

    double pitch = get_imu_pitch();
    atomic_store(&g_pitch_deg, pitch);
    update_rpm(now);

    double dt = state.timing_valid
              ? elapsed_seconds(state.previous_time, now)
              : NOMINAL_DT_SECONDS;
    double relative_pitch = remainder(pitch - TARGET_PITCH_DEG, 360.0);

    if (!isfinite(pitch)) {
        report_fault("invalid IMU pitch", pitch, dt);
        return;
    }
    if (fabs(relative_pitch) > TILT_LIMIT_DEG) {
        report_fault("tilt exceeds 30 degrees from upright", pitch, dt);
        return;
    }
    if (dt <= 0.0 || dt > MAX_DT_SECONDS) {
        report_fault("control timing outside limits", pitch, dt);
        return;
    }

    state.previous_time = now;
    state.timing_valid = true;
    if (!atomic_load(&g_enabled)) {
        reset_pid_state();
        command_motors(0.0);
        return;
    }

    gains_t current = read_gains();
    if (current.revision != state.applied_revision) {
        reset_pid_state();
        state.applied_revision = current.revision;
    }

    /* Derivative on measurement: for a fixed target, d(error)/dt = -d(pitch)/dt.
     * Wrap pitch changes to avoid an artificial jump at +/-180 degrees. */
    double raw_derivative = 0.0;
    if (state.derivative_valid) {
        double pitch_change = remainder(pitch - state.previous_pitch, 360.0);
        raw_derivative = -pitch_change / dt;
    }
    state.previous_pitch = pitch;
    state.derivative_valid = true;

    double alpha = dt / (DERIVATIVE_FILTER_TAU + dt);
    state.filtered_derivative += alpha *
        (raw_derivative - state.filtered_derivative);

    double error = -relative_pitch;
    double proportional = current.kp * error;
    double derivative = current.kd * state.filtered_derivative;
    if (current.ki == 0.0) {
        state.integral_duty = 0.0;
    }

    double increment = current.ki * error * dt;
    double candidate_integral = clamp_duty(state.integral_duty + increment);
    double candidate_output = proportional + candidate_integral + derivative;

    bool within_limits = fabs(candidate_output) <= MAX_DUTY;
    bool reducing_high_saturation = candidate_output > MAX_DUTY && increment < 0.0;
    bool reducing_low_saturation = candidate_output < -MAX_DUTY && increment > 0.0;
    if (within_limits || reducing_high_saturation || reducing_low_saturation) {
        state.integral_duty = candidate_integral;
    }

    double command = clamp_duty(proportional + state.integral_duty + derivative);
    command_motors(command);
}

void balance_loop_cleanup(void)
{
    atomic_store(&g_enabled, false);
    reset_pid_state();
    command_motors(0.0);
}
