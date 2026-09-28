#include <stdio.h>
#include <stdbool.h>
#include <math.h>

#include "encoder.h"
#include "pwm.h"
#include "step_test.h"

#define MOTOR 0
/*oiuhwsds*/
#define DEADBAND_STEP 0.02
#define DEADBAND_HOLD_LOOPS 500     
#define DEADBAND_SPEED_THRESHOLD 1.0  

#define PHASE_1_LEN 1000   /* 0.0 for 1 s */
#define PHASE_2_LEN 1000   /* +0.6 for 1 s */
#define PHASE_3_LEN 2000   /* 0.0 for 2 s */
#define PHASE_4_LEN 1000   /* -0.6 for 1 s */
#define PHASE_5_LEN 1000   /* +0.6 for 1 s */

#define STEP_DUTY 0.6

typedef enum
{
    DB_POSITIVE_SEARCH,
    DB_POSITIVE_DONE,
    DB_NEGATIVE_SEARCH,
    DB_DONE
} deadband_state_t;

static bool step_test_active = false;
static FILE *step_response_csv = NULL;

static deadband_state_t db_state = DB_POSITIVE_SEARCH;
static double db_current_duty = 0.0;
static unsigned int db_phase_start_loop = 0;
static double positive_deadband = -1.0;
static double negative_deadband = -1.0;

static unsigned int step_phase_start_loop = 0;
static int step_phase = 0; /* 0..5, see PHASE_* comments above */

void step_test_init(void)
{
    step_response_csv = fopen("step_response.csv", "w");
    if (step_response_csv == NULL)
    {
        perror("step_response.csv");
        step_test_active = false;
        return;
    }
    fprintf(step_response_csv, "time_ms,duty_cycle,speed,position\n");

    db_state = DB_POSITIVE_SEARCH;
    db_current_duty = 0.0;
    db_phase_start_loop = 0;
    positive_deadband = -1.0;
    negative_deadband = -1.0;

    step_phase_start_loop = 0;
    step_phase = 0;

    step_test_active = true;
}

static bool run_deadband_characterization(unsigned int loop_count)
{
    double speed = enc_get_speed(MOTOR);
    unsigned int elapsed = loop_count - db_phase_start_loop;

    switch (db_state)
    {
    case DB_POSITIVE_SEARCH:
        pwm_set_duty(MOTOR, (float)db_current_duty);
        if (elapsed >= DEADBAND_HOLD_LOOPS)
        {
            if (fabs(speed) >= DEADBAND_SPEED_THRESHOLD)
            {
                positive_deadband = db_current_duty;
                printf("[DEADBAND] positive_deadband = %.2f\n", positive_deadband);
                pwm_set_duty(MOTOR, 0.0f);
                db_state = DB_POSITIVE_DONE;
                db_phase_start_loop = loop_count;
            }
            else
            {
                db_current_duty += DEADBAND_STEP;
                db_phase_start_loop = loop_count;
                if (db_current_duty > 0.8)
                {
                    positive_deadband = db_current_duty;
                    pwm_set_duty(MOTOR, 0.0f);
                    db_state = DB_POSITIVE_DONE;
                    db_phase_start_loop = loop_count;
                }
            }
        }
        break;

    case DB_POSITIVE_DONE:
        if (elapsed >= 200)
        {
            db_current_duty = 0.0;
            db_state = DB_NEGATIVE_SEARCH;
            db_phase_start_loop = loop_count;
        }
        break;

    case DB_NEGATIVE_SEARCH:
        pwm_set_duty(MOTOR, (float)db_current_duty);
        if (elapsed >= DEADBAND_HOLD_LOOPS)
        {
            if (fabs(speed) >= DEADBAND_SPEED_THRESHOLD)
            {
                negative_deadband = db_current_duty;
                printf("[DEADBAND] negative_deadband = %.2f\n", negative_deadband);
                pwm_set_duty(MOTOR, 0.0f);
                db_state = DB_DONE;
                db_phase_start_loop = loop_count;
            }
            else
            {
                db_current_duty -= DEADBAND_STEP;
                db_phase_start_loop = loop_count;
                if (db_current_duty < -0.8)
                {
                    negative_deadband = db_current_duty;
                    pwm_set_duty(MOTOR, 0.0f);
                    db_state = DB_DONE;
                    db_phase_start_loop = loop_count;
                }
            }
        }
        break;

    case DB_DONE:
        return true;
    }

    return (db_state == DB_DONE);
}

static bool run_step_response(unsigned int loop_count)
{
    unsigned int elapsed = loop_count - step_phase_start_loop;
    double duty = 0.0;

    switch (step_phase)
    {
    case 0: /* 0.0 for 1 s */
        duty = 0.0;
        if (elapsed >= PHASE_1_LEN)
        {
            step_phase = 1;
            step_phase_start_loop = loop_count;
        }
        break;
    case 1: /* +0.6 for 1 s */
        duty = STEP_DUTY;
        if (elapsed >= PHASE_2_LEN)
        {
            step_phase = 2;
            step_phase_start_loop = loop_count;
        }
        break;
    case 2: /* 0.0 for 2 s */
        duty = 0.0;
        if (elapsed >= PHASE_3_LEN)
        {
            step_phase = 3;
            step_phase_start_loop = loop_count;
        }
        break;
    case 3: /* -0.6 for 1 s */
        duty = -STEP_DUTY;
        if (elapsed >= PHASE_4_LEN)
        {
            step_phase = 4;
            step_phase_start_loop = loop_count;
        }
        break;
    case 4: /* +0.6 for 1 s */
        duty = STEP_DUTY;
        if (elapsed >= PHASE_5_LEN)
        {
            step_phase = 5;
            step_phase_start_loop = loop_count;
        }
        break;
    case 5: /* 0.0, test complete */
    default:
        duty = 0.0;
        pwm_set_duty(MOTOR, 0.0f);
        return true;
    }

    pwm_set_duty(MOTOR, (float)duty);

    double speed = enc_get_speed(MOTOR);
    long position = enc_get_position(MOTOR);
    if (step_response_csv != NULL)
    {
        fprintf(step_response_csv, "%u,%f,%f,%ld\n", loop_count, duty, speed, position);
    }

    return false;
}

int step_test(unsigned int loop_count)
{
    if (!step_test_active)
    {
        return 0;
    }

    if (db_state != DB_DONE)
    {
        run_deadband_characterization(loop_count);
        if (db_state == DB_DONE)
        {
            step_phase_start_loop = loop_count;
        }
        return 1;
    }

    bool finished = run_step_response(loop_count);
    if (finished)
    {
        step_test_active = false;
        return 0;
    }

    return 1;
}

void step_test_cleanup(void)
{
    printf("[RESULT] positive_deadband = %.2f\n", positive_deadband);
    printf("[RESULT] negative_deadband = %.2f\n", negative_deadband);

    if (step_response_csv != NULL)
    {
        fflush(step_response_csv);
        fclose(step_response_csv);
        step_response_csv = NULL;
    }
}