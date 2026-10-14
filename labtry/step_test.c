#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <stdbool.h>
#include <math.h>

#include "encoder.h"
#include "pwm.h"
#include "step_test.h"

bool step_test_active = true;
static FILE *step_test_data = NULL;

void step_test_init()
{
    step_test_data = fopen("step_test_data.csv", "w");
    if (step_test_data == NULL)
    {
        perror("step_test_data.csv");
        step_test_active = false;
        return;
    }

    step_test_active = true;
}

#define STEP_TEST_LENGTH 10000
int step_test(unsigned int loop_count)
{
    if(!step_test_active)
    {
        printf("step test was not active\n");
        return 0;
    }

    if (step_test_data != NULL)
    {
        // printf("%ld,%.6f\n", encoder_get_count(1), encoder_get_speed(1));
        fprintf(step_test_data, "%d,%ld,%.6f\n", loop_count, encoder_get_count(1), encoder_get_speed(1));
    }
    
    switch(loop_count)
    {
        case 0:
            pwm_set_duty(0, 0); // Perform step test actions for loop_count 0
            printf("Setting pwm to 0\n");
            break;
        case 100:
            pwm_set_duty(0, .6); // Perform step test actions for loop_count 100
            printf("Setting pwm to 0.6\n");
            break;
        case 100+STEP_TEST_LENGTH:
            pwm_set_duty(0, 0); // Perform step test actions for loop_count 100
            printf("Setting pwm to 0\n");
            break;
        case 100+2*STEP_TEST_LENGTH:
            step_test_active = false;
            break;
        // Add more cases as needed
        default:
            // Perform step test actions for other loop_count values
            break;
    }
    return 1;
}

void step_test_cleanup()
{
    if (step_test_data != NULL)
    {
        fflush(step_test_data);
        fclose(step_test_data);
        step_test_data = NULL;
    }
}