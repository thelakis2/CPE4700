/*******************************************************************
 * Gerry Nagel
 * 2026/08/19
********************************************************************/ 
#define _GNU_SOURCE
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <pthread.h>
#include <sched.h>
#include <stdatomic.h>
#include <gpiod.h>
#include <time.h>
#include <bits/time.h>
#include <stdint.h>

// Jetson Orin Nano 40-Pin Header Configuration
// Replace with the exact chip name and offsets for your physical pins
#define ENCODER_CHIP "gpiochip0"
#define PIN_A_OFFSET 124
#define PIN_B_OFFSET 53
#define PIN_A2_OFFSET 52
#define PIN_B2_OFFSET 51

typedef _Atomic(double) atomic_double;

// Thread-safe atomic variables
static atomic_long encoder_count[2] = {0};
static atomic_double encoder_speed[2] = {0};
extern atomic_bool running;

pthread_t thread_id[2] = {0};

// Low-level C structural handles for libgpiod
struct gpiod_chip *chip = 0;
struct gpiod_line *line_a[2] = {0};
struct gpiod_line *line_b[2] = {0};

#define SPEED_SAMPLES 2
void *encoder_worker(void *arg);
static double speeds[SPEED_SAMPLES][2] = {0};
static int speed_idx[2] = {0};

int encoder_init_inst(int encoder_id, int line_a_offset, int line_b_offset);

int encoder_init()
{
    // Initialize both encoders
    if (encoder_init_inst(0, PIN_A_OFFSET, PIN_B_OFFSET) < 0)
    {
        printf("[ERROR] Failed to initialize encoder 0\n");
        return -1;
    }
    if (encoder_init_inst(1, PIN_A2_OFFSET, PIN_B2_OFFSET) < 0)
    {
        printf("[ERROR] Failed to initialize encoder 1\n");
        return -1;  
    }
    // usleep(50000); // Give the kernel 50ms to populate /proc/interrupts

    // // Explicit commands targeting each named encoder precisely
    // // Hex 10 = Core 4 | Hex 20 = Core 5
    // const char *route_irqs_cmd = 
    //     "IRQ_A0=$(cat /proc/interrupts | grep 'Encoder_A0' | awk -F: '{print $1}' | tr -d ' '); "
    //     "IRQ_A1=$(cat /proc/interrupts | grep 'Encoder_A1' | awk -F: '{print $1}' | tr -d ' '); "
        
    //     "if [ -n \"$IRQ_A0\" ] && [ -n \"$IRQ_A1\" ]; then "
    //     "  echo 10 | sudo tee /proc/irq/$IRQ_A0/smp_affinity > /dev/null; "
    //     "  echo 20 | sudo tee /proc/irq/$IRQ_A1/smp_affinity > /dev/null; "
    //     "  echo \"[IRQ Setup] Success: Routed Encoder_A0 (IRQ $IRQ_A0) to Core 4\"; "
    //     "  echo \"[IRQ Setup] Success: Routed Encoder_A1 (IRQ $IRQ_A1) to Core 5\"; "
    //     "else "
    //     "  echo \"[IRQ Setup] Error: Could not locate unique Encoder_A0 or Encoder_A1 symbols.\"; "
    //     "fi";

    // if(system(route_irqs_cmd) != 0)
    // {
    //     printf("[ERROR] Failed to route IRQs\n");
    // }

    return 0;
}   

//========================================OLD CODE========================================
// Explicit C initialization function
int encoder_init_inst(int encoder_id, int line_a_offset, int line_b_offset)
{
    // 1. Open the GPIO chip
    chip = gpiod_chip_open_by_name(ENCODER_CHIP);
    if (!chip)
    {
        fprintf(stderr, "[ERROR] Failed to open chip: %s\n", ENCODER_CHIP);
        return -1;
    }

    // 2. Fetch line handles
    line_a[encoder_id] = gpiod_chip_get_line(chip, line_a_offset);
    line_b[encoder_id] = gpiod_chip_get_line(chip, line_b_offset);
    if (!line_a[encoder_id] || !line_b[encoder_id])
    {
        fprintf(stderr, "[ERROR] Failed to fetch hardware lines\n");
        gpiod_chip_close(chip);
        return -1;
    }

   // 3. Request Pin A to monitor BOTH rising and falling edges
    if (gpiod_line_request_both_edges_events(line_a[encoder_id], encoder_id==0 ? "Encoder_A0" : "Encoder_A1") < 0)
    {
        fprintf(stderr, "[ERROR] Failed to request events on Pin A\n");
        gpiod_chip_close(chip);
        return -1;
    }

    // 4. Request Pin B as a high-speed digital state input
    if (gpiod_line_request_input(line_b[encoder_id], "Encoder_B") < 0)
    //if (gpiod_line_request_both_edges_events(line_b, "Encoder_B") < 0)
    {
        fprintf(stderr, "[ERROR] Failed to request input on Pin B\n");
        gpiod_line_release(line_a[encoder_id]);
        gpiod_chip_close(chip);
        return -1;
    }
    // Spawn the background POSIX thread
    if (pthread_create(&thread_id[encoder_id], NULL, encoder_worker, (void *) (long) encoder_id) != 0)
    {
        fprintf(stderr, "[ERROR] Failed to create background thread\n");
        return EXIT_FAILURE;
    }
    printf("[SUCCESS] Encoder C interface initialized successfully\n");
    return 0;
}

// Background POSIX thread worker
void *encoder_worker(void *arg)
{
    int encoder_id = (int)(long)arg;

    cpu_set_t cpuset;
    CPU_ZERO(&cpuset);
    CPU_SET(encoder_id+4, &cpuset); // Target Core 4,5

    pthread_t current_thread = pthread_self();
    if (pthread_setaffinity_np(current_thread, sizeof(cpu_set_t), &cpuset) != 0) {
        printf("Error setting affinity for Thread %d\n", encoder_id+4);
    }

    struct sched_param param;
    int policy = SCHED_FIFO;

    // Query the maximum valid priority for FIFO scheduling
    param.sched_priority = sched_get_priority_max(policy);

    // Apply the real-time policy and priority to the current thread
    int result = pthread_setschedparam(pthread_self(), policy, &param);
    if (result != 0)
    {
        fprintf(stderr, "Failed to set RT priority: %s\n", strerror(result));
        // Handle error: thread will fallback to normal priority
    }
    else
    {
        printf("Thread is now running at real-time priority!\n");
    }

    struct gpiod_line_event event;
    struct timespec timeout = {0, 100000000}; // 100ms timeout struct

    long last_time = 0;

    while (atomic_load(&running))
    {
        // Poll for a hardware edge change on Pin A
        int event_status = gpiod_line_event_wait(line_a[encoder_id], &timeout);

        if (event_status > 0)
        {
            if (gpiod_line_event_read(line_a[encoder_id], &event) < 0)
                continue;
            // Read the edge type (Rising vs Falling)
            long now = event.ts.tv_nsec;
            long delta_ns = now - last_time;
            if(delta_ns < 0)
            {
                delta_ns += 1000000000l; // Handle wrap-around
            }   
            if(delta_ns > 100000) // 50us software debounce
            {
                last_time = now;

                int state_a = (event.event_type == GPIOD_LINE_EVENT_RISING_EDGE) ? 1 : 0;
                int state_b = gpiod_line_get_value(line_b[encoder_id]);

                bool direction = (state_a == state_b); // true = forward, false = reverse
                if (direction)
                {
                    atomic_fetch_add(&encoder_count[encoder_id], 1);
                }
                else
                {
                    delta_ns = -delta_ns; // Reverse direction, negative delta
                    atomic_fetch_sub(&encoder_count[encoder_id], 1);
                    //printf("Encoder #1: Reverse direction detected!\n");
                }
                speeds[((speed_idx[encoder_id]++) % SPEED_SAMPLES)] [encoder_id] = 68220200.0 / 4.0 / delta_ns;
                double avg_speed = 0;
                for (int i = 0; i < SPEED_SAMPLES; i++)
                {
                    avg_speed += speeds[i][encoder_id]  ;
                }
                atomic_store(&encoder_speed[encoder_id], avg_speed / SPEED_SAMPLES);
                //printf("Encoder time: %ld. Count = %ld\n", delta_us, atomic_load(&encoder_count));
                //usleep(200); // Yield to allow other threads to run
            }  
        }
        else if (event_status == 0)
        {
            // for (int i = 0; i < SPEED_SAMPLES; i++)
            // {
            //     speeds[i][encoder_id] = 0;
            // }
            // atomic_store(&encoder_speed[encoder_id], 0);
            //printf("Encoder %d gpiod_line_event_wait timeout.\n", encoder_id);
        }
        else
        {
            printf("Encoder %d gpiod_line_event_wait failed.\n", encoder_id);
            break;
        }
    }

    // Resource Cleanup upon thread teardown
    gpiod_line_release(line_a[encoder_id]);
    gpiod_line_release(line_b[encoder_id]);
    return NULL;
}

double encoder_get_speed(int encoder_id)
{
    if (encoder_id != 0 && encoder_id != 1)
    {
        fprintf(stderr, "Invalid encoder ID: %d\n", encoder_id);
        return -1;
    }   
    // Return the current speed in pulses per second
    return (double)atomic_load(&encoder_speed[encoder_id]);
}

long encoder_get_count(int encoder_id)
{
    if (encoder_id != 0 && encoder_id != 1)
    {
        fprintf(stderr, "Invalid encoder ID: %d\n", encoder_id);
        return -1;
    }
    return (long)atomic_load(&encoder_count[encoder_id]);
}

int encoder_cleanup()
{
    pthread_join(thread_id[0], NULL);
    pthread_join(thread_id[1], NULL);
    gpiod_chip_close(chip);

    return EXIT_SUCCESS;
}
