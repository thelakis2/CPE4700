#ifndef ENCODER_H
#define ENCODER_H

#include <stdint.h>

// Configuration structure for an encoder
typedef struct {
    int gpio_a;
    int gpio_b;
    int target_cpu; // Core to pin the thread to (e.g., 1, 2, 3)
    int priority;   // Real-time priority (1 to 99)
} encoder_config_t;

// Initialize the encoder module with configurations for both encoders
int encoder_init();

// Getter methods for speed (pulses per second) and direction (1 = forward, -1 = reverse, 0 = stopped)
double encoder_get_speed(int encoder_id);
long encoder_get_count(int encoder_id);
double encoder_get_direction(int encoder_id);

// Cleanup resources
void encoder_cleanup(void);

#endif // ENCODER_H
