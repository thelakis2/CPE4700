#define _GNU_SOURCE
#include "pwm.h"

#include <errno.h>
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
/*oiulkjased*/
/* ---------- PWM DEFINITIONS ---------- */
// Confirmed PWM channels
// pin 15, pwmchip0 pwm0
// pin 32, pwmchip3 pwm0
// pin 33, pwmchip2 pwm0
#define PWM_CHIP_PATH "/sys/class/pwm/pwmchip2"
#define PWM_CHANNEL "0"
#define PWM_PATH PWM_CHIP_PATH "/pwm" PWM_CHANNEL

#define PWM_CHIP_PATH2 "/sys/class/pwm/pwmchip3"
#define PWM_PATH2 PWM_CHIP_PATH2 "/pwm" PWM_CHANNEL

#define PWM_PERIOD_NS 200000 // 10 kHz

/* ---------- GPIO (Direction: PY.04) ---------- */
#define GPIO_CHIP "/dev/gpiochip0"
// Confirmed line numbers
// pin 16, PY.04, line 126
// pin 18, PY.03, line 125
// pin 31, PQ.06, line 106
// pin 36, PR.04, line 113
#define DIR_LINE_OFFSET 106
#define DIR_LINE_OFFSET2 113

static struct gpiod_chip *chip;
static struct gpiod_line *dir_line[2];

#define PWM_MOTORS 2

static const char *chip_path = PWM_CHIP_PATH;
static int channels[PWM_MOTORS] = {0, 0};
static const char *direction_paths[PWM_MOTORS] = {NULL, NULL};
static int initialized = 0;

static int write_value(const char *path, const char *value)
{
    FILE *file = fopen(path, "w");
    if (!file)
        return -errno;
    int result = fputs(value, file) < 0 ? -errno : 0;
    if (result == 0 && fclose(file) != 0)
        return -errno;
    return result;
}

static int pwm_path(char *path, size_t size, int motor, const char *name)
{
    int written = snprintf(path, size, "%s/pwm%d/%s", chip_path,
                           channels[motor], name);
    return written < 0 || (size_t)written >= size ? -ENAMETOOLONG : 0;
}

int pwm_init(void)
{
    const char *configured_chip = getenv("PWM_CHIP");
    if (configured_chip && *configured_chip)
        chip_path = configured_chip;

    const char *channel0 = getenv("PWM_CHANNEL_0");
    const char *channel1 = getenv("PWM_CHANNEL_1");
    direction_paths[0] = getenv("PWM_DIRECTION_0");
    direction_paths[1] = getenv("PWM_DIRECTION_1");
    if (channel0)
        channels[0] = atoi(channel0);
    if (channel1)
        channels[1] = atoi(channel1);

    for (int motor = 0; motor < PWM_MOTORS; motor++) {
        char path[256];
        char channel[16];
        snprintf(channel, sizeof(channel), "%d", channels[motor]);
        if (pwm_path(path, sizeof(path), motor, "period") < 0)
            return -ENAMETOOLONG;
        if (access(path, F_OK) != 0) {
            char export_path[256];
            snprintf(export_path, sizeof(export_path), "%s/export", chip_path);
            write_value(export_path, channel);
        }
        if (pwm_path(path, sizeof(path), motor, "period") < 0)
            return -ENAMETOOLONG;
        int result = write_value(path, "1000000");
        if (result < 0)
            return result;
        if (pwm_path(path, sizeof(path), motor, "enable") < 0)
            return -ENAMETOOLONG;
        result = write_value(path, "1");
        if (result < 0)
            return result;
    }
    initialized = 1;
    return 0;
}

int pwm_set_duty(int motor, double duty)
{
    if (!initialized || motor < 0 || motor >= PWM_MOTORS)
        return -EINVAL;
    if (duty > 1.0)
        duty = 1.0;
    if (duty < -1.0)
        duty = -1.0;

    if (direction_paths[motor] && *direction_paths[motor]) {
        int direction = duty < 0.0;
        int result = write_value(direction_paths[motor], direction ? "1" : "0");
        if (result < 0)
            return result;
    }

    char path[256];
    if (pwm_path(path, sizeof(path), motor, "duty_cycle") < 0)
        return -ENAMETOOLONG;
    char duty_cycle[32];
    snprintf(duty_cycle, sizeof(duty_cycle), "%ld", (long)(fabs(duty) * 1000000.0));
    return write_value(path, duty_cycle);
}

void pwm_cleanup(void)
{
    for (int motor = 0; motor < PWM_MOTORS; motor++) {
        char path[256];
        if (pwm_path(path, sizeof(path), motor, "enable") == 0)
            write_value(path, "0");
    }
    initialized = 0;
}