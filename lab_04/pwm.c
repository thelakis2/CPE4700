#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <fcntl.h>
#include <string.h>
#include <gpiod.h>

#include "pwm.h"

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

/* ---------- Helper: write to sysfs ---------- */
void write_sysfs(const char *path, const char *value)
{
    int fd = open(path, O_WRONLY);
    if (fd < 0)
    {
        perror(path);
        exit(EXIT_FAILURE);
    }
    if (write(fd, value, strlen(value)) < 0)
    {
        printf("Failed to write %s to %s\n", value,path);
        perror(path);
        close(fd);
        exit(EXIT_FAILURE);
    }
    close(fd);
}

/* ---------- GPIO SETUP (Direction) ---------- */
int pwm_init()
{
    chip = gpiod_chip_open(GPIO_CHIP);
    if (!chip)
    {
        perror("gpiod_chip_open");
        return 1;
    }

    dir_line[0] = gpiod_chip_get_line(chip, DIR_LINE_OFFSET);
    if (!dir_line[0])
    {
        perror("gpiod_chip_get_line");
        return 1;
    }

    dir_line[1] = gpiod_chip_get_line(chip, DIR_LINE_OFFSET2);
    if (!dir_line[1])
    {
        perror("gpiod_chip_get_line2");
        return 1;
    }

    if (gpiod_line_request_output(dir_line[0], "mdd10-dir", 1) < 0)
    {
        perror("gpiod_line_request_output");
        return 1;
    }

    if (gpiod_line_request_output(dir_line[1], "mdd10-dir2", 1) < 0)
    {
        perror("gpiod_line_request_output");
        return 1;
    }

    /* ---------- PWM SETUP ---------- */
    if (access(PWM_PATH, F_OK) != 0)
    {
        write_sysfs(PWM_CHIP_PATH "/export", PWM_CHANNEL);
        usleep(100000);
    }

    if (access(PWM_PATH2, F_OK) != 0)
    {
        write_sysfs(PWM_CHIP_PATH2 "/export", PWM_CHANNEL);
        usleep(100000);
    }

    write_sysfs(PWM_PATH "/period", "200000"); // 10 kHz   
    write_sysfs(PWM_PATH2 "/period", "200000");
    write_sysfs(PWM_PATH "/duty_cycle", "0");
    write_sysfs(PWM_PATH2 "/duty_cycle", "0");
    write_sysfs(PWM_PATH "/enable", "1");
    write_sysfs(PWM_PATH2 "/enable", "1");

    return 0;
}
// Setter method for PWM duty cycle (0.0 to 1.0)
int pwm_set_duty(int channel, float duty_cycle)
{
    bool sat = false;
    if (channel != 0 && channel != 1)
    {
        fprintf(stderr, "Invalid PWM channel: %d\n", channel);
        return sat;
    }
    if (duty_cycle < -.8)
    {
        duty_cycle = -.8;
        sat = true;
    }
    else if (duty_cycle > .8)
    {
        duty_cycle = .8;
        sat = true;
    }

    if (channel != 0 && channel != 1)
    {
        fprintf(stderr, "Invalid PWM channel: %d\n", channel);
        return sat;
    }
    if (duty_cycle > 0)
    {
        gpiod_line_set_value(dir_line[channel], 1);
    }
    else if (duty_cycle < 0)
    {
        gpiod_line_set_value(dir_line[channel], 0);
        duty_cycle = -duty_cycle;
    }
        // Set duty cycle for channel 0
    char duty_cycle_str[20];
    snprintf(duty_cycle_str, sizeof(duty_cycle_str), "%d", (int)(duty_cycle * PWM_PERIOD_NS));
//    printf("Setting PWM channel %d duty cycle to %s ns\n", channel, duty_cycle_str);
    if (channel == 0)
    {
        write_sysfs(PWM_PATH "/duty_cycle", duty_cycle_str);
    }
    else if (channel == 1)
    {
        write_sysfs(PWM_PATH2 "/duty_cycle", duty_cycle_str);
    }
    return sat;
}

void pwm_cleanup(void)
{
    /* ---------- CLEANUP ---------- */
    printf("Motor stop\n");
    write_sysfs(PWM_PATH "/duty_cycle", "0");
    write_sysfs(PWM_PATH2 "/duty_cycle", "0");
    sleep(1);

    /* ---------- CLEANUP ---------- */
    write_sysfs(PWM_PATH "/enable", "0");
    write_sysfs(PWM_PATH2 "/enable", "0");
    gpiod_line_release(dir_line[0]);
    gpiod_line_release(dir_line[1]);
    gpiod_chip_close(chip);
}