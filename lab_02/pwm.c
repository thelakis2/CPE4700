#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <fcntl.h>
#include <string.h>
#include <gpiod.h>
#include "pwm.h"

#define PWM_CHIP2 2
#define PWM_CHIP3 3
#define PWM0 0

#define GPIO_CHIP "/dev/gpiochip0"
#define DIR_LINE_OFFSET 106
#define DIR_LINE_OFFSET2 113

#define PWM2_PATH "/sys/class/pwm/pwmchip2/pwm0"
#define PWM3_PATH "/sys/class/pwm/pwmchip3/pwm0"
#define PWM2_EXPORT "/sys/class/pwm/pwmchip2/export"
#define PWM3_EXPORT "/sys/class/pwm/pwmchip3/export"
#define PWM2_UNEXPORT "/sys/class/pwm/pwmchip2/unexport"
#define PWM3_UNEXPORT "/sys/class/pwm/pwmchip3/unexport"
#define PWM_PERIOD_NS 1000000

static struct gpiod_chip *chip;
static struct gpiod_line *dir_line[2];

static int write_sysfs(const char *path, const char *value) {
    int fd = open(path, O_WRONLY);
    if (fd < 0) {
        return -1;
    }

    if (write(fd, value, strlen(value)) < 0) {
        close(fd);
        return -1;
    }

    close(fd);
    return 0;
}

int pwm_init(void) {
    if (access(PWM2_PATH, F_OK) != 0) {
        write_sysfs(PWM2_EXPORT, "0");
        usleep(100000);
    }

    if (access(PWM3_PATH, F_OK) != 0) {
        write_sysfs(PWM3_EXPORT, "0");
        usleep(100000);
    }

    write_sysfs(PWM2_PATH "/period", "1000000");
    write_sysfs(PWM3_PATH "/period", "1000000");

    write_sysfs(PWM2_PATH "/duty_cycle", "0");
    write_sysfs(PWM3_PATH "/duty_cycle", "0");

    write_sysfs(PWM2_PATH "/enable", "1");
    write_sysfs(PWM3_PATH "/enable", "1");

    chip = gpiod_chip_open(GPIO_CHIP);
    if (!chip) {
        return -1;
    }

    dir_line[0] = gpiod_chip_get_line(chip, DIR_LINE_OFFSET);
    dir_line[1] = gpiod_chip_get_line(chip, DIR_LINE_OFFSET2);

    if (!dir_line[0] || !dir_line[1]) {
        return -1;
    }

    if (gpiod_line_request_output(dir_line[0], "motor_dir", 0) < 0)
        return -1;

    if (gpiod_line_request_output(dir_line[1], "motor_dir", 0) < 0)
        return -1;

    return 0;
}

int pwm_set_duty(int channel, float duty_cycle) {
    char duty_cycle_str[20];
    const char *pwm_path;
    char path[128];

    if (duty_cycle >= 0.0f)
        gpiod_line_set_value(dir_line[channel], 1);
    else {
        gpiod_line_set_value(dir_line[channel], 0);
        duty_cycle = -duty_cycle;
    }

    if (duty_cycle > 1.0f)
        duty_cycle = 1.0f;

    pwm_path = (channel == 0) ? PWM2_PATH : PWM3_PATH;

    snprintf(duty_cycle_str,
             sizeof(duty_cycle_str),
             "%d",
             (int)(duty_cycle * PWM_PERIOD_NS));

    snprintf(path, sizeof(path), "%s/duty_cycle", pwm_path);
    write_sysfs(path, duty_cycle_str);

    return 0;
}

int pwm_cleanup(void) {
    write_sysfs(PWM2_PATH "/enable", "0");
    write_sysfs(PWM3_PATH "/enable", "0");

    write_sysfs(PWM2_UNEXPORT, "0");
    write_sysfs(PWM3_UNEXPORT, "0");

    if (dir_line[0])
        gpiod_line_release(dir_line[0]);

    if (dir_line[1])
        gpiod_line_release(dir_line[1]);

    if (chip)
        gpiod_chip_close(chip);

    return 0;
}