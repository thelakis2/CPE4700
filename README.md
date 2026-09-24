# Lab 04 motor characterization

Build from this directory:

```sh
make
```

Run `./lab_04/lab04 step` for `step_response.csv`, or
`./lab_04/lab04 speed` for `speed_control.csv`. Use `both` to run both tests
in sequence. Stop with Ctrl-C; the motor is commanded to zero during cleanup.

The default Linux PWM and GPIO paths are placeholders. Set these variables for
the Jetson wiring before running:

```sh
export PWM_CHIP=/sys/class/pwm/pwmchip0
export PWM_CHANNEL_0=0 PWM_CHANNEL_1=1
export PWM_DIRECTION_0=/sys/class/gpio/gpioDIR0/value
export PWM_DIRECTION_1=/sys/class/gpio/gpioDIR1/value
export ENCODER_A0=/sys/class/gpio/gpioA0/value
export ENCODER_B0=/sys/class/gpio/gpioB0/value
```

Set the corresponding `*_A1`, `*_B1`, and direction variables when using the
second motor. `PWM_DIRECTION_*` is required for signed forward/reverse output;
the PWM interface itself receives the absolute duty-cycle magnitude.