# Lab 05: PI balance controller
Complete source package. Extract lab_05 into ~/dev, then make clean && make.
Install dependencies if needed: sudo apt install libgtk-4-dev libgpiod-dev build-essential pkg-config
Run from the Jetson graphical desktop: sudo ./robotest

Motor output is disabled initially. Lift the wheels and verify pitch zero and wheel direction before enabling. MOTOR0_SIGN and MOTOR1_SIGN default to +1 in balance_loop.c; change for your physical mounting/wiring.
PI-only: Kd is ignored and GUI Kd controls are disabled. Defaults Kp=0.01, Ki=0 are untested placeholders. Set Ki nonzero for PI. Disable output before changing gains; gain changes reset the integral.
Output is clamped to +/-0.75. Nonfinite pitch, tilt beyond +/-30 degrees, or controller gaps >20 ms disable output. These checks are not guaranteed safety protection. Keep a physical power disconnect available.
RPM uses count changes over 50 ms and assumes 700 actual decoded counts per wheel revolution. Verify that convention; the encoder counts both Channel A edges. The supplied encoder_get_speed function is NOT repaired or used for RPM display. No encoder damping is included.
Logging: balance_log.csv is buffered in RAM and written after shutdown. 600,000 sample limit (~10 min), then application stops. Each run overwrites the log; rename between experiments.
IMU read failures return NAN. encoder_cleanup return type matches the header. Original hardware pins and partial-initialization limitations remain.
FIFO scheduling falls back with a warning. One millisecond is the requested period, not a verified guarantee. GTK4 compilation and hardware behavior must be tested on Jetson.
Remaining lab work: collect several gain settings, plot pitch and RPM, discuss P/PI/PID (this implementation only runs P/PI), identify near-stable ranges, write report, commit and push. No measured balance results are fabricated.
