import argparse
from pathlib import Path

import matplotlib.pyplot as plt
import numpy as np
import pandas as pd


# -----------------------------------------------------------------------------
# Command-line arguments
# -----------------------------------------------------------------------------

parser = argparse.ArgumentParser()

parser.add_argument("baseline")
parser.add_argument("moving")
parser.add_argument("--output", default="lab6_results")

args = parser.parse_args()

output_dir = Path(args.output)
output_dir.mkdir(exist_ok=True)


# -----------------------------------------------------------------------------
# Plot setup
# -----------------------------------------------------------------------------

summary = []

fig, axes = plt.subplots(
    2,
    2,
    figsize=(13, 7),
    sharex="col",
)

test_runs = [
    ("Supported baseline", args.baseline),
    ("Moving test", args.moving),
]


# -----------------------------------------------------------------------------
# Process each test run
# -----------------------------------------------------------------------------

for column, (name, path) in enumerate(test_runs):
    data = pd.read_csv(path)

    if data.empty:
        raise ValueError(f"{path}: no samples")

    time_seconds = data["time_ms"] / 1000

    # Plot target speed and measured encoder speed.
    speed_axis = axes[0, column]

    speed_axis.plot(
        time_seconds,
        data["target_speed"],
        label="Target",
    )

    speed_axis.plot(
        time_seconds,
        data["measured_speed"],
        label="Single encoder measurement",
    )

    speed_axis.set(
        title=name,
        ylabel="Speed (Lab 4 units)",
    )

    # Plot the final left and right PWM commands.
    pwm_axis = axes[1, column]

    pwm_axis.plot(
        time_seconds,
        data["left_pwm"],
        label="Left channel command",
    )

    pwm_axis.plot(
        time_seconds,
        data["right_pwm"],
        label="Right channel command",
    )

    pwm_axis.set(
        xlabel="Time (s)",
        ylabel="Signed PWM duty",
        ylim=(-0.65, 0.65),
    )

    # Calculate summary metrics for each speed mode.
    for mode in sorted(data["mode"].unique()):
        mode_data = data[data["mode"] == mode]

        error = (
            mode_data["target_speed"]
            - mode_data["measured_speed"]
        )

        summary.append(
            {
                "run": name,
                "mode": mode,
                "samples": len(mode_data),
                "mae": error.abs().mean(),
                "rmse": np.sqrt((error**2).mean()),
                "measurement_std": mode_data["measured_speed"].std(),
            }
        )

    # Display sample-spacing statistics.
    spacing = data["time_ms"].diff().dropna()

    print(
        name,
        "sample spacing ms min/mean/max:",
        spacing.min(),
        spacing.mean(),
        spacing.max(),
    )


# -----------------------------------------------------------------------------
# Format and save the comparison plot
# -----------------------------------------------------------------------------

for axis in axes.flat:
    axis.grid(True)
    axis.legend()

fig.tight_layout()

fig.savefig(
    output_dir / "lab6_comparison.png",
    dpi=180,
)

plt.close(fig)


# -----------------------------------------------------------------------------
# Save summary metrics
# -----------------------------------------------------------------------------

pd.DataFrame(summary).to_csv(
    output_dir / "metrics.csv",
    index=False,
)
