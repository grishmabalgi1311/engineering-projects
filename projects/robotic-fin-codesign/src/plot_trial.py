"""
Plot a Fish Force Testing System trial CSV: Force and Actual Position, both on
ONE figure, sharing the actually-recorded Time (s) column as the x-axis (no
padding back to zero -- the axis starts and ends exactly where your recording
did). Important moments (CLAMP/UNCLAMP/TARE/DXL_START/DXL_STOP etc.) are
marked as vertical red lines with labels, so you can immediately see when
each thing happened.

Force (g) and Position (deg) are very different scales, so Force gets its own
axis on the right; Actual Position uses the left axis. Both share the same
Time (s) x-axis, so you can see exactly where a force change lines up with
the fin's position.

Usage:
    python plot_trial.py Data/Trial_20260709_152610.csv
    python plot_trial.py                                     (uses CSV_PATH below)
    python plot_trial.py Data/some_trial.csv --window 10 3    (zoom into 3s starting 10s
                                                                 into the recording)
    python plot_trial.py Data/some_trial.csv --envelope       (whole-trial force envelope,
                                                                 default 1s buckets)
    python plot_trial.py Data/some_trial.csv --envelope 2     (2s buckets instead)

Output:
    Saves a PNG next to the CSV (same name + "_plot.png"/"_window_..._plot.png"/
    "_envelope_plot.png") and opens it in a window.

Install (same as the rest of the project):
    pip install pandas matplotlib numpy

CHANGE LOG:
  - Added plot_trial_window(): zoom into a short slice of one trial (relative
    to the recording's own start, not absolute Time (s) values) so individual
    fin cycles are visible instead of thousands of them crammed into one
    image -- the full-trial plot_trial() plot is unreadable at high sway
    speeds/frequencies over a multi-minute recording.
  - Added plot_trial_envelope(): instead of the raw oscillating Force signal,
    plots the rolling peak |Force| and rolling RMS Force in fixed-size time
    buckets across the WHOLE trial, so a change in the force's amplitude
    partway through a trial (drift, a resonance, the clamp shifting, etc.)
    is immediately visible as a trend line, instead of being hidden under a
    solid block of overlapping oscillations.
  - plot_trial() itself is unchanged.
"""

import sys
import os
import pandas as pd
import numpy as np
import matplotlib.pyplot as plt

CSV_PATH = "Data/Trial_20260709_152610.csv"  # used if no path is given on the command line


def plot_trial(csv_path, show=True):
    """Plot one trial CSV. Returns the matplotlib Figure.

    show=False lets other scripts (e.g. plot_all_trials.py) call this in a
    loop over many files without popping up and blocking on a window for
    each one -- it still saves the PNG either way, just doesn't display it.
    """

    df = pd.read_csv(csv_path)

    time = df["Time (s)"]

    fig, ax_force = plt.subplots(figsize=(11, 6))

    # ---- Force on the left/primary axis ----
    force_color = "tab:blue"
    ax_force.plot(
        time, df["Force (g)"],
        color=force_color, linewidth=1, label="Force (g)"
    )
    ax_force.set_xlabel("Time (s)")
    ax_force.set_ylabel("Force (g)", color=force_color)
    ax_force.tick_params(axis="y", labelcolor=force_color)

    # Only the time range that was actually recorded -- Scatter/line plots in
    # Excel (and matplotlib) auto-scale to your data, but we set this
    # explicitly so there's never any padding back to t=0.
    ax_force.set_xlim(time.min(), time.max())

    # ---- Actual position on the right/secondary axis ----
    # Older CSVs recorded before the position-logging update won't have this
    # column -- skip it gracefully instead of erroring out.
    has_position_data = "Actual Position (deg)" in df.columns

    if has_position_data:

        ax_pos = ax_force.twinx()

        ax_pos.plot(
            time, df["Actual Position (deg)"],
            color="tab:green", linewidth=1.5,
            label="Actual Position (deg)"
        )

        ax_pos.set_ylabel("Actual Position (deg)", color="tab:green")
        ax_pos.tick_params(axis="y", labelcolor="tab:green")

    # ---- Mark the important timeline events (CLAMP/UNCLAMP/TARE/DXL_START/
    # DXL_STOP/...), if any -- these are the moments that matter, so make
    # them stand out clearly against the data. ----
    if "Event" in df.columns:

        events = df.dropna(subset=["Event"])
        events = events[events["Event"].astype(str).str.strip() != ""]

        ymin, ymax = ax_force.get_ylim()

        for _, row in events.iterrows():
            ax_force.axvline(row["Time (s)"], color="red", linestyle="--", linewidth=1.2, alpha=0.7)
            ax_force.text(
                row["Time (s)"], ymax, str(row["Event"]),
                rotation=90, va="top", ha="right", fontsize=8, color="red", fontweight="bold"
            )

    # ---- Combined legend from both axes ----
    lines, labels = ax_force.get_legend_handles_labels()
    if has_position_data:
        lines_pos, labels_pos = ax_pos.get_legend_handles_labels()
        lines += lines_pos
        labels += labels_pos
    ax_force.legend(lines, labels, loc="upper right")

    ax_force.set_title(f"Force & Actual Fin Position vs Time -- {os.path.basename(csv_path)}")
    ax_force.grid(True, alpha=0.3)

    fig.tight_layout()

    out_path = csv_path.rsplit(".", 1)[0] + "_plot.png"
    fig.savefig(out_path, dpi=150)
    print(f"Saved plot to {out_path}")

    if show:
        plt.show()
    else:
        plt.close(fig)

    return fig


def plot_trial_window(csv_path, start_s, duration_s, show=True):
    """Zoom into a short slice of one trial, so individual fin cycles are
    visible instead of a dense, unreadable block of overlapping oscillations
    (which is what plot_trial() produces for a multi-minute recording at a
    fast sway speed/frequency).

    start_s / duration_s are relative to the RECORDING'S OWN START (i.e.
    relative to this file's Time (s).min()), not absolute Time (s) values --
    so start_s=10 always means "10 seconds into this particular recording",
    regardless of what the raw Time (s) column happens to start at.

    Use this after spotting something interesting in plot_trial() or
    plot_trial_envelope() (e.g. "the force amplitude changed around the
    170s mark") to zoom into exactly that section.

    Saves to '<name>_window_<start_s>s_<duration_s>s_plot.png'.
    """

    df = pd.read_csv(csv_path)
    time = df["Time (s)"]

    t0 = time.min() + start_s
    t1 = t0 + duration_s

    window_df = df[(df["Time (s)"] >= t0) & (df["Time (s)"] <= t1)]

    if window_df.empty:
        raise ValueError(
            f"No data in the requested window (start_s={start_s}, duration_s={duration_s}). "
            f"This trial's recording covers 0 to {time.max() - time.min():.1f}s "
            f"relative to its own start -- pick a start_s within that range."
        )

    fig, ax_force = plt.subplots(figsize=(11, 6))

    force_color = "tab:blue"
    ax_force.plot(
        window_df["Time (s)"], window_df["Force (g)"],
        color=force_color, linewidth=1.3, marker="o", markersize=2.5,
        label="Force (g)"
    )
    ax_force.set_xlabel("Time (s)")
    ax_force.set_ylabel("Force (g)", color=force_color)
    ax_force.tick_params(axis="y", labelcolor=force_color)
    ax_force.set_xlim(window_df["Time (s)"].min(), window_df["Time (s)"].max())

    has_position_data = "Actual Position (deg)" in df.columns

    if has_position_data:

        ax_pos = ax_force.twinx()

        ax_pos.plot(
            window_df["Time (s)"], window_df["Actual Position (deg)"],
            color="tab:green", linewidth=1.5, marker="o", markersize=2.5,
            label="Actual Position (deg)"
        )

        ax_pos.set_ylabel("Actual Position (deg)", color="tab:green")
        ax_pos.tick_params(axis="y", labelcolor="tab:green")

    lines, labels = ax_force.get_legend_handles_labels()
    if has_position_data:
        lines_pos, labels_pos = ax_pos.get_legend_handles_labels()
        lines += lines_pos
        labels += labels_pos
    ax_force.legend(lines, labels, loc="upper right")

    ax_force.set_title(
        f"Zoomed {start_s:g}s-{start_s + duration_s:g}s into recording -- {os.path.basename(csv_path)}"
    )
    ax_force.grid(True, alpha=0.3)

    fig.tight_layout()

    out_path = csv_path.rsplit(".", 1)[0] + f"_window_{start_s:g}s_{duration_s:g}s_plot.png"
    fig.savefig(out_path, dpi=150)
    print(f"Saved windowed plot to {out_path}")

    if show:
        plt.show()
    else:
        plt.close(fig)

    return fig


def plot_trial_envelope(csv_path, window_s=1.0, show=True):
    """Plot the FULL trial's force envelope instead of the raw oscillating
    signal: rolling peak |Force| and rolling RMS Force, computed in
    window_s-second buckets across the whole recording.

    At high sway speeds/frequencies, plot_trial()'s raw signal is too dense
    to read over a multi-minute trial (thousands of overlapping cycles
    render as a solid block). This shows how the force's amplitude trends
    or changes over the course of the trial instead -- e.g. it drifting
    upward, or suddenly growing partway through -- which the raw plot hides
    completely.

    window_s : bucket size in seconds. Smaller = more temporal detail but a
        noisier-looking envelope; larger = smoother but coarser.

    Saves to '<name>_envelope_plot.png'.
    """

    df = pd.read_csv(csv_path)
    time = df["Time (s)"] - df["Time (s)"].min()
    force = df["Force (g)"]

    bucket = (time // window_s).astype(int)

    grouped = pd.DataFrame({"bucket": bucket, "time": time, "force": force}).groupby("bucket")

    bucket_time = grouped["time"].mean()
    bucket_peak = grouped["force"].apply(lambda f: f.abs().max())
    bucket_rms = grouped["force"].apply(lambda f: np.sqrt((f ** 2).mean()))

    fig, ax = plt.subplots(figsize=(11, 6))

    ax.plot(
        bucket_time, bucket_peak,
        color="tab:red", linewidth=2, marker="o", markersize=3,
        label=f"Peak |Force| (per {window_s:g}s)"
    )
    ax.plot(
        bucket_time, bucket_rms,
        color="tab:blue", linewidth=2, marker="o", markersize=3,
        label=f"RMS Force (per {window_s:g}s)"
    )

    ax.set_xlabel("Time (s, relative to recording start)")
    ax.set_ylabel("Force (g)")
    ax.set_title(f"Force Envelope Over Time -- {os.path.basename(csv_path)}")
    ax.legend(loc="upper right")
    ax.grid(True, alpha=0.3)

    fig.tight_layout()

    out_path = csv_path.rsplit(".", 1)[0] + "_envelope_plot.png"
    fig.savefig(out_path, dpi=150)
    print(f"Saved envelope plot to {out_path}")

    if show:
        plt.show()
    else:
        plt.close(fig)

    return fig


if __name__ == "__main__":
    args = sys.argv[1:]

    path = CSV_PATH
    if args and not args[0].startswith("--"):
        path = args[0]
        args = args[1:]

    if args and args[0] == "--window":
        start_s = float(args[1])
        duration_s = float(args[2])
        plot_trial_window(path, start_s, duration_s)

    elif args and args[0] == "--envelope":
        window_s = float(args[1]) if len(args) > 1 else 1.0
        plot_trial_envelope(path, window_s)

    else:
        plot_trial(path)
