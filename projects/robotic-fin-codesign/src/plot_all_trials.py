"""
Batch-process every trial CSV sitting in this folder: plot each one (both the
full raw plot and a whole-trial force envelope), compute force/power stats
for each, and write one combined trial_summary.csv.

CHANGE LOG:
  - Auto-discovers every *.csv in this folder instead of requiring a
    manually-maintained TRIALS dict of fixed filenames. The old dict-based
    design meant every new recording had to be added here by hand before it
    would show up in trial_summary.csv -- easy to forget, and it silently
    skipped anything not listed (no error, it just never got processed).
    Now: record a trial, run this script, it's in the summary. There's still
    no dict to point at "Small fin" vs "Big fin" by name -- that label only
    ever existed in this file, not in the data -- so each trial is now
    identified by its own filename (whatever you named it in the GUI's
    Experiment Name box, e.g. "ShortFin_0.6Hz").
  - Pulls Shape / Center / Amplitude / Speed / Frequency / Return Speed out
    of each CSV's own DXL_START event text (see parse_dxl_start() below)
    instead of only carrying a "Speed (deg/s)" number from the dict. Handles
    every event-text format this project has used, old and new.
  - Also generates a force-envelope plot for every trial now (rolling peak /
    RMS force in time buckets across the whole trial -- see
    plot_trial_envelope() in plot_trial.py), since the raw per-cycle plot is
    unreadable at high sway speeds/frequencies over a multi-minute trial.
  - Fitness Score (RMS Force / Mean Power) and Mean Power (W) computation are
    unchanged from before, just carried through per-trial the same way.
  - The old "Fin" grouping in the comparison chart is now a "Group" label
    derived from each file's own name (see derive_group_label() below) --
    e.g. "BIGFIN30.csv" and "SMALLFIN30.csv" become groups "BIGFIN" and
    "SMALLFIN" the same way they always were, just read from the filename
    instead of a manually-maintained dict. The comparison chart groups by
    Group + Shape together (not Shape alone) -- grouping by Shape alone
    would silently merge e.g. Big Fin and Small Fin's Triangle trials into
    one connected zigzag line, since fin identity was never recorded
    anywhere except in the filename. The x-axis is Frequency (Hz) instead
    of Speed (deg/s), since Frequency is now the one time-based control
    shared by every shape (see controller.py / fish_fin_sway.py) -- Speed
    is only ever set for old-style Triangle recordings and is
    back-converted to an equivalent Frequency for trials that never logged
    one directly.
"""

import glob
import os
import re
import pandas as pd
import numpy as np
import matplotlib.pyplot as plt

from plot_trial import plot_trial, plot_trial_envelope

# XL430-W250 stall current at its recommended 11.1V operating voltage
# (ROBOTIS spec) -- used to convert the logged "Present Load (%)" column into
# an estimated current draw, and from there into electrical power. This is a
# real-time proxy good enough for ranking trials against each other, not a
# lab-grade ammeter reading. Adjust if running at a meaningfully different
# supply voltage (stall current is ~1.0A at 9.0V, ~1.4A at 12.0V).
STALL_CURRENT_A = 1.3

# Bucket size (seconds) for the auto-generated envelope plots.
ENVELOPE_WINDOW_S = 2.0


def find_trial_csvs():
    """Every CSV in this folder except the summary this script itself writes."""
    return sorted(p for p in glob.glob("*.csv") if os.path.basename(p) != "trial_summary.csv")


def derive_group_label(trial_label):
    """Best-effort "which fin/config was this" label, parsed off the
    filename itself (there's nowhere else it could come from -- no sensor
    knows which physical fin is screwed onto the shaft). Strips one
    trailing numeric parameter token (with or without a unit and a leading
    underscore) so 'SMALLFIN30' -> 'SMALLFIN', 'BIGFIN120' -> 'BIGFIN',
    'ShortFin_60dps' -> 'ShortFin'. Auto-named 'Trial_<timestamp>_<freq>Hz'
    files keep most of their timestamp as a (basically unique) label, which
    is fine -- they were never meant to share an identity with anything
    else."""

    stripped = re.sub(r"_?[\d.]+(dps|Hz)?$", "", trial_label)
    return stripped if stripped else trial_label


def parse_dxl_start(event_text):
    """Pull whatever fields are present out of a DXL_START event string.
    Handles every format this project has logged over time:
        old:     'DXL_START speed=30dps'
        mid:     'DXL_START shape=Triangle center=0deg amp=35deg speed=60dps'
        current: 'DXL_START shape=Sawtooth center=0deg amp=35deg freq=0.6Hz return=400dps'
    Missing fields fall back to this project's long-standing defaults
    (Triangle shape, 0 deg center, 35 deg amplitude) instead of erroring,
    since the earliest recordings never logged shape/center/amplitude at
    all -- those were hardcoded constants in controller.py at the time.
    """

    def grab(pattern):
        m = re.search(pattern, event_text)
        return float(m.group(1)) if m else None

    shape_match = re.search(r"shape=(\S+)", event_text)
    shape = shape_match.group(1) if shape_match else "Triangle"

    center_deg = grab(r"center=(-?[\d.]+)deg")
    center_deg = 0.0 if center_deg is None else center_deg

    amplitude_deg = grab(r"amp=([\d.]+)deg")
    amplitude_deg = 35.0 if amplitude_deg is None else amplitude_deg

    speed_dps = grab(r"speed=([\d.]+)dps")
    frequency_hz = grab(r"freq=([\d.]+)Hz")
    return_speed_dps = grab(r"return=([\d.]+)dps")

    if frequency_hz is None and speed_dps is not None:
        # Old recordings only ever logged Speed (deg/s); back out the
        # equivalent Frequency the same way triangle_speed_dps_for_frequency()
        # in fish_fin_sway.py derives it the other direction
        # (speed = 4 * amplitude * frequency).
        frequency_hz = speed_dps / (4.0 * amplitude_deg)

    return {
        "shape": shape,
        "center_deg": center_deg,
        "amplitude_deg": amplitude_deg,
        "speed_dps": speed_dps,
        "frequency_hz": frequency_hz,
        "return_speed_dps": return_speed_dps,
    }


def summarize(csv_path):
    """Peak/mean/RMS force, mean electrical power, and Fitness Score,
    computed ONLY during the actual fin-swaying window (from the DXL_START
    event to the DXL_STOP event, or to the end of the file if it was never
    stopped) -- not the whole recording, which usually includes an
    idle/tare period before motion starts that would understate the real
    force (and power) during motion. Also returns whatever motion
    parameters could be parsed from the DXL_START event text."""

    df = pd.read_csv(csv_path)
    ev = df["Event"].astype(str)

    start_idx = df.index[ev.str.startswith("DXL_START")]
    stop_idx = df.index[ev.str.startswith("DXL_STOP")]

    start = start_idx[0] if len(start_idx) else df.index[0]
    stop = stop_idx[0] if len(stop_idx) else df.index[-1]

    if len(start_idx):
        params = parse_dxl_start(ev.loc[start_idx[0]])
    else:
        # No DXL_START at all in this file -- nothing to parse, leave the
        # motion-parameter fields blank rather than guessing.
        params = {
            "shape": None, "center_deg": None, "amplitude_deg": None,
            "speed_dps": None, "frequency_hz": None, "return_speed_dps": None,
        }

    motion = df.loc[start:stop]
    force = motion["Force (g)"]
    rms_force = np.sqrt((force ** 2).mean())

    # Mean Power (W) and Fitness Score (RMS Force / Mean Power) -- only if
    # this CSV was recorded with the Present Load / Input Voltage columns
    # (older trials recorded before that logging was added won't have them,
    # and there's no way to retroactively know the current draw for a trial
    # where it was simply never measured -- skip gracefully rather than
    # erroring out, same idea as plot_trial.py's handling of missing
    # Actual Position).
    has_power_data = "Present Load (%)" in df.columns and "Input Voltage (V)" in df.columns

    mean_power_w = np.nan
    fitness_score = np.nan

    if has_power_data:
        load_percent = motion["Present Load (%)"].abs()
        voltage_v = motion["Input Voltage (V)"]
        current_a = (load_percent / 100.0) * STALL_CURRENT_A
        power_w = current_a * voltage_v

        if power_w.notna().any():
            mean_power_w = power_w.mean()
            if mean_power_w > 0:
                fitness_score = rms_force / mean_power_w

    return {
        **params,
        "peak_abs_force": force.abs().max(),
        "mean_abs_force": force.abs().mean(),
        "rms_force": rms_force,
        "motion_duration_s": motion["Time (s)"].iloc[-1] - motion["Time (s)"].iloc[0],
        "had_dxl_stop": len(stop_idx) > 0,
        "mean_power_w": mean_power_w,
        "fitness_score": fitness_score,
    }


def main():

    csv_paths = find_trial_csvs()

    if not csv_paths:
        print("No trial CSVs found in this folder.")
        return

    summary_rows = []

    for path in csv_paths:

        label = os.path.splitext(os.path.basename(path))[0]

        print(f"Plotting {path} ...")
        try:
            plot_trial(path, show=False)
            plot_trial_envelope(path, window_s=ENVELOPE_WINDOW_S, show=False)
        except Exception as e:
            print(f"  (couldn't plot {path}: {e})")

        try:
            s = summarize(path)
        except Exception as e:
            print(f"  (skipping {path}, couldn't summarize: {e})")
            continue

        summary_rows.append({
            "Trial": label,
            "Group": derive_group_label(label),
            "Shape": s["shape"],
            "Center (deg)": s["center_deg"],
            "Amplitude (deg)": s["amplitude_deg"],
            "Speed (deg/s)": s["speed_dps"],
            "Frequency (Hz)": s["frequency_hz"],
            "Return Speed (deg/s)": s["return_speed_dps"],
            "Peak |Force| (g)": s["peak_abs_force"],
            "Mean |Force| (g)": s["mean_abs_force"],
            "RMS Force (g)": s["rms_force"],
            "Motion Duration (s)": s["motion_duration_s"],
            "Had DXL_STOP": s["had_dxl_stop"],
            "Mean Power (W)": s["mean_power_w"],
            "Fitness Score (gF/W)": s["fitness_score"],
        })

    if not summary_rows:
        print("\nNo trials could be summarized.")
        return

    summary = pd.DataFrame(summary_rows).sort_values("Trial")

    print("\nSummary (force/power stats computed only during DXL_START->DXL_STOP window):")
    print("Fitness Score = RMS Force / Mean Power -- a force-per-watt ranking figure,")
    print("not a thermodynamic efficiency percentage. NaN means either that trial's CSV")
    print("was recorded before Present Load / Input Voltage logging was added, or it has")
    print("no DXL_START event at all to parse motion parameters from.")
    print(summary.to_string(index=False))

    summary.to_csv("trial_summary.csv", index=False)
    print("\nSaved trial_summary.csv")

    # ---- Comparison chart: grouped by Group + Wave Shape together, x-axis
    # is Frequency (Hz). Grouping by Shape alone would silently connect e.g.
    # Big Fin's and Small Fin's Triangle trials into one zigzagging line,
    # since fin identity only ever existed in the filename -- Group (derived
    # from the filename, see derive_group_label()) keeps them separate. ----
    plottable = summary.dropna(subset=["Frequency (Hz)", "Shape"])

    if plottable.empty:
        print("\nNo trials had a parseable Frequency -- skipping comparison chart.")
        return

    fig, axes = plt.subplots(1, 2, figsize=(14, 6))

    for (group_label, shape_label), group in plottable.groupby(["Group", "Shape"]):
        group = group.sort_values("Frequency (Hz)")
        series_label = f"{group_label} - {shape_label}"
        axes[0].plot(group["Frequency (Hz)"], group["Peak |Force| (g)"], marker="o", linewidth=2, label=series_label)
        axes[1].plot(group["Frequency (Hz)"], group["RMS Force (g)"], marker="o", linewidth=2, label=series_label)

    axes[0].set_xlabel("Fin Sway Frequency (Hz)")
    axes[0].set_ylabel("Peak |Force| (g)")
    axes[0].set_title("Peak Force vs Frequency, by Fin/Config + Wave Shape")
    axes[0].legend()
    axes[0].grid(True, alpha=0.3)

    axes[1].set_xlabel("Fin Sway Frequency (Hz)")
    axes[1].set_ylabel("RMS Force (g)")
    axes[1].set_title("RMS Force vs Frequency, by Fin/Config + Wave Shape")
    axes[1].legend()
    axes[1].grid(True, alpha=0.3)

    fig.tight_layout()
    fig.savefig("speed_vs_force_comparison.png", dpi=150)
    print("Saved speed_vs_force_comparison.png")
    plt.close(fig)


if __name__ == "__main__":
    main()
