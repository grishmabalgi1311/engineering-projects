import tkinter as tk
from tkinter import ttk
import threading
import serial
import time
import os
import pandas as pd
from datetime import datetime

import matplotlib.pyplot as plt
from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg

from dynamixel_controller import Dynamixel
from dynamixel_address_book import ADDR_PRESENT_INPUT_VOLTAGE
from fish_fin_sway import (
    sway_fin_continuous,
    sway_fin_sine_continuous,
    sway_fin_square_continuous,
    sway_fin_sawtooth_continuous,
    triangle_speed_dps_for_frequency,
    WAVE_SHAPES,
)

# ==========================================================
# USER SETTINGS
# ==========================================================

ARDUINO_PORT = "COM8"
DYNAMIXEL_PORT = "COM9"

BAUD = 115200

# Must match the Dynamixel's actual EEPROM settings (check with Dynamixel
# Wizard's baudrate/ID scan if "Connect Dynamixel" reports no reply from the
# motor) -- a mismatch here is the #1 cause of "no status packet" errors.
DYNAMIXEL_BAUD = 57600
DYNAMIXEL_ID = 1

# Fish fin sway settings (used by Start Dynamixel) -- these are just the
# fallback defaults that pre-fill the GUI fields; Wave Shape, Center Position,
# Amplitude, Frequency, and Return Speed are all editable live in the
# sidebar.
DXL_CENTER_DEG = 0  # resting midpoint of the sway (deg) -- shifts both end angles together
DXL_AMPLITUDE_DEG = 35  # how far the fin swings from center, each direction (deg)
DXL_FREQUENCY_HZ = 0.5  # default / fallback frequency (Hz) -- used by every shape
DXL_RETURN_SPEED_DPS = 400  # default / fallback fast "snap back" speed (deg/s) -- used by Sawtooth

# For estimating electrical power draw (Present Load % * this = estimated
# current draw). 1.3A is the XL430-W250's stall current at its recommended
# 11.1V operating voltage (ROBOTIS spec). This is a real-time proxy good
# enough for ranking trials against each other -- not a lab-grade ammeter
# reading. If you're running at a meaningfully different supply voltage,
# adjust this to match (stall current is ~1.0A at 9.0V, ~1.4A at 12.0V).
DXL_STALL_CURRENT_A = 1.3


def compute_power_w(load_percent, voltage_v):
    """Estimated electrical power (W) from Present Load (%) and Present
    Input Voltage (V). Returns NaN if either input is unknown yet (e.g.
    before the Dynamixel's first poll). abs() on load_percent because Load
    is signed (direction of torque) -- power draw doesn't care which way the
    motor is pushing."""

    if load_percent is None or voltage_v is None:
        return float("nan")

    current_a = (abs(load_percent) / 100.0) * DXL_STALL_CURRENT_A
    return current_a * voltage_v


def compute_power_to_force_ratio(power_w, force_g):
    """Power (W) / |Force (g)| for one instant. NaN if power is unknown or
    force is ~0 -- Force oscillates through zero constantly, so this ratio
    spikes toward the extreme near every zero-crossing. That's an inherent
    property of dividing by a value near zero, not a bug: treat this column
    as a diagnostic, not something to average directly. For ranking
    trials/fins against each other, use the whole-trial Fitness Score
    (RMS Force / Mean Power) computed in Data/plot_all_trials.py instead."""

    if power_w != power_w:  # NaN check without needing to import math/numpy
        return float("nan")

    if abs(force_g) < 1e-6:
        return float("nan")

    return power_w / abs(force_g)

# ==========================================================
# GLOBAL VARIABLES
# ==========================================================

arduino = None
dynamixel = None

running = False
recording = False

current_force = 0.0
sample_count = 0

start_time = None

force_history = []
time_history = []
desired_pos_history = []
actual_pos_history = []

record_time = []
record_force = []
record_event = []
record_desired_pos = []
record_actual_pos = []
record_load = []
record_voltage = []
record_power = []
record_power_to_force_ratio = []

csv_filename = ""

# Dynamixel motion state
dynamixel_stop_event = threading.Event()
dynamixel_running = False

# Last known commanded (desired) / measured (actual) fin angle, in degrees.
# None until the dynamixel has been connected / moved at least once.
current_desired_position = None
current_actual_position = None

# Last known Present Load (%) / Present Input Voltage (V), read from the
# Dynamixel itself (used to estimate electrical power). None until read at
# least once. Polled from inside on_dynamixel_position_update() -- see the
# comment there for why it's throttled and why it isn't a separate thread.
current_present_load_percent = None
current_input_voltage_v = None
_last_power_poll_time = 0.0

# ==========================================================
# MAIN WINDOW
# ==========================================================

root = tk.Tk()

root.title("Fish Force Testing System")

# Size the window to the actual screen instead of a hardcoded 1100x700 -- on a
# small/short laptop display, a fixed 700px-tall window (plus title bar and
# taskbar) can be taller than the usable desktop, which clips the bottom of
# the sidebar. Cap it to the screen size with a margin instead.
screen_w = root.winfo_screenwidth()
screen_h = root.winfo_screenheight()

win_w = min(1100, screen_w - 60)
win_h = min(700, screen_h - 90)

root.geometry(f"{win_w}x{win_h}")
root.minsize(700, 480)

# ==========================================================
# MAIN LAYOUT
# ==========================================================

rightFrame = ttk.Frame(root,padding=15)
rightFrame.grid(row=0,column=1,sticky="nsew")

bottomFrame = ttk.Frame(root,padding=10)
bottomFrame.grid(row=1,column=0,columnspan=2,sticky="ew")

root.grid_columnconfigure(1,weight=1)
root.grid_rowconfigure(0,weight=1)

# --- Scrollable left sidebar --------------------------------------------
# The sidebar stacks a lot of controls (status, force readout, every button,
# the speed selector, the log box). Even with the screen-aware sizing above,
# a small enough laptop screen can still be shorter than everything fits in.
# Wrapping the sidebar in a canvas + scrollbar means nothing ever gets clipped
# off-screen -- worst case, you scroll the sidebar instead of losing buttons.

leftContainer = ttk.Frame(root)
leftContainer.grid(row=0,column=0,sticky="ns")

leftCanvas = tk.Canvas(leftContainer, borderwidth=0, highlightthickness=0, width=260)
leftScrollbar = ttk.Scrollbar(leftContainer, orient="vertical", command=leftCanvas.yview)
leftCanvas.configure(yscrollcommand=leftScrollbar.set)

leftCanvas.pack(side="left", fill="both", expand=True)
leftScrollbar.pack(side="right", fill="y")

leftFrame = ttk.Frame(leftCanvas, padding=15)
leftFrameWindow = leftCanvas.create_window((0, 0), window=leftFrame, anchor="nw")

def _update_left_scrollregion(event=None):
    leftCanvas.configure(scrollregion=leftCanvas.bbox("all"))

def _resize_left_frame(event):
    leftCanvas.itemconfig(leftFrameWindow, width=event.width)

leftFrame.bind("<Configure>", _update_left_scrollregion)
leftCanvas.bind("<Configure>", _resize_left_frame)

def _on_left_mousewheel(event):
    leftCanvas.yview_scroll(int(-1 * (event.delta / 120)), "units")

def _bind_left_mousewheel(event):
    leftCanvas.bind_all("<MouseWheel>", _on_left_mousewheel)

def _unbind_left_mousewheel(event):
    leftCanvas.unbind_all("<MouseWheel>")

# Only capture the scroll wheel while the cursor is actually over the
# sidebar, so scrolling elsewhere in the app (e.g. over the graph) is
# unaffected.
leftCanvas.bind("<Enter>", _bind_left_mousewheel)
leftCanvas.bind("<Leave>", _unbind_left_mousewheel)

# ==========================================================
# TITLE
# ==========================================================

title = ttk.Label(
    leftFrame,
    text="Fish Force\nTesting System",
    font=("Arial",22,"bold"),
    justify="center"
)

title.pack(pady=(0,12))

# ==========================================================
# STATUS
# ==========================================================

arduinoStatus = ttk.Label(
    leftFrame,
    text="Arduino : Not Connected",
    foreground="red",
    font=("Arial",11)
)

arduinoStatus.pack(anchor="w")

dynamixelStatus = ttk.Label(
    leftFrame,
    text="Dynamixel : Not Connected",
    foreground="red",
    font=("Arial",11)
)

dynamixelStatus.pack(anchor="w",pady=(0,12))

# ==========================================================
# CURRENT FORCE
# ==========================================================

ttk.Label(
    leftFrame,
    text="Current Force",
    font=("Arial",13)
).pack()

forceLabel = ttk.Label(
    leftFrame,
    text="0.00 g",
    font=("Arial",34,"bold")
)

forceLabel.pack(pady=(0,12))

# ==========================================================
# EXPERIMENT NAME
# ==========================================================

ttk.Label(
    leftFrame,
    text="Experiment Name",
    font=("Arial",11)
).pack(anchor="w")

experimentEntry = ttk.Entry(
    leftFrame,
    width=28
)

experimentEntry.pack(fill="x",pady=(0,12))

# ==========================================================
# CONTROL BUTTONS
# ==========================================================

connectArduinoBtn = ttk.Button(
    leftFrame,
    text="Connect Arduino",
    width=25
)

connectArduinoBtn.pack(pady=3)

connectDXLBtn = ttk.Button(
    leftFrame,
    text="Connect Dynamixel",
    width=25
)

connectDXLBtn.pack(pady=3)

startBtn = ttk.Button(
    leftFrame,
    text="Start Recording",
    width=25,
    state="disabled"
)

startBtn.pack(pady=3)

clampBtn = ttk.Button(
    leftFrame,
    text="Clamp",
    width=25,
    state="disabled"
)

clampBtn.pack(pady=3)

unclampBtn = ttk.Button(
    leftFrame,
    text="Unclamp",
    width=25,
    state="disabled"
)

unclampBtn.pack(pady=3)

# ==========================================================
# FIN MOTION SETTINGS
# (Wave Shape / Center / Amplitude / Frequency / Return Speed)
# ==========================================================
# None of these touch a running sway -- whatever is in the boxes when "Start
# Dynamixel" is pressed is what gets used for that run, so you can change
# them between runs. Only the fields relevant to the currently-selected Wave
# Shape actually matter (see each label) -- the rest are just ignored, no
# need to clear them out.

ttk.Label(
    leftFrame,
    text="Wave Shape",
    font=("Arial",11)
).pack(anchor="w",pady=(10,0))

dxlShapeVar = tk.StringVar(value=WAVE_SHAPES[0])  # "Triangle" (the original behavior)

dxlShapeCombo = ttk.Combobox(
    leftFrame,
    textvariable=dxlShapeVar,
    values=WAVE_SHAPES,
    state="readonly",
    width=22
)

# Triangle = constant-speed sway (speed derived from Frequency + Amplitude).
# Sine     = smooth continuous sway (Frequency).
# Square   = fast snap + even dwell at each end (Frequency).
# Sawtooth = slow rise (from Frequency + Amplitude), fast return (Return Speed).
dxlShapeCombo.pack(pady=(0,10))

ttk.Label(
    leftFrame,
    text="Center Position (deg)",
    font=("Arial",11)
).pack(anchor="w")

dxlCenterVar = tk.StringVar(value=str(DXL_CENTER_DEG))

dxlCenterEntry = ttk.Entry(
    leftFrame,
    textvariable=dxlCenterVar,
    width=24
)

# Shifts both end angles together -- e.g. Center=50, Amplitude=30 sways
# between 20 deg and 80 deg instead of around 0. Used by every shape.
dxlCenterEntry.pack(pady=(0,10))

ttk.Label(
    leftFrame,
    text="Amplitude (deg)",
    font=("Arial",11)
).pack(anchor="w")

dxlAmplitudeVar = tk.StringVar(value=str(DXL_AMPLITUDE_DEG))

dxlAmplitudeEntry = ttk.Entry(
    leftFrame,
    textvariable=dxlAmplitudeVar,
    width=24
)

# How far the sway swings from Center Position, each direction. Used by
# every shape.
dxlAmplitudeEntry.pack(pady=(0,10))

ttk.Label(
    leftFrame,
    text="Frequency (Hz)",
    font=("Arial",11)
).pack(anchor="w")

dxlFrequencyVar = tk.StringVar(value=str(DXL_FREQUENCY_HZ))

dxlFrequencyEntry = ttk.Entry(
    leftFrame,
    textvariable=dxlFrequencyVar,
    width=24
)

# Used by every shape -- for Triangle and Sawtooth's rise leg, it's converted
# into the constant angular speed needed to complete one cycle in 1/frequency
# seconds at the current Amplitude (speed = 4 * amplitude * frequency).
dxlFrequencyEntry.pack(pady=(0,10))

ttk.Label(
    leftFrame,
    text="Return Speed (deg/s) -- Sawtooth",
    font=("Arial",11)
).pack(anchor="w")

dxlReturnSpeedVar = tk.StringVar(value=str(DXL_RETURN_SPEED_DPS))

dxlReturnSpeedEntry = ttk.Entry(
    leftFrame,
    textvariable=dxlReturnSpeedVar,
    width=24
)

# The fast "snap back" speed for Sawtooth's return stroke. The slow rise
# speed is derived from Frequency + Amplitude + this value (see
# sway_fin_sawtooth_continuous()'s docstring in fish_fin_sway.py).
dxlReturnSpeedEntry.pack(pady=(0,10))

startDXLBtn = ttk.Button(
    leftFrame,
    text="Start Dynamixel",
    width=25,
    state="disabled"
)

startDXLBtn.pack(pady=3)

stopDXLBtn = ttk.Button(
    leftFrame,
    text="Stop Dynamixel",
    width=25,
    state="disabled"
)

stopDXLBtn.pack(pady=3)

tareBtn = ttk.Button(
    leftFrame,
    text="Tare",
    width=25,
    state="disabled"
)

tareBtn.pack(pady=3)

stopBtn = ttk.Button(
    leftFrame,
    text="Stop Recording",
    width=25,
    state="disabled"
)

stopBtn.pack(pady=3)

# ==========================================================
# GRAPH
# ==========================================================

figure = plt.Figure(figsize=(8,5),dpi=100)

ax = figure.add_subplot(111)

ax.set_title("Force vs Time")

ax.set_xlabel("Time (s)")
ax.set_ylabel("Force (g)")

ax.grid(True)

line, = ax.plot([],[],linewidth=2,color="tab:blue",label="Force (g)")

# Live fin position, plotted on a secondary axis (different scale/units than
# force) so both update together in real time on the same graph -- same idea
# as plot_trial.py's after-the-fact plot, just live while recording.
ax_pos = ax.twinx()
ax_pos.set_ylabel("Fin Position (deg)")

desiredPosLine, = ax_pos.plot(
    [],[],linewidth=1.3,linestyle="--",color="tab:orange",label="Desired Position (deg)"
)

actualPosLine, = ax_pos.plot(
    [],[],linewidth=1.5,color="tab:green",label="Actual Position (deg)"
)

_lines = [line, desiredPosLine, actualPosLine]
ax.legend(_lines, [l.get_label() for l in _lines], loc="upper right")

canvas = FigureCanvasTkAgg(
    figure,
    master=rightFrame
)

canvas.get_tk_widget().pack(
    fill="both",
    expand=True
)

# ==========================================================
# STATUS BAR
# ==========================================================

recordLabel = ttk.Label(
    bottomFrame,
    text="⚫ Not Recording"
)

recordLabel.pack(side="left",padx=10)

sampleLabel = ttk.Label(
    bottomFrame,
    text="Samples : 0"
)

sampleLabel.pack(side="left",padx=20)

dxlPositionLabel = ttk.Label(
    bottomFrame,
    text="Fin Position : desired -- / actual --"
)

dxlPositionLabel.pack(side="left",padx=20)

csvLabel = ttk.Label(
    bottomFrame,
    text="CSV : None"
)

csvLabel.pack(side="right",padx=10)

logBox = tk.Text(
    leftFrame,
    height=5,
    width=30,
    state="disabled"
)

logBox.pack(pady=(10,0),fill="both",expand=True)

# ==========================================================
# LOG FUNCTION
# ==========================================================

def add_log(message):

    logBox.configure(state="normal")

    timestamp = datetime.now().strftime("%H:%M:%S")

    logBox.insert(
        tk.END,
        f"{timestamp}   {message}\n"
    )

    logBox.see(tk.END)

    logBox.configure(state="disabled")


# ==========================================================
# CONNECT ARDUINO
# ==========================================================

def connect_arduino():

    global arduino
    global running
    global start_time

    try:

        arduino = serial.Serial(
            ARDUINO_PORT,
            BAUD,
            timeout=0.1
        )

        time.sleep(2)

        running = True

        start_time = time.time()

        threading.Thread(
            target=serial_thread,
            daemon=True
        ).start()

        arduinoStatus.config(
            text="Arduino : Connected",
            foreground="green"
        )

        startBtn.config(state="normal")
        clampBtn.config(state="normal")
        unclampBtn.config(state="normal")
        tareBtn.config(state="normal")

        add_log("Arduino Connected")

    except Exception as e:

        add_log(str(e))


# ==========================================================
# CONNECT DYNAMIXEL
# ==========================================================

def connect_dynamixel():

    global dynamixel

    try:

        dynamixel = Dynamixel(
            ID=DYNAMIXEL_ID,
            descriptive_device_name="Fish fin motor",
            series_name="xl",
            baudrate=DYNAMIXEL_BAUD,
            port_name=DYNAMIXEL_PORT
        )

        dynamixel.begin_communication()

        # begin_communication() only opens the local COM port -- it does NOT
        # confirm the motor itself is actually there and listening. A wrong
        # DYNAMIXEL_BAUD or DYNAMIXEL_ID (or bad wiring) will "connect"
        # successfully here and then fail on every subsequent read/write with
        # "There is no status packet!" errors. Ping the motor now so a bad
        # config shows up immediately as a clear error instead of a false
        # "Connected".
        if not dynamixel.ping():

            raise RuntimeError(
                f"Opened {DYNAMIXEL_PORT}, but the motor never replied to a ping "
                f"(no status packet). This almost always means DYNAMIXEL_BAUD "
                f"({DYNAMIXEL_BAUD}) or DYNAMIXEL_ID ({DYNAMIXEL_ID}) at the top "
                f"of this file don't match the motor's actual EEPROM settings -- "
                f"use Dynamixel Wizard's baudrate/ID scan to check, then update "
                f"those two constants. Also double-check wiring/power."
            )

        dynamixel.set_operating_mode("extended position")

        dynamixelStatus.config(
            text="Dynamixel : Connected",
            foreground="green"
        )

        startDXLBtn.config(state="normal")

        add_log("Dynamixel Connected")

    except Exception as e:

        if dynamixel is not None:
            try:
                dynamixel.port_handler.closePort()
            except Exception:
                pass

        dynamixel = None

        dynamixelStatus.config(
            text="Dynamixel : Not Connected",
            foreground="red"
        )

        startDXLBtn.config(state="disabled")

        add_log(f"Dynamixel connect failed: {e}")


# ==========================================================
# SERIAL THREAD
# ==========================================================

def serial_thread():

    global current_force
    global sample_count

    while running:

        try:

            if arduino.in_waiting:

                lineText = arduino.readline().decode(
                    errors="ignore"
                ).strip()

                if lineText.startswith("F,"):

                    value = float(
                        lineText.split(",")[1]
                    )

                    current_force = value

                    elapsed = time.time() - start_time

                    force_history.append(value)
                    time_history.append(elapsed)

                    # Snapshot whatever the fin's desired/actual position is
                    # right now, alongside this force sample, so the live
                    # graph can plot position vs the same time axis as force.
                    # NaN (not None) when unknown yet, so matplotlib just
                    # leaves a gap instead of drawing a false line to 0.
                    desired_pos_history.append(
                        current_desired_position if current_desired_position is not None else float("nan")
                    )
                    actual_pos_history.append(
                        current_actual_position if current_actual_position is not None else float("nan")
                    )

                    sample_count += 1

                    if len(force_history) > 1000:

                        force_history.pop(0)
                        time_history.pop(0)
                        desired_pos_history.pop(0)
                        actual_pos_history.pop(0)

                    if recording:

                        power_w = compute_power_w(current_present_load_percent, current_input_voltage_v)

                        record_time.append(elapsed)
                        record_force.append(value)
                        record_event.append("")
                        record_desired_pos.append(current_desired_position)
                        record_actual_pos.append(current_actual_position)
                        record_load.append(current_present_load_percent)
                        record_voltage.append(current_input_voltage_v)
                        record_power.append(power_w)
                        record_power_to_force_ratio.append(compute_power_to_force_ratio(power_w, value))

                elif lineText.startswith("E,"):

                    add_log(lineText[2:])

        except:

            pass


# ==========================================================
# DYNAMIXEL SWAY THREAD
# ==========================================================

def on_dynamixel_position_update(desired_deg, actual_deg):

    global current_desired_position
    global current_actual_position
    global current_present_load_percent
    global current_input_voltage_v
    global _last_power_poll_time

    current_desired_position = desired_deg

    if actual_deg is not None:

        current_actual_position = actual_deg

    # Piggyback the Present Load / Present Input Voltage reads onto this
    # existing per-step callback, which already runs on the one thread that
    # owns the Dynamixel's serial port during a sway (dynamixel_sway_thread).
    # This deliberately does NOT use a separate polling thread: a second
    # thread issuing its own reads on the same serial port at the same time
    # as the sway motion would race with it (the port isn't safe for two
    # threads to talk on at once) and could corrupt both readings. Throttled
    # to ~150ms so these two extra serial round-trips don't get added to
    # every single motion tick -- that would measurably slow down the sway's
    # actual timing, especially for the faster Sine/Random update rates.
    # Once every ~150ms is still plenty of samples for a trial-long average.
    now = time.time()

    if now - _last_power_poll_time >= 0.15:

        _last_power_poll_time = now

        try:
            # 0.1 %/unit -- see the note in dynamixel_address_book.py: this
            # is "Present Load" (%) on the XL-series, not raw current in mA.
            current_present_load_percent = dynamixel.read_current() * 0.1

            # 0.1 V/unit.
            current_input_voltage_v = dynamixel.read_from_address(
                2, ADDR_PRESENT_INPUT_VOLTAGE
            ) * 0.1

        except Exception:
            pass


def dynamixel_sway_thread(shape, center_deg, amplitude_deg, frequency_hz, return_speed_dps):

    try:

        if shape == "Sine":

            sway_fin_sine_continuous(
                dynamixel,
                dynamixel_stop_event,
                center_deg=center_deg,
                amplitude_deg=amplitude_deg,
                frequency_hz=frequency_hz,
                on_update=on_dynamixel_position_update
            )

        elif shape == "Square":

            sway_fin_square_continuous(
                dynamixel,
                dynamixel_stop_event,
                center_deg=center_deg,
                amplitude_deg=amplitude_deg,
                frequency_hz=frequency_hz,
                on_update=on_dynamixel_position_update
            )

        elif shape == "Sawtooth":

            sway_fin_sawtooth_continuous(
                dynamixel,
                dynamixel_stop_event,
                center_deg=center_deg,
                amplitude_deg=amplitude_deg,
                frequency_hz=frequency_hz,
                return_speed_dps=return_speed_dps,
                on_update=on_dynamixel_position_update
            )

        else:
            # "Triangle" -- speed derived from Frequency + Amplitude instead
            # of being a separate user-set field; motion itself is unchanged.

            speed_dps = triangle_speed_dps_for_frequency(amplitude_deg, frequency_hz)

            sway_fin_continuous(
                dynamixel,
                dynamixel_stop_event,
                center_deg=center_deg,
                amplitude_deg=amplitude_deg,
                speed_dps=speed_dps,
                on_update=on_dynamixel_position_update
            )

    except Exception as e:

        add_log(f"Dynamixel motion error: {e}")

    add_log("Dynamixel Motion Stopped")


# ==========================================================
# UPDATE DISPLAY
# ==========================================================

def update_display():

    forceLabel.config(
        text=f"{current_force:.2f} g"
    )

    sampleLabel.config(
        text=f"Samples : {sample_count}"
    )

    desired_text = "--" if current_desired_position is None else f"{current_desired_position:.1f}°"
    actual_text = "--" if current_actual_position is None else f"{current_actual_position:.1f}°"

    dxlPositionLabel.config(
        text=f"Fin Position : desired {desired_text} / actual {actual_text}"
    )

    if len(time_history) > 2:

        line.set_data(
            time_history,
            force_history
        )

        desiredPosLine.set_data(
            time_history,
            desired_pos_history
        )

        actualPosLine.set_data(
            time_history,
            actual_pos_history
        )

        ax.relim()

        ax.autoscale_view()

        ax_pos.relim()

        ax_pos.autoscale_view()

        canvas.draw_idle()

    root.after(
        50,
        update_display
    )


# ==========================================================
# RECORDING
# ==========================================================

def build_recording_filename(experiment_name):
    """Build a descriptive CSV filename that bakes in the currently selected
    fin sway frequency, e.g. 'ShortFin_0.5Hz.csv' (or
    'Trial_<timestamp>_0.5Hz.csv' if no experiment name was typed), so a
    trial's frequency is visible from the filename alone instead of only
    inside the CSV."""

    frequency_hz = get_selected_frequency_hz()
    freq_tag = f"{frequency_hz:g}Hz"

    if experiment_name == "":

        return datetime.now().strftime(
            f"Trial_%Y%m%d_%H%M%S_{freq_tag}.csv"
        )

    else:

        return f"{experiment_name}_{freq_tag}.csv"


def start_recording():

    global recording
    global csv_filename

    record_time.clear()
    record_force.clear()
    record_event.clear()
    record_desired_pos.clear()
    record_actual_pos.clear()
    record_load.clear()
    record_voltage.clear()
    record_power.clear()
    record_power_to_force_ratio.clear()

    experiment_name = experimentEntry.get().strip()

    csv_filename = build_recording_filename(experiment_name)

    recording = True

    recordLabel.config(
        text="🟢 Recording",
        foreground="green"
    )

    csvLabel.config(
        text=f"CSV : {csv_filename}"
    )

    stopBtn.config(
        state="normal"
    )

    add_log("Recording Started")


def stop_recording():

    global recording

    recording = False

    recordLabel.config(
        text="⚫ Not Recording",
        foreground="black"
    )

    os.makedirs(
        "Data",
        exist_ok=True
    )

    filepath = os.path.join(
        "Data",
        csv_filename
    )

    df = pd.DataFrame({

        "Time (s)" : record_time,
        "Force (g)" : record_force,
        "Event" : record_event,
        "Desired Position (deg)" : record_desired_pos,
        "Actual Position (deg)" : record_actual_pos,
        "Present Load (%)" : record_load,
        "Input Voltage (V)" : record_voltage,
        "Power (W)" : record_power,
        "Power/Force Ratio (W/g)" : record_power_to_force_ratio

    })

    df.to_csv(
        filepath,
        index=False
    )

    stopBtn.config(
        state="disabled"
    )

    add_log(
        f"CSV Saved : {csv_filename}"
    )


def save_event(event_name):

    if recording:

        record_time.append(
            time.time() - start_time
        )

        record_force.append(
            current_force
        )

        record_event.append(
            event_name
        )

        record_desired_pos.append(
            current_desired_position
        )

        record_actual_pos.append(
            current_actual_position
        )

        record_load.append(
            current_present_load_percent
        )

        record_voltage.append(
            current_input_voltage_v
        )

        _event_power_w = compute_power_w(current_present_load_percent, current_input_voltage_v)

        record_power.append(
            _event_power_w
        )

        record_power_to_force_ratio.append(
            compute_power_to_force_ratio(_event_power_w, current_force)
        )


# ==========================================================
# BUTTON FUNCTIONS
# ==========================================================

def clamp():

    if arduino:

        arduino.write(b"c\n")

        save_event("CLAMP")

        add_log("Clamp")


def unclamp():

    if arduino:

        arduino.write(b"u\n")

        save_event("UNCLAMP")

        add_log("Unclamp")


def tare():

    if arduino:

        arduino.write(b"t\n")

        save_event("TARE")

        add_log("Tare")


def get_selected_amplitude_deg():
    """Read + validate the amplitude typed in the Amplitude box. Falls back
    to DXL_AMPLITUDE_DEG (and logs a warning) if it's not a positive number."""

    raw = dxlAmplitudeVar.get().strip()

    try:
        amplitude_deg = float(raw)
        if amplitude_deg <= 0:
            raise ValueError

    except ValueError:

        add_log(f"Invalid amplitude '{raw}', using default {DXL_AMPLITUDE_DEG} deg")

        amplitude_deg = DXL_AMPLITUDE_DEG

    return amplitude_deg


def get_selected_frequency_hz():
    """Read + validate the frequency typed in the Frequency box (used by
    every wave shape). Falls back to DXL_FREQUENCY_HZ if invalid."""

    raw = dxlFrequencyVar.get().strip()

    try:
        frequency_hz = float(raw)
        if frequency_hz <= 0:
            raise ValueError

    except ValueError:

        add_log(f"Invalid frequency '{raw}', using default {DXL_FREQUENCY_HZ} Hz")

        frequency_hz = DXL_FREQUENCY_HZ

    return frequency_hz


def get_selected_center_deg():
    """Read + validate the center position typed in the Center Position box.
    Falls back to DXL_CENTER_DEG (and logs a warning) if it's not a number.
    Unlike the other fields this one is allowed to be negative/zero -- it's
    a position, not a magnitude."""

    raw = dxlCenterVar.get().strip()

    try:
        center_deg = float(raw)

    except ValueError:

        add_log(f"Invalid center position '{raw}', using default {DXL_CENTER_DEG} deg")

        center_deg = DXL_CENTER_DEG

    return center_deg


def get_selected_return_speed_dps():
    """Read + validate the speed typed in the Return Speed box (used by the
    Sawtooth wave shape). Falls back to DXL_RETURN_SPEED_DPS if invalid."""

    raw = dxlReturnSpeedVar.get().strip()

    try:
        return_speed_dps = float(raw)
        if return_speed_dps <= 0:
            raise ValueError

    except ValueError:

        add_log(f"Invalid return speed '{raw}', using default {DXL_RETURN_SPEED_DPS} deg/s")

        return_speed_dps = DXL_RETURN_SPEED_DPS

    return return_speed_dps


def get_selected_wave_shape():
    """Read the Wave Shape selector. Falls back to Triangle (the original
    behavior) if it's ever somehow empty."""

    shape = dxlShapeVar.get().strip()

    return shape if shape in WAVE_SHAPES else WAVE_SHAPES[0]


def start_dynamixel():

    global dynamixel_running

    if dynamixel is None:
        return

    shape = get_selected_wave_shape()
    center_deg = get_selected_center_deg()
    amplitude_deg = get_selected_amplitude_deg()
    frequency_hz = get_selected_frequency_hz()
    return_speed_dps = get_selected_return_speed_dps()

    dynamixel_stop_event.clear()

    dynamixel_running = True

    # Only mention the field(s) that actually matter for the chosen shape,
    # so the log/CSV event stays readable instead of listing every setting
    # every time.
    if shape == "Sawtooth":
        shape_details = f"freq={frequency_hz:g}Hz return={return_speed_dps:g}dps"
    else:
        shape_details = f"freq={frequency_hz:g}Hz"

    save_event(f"DXL_START shape={shape} center={center_deg:g}deg amp={amplitude_deg:g}deg {shape_details}".strip())
    add_log(f"Dynamixel Started -- {shape}, center {center_deg:g} deg, amplitude {amplitude_deg:g} deg ({shape_details})")

    startDXLBtn.config(state="disabled")
    stopDXLBtn.config(state="normal")

    threading.Thread(
        target=dynamixel_sway_thread,
        args=(shape, center_deg, amplitude_deg, frequency_hz, return_speed_dps),
        daemon=True
    ).start()


def stop_dynamixel():

    global dynamixel_running

    dynamixel_stop_event.set()

    dynamixel_running = False

    save_event("DXL_STOP")

    add_log("Dynamixel Stop Requested")

    startDXLBtn.config(state="normal")
    stopDXLBtn.config(state="disabled")


# ==========================================================
# WIRE UP BUTTONS
# ==========================================================

connectArduinoBtn.config(
    command=connect_arduino
)

connectDXLBtn.config(
    command=connect_dynamixel
)

startBtn.config(
    command=start_recording
)

stopBtn.config(
    command=stop_recording
)

clampBtn.config(
    command=clamp
)

unclampBtn.config(
    command=unclamp
)

startDXLBtn.config(
    command=start_dynamixel
)

stopDXLBtn.config(
    command=stop_dynamixel
)

tareBtn.config(
    command=tare
)

update_display()

# ==========================================================
# CLOSE WINDOW
# ==========================================================

def on_close():

    global running

    running = False

    dynamixel_stop_event.set()

    if arduino is not None:

        try:
            arduino.close()
        except Exception:
            pass

    if dynamixel is not None:

        try:
            dynamixel.end_communication()
        except Exception:
            pass

    root.destroy()


root.protocol(
    "WM_DELETE_WINDOW",
    on_close
)

root.mainloop()
