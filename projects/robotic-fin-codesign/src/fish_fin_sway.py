"""
Move a Dynamixel between angles, and sway it back and forth (like a fish fin)
at a controlled speed.

Requires: dynamixel_controller.py + dynamixel_address_book.py (in this folder)
Install:  pip install dynamixel-sdk

CHANGE LOG (see CHANGES.md for full details):
  - move_to_angle() now accepts an optional `stop_event` (threading.Event) so a
    move can be aborted early, and an optional `on_update(desired_deg, actual_deg)`
    callback so callers (the GUI) can be told the commanded and measured angle
    while the motion is happening.
  - Added sway_fin_continuous(): a version of sway_fin() that runs until told to
    stop (via a threading.Event) instead of a fixed number of cycles. This is
    what backs the new "Start Dynamixel" / "Stop Dynamixel" GUI buttons.
  - sway_fin() (fixed-cycle version) and run_fish_motion() are both kept
    unchanged for standalone / script use (`python fish_fin_sway.py`).
  - Added PRESET_SWAY_SPEEDS_DPS: the speed presets (deg/s) offered by the
    GUI's speed selector, so the GUI and this module share one source of
    truth instead of the speed list being hardcoded twice.
  - Added WAVE_SHAPES ("Triangle"/"Sine"/"Square") plus
    sway_fin_sine_continuous() and sway_fin_square_continuous(), so the GUI's
    Wave Shape selector has a motion function to call for each shape.
    sway_fin_continuous() (renamed nowhere, still the "Triangle" shape) is
    unchanged. Also added triangle_speed_dps_for_frequency(), a small
    helper to convert a Frequency (Hz) setting into the equivalent Triangle
    speed (deg/s), for anyone who wants to compare shapes at the same
    fin-beat frequency.
  - Added Sawtooth: a slow rise, fast return asymmetric stroke.
  - REMOVED Pulse, Trapezoidal, and Random (and accel_dps2_to_profile_acceleration(),
    which only Trapezoidal used) -- decided these weren't needed for the GUI's
    Wave Shape selector. If you want any of them back, they're in version
    control history / earlier chat exports.
  - Dropped the separate Fin Speed (deg/s) control entirely: Frequency (Hz) is
    now the single time-based control for every shape, including Triangle.
    Triangle's underlying motion still runs at a constant angular speed, but
    that speed is now derived from Frequency + Amplitude via
    triangle_speed_dps_for_frequency() (speed = 4 * amplitude * frequency --
    one full cycle covers 4x the amplitude in distance) instead of being a
    separate user-set number.
  - Sawtooth's signature changed to match: it now takes frequency_hz (overall
    cycle time) and return_speed_dps (how fast the return snap is) instead of
    a raw rise_speed_dps. The slow rise speed is derived from whatever time is
    left in the cycle after subtracting the fast return's travel time --
    see sway_fin_sawtooth_continuous()'s docstring for the exact math.
"""

import math
from time import sleep, time
from dynamixel_controller import Dynamixel

# XL/XM series: 4096 encoder ticks = 360 degrees -> 0.088 deg/tick
TICKS_PER_REV = 4096
DEG_PER_TICK = 360.0 / TICKS_PER_REV

# Profile velocity register: 1 unit = 0.229 rev/min (X series)
REV_PER_MIN_PER_UNIT = 0.229

# Wave shapes offered by the GUI's "Wave Shape" selector. "Triangle" is the
# original/default sway (constant angular speed between the two end angles --
# what every existing trial in Data/ was recorded with). Sine, Square, and
# Sawtooth are continuous-streaming or point-to-point motions, added below.
WAVE_SHAPES = ["Triangle", "Sine", "Square", "Sawtooth"]


def angle_to_position(angle_deg):
    """Convert an angle (0-360) to a raw position tick value."""
    return int(round(angle_deg / DEG_PER_TICK))


def position_to_angle(position):
    """Convert a raw position tick value to an angle in degrees."""
    return position * DEG_PER_TICK


def speed_dps_to_profile_velocity(speed_dps):
    """Convert a desired speed in degrees/second to a profile-velocity register value."""
    rev_per_min = (speed_dps / 360.0) * 60.0
    return max(1, int(round(rev_per_min / REV_PER_MIN_PER_UNIT)))


def move_to_angle(servo, angle_deg, speed_dps=60, ID=None, wait=True, tolerance_deg=1.0,
                   stop_event=None, on_update=None):
    """Move the servo to a target angle at a given speed (deg/s) and optionally wait
    until it arrives.

    stop_event : threading.Event, optional
        If provided and set() while this call is waiting for arrival, the wait
        loop breaks immediately (the servo keeps moving toward the last goal
        position it was given -- this function just stops *waiting*/blocking).
    on_update : callable(desired_deg, actual_deg), optional
        Called once right after the goal position is written (actual_deg=None,
        since we haven't read it yet), and then again on every position poll
        while waiting for arrival. Lets a caller (e.g. the GUI) track the
        commanded vs. measured angle in real time.
    """
    servo.write_profile_velocity(speed_dps_to_profile_velocity(speed_dps), ID=ID)
    servo.write_position(angle_to_position(angle_deg), ID=ID)

    if on_update:
        on_update(angle_deg, None)

    if wait:
        while True:
            if stop_event is not None and stop_event.is_set():
                break

            current_angle = position_to_angle(servo.read_position(ID=ID))

            if on_update:
                on_update(angle_deg, current_angle)

            if abs(current_angle - angle_deg) <= tolerance_deg:
                break

            sleep(0.02)


def sway_fin(servo, center_deg=180, amplitude_deg=30, speed_dps=120, cycles=10, ID=None, pause_at_ends=0.0):
    """Sway the servo side to side around a center angle, like a fish fin, for a
    fixed number of cycles. Kept for standalone/script use.

    center_deg: resting angle in the middle of the sway
    amplitude_deg: how far it swings each direction from center
    speed_dps: sway speed in degrees/second
    cycles: number of full left-right-left sways
    """
    left_angle = center_deg - amplitude_deg
    right_angle = center_deg + amplitude_deg

    for _ in range(cycles):
        move_to_angle(servo, right_angle, speed_dps=speed_dps, ID=ID)
        sleep(pause_at_ends)

        move_to_angle(servo, left_angle, speed_dps=speed_dps, ID=ID)
        sleep(pause_at_ends)


def sway_fin_continuous(servo, stop_event, center_deg=0, amplitude_deg=35, speed_dps=60,
                         ID=None, pause_at_ends=0.0, on_update=None):
    """Sway the servo side to side around a center angle, continuously, until
    stop_event is set. This is the "Start Dynamixel" behavior used by the GUI:
    it keeps swaying left-right-left... and returns as soon as possible after
    stop_event.set() is called (checked between moves and while waiting for
    each move to finish).

    stop_event : threading.Event (required) -- call .set() from the GUI's
        "Stop Dynamixel" button to halt the sway. The servo simply stops being
        commanded to a new position; it is left wherever it was on the way to
        its last goal (torque stays enabled -- it does not go limp).
    on_update : callable(desired_deg, actual_deg), optional -- forwarded to
        move_to_angle() on every step so the GUI can log desired/actual
        position alongside force readings.
    """
    left_angle = center_deg - amplitude_deg
    right_angle = center_deg + amplitude_deg

    while not stop_event.is_set():
        move_to_angle(servo, right_angle, speed_dps=speed_dps, ID=ID,
                      stop_event=stop_event, on_update=on_update)
        if stop_event.is_set():
            break
        sleep(pause_at_ends)

        if stop_event.is_set():
            break
        move_to_angle(servo, left_angle, speed_dps=speed_dps, ID=ID,
                      stop_event=stop_event, on_update=on_update)
        if stop_event.is_set():
            break
        sleep(pause_at_ends)


def triangle_speed_dps_for_frequency(amplitude_deg, frequency_hz):
    """Angular speed (deg/s) so a Triangle sway of the given amplitude
    completes one full cycle (center -> +A -> center -> -A -> center) at the
    given frequency. One cycle covers 4*amplitude_deg of travel, so
    speed = distance / time = 4*A*f. Handy for comparing wave shapes at a
    matched fin-beat frequency."""
    return 4.0 * amplitude_deg * frequency_hz


def sway_fin_sine_continuous(servo, stop_event, center_deg=0, amplitude_deg=35, frequency_hz=0.5,
                              ID=None, update_hz=25, on_update=None):
    """Continuously stream position setpoints following a sine wave, instead
    of moving point-to-point and waiting for arrival like sway_fin_continuous
    does. This is the "Sine" Wave Shape -- smoother, more continuously-varying
    motion than the Triangle sway's constant-speed back-and-forth. Runs until
    stop_event is set.

    frequency_hz : cycles per second (one full left-right-left sway = one
        cycle).
    update_hz : how often a new setpoint is written to the servo (higher =
        smoother tracking of the sine curve, at the cost of more serial
        traffic).
    on_update : callable(desired_deg, actual_deg), optional -- same as
        sway_fin_continuous, called on every step.
    """
    if frequency_hz <= 0:
        frequency_hz = 0.01  # avoid a divide-by-zero / a "frozen" sine

    # Profile velocity fast enough that the servo itself isn't the
    # bottleneck -- comfortably above the sine's own peak angular speed
    # (peak speed of A*sin(2*pi*f*t) is A*2*pi*f).
    peak_speed_dps = amplitude_deg * 2 * math.pi * frequency_hz
    servo.write_profile_velocity(
        speed_dps_to_profile_velocity(max(peak_speed_dps * 3, 30)), ID=ID
    )

    dt = 1.0 / update_hz
    t0 = time()

    while not stop_event.is_set():

        t = time() - t0
        angle = center_deg + amplitude_deg * math.sin(2 * math.pi * frequency_hz * t)

        servo.write_position(angle_to_position(angle), ID=ID)

        if on_update:
            try:
                actual = position_to_angle(servo.read_position(ID=ID))
            except Exception:
                actual = None
            on_update(angle, actual)

        sleep(dt)


def sway_fin_square_continuous(servo, stop_event, center_deg=0, amplitude_deg=35, frequency_hz=0.5,
                                ID=None, snap_speed_dps=800, on_update=None):
    """Square-wave-like fin motion: snap quickly to one end angle, dwell,
    snap to the other end angle, dwell, and so on -- built on top of
    sway_fin_continuous with a fast "snap" speed and a computed dwell time so
    the overall cycle still runs at frequency_hz. This is the "Square" Wave
    Shape. Runs until stop_event is set.

    snap_speed_dps : how fast the servo moves between end angles (deg/s).
        The default (800) is a fast snap; the dwell time is shortened
        automatically to compensate so frequency_hz is still honored.
    """
    if frequency_hz <= 0:
        frequency_hz = 0.01

    half_period = (1.0 / frequency_hz) / 2.0
    transit_time = (2 * amplitude_deg) / snap_speed_dps if snap_speed_dps > 0 else 0.0
    pause = max(0.0, half_period - transit_time)

    sway_fin_continuous(
        servo, stop_event, center_deg=center_deg, amplitude_deg=amplitude_deg,
        speed_dps=snap_speed_dps, ID=ID, pause_at_ends=pause, on_update=on_update
    )


def sway_fin_sawtooth_continuous(servo, stop_event, center_deg=0, amplitude_deg=35, frequency_hz=0.5,
                                  return_speed_dps=400, ID=None, pause_at_ends=0.0, on_update=None):
    """Sawtooth fin motion: a slow, constant-speed rise, then a fast snap
    back (return_speed_dps) -- an asymmetric stroke, closer to how a real
    fin's power stroke and recovery stroke often differ in speed. This is
    the "Sawtooth" Wave Shape. Runs until stop_event is set.

    frequency_hz sets the overall cycle time (rise + return together), the
    same as every other shape. Since the rise and return legs run at
    different speeds, frequency_hz and return_speed_dps together determine
    the rise speed: return_time = (2*amplitude)/return_speed_dps (however
    long the fast leg actually takes), and whatever's left of the period
    (1/frequency_hz) is the rise's budget, so
    rise_speed_dps = (2*amplitude) / (period - return_time).
    If return_speed_dps is so slow that it alone would eat the whole period
    (or more), the rise time is floored to a small positive value instead of
    going to zero or negative, so the shape stays sane rather than erroring.

    return_speed_dps : speed (deg/s) of the fast return (+amplitude -> center
        -> -amplitude -> center). Defaults fast (400 dps) so the "snap back"
        reads clearly as a sawtooth rather than a symmetric triangle.
    """
    if frequency_hz <= 0:
        frequency_hz = 0.01

    period = 1.0 / frequency_hz
    return_time = (2 * amplitude_deg) / return_speed_dps if return_speed_dps > 0 else 0.0

    min_rise_time = 0.05  # floor so a too-slow return_speed_dps can't zero out the rise
    rise_time = max(period - return_time, min_rise_time)
    rise_speed_dps = (2 * amplitude_deg) / rise_time

    left_angle = center_deg - amplitude_deg
    right_angle = center_deg + amplitude_deg

    while not stop_event.is_set():
        move_to_angle(servo, right_angle, speed_dps=rise_speed_dps, ID=ID,
                      stop_event=stop_event, on_update=on_update)
        if stop_event.is_set():
            break
        sleep(pause_at_ends)

        if stop_event.is_set():
            break
        move_to_angle(servo, left_angle, speed_dps=return_speed_dps, ID=ID,
                      stop_event=stop_event, on_update=on_update)
        if stop_event.is_set():
            break
        sleep(pause_at_ends)


def run_fish_motion():
    """Standalone entry point (python fish_fin_sway.py): connects, sways for a
    fixed 10 cycles, then disconnects. Unchanged -- kept for manual testing
    outside the GUI."""

    servo = Dynamixel(
        ID=1,
        descriptive_device_name="Fish fin motor",
        series_name="xl",
        baudrate=57600,
        port_name="COM9"
    )

    servo.begin_communication()

    servo.set_operating_mode("extended position")

    sway_fin(
        servo,
        center_deg=0,
        amplitude_deg=35,
        speed_dps=60,
        cycles=10
    )

    sleep(1)

    servo.end_communication()

if __name__ == "__main__":
    run_fish_motion()
