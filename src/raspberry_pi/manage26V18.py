#!/usr/bin/env python3
"""manage26V18.py - WRO Future Engineers 2026 vehicle program.

Run one of these modes (exactly one is required):
    --recording            PS4 pad driving, camera view, records to a Tub
    --driving              Trained model, including the parking-exit routine
    --driving-view         Same as --driving, with a live camera window
    --driving-skip         Trained model, skips the parking-exit routine
    --driving-view-skip    Same as --driving-skip, with a live camera window

Drive modes are FCW / FCCW (free run) and OCW / OCCW (obstacle run), chosen
with --drive-mode, inferred from --model, or picked on the Sense HAT joystick.

Architecture:
    Pi      camera -> cropped 160x120 -> KerasLinear model -> DYNAMIXEL
    Pico    WT901 IMU on core1 at 100 Hz, three VL53L0X on core0, one USB frame
    U2D2    /dev/ttyUSB0, Protocol 2.0, ID 1 drives and ID 2 steers

An obstacle run goes: parking exit, model driving until the yaw target confirms
past the start lockout, a short coast, a gyro-guided finish turn, then five
ToF-guided parking stages. A free run stops after the coast.

Constants are grouped by subsystem at the top of the file. No PARKING_* value
is read by the model-driving path, so parking can be retuned without affecting
a trained model.
"""

# ---------------------------------------------------------------------------
# Standard library
# ---------------------------------------------------------------------------
import os
import sys
import time
import signal
import re
import math
import threading
import subprocess
from pathlib import Path
from typing import Tuple
from time import sleep

# Third-party
import donkeycar as dk
import numpy as np
import pygame
try:
    from sense_hat import SenseHat
except ImportError:
    SenseHat = None
try:
    import serial
except ImportError:
    serial = None
from dynamixel_sdk import *
from donkeycar.parts.transform import Lambda
from donkeycar.parts.tub_v2 import TubWriter, TubWiper
from donkeycar.parts.datastore_v2 import Catalog

try:
    from donkeycar.parts.keras import KerasInterpreter, KerasLinear
except ImportError:
    sys.exit("Keras/TensorFlow not available - install DonkeyCar with AI support.")


FREERUN_MODEL_PATH_CW    = "~/WRO_FE_2026/models/fcwm/fcwm001-006/mypilot.h5"
FREERUN_MODEL_PATH_CCW   = "~/WRO_FE_2026/models/fccwm/fccwm001-006/mypilot.h5"
OBSTACLE_MODEL_PATH_CW   = "~/WRO_FE_2026/models/ocwm/ocwm016/mypilot.h5"
OBSTACLE_MODEL_PATH_CCW  = "~/WRO_FE_2026/models/occwm/occwm001-017/mypilot.h5"


def model_path_for_drive_mode(drive_mode: str) -> str:
    """Return the configured default model path for one drive mode."""
    paths = {
        "FCW": FREERUN_MODEL_PATH_CW,
        "FCCW": FREERUN_MODEL_PATH_CCW,
        "OCW": OBSTACLE_MODEL_PATH_CW,
        "OCCW": OBSTACLE_MODEL_PATH_CCW,
    }
    try:
        return os.path.expanduser(paths[drive_mode.upper()])
    except KeyError:
        raise ValueError(f"Unknown drive mode: {drive_mode}")


def infer_drive_mode_from_path(model_path: str):
    """Best-effort mode detection for manually supplied model paths."""
    name = str(model_path).lower()
    # Check the longer names first.
    for token, mode in (
        ("occw", "OCCW"),
        ("fccw", "FCCW"),
        ("ocw", "OCW"),
        ("fcw", "FCW"),
    ):
        if token in name:
            return mode
    return None


def select_model_path_with_sensehat() -> Tuple[str, str]:
    """Ask for FCW/FCCW/OCW/OCCW only when a driving mode needs it."""
    if SenseHat is None:
        raise RuntimeError(
            "sense_hat is not installed. Use --drive-mode FCW/FCCW/OCW/OCCW "
            "or install the Sense HAT package for joystick model selection."
        )
    sense = SenseHat()
    sense.set_rotation(180)
    sense.low_light = True
    sense.clear()

    def flash(msg, seconds=0.7):
        sense.show_message(msg, scroll_speed=0.10, text_colour=[255, 255, 255])
        sleep(seconds)

    drive_mode = "FCW"
    flash(drive_mode)
    print("Driving mode selected.")
    print("Move Sense HAT joystick to choose model, press middle to confirm.")

    while True:
        for ev in sense.stick.get_events():
            if ev.action != "pressed":
                continue
            if ev.direction == "left":
                drive_mode = "FCW";  flash("FCW")
            elif ev.direction == "right":
                drive_mode = "FCCW"; flash("FCCW")
            elif ev.direction == "up":
                drive_mode = "OCW";  flash("OCW")
            elif ev.direction == "down":
                drive_mode = "OCCW"; flash("OCCW")
            elif ev.direction == "middle":
                flash(drive_mode)
                sense.clear()
                print("Selection finished.")
                break
        else:
            continue
        break

    model_path = model_path_for_drive_mode(drive_mode)
    print("Mode:", drive_mode)
    print("Model:", model_path)
    return model_path, drive_mode


# ---------------------------------------------------------------------------
# Configuration
# ---------------------------------------------------------------------------
CAPTURE_W, CAPTURE_H = (176, 132)      # camera capture
CROP_W, CROP_H       = (160, 120)      # crop sent to network / tub

DATA_PATH          = os.path.expanduser("~/WRO_FE_2026/data")

DRIVE_LOOP_HZ      = 20
JOYSTICK_DEADZONE  = 0.05
RECORD_THRESHOLD   = 0.05               # throttle magnitude to start recording
MAX_SPEED_PERCENT  = 100
STEERING_MAX_SPEED = 100
angle_offset       = 0.8

TUB_INPUTS = [
    "cam/image_array",
    "user/angle",
    "user/throttle",
    "user/mode",
]
TUB_TYPES  = ["image_array", "float", "float", "str"]


# ---------------------------------------------------------------------------
# DYNAMIXEL / XL330 motor configuration
# ---------------------------------------------------------------------------
# U2D2 port. On Raspberry Pi this is usually /dev/ttyUSB0.
# On Windows this would look like COM3, COM4, etc.
DXL_PORT = "/dev/ttyUSB0"
DXL_BAUDRATE = 57600
DXL_PROTOCOL_VERSION = 2.0

# Motor IDs. Change these if your IDs are different.
DXL_STEER_ID = 2
DXL_THROTTLE_IDS = [1]

# If you use two drive motors, example:
# DXL_THROTTLE_IDS = [1, 3]
# DXL_THROTTLE_DIRECTIONS = [1, -1]
# Use -1 on a motor if it spins the wrong physical direction.
DXL_THROTTLE_DIRECTIONS = [1]

# Steering calibration.
# Current measured straight steering position.
# AI-DRIVING steering centre. Deliberately left at 3060 even though the parking
# stages now use 3126 + 0.7 deg (= 3134 straight), a 74-tick / 6.5 degree
# difference. Both cannot be physically straight, but this one is the value the
# obstacle/free-run MODELS were trained against: if 3060 is biased, the model
# learned to compensate for that bias, and "correcting" it here would put the
# car off by the same amount in the opposite direction on every lap.
# Re-measure and change this only alongside retraining or a full driving test.
DXL_STEER_CENTER_TICKS = 3060
DXL_STEER_LEFT_DEG = -60
DXL_STEER_RIGHT_DEG = 60
DXL_STEER_DIRECTION = 1          # change to -1 if steering is backwards
DXL_TICKS_PER_DEG = 4096 / 360

# Velocity conversion for many DYNAMIXEL X-series motors, including XL330.
DXL_VELOCITY_UNIT_RPM = 0.229
DXL_THROTTLE_MAX_RPM = 100       # full-scale rpm before MAX_SPEED_PERCENT limit
DXL_STEER_PROFILE_ACCEL = 200
DXL_THROTTLE_PROFILE_ACCEL = 200

# XL330 / X-series Protocol 2.0 control table addresses.
ADDR_OPERATING_MODE = 11
ADDR_TORQUE_ENABLE = 64
ADDR_GOAL_VELOCITY = 104
ADDR_PROFILE_ACCEL = 108
ADDR_PROFILE_VELOCITY = 112
ADDR_GOAL_POSITION = 116
ADDR_PRESENT_POSITION = 132

MODE_VELOCITY = 1
MODE_POSITION = 3
MODE_EXTENDED_POSITION = 4
TORQUE_OFF = 0
TORQUE_ON = 1


# ---------------------------------------------------------------------------
# run_for_degrees() accuracy and safety settings
# ---------------------------------------------------------------------------
RUN_FOR_DEGREES_TOLERANCE_TICKS = 15
RUN_FOR_DEGREES_POLL_SECONDS = 0.02
RUN_FOR_DEGREES_TIMEOUT_MARGIN = 3.0

# Camera-view window used by recording and all camera-view driving variants.
CAMERA_VIEW_W = 240
CAMERA_VIEW_H = 160

# Gyro stop lockout.
# The timer does NOT stop the model. It only prevents the gyro stop from being
# accepted too early. Time starts after any blocking parking-exit/startup
# program, when the AI model actually begins controlling the car.
GYRO_IGNORE_STOP_UNTIL_SECONDS = 32.0

# ---------------------------------------------------------------------------
# What changing MAX_SPEED_PERCENT does, and does not, do to the gyro
# ---------------------------------------------------------------------------
# Short answer: NO gyro equation changes when you change speed.
#
# Everything the gyro does is measured in DEGREES, and degrees of rotation do
# not depend on how fast the car drives. A lap is 360 degrees at 55% throttle
# and 360 degrees at 100% throttle. So none of these need touching:
#
#   GYRO_TARGET_DEG                 degrees - speed independent
#   GYRO_THREAD_DEG_MULTIPLIER      a yaw scale factor - speed independent
#   GYRO_FINAL_TURN_MULTIPLIER      same
#   GYRO_OBSTACLE_FINISH_OFFSET_DEG degrees - speed independent
#   the unwrap / integration math   see the headroom note below
#
# GYRO_OBSTACLE_FINISH_THROTTLE is also unaffected, which is easy to get wrong.
# It goes through set_throttle_velocity_normalized(), which computes
# DXL_THROTTLE_MAX_RPM * throttle and never reads MAX_SPEED_PERCENT. The finish
# turn therefore runs at the same physical speed no matter what you set here,
# so its geometry does not change. Same for every PARKING_* speed.
#
# What DOES scale with speed is anything measured in SECONDS, because a faster
# car covers the same ground in less time. There are exactly two:
#
#   GYRO_IGNORE_STOP_UNTIL_SECONDS  the lockout below
#   OBSTACLE_RUN_FINISH_SECONDS     extra driving after the target confirms
#
# The lockout is the one that actually bites. It blocks the gyro stop from
# being accepted before N seconds. If the car reaches 360 degrees BEFORE the
# lockout expires, the stop is ignored and the car keeps driving past target.
#
# MEASURED LOCKOUT VALUES, both hand-tuned on the real track:
#
#     MAX_SPEED_PERCENT =  90   ->  34.0 s
#     MAX_SPEED_PERCENT = 100   ->  32.0 s   <- the value set above
#
# Those two points are worth keeping, because they show that naive inverse
# scaling is WRONG here. Scaling 34 s by 90/100 predicts 30.6 s, but the real
# tuned value at 100% is 32.0 s. An 11% speed increase only bought a 6% time
# reduction, so lap time is not simply inversely proportional to
# MAX_SPEED_PERCENT - throttle is not pinned at 1.0 the whole lap, and
# acceleration and cornering do not scale with it either.
#
# So AUTO-SCALING IS OFF. The constant above is used exactly as written, which
# means it is whatever you last tuned by hand. That is the honest behavior when
# the numbers come from the track rather than from a formula.
#
# If you change MAX_SPEED_PERCENT again, re-time the lockout on the track and
# write the new pair into the table above. Turning auto-scale on gives a
# first guess only, and by the evidence above it will guess SHORT. Short is
# the safe direction - the lockout only suppresses an early stop, and the yaw
# target still has to be reached and confirmed independently - but it is a
# starting point for tuning, not a substitute for it.
GYRO_LOCKOUT_AUTO_SCALE_WITH_SPEED = False
GYRO_LOCKOUT_REFERENCE_SPEED_PERCENT = 100.0


def gyro_lockout_seconds():
    """GYRO_IGNORE_STOP_UNTIL_SECONDS adjusted for the current MAX_SPEED_PERCENT."""
    if not GYRO_LOCKOUT_AUTO_SCALE_WITH_SPEED:
        return float(GYRO_IGNORE_STOP_UNTIL_SECONDS)
    reference = max(1.0, float(GYRO_LOCKOUT_REFERENCE_SPEED_PERCENT))
    current = max(1.0, float(MAX_SPEED_PERCENT))
    return float(GYRO_IGNORE_STOP_UNTIL_SECONDS) * (reference / current)


def obstacle_finish_seconds_scaled():
    """OBSTACLE_RUN_FINISH_SECONDS adjusted so the car coasts the same DISTANCE.

    This is post-target travel, so at a higher speed the same number of seconds
    carries the car further. Scaling keeps the stopping point where it was.
    """
    if not GYRO_LOCKOUT_AUTO_SCALE_WITH_SPEED:
        return float(OBSTACLE_RUN_FINISH_SECONDS)
    reference = max(1.0, float(GYRO_LOCKOUT_REFERENCE_SPEED_PERCENT))
    current = max(1.0, float(MAX_SPEED_PERCENT))
    return float(OBSTACLE_RUN_FINISH_SECONDS) * (reference / current)

# WT901 yaw target used to end the AI/model section after the lockout expires.
# WT901 angle frames already report absolute yaw, so an earlier revision defaults to a 1.00
# multiplier. If a full physical lap does not show close to 360 degrees,
# tune GYRO_THREAD_DEG_MULTIPLIER slightly.
GYRO_TARGET_DEG = 360.0

# Main WT901 yaw multiplier.
#
# Set to 0.60 because the Pico's Angle field over-reads rotation by about
# 1.667x: a physical lap of 360 degrees was being reported as ~600.
#
# Evidence, in the order it became solid:
#   1. Observed directly: the car finishes a lap pointing the way it started,
#      so its TRUE net rotation is 360 degrees, while the gyro reported ~600.
#      Net rotation depends only on start and end heading, so the wandering on
#      the straights cancels and cannot account for the excess.
#   2. The Pi cannot manufacture it. raw_total_rotation_deg has exactly one
#      write site and every branch advances _last_wt901_yaw_deg, so the sum
#      telescopes to (Angle_end - Angle_start) minus what the deadband and the
#      max-rate rejection discarded. Both discard paths only ever REMOVE
#      rotation, and the deadband is hard-bounded by
#      GYRO_RATE_DEADBAND_DEG_PER_SEC * lap_seconds = 1.5 * 32 = 48 deg.
#      A 240 degree excess therefore has to exist in the Pico's Angle field
#      before manage ever sees it.
#   3. It is long-standing, not new. The project notes record that "the robot
#      sometimes stopped around the second lap even when the target was meant
#      for three laps". A 1.667x over-read against the old 1080 target stops
#      at 1.82 laps - exactly that symptom. Earlier runs looked correct only
#      because the lap count was assumed from the target rather than observed.
#
# Ruled out along the way: constant bias (would need 7.5 deg/s, visible on a
# stationary car), the rate deadband (only ~6% of logged samples fall below
# it), and speed dependence (a 90% run over-reads just the same).
#
# PROVISIONAL - THIS MAY BE COMPENSATING FOR A HARDWARE FAULT.
#
# The over-read appeared suddenly, which a fixed sensor scale error does not.
# The leading explanation is a DEAD ToF SENSOR:
#
#   Pico main.py does three blocking VL53L0X reads per loop at ~42 ms each.
#   If one sensor fails it stops blocking, so the loop speeds up from ~126 ms
#   to ~84 ms - 1.5x faster. If the WT901 integration uses a nominal dt rather
#   than measured elapsed time, the yaw integral scales up by exactly that
#   ratio. Measured loop rate was 6.8 Hz; a 1.667x over-read needs 11.3 Hz;
#   losing one ToF predicts 11.9 Hz. That is 95% agreement, and it explains
#   the ToF dropouts and the gyro over-read as ONE fault.
#
# CHECK BEFORE TRUSTING THIS VALUE:
#     python3 pico_tool.py --verify        # prints live frames/sec
#   ~6.8 Hz  -> loop unchanged, the scale error is real, keep 0.60
#   ~12  Hz  -> a ToF has died. FIX THE SENSOR and set this back to 1.00,
#               or the finish turn will under-rotate by 40%.
#
# Refine the exact figure with:  python3 pico_tool.py --gyro
GYRO_THREAD_DEG_MULTIPLIER = 0.60

# Require the gyro target to remain reached briefly before finish starts.
# This avoids one noisy sample ending the run.
GYRO_TARGET_CONFIRM_SECONDS = 0.20
GYRO_TARGET_CONFIRM_MARGIN_DEG = 0.0

FREE_RUN_FINISH_SECONDS = 2.0
# This is a DISTANCE expressed as a time: the car keeps driving at model speed
# for this long after the yaw target confirms, so the same number of seconds
# carries it further at a higher MAX_SPEED_PERCENT. Like the lockout above it
# is hand-tuned rather than computed - 1.0 s is the value for 100%.
OBSTACLE_RUN_FINISH_SECONDS = 1.0

# When True, OCW/OCCW still run the existing gyro-based finish correction before
# parking. If the gyro is too unreliable, set this False to skip that correction
# and go straight into the ToF parking sequence after the gyro target stop.
RUN_GYRO_FINISH_TURN_AFTER_GYRO_STOP = True

# Pico-stream WT901 setup.
# The WT901 is wired to the Raspberry Pi Pico. The Pi reads the Pico USB
# serial stream, not the WT901 UART directly. The Pico line should include
# an angle field such as:
#     Angle=12.34, A=50, B=51, C=100, D=75
PICO_GYRO_PORT = "/dev/ttyACM0"
PICO_GYRO_BAUD = 115200
PICO_GYRO_SERIAL_TIMEOUT = 0.01

# an earlier revision: Pico main.py needs roughly 1.6-1.7 s after a soft reboot before the
# first Angle frame appears: 300 ms XSHUT settle plus about 430 ms for each of
# the three VL53L0X sensors. An earlier revision value of 1.20 s guaranteed a timeout, and
# every timeout sent another Ctrl-C that aborted the Pico mid-initialization.
PICO_GYRO_STARTUP_TIMEOUT_SECONDS = 8.00

# an earlier revision: at OUTPUT_HZ = 20 with three blocking ToF reads per Pico loop, a single
# sensor timeout can open a gap longer than the old 0.60 s and cause a
# spurious reopen in the middle of a run.
PICO_GYRO_STALE_TIMEOUT_SECONDS = 1.50

# an earlier revision: the reader blocks here after opening/rebooting until a real Angle frame
# is parsed, so the model never starts against a dead gyro stream.
PICO_GYRO_FIRST_FRAME_TIMEOUT_SECONDS = 10.00
PICO_GYRO_REQUIRE_FIRST_FRAME = False

# Gyro debug printing. These values only affect terminal output, not control.
# The normal status print still runs at GYRO_STATUS_PRINT_SECONDS.
PICO_GYRO_PRINT_LATEST_LINE = False
PICO_GYRO_PRINT_LINE_MAX_CHARS = 140

# Pico restart settings.
# An earlier revision does NOT soft-reboot the Pico at the very beginning of manage, because
# that can leave the Pico printing for several seconds before manage is ready
# to drain the stream. Instead, the gyro reader opens /dev/ttyACM0, keeps it
# open, sends Ctrl-C + Ctrl-D, and immediately starts draining/parsing lines.
#
# An earlier revision note: Pico main.py autostarts on power-up and streams forever, so a
# reboot is only needed to escape a stale REPL, and recovery handles that case
# on demand. Set PICO_REBOOT_WHEN_GYRO_READER_OPENS = True only if you must
# force a clean Pico restart at the top of every run.
PICO_RESTART_MAIN_ON_RUN = False

# an earlier revision: the proactive reboot is OFF by default. Probe testing showed the Pico
# streams Angle frames continuously on its own, and that writing Ctrl-C/Ctrl-D
# to a *running* script is what broke it: MicroPython buffers \x04 as ordinary
# stdin data, which then blocks the Pico's readline() and freezes its output
# loop. Listening passively avoids the hazard entirely and picks up a frame in
# well under a second.
#
# Recovery still reboots when it is actually needed. If no Angle frame arrives,
# _reopen_pico_stream_after_no_input() reopens with soft_reboot=True for the
# first PICO_AUTO_SOFT_REBOOT_MAX_ATTEMPTS tries, which rescues a Pico that is
# genuinely sitting at a REPL prompt after Thonny.
PICO_REBOOT_WHEN_GYRO_READER_OPENS = False
PICO_RESTART_PORT = PICO_GYRO_PORT
PICO_RESTART_BAUD = PICO_GYRO_BAUD
PICO_RESTART_OPEN_WAIT_SECONDS = 0.10
PICO_RESTART_CTRL_C_COUNT = 2
PICO_RESTART_BETWEEN_CTRL_C_SECONDS = 0.03
PICO_RESTART_AFTER_CTRL_C_SECONDS = 0.08
PICO_RESTART_AFTER_CTRL_D_SECONDS = 0.50

# ---------------------------------------------------------------------------
# MicroPython RAW REPL escape (Ctrl-B)
# ---------------------------------------------------------------------------
# This is the actual cause of the recurring "gyro stopped working" failures.
#
# A run captured this from the Pico itself:
#     Pico WT901 WARNING: no Angle frame ... bytes=49 lines=3 parsed=0 unparsed=3
#     Last unparsed Pico line: raw REPL; CTRL-B to exit
#
# "raw REPL; CTRL-B to exit" is MicroPython's RAW REPL banner. The Pico enters
# raw REPL when something sends Ctrl-A (\x01) - Thonny, mpremote, ampy, rshell
# and pyboard.py all do this to upload files. If one of those disconnects
# without sending Ctrl-B, the Pico is LEFT in raw REPL and main.py is not
# running. In raw REPL the Pico is also silent until it is written to, which is
# why the very first probe reported bytes=0.
#
# The critical detail: Ctrl-D means DIFFERENT things in the two REPL modes.
#
#     friendly REPL : Ctrl-D = soft reboot -> boot.py/main.py run   (what we want)
#     raw REPL      : Ctrl-D = "end of input, execute the buffer"   (does nothing)
#
# So an earlier revision recovery handshake of Ctrl-C + Ctrl-D can NEVER escape raw
# REPL. It just executes an empty block, the Pico reprints "raw REPL; CTRL-B to
# exit", and the cycle repeats until the attempt budget runs out. That matches
# the captured log exactly, and it also explains why the old manual test worked:
# back then the Pico was in the FRIENDLY repl, where Ctrl-D does reboot.
#
# The fix is to send Ctrl-B before Ctrl-D, exactly as the Pico's own message
# instructs. Ctrl-B is harmless in friendly REPL (it just reprints the banner),
# so it is safe to send unconditionally as part of the escape sequence.
PICO_RESTART_SEND_CTRL_B = True
PICO_RESTART_AFTER_CTRL_B_SECONDS = 0.15

# Passive-listen-then-escape on open.
#
# An earlier revision correctly established that writing Ctrl-C/Ctrl-D to a *running* main.py
# breaks it, because MicroPython buffers \x04 as ordinary stdin data. An earlier revision's
# answer was to never write on open, which left the only escape path in the
# recovery loop - and that path took 10 s per attempt and could not escape raw
# REPL anyway. The run log shows 35 s spent driving blind before giving up.
#
# This program keeps an earlier revision\'s insight but acts on it much faster: listen silently first,
# and only write if the Pico is provably NOT streaming. After this much silence
# there is no live print loop left to disturb, so the write is safe.
PICO_GYRO_PASSIVE_LISTEN_SECONDS = 2.00
PICO_GYRO_ESCAPE_ATTEMPTS_ON_OPEN = 2
PICO_GYRO_WAIT_AFTER_ESCAPE_SECONDS = 3.00

# an earlier revision: recognize the raw REPL banner so the log says what is wrong in plain
# words instead of only showing it as an unparsed line.
PICO_RAW_REPL_MARKERS = ("raw REPL", "CTRL-B to exit")

# Pico WT901 gyro-reset command settings. Older versions sent these after the Pico
# restart so your Pico main.py can zero/reset the WT901 before the run.
# If your Pico main.py only supports one command, it can ignore the other.
PICO_GYRO_RESET_ON_RUN = False
PICO_GYRO_RESET_COMMANDS = ("RESET_GYRO", "ZERO_GYRO")
PICO_GYRO_RESET_WAIT_SECONDS = 0.15

# If the Pi cannot parse any Pico input, an earlier revision first closes/reopens the serial port.
# The newly opened gyro reader can soft-reboot Pico main.py while keeping the
# port open and draining the stream. It still does NOT send gyro-zero commands.
PICO_AUTO_REOPEN_ON_NO_INPUT = True
PICO_AUTO_REOPEN_MAX_ATTEMPTS = 3

# an earlier revision: must exceed PICO_GYRO_STARTUP_TIMEOUT_SECONDS plus Pico boot time, or a
# retry can fire while the Pico is still initializing from the previous retry.
PICO_AUTO_REOPEN_COOLDOWN_SECONDS = 6.00
PICO_AUTO_SOFT_REBOOT_ON_NO_INPUT = True
PICO_AUTO_SOFT_REBOOT_MAX_ATTEMPTS = 2

# Serial-port blocker cleanup. This runs before manage opens the Pico.
# It is meant to clear Thonny/serial-debug/old-manage processes that are
# already holding /dev/ttyACM0. It will not kill this current manage process.
PICO_AUTO_FREE_SERIAL_PORT_ON_RUN = True
PICO_SERIAL_PORTS_TO_FREE = (PICO_GYRO_PORT,)
PICO_SERIAL_KILL_GRACE_SECONDS = 0.35
PICO_SERIAL_SAFE_KILL_KEYWORDS = (
    "thonny",
    "pico_serial_debug.py",
    "manage26V",
    "mpremote",
    "rshell",
    "ampy",
    "screen",
    "minicom",
    "picocom",
    "putty",
    # an earlier revision: the Pi-side helper tools. pico_tool.py --monitor is meant to be left
    # running while testing, and without these entries manage would find the
    # port held, refuse to kill an "unknown" process, and fail to get the gyro.
    "pico_tool.py",
    "read_pico_serial",
)

# ---------------------------------------------------------------------------
# Automatic pico_tool --fix equivalent, run at manage startup
# ---------------------------------------------------------------------------
# The gyro reader already escapes a stale REPL when it opens. Doing it once more
# at the very top of manage is still worth it:
#
#   * It runs BEFORE the camera and the TensorFlow model load. If the Pico needs
#     a reboot, its 1.6-1.7 s of ToF initialization then overlaps with the model
#     load instead of adding to the startup time.
#   * You find out the Pico is broken immediately, instead of after waiting for
#     the model to load.
#   * By the time PicoWT901YawReader opens the port, the Pico is already
#     streaming, so its passive listen succeeds in well under a second.
#
# The in-reader handshake stays as the second line of defense, and costs almost
# nothing when the Pico is healthy.
PICO_AUTO_FIX_ON_RUN = True
PICO_AUTO_FIX_MAX_ATTEMPTS = 3
PICO_AUTO_FIX_VERIFY_SECONDS = 6.00

GYRO_RATE_DEADBAND_DEG_PER_SEC = 1.5
GYRO_MAX_VALID_RATE_DEG_PER_SEC = 500.0

# The WT901 is sampled independently from DRIVE_LOOP_HZ. A 100 Hz sampling
# thread captures yaw changes more consistently than the 20 Hz model loop.
GYRO_SAMPLE_HZ = 100.0
GYRO_SAMPLE_SECONDS = 1.0 / GYRO_SAMPLE_HZ
GYRO_STATUS_PRINT_SECONDS = 1.00

# Obstacle-only finish correction. After the gyro target and finish delay, the
# code drives forward while steering until raw gyro yaw reaches
# (GYRO_TARGET_DEG - 90). OCW turns left; OCCW turns right.
# Version 21 removes the backward re-align phase.
GYRO_OBSTACLE_FINISH_OFFSET_DEG = 90.0
GYRO_OBSTACLE_FINISH_STEER_DEG = 50.0
GYRO_OBSTACLE_FINISH_THROTTLE = 0.35
GYRO_OBSTACLE_FINISH_TOLERANCE_DEG = 2.0

# Final correction tuning.
# Main lap stopping uses GYRO_THREAD_DEG_MULTIPLIER above. The forward-only
# finish correction uses its own multiplier so one good lap calibration does
# not force the final 270-degree correction to stop early or late.
# Start at 1.00 for WT901 yaw frames.
# If the final correction is still SHORT, lower this number.
# If the final correction OVERSHOOTS, raise this number.
# Must match GYRO_THREAD_DEG_MULTIPLIER whenever the SENSOR itself is
# the thing being corrected, because both paths read the same over-reading
# Angle field - they are separate constants only so a lap calibration and a
# finish-turn calibration can differ, not so a sensor scale error can be
# applied to one and not the other.
# Setting the lap multiplier alone leaves the finish turn rotating 54 degrees
# where 90 was requested, a 40% shortfall (verified by tracing both paths).
GYRO_FINAL_TURN_MULTIPLIER = 0.60
GYRO_OBSTACLE_FINISH_MAX_TURN_DEG = 140.0
GYRO_OBSTACLE_FINISH_TIMEOUT_SECONDS = 15.0


# ---------------------------------------------------------------------------
# Parking integration settings
# ---------------------------------------------------------------------------
# Integrated parking can run after either obstacle direction.
# The pasted sequence is CW/OCW by default; an earlier revision mirrors the reverse turn before
# parking for OCCW by steering the opposite direction.
PARKING_ENABLED = True
PARKING_ENABLED_MODES = {"OCW", "OCCW"}

# The standalone parking program stages were:
#   1) full-left forward arc
#   2) C ToF front-wall approach
#   3) reverse-right arc
#   3B) reverse A/B balance
#   4) PID ToF wall follow until parking wall
#   5) parking entry
# This integrated version starts at stage 2 as requested.
PARKING_SKIP_TO_C_TOF_STAGE = True

# Parking-only DYNAMIXEL behavior. These values are only used by the parking
# functions below and do not change normal AI model steering/throttle settings.
# an earlier revision: 3136 -> 3126 and 0.8 -> 0.7, which is exactly the steering calibration
# used by parkingv35 / parkingv38 (DXL_CENTER_TICKS=3126, ANGLE_OFFSET_DEG=0.7).
# Those standalone programs are the ones that parked reliably, so their numbers
# are the trustworthy ones for the parking stages.
#
# What this actually moves:
#     old parking straight : 3136 + 0.8 deg = 3145 ticks
#     new parking straight : 3126 + 0.7 deg = 3134 ticks
#     difference           : 11 ticks = 0.97 deg to the LEFT
#
# Note the coincidence that explains the symptom: under the OLD calibration a
# command of -1.0 deg landed on 3134 ticks - precisely the standalone programs'
# straight-ahead. So "I set -1 and it drove roughly straight, maybe drifting
# right" is exactly what a 1 degree rightward centre error looks like.
#
# Does NOT touch DXL_STEER_CENTER_TICKS (3060), which the trained model drives
# through. See the note there.
PARKING_STEERING_CENTER_TICKS = 3126
PARKING_STEERING_OFFSET_DEG = 0.7
PARKING_STEERING_DIRECTION = 1
PARKING_MAX_THROTTLE_RAW = 120
PARKING_STEERING_PROFILE_VELOCITY = 180

# Pico / ToF serial settings.
PARKING_PICO_PORT = "/dev/ttyACM0"
PARKING_PICO_BAUD = 115200
PARKING_PRINT_PICO_RAW_LINE = False
PARKING_REQUIRED_SENSORS = {"A", "B", "C"}
PARKING_TOF_PRINT_INTERVAL = 0.15
PARKING_VALID_STATUS_CODES = {0, 11}
PARKING_USE_SENSOR_STATUS = True

# Parking speeds.
PARKING_APPROACH_SPEED_PERCENT = 100
# an earlier revision: lowered again, 80 -> 55. The run log measured the real control rate at
# about 6.8 ToF frames per second, not the 20 Hz assumed earlier: Pico main.py
# does three blocking VL53L0X reads at roughly 42 ms each, so ~7-8 Hz is a hard
# ceiling no software change on the Pi can lift. At 6.8 Hz the car covers a
# large distance between corrections, and distance-per-correction is what sets
# achievable tracking error. Halving speed doubles the number of corrections
# per unit of travel, which is the cheapest real improvement available.
PARKING_FOLLOW_SPEED_PERCENT = 55
PARKING_ENTRY_SPEED_PERCENT = 100
PARKING_TURN_SPEED_PERCENT = 80

# Parking turn/alignment settings.
PARKING_FULL_LEFT_STEER_DEG = -50.0
PARKING_FULL_RIGHT_STEER_DEG = 50.0
# Stage 3 reverse turn before parking. OCW uses the pasted full-right reverse
# turn. OCCW mirrors it and uses full-left reverse.
PARKING_OCW_PRE_PARK_REVERSE_STEER_DEG = PARKING_FULL_RIGHT_STEER_DEG
PARKING_OCCW_PRE_PARK_REVERSE_STEER_DEG = PARKING_FULL_LEFT_STEER_DEG
PARKING_TURN_MOTOR_DEGREES = 720.0
PARKING_STEERING_SETTLE_SECONDS = 0.45
PARKING_RUN_FOR_DEGREES_TIMEOUT = 15.0
# NOT USED. Left over from the standalone v26/v27 sensor-based second turn,
# which was replaced by the fixed motor-degree turn above. Changing it does
# nothing. Kept only so old notes and diffs still line up.
PARKING_SECOND_TURN_AB_TIMEOUT = 15.0

# ---------------------------------------------------------------------------
# Stage 3B A/B alignment settings (rework)
# ---------------------------------------------------------------------------
PARKING_AB_BALANCE_DIFF_LEEWAY_MM = 1.0
PARKING_AB_BALANCE_TIMEOUT = 15.0
PARKING_AB_BALANCE_INVALID_TIMEOUT = 15.0

# an earlier revision: PARKING_AB_BALANCE_STEER_DEG is now the MAXIMUM steering the aligner may
# use, not a fixed bang-bang value. Steering is proportional to the filtered
# |A - B| difference, so the correction shrinks as the car approaches balance.
# This is the main reason an earlier revision oscillated instead of settling.
#
#   steer_magnitude = clamp(KP_DEG_PER_MM * diff, MIN_STEER_DEG, STEER_DEG)
#
# Raise PARKING_AB_ALIGN_KP_DEG_PER_MM if alignment converges too slowly.
# Lower it if the car still swings past balanced.
PARKING_AB_BALANCE_STEER_DEG = 10.0
PARKING_AB_ALIGN_KP_DEG_PER_MM = 2.0
PARKING_AB_ALIGN_MIN_STEER_DEG = 1.5

# an earlier revision: reverse speed lowered from 70. Angular error per frame of sensor/servo
# latency scales directly with speed, so a slower creep is the single cheapest
# accuracy improvement. Raise back toward 70 only if alignment feels too slow.
PARKING_AB_BALANCE_SPEED_PERCENT = 45
# NOT USED. Forward alignment was removed from the integrated manage parking
# path; only the reverse aligner runs. Kept for the standalone v38 program.
PARKING_AB_BALANCE_FORWARD_SPEED_PERCENT = 45

# an earlier revision: two-phase alignment. While the difference is large the car uses the
# normal speed above; once it drops under the fine threshold it creeps, which
# lets it stop precisely on balance instead of coasting through it.
PARKING_AB_ALIGN_FINE_DIFF_MM = 4.0
PARKING_AB_ALIGN_FINE_SPEED_PERCENT = 25

# an earlier revision: filtering. Median-of-3 rejects the single-sample spikes a VL53L0X
# produces without the phase lag a heavy EMA adds; the EMA afterward is light.
# An earlier revision used a single alpha=0.45 EMA, which lagged about 1.2 frames and made the
# aligner react to where the car WAS rather than where it is.
PARKING_AB_BALANCE_SMOOTHING_ALPHA = 0.70
PARKING_AB_ALIGN_MEDIAN_WINDOW = 3

# an earlier revision: read timeout raised slightly. The Pico emits roughly 20 frames/s, so
# 0.10 s was close enough to one frame period that an ordinary slow ToF read
# was treated as a dropout. Dropouts no longer force the steering straight,
# but avoiding them entirely is still better.
PARKING_AB_BALANCE_READ_TIMEOUT = 0.15
PARKING_AB_BALANCE_PRINT_INTERVAL = 0.05

# an earlier revision: confirmation is no longer "2 frames under the leeway". The difference
# must also be SETTLED, meaning it is not still sweeping through zero, and the
# confirmations must span a minimum wall-clock time. An earlier revision could confirm in the
# middle of a swing, which is exactly the reported inaccuracy.
PARKING_AB_BALANCE_REQUIRED_COUNT = 3
PARKING_AB_BALANCE_REQUIRED_SECONDS = 0.15
PARKING_AB_ALIGN_SETTLED_RATE_MM_PER_SEC = 8.0

# an earlier revision: after balance is confirmed, center the steering, stop, let the car come
# to rest, then measure and print the real residual |A - B|. That number is the
# achieved alignment accuracy and is what to watch when tuning.
PARKING_AB_ALIGN_SETTLE_SECONDS = 0.30
PARKING_AB_ALIGN_REPORT_RESIDUAL = True

# What to steer when a sensor goes invalid mid-alignment.
#
# Alignment controls purely on |A - B|, so one invalid sensor leaves NO angle
# information at all - there is no correct direction to turn. An earlier revision held the last
# steering through the dropout, which is wrong here: the rotation that was in
# progress is usually what swung the sensor to the 35-45 degree angle that
# caused the dropout, so holding it keeps rotating further into the blind spot
# and the reading never comes back.
#
# parkingv35 and v38 both command steering 0.0 for the entire invalid period
# and keep reversing. That stops the blind rotation and lets the geometry
# settle, which is why those programs recovered on their own.
#
# This program keeps a short grace window so a single noisy frame does not jerk the
# steering straight mid-correction, then straightens exactly like v35.
PARKING_AB_ALIGN_INVALID_GRACE_FRAMES = 1
PARKING_AB_ALIGN_STRAIGHTEN_WHEN_INVALID = True

# If Stage 3B cannot finish in time, do not stop the run; go into PID anyway.
PARKING_AB_BALANCE_TIMEOUT_CONTINUE_TO_PID = True
PARKING_AB_BALANCE_TIMEOUT_BRAKE_SECONDS = 0.15

# Distance targets and leeways.
PARKING_FRONT_STOP_MM = 5.0
PARKING_WALL_TARGET_MM = 32.0
PARKING_PARK_TRIGGER_MM = 15.0
PARKING_C_DISTANCE_LEEWAY_MM = 0.7
# Steering used while driving toward the front wall using C ToF.
# Negative values steer left. User wants future manage versions to use -1.0.
PARKING_C_APPROACH_STEER_DEG = -1.0
# NOT USED. The alignment leeway that is actually in effect is
# PARKING_AB_BALANCE_DIFF_LEEWAY_MM in the Stage 3B section below.
PARKING_BALANCE_LEEWAY_MM = 0.5
PARKING_WALL_FIND_LEEWAY_MM = 5.0
# Stage 2 C-approach confirm count only. Stage 4 parking-wall detection now has
# its own PARKING_WALL_FIND_CONFIRM_READINGS.
PARKING_DISTANCE_CONFIRM_READINGS = 1

# ---------------------------------------------------------------------------
# Stage 4 wall-follow PID settings (retuned)
# ---------------------------------------------------------------------------
# The error equation is unchanged in shape:
#
#     error = Wa * (A - B) + Wd * ((A + B) / 2 - Td)
#     steer = K() * error
#
# but This program splits the two terms so they can be tuned independently, and drops
# the overall gain. Understanding the two terms is the whole tuning story:
#
#   (A - B) is the WALL-ANGLE term. A and B sit at different points along the
#   same side of the car, so their difference is proportional to how far the
#   car is rotated relative to the wall. This term is what DAMPS the approach.
#   It is already a derivative-style lead term, which is why KD is not the knob
#   to reach for here.
#
#   ((A + B)/2 - Td) is the CROSS-TRACK term. It is how far the car is from the
#   target distance. This is what DRIVES the car back toward the wall.
#
# A stable wall follower needs the angle term weighted more heavily than the
# offset term. An earlier revision weighted them 1:1 with K = 2.0, so a car that was parallel
# but only 15 mm off target already hit the +/-30 degree steering clamp; the
# controller was saturated bang-bang for most of the run and weaved.
#
# With an earlier revision defaults, a car parallel and 10 mm too far out steers a gentle
# 8 degrees toward the wall, and it settles at a constant approach angle
# proportional to the remaining offset instead of overshooting.
#
# Tuning order:
#   1. Car weaves / oscillates along the wall  -> lower PARKING_K() first,
#      then raise PARKING_PID_ANGLE_WEIGHT.
#   2. Car is smooth but sits at the wrong distance -> raise
#      PARKING_PID_DISTANCE_WEIGHT (or lower PARKING_PID_ANGLE_WEIGHT).
#   3. Car corrects too slowly -> raise PARKING_K().
#
# an earlier revision: raised again, 1.2 -> 1.8, because the car still visibly barely turns.
#
# The measured picture across runs: mean steering 2.2 degrees, peak 9.4, against
# a 30 degree clamp. 64% of frames commanded under 2 degrees. One to two degrees
# of steering is a very gentle curve on a car this size, which is why the log
# reads LEFT while the car looks like it is going straight.
#
# (Correction to an earlier note here: the 0.8 degree PARKING_STEERING_OFFSET_DEG
# does NOT swallow small commands. It is an additive trim that defines where
# straight is - commands map 1:1 to physical degrees on top of it. The commands
# were simply small.)
#
# Why raising K is the safe lever: the steady-state distance offset caused by
# any A/B sensor mismatch is -(Wa/Wd) * bias, which does not contain K at all.
# Raising K converges FASTER to the same equilibrium rather than shifting it,
# so this cannot make the "sits too close to the wall" problem worse.
#
# 1.8 keeps the angle:distance ratio, and the replayed peak stays near 21 of the
# 30 degree clamp, so the loop still has headroom and does not become bang-bang.
# If it now weaves, come back down toward 1.5 before changing anything else.
def PARKING_K():
    return 1.8


def PARKING_TD():
    return PARKING_WALL_TARGET_MM


# an earlier revision: lowered from the 2.0 that an earlier revision shipped with. See PARKING_AB_OFFSET_MM
# below - a high angle weight amplifies any A/B sensor mismatch straight into a
# steady-state distance error, which made the car track closer to the wall than
# PARKING_WALL_TARGET_MM. Once PARKING_AB_OFFSET_MM is measured this can go
# back up to 2.0 for more damping.
PARKING_PID_ANGLE_WEIGHT = 1.5
PARKING_PID_DISTANCE_WEIGHT = 1.0

# ---------------------------------------------------------------------------
# A/B differential offset calibration - affects BOTH parking stages
# ---------------------------------------------------------------------------
# Two VL53L0X sensors do not read identically against the same surface. A few
# mm of difference between A and B is normal unit-to-unit variation. Nothing in
# the code can tell that apart from the car being rotated, so an uncalibrated
# mismatch behaves like a permanent heading error. It breaks both stages:
#
#   Stage 3B alignment: balances until |A - B| <= 1 mm, so it physically rotates
#   the car until the READINGS match. With a 3 mm sensor mismatch it parks the
#   car crooked by whatever angle produces 3 mm of real difference.
#
#   Stage 4 PID: settles where Wa*(A-B) + Wd*(avg - Td) = 0, which means
#       avg = Td - (Wa/Wd) * (A - B)
#   so a constant (A - B) bias becomes a constant distance error, scaled by the
#   angle weight. A positive bias parks the car CLOSER to the wall than Td.
#
# HOW TO MEASURE IT:
#   1. Put the car next to the wall and set it parallel BY HAND, as accurately
#      as you can - use the chassis edge against the wall, not the sensors.
#   2. Run:  python3 pico_tool.py --calib
#   3. It averages A and B for a few seconds and prints the value to paste here.
#
# Sign convention: PARKING_AB_OFFSET_MM is what (A - B) reads when the car is
# physically parallel. The code subtracts it, so a calibrated car reads a true
#
# an earlier revision: BACK TO 0.0. An earlier revision set this to 2.1 from two run-log numbers that
# appeared to agree to 0.01 mm. That inference was wrong, for two reasons:
#
#   1. The two numbers were not independent. Both were taken with the car in a
#      pose the ALIGNER had chosen - and the aligner's whole job is to drive
#      the readings equal. Measuring sensor disagreement in a pose selected by
#      those same sensors is circular. Neither number established that the car
#      was physically parallel, which is the only thing that makes (A - B) mean
#      "sensor mismatch".
#   2. The readings themselves were suspect: an earlier revision\'s parser fix above shows
#      damaged lines were being accepted with wrong, biased-low values.
#
# The reported symptom fits: with a wrong offset the aligner squares the car to
# a pose that LOOKS crooked, which is the "it seemed balanced but kept going"
# behavior.
#
# Only one measurement is valid here - set the car parallel BY HAND against the
# chassis edge, then run:  python3 pico_tool.py --calib
# Leave this at 0.0 until that has been done.
PARKING_AB_OFFSET_MM = 0.0

# KI/KD stay at zero by default. The (A - B) angle term above already supplies
# the damping a wall follower needs, so raising PARKING_PID_ANGLE_WEIGHT is a
# better first move than raising KD. If a persistent distance offset remains
# after the run is otherwise smooth, a very small KI (0.02 - 0.08) removes it.
# This program adds conditional integration, so the integral no longer winds up while
# the steering output is clamped, and derivative filtering so a nonzero KD does
# not simply amplify VL53L0X noise.
# an earlier revision: KI turned on, small. The run log showed a steady one-way drift of
# -8.2 mm across the run (50% of frames shrinking, linear trend -0.11 mm/frame)
# rather than a weave. A proportional-only controller cannot remove a constant
# disturbance like a steering-trim error; it can only lean against it and
# settle at a permanent offset. Rejecting a constant disturbance is exactly
# what integral action is for, so this is the textbook lever for what the data
# shows - not a guess.
#
# Deliberately small: the ToF stream runs at only about 6.8 Hz, and integral
# action on a slow loop is what causes integral-induced oscillation. If the car
# starts weaving, halve this FIRST, before touching PARKING_K(). If it still
# drifts in, raise it toward 0.20.
# Conditional integration (an earlier revision) already stops windup while the steering is
# clamped, so the integrator cannot charge up against the limit.
# an earlier revision: BACK TO 0.0. An earlier revision reasoning (integral action rejects a constant
# disturbance) is still correct in principle, but it was applied to a drift
# measured from readings the parser was corrupting. Worse, an integrator on top
# of a biased-low distance accumulates a one-way error and pins the steering
# hard over - which is exactly the reported "it just turned straight left".
# Do not re-enable this until a run shows clean readings and STILL drifts.
PARKING_PID_KI = 0.0
PARKING_PID_KD = 0.0
PARKING_PID_INTEGRAL_LIMIT = 50.0
PARKING_PID_DERIVATIVE_FILTER_ALPHA = 0.30
PARKING_PID_STEERING_DEADBAND_DEG = 0.2
PARKING_WALL_STEERING_SIGN = 1
PARKING_PID_STEERING_DIRECTION = PARKING_WALL_STEERING_SIGN
PARKING_ENABLE_PID_AUTO_BREAK = False
PARKING_PID_BREAK_ERROR_TOLERANCE = 1.0
PARKING_PID_BREAK_REQUIRED_COUNT = 1
# NOT USED. The wall-follow gain that is actually in effect is PARKING_K()
# above. PARKING_WALL_KP is a leftover from an older wall follower; setting it
# has no effect, so do not tune it.
PARKING_WALL_KP = 1
PARKING_MAX_WALL_STEER_DEG = 30.0

# an earlier revision: filter A and B before they reach the PID. An earlier revision fed raw readings into a
# 2.0 deg/mm gain, so a few mm of VL53L0X noise became several degrees of
# steering jitter every frame. Median-of-3 then a light EMA.
PARKING_PID_MEDIAN_WINDOW = 3
PARKING_PID_SMOOTHING_ALPHA = 0.65

# an earlier revision: hold the last valid reading through a short dropout instead of
# substituting PARKING_TD(). Substituting the target makes (A - B) jump by the
# full offset for a single bad frame, which injects a large fake angle error.
# After this window expires the old Td substitution is used as before.
PARKING_PID_INVALID_HOLD_SECONDS = 0.30

# ---------------------------------------------------------------------------
# Graceful degradation when ONE ToF sensor is invalid
# ---------------------------------------------------------------------------
# A VL53L0X returns nothing when it sits 35-45 degrees off the black wall, so
# single-sensor dropouts are normal here, not exceptional. The wall-follow
# equation uses A and B jointly:
#
#     error = Wa*(A - B) + Wd*((A + B)/2 - Td)
#
# so ANY fixed substitution for the missing sensor invents an angle that the
# car is not actually at. Substituting Td is the worst case: if B drops out
# while A reads 40 and Td is 32, the angle term becomes 40 - 32 = +8 mm of
# pure fiction, and the controller swerves to correct a rotation that does not
# exist. Substituting a fixed "max" like 55 has the same defect, just larger -
# which is why that workaround was never satisfying.
#
# This program estimates the missing sensor instead of substituting a constant. The
# valid sensor still reports the distance correctly; only the ANGLE becomes
# unmeasurable. So the angle is held at its last known-good value and the
# missing reading is reconstructed from it:
#
#     B invalid -> B_est = A - last_good_angle
#     A invalid -> A_est = B + last_good_angle
#
# The angle term then stays exactly at the last real measurement instead of
# jumping, while the distance term keeps tracking the sensor that still works.
# No invented geometry, and no magic constant.
PARKING_PID_DEGRADED_HOLD_ANGLE = True

# The held angle gets less trustworthy the longer the dropout lasts, so it
# decays toward zero each degraded frame. A long dropout therefore settles into
# pure distance-following rather than steering on a stale angle. At about 7 ToF
# frames per second, 0.85 fades a held angle to roughly half in 0.6 s.
PARKING_PID_DEGRADED_ANGLE_DECAY = 0.85

# Reduce authority while running on a reconstructed reading. Confidence is
# lower, so the controller should be correspondingly less aggressive.
PARKING_PID_DEGRADED_GAIN_SCALE = 0.6

# Tighter steering clamp while degraded. parkingv35 - which self-corrected
# through invalid readings - clamped ALL wall-follow steering to 10 degrees.
# This program allows 30, so a bad frame can command three times the correction that
# v35 could. Rather than throttle the healthy case, This program only tightens the
# clamp for frames that are actually running on estimated data.
PARKING_PID_DEGRADED_MAX_STEER_DEG = 12.0

# an earlier revision: limit how fast the steering command may change. The servo cannot follow
# a step anyway, and rate limiting stops the controller from chasing noise.
# Set very high (e.g. 10000.0) to disable.
PARKING_PID_MAX_STEER_RATE_DEG_PER_SEC = 150.0

# an earlier revision: parking-wall detection gets its own confirm count, separate from the
# Stage 2 C approach which still uses PARKING_DISTANCE_CONFIRM_READINGS.
# An earlier revision stopped the run on ONE frame of A <= threshold, so a single spurious
# short VL53L0X reading parked the car in the wrong place. Detection still uses
# the RAW (unfiltered) A value so there is no filter lag; requiring 2 in a row
# is a debounce, and costs about one Pico frame of extra travel.
PARKING_WALL_FIND_CONFIRM_READINGS = 2

# an earlier revision: the parking bay must look like a STEP, not just a small A.
#
# "A is close" cannot by itself tell "found the parking bay" apart from "drifted
# into the wall I was following" - in both cases A is small. The run log came
# within 1 mm of that mistake: frames 190-193 sat at A ~ 21 mm with B ~ 21 mm,
# no step between them, purely because the car had drifted 8 mm inward. The
# real trigger a moment later had A = 16.8 with B ~ 21, a genuine 4.2 mm step.
#
# Requiring B - A >= this margin accepts the real bay and rejects pure drift.
# Set PARKING_WALL_FIND_REQUIRE_STEP = False for the old A-only behavior.
PARKING_WALL_FIND_REQUIRE_STEP = True
PARKING_WALL_FIND_MIN_STEP_MM = 2.5

# an earlier revision: print the mean/worst tracking error when Stage 4 ends. This is the
# number that quantifies "the PID is inaccurate", so it is worth seeing.
PARKING_PID_REPORT_TRACKING_ERROR = True

# Parking safety timeouts.
PARKING_APPROACH_TIMEOUT = 12.0
PARKING_FOLLOW_TIMEOUT = 30.0
PARKING_SENSOR_INVALID_TIMEOUT = 0.8


def _process_exists(pid: int) -> bool:
    """Return True if a process still exists."""
    if pid <= 0:
        return False
    try:
        os.kill(pid, 0)
        return True
    except ProcessLookupError:
        return False
    except PermissionError:
        return True


def _process_cmdline(pid: int) -> str:
    """Read a Linux process command line for serial-port cleanup messages."""
    try:
        raw = Path(f"/proc/{int(pid)}/cmdline").read_bytes()
    except Exception:
        return ""
    parts = [part.decode("utf-8", errors="replace") for part in raw.split(b"\0") if part]
    return " ".join(parts)


def _pids_using_serial_port(port: str):
    """Return PIDs using a serial port using fuser, or an empty list."""
    if not port or not os.path.exists(port):
        return []
    try:
        result = subprocess.run(
            ["fuser", str(port)],
            capture_output=True,
            text=True,
            timeout=1.0,
        )
    except Exception as error:
        print(f"Serial cleanup warning: could not run fuser for {port}: {error}", flush=True)
        return []

    # Bare fuser prints only the PID list to stdout. Avoid parsing stderr because
    # verbose/status text can contain numbers from /dev/ttyACM0.
    return [int(pid_text) for pid_text in re.findall(r"\d+", result.stdout or "")]


def free_pico_serial_port(reason="program startup"):
    """Free Pico serial ports from known blockers before manage starts.

    This does NOT restart the Pico and does NOT zero the gyro. It only closes
    other Linux processes that already have the Pico USB serial port open.
    """
    if not PICO_AUTO_FREE_SERIAL_PORT_ON_RUN:
        print("Pico serial auto-free disabled; skipping serial cleanup.", flush=True)
        return False

    current_pid = os.getpid()
    freed_any = False
    checked_any = False

    for port in PICO_SERIAL_PORTS_TO_FREE:
        if not port:
            continue
        checked_any = True
        pids = _pids_using_serial_port(port)
        if not pids:
            print(f"Pico serial cleanup: {port} is free.", flush=True)
            continue

        print(f"Pico serial cleanup: {port} is being used by PID(s) {pids}.", flush=True)
        for pid in pids:
            if int(pid) == current_pid:
                print(
                    f"Pico serial cleanup: PID {pid} is this manage process; not killing it.",
                    flush=True,
                )
                continue

            cmdline = _process_cmdline(pid)
            cmd_lower = cmdline.lower()
            safe_to_kill = any(keyword.lower() in cmd_lower for keyword in PICO_SERIAL_SAFE_KILL_KEYWORDS)

            if not safe_to_kill:
                print(
                    f"Pico serial cleanup: NOT killing PID {pid}; unknown command: {cmdline or '(unknown)'}",
                    flush=True,
                )
                print(
                    "Close that program manually, or add a safe keyword if it is a known serial viewer.",
                    flush=True,
                )
                continue

            print(
                f"Pico serial cleanup: closing PID {pid}: {cmdline or '(unknown command)'}",
                flush=True,
            )
            try:
                os.kill(pid, signal.SIGTERM)
                deadline = time.monotonic() + PICO_SERIAL_KILL_GRACE_SECONDS
                while time.monotonic() < deadline and _process_exists(pid):
                    time.sleep(0.03)
                if _process_exists(pid):
                    print(f"Pico serial cleanup: PID {pid} did not exit; force killing.", flush=True)
                    os.kill(pid, signal.SIGKILL)
                freed_any = True
            except PermissionError:
                print(
                    f"Pico serial cleanup: permission denied killing PID {pid}. "
                    f"Try: sudo kill {pid}",
                    flush=True,
                )
            except ProcessLookupError:
                freed_any = True
            except Exception as error:
                print(f"Pico serial cleanup warning for PID {pid}: {error}", flush=True)

        remaining = [pid for pid in _pids_using_serial_port(port) if pid != current_pid]
        if remaining:
            print(f"Pico serial cleanup: {port} still used by PID(s) {remaining}.", flush=True)
        else:
            print(f"Pico serial cleanup: {port} is free after cleanup.", flush=True)

    if not checked_any:
        print("Pico serial cleanup: no ports configured.", flush=True)
    return freed_any


# ---------------------------------------------------------------------------
# Shared Pico REPL-escape helpers
# ---------------------------------------------------------------------------
# One definition of the escape sequence and the Angle parser, used by both the
# startup auto-fix below and PicoWT901YawReader, so the two can never drift
# apart. Both patterns are also the class attributes on the reader.
PICO_ANGLE_RE = re.compile(
    r"\b(?:Angle|ANGLE|Yaw|YAW)\s*[:=]\s*(-?\d+(?:\.\d+)?)"
)
PICO_FIRST_VALUE_RE = re.compile(r"^\s*(-?\d+(?:\.\d+)?)\s*(?:,|\s)")


def pico_send_escape_sequence(pico_serial):
    """Send Ctrl-C, Ctrl-C, Ctrl-B, Ctrl-D on an already-open Pico port.

    Ctrl-B is the step that matters and the one every version before an earlier revision was
    missing. MicroPython's Ctrl-D is mode-dependent:

        friendly REPL : Ctrl-D = soft reboot -> boot.py / main.py run
        raw REPL      : Ctrl-D = execute the buffer -> nothing happens

    Thonny, mpremote, ampy and rshell all put the Pico into raw REPL to copy
    files, so a Pico left behind by any of them can only be rescued by sending
    Ctrl-B first to return to the friendly REPL.
    """
    try:
        pico_serial.reset_input_buffer()
    except Exception:
        pass
    try:
        pico_serial.reset_output_buffer()
    except Exception:
        pass

    for _ in range(max(1, int(PICO_RESTART_CTRL_C_COUNT))):
        pico_serial.write(b"\x03")
        pico_serial.flush()
        time.sleep(PICO_RESTART_BETWEEN_CTRL_C_SECONDS)
    time.sleep(PICO_RESTART_AFTER_CTRL_C_SECONDS)

    if PICO_RESTART_SEND_CTRL_B:
        pico_serial.write(b"\x02")
        pico_serial.flush()
        time.sleep(PICO_RESTART_AFTER_CTRL_B_SECONDS)

    pico_serial.write(b"\x04")
    pico_serial.flush()
    time.sleep(PICO_RESTART_AFTER_CTRL_D_SECONDS)


def _pico_listen_for_angle_frame(pico_serial, seconds, state):
    """Drain the port for up to `seconds`, looking for one parsable Angle line.

    `state` is a dict carrying counters across calls so the failure message can
    say which layer broke. Returns the matching line, or None.
    """
    deadline = time.monotonic() + float(seconds)
    while time.monotonic() < deadline:
        try:
            chunk = pico_serial.read(256)
        except Exception as error:
            state["error"] = str(error)
            return None
        if chunk:
            state["bytes"] += len(chunk)
            state["buffer"] += chunk.decode("utf-8", errors="ignore")

        while "\n" in state["buffer"]:
            line, state["buffer"] = state["buffer"].split("\n", 1)
            line = line.strip()
            if not line:
                continue
            state["lines"] += 1
            if any(marker in line for marker in PICO_RAW_REPL_MARKERS):
                state["raw_repl"] = True
            match = PICO_ANGLE_RE.search(line) or PICO_FIRST_VALUE_RE.search(line)
            if match:
                try:
                    value = float(match.group(1))
                except ValueError:
                    state["last_unparsed"] = line
                    continue
                if math.isfinite(value):
                    return line
            state["last_unparsed"] = line

        if len(state["buffer"]) > 512:
            state["buffer"] = state["buffer"][-512:]
        time.sleep(0.01)
    return None


def ensure_pico_streaming(reason="manage startup"):
    """The pico_tool.py --fix routine, run automatically inside manage.

    Listen first, escape only if needed. A healthy Pico that is already
    streaming is never written to, which preserves an earlier revision\'s finding that writing
    to a live main.py can freeze its output loop.

    Returns True if the Pico is confirmed streaming Angle frames.
    """
    if not PICO_AUTO_FIX_ON_RUN:
        print("Pico auto-fix disabled; skipping startup Pico check.", flush=True)
        return False
    if serial is None:
        print("Warning: pyserial not available; cannot run Pico auto-fix.", flush=True)
        return False

    port = PICO_GYRO_PORT
    if not os.path.exists(port):
        print(
            f"Pico auto-fix: {port} does not exist. Check the USB cable and "
            f"run 'ls /dev/ttyACM*'.",
            flush=True,
        )
        return False

    print(f"Pico auto-fix starting on {port} ({reason}).", flush=True)
    state = {
        "bytes": 0,
        "lines": 0,
        "buffer": "",
        "raw_repl": False,
        "last_unparsed": "",
        "error": None,
    }

    try:
        pico_serial = serial.Serial(port, PICO_GYRO_BAUD, timeout=PICO_GYRO_SERIAL_TIMEOUT)
    except Exception as error:
        print(f"Pico auto-fix: could not open {port}: {error}", flush=True)
        return False

    try:
        # DTR must stay asserted: MicroPython on RP2 only emits stdout while
        # tud_cdc_connected() is true, and that flag follows the DTR line.
        try:
            pico_serial.dtr = True
            pico_serial.rts = False
        except Exception:
            pass

        line = _pico_listen_for_angle_frame(
            pico_serial, PICO_GYRO_PASSIVE_LISTEN_SECONDS, state
        )
        if line:
            print(
                f"Pico auto-fix: already streaming, nothing to do. "
                f"{line[:PICO_GYRO_PRINT_LINE_MAX_CHARS]}",
                flush=True,
            )
            return True

        attempts = max(1, int(PICO_AUTO_FIX_MAX_ATTEMPTS))
        for attempt in range(1, attempts + 1):
            if state["raw_repl"]:
                sequence = (
                    "Sending Ctrl-C + Ctrl-B + Ctrl-D"
                    if PICO_RESTART_SEND_CTRL_B
                    else "PICO_RESTART_SEND_CTRL_B is False, so this cannot be "
                         "escaped - Ctrl-D does not reboot from raw REPL"
                )
                print(
                    f"Pico auto-fix: Pico is in MicroPython RAW REPL, so main.py "
                    f"is not running. {sequence} ({attempt}/{attempts}).",
                    flush=True,
                )
            else:
                print(
                    f"Pico auto-fix: silent for "
                    f"{PICO_GYRO_PASSIVE_LISTEN_SECONDS:.1f}s, so main.py is not "
                    f"streaming. Escape attempt {attempt}/{attempts}.",
                    flush=True,
                )

            try:
                pico_send_escape_sequence(pico_serial)
            except Exception as error:
                print(f"Pico auto-fix: escape write failed: {error}", flush=True)
                return False

            state["buffer"] = ""
            line = _pico_listen_for_angle_frame(
                pico_serial, PICO_AUTO_FIX_VERIFY_SECONDS, state
            )
            if line:
                print(
                    f"Pico auto-fix: RECOVERED on attempt {attempt}. "
                    f"{line[:PICO_GYRO_PRINT_LINE_MAX_CHARS]}",
                    flush=True,
                )
                return True

        print(
            f"Pico auto-fix FAILED after {attempts} attempts. "
            f"bytes={state['bytes']} lines={state['lines']} "
            f"raw_repl={state['raw_repl']}",
            flush=True,
        )
        if state["last_unparsed"]:
            print(
                f"Last Pico line: "
                f"{state['last_unparsed'][:PICO_GYRO_PRINT_LINE_MAX_CHARS]}",
                flush=True,
            )
        # Check raw REPL first: it is the most common cause, and its symptoms
        # otherwise get misreported as one of the two below.
        if state["raw_repl"]:
            print(
                "Pico auto-fix hint: the Pico is still in RAW REPL. If "
                "PICO_RESTART_SEND_CTRL_B is False, set it True - Ctrl-D alone "
                "cannot escape raw REPL. Otherwise unplug and replug the Pico, "
                "or run 'mpremote reset'.",
                flush=True,
            )
        elif state["bytes"] == 0:
            print(
                "Pico auto-fix hint: zero bytes. Unplug and replug the Pico, or "
                "check that nothing else holds the port.",
                flush=True,
            )
        else:
            print(
                "Pico auto-fix hint: the Pico talks but never streams frames. "
                "Look for [TOF FATAL] or [FATAL] from main.py.",
                flush=True,
            )
        print(
            "Continuing anyway; the gyro reader will retry when it opens.",
            flush=True,
        )
        return False

    finally:
        # Release the port so PicoWT901YawReader can take ownership. Dropping
        # DTR on close pauses MicroPython's stdout, and re-asserting it on the
        # next open resumes it; main.py itself keeps running throughout.
        try:
            pico_serial.close()
        except Exception:
            pass
        time.sleep(0.30)


def _to_uint32(value: int) -> int:
    """Convert signed int to the unsigned 32-bit value expected by the SDK."""
    return int(value) & 0xFFFFFFFF


def _rpm_to_velocity_lsb(rpm: float) -> int:
    return int(round(float(rpm) / DXL_VELOCITY_UNIT_RPM))


def restart_pico_main(reason="program startup"):
    """Legacy Pico restart. SUPERSEDED by ensure_pico_streaming().

    Only runs when PICO_RESTART_MAIN_ON_RUN is True, which is not the default.
    ensure_pico_streaming() does strictly more: it listens before writing so a
    healthy Pico is never disturbed, it verifies that Angle frames actually
    arrive afterwards, and it retries. Prefer that. This is kept so the old
    flag still behaves sensibly if it is ever switched back on.

    The Pico is shared by the WT901 gyro stream and parking ToF stream. This
    function opens the Pico USB serial port only long enough to send the reset
    sequence, then closes it so the gyro reader or parking reader can reopen it.
    """
    if not PICO_RESTART_MAIN_ON_RUN:
        print("Pico main.py restart disabled; skipping Pico restart signal.", flush=True)
        return False

    if serial is None:
        print("Warning: pyserial is not available; cannot restart Pico main.py.", flush=True)
        return False

    port = PICO_RESTART_PORT or PICO_GYRO_PORT
    baud = PICO_RESTART_BAUD or PICO_GYRO_BAUD
    print(
        f"Pico main.py restart signal starting on {port} at {baud} baud "
        f"({reason}).",
        flush=True,
    )

    try:
        with serial.Serial(
            port,
            baud,
            timeout=0.05,
            write_timeout=0.25,
        ) as pico_serial:
            time.sleep(PICO_RESTART_OPEN_WAIT_SECONDS)
            # an earlier revision: one shared escape implementation, so this legacy path can
            # never drift from ensure_pico_streaming() and the gyro reader.
            pico_send_escape_sequence(pico_serial)

        print(
            f"Pico soft reboot signal sent. Waiting "
            f"{PICO_RESTART_AFTER_CTRL_D_SECONDS:.1f} seconds for main.py...",
            flush=True,
        )
        time.sleep(PICO_RESTART_AFTER_CTRL_D_SECONDS)
        print("Pico main.py restart wait complete.", flush=True)
        return True

    except Exception as error:
        print(
            f"Warning: Pico main.py restart signal failed on {port}: {error}",
            flush=True,
        )
        return False


def send_pico_gyro_reset(reason="program startup"):
    """Ask Pico main.py to reset/zero the WT901 gyro for a fresh run.

    This does not talk to the WT901 directly. The WT901 is wired to the Pico,
    so older versions sent simple text commands to Pico main.py. Your Pico program can
    handle either RESET_GYRO or ZERO_GYRO and ignore unknown commands.
    """
    if not PICO_GYRO_RESET_ON_RUN:
        print("Pico WT901 reset command disabled; skipping gyro reset signal.", flush=True)
        return False

    if serial is None:
        print("Warning: pyserial is not available; cannot send Pico gyro reset.", flush=True)
        return False

    port = PICO_RESTART_PORT or PICO_GYRO_PORT
    baud = PICO_RESTART_BAUD or PICO_GYRO_BAUD
    commands = tuple(str(cmd).strip() for cmd in PICO_GYRO_RESET_COMMANDS if str(cmd).strip())
    if not commands:
        print("No Pico gyro reset commands configured.", flush=True)
        return False

    print(
        f"Pico WT901 gyro reset signal starting on {port} at {baud} baud "
        f"({reason}). Commands={commands}",
        flush=True,
    )

    try:
        with serial.Serial(
            port,
            baud,
            timeout=0.05,
            write_timeout=0.25,
        ) as pico_serial:
            time.sleep(PICO_RESTART_OPEN_WAIT_SECONDS)
            try:
                pico_serial.reset_input_buffer()
            except Exception:
                pass

            for command in commands:
                pico_serial.write((command + "\n").encode("utf-8"))
                pico_serial.flush()
                time.sleep(0.05)

        time.sleep(PICO_GYRO_RESET_WAIT_SECONDS)
        print("Pico WT901 gyro reset signal sent.", flush=True)
        return True

    except Exception as error:
        print(
            f"Warning: Pico WT901 gyro reset signal failed on {port}: {error}",
            flush=True,
        )
        return False


class DynamixelBus:
    """Shared DYNAMIXEL bus for steering and throttle parts."""
    def __init__(self, port=DXL_PORT, baudrate=DXL_BAUDRATE):
        self.port_handler = PortHandler(port)
        self.packet_handler = PacketHandler(DXL_PROTOCOL_VERSION)
        self.ref_count = 0
        self.closed = False

        if not self.port_handler.openPort():
            raise RuntimeError(f"Failed to open DYNAMIXEL port: {port}")
        if not self.port_handler.setBaudRate(baudrate):
            raise RuntimeError(f"Failed to set DYNAMIXEL baudrate: {baudrate}")

        print(f"DYNAMIXEL bus opened on {port} at {baudrate} baud")

    def register_part(self):
        self.ref_count += 1

    def release_part(self):
        self.ref_count = max(0, self.ref_count - 1)
        if self.ref_count == 0:
            self.close()

    def _check(self, dxl_id, result, error, action):
        if result != COMM_SUCCESS:
            print(f"[DYNAMIXEL ID {dxl_id}] {action} failed: "
                  f"{self.packet_handler.getTxRxResult(result)}")
            return False
        if error != 0:
            print(f"[DYNAMIXEL ID {dxl_id}] {action} error: "
                  f"{self.packet_handler.getRxPacketError(error)}")
            return False
        return True

    def write1(self, dxl_id, addr, value, action="write1"):
        result, error = self.packet_handler.write1ByteTxRx(
            self.port_handler, dxl_id, addr, int(value)
        )
        return self._check(dxl_id, result, error, action)

    def write4(self, dxl_id, addr, value, action="write4"):
        result, error = self.packet_handler.write4ByteTxRx(
            self.port_handler, dxl_id, addr, _to_uint32(value)
        )
        return self._check(dxl_id, result, error, action)

    def read4(self, dxl_id, addr, action="read4"):
        value, result, error = self.packet_handler.read4ByteTxRx(
            self.port_handler, dxl_id, addr
        )
        if not self._check(dxl_id, result, error, action):
            return None
        return int(value)

    def configure_motor(self, dxl_id, mode, profile_accel=200, profile_velocity=None):
        # Operating mode can only be changed with torque off.
        self.write1(dxl_id, ADDR_TORQUE_ENABLE, TORQUE_OFF, "torque off")
        time.sleep(0.02)
        self.write1(dxl_id, ADDR_OPERATING_MODE, mode, "set operating mode")
        time.sleep(0.02)
        self.write4(dxl_id, ADDR_PROFILE_ACCEL, profile_accel, "set profile accel")
        if profile_velocity is not None:
            self.write4(dxl_id, ADDR_PROFILE_VELOCITY, profile_velocity,
                        "set profile velocity")
        self.write1(dxl_id, ADDR_TORQUE_ENABLE, TORQUE_ON, "torque on")
        time.sleep(0.02)

    def close(self):
        if self.closed:
            return
        self.closed = True
        try:
            self.port_handler.closePort()
            print("DYNAMIXEL bus closed")
        except Exception:
            pass


_DXL_BUS = None


def get_dxl_bus() -> DynamixelBus:
    global _DXL_BUS
    if _DXL_BUS is None or _DXL_BUS.closed:
        _DXL_BUS = DynamixelBus()
    return _DXL_BUS



def _from_uint32(value: int) -> int:
    """Interpret an unsigned SDK position value as a signed 32-bit integer."""
    value = int(value) & 0xFFFFFFFF
    return value - 0x100000000 if value & 0x80000000 else value


def _motor_ids(value):
    """Allow run_for_degrees() to receive one motor ID or several motor IDs."""
    if isinstance(value, (list, tuple, set)):
        return [int(v) for v in value]
    return [int(value)]


def _throttle_direction_for_id(motor_id: int) -> int:
    """Return the configured physical-forward multiplier for a throttle motor."""
    try:
        index = list(DXL_THROTTLE_IDS).index(int(motor_id))
    except ValueError:
        return 1

    if index < len(DXL_THROTTLE_DIRECTIONS):
        return -1 if DXL_THROTTLE_DIRECTIONS[index] < 0 else 1
    return 1


def run_position(motor_id, speed, angle, bus=None):
    """Move the steering motor to an angle relative to center.

    Args:
        motor_id: DYNAMIXEL steering ID.
        speed: profile speed percentage from 1 to 100.
        angle: steering angle in physical motor degrees. Positive is right.
        bus: optional shared DynamixelBus.
    """
    bus = bus or get_dxl_bus()
    speed_percent = max(1.0, min(abs(float(speed)), 100.0))
    profile_rpm = DXL_THROTTLE_MAX_RPM * speed_percent / 100.0
    profile_velocity = max(1, _rpm_to_velocity_lsb(profile_rpm))

    bus.configure_motor(
        int(motor_id),
        MODE_POSITION,
        profile_accel=DXL_STEER_PROFILE_ACCEL,
        profile_velocity=profile_velocity,
    )

    goal = DXL_STEER_CENTER_TICKS + (
        DXL_STEER_DIRECTION * float(angle) * DXL_TICKS_PER_DEG
    )
    goal = int(max(0, min(4095, round(goal))))
    bus.write4(int(motor_id), ADDR_GOAL_POSITION, goal, "run_position goal")
    return goal


def run_for_degrees(motor_id, speed, degrees, bus=None):
    """Rotate one or more throttle motors by a relative number of degrees.

    This is the earlier run_for_degrees style adapted to the shared bus used by
    manage26. Positive degrees means physical forward according to
    DXL_THROTTLE_DIRECTIONS. Negative degrees means reverse. A negative speed
    also reverses the requested movement, while its magnitude controls speed.

    The function blocks until every selected motor reaches its target or the
    calculated timeout expires.
    """
    bus = bus or get_dxl_bus()
    ids = _motor_ids(motor_id)
    if not ids:
        return True

    speed_value = float(speed)
    speed_percent = max(1.0, min(abs(speed_value), 100.0))
    signed_degrees = float(degrees) * (-1.0 if speed_value < 0 else 1.0)

    profile_rpm = DXL_THROTTLE_MAX_RPM * speed_percent / 100.0
    profile_velocity = max(1, _rpm_to_velocity_lsb(profile_rpm))

    starts = {}
    targets = {}

    # Extended position mode allows relative movements that continue across the
    # normal 0..4095 single-turn boundary.
    for dxl_id in ids:
        bus.configure_motor(
            dxl_id,
            MODE_EXTENDED_POSITION,
            profile_accel=DXL_THROTTLE_PROFILE_ACCEL,
            profile_velocity=profile_velocity,
        )

        raw_position = bus.read4(
            dxl_id, ADDR_PRESENT_POSITION, "read starting position"
        )
        if raw_position is None:
            raise RuntimeError(
                f"Could not read starting position from DYNAMIXEL ID {dxl_id}"
            )

        current_position = _from_uint32(raw_position)
        physical_direction = _throttle_direction_for_id(dxl_id)
        delta_ticks = int(round(
            signed_degrees * DXL_TICKS_PER_DEG * physical_direction
        ))
        target_position = current_position + delta_ticks

        starts[dxl_id] = current_position
        targets[dxl_id] = target_position

    # Send all targets together before waiting, so multi-motor drive systems
    # begin moving nearly simultaneously.
    for dxl_id in ids:
        bus.write4(
            dxl_id,
            ADDR_GOAL_POSITION,
            targets[dxl_id],
            "run_for_degrees goal",
        )

    degrees_per_second = max(1.0, profile_rpm * 6.0)
    expected_seconds = abs(signed_degrees) / degrees_per_second
    timeout = max(1.0, expected_seconds + RUN_FOR_DEGREES_TIMEOUT_MARGIN)
    deadline = time.monotonic() + timeout

    while time.monotonic() < deadline:
        all_reached = True
        for dxl_id in ids:
            raw_position = bus.read4(
                dxl_id, ADDR_PRESENT_POSITION, "read movement position"
            )
            if raw_position is None:
                all_reached = False
                continue

            current_position = _from_uint32(raw_position)
            if abs(targets[dxl_id] - current_position) > RUN_FOR_DEGREES_TOLERANCE_TICKS:
                all_reached = False

        if all_reached:
            return True

        time.sleep(RUN_FOR_DEGREES_POLL_SECONDS)

    for dxl_id in ids:
        raw_position = bus.read4(dxl_id, ADDR_PRESENT_POSITION, "read timeout position")
        current_position = None if raw_position is None else _from_uint32(raw_position)
        print(
            f"run_for_degrees timeout on ID {dxl_id}: "
            f"start={starts[dxl_id]}, target={targets[dxl_id]}, "
            f"current={current_position}",
            flush=True,
        )
    return False


def restore_throttle_velocity_mode(bus=None):
    """Return all drive motors to the velocity mode used by the model."""
    bus = bus or get_dxl_bus()
    for dxl_id in DXL_THROTTLE_IDS:
        bus.configure_motor(
            int(dxl_id),
            MODE_VELOCITY,
            profile_accel=DXL_THROTTLE_PROFILE_ACCEL,
            profile_velocity=None,
        )
        bus.write4(int(dxl_id), ADDR_GOAL_VELOCITY, 0, "restore throttle stop")


def set_throttle_velocity_normalized(throttle: float, bus=None, action="direct throttle velocity"):
    """Command throttle motors directly with a normalized -1.0 to +1.0 value."""
    bus = bus or get_dxl_bus()
    throttle = max(min(float(throttle), 1.0), -1.0)
    rpm = DXL_THROTTLE_MAX_RPM * throttle
    velocity_lsb = _rpm_to_velocity_lsb(rpm)

    for dxl_id in DXL_THROTTLE_IDS:
        direction = _throttle_direction_for_id(int(dxl_id))
        bus.write4(
            int(dxl_id),
            ADDR_GOAL_VELOCITY,
            velocity_lsb * direction,
            action,
        )


def _gyro_turn_relative(mode: str, phase_name: str, turn_degrees: float,
                        throttle: float, steering_angle: float,
                        get_current_yaw_deg):
    """Turn a relative number of raw-gyro degrees from the current yaw."""
    bus = get_dxl_bus()
    turn_degrees = abs(float(turn_degrees))

    if turn_degrees <= GYRO_OBSTACLE_FINISH_TOLERANCE_DEG:
        print(
            f"{mode}: {phase_name} skipped, only {turn_degrees:.1f} degrees needed.",
            flush=True,
        )
        return True

    print(
        f"{mode}: {phase_name} starting. Turning relative "
        f"{turn_degrees:.1f} degrees with throttle {throttle:+.2f}.",
        flush=True,
    )

    restore_throttle_velocity_mode(bus=bus)
    run_position(DXL_STEER_ID, 60, steering_angle, bus=bus)
    time.sleep(0.15)

    start_yaw = float(get_current_yaw_deg())
    deadline = time.monotonic() + GYRO_OBSTACLE_FINISH_TIMEOUT_SECONDS

    try:
        set_throttle_velocity_normalized(
            throttle,
            bus=bus,
            action=f"{phase_name} throttle",
        )

        while time.monotonic() < deadline:
            current_yaw = float(get_current_yaw_deg())
            turned_deg = abs(current_yaw - start_yaw)

            if turned_deg >= max(0.0, turn_degrees - GYRO_OBSTACLE_FINISH_TOLERANCE_DEG):
                print(
                    f"{mode}: {phase_name} complete "
                    f"({turned_deg:.1f}/{turn_degrees:.1f} degrees).",
                    flush=True,
                )
                return True

            time.sleep(0.01)

        current_yaw = float(get_current_yaw_deg())
        turned_deg = abs(current_yaw - start_yaw)
        print(
            f"{mode}: {phase_name} timed out "
            f"({turned_deg:.1f}/{turn_degrees:.1f} degrees).",
            flush=True,
        )
        return False

    finally:
        set_throttle_velocity_normalized(0.0, bus=bus, action=f"{phase_name} stop")
        run_position(DXL_STEER_ID, 100, 0, bus=bus)
        restore_throttle_velocity_mode(bus=bus)
        time.sleep(0.10)

def gyro_obstacle_finish_turn(drive_mode: str, end_yaw_deg: float, get_current_yaw_deg):
    """Obstacle-only forward final gyro correction.

    This computes how many degrees still need to be corrected, then turns that
    relative amount from the current yaw instead of chasing an absolute yaw.
    The requested amount is based on the main calibrated gyro, but the relative
    final-turn measurement can use a separate multiplier through the
    get_current_yaw_deg callback.
    """
    mode = str(drive_mode).upper()
    if mode not in ("OCW", "OCCW"):
        return False

    target_minus_90 = max(0.0, GYRO_TARGET_DEG - GYRO_OBSTACLE_FINISH_OFFSET_DEG)
    end_yaw_abs = abs(float(end_yaw_deg))
    requested_turn_deg = abs(end_yaw_abs - target_minus_90)

    if requested_turn_deg <= GYRO_OBSTACLE_FINISH_TOLERANCE_DEG:
        print(
            f"{mode}: no finish gyro correction needed "
            f"(end yaw {end_yaw_abs:.1f}, target-90 {target_minus_90:.1f}).",
            flush=True,
        )
        return True

    turn_deg = min(requested_turn_deg, GYRO_OBSTACLE_FINISH_MAX_TURN_DEG)
    if turn_deg < requested_turn_deg:
        print(
            f"{mode}: finish gyro correction clipped from "
            f"{requested_turn_deg:.1f} to {turn_deg:.1f} degrees.",
            flush=True,
        )

    steering_angle = (
        -GYRO_OBSTACLE_FINISH_STEER_DEG
        if mode == "OCW"
        else GYRO_OBSTACLE_FINISH_STEER_DEG
    )
    turn_name = "left" if mode == "OCW" else "right"

    print(
        f"{mode}: forward-only finish correction starting. End yaw={end_yaw_abs:.1f}, "
        f"target-90={target_minus_90:.1f}, turning {turn_name} "
        f"{turn_deg:.1f} relative degrees forward. "
        f"Final-turn gyro multiplier={GYRO_FINAL_TURN_MULTIPLIER:.3f}, "
        f"throttle={GYRO_OBSTACLE_FINISH_THROTTLE:+.2f}.",
        flush=True,
    )

    return _gyro_turn_relative(
        mode,
        phase_name="forward relative finish turn",
        turn_degrees=turn_deg,
        throttle=GYRO_OBSTACLE_FINISH_THROTTLE,
        steering_angle=steering_angle,
        get_current_yaw_deg=get_current_yaw_deg,
    )

# ---------------------------------------------------------------------------
# Integrated parking helpers
# ---------------------------------------------------------------------------
def _parking_clamp(value, minimum, maximum):
    return max(minimum, min(maximum, value))


def _parking_throttle_id():
    return int(DXL_THROTTLE_IDS[0])


def _parking_valid_tof(value):
    """Return True when a millimeter reading is usable."""
    return value is not None and 5.0 <= float(value) <= 2000.0


def _parking_valid_sensor_status(status_value):
    if not PARKING_USE_SENSOR_STATUS:
        return True
    if status_value is None:
        return True
    return status_value in PARKING_VALID_STATUS_CODES


def _parking_compute_pid_equation_value(a_mm, b_mm, td_mm,
                                        angle_weight=None,
                                        distance_weight=None):
    """Parking wall-follow error equation.

    an earlier revision shape, with the two physical terms now separately weighted:

        angle term    = A - B                 (car rotation relative to wall)
        distance term = (A + B) / 2 - Td      (cross-track offset from target)

    Defaults of 1.0/1.0 reproduce the original an earlier revision equation exactly.
    """
    if angle_weight is None:
        angle_weight = PARKING_PID_ANGLE_WEIGHT
    if distance_weight is None:
        distance_weight = PARKING_PID_DISTANCE_WEIGHT

    # an earlier revision: remove the constant A/B sensor mismatch before it is treated as a
    # wall angle. Without this, the bias survives into steady state as a
    # distance error of -(Wa/Wd)*bias. See PARKING_AB_OFFSET_MM.
    angle_term = (float(a_mm) - float(b_mm)) - PARKING_AB_OFFSET_MM
    distance_term = (float(a_mm) + float(b_mm)) / 2.0 - float(td_mm)
    total = angle_weight * angle_term + distance_weight * distance_term
    return total, angle_term, distance_term


class ParkingSensorFilter:
    """Median-of-N spike rejection followed by a light EMA.

    This program uses this for A and B in both Stage 3B alignment and Stage 4 PID.

    A VL53L0X produces occasional single-sample spikes, especially against the
    dark walls this project uses. an earlier revision tried to handle that with one heavy EMA
    (alpha 0.45), which suppresses spikes only by lagging every reading by
    roughly a frame and a half. Taking a median first removes an isolated spike
    outright, so the EMA afterwards can be light and the filtered value stays
    close to real time. Latency is what was costing alignment accuracy, so
    trading EMA weight for a median is a direct win.

    Measured on a 40 mm stream with one spurious 8 mm sample:
        filter: 40.0 40.0 40.0 25.6 32.1 35.6 37.6   (14.4 mm worst error)
        filter: 40.0 40.0 40.0 40.0 40.0 40.0 40.0   ( 0.0 mm worst error)
    At the Stage 4 distance gain that spike was worth about 11 degrees of false
    steering in an earlier revision and none in an earlier revision.

    Honest trade-off: on a genuine STEP the median holds the old value for one
    extra frame before it moves, so the first frame after a step is slower than
    an earlier revision's EMA; it catches up and is ahead of an earlier revision from the second frame onward.
    That is the right trade here, because real wall distance changes as a ramp
    while the car drives, and it is the spikes that actually break the control.
    """

    def __init__(self, median_window=3, alpha=0.7):
        self.median_window = max(1, int(median_window))
        self.alpha = _parking_clamp(float(alpha), 0.01, 1.0)
        self.recent = []
        self.value = None
        self.last_valid_time = 0.0

    def reset(self):
        self.recent = []
        self.value = None
        self.last_valid_time = 0.0

    def update(self, raw_value):
        """Feed one raw millimeter reading and return the filtered value."""
        raw_value = float(raw_value)
        self.recent.append(raw_value)
        if len(self.recent) > self.median_window:
            self.recent.pop(0)

        ordered = sorted(self.recent)
        median_value = ordered[len(ordered) // 2]

        if self.value is None:
            self.value = median_value
        else:
            self.value = self.alpha * median_value + (1.0 - self.alpha) * self.value

        self.last_valid_time = time.monotonic()
        return self.value

    def age_seconds(self):
        """Seconds since the last valid reading, or None if there never was one."""
        if self.last_valid_time <= 0.0:
            return None
        return time.monotonic() - self.last_valid_time


class ParkingRunStats:
    """Collects Stage 4 telemetry and turns it into a tuning verdict.

    an earlier revision. Reading a wall-follow run by eye from 70+ log lines is slow and easy
    to get wrong - the first pass at tuning this treated a one-way drift as an
    oscillation and lowered the gain, which made it worse. These are the four
    numbers that actually distinguish the failure modes:

      mean (A - B)  A constant non-zero average is a SENSOR MISMATCH, because a
                    real heading error wanders and averages near zero. This
                    value IS the PARKING_AB_OFFSET_MM correction.
      drift slope   A steady one-way trend means the controller is losing to a
                    constant disturbance -> raise PARKING_PID_KI.
      steer usage   If the peak is far below the clamp, the controller never
                    tried hard -> raise PARKING_K().
      sign flips    Frequent reversals with a flat trend is a real oscillation
                    -> lower PARKING_K().
    """

    def __init__(self, target_mm, tof=None):
        self.target_mm = float(target_mm)
        # an earlier revision: kept so the summary can report serial line integrity. Corrupt
        # readings and a mistuned controller produce identical-looking traces,
        # so the data quality must be visible next to the tuning verdict.
        self.tof = tof
        self.errors = []
        self.angles = []
        self.distances = []
        self.steers = []
        # an earlier revision: which frames ran on a reconstructed (estimated) sensor reading.
        self.degraded = []

    def add(self, error, angle_term, average_mm, steer_deg, degraded=False):
        self.errors.append(abs(float(error)))
        self.angles.append(float(angle_term))
        self.distances.append(float(average_mm))
        self.steers.append(float(steer_deg))
        self.degraded.append(bool(degraded))

    @staticmethod
    def _slope_per_sample(values):
        """Least-squares trend, in units per sample."""
        count = len(values)
        if count < 3:
            return 0.0
        mean_x = (count - 1) / 2.0
        mean_y = sum(values) / count
        denominator = sum((i - mean_x) ** 2 for i in range(count))
        if denominator <= 0.0:
            return 0.0
        return sum((i - mean_x) * (values[i] - mean_y) for i in range(count)) / denominator

    def report(self, headline):
        count = len(self.errors)
        if count == 0:
            print(f"{headline} no samples collected.", flush=True)
            return

        mean_error = sum(self.errors) / count
        worst_error = max(self.errors)
        mean_angle = sum(self.angles) / count
        peak_steer = max(abs(s) for s in self.steers)
        mean_steer = sum(abs(s) for s in self.steers) / count
        slope = self._slope_per_sample(self.distances)
        total_drift = slope * count
        flips = sum(
            1
            for i in range(1, count)
            if self.steers[i] * self.steers[i - 1] < 0.0
        )
        flip_fraction = flips / max(1, count - 1)

        print(headline, flush=True)
        print(
            f"  tracking   | mean |e|={mean_error:5.2f}  worst |e|={worst_error:5.2f}  "
            f"samples={count}",
            flush=True,
        )
        print(
            f"  distance   | start={self.distances[0]:5.1f}  end={self.distances[-1]:5.1f}  "
            f"min={min(self.distances):5.1f}  max={max(self.distances):5.1f}  "
            f"target={self.target_mm:.1f} mm",
            flush=True,
        )
        print(
            f"  drift      | {slope:+.3f} mm/sample  ({total_drift:+.1f} mm over the run)",
            flush=True,
        )
        print(
            f"  steering   | mean={mean_steer:4.1f} deg  peak={peak_steer:4.1f} deg  "
            f"of {PARKING_MAX_WALL_STEER_DEG:.0f} deg clamp  "
            f"({100.0 * peak_steer / max(1e-6, PARKING_MAX_WALL_STEER_DEG):.0f}% used)  "
            f"| reversals={100.0 * flip_fraction:.0f}%",
            flush=True,
        )
        print(
            f"  angle term | mean={mean_angle:+.2f} mm  "
            f"(current PARKING_AB_OFFSET_MM={PARKING_AB_OFFSET_MM:+.2f})",
            flush=True,
        )
        degraded_count = sum(1 for d in self.degraded if d)
        degraded_share = 100.0 * degraded_count / count
        print(
            f"  sensors    | {degraded_count}/{count} frames "
            f"({degraded_share:.0f}%) ran on a RECONSTRUCTED reading because "
            f"one ToF was invalid",
            flush=True,
        )

        if self.tof is not None:
            rejected = int(getattr(self.tof, "rejected_lines", 0))
            drops = int(getattr(self.tof, "partial_drops", 0))
            total = int(getattr(self.tof, "frame_number", 0)) + rejected
            share = 100.0 * rejected / max(1, total)
            print(
                f"  link       | {rejected} rejected / {total} lines "
                f"({share:.1f}%), {drops} oversized fragments dropped",
                flush=True,
            )
            if share > 5.0:
                print(
                    f"  WARNING    | {share:.0f}% of Pico lines were malformed. The "
                    f"serial link is dropping bytes, so every distance above is "
                    f"suspect. Fix this BEFORE tuning any gain - a bad reading "
                    f"looks exactly like a bad controller. "
                    f"Last rejected: "
                    f"{str(getattr(self.tof, 'last_rejected_line', ''))[:80]}",
                    flush=True,
                )

        # Oscillation is judged by AMPLITUDE vs NET TRAVEL, not by counting sign
        # flips. A realistic weave has a period of several samples, so it
        # reverses only about a third of the time - a flip-count threshold high
        # enough to avoid false positives misses real oscillations entirely.
        # Swinging a long way while ending up where it started is the reliable
        # signature; drifting swings just as far but does not come back.
        swing = max(self.distances) - min(self.distances)
        oscillating = (
            swing >= 6.0
            and abs(total_drift) < swing * 0.5
            and peak_steer > 0.25 * PARKING_MAX_WALL_STEER_DEG
        )

        # Turn the numbers into one concrete next action. Order matters: bad or
        # missing DATA invalidates every conclusion about the controller, so
        # data quality is checked before any tuning advice is offered.
        if degraded_share >= 40.0:
            print(
                f"  ACTION     | {degraded_share:.0f}% of frames had a ToF sensor "
                f"invalid, so most of this run steered on a reconstructed angle. "
                f"Do NOT tune gains from this run. A VL53L0X drops out at 35-45 "
                f"degrees to a black wall, so reduce how far the car is rotated "
                f"during the follow, or re-aim the sensors.",
                flush=True,
            )
        elif abs(mean_angle) >= 1.0:
            print(
                f"  ACTION     | the angle term still averages {mean_angle:+.2f} mm, so the "
                f"sensors disagree by that much. Set "
                f"PARKING_AB_OFFSET_MM = {PARKING_AB_OFFSET_MM + mean_angle:.2f} "
                f"(current {PARKING_AB_OFFSET_MM:+.2f} plus this run's "
                f"{mean_angle:+.2f}). Until then the car sits "
                f"{abs(mean_angle) * PARKING_PID_ANGLE_WEIGHT / max(1e-6, PARKING_PID_DISTANCE_WEIGHT):.1f} mm "
                f"off target no matter what the gains are.",
                flush=True,
            )
        elif oscillating:
            print(
                f"  ACTION     | distance swung {swing:.1f} mm but only netted "
                f"{total_drift:+.1f} mm, with {100.0 * flip_fraction:.0f}% steering "
                f"reversals at up to {peak_steer:.1f} deg. That is a real "
                f"oscillation, not drift. Lower PARKING_K() "
                f"(now {PARKING_K():.2f}); if PARKING_PID_KI "
                f"(now {PARKING_PID_KI:.2f}) was raised recently, lower that first.",
                flush=True,
            )
        elif abs(total_drift) >= 3.0:
            direction = "toward" if total_drift < 0 else "away from"
            print(
                f"  ACTION     | one-way drift {direction} the wall, not oscillation. "
                f"A P-only loop cannot reject a constant disturbance. Raise "
                f"PARKING_PID_KI (now {PARKING_PID_KI:.2f}) before touching "
                f"PARKING_K().",
                flush=True,
            )
        elif peak_steer < 0.25 * PARKING_MAX_WALL_STEER_DEG and mean_error > 1.5:
            print(
                f"  ACTION     | never used more than {peak_steer:.1f} of "
                f"{PARKING_MAX_WALL_STEER_DEG:.0f} deg while holding a mean error of "
                f"{mean_error:.1f}. The loop is too timid: raise PARKING_K() "
                f"(now {PARKING_K():.2f}).",
                flush=True,
            )
        else:
            print(
                f"  ACTION     | nothing obviously wrong. Mean error {mean_error:.2f} mm, "
                f"drift {total_drift:+.1f} mm. If it still looks bad on the "
                f"floor, lower PARKING_FOLLOW_SPEED_PERCENT "
                f"(now {PARKING_FOLLOW_SPEED_PERCENT:.0f}).",
                flush=True,
            )


class ParkingPIDController:
    """Parking wall-follow PID.

    Key points:
      * conditional integration - the integral stops accumulating while the
        steering output is saturated, so KI cannot wind up against the clamp
      * filtered derivative - a raw dError/dt on noisy ToF data is unusable, so
        the derivative is smoothed before KD is applied
    Both are inert while KI and KD are 0.0, which is still the default.
    """

    def __init__(self, kp, ki, kd, integral_limit,
                 derivative_alpha=None):
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.integral_limit = abs(integral_limit)
        if derivative_alpha is None:
            derivative_alpha = PARKING_PID_DERIVATIVE_FILTER_ALPHA
        self.derivative_alpha = _parking_clamp(float(derivative_alpha), 0.01, 1.0)
        self.integral = 0.0
        self.previous_error = None
        self.previous_time = None
        self.filtered_derivative = 0.0

    def reset(self):
        self.integral = 0.0
        self.previous_error = None
        self.previous_time = None
        self.filtered_derivative = 0.0

    def update(self, error, output_saturated=False):
        now = time.monotonic()
        if self.previous_time is None:
            dt = 0.0
        else:
            dt = now - self.previous_time
        if dt <= 0.0:
            dt = 0.001

        # Conditional integration: do not keep charging the integrator while the
        # steering command is already clamped, unless the error now pushes the
        # output back out of saturation.
        if not output_saturated:
            self.integral += error * dt
            self.integral = _parking_clamp(
                self.integral,
                -self.integral_limit,
                self.integral_limit,
            )

        if self.previous_error is None:
            raw_derivative = 0.0
            self.filtered_derivative = 0.0
        else:
            raw_derivative = (error - self.previous_error) / dt
            self.filtered_derivative = (
                self.derivative_alpha * raw_derivative
                + (1.0 - self.derivative_alpha) * self.filtered_derivative
            )

        p_term = self.kp * error
        i_term = self.ki * self.integral
        d_term = self.kd * self.filtered_derivative
        output = p_term + i_term + d_term

        self.previous_error = error
        self.previous_time = now
        return output, p_term, i_term, d_term, dt


def _parking_position_step(previous_raw, current_raw):
    """Signed 32-bit position delta, matching the standalone parking program."""
    step = (int(current_raw) - int(previous_raw)) & 0xFFFFFFFF
    if step & 0x80000000:
        step -= 0x100000000
    return step


def parking_steering_angle_to_ticks(angle_deg):
    angle_deg = _parking_clamp(float(angle_deg), -50.0, 50.0)
    corrected_angle = (
        angle_deg * PARKING_STEERING_DIRECTION
        + PARKING_STEERING_OFFSET_DEG
    )
    ticks = round(
        PARKING_STEERING_CENTER_TICKS
        + corrected_angle * DXL_TICKS_PER_DEG
    )
    return int(_parking_clamp(ticks, 0, 4095))


def parking_set_steering(angle_deg, bus=None):
    """Parking-only steering command using parking center/offset variables."""
    bus = bus or get_dxl_bus()
    goal_ticks = parking_steering_angle_to_ticks(angle_deg)
    bus.write4(
        DXL_STEER_ID,
        ADDR_GOAL_POSITION,
        goal_ticks,
        "parking steering position",
    )


def parking_set_steering_profile_velocity(profile_velocity=None, bus=None):
    """Set how fast the steering servo is allowed to move, in DXL LSB units.

    an earlier revision fix. PARKING_STEERING_PROFILE_VELOCITY existed in an earlier revision but was never
    written to the motor. parking_set_steering() only writes ADDR_GOAL_POSITION,
    so the servo silently kept whatever profile velocity the previous
    parking_run_position(DXL_STEER_ID, 80, ...) call had left behind.

        80 LSB * 0.229 rpm/LSB = 18.3 rpm = about 110 deg/s

    At that rate a 20 degree steering correction needs roughly 180 ms, which is
    3-4 whole control periods at the Pico's ~20 Hz frame rate. Stage 3B and
    Stage 4 were both issuing corrections faster than the servo could execute
    them, which is a large part of the reported inaccuracy in both stages.

        180 LSB * 0.229 rpm/LSB = 41.2 rpm = about 247 deg/s

    Call this once before entering a closed-loop parking stage.
    """
    bus = bus or get_dxl_bus()
    if profile_velocity is None:
        profile_velocity = PARKING_STEERING_PROFILE_VELOCITY
    profile_velocity = max(1, int(profile_velocity))
    bus.write4(
        DXL_STEER_ID,
        ADDR_PROFILE_VELOCITY,
        profile_velocity,
        "parking closed-loop steering profile velocity",
    )
    return profile_velocity


def parking_set_throttle_percent(speed_percent, bus=None):
    """Parking-only throttle command using raw velocity units."""
    bus = bus or get_dxl_bus()
    speed_percent = _parking_clamp(float(speed_percent), -100.0, 100.0)
    raw_velocity = round((speed_percent / 100.0) * PARKING_MAX_THROTTLE_RAW)

    for dxl_id in DXL_THROTTLE_IDS:
        direction = _throttle_direction_for_id(int(dxl_id))
        bus.write4(
            int(dxl_id),
            ADDR_GOAL_VELOCITY,
            raw_velocity * direction,
            "parking throttle velocity",
        )


def parking_set_motion(speed_percent, steering_angle, bus=None):
    parking_set_steering(steering_angle, bus=bus)
    parking_set_throttle_percent(speed_percent, bus=bus)


def parking_stop(bus=None):
    parking_set_throttle_percent(0.0, bus=bus)


def parking_stop_drive_immediately(bus=None):
    """Send parking throttle stop several times in case one packet is missed."""
    bus = bus or get_dxl_bus()
    for _ in range(3):
        parking_stop(bus=bus)
        time.sleep(0.03)


def parking_run_position(motor_id, speed, angle, bus=None):
    """Parking version of run_position(id, speed, angle)."""
    bus = bus or get_dxl_bus()
    if int(motor_id) != int(DXL_STEER_ID):
        raise ValueError(f"parking_run_position expects steering ID {DXL_STEER_ID}")
    profile_velocity = max(1, abs(int(speed)))
    bus.write4(
        int(motor_id),
        ADDR_PROFILE_VELOCITY,
        profile_velocity,
        "parking steering profile velocity",
    )
    parking_set_steering(angle, bus=bus)


def parking_run_for_degrees(motor_id, speed, degrees, bus=None):
    """Parking version of run_for_degrees(id, speed, degrees)."""
    bus = bus or get_dxl_bus()
    motor_id = int(motor_id)
    speed = abs(float(speed))
    degrees = float(degrees)

    if motor_id != _parking_throttle_id():
        raise ValueError(f"parking_run_for_degrees expects throttle ID {_parking_throttle_id()}")
    if speed <= 0:
        raise ValueError("parking_run_for_degrees speed must be greater than 0")
    if degrees == 0:
        parking_stop(bus=bus)
        return

    target_ticks = abs(degrees) * DXL_TICKS_PER_DEG
    direction = 1 if degrees > 0 else -1
    commanded_speed = direction * speed

    previous_position = bus.read4(
        motor_id,
        ADDR_PRESENT_POSITION,
        "parking read starting throttle position",
    )
    if previous_position is None:
        raise RuntimeError("Could not read starting throttle position for parking")

    traveled_ticks = 0.0
    start_time = time.monotonic()
    last_print_time = 0.0

    print(
        f"parking_run_for_degrees(id={motor_id}, speed={speed:.1f}, "
        f"degrees={degrees:.1f})",
        flush=True,
    )

    parking_set_throttle_percent(commanded_speed, bus=bus)

    try:
        while traveled_ticks < target_ticks:
            if time.monotonic() - start_time >= PARKING_RUN_FOR_DEGREES_TIMEOUT:
                raise RuntimeError(
                    "parking_run_for_degrees timed out at "
                    f"{traveled_ticks / DXL_TICKS_PER_DEG:.1f}/"
                    f"{abs(degrees):.1f} motor degrees"
                )

            current_position = bus.read4(
                motor_id,
                ADDR_PRESENT_POSITION,
                "parking read throttle position",
            )
            if current_position is None:
                continue

            step = _parking_position_step(previous_position, current_position)
            previous_position = current_position
            traveled_ticks += abs(step)

            now = time.monotonic()
            if now - last_print_time >= 0.15:
                print(
                    "Parking throttle rotation: "
                    f"{traveled_ticks / DXL_TICKS_PER_DEG:.1f}/"
                    f"{abs(degrees):.1f} degrees",
                    flush=True,
                )
                last_print_time = now

            time.sleep(0.01)

    finally:
        parking_stop(bus=bus)

    print("parking_run_for_degrees complete.", flush=True)


class ParkingPicoToF:
    SENSOR_PATTERN = re.compile(
        r"(?<!S)\b([ABCD])\s*=\s*(-?\d+(?:\.\d+)?)",
        re.IGNORECASE,
    )
    STATUS_PATTERN = re.compile(
        r"\b(?:S([ABCD])|([ABCD])S)\s*=\s*(-?\d+)",
        re.IGNORECASE,
    )
    ORDERED_NUMBER_PATTERN = re.compile(r"-?\d+(?:\.\d+)?")

    def __init__(self, port=None):
        if serial is None:
            raise RuntimeError("pyserial is required for parking ToF serial input")

        self.port_name = port or PARKING_PICO_PORT
        self.serial = serial.Serial(
            self.port_name,
            PARKING_PICO_BAUD,
            timeout=0.05,
        )
        # an earlier revision: assert DTR explicitly rather than relying on the pyserial
        # default. MicroPython on RP2 gates all USB stdout on
        # tud_cdc_connected(), which follows the DTR line, so a cleared DTR
        # silently discards every ToF frame the parking stages depend on.
        try:
            self.serial.dtr = True
            self.serial.rts = False
        except Exception:
            pass
        self.last_raw_line = ""
        self.last_values = None
        self.last_update_time = 0.0
        self.frame_number = 0
        # An earlier revision line-integrity counters. If rejected_lines climbs during a run,
        # the serial link is dropping bytes and the readings cannot be trusted.
        self._partial = b""
        self.partial_drops = 0
        self.rejected_lines = 0
        self.last_rejected_line = ""
        # an earlier revision: the gyro and the ToF read the SAME Pico line, but they accept
        # different things - the gyro needs only an Angle field, while this
        # reader rejects the whole line unless A, B and C are all present. A
        # part-degraded Pico line therefore feeds one subsystem and starves the
        # other, which looks like "only one of them ever works". These counters
        # say which field was actually missing.
        self.bytes_received = 0
        self.missing_sensor_counts = {}
        self._lock = threading.Lock()
        self._stop_event = threading.Event()

        time.sleep(0.25)
        self.serial.reset_input_buffer()

        self._reader_thread = threading.Thread(
            target=self._reader_loop,
            name="ParkingPicoToFReader",
            daemon=True,
        )
        self._reader_thread.start()

    def _parse_line(self, line):
        """Parse Pico output: Angle, A, B, C, D with no status fields required.

        Supported examples:
            Angle=12.34, A=50, B=51, C=100, D=75
            12.34,50,51,100,75

        Status fields such as AS/BS/CS are no longer required. If an older Pico
        program still prints them, they are tolerated but ignored unless present.
        """
        counts = {}
        values = {}
        statuses = {}

        # Preferred labeled format.
        for sensor_name, value_text in self.SENSOR_PATTERN.findall(line):
            sensor_name = sensor_name.upper()
            counts[sensor_name] = counts.get(sensor_name, 0) + 1
            try:
                values[sensor_name] = float(value_text)
            except ValueError:
                return None

        # Backward-compatible optional status parsing. An earlier revision does not require it.
        for status_prefix_name, status_suffix_name, status_text in self.STATUS_PATTERN.findall(line):
            sensor_name = (status_prefix_name or status_suffix_name).upper()
            try:
                statuses["S" + sensor_name] = int(status_text)
            except ValueError:
                return None

        # New compact ordered format: Angle, A, B, C, D.
        #
        # An earlier revision CRITICAL FIX. This fallback used to run whenever a required
        # sensor was missing, including on labeled lines that had merely been
        # damaged. On this Pico's format that silently produces WRONG readings,
        # because the old AS=/BS=/CS= status fields are numbers too and slide
        # into the sensor slots:
        #
        #   "Angle=1275.34,A=30.5,AS=0,B=33.1,BS=0,C"   (truncated line)
        #   numbers  -> [1275.34, 30.5, 0, 33.1, 0]
        #   mapped   -> Angle=1275.34, A=30.5, B=0, C=33.1, D=0
        #                                     ^^^         ^^^^
        #                              B gets a status,  C gets B's distance
        #
        # True (A+B)/2 = 31.8 mm; parsed (A+B)/2 = 15.2 mm. Always biased LOW,
        # which matches "manage read 18-27 where main.py read 33-34".
        #
        # Positional mapping is only meaningful on a bare numeric line. If the
        # line carries ANY label or status field it is the labeled format, and
        # a missing sensor means the line is damaged - so reject it and wait
        # for the next frame rather than inventing values.
        line_carries_labels = bool(counts) or bool(statuses)
        if not PARKING_REQUIRED_SENSORS.issubset(values) and not line_carries_labels:
            try:
                ordered_numbers = [
                    float(text) for text in self.ORDERED_NUMBER_PATTERN.findall(line)
                ]
            except ValueError:
                ordered_numbers = []

            if len(ordered_numbers) >= 5:
                values.update({
                    "Angle": ordered_numbers[0],
                    "A": ordered_numbers[1],
                    "B": ordered_numbers[2],
                    "C": ordered_numbers[3],
                    "D": ordered_numbers[4],
                })
                for name in ("A", "B", "C", "D"):
                    counts[name] = 1

        if not PARKING_REQUIRED_SENSORS.issubset(values):
            return None
        if any(counts.get(name, 0) != 1 for name in PARKING_REQUIRED_SENSORS):
            return None

        frame = {
            sensor_name: values[sensor_name]
            for sensor_name in PARKING_REQUIRED_SENSORS
        }
        if "D" in values:
            frame["D"] = values["D"]
        if "Angle" in values:
            frame["Angle"] = values["Angle"]
        for status_name in ("SA", "SB", "SC", "SD"):
            if status_name in statuses:
                frame[status_name] = statuses[status_name]
        return frame

    def _reader_loop(self):
        while not self._stop_event.is_set():
            try:
                raw = self.serial.readline()
            except Exception:
                return
            if not raw:
                continue

            # an earlier revision: pyserial's readline() returns whatever has arrived when its
            # timeout expires, WITHOUT a trailing newline. A half-received line
            # is therefore indistinguishable from a complete one, and gets
            # parsed as if it were whole. Stitch fragments together instead and
            # only parse once a real newline has arrived.
            if not raw.endswith(b"\n"):
                self._partial += raw
                if len(self._partial) > 512:
                    # Never seen a newline in a sane amount of data; drop it
                    # rather than growing without bound.
                    self._partial = b""
                    self.partial_drops += 1
                continue

            raw = self._partial + raw
            self._partial = b""
            self.bytes_received += len(raw)

            line = raw.decode("utf-8", errors="ignore").strip()
            values = self._parse_line(line)
            if values is None:
                self.rejected_lines += 1
                self.last_rejected_line = line
                # an earlier revision: record WHICH required sensor was absent. "C missing on
                # every line" is a dead ToF sensor; "nothing present at all" is
                # a serial or Pico problem. Very different fixes.
                present = {
                    match.group(1).upper()
                    for match in self.SENSOR_PATTERN.finditer(line)
                }
                for name in PARKING_REQUIRED_SENSORS:
                    if name not in present:
                        self.missing_sensor_counts[name] = (
                            self.missing_sensor_counts.get(name, 0) + 1
                        )
                continue
            with self._lock:
                self.last_raw_line = line
                self.last_values = values
                self.last_update_time = time.monotonic()
                self.frame_number += 1

    def read_sensor(self, sensor_name, timeout=0.5):
        sensor_name = sensor_name.upper()
        with self._lock:
            starting_frame = self.frame_number
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            with self._lock:
                if (
                    self.last_values is not None
                    and self.frame_number > starting_frame
                    and sensor_name in self.last_values
                ):
                    raw_value = self.last_values[sensor_name]
                    return {
                        "raw": raw_value,
                        "mm": raw_value,
                        "line": self.last_raw_line,
                        "frame": self.frame_number,
                        "age": time.monotonic() - self.last_update_time,
                    }
            time.sleep(0.002)
        return None

    def read_frame(self, required_sensors=("A", "B"), timeout=0.5):
        required_sensors = tuple(sensor.upper() for sensor in required_sensors)
        with self._lock:
            starting_frame = self.frame_number
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            with self._lock:
                if (
                    self.last_values is not None
                    and self.frame_number > starting_frame
                    and all(sensor in self.last_values for sensor in required_sensors)
                ):
                    return {
                        "values": dict(self.last_values),
                        "line": self.last_raw_line,
                        "frame": self.frame_number,
                        "age": time.monotonic() - self.last_update_time,
                    }
            time.sleep(0.002)
        return None

    def link_report(self):
        """Say WHY the ToF stage got no usable frames.

        The gyro and this reader share one Pico line, so "the gyro worked but
        the ToF did not" is almost never a hardware failure of the link. It is
        usually one sensor missing from an otherwise fine line, because this
        reader requires A, B and C together while the gyro needs only Angle.
        """
        print("\nParking ToF link report:", flush=True)
        print(
            f"  bytes={self.bytes_received}  good frames={self.frame_number}  "
            f"rejected lines={self.rejected_lines}  "
            f"partial fragments dropped={self.partial_drops}",
            flush=True,
        )
        if self.last_rejected_line:
            print(f"  last rejected line: {self.last_rejected_line[:110]}", flush=True)

        if self.bytes_received == 0:
            print(
                "  DIAGNOSIS: nothing arrived at all. This is the port, not the "
                "sensors - another process holds /dev/ttyACM0, or the Pico is "
                "not streaming. Run 'python3 pico_tool.py --fix'.",
                flush=True,
            )
            return

        if self.missing_sensor_counts:
            worst = sorted(
                self.missing_sensor_counts.items(),
                key=lambda kv: kv[1],
                reverse=True,
            )
            summary = ", ".join(f"{name} missing from {n} lines" for name, n in worst)
            print(f"  missing fields: {summary}", flush=True)

            top_name, top_count = worst[0]
            lowest = worst[-1][1]
            # If EVERY required sensor is missing at a similar rate, no single
            # sensor is at fault - the lines are not sensor lines at all (boot
            # text, a REPL banner, the wrong format). Blaming whichever name
            # sorted first would send you to inspect healthy hardware.
            all_missing_together = (
                len(worst) >= len(PARKING_REQUIRED_SENSORS)
                and lowest >= top_count * 0.8
            )
            if all_missing_together:
                print(
                    "  DIAGNOSIS: ALL sensor fields are missing at the same rate, "
                    "so these are not sensor lines - the Pico is printing "
                    "something else (boot text, a REPL banner, or a different "
                    "format). No individual ToF is implicated. Run "
                    "'python3 pico_tool.py --verify' to see the raw lines.",
                    flush=True,
                )
                return

            if top_count > max(1, self.rejected_lines * 0.8):
                print(
                    f"  DIAGNOSIS: sensor {top_name} is absent from almost every "
                    f"line while the others are present, so the Pico is not "
                    f"publishing it. The gyro keeps working because it only "
                    f"needs the Angle field off that same line - that is why it "
                    f"looks like 'only one of them works'. Check that ToF "
                    f"{top_name} initialises on the Pico (watch for "
                    f"[TOF FATAL]), and its XSHUT wiring.",
                    flush=True,
                )
                return

        if self.frame_number == 0:
            print(
                "  DIAGNOSIS: bytes arrived but no line ever parsed. Check the "
                "Pico output format against PARKING_REQUIRED_SENSORS "
                f"({sorted(PARKING_REQUIRED_SENSORS)}), and run "
                "'python3 pico_tool.py --verify' to see raw vs parsed.",
                flush=True,
            )

    def close(self):
        self._stop_event.set()
        if hasattr(self, "_reader_thread"):
            self._reader_thread.join(timeout=0.3)
        if self.serial.is_open:
            self.serial.close()


def parking_wait_until_front_wall(tof, bus=None):
    bus = bus or get_dxl_bus()
    c_stop_threshold = PARKING_FRONT_STOP_MM + PARKING_C_DISTANCE_LEEWAY_MM
    print("\nPARKING STAGE 2: Driving straight toward the front wall", flush=True)
    print(
        f"C target={PARKING_FRONT_STOP_MM:.1f} mm | "
        f"C leeway={PARKING_C_DISTANCE_LEEWAY_MM:.1f} mm | "
        f"stop threshold={c_stop_threshold:.1f} mm",
        flush=True,
    )

    start_time = time.monotonic()
    invalid_start = None
    confirmed = 0
    last_print_time = 0.0
    parking_set_motion(PARKING_APPROACH_SPEED_PERCENT, PARKING_C_APPROACH_STEER_DEG, bus=bus)

    while time.monotonic() - start_time < PARKING_APPROACH_TIMEOUT:
        reading = tof.read_sensor("C")
        distance = None if reading is None else reading["mm"]

        if not _parking_valid_tof(distance):
            confirmed = 0
            if invalid_start is None:
                invalid_start = time.monotonic()
            if time.monotonic() - invalid_start >= PARKING_SENSOR_INVALID_TIMEOUT:
                parking_stop(bus=bus)
                raise RuntimeError("Parking sensor C was invalid for too long")
            continue

        invalid_start = None
        if distance <= c_stop_threshold:
            confirmed += 1
        else:
            confirmed = 0

        now = time.monotonic()
        if now - last_print_time >= PARKING_TOF_PRINT_INTERVAL:
            display = (
                f"frame={reading['frame']} | C={distance:6.1f} mm | "
                f"age={reading['age'] * 1000:5.1f} ms | "
                f"stop {confirmed}/{PARKING_DISTANCE_CONFIRM_READINGS}"
            )
            if PARKING_PRINT_PICO_RAW_LINE:
                display += f" | Pico: {reading['line']}"
            print(display, flush=True)
            last_print_time = now

        if confirmed >= PARKING_DISTANCE_CONFIRM_READINGS:
            parking_stop(bus=bus)
            print(f"Front-wall position reached at C={distance:.1f} mm.", flush=True)
            return

        parking_set_motion(PARKING_APPROACH_SPEED_PERCENT, PARKING_C_APPROACH_STEER_DEG, bus=bus)

    parking_stop(bus=bus)
    raise RuntimeError("Parking front-wall approach timed out")


def parking_turn_left_for_degrees(motor_degrees, bus=None):
    bus = bus or get_dxl_bus()
    print(
        f"\nPARKING STAGE 1: Full-left forward turn for {motor_degrees:.1f} motor degrees",
        flush=True,
    )
    parking_stop(bus=bus)
    parking_run_position(DXL_STEER_ID, 80, PARKING_FULL_LEFT_STEER_DEG, bus=bus)
    time.sleep(PARKING_STEERING_SETTLE_SECONDS)
    try:
        parking_run_for_degrees(
            _parking_throttle_id(),
            PARKING_TURN_SPEED_PERCENT,
            abs(motor_degrees),
            bus=bus,
        )
    finally:
        parking_stop(bus=bus)
        parking_run_position(DXL_STEER_ID, 80, 0, bus=bus)
        time.sleep(0.3)
    print("Parking forward-left turn complete.", flush=True)


def parking_turn_right_backwards_for_degrees(motor_degrees, bus=None):
    bus = bus or get_dxl_bus()
    print(
        f"\nPARKING STAGE 3: Full-right reverse turn for {motor_degrees:.1f} motor degrees",
        flush=True,
    )
    parking_stop(bus=bus)
    parking_run_position(DXL_STEER_ID, 80, PARKING_FULL_RIGHT_STEER_DEG, bus=bus)
    time.sleep(PARKING_STEERING_SETTLE_SECONDS)
    try:
        parking_run_for_degrees(
            _parking_throttle_id(),
            PARKING_TURN_SPEED_PERCENT,
            -abs(motor_degrees),
            bus=bus,
        )
    finally:
        parking_stop(bus=bus)
        parking_run_position(DXL_STEER_ID, 80, 0, bus=bus)
        time.sleep(0.2)
    print("Parking fixed reverse-right turn complete.", flush=True)



def parking_turn_backwards_before_parking(drive_mode: str, motor_degrees, bus=None):
    """Reverse turn before parking, mirrored by obstacle direction.

    OCW keeps the pasted program behavior: full-right reverse.
    OCCW uses the opposite steering direction: full-left reverse.
    """
    mode = str(drive_mode).upper()
    if mode == "OCCW":
        steering_angle = PARKING_OCCW_PRE_PARK_REVERSE_STEER_DEG
        turn_label = "full-left reverse mirrored for OCCW"
    else:
        steering_angle = PARKING_OCW_PRE_PARK_REVERSE_STEER_DEG
        turn_label = "full-right reverse pasted OCW"

    print(
        f"\nPARKING STAGE 3: {turn_label} turn for "
        f"{float(motor_degrees):.1f} motor degrees",
        flush=True,
    )

    parking_stop(bus=bus)
    parking_run_position(DXL_STEER_ID, 80, steering_angle, bus=bus)
    time.sleep(PARKING_STEERING_SETTLE_SECONDS)

    try:
        parking_run_for_degrees(
            _parking_throttle_id(),
            PARKING_TURN_SPEED_PERCENT,
            -abs(float(motor_degrees)),
            bus=bus,
        )
    finally:
        parking_stop(bus=bus)
        parking_run_position(DXL_STEER_ID, 80, 0, bus=bus)
        time.sleep(0.2)

    print("Pre-parking reverse turn complete.", flush=True)


def _parking_align_measure_residual(tof, samples=5, per_read_timeout=0.20):
    """Measure |A - B| while the car is stopped, to report real alignment error.

    an earlier revision diagnostic. Everything printed during the alignment loop is measured
    while the car is still moving, so it does not tell you how well the car
    actually ended up aligned. This reads a few frames after the car has come
    to rest and returns the median |A - B|.
    """
    differences = []
    for _ in range(max(1, int(samples))):
        frame_reading = tof.read_frame(("A", "B"), timeout=per_read_timeout)
        if frame_reading is None:
            continue
        frame = frame_reading["values"]
        a_value = frame.get("A")
        b_value = frame.get("B")
        if not _parking_valid_tof(a_value) or not _parking_valid_tof(b_value):
            continue
        if not _parking_valid_sensor_status(frame.get("SA")):
            continue
        if not _parking_valid_sensor_status(frame.get("SB")):
            continue
        differences.append(
            abs((float(a_value) - float(b_value)) - PARKING_AB_OFFSET_MM)
        )

    if not differences:
        return None
    differences.sort()
    return differences[len(differences) // 2]


def parking_balance_ab_backwards(tof, difference_leeway_mm=PARKING_AB_BALANCE_DIFF_LEEWAY_MM, bus=None):
    """Stage 3B reverse A/B alignment.

    an earlier revision rewrite. an earlier revision was a bang-bang controller: any |A - B| above the leeway
    produced a fixed +/-10 degrees of steering, and balance was declared after
    2 consecutive frames inside the leeway. With ToF latency, EMA lag and a
    servo limited to ~110 deg/s, that combination cannot settle - the car
    sweeps through balanced and the 2-frame check happily fires mid-swing.

    an earlier revision changes, in order of how much they matter:
      1. steering is proportional to the difference, so the correction fades
         out as the car approaches balance instead of slamming to full deflection
      2. balance must be SETTLED - the difference must also have stopped
         changing, and the confirmations must span a minimum wall-clock time
      3. two-phase speed - creep once the difference is small
      4. median-of-3 filtering instead of a heavy laggy EMA
      5. a dropped ToF frame holds the last steering instead of snapping straight
      6. the servo profile velocity is raised so it can follow the controller
    """
    bus = bus or get_dxl_bus()
    print(
        f"\nPARKING STAGE 3B: Proportional reverse A/B balance until filtered "
        f"|A - B| <= {difference_leeway_mm:.1f} mm and settled",
        flush=True,
    )

    steer_profile = parking_set_steering_profile_velocity(bus=bus)

    print(
        f"A/B balance settings: speed={PARKING_AB_BALANCE_SPEED_PERCENT:.0f}% reverse "
        f"(fine {PARKING_AB_ALIGN_FINE_SPEED_PERCENT:.0f}% under "
        f"{PARKING_AB_ALIGN_FINE_DIFF_MM:.1f} mm) | "
        f"steer Kp={PARKING_AB_ALIGN_KP_DEG_PER_MM:.2f} deg/mm, "
        f"range {PARKING_AB_ALIGN_MIN_STEER_DEG:.1f}-{PARKING_AB_BALANCE_STEER_DEG:.1f} deg | "
        f"servo profile={steer_profile} LSB",
        flush=True,
    )
    print(
        f"A/B confirm: {PARKING_AB_BALANCE_REQUIRED_COUNT} frames over "
        f">={PARKING_AB_BALANCE_REQUIRED_SECONDS:.2f}s with rate "
        f"<={PARKING_AB_ALIGN_SETTLED_RATE_MM_PER_SEC:.1f} mm/s | "
        f"median={PARKING_AB_ALIGN_MEDIAN_WINDOW} alpha={PARKING_AB_BALANCE_SMOOTHING_ALPHA:.2f} | "
        f"read timeout={PARKING_AB_BALANCE_READ_TIMEOUT:.2f}s | "
        f"timeout continues to PID={PARKING_AB_BALANCE_TIMEOUT_CONTINUE_TO_PID}",
        flush=True,
    )

    parking_stop(bus=bus)
    start_time = time.monotonic()
    invalid_start = None
    confirmed = 0
    confirm_started = None
    last_print_time = 0.0
    last_state = "starting"

    filter_a = ParkingSensorFilter(
        PARKING_AB_ALIGN_MEDIAN_WINDOW,
        PARKING_AB_BALANCE_SMOOTHING_ALPHA,
    )
    filter_b = ParkingSensorFilter(
        PARKING_AB_ALIGN_MEDIAN_WINDOW,
        PARKING_AB_BALANCE_SMOOTHING_ALPHA,
    )

    # an earlier revision: a dropped frame must not change the steering command. An earlier revision called
    # parking_set_motion(speed, 0.0) on every dropout, which kicked the car
    # straight for a frame and then kicked it back - a self-inflicted
    # disturbance right in the middle of a fine alignment.
    last_steering = 0.0
    last_speed = -abs(PARKING_AB_BALANCE_SPEED_PERCENT)
    # an earlier revision: consecutive frames with a sensor invalid, reset on every good frame.
    invalid_frames = 0

    # Signed difference and its rate, used for the settled test.
    previous_signed_difference = None
    previous_difference_time = None
    difference_rate = 0.0

    while time.monotonic() - start_time < PARKING_AB_BALANCE_TIMEOUT:
        frame_reading = tof.read_frame(
            ("A", "B"),
            timeout=PARKING_AB_BALANCE_READ_TIMEOUT,
        )
        frame = None if frame_reading is None else frame_reading["values"]
        if frame is None:
            confirmed = 0
            confirm_started = None
            if invalid_start is None:
                invalid_start = time.monotonic()
            # Hold the previous command through the dropout.
            parking_set_motion(last_speed, last_steering, bus=bus)
            if time.monotonic() - invalid_start >= PARKING_AB_BALANCE_INVALID_TIMEOUT:
                parking_stop_drive_immediately(bus=bus)
                parking_run_position(DXL_STEER_ID, 80, 0, bus=bus)
                raise RuntimeError("Parking A/B balance did not receive valid frames")
            continue

        a_distance = frame["A"]
        b_distance = frame["B"]
        a_valid = _parking_valid_tof(a_distance) and _parking_valid_sensor_status(frame.get("SA"))
        b_valid = _parking_valid_tof(b_distance) and _parking_valid_sensor_status(frame.get("SB"))
        if not a_valid or not b_valid:
            confirmed = 0
            confirm_started = None
            if invalid_start is None:
                invalid_start = time.monotonic()

            # With one sensor invalid there is NO angle information, so
            # there is no correct direction to turn. Hold the current steering
            # for one frame in case it is just noise, then straighten and keep
            # reversing - the parkingv35/v38 behavior. Continuing to rotate
            # blind is what drives the sensor further into the 35-45 degree
            # blind spot that caused the dropout in the first place.
            invalid_frames += 1
            if (
                PARKING_AB_ALIGN_STRAIGHTEN_WHEN_INVALID
                and invalid_frames > PARKING_AB_ALIGN_INVALID_GRACE_FRAMES
            ):
                blind_steer = 0.0
            else:
                blind_steer = last_steering

            parking_set_motion(last_speed, blind_steer, bus=bus)
            last_steering = blind_steer

            now = time.monotonic()
            if now - last_print_time >= PARKING_AB_BALANCE_PRINT_INTERVAL:
                print(
                    f"ALIGN A/B | t={now - start_time:4.1f}/"
                    f"{PARKING_AB_BALANCE_TIMEOUT:.1f}s | "
                    f"raw A={a_distance:5.1f}{'' if a_valid else '*'} "
                    f"B={b_distance:5.1f}{'' if b_valid else '*'} | "
                    f"INVALID x{invalid_frames} -> "
                    f"{'reversing STRAIGHT until it recovers' if blind_steer == 0.0 else 'holding steer (grace)'} | "
                    f"steer={blind_steer:+5.1f}",
                    flush=True,
                )
                last_print_time = now

            if time.monotonic() - invalid_start >= PARKING_AB_BALANCE_INVALID_TIMEOUT:
                parking_stop_drive_immediately(bus=bus)
                parking_run_position(DXL_STEER_ID, 80, 0, bus=bus)
                raise RuntimeError("Parking A/B balance sensors invalid for too long")
            continue

        invalid_start = None
        invalid_frames = 0
        now = time.monotonic()
        smoothed_a = filter_a.update(a_distance)
        smoothed_b = filter_b.update(b_distance)

        # an earlier revision: subtract the calibrated sensor mismatch so the aligner squares
        # the CAR to the wall instead of squaring the two READINGS to each
        # other. With an uncalibrated mismatch it parks the car crooked by
        # exactly the angle that produces that much real difference.
        raw_difference = abs((float(a_distance) - float(b_distance)) - PARKING_AB_OFFSET_MM)
        signed_difference = (smoothed_a - smoothed_b) - PARKING_AB_OFFSET_MM
        filtered_difference = abs(signed_difference)

        # an earlier revision: how fast the alignment error is changing. Reversing straight
        # along a wall at a fixed angle keeps (A - B) constant, so a nonzero
        # rate here means the car is still rotating. That is exactly the state
        # An earlier revision could mistake for "balanced".
        if previous_signed_difference is not None and previous_difference_time is not None:
            dt = now - previous_difference_time
            if dt > 0.0:
                difference_rate = (signed_difference - previous_signed_difference) / dt
        previous_signed_difference = signed_difference
        previous_difference_time = now

        settled = abs(difference_rate) <= PARKING_AB_ALIGN_SETTLED_RATE_MM_PER_SEC
        within_leeway = filtered_difference <= difference_leeway_mm

        if within_leeway and settled:
            confirmed += 1
            if confirm_started is None:
                confirm_started = now
            steering = 0.0
            state = "BALANCED"
        elif within_leeway:
            # Inside the leeway but still swinging. Do not count it, and do not
            # fight it either - let the car coast through so the rate decays.
            confirmed = 0
            confirm_started = None
            steering = 0.0
            state = "IN-LEEWAY SWINGING"
        else:
            confirmed = 0
            confirm_started = None
            # an earlier revision: proportional magnitude instead of a fixed full-deflection kick.
            magnitude = PARKING_AB_ALIGN_KP_DEG_PER_MM * filtered_difference
            magnitude = _parking_clamp(
                magnitude,
                abs(PARKING_AB_ALIGN_MIN_STEER_DEG),
                abs(PARKING_AB_BALANCE_STEER_DEG),
            )
            if signed_difference > 0.0:
                steering = -magnitude
                state = "A>B BACK-LEFT"
            else:
                steering = magnitude
                state = "B>A BACK-RIGHT"

        # an earlier revision: creep once the error is small so the car can stop on balance
        # rather than coasting through it.
        if filtered_difference <= PARKING_AB_ALIGN_FINE_DIFF_MM:
            speed = -abs(PARKING_AB_ALIGN_FINE_SPEED_PERCENT)
            phase = "FINE"
        else:
            speed = -abs(PARKING_AB_BALANCE_SPEED_PERCENT)
            phase = "COARSE"

        parking_set_motion(speed, steering, bus=bus)
        last_steering = steering
        last_speed = speed
        last_state = state

        if now - last_print_time >= PARKING_AB_BALANCE_PRINT_INTERVAL:
            confirm_span = 0.0 if confirm_started is None else now - confirm_started
            print(
                f"ALIGN A/B | t={now - start_time:4.1f}/{PARKING_AB_BALANCE_TIMEOUT:.1f}s | "
                f"raw A={a_distance:5.1f} B={b_distance:5.1f} d={raw_difference:4.1f} | "
                f"filt A={smoothed_a:5.1f} B={smoothed_b:5.1f} d={filtered_difference:4.1f} | "
                f"rate={difference_rate:+6.1f} mm/s | "
                f"steer={steering:+5.1f} spd={speed:+4.0f}% {phase} | {state} | "
                f"match {confirmed}/{PARKING_AB_BALANCE_REQUIRED_COUNT} "
                f"({confirm_span:.2f}/{PARKING_AB_BALANCE_REQUIRED_SECONDS:.2f}s)",
                flush=True,
            )
            last_print_time = now

        confirm_span_ok = (
            confirm_started is not None
            and (now - confirm_started) >= PARKING_AB_BALANCE_REQUIRED_SECONDS
        )
        if confirmed >= PARKING_AB_BALANCE_REQUIRED_COUNT and confirm_span_ok:
            # an earlier revision: center the steering before braking, then let the car come
            # fully to rest before declaring the stage finished. An earlier revision stopped
            # and slept 0.15 s, which was not long enough for the car to settle.
            parking_run_position(DXL_STEER_ID, 80, 0, bus=bus)
            parking_stop_drive_immediately(bus=bus)
            time.sleep(PARKING_AB_ALIGN_SETTLE_SECONDS)

            residual_text = ""
            if PARKING_AB_ALIGN_REPORT_RESIDUAL:
                residual = _parking_align_measure_residual(tof)
                if residual is not None:
                    residual_text = (
                        f" Measured residual |A-B| at rest = {residual:.1f} mm."
                    )
            print(
                f"Parking A/B reverse balance complete: "
                f"raw A={a_distance:.1f} mm, raw B={b_distance:.1f} mm, "
                f"filtered |A-B|={filtered_difference:.1f} mm, "
                f"rate={difference_rate:+.1f} mm/s." + residual_text,
                flush=True,
            )
            return True

    parking_run_position(DXL_STEER_ID, 80, 0, bus=bus)
    parking_stop_drive_immediately(bus=bus)
    time.sleep(PARKING_AB_BALANCE_TIMEOUT_BRAKE_SECONDS)
    message = (
        "Parking A/B reverse balance timed out after "
        f"{PARKING_AB_BALANCE_TIMEOUT:.1f}s; last state={last_state}."
    )
    if PARKING_AB_BALANCE_TIMEOUT_CONTINUE_TO_PID:
        print(message + " Continuing into PID ToF_run anyway.", flush=True)
        return False

    raise RuntimeError(message)


def parking_tof_run(tof, bus=None):
    """Stage 4 wall-follow PID.

    an earlier revision retune. An earlier revision controller used a single gain of 2.0 deg/mm on the
    lumped error (A - B) + ((A + B)/2 - Td), fed with unfiltered ToF readings,
    at 100% speed, through a servo limited to about 110 deg/s. A car that was
    parallel but 15 mm off target already saturated the +/-30 degree clamp, so
    in practice the controller was bang-bang for most of the run - which is
    what "the PID is inaccurate" looks like from the driver's seat.

    Key points:
      1. separate weights for the angle term and the cross-track term, with the
         overall gain lowered, so the car settles onto a converging approach
         angle instead of overshooting
      2. median + light EMA on A and B so sensor noise is not amplified by the
         gain into steering jitter
      3. invalid readings hold their last valid value for a short window rather
         than jumping to Td, which used to fake a large angle error
      4. steering slew-rate limiting
      5. conditional integration in the PID so KI is usable if it is ever needed
      6. parking-wall detection debounced with its own confirm count
      7. the servo profile velocity is raised so it can follow the controller
      8. a tracking-error summary is printed when the stage ends
    """
    bus = bus or get_dxl_bus()
    speed = PARKING_FOLLOW_SPEED_PERCENT
    parking_trigger_mm = PARKING_PARK_TRIGGER_MM
    wall_find_leeway_mm = PARKING_WALL_FIND_LEEWAY_MM
    td_value = PARKING_TD()
    k_value = PARKING_K()
    wall_found_threshold = parking_trigger_mm + wall_find_leeway_mm
    wall_confirm_needed = max(1, int(PARKING_WALL_FIND_CONFIRM_READINGS))
    pid = ParkingPIDController(
        k_value,
        PARKING_PID_KI,
        PARKING_PID_KD,
        PARKING_PID_INTEGRAL_LIMIT,
    )

    steer_profile = parking_set_steering_profile_velocity(bus=bus)

    start_time = time.monotonic()
    invalid_start = None
    wall_confirmed = 0
    break_count = 0
    last_print_time = 0.0

    filter_a = ParkingSensorFilter(
        PARKING_PID_MEDIAN_WINDOW,
        PARKING_PID_SMOOTHING_ALPHA,
    )
    filter_b = ParkingSensorFilter(
        PARKING_PID_MEDIAN_WINDOW,
        PARKING_PID_SMOOTHING_ALPHA,
    )

    # an earlier revision: hold the last command through a dropout instead of forcing straight.
    last_steering = 0.0
    last_steering_time = None

    # an earlier revision: full telemetry, summarized into a tuning verdict when the stage ends.
    stats = ParkingRunStats(td_value, tof=tof)

    # an earlier revision: the last angle measured while BOTH sensors were valid. Used to
    # reconstruct whichever sensor drops out, instead of substituting Td.
    last_good_angle = 0.0
    degraded_frames = 0

    print("\nPARKING STAGE 4: Weighted PID ToF_run using sensors A and B", flush=True)
    print(
        "Equation: K * [ Wa*(A - B) + Wd*(((A + B) / 2) - PARKING_TD()) ]",
        flush=True,
    )
    print(
        f"K()={k_value:.2f} | Wa(angle)={PARKING_PID_ANGLE_WEIGHT:.2f} | "
        f"Wd(distance)={PARKING_PID_DISTANCE_WEIGHT:.2f} | "
        f"KI={PARKING_PID_KI:.3f} KD={PARKING_PID_KD:.3f} | "
        f"Td()={td_value:.1f} mm | speed={speed:.0f}%",
        flush=True,
    )
    # Any uncalibrated A/B mismatch shows up as a steady-state distance error
    # of -(Wa/Wd) mm per mm of bias, so make that sensitivity visible.
    offset_gain = PARKING_PID_ANGLE_WEIGHT / max(1e-6, PARKING_PID_DISTANCE_WEIGHT)
    print(
        f"A/B offset calibration={PARKING_AB_OFFSET_MM:+.2f} mm | "
        f"every 1.0 mm of REMAINING A/B bias moves the car "
        f"{offset_gain:.2f} mm off the {td_value:.1f} mm target "
        f"(negative bias = further out, positive = closer in). "
        f"Measure with: python3 pico_tool.py --calib",
        flush=True,
    )
    print(
        f"Filter: median={PARKING_PID_MEDIAN_WINDOW} alpha={PARKING_PID_SMOOTHING_ALPHA:.2f} | "
        f"steer clamp=+/-{PARKING_MAX_WALL_STEER_DEG:.0f} deg, "
        f"rate limit={PARKING_PID_MAX_STEER_RATE_DEG_PER_SEC:.0f} deg/s, "
        f"servo profile={steer_profile} LSB | "
        f"parking wall threshold=raw A <= {wall_found_threshold:.1f} mm "
        f"confirmed {wall_confirm_needed}x",
        flush=True,
    )

    while time.monotonic() - start_time < PARKING_FOLLOW_TIMEOUT:
        frame_reading = tof.read_frame(("A", "B"))
        frame = None if frame_reading is None else frame_reading["values"]
        if frame is None:
            pid.reset()
            wall_confirmed = 0
            break_count = 0
            if invalid_start is None:
                invalid_start = time.monotonic()
            # an earlier revision: hold the previous steering rather than snapping to straight.
            parking_set_motion(speed, last_steering, bus=bus)
            if time.monotonic() - invalid_start >= PARKING_SENSOR_INVALID_TIMEOUT:
                parking_stop_drive_immediately(bus=bus)
                parking_run_position(DXL_STEER_ID, 80, 0, bus=bus)
                raise RuntimeError("Parking PID ToF_run did not receive valid A/B frame")
            continue

        now = time.monotonic()
        original_a = frame["A"]
        original_b = frame["B"]
        a_valid = _parking_valid_tof(original_a) and _parking_valid_sensor_status(frame.get("SA"))
        b_valid = _parking_valid_tof(original_b) and _parking_valid_sensor_status(frame.get("SB"))

        # an earlier revision: reconstruct a missing sensor from the one that still works,
        # using the last measured angle. See PARKING_PID_DEGRADED_HOLD_ANGLE.
        # The distance stays real; only the unmeasurable angle is held, and it
        # decays so a long dropout becomes distance-only following.
        degraded = False
        if a_valid and b_valid:
            a_value = filter_a.update(original_a)
            b_value = filter_b.update(original_b)
            last_good_angle = a_value - b_value
            a_source = b_source = "ok"

        elif a_valid and PARKING_PID_DEGRADED_HOLD_ANGLE:
            a_value = filter_a.update(original_a)
            b_value = a_value - last_good_angle
            last_good_angle *= PARKING_PID_DEGRADED_ANGLE_DECAY
            a_source, b_source = "ok", "est"
            degraded = True

        elif b_valid and PARKING_PID_DEGRADED_HOLD_ANGLE:
            b_value = filter_b.update(original_b)
            a_value = b_value + last_good_angle
            last_good_angle *= PARKING_PID_DEGRADED_ANGLE_DECAY
            a_source, b_source = "est", "ok"
            degraded = True

        else:
            # Either both sensors are invalid, or angle-hold is disabled. Fall
            # back to the old behavior: hold the last filtered value briefly,
            # then substitute Td.
            degraded = True
            if a_valid:
                a_value = filter_a.update(original_a)
                a_source = "ok"
            else:
                age = filter_a.age_seconds()
                if (filter_a.value is not None and age is not None
                        and age <= PARKING_PID_INVALID_HOLD_SECONDS):
                    a_value, a_source = filter_a.value, "held"
                else:
                    a_value, a_source = td_value, "Td"
            if b_valid:
                b_value = filter_b.update(original_b)
                b_source = "ok"
            else:
                age = filter_b.age_seconds()
                if (filter_b.value is not None and age is not None
                        and age <= PARKING_PID_INVALID_HOLD_SECONDS):
                    b_value, b_source = filter_b.value, "held"
                else:
                    b_value, b_source = td_value, "Td"

        if degraded:
            degraded_frames += 1

        if a_valid or b_valid:
            invalid_start = None
        else:
            if invalid_start is None:
                invalid_start = now
            if time.monotonic() - invalid_start >= PARKING_SENSOR_INVALID_TIMEOUT:
                parking_stop_drive_immediately(bus=bus)
                parking_run_position(DXL_STEER_ID, 80, 0, bus=bus)
                raise RuntimeError("Parking PID ToF_run A and B invalid too long")

        equation_value, angle_term, distance_term = _parking_compute_pid_equation_value(
            a_value, b_value, td_value
        )

        if (
            PARKING_ENABLE_PID_AUTO_BREAK
            and abs(equation_value) <= PARKING_PID_BREAK_ERROR_TOLERANCE
        ):
            break_count += 1
            if break_count >= PARKING_PID_BREAK_REQUIRED_COUNT:
                parking_stop_drive_immediately(bus=bus)
                parking_run_position(DXL_STEER_ID, 80, -1.0, bus=bus)
                print(f"Parking PID break reached at error={equation_value:.2f}.", flush=True)
                return
        else:
            break_count = 0

        # Saturation is evaluated on the previous command so the integrator can
        # be frozen on the very cycle the output is clamped.
        was_saturated = abs(last_steering) >= (PARKING_MAX_WALL_STEER_DEG - 0.01)
        pid_output, p_term, i_term, d_term, dt = pid.update(
            equation_value,
            output_saturated=was_saturated,
        )

        steering = pid_output * PARKING_PID_STEERING_DIRECTION

        # an earlier revision: back off while running on a reconstructed reading. Scaling the
        # OUTPUT rather than the gain keeps the integrator's own state clean.
        if degraded:
            steering *= PARKING_PID_DEGRADED_GAIN_SCALE
            steer_limit = min(
                PARKING_MAX_WALL_STEER_DEG,
                PARKING_PID_DEGRADED_MAX_STEER_DEG,
            )
        else:
            steer_limit = PARKING_MAX_WALL_STEER_DEG

        if abs(steering) < PARKING_PID_STEERING_DEADBAND_DEG:
            steering = 0.0
        steering = _parking_clamp(steering, -steer_limit, steer_limit)

        # an earlier revision: slew-rate limit. The servo cannot execute a step command anyway,
        # and limiting the rate stops the controller from chasing sensor noise.
        if last_steering_time is not None:
            elapsed = now - last_steering_time
            if elapsed > 0.0:
                max_change = PARKING_PID_MAX_STEER_RATE_DEG_PER_SEC * elapsed
                steering = _parking_clamp(
                    steering,
                    last_steering - max_change,
                    last_steering + max_change,
                )
        last_steering_time = now

        parking_set_motion(speed, steering, bus=bus)
        last_steering = steering

        stats.add(
            equation_value,
            angle_term,
            (float(a_value) + float(b_value)) / 2.0,
            steering,
            degraded=degraded,
        )

        # Parking-wall detection uses the RAW A value so there is no filter
        # lag, but This program requires it to repeat so one spurious short reading
        # cannot stop the car in the wrong place.
        #
        # An earlier revision also requires a STEP: the bay must make A read meaningfully
        # closer than the wall B is still tracking. Without this, drifting into
        # the followed wall looks identical to finding the bay. Filtered B is
        # the stable reference; raw A stays the fast detector.
        wall_step_mm = float(b_value) - float(original_a)
        step_ok = (
            not PARKING_WALL_FIND_REQUIRE_STEP
            or wall_step_mm >= PARKING_WALL_FIND_MIN_STEP_MM
        )
        if a_valid and original_a <= wall_found_threshold and step_ok:
            wall_confirmed += 1
        else:
            wall_confirmed = 0
            # Being close enough but with no step is the drift case. Say so,
            # because otherwise it looks like the detector is simply not firing.
            if (
                a_valid
                and original_a <= wall_found_threshold
                and not step_ok
                and now - last_print_time >= PARKING_TOF_PRINT_INTERVAL
            ):
                print(
                    f"PARKING WALL REJECTED | A={original_a:.1f} is close but "
                    f"step B-A={wall_step_mm:+.1f} mm < "
                    f"{PARKING_WALL_FIND_MIN_STEP_MM:.1f} mm. This is drift "
                    f"into the followed wall, not the parking bay.",
                    flush=True,
                )

        if wall_confirmed >= wall_confirm_needed:
            parking_stop_drive_immediately(bus=bus)
            parking_run_position(DXL_STEER_ID, 80, -1.0, bus=bus)
            print(
                f"Parking wall found with A={original_a:.1f} mm "
                f"(confirmed {wall_confirmed}x). "
                "Stopped abruptly and steering set 1 degree left.",
                flush=True,
            )
            if PARKING_PID_REPORT_TRACKING_ERROR:
                stats.report("\nStage 4 summary (wall found):")
            return

        if now - last_print_time >= PARKING_TOF_PRINT_INTERVAL:
            if steering > 0:
                direction = "RIGHT"
            elif steering < 0:
                direction = "LEFT"
            else:
                direction = "STRAIGHT"
            print(
                f"frame={frame_reading['frame']} | A={a_value:6.1f} mm | "
                f"B={b_value:6.1f} mm | ang={angle_term:+6.1f} dist={distance_term:+6.1f} | "
                f"error={equation_value:7.2f} | "
                f"P={p_term:7.2f} I={i_term:7.2f} D={d_term:7.2f} | "
                f"steer={steering:6.1f} | {direction} | "
                f"wall {wall_confirmed}/{wall_confirm_needed}",
                flush=True,
            )
            if a_source != "ok" or b_source != "ok":
                print(
                    "PARKING SENSOR FALLBACK | "
                    f"A={original_a:.1f} SA={frame.get('SA')} -> {a_value:.1f} ({a_source}) | "
                    f"B={original_b:.1f} SB={frame.get('SB')} -> {b_value:.1f} ({b_source})",
                    flush=True,
                )
            last_print_time = now

    parking_stop_drive_immediately(bus=bus)
    parking_run_position(DXL_STEER_ID, 80, 0, bus=bus)
    if PARKING_PID_REPORT_TRACKING_ERROR:
        stats.report("\nStage 4 summary (TIMED OUT before finding the wall):")
    raise RuntimeError("Parking PID ToF_run timed out before finding the wall")


def parking_entry_cw(bus=None):
    bus = bus or get_dxl_bus()
    print("\nPARKING STAGE 5: Entering the parking lot", flush=True)
    parking_stop(bus=bus)

    parking_run_position(DXL_STEER_ID, 50, -3, bus=bus)
    parking_run_for_degrees(_parking_throttle_id(), 80, 1250, bus=bus)
    time.sleep(0.15)
    parking_run_position(DXL_STEER_ID, 80, 45, bus=bus)
    parking_run_for_degrees(_parking_throttle_id(), 80, -750, bus=bus)
    time.sleep(0.15)
    parking_run_position(DXL_STEER_ID, 80, -6.25, bus=bus)
    parking_run_for_degrees(_parking_throttle_id(), 80, 200, bus=bus)
    time.sleep(0.15)
    parking_run_position(DXL_STEER_ID, 80, -50, bus=bus)
    parking_run_for_degrees(_parking_throttle_id(), 80, -300, bus=bus)
    time.sleep(0.15)
    parking_run_position(DXL_STEER_ID, 80, 50, bus=bus)
    parking_run_for_degrees(_parking_throttle_id(), 80, 180, bus=bus)
    time.sleep(0.15)
    parking_run_position(DXL_STEER_ID, 80, -50, bus=bus)
    parking_run_for_degrees(_parking_throttle_id(), 80, -280, bus=bus)
    time.sleep(0.15)
    parking_run_position(DXL_STEER_ID, 80, 50, bus=bus)
    parking_run_for_degrees(_parking_throttle_id(), 80, 150, bus=bus)
    time.sleep(0.15)
    parking_run_position(DXL_STEER_ID, 80, -50, bus=bus)
    parking_run_for_degrees(_parking_throttle_id(), 80, -180, bus=bus)
    time.sleep(0.15)
    parking_run_position(DXL_STEER_ID, 80, 50, bus=bus)
    parking_run_for_degrees(_parking_throttle_id(), 80, 200, bus=bus)

    print("Parking-entry movement complete.", flush=True)


def run_parking_sequence_after_obstacle(drive_mode: str):
    """Run pasted CW parking sequence after obstacle AI/gyro finish."""
    mode = str(drive_mode).upper()
    if not PARKING_ENABLED:
        print("Parking integration disabled; skipping parking sequence.", flush=True)
        return False
    if mode not in PARKING_ENABLED_MODES:
        print(f"{mode}: no integrated parking sequence enabled; skipping.", flush=True)
        return False

    bus = get_dxl_bus()
    tof = None
    print("\nIntegrated parking sequence starting.", flush=True)
    print(
        f"Using parking steering center {PARKING_STEERING_CENTER_TICKS} ticks, "
        f"parking offset {PARKING_STEERING_OFFSET_DEG} deg, "
        f"Pico port {PARKING_PICO_PORT}.",
        flush=True,
    )

    try:
        restore_throttle_velocity_mode(bus=bus)
        parking_stop_drive_immediately(bus=bus)
        parking_run_position(DXL_STEER_ID, 80, 0, bus=bus)
        tof = ParkingPicoToF(PARKING_PICO_PORT)
        print(f"Parking Pico connected on {tof.port_name}.", flush=True)

        if PARKING_SKIP_TO_C_TOF_STAGE:
            print("Skipping standalone parking Stage 1; starting at C ToF stage.", flush=True)
        else:
            parking_turn_left_for_degrees(PARKING_TURN_MOTOR_DEGREES, bus=bus)

        parking_wait_until_front_wall(tof, bus=bus)
        parking_turn_backwards_before_parking(mode, PARKING_TURN_MOTOR_DEGREES, bus=bus)
        ab_balance_ok = parking_balance_ab_backwards(tof, bus=bus)
        if not ab_balance_ok:
            print(
                "A/B alignment timed out; starting PID ToF_run anyway.",
                flush=True,
            )
        parking_tof_run(tof, bus=bus)
        parking_entry_cw(bus=bus)

        parking_stop_drive_immediately(bus=bus)
        parking_run_position(DXL_STEER_ID, 80, 0, bus=bus)
        print("Integrated parking sequence finished successfully.", flush=True)
        return True

    except KeyboardInterrupt:
        print("\nCtrl+C detected during integrated parking sequence.", flush=True)
        parking_stop_drive_immediately(bus=bus)
        try:
            parking_run_position(DXL_STEER_ID, 80, 0, bus=bus)
        except Exception:
            pass
        raise

    except Exception as error:
        # an earlier revision: stop the car first, then say why. Every parking stage raises
        # RuntimeError on sensor failure, and without this the message is just
        # "did not receive valid frames" with no indication of which layer.
        print(f"\nParking sequence failed: {error}", flush=True)
        parking_stop_drive_immediately(bus=bus)
        try:
            parking_run_position(DXL_STEER_ID, 80, 0, bus=bus)
        except Exception:
            pass
        if tof is not None:
            try:
                tof.link_report()
            except Exception:
                pass
        raise

    finally:
        if tof is not None:
            try:
                tof.close()
            except Exception as error:
                print(f"Parking Pico shutdown warning: {error}", flush=True)


def obstacle_start_program(drive_mode: str):
    """Exit obstacle parking, then hand control to the trained model.

    The maneuver intentionally uses the reusable motor functions directly:
        run_position(motor_id, speed, angle)
        run_for_degrees(motor_id, speed, degrees)

    OCW and OCCW use different travel distances. OCCW is shorter so the car
    stays behind the traffic-signal intersection line before model takeover.
    """
    mode = str(drive_mode).upper()
    if mode not in ("OCW", "OCCW"):
        return False

    bus = get_dxl_bus()
    print(f"{mode}: parking-exit start program running.", flush=True)
    time.sleep(0.50)

    try:
        if mode == "OCW":
            # Turn right and leave the parking space.
            run_position(DXL_STEER_ID, 50, 65, bus=bus)
            time.sleep(0.15)
            run_for_degrees(DXL_THROTTLE_IDS, 50, 400, bus=bus)

            # Turn left to straighten the car.
            run_position(DXL_STEER_ID, 50, -50, bus=bus)
            time.sleep(0.15)
            run_for_degrees(DXL_THROTTLE_IDS, 50, 400, bus=bus)

            # Center and move far enough for a clean model handoff.
            run_position(DXL_STEER_ID, 50, 0, bus=bus)
            time.sleep(0.15)
#             run_for_degrees(DXL_THROTTLE_IDS, 100, 19, bus=bus)

        else:  # OCCW
            # Shorter conservative exit to remain behind the traffic-signal line.
            run_position(DXL_STEER_ID, 50, -65, bus=bus)
            time.sleep(0.15)
            run_for_degrees(DXL_THROTTLE_IDS, 50, 430, bus=bus)

            run_position(DXL_STEER_ID, 50, 60, bus=bus)
            time.sleep(0.15)
            run_for_degrees(DXL_THROTTLE_IDS, 50, 450, bus=bus)

            run_position(DXL_STEER_ID, 50, -7, bus=bus)
            time.sleep(0.15)
            run_for_degrees(DXL_THROTTLE_IDS, -50, 1500, bus=bus)
            
            run_position(DXL_STEER_ID, 50, 15, bus=bus)
            time.sleep(0.15)
            run_for_degrees(DXL_THROTTLE_IDS, -50, 600, bus=bus)

    finally:
        # The model drives with velocity mode after the position-based startup.
        restore_throttle_velocity_mode(bus=bus)
        run_position(DXL_STEER_ID, 100, 0, bus=bus)

    time.sleep(0.15)
    print("Obstacle parking exit complete. Model now controls the car.", flush=True)
    return True
# ---------------------------------------------------------------------------
# Camera parameters
# ---------------------------------------------------------------------------
# An earlier revision fast-start setting. An earlier revision waited 2.0 seconds after creating the PiCamera.
# Lowering this reduces startup time before the model begins. If the first camera
# frames look unstable, raise this back toward 2.0.
CAMERA_STARTUP_SETTLE_SECONDS = 0.75

CAMERA_FRAMERATE             = 30
PICAMERA_AWB_MODE            = 'off'
PICAMERA_EXPOSURE_MODE       = 'off'
PICAMERA_ISO                 = 100
PICAMERA_SHUTTER_SPEED       = 15000
PICAMERA_AWB_GAINS           = (1.5, 1.2)
PICAMERA_EXPOSURE_COMPENSATION = 0


# ---------------------------------------------------------------------------
# Helpers
# ---------------------------------------------------------------------------
def center_crop(img, tw=CROP_W, th=CROP_H):
    h, w = img.shape[:2]
    x0 = (w - tw) // 2
    y0 = (h - th) // 2 - 6
    return img[y0:y0 + th, x0:x0 + tw]


# ---------------------------------------------------------------------------
# Hardware parts
# ---------------------------------------------------------------------------
class DynamixelSteering:
    """Single steering motor using DYNAMIXEL / XL330 position mode."""
    def __init__(self, motor_id=DXL_STEER_ID,
                 left=DXL_STEER_LEFT_DEG,
                 right=DXL_STEER_RIGHT_DEG):
        self.bus = get_dxl_bus()
        self.bus.register_part()
        self.id = int(motor_id)
        self.left = float(left)
        self.right = float(right)
        self.prev_goal = None
        self.closed = False

        steer_profile_velocity = _rpm_to_velocity_lsb(STEERING_MAX_SPEED)
        self.bus.configure_motor(
            self.id,
            MODE_POSITION,
            profile_accel=DXL_STEER_PROFILE_ACCEL,
            profile_velocity=steer_profile_velocity
        )
        time.sleep(0.1)
        self._write_goal(DXL_STEER_CENTER_TICKS)

    def _angle_to_goal_ticks(self, angle: float) -> int:
        angle = angle * angle_offset
        angle = max(min(angle, 1.0), -1.0)

        # Same left/right style as the old BuildHAT class:
        # -1 -> left, 0 -> center, +1 -> right.
        steer_deg = self.left + (angle + 1) * (self.right - self.left) / 2
        goal = DXL_STEER_CENTER_TICKS + DXL_STEER_DIRECTION * steer_deg * DXL_TICKS_PER_DEG

        # XL330 normal position mode range is 0 to 4095.
        return int(max(0, min(4095, round(goal))))

    def _write_goal(self, goal_ticks: int):
        if goal_ticks == self.prev_goal:
            return
        self.bus.write4(self.id, ADDR_GOAL_POSITION, goal_ticks, "set steering goal")
        self.prev_goal = goal_ticks

    def run(self, angle: float):
        goal_ticks = self._angle_to_goal_ticks(angle)
        self._write_goal(goal_ticks)

    def shutdown(self):
        if self.closed:
            return
        self.closed = True
        try:
            self._write_goal(DXL_STEER_CENTER_TICKS)
            time.sleep(0.25)
            self.bus.write1(self.id, ADDR_TORQUE_ENABLE, TORQUE_OFF, "steering torque off")
        finally:
            self.bus.release_part()


class DynamixelThrottle:
    """Drive one or more DYNAMIXEL / XL330 motors using velocity mode."""
    def __init__(self, motor_ids=DXL_THROTTLE_IDS, max_speed=MAX_SPEED_PERCENT):
        self.bus = get_dxl_bus()
        self.bus.register_part()
        self.ids = [int(i) for i in motor_ids]
        self.max_speed = int(max(min(max_speed, 100), 0))
        self.last_speed = None
        self.closed = False

        for dxl_id in self.ids:
            self.bus.configure_motor(
                dxl_id,
                MODE_VELOCITY,
                profile_accel=DXL_THROTTLE_PROFILE_ACCEL,
                profile_velocity=None
            )
        self._stop()

    def _direction_for_index(self, i: int) -> int:
        if i < len(DXL_THROTTLE_DIRECTIONS):
            return -1 if DXL_THROTTLE_DIRECTIONS[i] < 0 else 1
        return 1

    def _stop(self):
        print('run stop!')
        for dxl_id in self.ids:
            self.bus.write4(dxl_id, ADDR_GOAL_VELOCITY, 0, "stop throttle")

    def run(self, throttle: float):
        throttle = max(min(throttle, 1.0), -1.0)
        speed = int(throttle * self.max_speed)
        speed = int(round(speed / 10.0) * 10)

        if self.last_speed == 0 and speed == 0:
            return

        if self.last_speed != 0 and speed == 0:
            self._stop()
            self.last_speed = speed
            return

        if speed != 0 and speed == self.last_speed:
            return

        if speed != 0 and speed != self.last_speed:
            rpm = DXL_THROTTLE_MAX_RPM * (speed / 100.0)
            velocity_lsb = _rpm_to_velocity_lsb(rpm)
            for i, dxl_id in enumerate(self.ids):
                goal_velocity = velocity_lsb * self._direction_for_index(i)
                self.bus.write4(dxl_id, ADDR_GOAL_VELOCITY, goal_velocity,
                                "set throttle velocity")

        self.last_speed = speed

    def shutdown(self):
        if self.closed:
            return
        self.closed = True
        try:
            self._stop()
            time.sleep(0.05)
            for dxl_id in self.ids:
                self.bus.write1(dxl_id, ADDR_TORQUE_ENABLE, TORQUE_OFF,
                                "throttle torque off")
        finally:
            self.bus.release_part()


class PS4Joystick:
    """Read the PS4 controller and generate one erase event per press.

    Some controllers briefly flicker between pressed and released states. The
    release debounce below prevents one physical Triangle press from producing
    several erase requests. Triangle must remain released for 0.25 seconds
    before another deletion can be requested.
    """

    TRIANGLE_BUTTON = 2
    STOP_BUTTON = 4
    TRIANGLE_RELEASE_DEBOUNCE = 0.25

    def __init__(self, deadzone=JOYSTICK_DEADZONE):
        pygame.init(); pygame.joystick.init()
        if pygame.joystick.get_count() == 0:
            raise RuntimeError("No PS4 controller detected.")
        self.js = pygame.joystick.Joystick(0); self.js.init()
        self.dz = deadzone

        # Triangle begins armed. After one erase request, it remains disarmed
        # until the button has been continuously released for the debounce time.
        self._triangle_armed = True
        self._triangle_release_started = None

        print(f"Connected joystick: {self.js.get_name()}", flush=True)

    def _dz(self, v):
        return 0.0 if abs(v) < self.dz else float(v)

    def run(self) -> Tuple[float, float, str]:
        pygame.event.pump()
        angle = self._dz(self.js.get_axis(0))
        throttle = -self._dz(self.js.get_axis(4))
        now = time.monotonic()

        # L1 always has priority, even if Triangle is being held or glitches.
        if self.js.get_button(self.STOP_BUTTON):
            raise KeyboardInterrupt

        triangle_down = bool(self.js.get_button(self.TRIANGLE_BUTTON))

        if triangle_down:
            self._triangle_release_started = None

            if self._triangle_armed:
                self._triangle_armed = False
                print(
                    "Triangle detected: deleting the newest 100 records...",
                    flush=True,
                )
                # A single-loop event consumed by PromptWiper.
                return angle, 0.0, "erase"

            # Keep the car stopped and recording disabled while Triangle remains
            # held, but do not send another deletion request.
            return angle, 0.0, "erase_hold"

        # Rearm only after Triangle has stayed released for a short time.
        if not self._triangle_armed:
            if self._triangle_release_started is None:
                self._triangle_release_started = now
            elif now - self._triangle_release_started >= self.TRIANGLE_RELEASE_DEBOUNCE:
                self._triangle_armed = True
                self._triangle_release_started = None

        return angle, throttle, "user"

    def shutdown(self):
        pygame.quit()


# ---------------------------------------------------------------------------
# Display and control parts for driving modes
# ---------------------------------------------------------------------------
class ConsoleDisplay:
    def __init__(self):
        self.last_t = 0

    def run(self, angle: float, throttle: float):
        t = time.monotonic()
        if t - self.last_t >= 1.0:
            angle = 0.0 if angle is None else float(angle)
            throttle = 0.0 if throttle is None else float(throttle)
            print(f"Pred -> angle {angle:+.2f}  thr {throttle:+.2f}")
            self.last_t = t


class CameraViewer:
    """Show the cropped camera input during recording or view-enabled driving."""
    def __init__(self, width=CAMERA_VIEW_W, height=CAMERA_VIEW_H):
        pygame.init()
        self.size = (int(width), int(height))
        self.screen = pygame.display.set_mode(self.size)
        pygame.display.set_caption("WRO FE 2026 - Live Driving Camera")
        self.closed = False

    def run(self, image):
        if image is None or self.closed:
            return

        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                raise KeyboardInterrupt
            if event.type == pygame.KEYDOWN and event.key == pygame.K_ESCAPE:
                raise KeyboardInterrupt

        frame = np.asarray(image)
        if frame.ndim != 3 or frame.shape[2] < 3:
            return

        frame = frame[:, :, :3]
        if frame.dtype != np.uint8:
            frame = np.clip(frame, 0, 255).astype(np.uint8)
        frame = np.ascontiguousarray(frame)

        # pygame.surfarray expects width x height x channels.
        surface = pygame.surfarray.make_surface(np.transpose(frame, (1, 0, 2)))
        surface = pygame.transform.smoothscale(surface, self.size)
        self.screen.blit(surface, (0, 0))
        pygame.display.flip()

    def shutdown(self):
        if self.closed:
            return
        self.closed = True
        try:
            pygame.display.quit()
        except Exception:
            pass


class ObstacleStartController:
    """Run the parking-exit start program once before model control begins."""
    def __init__(self, drive_mode: str):
        self.drive_mode = str(drive_mode).upper()
        self.required = self.drive_mode in ("OCW", "OCCW")
        self.finished = False

        if self.required:
            print(
                f"{self.drive_mode}: parking-exit start program armed."
            )
        else:
            print(f"{self.drive_mode}: no parking-exit program required.")

    def run(self):
        if self.finished or not self.required:
            return 0.0, 0.0, False

        # This deliberately blocks the vehicle loop while run_for_degrees()
        # performs the complete physical startup maneuver. The camera capture
        # thread remains alive; model control begins immediately afterward.
        obstacle_start_program(self.drive_mode)
        self.finished = True
        return 0.0, 0.0, False


class DriveControlMux:
    """Use startup controls while active, otherwise pass through model controls."""
    def run(self, start_angle, start_throttle, start_active,
            pilot_angle, pilot_throttle):
        if bool(start_active):
            return float(start_angle), float(start_throttle)

        angle = 0.0 if pilot_angle is None else float(pilot_angle)
        throttle = 0.0 if pilot_throttle is None else float(pilot_throttle)
        return angle, throttle


class PicoWT901YawReader:
    """Read WT901 yaw from the Raspberry Pi Pico text stream.

    The WT901 is connected to the Pico. The Pico should send lines that include
    an angle/yaw field, for example:
        Angle=12.34, A=50, B=51, C=100, D=75

    This class only opens the Pico USB serial port and parses Angle/Yaw values.
    The controller unwraps those values across +/-180 or 0/360 wrap boundaries.
    """

    # an earlier revision: shared with ensure_pico_streaming() so the startup auto-fix and the
    # reader can never disagree about what counts as a valid Angle frame.
    ANGLE_PATTERN = PICO_ANGLE_RE
    FIRST_VALUE_PATTERN = PICO_FIRST_VALUE_RE

    def __init__(self, port=PICO_GYRO_PORT, baudrate=PICO_GYRO_BAUD,
                 soft_reboot=None):
        if serial is None:
            raise RuntimeError("pyserial is required for Pico WT901 serial input")

        # an earlier revision: soft_reboot=None follows PICO_REBOOT_WHEN_GYRO_READER_OPENS.
        # Recovery reopens pass False once PICO_AUTO_SOFT_REBOOT_MAX_ATTEMPTS is
        # used up. An earlier revision printed that it would stop rebooting but rebooted anyway.
        if soft_reboot is None:
            soft_reboot = bool(PICO_REBOOT_WHEN_GYRO_READER_OPENS)
        self.soft_reboot_on_open = bool(soft_reboot)

        self.port_name = port
        self.baudrate = int(baudrate)
        self.serial = serial.Serial(
            self.port_name,
            self.baudrate,
            timeout=PICO_GYRO_SERIAL_TIMEOUT,
        )
        # an earlier revision: DTR must stay asserted. MicroPython on RP2 only writes stdout to
        # the USB CDC interface when tud_cdc_connected() is true, and that flag
        # is the DTR line state. An earlier revision cleared DTR here, so every print() on the
        # Pico was discarded and no Angle frame could ever reach the Pi.
        try:
            self.serial.dtr = True
            self.serial.rts = False
        except Exception:
            pass
        self.text_buffer = ""
        self.last_yaw_deg = None
        self.last_line = ""
        self.last_unparsed_line = ""
        self.last_frame_time = 0.0
        self.frame_number = 0
        self.lines_seen = 0
        self.unparsed_lines_seen = 0
        self.bytes_received = 0

        if self.soft_reboot_on_open:
            self._soft_reboot_pico_main_on_open()
        else:
            time.sleep(0.25)
            self.serial.reset_input_buffer()

        print(
            f"Pico WT901 stream opened on {self.port_name} at "
            f"{self.baudrate} baud. "
            f"soft_reboot_on_open={self.soft_reboot_on_open}",
            flush=True,
        )

        # an earlier revision: listen-then-escape. Try a short passive listen first, because a
        # healthy Pico is already streaming and must not be written to. Only if
        # it stays silent - which proves main.py is not printing - send the
        # Ctrl-C/Ctrl-B/Ctrl-D escape and try again.
        #
        # An earlier revision instead waited the full PICO_GYRO_FIRST_FRAME_TIMEOUT_SECONDS
        # here and left all recovery to the sampling thread. The captured run
        # spent 35 seconds driving with no gyro before giving up, and never
        # escaped raw REPL because the sequence lacked Ctrl-B.
        self.first_frame_ok = self._open_handshake()
        if not self.first_frame_ok and PICO_GYRO_REQUIRE_FIRST_FRAME:
            try:
                self.serial.close()
            except Exception:
                pass
            raise RuntimeError(
                "Pico WT901 produced no Angle frame within "
                f"{PICO_GYRO_FIRST_FRAME_TIMEOUT_SECONDS:.1f} seconds on "
                f"{self.port_name}."
            )

    def _open_handshake(self):
        """Get the Pico streaming Angle frames, escaping a stale REPL if needed.

        Order matters:
          1. Listen silently for PICO_GYRO_PASSIVE_LISTEN_SECONDS. A Pico whose
             main.py is running streams immediately, and writing to it would
             break it (an earlier revision's finding), so the healthy path never writes at all.
          2. If it is silent, main.py is definitively not printing, so writing
             is now safe. Send Ctrl-C/Ctrl-B/Ctrl-D and wait for the boot.
          3. Repeat up to PICO_GYRO_ESCAPE_ATTEMPTS_ON_OPEN times.

        Returns True as soon as a real Angle frame is parsed.
        """
        if self.wait_for_first_frame(
            PICO_GYRO_PASSIVE_LISTEN_SECONDS,
            quiet=True,
        ):
            return True

        attempts = max(0, int(PICO_GYRO_ESCAPE_ATTEMPTS_ON_OPEN))
        for attempt in range(1, attempts + 1):
            if self.saw_raw_repl_banner():
                remedy = (
                    "Sending Ctrl-B to leave raw REPL, then Ctrl-D to reboot."
                    if PICO_RESTART_SEND_CTRL_B
                    else "PICO_RESTART_SEND_CTRL_B is False, so this CANNOT be "
                         "escaped - Ctrl-D does not reboot from raw REPL. "
                         "Set it True."
                )
                print(
                    "Pico WT901: the Pico is sitting in MicroPython RAW REPL, so "
                    "main.py is not running. Something left it there - Thonny, "
                    "mpremote, ampy or rshell all enter raw REPL to copy files. "
                    + remedy,
                    flush=True,
                )
            else:
                print(
                    f"Pico WT901: silent for "
                    f"{PICO_GYRO_PASSIVE_LISTEN_SECONDS:.1f} s, so main.py is not "
                    f"streaming. Escape attempt {attempt}/{attempts}.",
                    flush=True,
                )

            self._soft_reboot_pico_main_on_open()

            if self.wait_for_first_frame(
                PICO_GYRO_WAIT_AFTER_ESCAPE_SECONDS,
                quiet=(attempt < attempts),
            ):
                print(
                    f"Pico WT901: recovered on escape attempt {attempt}.",
                    flush=True,
                )
                return True

        return False

    def wait_for_first_frame(self, timeout_seconds=None, quiet=False):
        """Block until the Pico sends a parsable Angle frame, or time out.

        an earlier revision had no handshake. The sampling thread simply gave the Pico 1.20 s
        and then declared failure, which was shorter than the Pico's own boot
        sequence. This method waits for the real thing and, on failure, prints
        counters that identify which layer is broken:

            bytes_received == 0
                Nothing arrives at all. Suspect DTR, the wrong port, another
                process holding /dev/ttyACM0, or Pico main.py exiting early.
            bytes_received > 0 and lines_seen > 0 and frame_number == 0
                The Pico prints boot text and then stops. Look at
                last_unparsed_line for a [TOF FATAL] or [FATAL] message.
            frame_number > 0
                The link is healthy.

        an earlier revision: pass quiet=True for the short passive-listen probes so the log is
        not filled with WARNING blocks for a probe that is expected to fail.
        """
        if timeout_seconds is None:
            timeout_seconds = PICO_GYRO_FIRST_FRAME_TIMEOUT_SECONDS

        deadline = time.monotonic() + float(timeout_seconds)
        started = time.monotonic()

        while time.monotonic() < deadline:
            yaw_deg = self.read_yaw()
            if yaw_deg is not None and math.isfinite(yaw_deg):
                elapsed = time.monotonic() - started
                print(
                    f"Pico WT901 first Angle frame after {elapsed:.2f} s: "
                    f"{self.last_line[:PICO_GYRO_PRINT_LINE_MAX_CHARS]}",
                    flush=True,
                )
                return True
            time.sleep(0.02)

        if quiet:
            return False

        snapshot = self.get_debug_snapshot()
        print(
            f"Pico WT901 WARNING: no Angle frame within {timeout_seconds:.1f} s. "
            f"bytes={snapshot['bytes_received']} "
            f"lines={snapshot['lines_seen']} "
            f"parsed={snapshot['frame']} "
            f"unparsed={snapshot['unparsed_lines_seen']}",
            flush=True,
        )
        if snapshot.get("last_unparsed_line"):
            print(
                "Last unparsed Pico line: "
                f"{snapshot['last_unparsed_line'][:PICO_GYRO_PRINT_LINE_MAX_CHARS]}",
                flush=True,
            )

        # an earlier revision: raw REPL is checked first because it is both the most common
        # cause here and the one whose symptoms look like the other two.
        if self.saw_raw_repl_banner():
            print(
                "Pico WT901 hint: the Pico is stuck in MicroPython RAW REPL and "
                "main.py is NOT running. Ctrl-D alone cannot fix this - in raw "
                "REPL it means 'execute buffer', not 'soft reboot'. Ctrl-B is "
                "required first (PICO_RESTART_SEND_CTRL_B). Something left the "
                "Pico in raw REPL: close Thonny, or run "
                "'mpremote reset', or just unplug/replug the Pico.",
                flush=True,
            )
        elif int(snapshot["bytes_received"]) == 0:
            print(
                "Pico WT901 hint: zero bytes received. Either DTR is not "
                "asserted, another program holds the port, Pico main.py exited "
                "during startup, or the Pico is in raw REPL, which stays "
                "silent until it is written to.",
                flush=True,
            )
        elif int(snapshot["frame"]) == 0:
            print(
                "Pico WT901 hint: the Pico printed boot text but no Angle "
                "frames. Look for [TOF FATAL] or [FATAL] core1 above.",
                flush=True,
            )
        return False

    def _soft_reboot_pico_main_on_open(self):
        """Escape any REPL state and soft-reboot Pico main.py, port kept open.

        an earlier revision rebooted the Pico at the top of manage, then closed the port. That
        could leave the Pico printing for several seconds before the gyro reader
        started draining data. an earlier revision reboots here instead, so manage is already
        holding and draining /dev/ttyACM0 as soon as main.py starts streaming.

        This program adds Ctrl-B, which is what actually fixes the recurring failure.
        The full sequence is now:

            Ctrl-C  x2   interrupt whatever is running
            Ctrl-B       leave RAW repl and return to the FRIENDLY repl
            Ctrl-D       soft reboot -> boot.py / main.py run

        Without the Ctrl-B step this sequence is a no-op against a Pico sitting
        in raw REPL, because raw REPL interprets Ctrl-D as "execute the buffer"
        rather than "soft reboot". See PICO_RESTART_SEND_CTRL_B above.
        """
        sequence = "Ctrl-C + Ctrl-B + Ctrl-D" if PICO_RESTART_SEND_CTRL_B else "Ctrl-C + Ctrl-D"
        print(
            f"Pico WT901: sending {sequence} with gyro serial kept open...",
            flush=True,
        )
        try:
            time.sleep(PICO_RESTART_OPEN_WAIT_SECONDS)
            # an earlier revision: one shared implementation, also used by the startup
            # ensure_pico_streaming() auto-fix.
            pico_send_escape_sequence(self.serial)
            self.text_buffer = ""
        except Exception as error:
            print(
                f"Pico WT901 soft reboot-on-open warning: {error}",
                flush=True,
            )

    def saw_raw_repl_banner(self):
        """True if the Pico has announced that it is sitting in raw REPL."""
        text = f"{self.last_unparsed_line} {self.last_line}"
        return any(marker in text for marker in PICO_RAW_REPL_MARKERS)

    def _parse_yaw_from_line(self, line):
        match = self.ANGLE_PATTERN.search(line)
        if match:
            return float(match.group(1))

        # An earlier revision also supports compact ordered Pico output:
        #     Angle, A, B, C, D
        # Example: 12.34, 50, 51, 100, 75
        first = self.FIRST_VALUE_PATTERN.search(line)
        if first:
            return float(first.group(1))

        return None

    def read_yaw(self):
        """Return the newest Pico-reported WT901 yaw angle in degrees."""
        try:
            chunk = self.serial.read(256)
        except Exception as exc:
            print(f"Pico WT901 serial read failed: {exc}", flush=True)
            return None

        if chunk:
            self.bytes_received += len(chunk)
            self.text_buffer += chunk.decode("utf-8", errors="ignore")

        newest_yaw = None
        while "\n" in self.text_buffer:
            line, self.text_buffer = self.text_buffer.split("\n", 1)
            line = line.strip()
            if not line:
                continue

            self.lines_seen += 1
            try:
                yaw_deg = self._parse_yaw_from_line(line)
            except ValueError:
                yaw_deg = None

            if yaw_deg is not None and math.isfinite(yaw_deg):
                newest_yaw = yaw_deg
                self.last_yaw_deg = yaw_deg
                self.last_line = line
                self.last_frame_time = time.monotonic()
                self.frame_number += 1
            else:
                self.last_unparsed_line = line
                self.unparsed_lines_seen += 1

        # Keep the buffer from growing forever if the Pico sends partial/no-newline data.
        if len(self.text_buffer) > 512:
            self.text_buffer = self.text_buffer[-512:]

        return newest_yaw

    def get_debug_snapshot(self):
        """Return the newest parsed Pico/WT901 values for terminal printing."""
        last_time = float(self.last_frame_time or 0.0)
        age_seconds = None
        if last_time > 0.0:
            age_seconds = time.monotonic() - last_time
        return {
            "yaw_deg": self.last_yaw_deg,
            "line": self.last_line,
            "frame": int(self.frame_number),
            "age_seconds": age_seconds,
            "lines_seen": int(self.lines_seen),
            "unparsed_lines_seen": int(self.unparsed_lines_seen),
            "bytes_received": int(self.bytes_received),
            "last_unparsed_line": self.last_unparsed_line,
        }

    def close(self):
        try:
            if self.serial and self.serial.is_open:
                self.serial.close()
                print("Pico WT901 serial stream closed.", flush=True)
        except Exception:
            pass


class GyroThreeLapController:
    """Stop autonomous driving by Pico-reported WT901 yaw, but only after a time lockout.

    This program uses the Pico WT901 text stream with a Pico serial stream
    carrying WT901 Angle/Yaw values. The reader runs in its own 100 Hz thread,
    unwraps yaw across +/-180 or 0/360 boundaries, and stores both raw
    accumulated yaw and calibrated yaw.

    The seconds value only unlocks the gyro stop. After that time, the model
    keeps driving until the calibrated WT901 yaw target is reached and confirmed.

    During the finish delay, the controller keeps passing the model's predicted
    angle/throttle through. It does not force the robot to drive straight.
    """

    def __init__(self, drive_mode: str):
        self.drive_mode = str(drive_mode).upper()
        self.wt901 = PicoWT901YawReader(PICO_GYRO_PORT, PICO_GYRO_BAUD)

        self.raw_total_rotation_deg = 0.0
        self.raw_current_rate_deg_per_sec = 0.0
        self.total_rotation_deg = 0.0
        self.current_rate_deg_per_sec = 0.0
        self.finish_deadline = None
        self.model_start_time = None
        self.gyro_target_first_seen = None
        self.stopped = False
        self.last_print_time = 0.0
        self.finish_turn_done = False
        self.read_error_printed = False

        self._last_wt901_yaw_deg = None
        self._last_wt901_time = None
        self._zero_printed = False
        self._last_frame_received_time = 0.0
        self._auto_reopen_attempts = 0
        self._last_auto_reopen_time = 0.0
        # The Pico's Angle field is already an integrated absolute yaw,
        # so (Angle_now - Angle_at_zero) is the sensor's own opinion of how far
        # the car turned. Comparing that against our accumulated total isolates
        # how much the Pi-side deadband and rate rejections changed the answer.
        # Without this you cannot tell a Pico problem from a manage problem.
        self._angle_at_zero = None
        self._last_raw_angle = None

        self._lock = threading.Lock()
        self._stop_event = threading.Event()
        self._sample_thread = None
        self._sampling_started = False

        # an earlier revision: resolve the speed-scaled timings ONCE, at construction, so the
        # whole run uses one consistent number even if a constant is edited.
        self.lockout_seconds = gyro_lockout_seconds()

        if self.drive_mode in ("OCW", "OCCW"):
            self.finish_seconds = obstacle_finish_seconds_scaled()
            run_name = "obstacle"
        else:
            self.finish_seconds = FREE_RUN_FINISH_SECONDS
            run_name = "free"

        if GYRO_LOCKOUT_AUTO_SCALE_WITH_SPEED:
            print(
                f"Gyro timing scaled for speed: MAX_SPEED_PERCENT="
                f"{MAX_SPEED_PERCENT:.0f} vs reference "
                f"{GYRO_LOCKOUT_REFERENCE_SPEED_PERCENT:.0f} -> lockout "
                f"{GYRO_IGNORE_STOP_UNTIL_SECONDS:.1f}s becomes "
                f"{self.lockout_seconds:.1f}s, obstacle finish "
                f"{OBSTACLE_RUN_FINISH_SECONDS:.1f}s becomes "
                f"{self.finish_seconds:.1f}s. Degrees are unchanged.",
                flush=True,
            )

        print(
            f"Pico WT901 gyro-lockout {run_name} run armed: ignoring gyro stop for "
            f"{self.lockout_seconds:.1f} seconds, then waiting for "
            f"{GYRO_TARGET_DEG:.1f} calibrated Pico WT901 yaw degrees. "
            f"Finish delay is {self.finish_seconds:.1f} seconds. "
            f"Pico WT901 samples at {GYRO_SAMPLE_HZ:.0f} Hz in its own thread; "
            f"main multiplier={GYRO_THREAD_DEG_MULTIPLIER:.3f}, "
            f"final multiplier={GYRO_FINAL_TURN_MULTIPLIER:.3f}. "
            f"Gyro debug line printing={PICO_GYRO_PRINT_LATEST_LINE}.",
            flush=True,
        )

        # Start draining Pico serial immediately. The software yaw is reset again
        # when model control actually starts, so startup/parking-exit movement
        # does not count toward the lap gyro target.
        self._start_sampling()
        print(
            "Pico WT901 serial drain started early; yaw will re-zero at model start.",
            flush=True,
        )

    @staticmethod
    def _unwrap_delta_deg(current_yaw, previous_yaw):
        delta = float(current_yaw) - float(previous_yaw)
        while delta > 180.0:
            delta -= 360.0
        while delta < -180.0:
            delta += 360.0
        return delta

    def _start_sampling(self):
        """Start the high-rate Pico WT901 sampler / serial drainer."""
        if self._sampling_started:
            return

        self._sampling_started = True
        self._stop_event.clear()
        self._sample_thread = threading.Thread(
            target=self._sample_loop,
            name="pico-wt901-gyro-100hz",
            daemon=True,
        )
        self._sample_thread.start()
        print(
            f"Pico WT901 gyro sampling started at {GYRO_SAMPLE_HZ:.0f} Hz.",
            flush=True,
        )

    def _reset_software_yaw_state(self):
        """Zero software yaw accumulation for a fresh WT901/Pico stream."""
        self._last_wt901_yaw_deg = None
        self._last_wt901_time = None
        self._last_frame_received_time = 0.0
        self._zero_printed = False
        self.read_error_printed = False
        self._angle_at_zero = None
        self._last_raw_angle = None
        with self._lock:
            self.raw_total_rotation_deg = 0.0
            self.raw_current_rate_deg_per_sec = 0.0
            self.total_rotation_deg = 0.0
            self.current_rate_deg_per_sec = 0.0

    def _reopen_pico_stream_after_no_input(self, reason):
        """Close/reopen the WT901 stream, and optionally soft-reboot Pico main.py."""
        if not PICO_AUTO_REOPEN_ON_NO_INPUT:
            return False

        now = time.monotonic()
        if self._auto_reopen_attempts >= int(PICO_AUTO_REOPEN_MAX_ATTEMPTS):
            return False
        if (
            self._last_auto_reopen_time > 0.0
            and now - self._last_auto_reopen_time < PICO_AUTO_REOPEN_COOLDOWN_SECONDS
        ):
            return False

        self._auto_reopen_attempts += 1
        self._last_auto_reopen_time = now
        print(
            f"Pico WT901 auto-reopen {self._auto_reopen_attempts}/"
            f"{PICO_AUTO_REOPEN_MAX_ATTEMPTS}: {reason}. "
            "Closing and reopening Pico serial; gyro-zero commands stay disabled.",
            flush=True,
        )

        try:
            self.wt901.close()
        except Exception:
            pass

        # an earlier revision: this flag now actually controls the reboot. In an earlier revision it only
        # controlled the message, and every reopen rebooted the Pico, which
        # meant a Pico that simply booted slowly was killed again and again.
        soft_reboot = bool(
            PICO_AUTO_SOFT_REBOOT_ON_NO_INPUT
            and self._auto_reopen_attempts <= int(PICO_AUTO_SOFT_REBOOT_MAX_ATTEMPTS)
        )
        if soft_reboot:
            print(
                "Pico WT901 recovery: the next reader open will soft-reboot "
                "main.py while keeping /dev/ttyACM0 open.",
                flush=True,
            )
        else:
            print(
                "Pico WT901 recovery: soft-reboot attempts used up; reopening "
                "and listening only.",
                flush=True,
            )

        try:
            self.wt901 = PicoWT901YawReader(
                PICO_GYRO_PORT,
                PICO_GYRO_BAUD,
                soft_reboot=soft_reboot,
            )
        except Exception as error:
            print(f"Pico WT901 auto-reopen failed: {error}", flush=True)
            return False

        self._reset_software_yaw_state()
        return True

    def _sample_loop(self):
        """Continuously read Pico WT901 yaw values and accumulate unwrapped yaw."""
        next_sample_time = time.monotonic()
        startup_deadline = time.monotonic() + PICO_GYRO_STARTUP_TIMEOUT_SECONDS

        while not self._stop_event.is_set():
            now = time.monotonic()
            yaw_deg = self.wt901.read_yaw()

            if yaw_deg is not None and math.isfinite(yaw_deg):
                self._last_frame_received_time = now
                self._last_raw_angle = yaw_deg
                if self._angle_at_zero is None:
                    self._angle_at_zero = yaw_deg

                if self._last_wt901_yaw_deg is None:
                    # Zero the accumulated yaw at the first valid frame after
                    # model control begins.
                    self._last_wt901_yaw_deg = yaw_deg
                    self._last_wt901_time = now
                    with self._lock:
                        self.raw_total_rotation_deg = 0.0
                        self.raw_current_rate_deg_per_sec = 0.0
                        self.total_rotation_deg = 0.0
                        self.current_rate_deg_per_sec = 0.0
                    if not self._zero_printed:
                        debug = self.wt901.get_debug_snapshot()
                        print(
                            f"Pico WT901 yaw zero set at {yaw_deg:+.2f} degrees "
                            f"from frame {debug['frame']}.",
                            flush=True,
                        )
                        if PICO_GYRO_PRINT_LATEST_LINE and debug.get("line"):
                            print(
                                "Pico WT901 zero line: "
                                f"{debug['line'][:PICO_GYRO_PRINT_LINE_MAX_CHARS]}",
                                flush=True,
                            )
                        self._zero_printed = True
                else:
                    dt = now - self._last_wt901_time if self._last_wt901_time else 0.0
                    if dt <= 0.0:
                        dt = GYRO_SAMPLE_SECONDS

                    raw_delta_deg = self._unwrap_delta_deg(
                        yaw_deg,
                        self._last_wt901_yaw_deg,
                    )
                    rate_deg_per_sec = raw_delta_deg / dt

                    if abs(rate_deg_per_sec) < GYRO_RATE_DEADBAND_DEG_PER_SEC:
                        raw_delta_deg = 0.0
                        rate_deg_per_sec = 0.0

                    if abs(rate_deg_per_sec) <= GYRO_MAX_VALID_RATE_DEG_PER_SEC:
                        calibrated_delta_deg = raw_delta_deg * GYRO_THREAD_DEG_MULTIPLIER
                        with self._lock:
                            self.raw_total_rotation_deg += raw_delta_deg
                            self.raw_current_rate_deg_per_sec = rate_deg_per_sec
                            self.total_rotation_deg += calibrated_delta_deg
                            self.current_rate_deg_per_sec = (
                                rate_deg_per_sec * GYRO_THREAD_DEG_MULTIPLIER
                            )

                        self._last_wt901_yaw_deg = yaw_deg
                        self._last_wt901_time = now
                    else:
                        print(
                            f"Ignored unusual Pico WT901 yaw rate of "
                            f"{rate_deg_per_sec:+.1f} deg/s.",
                            flush=True,
                        )
                        # Do not add the jump, but resync to the newest yaw so
                        # a bad frame cannot poison all later deltas.
                        self._last_wt901_yaw_deg = yaw_deg
                        self._last_wt901_time = now
            else:
                if (
                    self._last_frame_received_time > 0.0
                    and now - self._last_frame_received_time > PICO_GYRO_STALE_TIMEOUT_SECONDS
                ):
                    if self._reopen_pico_stream_after_no_input("Pico stream became stale"):
                        startup_deadline = time.monotonic() + PICO_GYRO_STARTUP_TIMEOUT_SECONDS
                    elif not self.read_error_printed:
                        print(
                            f"Pico WT901 warning: no valid yaw value for "
                            f"{PICO_GYRO_STALE_TIMEOUT_SECONDS:.2f} seconds.",
                            flush=True,
                        )
                        self.read_error_printed = True
                elif self._last_frame_received_time == 0.0 and now > startup_deadline:
                    if self._reopen_pico_stream_after_no_input("no Pico input at startup"):
                        startup_deadline = time.monotonic() + PICO_GYRO_STARTUP_TIMEOUT_SECONDS
                    elif not self.read_error_printed:
                        debug = self.wt901.get_debug_snapshot()
                        extra = (
                            f" bytes={debug.get('bytes_received')}"
                            f" lines={debug.get('lines_seen')}"
                            f" parsed_frames={debug.get('frame')}"
                            f" unparsed={debug.get('unparsed_lines_seen')}"
                        )
                        print(
                            "Pico WT901 warning: no Angle/Yaw values parsed from Pico serial stream yet. "
                            "an earlier revision can auto-reopen and soft-reboot Pico main.py if enabled. "
                            "Check port, baud, newline, and printed format." + extra,
                            flush=True,
                        )
                        if debug.get("last_unparsed_line"):
                            print(
                                "Last unparsed Pico line: "
                                f"{debug['last_unparsed_line'][:PICO_GYRO_PRINT_LINE_MAX_CHARS]}",
                                flush=True,
                            )
                        self.read_error_printed = True

            if yaw_deg is not None:
                self.read_error_printed = False

            next_sample_time += GYRO_SAMPLE_SECONDS
            sleep_seconds = next_sample_time - time.monotonic()
            if sleep_seconds > 0.0:
                self._stop_event.wait(sleep_seconds)
            else:
                next_sample_time = time.monotonic()

    def _get_total_rotation_deg(self):
        """Return the main calibrated signed accumulated Pico WT901 yaw angle."""
        with self._lock:
            return float(self.total_rotation_deg)

    def _get_final_turn_rotation_deg(self):
        """Return the separately calibrated Pico WT901 yaw angle for final turn."""
        with self._lock:
            return float(self.raw_total_rotation_deg) * GYRO_FINAL_TURN_MULTIPLIER

    def _get_status_snapshot(self):
        """Return calibrated accumulated angle and calibrated current rate."""
        with self._lock:
            return (
                float(self.total_rotation_deg),
                float(self.current_rate_deg_per_sec),
            )

    def run(self, angle, throttle):
        angle = 0.0 if angle is None else float(angle)
        throttle = 0.0 if throttle is None else float(throttle)
        now = time.monotonic()

        # The first call occurs only after any blocking parking-exit program.
        if not self._sampling_started:
            self._start_sampling()

        if self.model_start_time is None:
            self.model_start_time = now
            self._reset_software_yaw_state()
            print(
                f"Model run started. Pico WT901 yaw re-zeroing now; "
                f"gyro stop is locked out for {self.lockout_seconds:.1f} seconds.",
                flush=True,
            )

        if self.stopped:
            return 0.0, 0.0

        total_rotation_deg, current_rate = self._get_status_snapshot()
        progress = abs(total_rotation_deg)
        with self._lock:
            raw_progress = abs(float(self.raw_total_rotation_deg))
            raw_rate = float(self.raw_current_rate_deg_per_sec)
        model_elapsed = now - self.model_start_time
        gyro_stop_unlocked = model_elapsed >= self.lockout_seconds

        if now - self.last_print_time >= GYRO_STATUS_PRINT_SECONDS:
            lock_state = "ACTIVE" if gyro_stop_unlocked else "LOCKED"
            debug = self.wt901.get_debug_snapshot()
            latest_angle = debug.get("yaw_deg")
            # an earlier revision: plain ASCII. The UTF-8 degree sign came out garbled in the
            # run log because the Pi console is not reading it as UTF-8, which
            # made the one status line you actually watch harder to read.
            angle_text = "--" if latest_angle is None else f"{latest_angle:+.1f}deg"
            age = debug.get("age_seconds")
            age_text = "--" if age is None else f"{age * 1000:.0f}ms"
            seconds_left = max(0.0, self.lockout_seconds - model_elapsed)

            print(
                f"SECONDS: {model_elapsed:5.1f}s / {self.lockout_seconds:.1f}s "
                f"| until gyro active: {seconds_left:4.1f}s "
                f"| GYRO STOP: {lock_state}",
                flush=True,
            )
            print(
                f"GYRO WT901: angle={angle_text} "
                f"| yaw={progress:6.1f}/{GYRO_TARGET_DEG:.1f}deg "
                f"| rate={current_rate:+6.1f}deg/s "
                f"| frame={debug.get('frame')} | age={age_text} "
                f"| bytes={debug.get('bytes_received')} lines={debug.get('lines_seen')}",
                flush=True,
            )
            if PICO_GYRO_PRINT_LATEST_LINE and debug.get("line"):
                print(
                    "PICO RAW: "
                    f"{debug['line'][:PICO_GYRO_PRINT_LINE_MAX_CHARS]}",
                    flush=True,
                )
            self.last_print_time = now

        # The seconds value only unlocks the stop. It does not stop the model.
        if self.finish_deadline is None:
            target_ready = (
                gyro_stop_unlocked
                and progress >= (GYRO_TARGET_DEG + GYRO_TARGET_CONFIRM_MARGIN_DEG)
            )

            if target_ready:
                if self.gyro_target_first_seen is None:
                    self.gyro_target_first_seen = now
                    print(
                        f"Pico WT901 gyro target first seen after lockout at "
                        f"{progress:.1f} calibrated degrees. Confirming...",
                        flush=True,
                    )
                elif now - self.gyro_target_first_seen >= GYRO_TARGET_CONFIRM_SECONDS:
                    self.finish_deadline = now + self.finish_seconds
                    print(
                        f"Pico WT901 gyro target confirmed at {progress:.1f} calibrated degrees after "
                        f"{model_elapsed:.1f} seconds. Continuing model driving "
                        f"for {self.finish_seconds:.1f} seconds.",
                        flush=True,
                    )
                    self._print_integration_audit(model_elapsed)
            else:
                self.gyro_target_first_seen = None

        if self.finish_deadline is None or now < self.finish_deadline:
            return angle, throttle

        if self.drive_mode in ("OCW", "OCCW") and not self.finish_turn_done:
            self.finish_turn_done = True
            if RUN_GYRO_FINISH_TURN_AFTER_GYRO_STOP:
                end_yaw_deg = self._get_total_rotation_deg()
                gyro_obstacle_finish_turn(
                    self.drive_mode,
                    end_yaw_deg=end_yaw_deg,
                    get_current_yaw_deg=self._get_final_turn_rotation_deg,
                )
            else:
                print(
                    "Skipping gyro finish correction because "
                    "RUN_GYRO_FINISH_TURN_AFTER_GYRO_STOP is False.",
                    flush=True,
                )
            # The WT901 is connected through the same Pico USB serial stream
            # used by parking ToF, so release the Pico port before parking opens it.
            self._stop_sampling_and_release_pico()
            run_parking_sequence_after_obstacle(self.drive_mode)

        self.stopped = True
        self._stop_event.set()
        print(
            "Pico WT901 gyro-lockout finish complete. Steering centered and throttle stopped.",
            flush=True,
        )
        return 0.0, 0.0

    def _print_integration_audit(self, model_elapsed):
        """Compare the Pico's own yaw span against what manage accumulated.

        When the lap stops early, the question is always the same: is the
        SENSOR over-reading, or is manage's integration distorting it? These two
        numbers separate those cases, and nothing else in the log does.

            sensor span   = Angle_now - Angle_at_zero, straight off the Pico
            accumulated   = what the stop actually used

        They differ only by what the Pi-side deadband and rate rejections
        removed. So:

          spans agree, both high   -> the PICO is over-reading. Measure it with
                                      'pico_tool.py --gyro', then set BOTH
                                      GYRO_THREAD_DEG_MULTIPLIER and
                                      GYRO_FINAL_TURN_MULTIPLIER.
          spans disagree           -> GYRO_RATE_DEADBAND_DEG_PER_SEC is
                                      reshaping the total. Simulation puts that
                                      at up to ~3.5% of a lap, in either
                                      direction, depending on how much the car
                                      weaves. Set it to 0.0 to make the Pi-side
                                      sum exact and re-test.
        """
        if self._angle_at_zero is None or self._last_raw_angle is None:
            return

        sensor_span = self._last_raw_angle - self._angle_at_zero
        with self._lock:
            accumulated = float(self.raw_total_rotation_deg)
        removed = accumulated - sensor_span
        share = 100.0 * removed / max(1.0, abs(sensor_span))

        print(
            f"GYRO AUDIT | Pico Angle span {sensor_span:+.1f} deg "
            f"({self._angle_at_zero:+.1f} -> {self._last_raw_angle:+.1f}) | "
            f"manage accumulated {accumulated:+.1f} deg | "
            f"deadband/rejections changed it by {removed:+.1f} deg "
            f"({share:+.1f}%)",
            flush=True,
        )
        if abs(share) >= 2.0:
            print(
                f"GYRO AUDIT | that {abs(share):.1f}% is manage's own filtering, "
                f"not the sensor. GYRO_RATE_DEADBAND_DEG_PER_SEC="
                f"{GYRO_RATE_DEADBAND_DEG_PER_SEC} suppresses slow rotation, and "
                f"how much it removes depends on how much the car weaves - so it "
                f"varies run to run. Set it to 0.0 for an exact Pi-side sum.",
                flush=True,
            )
        else:
            implied = abs(sensor_span) / max(1.0, GYRO_TARGET_DEG)
            print(
                f"GYRO AUDIT | manage's filtering is not the issue. The Pico "
                f"itself reported {sensor_span:+.1f} deg for what should be "
                f"{GYRO_TARGET_DEG:.0f}. If the car did not physically complete "
                f"the lap, the sensor over-reads by about "
                f"{100.0*(implied-1.0):+.1f}% - measure it with "
                f"'python3 pico_tool.py --gyro'.",
                flush=True,
            )

    def _stop_sampling_and_release_pico(self):
        """Stop gyro sampling and close the Pico port before parking opens it."""
        self._stop_event.set()
        thread = self._sample_thread
        if thread is not None and thread.is_alive():
            thread.join(timeout=1.0)
        try:
            self.wt901.close()
        except Exception:
            pass

    def shutdown(self):
        """Stop the high-rate sampling thread and close Pico WT901 serial."""
        self._stop_sampling_and_release_pico()


# ---------------------------------------------------------------------------
# Reusable camera initialization for recording and driving
# ---------------------------------------------------------------------------
def add_camera(car: dk.vehicle.Vehicle):
    from donkeycar.parts.camera import PiCamera

    cam = PiCamera(
        image_w = CAPTURE_W,
        image_h = CAPTURE_H,
        image_d = 3
    )

    time.sleep(CAMERA_STARTUP_SETTLE_SECONDS)

    cam.camera.framerate           = CAMERA_FRAMERATE
    cam.camera.exposure_mode       = PICAMERA_EXPOSURE_MODE
    cam.camera.awb_mode            = PICAMERA_AWB_MODE
    cam.camera.iso                 = PICAMERA_ISO
    cam.camera.shutter_speed       = PICAMERA_SHUTTER_SPEED
    cam.camera.exposure_compensation = PICAMERA_EXPOSURE_COMPENSATION
    cam.camera.awb_gains           = PICAMERA_AWB_GAINS

    car.add(cam, outputs=["cam/raw"], threaded=True)
    car.add(Lambda(center_crop), inputs=["cam/raw"], outputs=["cam/image_array"])


# ---------------------------------------------------------------------------
# Vehicle builders
# ---------------------------------------------------------------------------
def build_vehicle_recording() -> dk.vehicle.Vehicle:
    car = dk.vehicle.Vehicle()
    add_camera(car)

    # Recording always includes the live camera window.
    car.add(CameraViewer(), inputs=["cam/image_array"], outputs=[])

    # Controller and DYNAMIXEL actuators
    car.add(PS4Joystick(), outputs=["user/angle", "user/throttle", "user/mode"])
    car.add(DynamixelSteering(), inputs=["user/angle"])
    car.add(DynamixelThrottle(), inputs=["user/throttle"])

    # Write only during normal user driving. Do not record while the
    # controller is in erase mode, even if the throttle stick is moved.
    car.add(Lambda(lambda t, mode: mode == "user" and abs(t) > RECORD_THRESHOLD),
            inputs=["user/throttle", "user/mode"], outputs=["recording"])

    tub = TubWriter(base_path=DATA_PATH, inputs=TUB_INPUTS, types=TUB_TYPES)
    car.add(tub, inputs=TUB_INPUTS, outputs=["tub/num_records"], run_condition="recording")

    class RecordCounter:
        """Display total sample count every 10 new records."""
        def __init__(self):
            self.last_ten = -1
        def run(self, n):
            if n is None:
                return
            ten = n // 10
            if ten != self.last_ten:
                print(f"Recorded samples: {n}")
                self.last_ten = ten

    car.add(RecordCounter(), inputs=["tub/num_records"], outputs=[])

    class PromptWiper:
        """Hard-delete the newest tub records from every storage file.

        DonkeyCar's normal tub delete only marks indexes as deleted in the
        manifest. That hides the records during training, but the catalog rows
        and image files can still remain on disk. This version physically
        removes the newest records by:

        1. deleting matching image files
        2. deleting old-style per-record JSON files if they exist
        3. truncating/rebuilding catalog files and catalog manifests
        4. lowering manifest.current_index so new recording can reuse numbers
        """
        def __init__(self, tub, num_records=100):
            self.tub = tub
            self.num = int(num_records)
            self.last_delete_time = float("-inf")
            self.delete_cooldown = 1.0

        def _tub_path(self):
            for attr in ("base_path", "path", "tub_path", "dir"):
                value = getattr(self.tub, attr, None)
                if value:
                    return Path(value).expanduser()

            data_root = Path(DATA_PATH).expanduser()
            tub_dirs = []
            if data_root.exists():
                for child in data_root.iterdir():
                    if child.is_dir() and ((child / "images").exists() or (child / "manifest.json").exists()):
                        tub_dirs.append(child)
            if tub_dirs:
                return max(tub_dirs, key=lambda x: x.stat().st_mtime)

            return data_root

        @staticmethod
        def _index_from_name(path):
            """Return the first integer in a DonkeyCar file name."""
            # Common names:
            #   123_cam_image_array_.jpg
            #   123_cam-image_array_.jpg
            #   record_123.json
            #   123.json
            match = re.search(r"(\d+)", path.stem)
            if not match:
                return None
            try:
                return int(match.group(1))
            except ValueError:
                return None

        @staticmethod
        def _catalog_manifest_path(catalog_path):
            return catalog_path.with_name(f"{catalog_path.stem}.catalog_manifest")

        @staticmethod
        def _safe_unlink(path):
            try:
                path.unlink()
                return 1
            except FileNotFoundError:
                return 0
            except OSError as e:
                print(f"Could not delete {path}: {e}")
                return 0

        def _delete_data_files_for_range(self, tub_path, start_index, end_index):
            """Delete physical image/json files whose index is in [start, end)."""
            deleted = 0
            image_dirs = []

            if (tub_path / "images").exists():
                image_dirs.append(tub_path / "images")
            if tub_path.exists():
                image_dirs.append(tub_path)

            image_exts = {".jpg", ".jpeg", ".png", ".npy"}
            seen = set()
            for folder in image_dirs:
                for path in folder.iterdir():
                    if not path.is_file() or path in seen:
                        continue
                    seen.add(path)
                    if path.suffix.lower() not in image_exts:
                        continue
                    idx = self._index_from_name(path)
                    if idx is not None and start_index <= idx < end_index:
                        deleted += self._safe_unlink(path)

            # Older DonkeyCar tub formats used per-record JSON files. Tub v2
            # usually does not, but delete them too if they are present.
            protected_json = {"manifest.json"}
            if tub_path.exists():
                for path in tub_path.rglob("*.json"):
                    if not path.is_file():
                        continue
                    if path.name in protected_json:
                        continue
                    idx = self._index_from_name(path)
                    if idx is not None and start_index <= idx < end_index:
                        deleted += self._safe_unlink(path)

            return deleted

        def _truncate_catalogs(self, tub_path, delete_start, old_current_index):
            manifest = self.tub.manifest
            old_catalog_paths = list(getattr(manifest, "catalog_paths", []))
            new_catalog_paths = []
            removed_catalog_files = 0

            # Close the active catalog before physically rewriting catalog files.
            try:
                if manifest.current_catalog:
                    manifest.current_catalog.close()
            except Exception as e:
                print(f"Warning: could not close active catalog before erase: {e}")

            for rel_path in old_catalog_paths:
                catalog_path = tub_path / rel_path

                if not catalog_path.exists():
                    continue

                try:
                    cat = Catalog(catalog_path.as_posix(), read_only=False)
                    start_index = int(cat.manifest.start_index())
                    line_count = int(cat.seekable.lines())
                    end_index = start_index + line_count

                    if start_index >= delete_start:
                        # This whole catalog is inside the deleted range. Keep
                        # the first catalog as an empty file so the tub still has
                        # a valid place to continue recording from index 0.
                        if delete_start == 0 and not new_catalog_paths:
                            cat.seekable.truncate_until_end(0)
                            cat.manifest.update_line_lengths([])
                            new_catalog_paths.append(rel_path)
                            cat.close()
                        else:
                            cat.close()
                            removed_catalog_files += self._safe_unlink(catalog_path)
                            removed_catalog_files += self._safe_unlink(self._catalog_manifest_path(catalog_path))

                    elif start_index < delete_start < end_index:
                        # The delete range starts inside this catalog. Keep the
                        # front part and cut off the newest rows.
                        keep_count = max(0, delete_start - start_index)
                        cat.seekable.truncate_until_end(keep_count)
                        cat.manifest.update_line_lengths(cat.seekable.line_lengths)
                        new_catalog_paths.append(rel_path)
                        cat.close()

                    else:
                        # This catalog is fully before the deleted range.
                        new_catalog_paths.append(rel_path)
                        cat.close()

                except Exception as e:
                    print(f"Could not rewrite catalog {catalog_path}: {e}")

            # If something went wrong or the tub had no catalog yet, recreate a
            # valid empty first catalog.
            if not new_catalog_paths:
                rel_path = "catalog_0.catalog"
                catalog_path = tub_path / rel_path
                try:
                    cat = Catalog(catalog_path.as_posix(), read_only=False, start_index=0)
                    cat.seekable.truncate_until_end(0)
                    cat.manifest.update_line_lengths([])
                    cat.close()
                    new_catalog_paths = [rel_path]
                except Exception as e:
                    print(f"Could not recreate empty catalog: {e}")

            # Update the live manifest object so DonkeyCar continues recording
            # cleanly after the hard delete.
            manifest.catalog_paths = new_catalog_paths
            manifest.current_index = delete_start
            manifest.deleted_indexes = set(i for i in manifest.deleted_indexes if i < delete_start)
            manifest._update_catalog_metadata(update=True)

            # Reopen the active catalog. If current_index is exactly on a catalog
            # boundary, DonkeyCar will create the next catalog on the next write.
            try:
                last_catalog_path = tub_path / manifest.catalog_paths[-1]
                manifest.current_catalog = Catalog(last_catalog_path.as_posix(), read_only=False)
            except Exception as e:
                print(f"Warning: could not reopen active catalog after erase: {e}")

            return removed_catalog_files

        def _hard_delete_last_n_records(self):
            manifest = getattr(self.tub, "manifest", None)
            if manifest is None:
                print("Hard delete failed: tub has no manifest object.")
                return 0, 0, 0

            old_current_index = int(getattr(manifest, "current_index", 0))
            if old_current_index <= 0:
                print("No tub records to delete.")
                return 0, 0, 0

            delete_start = max(0, old_current_index - self.num)
            actual_deleted_records = old_current_index - delete_start
            tub_path = self._tub_path()

            deleted_data_files = self._delete_data_files_for_range(
                tub_path, delete_start, old_current_index
            )
            removed_catalog_files = self._truncate_catalogs(
                tub_path, delete_start, old_current_index
            )

            return actual_deleted_records, deleted_data_files, removed_catalog_files

        def run(self, mode):
            # PS4Joystick emits "erase" for exactly one vehicle loop. The
            # cooldown is a second safety layer against unusual controller noise.
            if mode != "erase":
                return

            now = time.monotonic()
            if now - self.last_delete_time < self.delete_cooldown:
                return
            self.last_delete_time = now

            print("Erase request accepted. Updating tub files...", flush=True)
            try:
                deleted_records, deleted_data_files, removed_catalog_files = (
                    self._hard_delete_last_n_records()
                )
            except Exception as e:
                print(f"Hard delete failed: {e}", flush=True)
                return

            print(
                f"Hard-deleted {deleted_records} records, "
                f"{deleted_data_files} image/json files, "
                f"and {removed_catalog_files} old catalog files.",
                flush=True,
            )

    wiper = PromptWiper(tub.tub, num_records=100)
    car.add(wiper, inputs=["user/mode"], outputs=[])
    return car


def build_vehicle_driving(model_path: str, drive_mode: str,
                          show_camera: bool = False,
                          skip_parking: bool = False) -> dk.vehicle.Vehicle:
    """Build one of the autonomous driving variants.

    show_camera:
        False for --driving / --driving-skip
        True for --driving-view / --driving-view-skip

    skip_parking:
        False runs obstacle_start_program() for OCW/OCCW.
        True connects the model directly to the DYNAMIXEL motors immediately.
    """
    car = dk.vehicle.Vehicle()
    add_camera(car)

    interpreter = KerasInterpreter()
    pilot = KerasLinear(interpreter=interpreter, input_shape=(CROP_H, CROP_W, 3))
    pilot.load(model_path)

    car.add(pilot, inputs=["cam/image_array"],
            outputs=["pilot/angle", "pilot/throttle"])

    # Live console read-out of model predictions.
    car.add(ConsoleDisplay(), inputs=["pilot/angle", "pilot/throttle"], outputs=[])

    # Camera display is independent from whether parking is skipped.
    if show_camera:
        car.add(CameraViewer(), inputs=["cam/image_array"], outputs=[])

    if skip_parking:
        # The -skip variants give the model immediate control. No startup
        # controller or control mux is added to the vehicle pipeline.
        print("Parking-exit program skipped. Model controls the car immediately.")
        steering_input = "pilot/angle"
        throttle_input = "pilot/throttle"
    else:
        # Regular OCW/OCCW driving runs the parking-exit start program.
        # FCW/FCCW naturally return an empty sequence and pass model controls.
        car.add(
            ObstacleStartController(drive_mode),
            outputs=["startup/angle", "startup/throttle", "startup/active"]
        )
        car.add(
            DriveControlMux(),
            inputs=[
                "startup/angle", "startup/throttle", "startup/active",
                "pilot/angle", "pilot/throttle",
            ],
            outputs=["drive/angle", "drive/throttle"]
        )
        steering_input = "drive/angle"
        throttle_input = "drive/throttle"

    # All autonomous variants use the WT901 gyro lap controller. It passes
    # model controls through until GYRO_TARGET_DEG degrees of net yaw, continues for the
    # mode-specific finish delay, and then outputs zero throttle.
    car.add(
        GyroThreeLapController(drive_mode),
        inputs=[steering_input, throttle_input],
        outputs=["final/angle", "final/throttle"],
    )

    car.add(DynamixelSteering(), inputs=["final/angle"])
    car.add(DynamixelThrottle(), inputs=["final/throttle"])
    return car


# ---------------------------------------------------------------------------
# Main
# ---------------------------------------------------------------------------
if __name__ == "__main__":
    import argparse

    print(
        f"manage26V18 starting | model speed {MAX_SPEED_PERCENT:.0f}% | "
        f"gyro stop ignored for {gyro_lockout_seconds():.1f}s "
        f"(constant {GYRO_IGNORE_STOP_UNTIL_SECONDS:.1f}s at "
        f"{GYRO_LOCKOUT_REFERENCE_SPEED_PERCENT:.0f}%, "
        f"auto-scale={GYRO_LOCKOUT_AUTO_SCALE_WITH_SPEED}) | "
        f"Pico WT901 target: {GYRO_TARGET_DEG:.1f} calibrated degrees | "
        f"Pico WT901 sample rate: {GYRO_SAMPLE_HZ:.0f} Hz | "
        f"main yaw multiplier: {GYRO_THREAD_DEG_MULTIPLIER:.3f} | "
        f"final turn multiplier: {GYRO_FINAL_TURN_MULTIPLIER:.3f} | "
        f"final turn throttle: {GYRO_OBSTACLE_FINISH_THROTTLE:+.2f} | "
        f"Pico gyro port: {PICO_GYRO_PORT} | "
        f"lap status print: every {GYRO_STATUS_PRINT_SECONDS:.1f}s | "
        f"raw Pico line print: {PICO_GYRO_PRINT_LATEST_LINE} | "
        f"restart Pico main.py: {PICO_RESTART_MAIN_ON_RUN} | "
        f"reset WT901 gyro: {PICO_GYRO_RESET_ON_RUN} | "
        f"auto-reopen on no Pico input: {PICO_AUTO_REOPEN_ON_NO_INPUT} | "
        f"auto-free Pico serial: {PICO_AUTO_FREE_SERIAL_PORT_ON_RUN} | "
        f"Pico soft reboot on startup: {PICO_RESTART_MAIN_ON_RUN} | "
        f"Pico soft reboot on gyro-reader open: {PICO_REBOOT_WHEN_GYRO_READER_OPENS} | "
        f"Pico raw-REPL escape Ctrl-B: {PICO_RESTART_SEND_CTRL_B} | "
        f"Pico passive listen: {PICO_GYRO_PASSIVE_LISTEN_SECONDS:.1f}s then "
        f"{PICO_GYRO_ESCAPE_ATTEMPTS_ON_OPEN} escape attempts | "
        f"gyro-zero commands: {PICO_GYRO_RESET_ON_RUN} | "
        f"C ToF steer: {PARKING_C_APPROACH_STEER_DEG:+.1f} deg | "
        f"camera settle {CAMERA_STARTUP_SETTLE_SECONDS:.2f}s",
        flush=True,
    )

    # An earlier revision parking tuning summary. These are the values to change when alignment
    # or wall following is off; nothing above this line affects them.
    print(
        f"parking align | leeway {PARKING_AB_BALANCE_DIFF_LEEWAY_MM:.1f} mm | "
        f"Kp {PARKING_AB_ALIGN_KP_DEG_PER_MM:.2f} deg/mm, steer "
        f"{PARKING_AB_ALIGN_MIN_STEER_DEG:.1f}-{PARKING_AB_BALANCE_STEER_DEG:.1f} deg | "
        f"speed {PARKING_AB_BALANCE_SPEED_PERCENT:.0f}% / fine "
        f"{PARKING_AB_ALIGN_FINE_SPEED_PERCENT:.0f}% under "
        f"{PARKING_AB_ALIGN_FINE_DIFF_MM:.1f} mm | "
        f"confirm {PARKING_AB_BALANCE_REQUIRED_COUNT}x over "
        f"{PARKING_AB_BALANCE_REQUIRED_SECONDS:.2f}s, settled rate "
        f"<={PARKING_AB_ALIGN_SETTLED_RATE_MM_PER_SEC:.1f} mm/s | "
        f"timeout to PID: {PARKING_AB_BALANCE_TIMEOUT_CONTINUE_TO_PID}",
        flush=True,
    )
    print(
        f"parking PID | K {PARKING_K():.2f} | angle weight "
        f"{PARKING_PID_ANGLE_WEIGHT:.2f} | distance weight "
        f"{PARKING_PID_DISTANCE_WEIGHT:.2f} | KI {PARKING_PID_KI:.3f} "
        f"KD {PARKING_PID_KD:.3f} | target {PARKING_WALL_TARGET_MM:.1f} mm | "
        f"follow speed {PARKING_FOLLOW_SPEED_PERCENT:.0f}% | "
        f"steer clamp {PARKING_MAX_WALL_STEER_DEG:.0f} deg, rate "
        f"{PARKING_PID_MAX_STEER_RATE_DEG_PER_SEC:.0f} deg/s | "
        f"wall confirm {PARKING_WALL_FIND_CONFIRM_READINGS}x | "
        f"steering servo profile {PARKING_STEERING_PROFILE_VELOCITY} LSB",
        flush=True,
    )

    free_pico_serial_port("manage26V18 startup")

    # an earlier revision: run the pico_tool.py --fix routine automatically, before the camera
    # and the TensorFlow model load. If the Pico needs a reboot, its ToF
    # initialization then overlaps the model load instead of delaying the run.
    ensure_pico_streaming("manage26V18 startup")
    # An earlier revision does not restart Pico main.py here. The Pico is soft-rebooted later
    # when PicoWT901YawReader opens /dev/ttyACM0 and keeps the port open.

    parser = argparse.ArgumentParser(
        description=(
            "Record data or run one of four autonomous driving variants, "
            "with optional camera view, optional parking-exit skip, and "
            "Pico WT901 gyro-target stopping and obstacle finish correction"
        )
    )
    group = parser.add_mutually_exclusive_group(required=True)
    group.add_argument(
        "--recording", action="store_true",
        help="Manual PS4 driving, live camera view, and data logging"
    )
    group.add_argument(
        "--driving", action="store_true",
        help="Autonomous model driving without a camera window"
    )
    group.add_argument(
        "--driving-view", action="store_true",
        help="Autonomous driving with parking exit and a live camera window"
    )
    group.add_argument(
        "--driving-skip", action="store_true",
        help="Autonomous model driving that skips the parking-exit program"
    )
    group.add_argument(
        "--driving-view-skip", action="store_true",
        help="Skip the parking exit and show the live camera window"
    )
    parser.add_argument(
        "--model", default=None,
        help="Optional .h5 model path for any driving variant"
    )
    parser.add_argument(
        "--drive-mode", choices=["FCW", "FCCW", "OCW", "OCCW"], default=None,
        help="Optional mode override, especially useful with --model"
    )
    args = parser.parse_args()

    if args.recording:
        print("Recording mode selected with live camera view. Skipping model selection.")
        vehicle = build_vehicle_recording()
    else:
        show_camera = bool(args.driving_view or args.driving_view_skip)
        skip_parking = bool(args.driving_skip or args.driving_view_skip)

        if show_camera and skip_parking:
            print("Driving with camera view selected; parking exit will be skipped.")
        elif show_camera:
            print("Driving with camera view selected; parking exit remains enabled.")
        elif skip_parking:
            print("Driving-skip selected; parking exit will be skipped.")
        else:
            print("Normal driving selected; parking exit remains enabled.")

        if args.model:
            model_path = os.path.expanduser(args.model)
            drive_mode = args.drive_mode or infer_drive_mode_from_path(model_path)
            if drive_mode is None:
                print(
                    "Warning: drive mode could not be inferred from the model path. "
                    "The obstacle parking-exit program will be disabled. Use "
                    "--drive-mode OCW or --drive-mode OCCW when needed."
                )
                drive_mode = "UNKNOWN"
        elif args.drive_mode:
            drive_mode = args.drive_mode
            model_path = model_path_for_drive_mode(drive_mode)
        else:
            model_path, drive_mode = select_model_path_with_sensehat()

        print("Mode:", drive_mode)
        print("Model:", model_path)

        mpath = Path(model_path).expanduser()
        if not mpath.is_file():
            sys.exit(f"Model file not found: {mpath}")

        vehicle = build_vehicle_driving(
            str(mpath),
            drive_mode=drive_mode,
            show_camera=show_camera,
            skip_parking=skip_parking,
        )

    def _sigterm(_s, _f):
        print("\nSIGTERM - shutting down...")
        (vehicle.shutdown() if hasattr(vehicle, "shutdown") else vehicle.stop())
        sys.exit(0)

    signal.signal(signal.SIGTERM, _sigterm)

    try:
        vehicle.start(rate_hz=DRIVE_LOOP_HZ)
    except KeyboardInterrupt:
        print("\nCtrl-C / window close - shutting down...")
    finally:
        (vehicle.shutdown() if hasattr(vehicle, "shutdown") else vehicle.stop())
