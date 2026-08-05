# Instructions

How to run and test the robot. Everything here assumes you're on the Raspberry Pi with the DonkeyCar environment activated.

---

## Before running

1. **Charge the battery packs.** Both packs, and check the switch on the holder is off while you're handling wiring.
2. **Check the connections.** The Pi should have two USB devices plugged in: the Pico on `/dev/ttyACM0` and the U2D2 on `/dev/ttyUSB0`. The Sense HAT sits on the GPIO header and the camera ribbon goes to the CSI port.
   ```bash
   ls /dev/ttyACM* /dev/ttyUSB*
   ```
   If either is missing, the program can't run. See troubleshooting below.
3. **Confirm the motor IDs.** ID 1 is the drive motor and ID 2 is steering. If these are swapped the car will try to drive with the steering servo.
4. **Check the Pico is running `main.py`.** The onboard LED blinks about four times a second while `main.py` is alive. A solid or dark LED means it isn't running, usually because something left it at a REPL prompt.
5. **Put the car down and don't touch it during gyro calibration.** The Pico spends the first 3 seconds after boot averaging the gyro to find its bias. Moving the car during those 3 seconds bakes an error into every yaw reading for the whole session. You'll see it on the console:
   ```text
   [WT901] keep robot still: 3-second gyro calibration
   [WT901] calibration complete; bias=-0.0142 dps
   ```
6. **First run of the day, put the wheels on a stand.** The car drives at full throttle in autonomous mode and the parking exit starts moving immediately.

### What happens at startup, before anything else

The program does two things before it even parses your command line, so they run no matter which mode you pick:

- `free_pico_serial_port()` checks whether anything else is holding `/dev/ttyACM0` and closes it if it recognizes the program. Anything it doesn't recognize gets reported and left alone for you to close.
- `ensure_pico_streaming()` listens for a Pico frame, and if the port is silent it sends the escape sequence and re-checks, up to three times. Worst case this takes about 22 seconds.

Both run before the camera and TensorFlow load, so if the Pico needs a reboot its sensor startup overlaps the model load instead of adding to it. If you see the auto-fix messages scroll by, that's normal.

---

## Recording mode

Used for collecting training data. Manual driving with the PS4 controller, camera preview on, and every frame written to a tub.

```bash
python3 manage26V18.py --recording
```

### Controls

| Input | Action |
|---|---|
| Left stick, X axis | Steering |
| Right stick, Y axis | Throttle (push up to go forward) |
| Triangle | Delete the newest 100 records |
| L1 | Stop and shut down cleanly |
| Esc, or close the preview window | Same as L1 |

Both sticks have a 0.05 deadzone, so small drift near center reads as zero.

### What gets recorded

A frame is only written when you're in user mode **and** throttle is above 0.05. A parked car with the sticks moving records nothing, which is deliberate: otherwise the network learns to steer while stopped.

Each record holds the cropped 160×120 image plus your steering, throttle, and mode. Data goes to `~/WRO_FE_2026/data`. The console prints a running count every 10 records.

### Deleting bad data

If a run goes wrong, press Triangle and the newest 100 records are removed immediately: images, JSON records, and the catalog entries. There's a 0.25 s debounce so one press can't fire twice. Do this on the spot rather than trying to clean the tub later.

---

## Driving modes

Five ways to run the trained model:

| Command | What it does |
|---|---|
| `--driving` | Normal autonomous run, including the parking-exit routine |
| `--driving-view` | Same, with the camera preview window |
| `--driving-skip` | Skips the parking-exit routine |
| `--driving-view-skip` | Skips parking exit, with the preview window |
| `--recording` | Manual driving, described above |

Use the `-skip` variants when the car isn't starting inside the parking bay, like during free-run testing. Use the preview window while testing and leave it off for competition runs, since drawing it costs a little time every loop.

### Picking a model

Three ways, in order of precedence:

```bash
# 1. Explicit path
python3 manage26V18.py --driving --model ~/WRO_FE_2026/models/ocwm/ocwm016/mypilot.h5

# 2. By drive mode, uses the configured default path
python3 manage26V18.py --driving --drive-mode OCW

# 3. Neither flag, pick with the Sense HAT joystick
python3 manage26V18.py --driving
```

The four modes are `FCW` and `FCCW` for free run clockwise and counter-clockwise, and `OCW` and `OCCW` for the obstacle rounds. Direction is announced before each round, so pick it then.

With the joystick, the Sense HAT shows the current selection on the LED matrix:

| Joystick | Mode |
|---|---|
| Left | FCW |
| Right | FCCW |
| Up | OCW |
| Down | OCCW |
| Press middle | Confirm and start |

If you pass `--model` without `--drive-mode`, the mode gets guessed from the path by looking for `occw`, `fccw`, `ocw` or `fcw` in the filename. If it can't tell, it warns and runs without the parking-exit routine.

A model path that doesn't exist stops the program immediately with `Model file not found`.

---

## What to expect during a run

An obstacle run goes through these phases. Knowing the order helps you tell a real failure from normal behavior.

1. **Parking exit.** The car makes a fixed maneuver out of the bay. The model isn't driving yet and the vehicle loop is paused, so console output goes quiet for a couple of seconds. This is normal.
2. **Model driving, stop locked out.** The network drives. The gyro stop can't fire yet.
3. **Stop armed** once 32 seconds have passed.
4. **Target confirmed** when accumulated yaw passes 360° and holds for 0.2 s.
5. **Finish coast**, one more second of model driving.
6. **Finish turn**, a gyro-guided correction to line up with the bay.
7. **Parking**, five stages on the rangefinders.
8. **Stopped.** Steering centers, throttle latches at zero.

A free run stops after step 5 and skips the turn and parking.

### Reading the console

Twice a second you get two status lines:

```text
SECONDS:  12.4s / 32.0s | until gyro active: 19.6s | GYRO STOP: LOCKED
GYRO WT901: angle=+143.2deg | yaw= 143.2/360.0deg | rate=  +2.1deg/s | frame=1284 | age=48ms | bytes=61200 lines=1290
```

What to watch:

- **`GYRO STOP`** flips from `LOCKED` to `ACTIVE` at 32 seconds.
- **`yaw`** should climb steadily. If it's stuck at 0.0 the gyro isn't being read.
- **`age`** is how old the newest Pico frame is. It should stay under about 150 ms. Numbers in the hundreds mean the link is struggling.
- **`bytes`** and **`lines`** should both be climbing. `bytes=0` means nothing is arriving at all.

You'll also see `Pred -> angle +0.12 thr +0.85` once a second, which is the raw model output before the mission controller sees it.

The parking stages print their own progress, and stage 4 ends with a statistics block covering tracking error, drift, steering distribution, and link integrity, followed by one tuning recommendation.

### Stopping

`Esc` or closing the preview window stops a driving run. L1 does the same in recording mode. Either way the shutdown centers the steering, zeroes the throttle, and releases motor torque. Killing the terminal with Ctrl-C also works but is less clean.

---

## Training workflow

Training runs on the laptop, not the Pi.

1. Record driving data on the robot.
2. Copy the tubs from `~/WRO_FE_2026/data` to the training machine.
3. Put them in `training/data/`.
4. Activate the DonkeyCar environment.
5. Train:
   ```bash
   python3 train.py --tubs data/ --model models/mypilot.h5
   ```
6. Check the training and validation loss. If validation loss stops improving well before the epoch limit, you probably need more or better data rather than more epochs.
7. Copy the model back to the Pi under the right folder for its drive mode.
8. Test on the car with the wheels on a stand first.

---

## Pico tools

`pico_tool.py` talks to the Pico without loading the rest of the program, which makes it much faster for debugging the link.

```bash
python3 pico_tool.py --monitor    # watch live frames
python3 pico_tool.py --fix        # escape a stuck REPL and get main.py running
python3 pico_tool.py --calib      # measure the A/B sensor offset
```

For `--calib`, set the car parallel to a wall **by hand, using the chassis edge as your reference**, not the sensor readings. Then run it and paste the reported value into `PARKING_AB_OFFSET_MM`. Measuring the offset in a pose the alignment code chose is circular and gives a wrong answer.

Only one program can own `/dev/ttyACM0` at a time. Close the monitor before starting a run.

---

## Troubleshooting

| Symptom | Likely cause | Fix |
|---|---|---|
| `yaw` stuck at 0.0, or "no Angle frame" warnings | Pico left in the raw REPL by Thonny, `mpremote`, `ampy` or `rshell`. It stays silent until written to | `python3 pico_tool.py --fix`. Look for `raw REPL; CTRL-B to exit` in the log to confirm |
| `bytes=0` in the status line | Nothing arriving at all. Port held by another process, cable, or `main.py` not running | Check the Pico LED. Then `fuser /dev/ttyACM0` and close whatever owns it |
| Pico LED dark or solid | `main.py` isn't running | `pico_tool.py --fix`, or replug the Pico |
| `age` climbing into the hundreds of ms | Link is dropping frames, or a rangefinder read is stalling | Check the ToF wiring. The stream recovers on its own up to three times, then gives up |
| `/dev/ttyUSB0` missing | U2D2 not detected, or no permission | `ls /dev/ttyUSB*`. If it's there, run `sudo usermod -a -G dialout $USER` and log out and back in |
| `No module named donkeycar` | Environment not activated | Activate the DonkeyCar environment first |
| `Model file not found` | Wrong path, or the model wasn't copied over | Check the path under `~/WRO_FE_2026/models/` |
| Car drives but won't stop after three laps | Gyro isn't accumulating | Watch the `yaw` field during the run. If it isn't climbing, the run will only end on the lockout timer |
| Car stops too early | Gyro over-reading, or the lockout is too short for the current speed | Re-time the lockout on the track. It's speed-dependent and not auto-scaled |
| Parking stops in the wrong place | The bay wasn't detected, or the car drifted into the wall it was following | Check the log for `PARKING WALL REJECTED`. That means it saw a candidate and correctly refused it |
| Car barely turns during wall follow | Gain too low, or the servo profile velocity wasn't applied | Check the stage 4 statistics. If peak steering is under 7.5° it says so and recommends raising the gain |
| Alignment never finishes | A rangefinder is dropping out against the black wall | It gives up after 15 s and continues into the wall follow anyway, so the run isn't lost |
| Steering feels off-center | Mechanical drift in the Technic linkage | Re-check the linkage. Don't change `DXL_STEER_CENTER_TICKS` without retraining, since the model compensates for it |

---

## Pre-round checklist

- [ ] Battery packs charged, switch off while handling
- [ ] Both USB devices present: `ls /dev/ttyACM* /dev/ttyUSB*`
- [ ] Pico LED blinking
- [ ] Steering linkage tight, wheels straight at rest
- [ ] Correct model selected for the announced direction
- [ ] Car placed in the bay, held still through gyro calibration
- [ ] Preview window off for the actual round
- [ ] Nothing else holding the serial ports, especially Thonny
- [ ] Lockout timing matches the speed setting you're running
