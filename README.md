# WRO 2026 FUTURE ENGINEERS: INNOVATION NATION
Welcome to the WRO Future Engineers 2026 **Innovation Nation** Documentation! Here you will find all the documentation, source code, images, and models related to our autonomous robot.

(Innovation Nation Photo or sm)

## Our Team: 

Team Innovation Nation: Kyle Ho, Chaitra Ayinampudi, and Narasimha Yalamanchi
(Put more abt team)

(IMAGE of us)

## Project Overview: 

Stuff

## Mobility & Mechanical Design

Stuff

### Our Robot:

IMAGE of the robot 

**Dimensions:** 

(put the width x height of our robot)

**Weight:** 

(Put the weight of our robot)

**Torque/speed:**
Our top speed is 0.11Km/h (1.9 meters per minute)

**Design trade-offs:**

### Materials:

##### 2 battery boxes (stores 2 each): 
The battery holders and batteries are our power source for the robot; our holder has a switch to easily turn our robot on or off. Easy to take off or put on our robot.

<img width="200" height="200" alt="image" src="https://github.com/user-attachments/assets/7e7b7fcf-63da-480c-abd1-865ac9a1874e" />

##### 4 TAKEN 18 x 67mm 3.7V lithium Rechargeable batteries: 
Our robot inputs 16 Volts of power. Rechargeable batteries for long-term use.

<img width="200" height="200" alt="image" src="https://github.com/user-attachments/assets/1ad300cd-df51-41d6-ae28-875abdadc6d0" />


##### DC/DC Converter:
DC/DC Converter to give power to our Raspberry Pi, U2D2 controller, and Pi Pico.

<img width="250" height="200" alt="Screenshot 2026-08-02 144234" src="https://github.com/user-attachments/assets/75fcd9cf-2528-4111-bc38-31dceb744f4a" />

##### U2D2 Controller:
U2D2 Controller so that our Raspberry Pi can control our DYNAMIXEL motors

##### 2 DYNAMIXEL XL330-M288-T Motors:
XL330-M288-T motors since it's a fast, reliable digital motors.

<img width="250" height="250" alt="image" src="https://github.com/user-attachments/assets/139d5b1b-f1a3-46ba-8864-4f0d229e2664" />

##### Raspberry Pi 4 Model B:
Raspberry Pi 4 Model B since it's a reliable and cost-effective computer.

<img width="300" height="200" alt="image" src="https://github.com/user-attachments/assets/5a6db2f7-57f1-40a3-a503-edfa3f40d65b" />

##### Raspberry Pi Sense HAT:
Sense HAT to use their LED to print which model will be run during rounds; the joystick is used to select and run a model.

##### Raspberry Pi Pico:
Pi Pico controls our ToF sensors and IMU Sensor without using the Raspberry Pi too much.

##### 4 VL53L0X ToF Sensors:
The ToF Sensors calculate the distance between the robot, wall, and obstacles. We placed one in the front, one in the back, and two on the right side of the robot. We placed two on the right side to make the robot's distance and rotation (angle) more accurate. We also used ToF sensors as distance sensors for parking and finding the parking boundaries

##### WT901 9-axis IMU Sensor:
IMU sensor for reliable readings of how many laps we ran; 360 degrees is one lap.

##### Raspberry Pi Camera Module v1 / OV5647:
Pi Camera to train and run our AI models.

##### USB to MicroUSB Wire
USB to microUSB wire to have our Raspberry Pi control our Pi Pico.

##### USB to USB-C Wire
USB to USB-C wire to have our Raspberry Pi control our U2D2 Controller.

##### More wires:
Many of the wires to connect our battery holders to the DC/DC converter, and our DC/DC converter to give power to the Raspberry Pi, U2D2 controller, and Pi Pico

#### LEGOs: 
We used LEGOs as our base and to structure our robot. 

#### Cost: 
| Component                              | Quantity | Approx. Price Each |                           Total |
| -------------------------------------- | :------: | -----------------: | ------------------------------: |
| Battery Holder (2×18650 with switch)   |     2    |                 $3 |                          **$6** |
| 18650 3.7V Rechargeable Batteries      |     4    |                 $6 |                         **$24** |
| DC/DC Buck Converter                   |     1    |                 $8 |                          **$8** |
| U2D2 Controller                        |     1    |                $33 |                         **$33** |
| DYNAMIXEL XL330-M288-T Motors          |     2    |             $27.49 |                      **$54.98** |
| Raspberry Pi 4 Model B (4 GB)          |     1    |                $60 |                         **$60** |
| Raspberry Pi Sense HAT                 |     1    |                $33 |                         **$33** |
| Raspberry Pi Pico                      |     1    |                 $4 |                          **$4** |
| VL53L0X ToF Sensors                    |     4    |                 $5 |                         **$20** |
| WT901 9-Axis IMU Sensor                |     1    |                $18 |                         **$18** |
| Raspberry Pi Camera Module v1 (OV5647) |     1    |                $10 |                         **$10** |
| USB-A to Micro-USB Cable               |     1    |                 $4 |                          **$4** |
| USB-A to USB-C Cable                   |     1    |                 $5 |                          **$5** |
| Additional Wires                       |    N/A   |                $10 |                         **$10** |
| LEGO Pieces (estimated)                |    N/A   |             $20–50 |                        **≈$35** |
Estimated Total Hardware Cost: ≈ $325 USD                                                            

### Reason for Chosen Components:

(include testing or iterations affecting performance)

## Power & Sensor Architecture:

**Power budget:**
 
**Sensor trade-offs:** Include placement justified using field geometry; calibration method; failure point considerations; iteration evidence

**Diagrams:**

**Wiring** 

**Current strategy:**

**Sensor choices/placement:**

**Calibration:**

**More Diagrams:**

# Software Architecture & Obstacle Strategy

Our software splits the job in two. Driving the lap is a neural network's job: staying in the lane, seeing a red or green pillar, picking a side to pass it on. Everything that has to be exact is hand-written code reading an IMU and three time-of-flight rangefinders: counting laps, ending the run in the right spot, reversing into a 20 cm bay parallel to the wall. A color-threshold pipeline needs re-tuning every time venue lighting changes, and a neural network has no idea it has turned exactly 360 degrees.

The car runs two processors. A **Raspberry Pi Pico** does nothing but read sensors. It owns the WT901 IMU and the three VL53L0X rangefinders and publishes one text frame over USB. A **Raspberry Pi** runs the camera, the model, the mission sequence, the parking code, and the DYNAMIXEL bus. That way the Pi never blocks waiting on a sensor, and a broken sensor degrades the frame instead of stalling the drive loop.

```
                    Raspberry Pi Pico  (MicroPython, main.py)
   WT901 IMU ──I2C0──► core1 @100 Hz ──┐
   (GP4/GP5)          bias cal, EMA,   │  shared state
                      trapezoidal      │  + print lock
                      yaw integration  │
                                       ├──► "Angle=..,A=..,AS=..,B=..,BS=..,C=..,CS=.."
   VL53L0X A/B/C ─I2C1─► core0 loop ───┘        USB CDC @115200
   (GP2/GP3, XSHUT
    GP18/19/20)

                    Raspberry Pi  (Python 3, manage26V18.py)
   PiCamera 176x132 ──► center_crop 160x120 ──► KerasLinear CNN ──► pilot/angle
                                                                    pilot/throttle
                                                                          │
   Pico stream ──► PicoWT901YawReader ──► GyroThreeLapController ◄────────┘
                   (parse)                (100 Hz sampler thread,          │
                                           stop condition, finish,         ▼
                                           parking handoff)          final/angle
                                                    │                final/throttle
                                                    ▼                      │
                                          ParkingPicoToF + 5 stages        │
                                                    │                      │
                                                    └──────► DYNAMIXEL XL330 ◄┘
                                                             (U2D2, /dev/ttyUSB0)
```

---

## Code structure

### Repository layout

| Path | Role |
|---|---|
| `src/manage26V18.py` | The Raspberry Pi program. DonkeyCar part graph, model pilot, mission sequence, all five parking stages, DYNAMIXEL drivers. |
| `src/pico/main.py` | Pico firmware. Dual-core IMU and ToF reader, USB frame publisher, host command handler. |
| `src/pico/vl53l0x.py` | Patched MicroPython VL53L0X driver, so we can reassign I²C addresses at runtime. |
| `tools/pico_tool.py` | Bench utility. `--monitor` shows live frames, `--fix` escapes a stuck REPL, `--calib` measures the A/B sensor offset. |
| `models/fcwm/`, `models/fccwm/`, `models/ocwm/`, `models/occwm/` | Four trained `mypilot.h5` files, one per challenge and direction. |
| `data/` | DonkeyCar Tub datasets from our recording runs. |

On the robot the last two live at `~/WRO_FE_2026/models/…` and `~/WRO_FE_2026/data`, not in the repo tree.

`manage26V18.py` is one big file instead of a package. Almost every constant that changes behavior sits in one config block at the top, named in caps and grouped by subsystem: `DXL_*`, `GYRO_*`, `PICO_*`, `PARKING_*`. The PiCamera exposure and white-balance constants are the exception, in their own block further down. **No `PARKING_*` constant is read by the model-driving code, and no model constant is read by the parking code**, which is what lets us retune parking between rounds without invalidating a trained model.

### Mission state machine

The run is a straight line of phases with no way back, so a phase that fails can't be re-entered. Every phase either finishes, or times out and hands control forward, still moving, in a worse but still working state.

`GyroThreeLapController` owns the sequence. It sits at the end of the DonkeyCar part graph, so it gets the last word on what reaches the servos.

| # | Phase | Entry condition | What controls the car | Exit |
|---|---|---|---|---|
| 0 | Startup | process start | nothing, motors idle | Pico verified streaming |
| 1 | Parking exit | obstacle run only (`OCW`/`OCCW`) | open-loop `obstacle_start_program` | maneuver done |
| 2 | Model driving, stop locked out | first pilot frame | trained CNN | `32.0 s` elapsed |
| 3 | Model driving, stop armed | lockout expired | trained CNN | integrated yaw ≥ `360.0°` |
| 4 | Confirming | target first seen | trained CNN, unchanged | target held `0.20 s` |
| 5 | Finish coast | target confirmed | trained CNN, unchanged | `1.0 s` obstacle / `2.0 s` open |
| 6 | Finish turn | obstacle run only | gyro closed loop, `±50°` steer, `0.35` throttle | yaw within `2.0°` of 270°, or `15 s` |
| 7 | Parking | obstacle run only | five ToF stages, below | bay entered, or a stage raises |
| 8 | Stopped | — | latched `(0.0, 0.0)` forever | — |

Those phase names are labels, not code. There's no enum: the sequence lives in booleans and timestamps on `GyroThreeLapController` (`stopped`, `finish_deadline`, `gyro_target_first_seen`, `finish_turn_done`), so the state isn't visible in one place. Turning it into a real enum is on our list at the end.

**How the run actually ends.** Two things have to be true before the car stops. Integrated yaw has to pass `GYRO_TARGET_DEG = 360.0`, and at least `GYRO_IGNORE_STOP_UNTIL_SECONDS = 32.0` of wall-clock time has to have passed since the model took over. One lap of the mat adds up to roughly 360 degrees of turning, so at race speed the yaw condition is already satisfied during the first lap, and the lockout is what really decides when the run ends. The yaw target works as a sanity check: it stops the run from ending if the car has been pinned against a wall and hasn't been turning at all.

We measured that timer separately for each speed setting: `34.0 s` at `MAX_SPEED_PERCENT = 90`, `32.0 s` at `100`. Dividing by the speed ratio predicts 30.6 s, so an 11% speed increase only bought a 6% time reduction. Throttle isn't pinned at full for a whole lap and cornering doesn't scale with the setting either. So `GYRO_LOCKOUT_AUTO_SCALE_WITH_SPEED` is `False`, both measured pairs are in a comment next to the constant, and changing speed means re-timing on the track.

`_reset_software_yaw_state()` re-zeros the accumulator on the first model frame, after the blocking parking-exit maneuver, so rotation from the exit is excluded. `GYRO_THREAD_DEG_MULTIPLIER` trims the yaw scale if a full lap doesn't read close to 360.

Raising the target to a real three-lap figure near 1080 degrees is the most useful change we have left. We kept the timer this season because it's the version we have track data for, and because a badly scaled yaw target ends the round in the wrong place and costs the whole parking score.

**Everything gets confirmed before it acts.** Any of our sensors can hand us one bad sample, so no threshold fires on a single frame. The lap target holds for `GYRO_TARGET_CONFIRM_SECONDS = 0.20`. Parking-bay detection needs `PARKING_WALL_FIND_CONFIRM_READINGS = 2` frames in a row. Wall alignment needs three confirmations spanning at least `0.15 s`. A frame that fails resets the counter to zero.

### Handling edge cases

Almost every safety check here is there because something broke on the track first.

**A dead sensor must not silence the stream.** In the first Pico firmware, a VL53L0X that didn't show up at `0x29` returned from `main()`. The Pico printed a fatal message and went quiet forever, even though the IMU thread on core1 was fine, and from the Pi that looks exactly like a dead gyro. Now each rangefinder initializes on its own. A failure marks that sensor missing, reports `-1` with status `9` from then on, and the frame keeps flowing. Failing to start core1 isn't fatal either: it retries once, then streams `Angle=0.00`.

**One byte from the host must not freeze a core.** The old command handler called `sys.stdin.readline()`, which blocks until a newline shows up, and a soft-reboot handshake sends bytes without one. `poll_usb_command()` now uses `select.select(..., 0)` and reads a character at a time.

**Two cores printing must not corrupt a frame.** Core0 prints data frames and core1 prints IMU status. Without locking their output interleaves mid-line and the Pi sees things like `Angle=12.[WT901 READ ERROR 3]`. `send_line()` takes a `_thread.allocate_lock()`.

**The Pico can get stuck in a state that waiting won't fix.** Thonny, `mpremote`, `ampy` and `rshell` all drop MicroPython into its *raw* REPL to copy files. If one disconnects without cleaning up, the Pico stays there, silent until something writes to it, with `main.py` not running. Our recovery handshake sent Ctrl-C then Ctrl-D, which soft-reboots in the friendly REPL but only means "run the buffer" in the raw one, so it did nothing and the retry loop used up all its retries. One log line finally told us: `raw REPL; CTRL-B to exit`. The sequence is now Ctrl-C, Ctrl-C, **Ctrl-B**, Ctrl-D, and `PICO_RAW_REPL_MARKERS` makes the reader name that banner in the log instead of showing an unparsed line.

**Two more link faults.** MicroPython on the RP2040 gates all USB output on `tud_cdc_connected()`, which follows DTR, so an earlier version that de-asserted DTR after opening the port silently threw away every `print()`. We now force `dtr = True, rts = False` everywhere. Writing to a *healthy* streaming `main.py` is also harmful, because MicroPython buffers `\x04` as ordinary stdin data, so we listen for `PICO_GYRO_PASSIVE_LISTEN_SECONDS = 2.00` and only write if the port is definitely silent.

**Port contention.** A leftover Thonny or monitor session holding `/dev/ttyACM0` looks like a dead gyro. `free_pico_serial_port()` runs `fuser` and kills what's blocking the port, but only if the command line matches `PICO_SERIAL_SAFE_KILL_KEYWORDS` (thonny, mpremote, rshell, ampy, screen, minicom, picocom, our own tools, older `manage26V*` instances). Anything it doesn't recognize gets reported and left alone. `ensure_pico_streaming()` then runs the listen-escape-verify cycle up to `PICO_AUTO_FIX_MAX_ATTEMPTS = 3` times, *before* the camera and TensorFlow load, so a Pico reboot's 1.7 s of rangefinder startup overlaps the model load instead of adding to it.

**One serial port, two consumers.** The parking rangefinders and the gyro stream come from the same Pico. Before parking starts, `_stop_sampling_and_release_pico()` stops the 100 Hz sampler thread, joins it, and closes the reader so `ParkingPicoToF` can take the port.

**Bad frames get rejected, not reinterpreted.** The parser reads both a labeled form (`A=50,AS=0,...`) and a bare positional form. On a damaged labeled line, the status numbers slid into the distance slots and produced readings that were wrong but believable, and biased low, which is the worst direction for a wall follower. The parser now only falls back to positional reading when the line has *no* labels at all, and every required field has to appear **exactly once** or the whole line is thrown out and counted. Fragments get reassembled with a 512-byte cap.

**Two steering centers, on purpose.** The model drives through `DXL_STEER_CENTER_TICKS = 3060`, parking uses `PARKING_STEERING_CENTER_TICKS = 3126` with a `0.7°` trim, and they're about 6.5 degrees apart. 3060 is the value the models were *trained against*, so if it's biased the network already learned to compensate, and changing it without retraining would put the car off by the same amount the other way. The parking number came off the standalone parking program that parked reliably.

### Testing and tuning process

The loop is: run it, capture the log, find the number that explains what we saw, change one thing, write down what the old value was. Every behavior change becomes a new version with the root cause written into the module docstring.

The wall follower and the aligner both looked like they were sending corrections that did nothing, and the log showed steering commands changing every frame while the car barely turned. It wasn't a gain problem. `PARKING_STEERING_PROFILE_VELOCITY = 180` was defined but never actually *written*, so the servo kept whatever profile velocity the last positional move had left in the register: 80 LSB, about 110°/s. A 20-degree correction took roughly 180 ms, three or four control periods. We now write the profile velocity at the start of every closed-loop stage.

The parking loop was designed assuming 20 Hz. The log measured the real rate at about **6.8 frames per second**, because the Pico does three blocking rangefinder reads at roughly 42 ms each, and that ceiling can't be lifted from the Pi. Distance traveled per correction is what sets tracking accuracy, so instead of chasing gains we cut the speed: `PARKING_FOLLOW_SPEED_PERCENT` from 100 to 55, alignment creep from 70 to 45, plus a second `25` fine phase.

**We also revert changes when the evidence behind them doesn't hold up.** One version set the A/B sensor offset to `2.1` based on two run-log numbers that agreed to two decimal places. Both numbers came from a pose the *aligner itself had chosen*, and the aligner's whole job is to drive those two readings equal, so neither one showed the car was physically parallel. The readings also came through the parser bug above. In the same pass we turned off an integral term added because of a drift measured from those same corrupted readings. `PARKING_AB_OFFSET_MM` is back at `0.0` until somebody sets the car parallel *by hand against the chassis edge* and runs `pico_tool.py --calib`.

Every constant that replaced older behavior records the value that brings the old behavior back: `# Set MEDIAN_WINDOW=1 and SMOOTHING_ALPHA=1.0 for raw behavior`, `# Set to 1 for exact V44 stopping distance`. That way we can find which change broke something at the track by editing constants, with no git and no laptop. Dead constants are labeled `NOT USED` with a note on what replaced them.

### Metrics used to validate performance

`ParkingRunStats.report()` prints after every wall-follow stage:

| Metric | What it tells us | Action threshold |
|---|---|---|
| mean \|error\|, worst \|error\| | tracking accuracy | mean > 1.5 cm with timid steering → raise `PARKING_K()` |
| distance first / last / min / max vs the 32 cm target | where the car actually sat | — |
| least-squares drift, per sample and total | one-way creep vs oscillation | \|total\| ≥ 3 cm → integral action |
| mean \|steer\|, peak \|steer\|, % of the 30° clamp | saturated or asleep | peak < 7.5° → too timid; near the clamp → bang-bang |
| steering reversal fraction | weaving | swing ≥ 6 cm with peak > 7.5° → lower `PARKING_K()` |
| mean angle term | uncalibrated A/B sensor bias | \|mean\| ≥ 1.0 cm → recalibrate `PARKING_AB_OFFSET_MM` |
| degraded-frame share | how often we ran on one rangefinder | ≥ 40% → the data isn't tunable, fix sensing first |
| rejected lines / total, partial drops | serial link integrity | > 5% → warning printed with the offending line |

The report ends with exactly one recommendation, ranked, so two symptoms with a shared cause can't produce two contradictory suggestions. Sensor bias outranks oscillation, which outranks drift, which outranks timidity. At 40% or more degraded frames it refuses to recommend anything.

Alignment reports its own number. Once balance confirms, the car centers the steering, stops, waits `0.30 s` to settle, and measures the residual `|A − B|` at rest with `_parking_align_measure_residual()`, a median of up to five fresh frames. That residual is our alignment accuracy, measured after the car stopped rather than mid-correction.

For the gyro, `_print_integration_audit()` compares the Pico's own angle span against the Pi's independently integrated total and prints the gap as a percentage. Above 2% points at the rate deadband throwing away real motion; below that it estimates sensor over-read against the 360-degree target.

---

## Modules

| Module | File / class | Rate | Responsibility |
|---|---|---|---|
| Sensor hub | `main.py` core1 | 100 Hz | WT901 read, bias calibration, yaw integration |
| Sensor hub | `main.py` core0 | ~7 Hz | three VL53L0X reads, frame publishing, host commands |
| Camera | `add_camera()` + `center_crop` | 30 fps capture | 176×132 capture, locked exposure/AWB, 160×120 crop |
| Pilot | `KerasLinear` | 20 Hz | image → steering, throttle |
| Gyro link | `PicoWT901YawReader` | on demand | frame parsing, REPL recovery, link counters |
| Mission control | `GyroThreeLapController` | 20 Hz + 100 Hz thread | yaw-target and lockout stop, finish turn, phase sequencing |
| Startup | `ObstacleStartController` | once | parking-bay exit |
| Parking sensing | `ParkingPicoToF` | ~7 Hz | threaded frame reader, validation, freshness |
| Parking control | `parking_balance_ab_backwards`, `parking_tof_run` | ~7 Hz | alignment and wall-follow closed loops |
| Actuation | `DynamixelSteering`, `DynamixelThrottle`, `DynamixelBus` | 20 Hz | XL330 position and velocity control over one shared bus |

### Sensor hub (Pico, `main.py`)

The two cores are split by timing requirement. Yaw integration is only as good as its sample spacing, so it gets core1 to itself at a strict `WT_SAMPLE_PERIOD_MS = 10`. The rangefinders are slow no matter what, about 42 ms per read, so they get core0 where blocking costs nothing.

All three rangefinders share I²C1 and would all answer at the factory address `0x29`. `initialize_tof()` brings them up one at a time: pull every `XSHUT` low, raise one, check the sensor appears at `0x29`, move it to a private address (`0x2A`, `0x2B`, `0x2C`), check the new address answers, move on. A fourth position on GP21 is held off. Distances go out in centimeters with a status byte: `0` for a reading inside `[20, 1500] mm`, `9` for out of range, a read error, or a sensor that never came up. Reporting a fault as data instead of raising an exception lets the Pi decide how to degrade.

**One naming problem, since it will confuse anyone reading the source.** `read_tof_cm_status()` divides by ten before publishing and nothing on the Pi converts back, so every `PARKING_*_MM` constant holds a **centimeter** value despite the suffix. The math is internally consistent, so only the names lie: the wall-follow standoff is 32 cm and the alignment leeway is 1 cm. We use real units in this document and the rename is on the list at the end, deliberately not done mid-season.

Yaw on core1 is more than integrating the gyro rate. Rate bias is the biggest error term in a MEMS gyro, so the thread spends `WT_CALIBRATION_SECONDS = 3.0` averaging `gz` while we hold the car still, then subtracts that bias. The de-biased rate goes through an EMA at `WT_EMA_ALPHA = 0.18`, then an asymmetric scale correction, `1.00621` for positive rates and `1.00007` for negative, because our unit over-reports one turn direction more than the other. Integration is trapezoidal, `yaw += 0.5 · (prev + current) · dt`, which halves the systematic error a rectangular sum builds up when the rate is changing. Any `dt` above `WT_DT_MAX_SECONDS = 0.05` is thrown out so a scheduling hiccup can't inject a false step. Whenever the filtered rate stays under `WT_STILL_THRESHOLD_DPS = 1.0` for `WT_STILL_HOLD_SECONDS = 0.6`, the bias estimate re-adapts at `WT_BIAS_ADAPT_ALPHA = 0.015`, since bias drifts with temperature over a three-minute round.

`poll_usb_command()` drains the host link without blocking and `handle_usb_command()` acts on complete lines, taking `restart` (hardware `reset()`) and `reset gyro`. The onboard LED blinks every 250 ms from both cores, so a dark LED means `main.py` isn't running and we can see that across the pit table.

### Actuation

Both XL330 servos share one `DynamixelBus` singleton over the U2D2, reference counted so the port closes once when the last part lets go. `DynamixelSteering` runs in position mode, `DynamixelThrottle` in velocity mode. `run_for_degrees()` uses extended position mode so a drive move can cross the single-turn boundary, and checks arrival within `RUN_FOR_DEGREES_TOLERANCE_TICKS = 15` (about 1.3 degrees), polling every 20 ms, with a timeout of the predicted duration plus a 3 s margin.

Steering maps a normalized command to ticks as `angle · 0.8`, clamped to `±1`, spread across `DXL_STEER_LEFT_DEG = -60 … DXL_STEER_RIGHT_DEG = +60`, then `3060 + steer_deg · 11.3778` ticks. Since the `0.8` gain is applied *before* the clamp, the clamp is unreachable for any legal input and the real steering range is **±48°, not ±60°**, with full lock at 3606 and 2514 ticks. The network can never command the positions closest to full mechanical lock, where our chassis binds. Goal writes are change-gated, so a steady command costs no bus traffic.

Throttle quantizes to 10% steps, which collapses anything under 0.05 to a true zero and leaves 21 levels. That keeps the network's frame-to-frame jitter from becoming constant tiny speed changes on the drive motor.

---

## Lane Following

Lane following is **end-to-end behavioral cloning**. There's no edge detector, no color threshold, no Hough transform, and no explicit lane model anywhere in our code. A `KerasLinear` convolutional network takes the 160×120×3 RGB crop and outputs two numbers, steering and throttle, at the 20 Hz drive loop rate.

### Why a learned controller

The hard part of this challenge isn't the geometry, it's that the track's *appearance* changes: venue lighting, shadows from spectators, glare on the mat, and black walls our rangefinders already struggle with. A threshold-based pipeline has to be re-tuned for all of that on site, and we chose to spend our build hours collecting data instead.

Lane following and obstacle avoidance are also *the same decision*. A geometric wall follower plus a separate pillar-avoidance module needs an arbitration layer to pick a winner, and the two controllers disagree in exactly the situations that matter, like a pillar sitting near a corner. A single network that outputs one steering angle has no arbitration layer to get wrong.

### Locking down the camera

A learned controller is only as stable as the images you feed it. `add_camera()` waits `0.75 s` for the sensor to settle, then turns off every automatic behavior: `exposure_mode = 'off'`, `awb_mode = 'off'`, `iso = 100`, `shutter_speed = 15000` µs, `exposure_compensation = 0`, and fixed `awb_gains = (1.5, 1.2)`.

With auto-exposure on, driving from a bright straight into a shadowed corner changes the brightness of the whole frame and the network sees an image unlike anything it trained on. Locked, a red pillar has the same pixel values in training and at the competition. The cost is dynamic range, and venue lighting has to be close enough to what we trained under, which is why we re-record a calibration dataset on site.

`center_crop()` takes a 160×120 window, centered horizontally and shifted **6 pixels up** (`y0 = (h - th) // 2 - 6`). At our 176×132 capture that puts the crop at the top of the frame and throws away the bottom rows, which are mostly our own chassis and the mat right in front of the bumper. `KerasLinear` is built with `input_shape = (120, 160, 3)`, so changing the crop invalidates every model we've trained.

### Direction-specific models

We ship **four** models instead of one:

| Model | Challenge | Direction | Path |
|---|---|---|---|
| `FCW` | Open | clockwise | `models/fcwm/…/mypilot.h5` |
| `FCCW` | Open | counter-clockwise | `models/fccwm/…/mypilot.h5` |
| `OCW` | Obstacle | clockwise | `models/ocwm/…/mypilot.h5` |
| `OCCW` | Obstacle | counter-clockwise | `models/occwm/…/mypilot.h5` |

Driving direction is randomized per round, so both have to work. One direction-agnostic network would need roughly twice the data, since it would have to learn the mirror symmetry we already know for free. Direction is announced before the round, so picking a model is a startup decision: `--drive-mode FCW|FCCW|OCW|OCCW`, or a Sense HAT joystick pick (left, right, up, down, middle to confirm) when we're running without a keyboard. Passing `--model` works too, and the mode gets inferred from the path by substring match, testing `occw` and `fccw` before `ocw` and `fcw` so a counter-clockwise path can't be misread.

### Data collection

Recording mode (`--recording`) builds a different part graph: camera and crop, a live preview window, a PS4 controller, both actuators, and a DonkeyCar `TubWriter`. Steering comes from pygame axis 0 (left stick X) and throttle from axis 4 inverted (right stick Y on our controller's Linux mapping), both through a `JOYSTICK_DEADZONE = 0.05` deadzone. Recording is gated by a condition part rather than a button: a frame is written only when the mode is `"user"` *and* `|throttle| > RECORD_THRESHOLD = 0.05`. A parked car would otherwise contribute a label that says "steering while not moving."

Bad demonstrations are worse than no demonstrations, so we made deleting them easy. Triangle (`TRIANGLE_BUTTON = 2`) emits an `"erase"` mode that a `PromptWiper` part picks up, hard-deleting the newest **100** records (images, JSON records, catalog entries) with a `0.25 s` release debounce so one press can't fire twice. `STOP_BUTTON = 4` (L1 on our pad) raises `KeyboardInterrupt` for a clean shutdown that centers the steering and releases torque. During autonomous runs `Esc` or closing the preview window does the same, and that's our software kill switch.

<!-- Fill in before submission: dataset size per model (record counts), number of recording sessions, and the exact `donkey train` invocation and hyperparameters used. -->

---

## Obstacle Logic

A **red** pillar gets passed on its **right**, a **green** pillar on its **left**, and moving a pillar outside the 85 mm circle around its seat ends the round. After the third lap the traffic rules stop applying and the car has to find the parking bay.

### The pillar decision is learned, not coded

Our `OCW` and `OCCW` models are trained on obstacle-course demonstration data, so red-right and green-left live in the network's weights instead of in an `if` statement. The color is already in the 160×120 RGB crop, the human driver supplies the correct side, and the network learns the mapping along with everything else about the track.

The cost is that we can't inspect the decision. There's no line of code we can point at to prove the car will pass a red pillar on the right, only runs where it does. Whether it's correct depends on how well our dataset covers the cases, so our testing has to be *scenario* tests rather than unit tests, and a pillar placement that never appears in training is a placement where behavior is undefined. The network is never trusted with anything exact: it doesn't decide when the run ends, where to stop, or how to park.

### Getting out of the parking bay

An obstacle round starts with the car inside the parking bay, so `obstacle_start_program()` runs once first, called by `ObstacleStartController`. It takes no environmental feedback. The loop closes only on wheel rotation read back from the servo's own position register, since we placed the car in a bay of known size and there's nothing to sense. Clockwise steers right at `+65°` for 400 motor degrees, then left at `-50°` for another 400, then centers. Counter-clockwise isn't a mirror image but a separately tuned sequence: `-65°` for 430, `+60°` for 450, then two reverse moves of 1500 and 600 motor degrees at `-7°` and `+15°`. The bay sits differently relative to the first corner depending on direction, and backing up further keeps the car behind the first traffic sign.

The maneuver runs synchronously inside the drive loop, which is how it gates the model: while it executes the vehicle loop doesn't advance, so no prediction reaches the servos. The `DriveControlMux` part downstream is a `None`-safe pass-through and a hook for a future non-blocking startup controller. When the maneuver finishes, `restore_throttle_velocity_mode()` puts the drive motor back in velocity mode and the steering is centered. One thing we accept: the first command afterward comes from a prediction made on a pre-maneuver frame, stale by a single 50 ms cycle.

### Ending the third lap in the right place

After the lap target confirms, the car keeps driving under model control for `OBSTACLE_RUN_FINISH_SECONDS = 1.0` more, a distance expressed as a time and hand-tuned at our current speed. Then `gyro_obstacle_finish_turn()` takes over.

That correction is a closed loop on the same integrated yaw. It computes `270°` as `GYRO_TARGET_DEG − GYRO_OBSTACLE_FINISH_OFFSET_DEG`, takes the difference between the yaw where the model stopped and that figure, skips the turn if the difference is inside `GYRO_OBSTACLE_FINISH_TOLERANCE_DEG = 2.0`, and clips it at `GYRO_OBSTACLE_FINISH_MAX_TURN_DEG = 140.0` so a bad yaw estimate can't command an unbounded rotation. Then it steers `-50°` for a clockwise run or `+50°` for counter-clockwise at `0.35` throttle, polling yaw every 10 ms until the rotation is within 2 degrees of target or `15 s` runs out. This turn has its own `GYRO_FINAL_TURN_MULTIPLIER` so calibrating the lap threshold doesn't force the final correction early or late. The `finally` block stops the drive motor, re-centers the steering, and restores velocity mode, so parking always starts from the same actuator state.

### Parking: five deterministic stages

Full marks need the car's projection entirely inside a bay 20 cm wide and parallel to within 2 cm of wheel-distance variance, and touching the magenta boundary elements zeroes the parking score. Two centimeters over the length of our car is a small angle, so parking is hand-written with its own sensors, calibration, and gains.

`run_parking_sequence_after_obstacle()` runs the stages in order and lets a failure stop the run rather than pressing on blindly.

**Stage 1, approach arc.** A full-left forward arc, currently skipped because `PARKING_SKIP_TO_C_TOF_STAGE = True` — the gyro finish turn already leaves the car in the pose it was written to produce. We keep it for the standalone parking program and for layouts where the finish turn is off.

**Stage 2, front wall approach (rangefinder C).** Drive forward at full speed with a `-1.0°` left trim until the forward-facing rangefinder reads `PARKING_FRONT_STOP_MM + PARKING_C_DISTANCE_LEEWAY_MM`, 5.7 cm, then stop. Capped by `PARKING_APPROACH_TIMEOUT = 12.0 s`, and a rangefinder invalid for `PARKING_SENSOR_INVALID_TIMEOUT = 0.8 s` aborts. This sets the reference for how far forward the car is.

**Stage 3, mirrored reverse arc.** Reverse at full lock for `PARKING_TURN_MOTOR_DEGREES = 720` motor degrees, steering `+50°` clockwise or `-50°` counter-clockwise. This is the only place the two directions differ in parking, and only in steering sign.

**Stage 3B, squaring up (rangefinders A and B).** A and B look at the same side wall from different points along the car, so their difference is proportional to how far the car is rotated relative to that wall. Reversing slowly while driving `|A − B|` to zero makes the car parallel, which is what the rules score. Details below.

**Stage 4, wall follow (rangefinders A and B).** Creep forward holding a `PARKING_WALL_TARGET_MM` standoff, 32 cm, under weighted proportional control until the bay opening shows up. Details below.

**Stage 5, bay entry.** Nine open-loop steer-and-drive pairs netting `+470` motor degrees: a `+1250`-degree forward move at a `-3°` trim, a `+45°` hard-right reverse of `-750`, a short `+200` forward at `-6.25°`, then six alternating full-lock `±50°` moves (`-300`, `+180`, `-280`, `+150`, `-180`, `+200`) that shuffle the car square in the bay. Wheel rotation only, no environmental feedback, since the car is inside a 20 cm slot where the rangefinders have no useful line of sight.

---

## Algorithm Explanation

### Yaw estimation: why we integrate rate instead of using the sensor's yaw

The WT901 reports a yaw angle directly and we don't use it. Its yaw depends on the magnetometer, and the mat sits on a floor with an unknown amount of metal in it, next to other robots' motors. So we integrate the *rate* instead, on both processors, and cross-check them.

The Pico integrates as described above. The Pi's 100 Hz sampler thread does its own accumulation from the reported angle, and that's where the guards live. `_unwrap_delta_deg()` wraps each frame-to-frame difference into `±180°` so a wrap doesn't register as a 360-degree jump. Deltas implying a rate under `GYRO_RATE_DEADBAND_DEG_PER_SEC = 1.5` get zeroed, since integrating sub-degree noise for three minutes builds up a big fake rotation while the car is going straight, and going straight is most of a lap. Deltas implying more than `GYRO_MAX_VALID_RATE_DEG_PER_SEC = 500` get rejected as corruption, **but the reference value still gets resynced**, so one bad frame doesn't cause a second, bigger error on the next good one.

The deadband throws away a little real motion on gentle curves, which is what the integration audit is for. It only prints on a confirmed target, though, so a run we stopped by hand gives no audit at all.

Sampling at 100 Hz rather than the 20 Hz drive-loop rate matters because yaw error from discrete integration scales with sample spacing, and the drive loop is too slow to catch a corner's rate profile.

If the stream goes stale for `PICO_GYRO_STALE_TIMEOUT_SECONDS = 1.50`, the sampler reopens the port, up to `PICO_AUTO_REOPEN_MAX_ATTEMPTS = 3` times with a `6.0 s` cooldown, soft-rebooting the Pico on the first two only. We raised the stale timeout from 0.6 s because one slow rangefinder read can open a gap that long, and the cooldown has to exceed the Pico's own startup time or a retry fires while the previous one is still initializing.

### Wall alignment: proportional control with a settling test

Stage 3B drives `|A − B|` to zero. Version 44 did it with bang-bang control, a fixed `±10°` whenever the difference passed the leeway, which with sensor latency and a slow servo guarantees a limit cycle: the car sweeps *through* balanced instead of settling on it. It could also confirm success mid-swing, since two frames go by in about 100 ms at 20 Hz while the car is still rotating.

Steering is now proportional, so the correction shrinks as the error does:

```
signed_diff = (A_filt - B_filt) - PARKING_AB_OFFSET_MM
magnitude   = clamp(PARKING_AB_ALIGN_KP_DEG_PER_MM · |signed_diff|,
                    PARKING_AB_ALIGN_MIN_STEER_DEG,     #  1.5°
                    PARKING_AB_BALANCE_STEER_DEG)       # 10.0°
steering    = -magnitude  if signed_diff > 0  else  +magnitude
```

`PARKING_AB_ALIGN_KP_DEG_PER_MM = 2.0` is 2 degrees per centimeter in real units, so a 1 cm difference commands 2 degrees and the law saturates at 5 cm. Speed is two-phase: `-45%` while the difference is over `PARKING_AB_ALIGN_FINE_DIFF_MM = 4.0` (4 cm), then a `-25%` creep so the car can stop *on* balance instead of coasting through it. Angular error per frame of latency scales directly with speed, so slowing down was the cheapest accuracy improvement available.

Confirmation needs three conditions:

```
within_leeway = |signed_diff| <= 1.0 cm
settled       = |d(signed_diff)/dt| <= 8.0 cm/s
confirmed     = within_leeway AND settled, for >= 3 frames AND >= 0.15 s
```

The rate test stops a mid-swing confirmation, and when the car is inside the leeway but still moving too fast the controller commands **zero** steering and coasts, letting the rate decay instead of fighting it. Any failing frame resets both the count and the span timer.

The gain is 2 degrees per centimeter and a VL53L0X throws single-sample spikes, so each sensor runs a median-of-3 spike rejector followed by a light EMA (`α = 0.70`). A heavy EMA alone would cost about 1.2 frames of phase lag, which at 7 Hz means aligning to where the car *was*.

Invalid readings get two different responses. A dropped *frame* holds the previous command exactly, since yanking the steering straight over one late frame is a worse disturbance than the missing data. An *invalid sensor* is different: alignment controls purely on `|A − B|`, so with one sensor gone we have no angle information and no correct direction to turn. The rotation already in progress is usually what swung the sensor to the 35–45 degree angle that caused the dropout, so holding it keeps rotating further into the blind spot and the reading never comes back. After a single grace frame the controller straightens (`steering = 0.0`) and keeps reversing, which is what our earlier standalone parking programs did.

The stage is capped at `PARKING_AB_BALANCE_TIMEOUT = 15.0 s`, and on expiry it goes into the wall follow anyway (`PARKING_AB_BALANCE_TIMEOUT_CONTINUE_TO_PID = True`), since a partly aligned car can still earn partial parking points.

### Wall following: two error terms with separate weights

Stage 4 holds a fixed standoff while creeping toward the bay opening. The error has two physically different parts:

```
angle_term    = (A - B) - PARKING_AB_OFFSET_MM         # how rotated we are
distance_term = (A + B)/2 - PARKING_WALL_TARGET_MM     # how far out we are
error         = 1.5 · angle_term + 1.0 · distance_term
steering      = 1.8 · error
```

`(A − B)` is the wall-angle term. A and B sit at different points along the same side of the car, so their difference is proportional to rotation relative to the wall. It's already a derivative-style lead term and it's what **damps** the approach. `((A + B)/2 − T_d)` is the cross-track term: how far off target we are, and what **drives** the car back. A stable wall follower needs the angle term weighted heavier, hence `PARKING_PID_ANGLE_WEIGHT = 1.5` against `PARKING_PID_DISTANCE_WEIGHT = 1.0`.

Version 44 weighted them 1:1 with a gain of 2.0, so a car parallel but 15 cm off target already saturated the `±30°` clamp and the controller was effectively bang-bang for most of the run. Separating the weights let the damping term dominate instead of tie.

The gain itself had to be raised twice, `1.2 → 1.8`, because the car visibly barely turned. Replaying a run log gave mean steering of 2.2 degrees and a peak of 9.4 against a 30-degree clamp, with 64% of frames under 2 degrees. At `1.8`, a car parallel and 1 cm off target commands 1.8 degrees and saturating the clamp needs roughly 16.7 cm of offset, so the loop keeps headroom.

Tuning order is written next to the constants: if the car weaves, lower the gain first and only then raise the angle weight; if it's smooth but sits at the wrong distance, raise the distance weight; if it corrects too slowly, raise the gain. Raising the gain is the safe lever because solving `1.5·(A−B) + 1.0·(avg − T_d) = 0` gives `avg = T_d − 1.5·(A−B)`, which has no gain term in it. A residual A/B sensor bias produces a fixed standoff error of 1.5 units per unit of bias regardless of gain, so raising the gain converges faster to the same equilibrium.

`PARKING_PID_KI` and `PARKING_PID_KD` both sit at **zero**. The angle term already provides the damping a wall follower needs, and at 7 Hz integral action is what causes integral-induced oscillation. When they are turned on, the integrator is conditionally gated, frozen while the output is clamped so it can't wind up, and the derivative is filtered at `α = 0.30`. The output chain then applies a `0.2°` deadband, the `±30°` clamp, and a `150°/s` slew limit, since the servo can't follow a step anyway.

### Degraded mode: estimating a missing sensor instead of substituting a constant

A VL53L0X returns nothing when it sits 35–45 degrees off a **black** wall, and the rules make every wall black, so single-sensor dropouts are normal here rather than a rare edge case.

The error uses A and B *jointly*, so any fixed substitution for a missing sensor invents an angle the car isn't at. Substituting the target distance is the worst case: if B drops out while A reads 40 cm and the target is 32, the angle term becomes `40 − 32 = +8 cm` of pure fiction and the controller swerves to correct a rotation that doesn't exist. A fixed maximum has the same problem, only bigger.

So we estimate instead. The valid sensor still reports distance correctly and only the *angle* becomes impossible to measure, so we hold the angle at its last known-good value and reconstruct the missing reading from it:

```
B invalid  ->  B_est = A - last_good_angle
A invalid  ->  A_est = B + last_good_angle
```

The angle term then sits at the last real measurement instead of jumping, while the distance term keeps tracking the sensor that works. A held angle decays by `PARKING_PID_DEGRADED_ANGLE_DECAY = 0.85` per degraded frame, roughly half in 0.6 s at our frame rate, so a long dropout settles into pure distance-following rather than steering on a stale angle. Output is scaled by `PARKING_PID_DEGRADED_GAIN_SCALE = 0.6` and the clamp tightens from 30 degrees to `PARKING_PID_DEGRADED_MAX_STEER_DEG = 12.0`. Scaling the output rather than the gain keeps the controller's internal state clean, so recovery is immediate. If both sensors fail, the last filtered values hold for `PARKING_PID_INVALID_HOLD_SECONDS = 0.30` before the target substitution kicks in, and `0.8 s` of both-invalid aborts the stage.

### Finding the bay: requiring a step, not just a small reading

"A is close" can't tell *found the bay* apart from *drifted into the wall I was following*, because A is small either way. One captured log came within a centimeter of the mistake: four frames in a row at A ≈ 21 cm with B ≈ 21 cm and no step between them, purely because the car had drifted 8 cm inward. The real trigger a moment later read A = 16.8 cm against B ≈ 21 cm, a genuine 4.2 cm step.

So detection needs three conditions together:

```
raw A <= PARKING_PARK_TRIGGER_MM + PARKING_WALL_FIND_LEEWAY_MM   # 20.0 cm
(B_filt - raw A) >= PARKING_WALL_FIND_MIN_STEP_MM                #  2.5 cm
both true for PARKING_WALL_FIND_CONFIRM_READINGS = 2 consecutive frames
```

Entering a bay drops A while B still reads the wall, so a *difference* appears; drift moves both together. Detection uses **raw**, unfiltered A so there's no filter lag on the one measurement where stopping position depends on latency, and the two-frame confirmation costs about one frame of extra travel. A rejected candidate is logged as rejected, so a drift run looks different in the log from a run that never saw the bay.

### Known limitations and next steps

| Area | Current limitation | Planned improvement |
|---|---|---|
| Lap detection | `GYRO_TARGET_DEG = 360.0` is roughly one lap, so the 32 s lockout, not the yaw target, is what ends the run. Run length is a hand-timed constant and a slow lap would get cut off | Raise the target to a measured three-lap figure near 1080° so the yaw condition becomes the one that actually decides and the timer becomes a floor |
| Rangefinder rate | Three blocking reads cap the parking control loop near 7 Hz, which sets how well we can track | Move to continuous or interrupt-driven VL53L0X reads, or stagger the three sensors across Pico loop iterations |
| A/B sensor offset | `PARKING_AB_OFFSET_MM` is still `0.0`, and unit-to-unit mismatch shows up as 1.5 units of standoff error per unit of bias | Measure it once with the car set parallel by hand against the chassis edge, using `pico_tool.py --calib` |
| Constant naming | Every `PARKING_*_MM` name holds a centimeter value, because the Pico publishes cm and the Pi never converts | Rename to `_CM` in one single pass, out of season, with a replayed log to confirm no controller changed behavior |
| Mission state | The phase sequence lives in booleans on `GyroThreeLapController`, so it can't be inspected or logged as one value | Promote it to an explicit enum with a single dispatcher, and log the transition on every change |
| Finish turn geometry | The correction uses an absolute yaw difference, so overshoot and undershoot look the same and the steering sign is fixed by direction | Use the signed error so an overshoot corrects back instead of turning further |
| Bay entry | Stage 5 is fully open-loop and shares one tuned sequence for both directions | Add a rangefinder check between the shuffle moves to verify progress instead of assuming it |
| Obstacle decisions | Learned rather than explicit, so correctness depends on dataset coverage and can't be proven by inspection | Log per-frame pillar-passing outcomes against a scripted placement matrix to measure coverage directly |
| Startup handoff | The first command after the exit maneuver comes from a pre-maneuver frame, stale by one 50 ms cycle | Make the startup controller non-blocking and use the `DriveControlMux` gate it was written for |
| Model calibration | The trained models compensate for whatever bias is in `DXL_STEER_CENTER_TICKS = 3060`, so it can't be corrected without retraining | Re-measure the mechanical center and retrain, as one combined change |


## Systems Thinking & Engineering Decisions:

**Subsystem interactions**

**Constraints, Trade-offs, and Risk Analysis:***

**Iteration cycles:**

**Engineering Reasoning:**

## Conclusion: 

