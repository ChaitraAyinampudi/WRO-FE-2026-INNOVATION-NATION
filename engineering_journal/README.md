# Engineering Journal

This file records the decisions, tests, failures and fixes from building the robot. It's kept in rough chronological order, and it includes the things that didn't work, because most of what's in the current code exists because of something that broke first.

Version numbers below are the ones in each file's own header comment. The manage program's changelog runs from V6 through V45i.

---

## Timeline

| Period | Focus |
|---|---|
| 2025 season | Previous robot: OpenCV color thresholds, ROI-based steering, ROS2 nodes, orange-line lap counting |
| June 2026 | Migrated from Build HAT motors to DYNAMIXEL XL330 + U2D2. Ported manage to the new motor stack. Camera view variants, tub recording, record deletion. |
| Late June | Added the obstacle parking-exit routine and the `--driving-skip` variants. Set the camera view to 240×160 and moved data and models to `~/WRO_FE_2026`. |
| July | Gyro lap detection. Long stretch of trying to make a Sense HAT gyro produce reliable lap counts, then giving up on it. |
| July 29–31 | Finish-turn correction, forward vs backward, independent gyro thread, and integrating standalone parking into manage. |
| August 1–2 | WT901 moved onto the Pico. Serial reliability work, no-input recovery, port conflicts, faster A/B alignment. |
| August 3–4 | V45 series: found the real cause of the recurring gyro failures, then reworked both parking control loops. |

---

## Major decisions

**Build HAT out, DYNAMIXEL XL330 + U2D2 in.** The Build HAT was the previous season's motor setup. Switching gave us position mode and velocity mode on the same bus, closed-loop position feedback we could actually read back, and two motors on one cable. That feedback is what makes `run_for_degrees()` possible, and repeatable motor-degree moves ended up being the backbone of the parking sequence.

**Raspberry Pi runs the camera and the model. Pico reads the sensors.** Yaw integration needs a strict 10 ms sample period, and the Pi can't promise that while running TensorFlow at 20 Hz. Splitting them also means a stuck sensor read can't stall the drive loop.

**Sense HAT gyro out, WT901 on the Pico in.** This was the biggest single change of the season and it took a long time to commit to. The Sense HAT gyro failed in four separate ways during July: the reported angle drifted upward linearly while the robot sat still, the robot sometimes stopped around the second lap when the target was set for three, updates were too slow to catch a corner, and the final correction angle never matched the physical turn. We tried target values of 1080°, 1440°, 340° and 360° trying to make the numbers line up before accepting that the sensor was the problem, not the threshold.

**End-to-end learned driving instead of color thresholds.** Last season's robot used HSV thresholds with fixed regions of interest, and lap counting worked by counting orange line crossings. It needed re-tuning at every venue. This year the camera feeds a trained network directly and lap counting comes from the IMU, so there are no thresholds to re-tune on site.

**ToF sensors for parking, not the camera.** The rules score parking parallel to within 2 cm of wheel-distance variance. A network trained on 160×120 images has no way to hit that. Two side rangefinders reading the same wall from different points along the car give us rotation and distance directly.

**Parking constants kept separate from driving constants.** Every parking value is prefixed `PARKING_*` and none of them are read by the model-driving path. This started as tidiness and turned out to matter: parking has its own steering center, and merging the two would have broken the trained models.

**Old versions kept, never overwritten.** Each behavior change becomes a new numbered file with the reason written into the header. We've gone back to an older version to bisect a regression more than once.

---

## Version history

### manage

| Version | Change |
|---|---|
| V6 | Camera-view variant |
| V7 | Added `--driving-skip` and `--driving-view-skip` to bypass the parking-exit routine |
| V13–V16 | Gyro debugging. Target moved to 1440°, then attempts at faster gyro updates |
| V17 | Last full snapshot on the Sense HAT gyro architecture |
| V18 | Obstacle finish correction: compute the finish turn from yaw error |
| V19 | Finish turn done in reverse |
| V20 | Reverse realignment, then forward correction |
| V21 | Removed the reverse realignment. Forward-only correction from here on |
| V24–V25 | `TypeError` on `now - gyro_target_first_seen` when the timestamp was still `None`. First fix attempt didn't hold |
| V26 | Parking integration rebuilt on the V21 base, with parking constants near the top and the sequence starting at the C-ToF stage |
| V27 | Parking enabled for both OCW and OCCW, with the pre-park reverse turn mirrored |
| V28–V29 | Removed a separate degree-scale factor. Established that the timer is a lockout on the stop, not a stop condition |
| V30 | Gyro reading moved to its own thread, independent of the 20 Hz drive loop |
| V31 | Target settled near 360° |
| V32 | Finish correction came up about 35° short in one test. Logged end yaw 414.2°, target−90 = 270.0°, calculated turn 144.2°, physical result about 142.5° |
| V34 | Switched to the Pico WT901 stream |
| V37 | Parser accepts `Angle, A, B, C, D` without requiring status fields |
| V38 | Ctrl-C + Ctrl-D restart on run, gyro reset commands, auto-restart on no input |
| V39 | Turned restart-on-run back off, because writing to a Pico that was already streaming broke it |
| V40 | No-input recovery reopens the serial port first instead of resetting the gyro. Added serial-blocker cleanup |
| V41 | Re-enabled Pico soft restart at startup after a manual Ctrl-C/Ctrl-D test worked reliably |
| V42 | A/B alignment: 1.0 leeway, 10° steering, 70% speed, faster reads, smoothing, two good frames required, timeout continues into PID |
| V43 | More serial startup and recovery adjustments |
| V44 | Fixed the gyro link. Startup timeout 1.20 s → 8.00 s, kept DTR asserted, added byte/line/parse counters, raised the stale timeout |
| V45 | Reworked both parking control loops. Eight separate root causes, listed below |
| V45b | Found the actual cause of the recurring gyro failures: raw REPL |
| V45c | Added the startup auto-fix. Angle weight 2.0 → 1.5 |
| V45d | Follow speed 80 → 55 after measuring the real frame rate. Added the wall-find step requirement |
| V45e | Reverted the A/B offset and the integral term. Fixed the parser |
| V45f | Degraded-mode angle hold, and straighten-when-invalid during alignment |
| V45g | Wall-follow gain 1.2 → 1.8 |
| V45h | Parking steering center 3136 → 3126, offset 0.8 → 0.7 |
| V45i | Worked out which constants scale with speed and which don't |

### Standalone parking program

Parking was built as its own program first so it could be tested without driving a lap in front of it.

| Version | Change |
|---|---|
| v22 | First full sequence: C approach, reverse turn, A/B balance, PID wall follow, entry |
| v26 | Second turn read A and B, stopping when the difference reached 1–2 |
| v27 | Separated the second-turn timeout from the sensor-invalid timeout |
| v28 | Second turn became a fixed motor-degree move, with alignment as a separate reverse step |
| v29 | Required the PID target to be reached twice before leaving |
| v30 | Align backwards before the straight section |
| v31 | Parking wall detected from B |
| v32 | Second alignment moved forward. Longer wall search |
| v33 | Alignment logic reversed |
| v34 | Stop only when both the distance target and the alignment leeway were satisfied |
| v35 | Removed the second alignment. PID only. Wall detection moved to A |
| v36 | Alignment timeout continues into PID instead of failing the run |
| v37 | Alignment skipped entirely |
| v38 | Alignment restored, choosing forward or backward based on the size of the A/B difference |

v35 and v38 are the versions that parked reliably, which is why the integrated code uses their steering calibration rather than the model's.

---

## Testing notes

We test bottom-up, one layer at a time, because a bad calibration and a bad control loop look identical on the track.

**Motors before autonomy.** Torque enable and disable, reading present position, reading motor IDs, then a square-driving test at ±45°. That test found two real bugs: both directions initially turned the same way, and negative speed was being ignored instead of reversing the move.

**Pico serial on its own.** We wrote three Pi-side viewers (`read_pico_serial.py`, plus reset and restart-main variants) to watch `/dev/ttyACM0` and parse frames without loading manage. This later became `pico_tool.py` with `--monitor`, `--fix` and `--calib`. Debugging the link without waiting on TensorFlow turned a ten-minute cycle into a ten-second one.

**ToF timing swept by hand.** We tested read timing, measurement budget and inter-measurement period as a set. `TOF=42 ms, TIMING=33000 µs, INTER=40 ms` worked best. That 42 ms per read is also what caps the parking loop near 6.8 Hz, which we didn't realize until much later.

**Parking standalone before integration.** Thirteen numbered versions before it went into manage.

**Training data recorded then cleaned.** A frame is only recorded when the mode is `"user"` and throttle is above 0.05, so a stopped car can't teach the network to steer while stationary. Bad runs get deleted on the spot with the Triangle button, which removes the newest 100 records.

**Log replay for tuning.** The controllers print statistics at the end of each parking stage, and most of the V45 changes came from replaying those logs rather than from watching the car.

---

## Problems and fixes

| Problem | What we saw | Cause | Fix |
|---|---|---|---|
| Pico stopped streaming | Manage reported no gyro while the Pico's LED was still blinking | Several causes, found one at a time | Three recovery layers: free the port, auto-fix before the model loads, in-reader escape with bounded reopen |
| Gyro dead after using Thonny | Pico silent, `bytes=0` on the first probe, then `raw REPL; CTRL-B to exit` in the log | Thonny, `mpremote`, `ampy` and `rshell` leave MicroPython in the **raw** REPL. Ctrl-D means "soft reboot" in the friendly REPL but "run the buffer" in the raw one, so our handshake did nothing | Escape sequence is now Ctrl-C, Ctrl-C, **Ctrl-B**, Ctrl-D. This was the real cause behind months of intermittent gyro failures |
| Every `print()` on the Pico discarded | Port opened fine, zero bytes received | V43 de-asserted DTR after opening. MicroPython on RP2 gates USB output on `tud_cdc_connected()`, which follows DTR | Force `dtr = True, rts = False` at every open |
| Writing to a healthy Pico broke it | Output stopped after we sent the handshake | MicroPython buffers `\x04` as ordinary stdin data, which blocked the Pico's read loop | Listen silently for 2.00 s first, only write if the port is provably silent |
| Startup always timed out | Every run closed and reopened the port, killing `main.py` mid-init | V43 allowed 1.20 s for the first frame. The Pico needs about 1.7 s just to bring up three rangefinders | Startup timeout raised to 8.00 s. Stale timeout 0.60 → 1.50 s, reopen cooldown 1.50 → 6.00 s |
| One dead sensor silenced everything | Pico printed one fatal line and went quiet, even though the IMU thread was fine | A failed VL53L0X init returned from `main()` | Each sensor initializes independently. A failure reports `-1` with status `9` forever and the frame keeps flowing |
| A stray byte froze the Pico | All output stopped, no error | `sys.stdin.readline()` blocks until a newline arrives | `select.select(..., 0)` and read one character at a time |
| Corrupted frames mid-line | Lines like `Angle=12.[WT901 READ ERROR 3]` | Both cores printing without a lock | `send_line()` takes a lock |
| Bad readings that looked plausible | Distances biased low, worst possible direction for a wall follower | On a damaged labeled line, status numbers slid into the distance slots and got accepted | Positional parsing only when the line carries no labels at all, and every required field must appear exactly once |
| ToF returned 8191 mm | Constant out-of-range value | Range limit, weak reflection off dark surfaces, loose connections, XSHUT address collisions | Invalid readings report `-1.0` with a status byte instead of a fake distance |
| ToF dropouts against black walls | Sensors returned nothing at certain approach angles | A VL53L0X 35–45° off a black wall often returns nothing, and every wall on this field is black | Reconstruct the missing reading from the last known angle, decay it, and cut controller authority while degraded |
| `OSError: EIO` on the ToF bus | Intermittent I²C failures | Bus and wiring issues | Per-sensor error handling, and the sensor reports as missing rather than crashing |
| Steering center kept changing | Three different values used across the season: 3060, 3126, 3136 | Mechanical adjustments moved where straight actually was, and the trained models had already learned the old value | Parking and driving use separate calibration. 3060 stays because the models compensate for it |
| Parking corrections did nothing | Steering commands changed every frame, car barely turned | `PARKING_STEERING_PROFILE_VELOCITY = 180` was defined but never written, so the servo kept the 80 LSB value from the last positional move, about 110°/s. A 20° correction took 180 ms | Write the profile velocity at the start of every closed-loop stage |
| Alignment oscillated instead of settling | Car swept through balanced repeatedly | Bang-bang control, fixed ±10° with no reduction near balance | Proportional steering at 2°/cm, plus a two-phase speed with a slow creep near balance |
| Alignment confirmed while still moving | Reported success at a crooked angle | Two frames pass in about 100 ms at 20 Hz even mid-swing | Added a settling test on the rate of change, three confirmations, and a minimum 0.15 s span |
| Wall follower weaved | Constant oscillation along the wall | Both error terms weighted 1:1 with a gain of 2.0. A car parallel but 15 cm off target already saturated the ±30° clamp | Separated the weights, 1.5 for angle and 1.0 for distance, and dropped the gain |
| Then the car barely turned | Log said LEFT while the car looked straight | Gain had been dropped too far. Replay showed mean steering 2.2°, peak 9.4°, 64% of frames under 2° | Gain raised 1.2 → 1.8 |
| Parked in the wrong place | Run stopped early | A single spurious short reading triggered wall detection | Two-frame confirmation, plus a required B−A step of 2.5 cm so drifting into the followed wall isn't mistaken for the bay |
| Gyro-based turns unreliable | Timed out, stopped at 79.4° in one case, overshot to 95–99° in others | Sense HAT gyro | Fixed motor-degree turns using `run_for_degrees()` |
| Two Raspberry Pis stopped booting | Red and green LEDs both solid, known-good SD card made no difference | Not fully diagnosed. Suspected voltage stability or spikes on the shared rail | Still open. This is part of why splitting the motor supply is our top hardware priority |
| `NameError: read_tof_cm_status` | Pico wouldn't start after a firmware merge | Helper lost during a revision | Restored the function |
| `No module named donkeycar` | Manage wouldn't start | Environment not activated on a fresh Pi | Part of the Pi setup checklist now |
| `/dev/ttyUSB0` missing | U2D2 not found | Permissions | `sudo usermod -a -G dialout $USER`, then log out and back in |
| Triangle delete unreliable | Sometimes fired twice, sometimes not at all | No debounce | 0.25 s release debounce before it can re-arm |

---

## Changes we made and then reverted

Two of these are worth recording because the reasoning that led to them looked sound at the time.

**A/B sensor offset set to 2.1, then back to 0.0.** We took two numbers from a run log that agreed to two decimal places, which felt like strong evidence for a real sensor mismatch. Both numbers came from a pose the aligner itself had chosen, and the aligner's whole job is to drive those two readings equal, so neither one showed the car was physically parallel. The readings had also come through the parser bug above. Two numbers agreeing isn't independent evidence when they share a cause. It stays at 0.0 until someone sets the car parallel by hand against the chassis edge and runs `pico_tool.py --calib`.

**Integral term enabled, then disabled.** A run log showed a steady one-way drift of about 8 cm across the run rather than a weave, and integral action is the textbook fix for a constant disturbance. But the drift was measured from those same corrupted readings, and an integrator on top of biased-low distances accumulates a one-way error and pins the steering hard over. That matched the reported "it just turned straight left." Both `PARKING_PID_KI` and `PARKING_PID_KD` are back at 0.0.

**Restart-on-run, on in V38, off in V39, on in V41, replaced in V45b.** Whether to send a soft-reboot handshake at startup flipped back and forth for weeks, because the answer depended on which REPL state the Pico happened to be in. V45b resolved it by making the decision at runtime: listen first, escape only if silent.

---

## Dead ends

**Sense HAT gyro.** Roughly a month of work across V13–V33. It produced angle values that drifted while stationary and turns that didn't match reality. We kept adjusting the target instead of suspecting the sensor for too long.

**WT901 over UART.** An early version had the WT901 on GPIO1 using UART0 at 100 Hz. The current firmware uses I²C0 on GP4/GP5 instead.

**Ultrasonic instead of ToF.** After enough trouble with black and magenta walls we looked at ultrasonic rangefinders, since they don't care about ambient light or surface color. We wanted something easy to integrate with the Pico and cost-effective. We never migrated. The VL53L0X problems turned out to be manageable in software once we stopped substituting constants for missing readings.

**A wall-follow formula using an explicit angle estimate.** We sketched `(A + B) / 2 * 0.5 + Angle * Ka` with `Ka = abs(B - A) / 10`. The current weighted two-term PID does the same job with terms that can be tuned separately.

**Reverse realignment before the finish turn.** Added in V19 and V20, removed in V21. Forward-only correction was simpler and no worse.

---

## Numbers we measured

These came off the track or out of run logs, not from a formula.

| Measurement | Value |
|---|---|
| ToF read time | ~42 ms per sensor |
| Real parking control rate | ~6.8 frames per second, three blocking reads per Pico loop |
| Best ToF timing set | TOF 42 ms, TIMING 33000 µs, INTER 40 ms |
| Stop lockout at 90% speed | 34.0 s |
| Stop lockout at 100% speed | 32.0 s (inverse scaling predicts 30.6 s, so it isn't proportional) |
| Wall-follow steering, replayed | mean 2.2°, peak 9.4°, 64% of frames under 2°, against a 30° clamp |
| One finish-turn test | end yaw 414.2°, target−90 = 270.0°, calculated 144.2°, physical ~142.5° |
| Failed gyro turns | stopped at 79.4° once, overshot to 95–99° others |
| Parking bay size | about 1.5× car length, roughly 24.5 cm |
| Pico startup time | ~1.7 s for three rangefinders |

---

## Training setup and history

Training runs on a separate computer, not the Pi.

| Item | Value |
|---|---|
| Machine | MSI Katana A15, Windows 11, WSL Ubuntu 22.04 |
| GPU | NVIDIA RTX 4060 Laptop, 8 GB |
| Framework | TensorFlow 2.16.1 GPU, cuDNN 8.9 |
| DonkeyCar | 5.1.0, later 5.2.dev3 |
| Model type | `linear` |
| Batch size | 512 (from 128) |
| Max epochs | 200 (from 100) |
| Early-stop patience | 15 (from 5) |
| Minimum improvement | 0.0002 (from 0.0005) |
| Split | 80% train / 20% validation |

An earlier desktop with a Ryzen 5 7600X3D and a GTX 1080 was used before the laptop.

TensorFlow prints a Grappler layout message during training (`Size of values 0 does not match size of permutation 4`). We checked it and it's an optimizer message, not a failed run. GPU and cuDNN initialization continue normally afterward and the model trains.

The camera is an OV5647 on the `vc4` pipeline through libcamera.

<!-- Fill in before submission: actual record counts per model, number of recording sessions, and the exact train.py command used. The original plan was 160,000 records as 40 tubs of 4,000. -->

---

## Open issues

1. **Lap detection is time-dominated.** `GYRO_TARGET_DEG = 360.0` is roughly one lap, so the 32 s lockout is what actually ends the run. Raising the target to a measured three-lap figure near 1080° is the most useful change left.
2. **The motor and compute rails are shared.** A stall spike reaches the rail the Pi runs on. Two Pis have already died from something we couldn't fully diagnose.
3. **`PARKING_AB_OFFSET_MM` is uncalibrated** and still at 0.0.
4. **The `PARKING_*_MM` constants hold centimeter values**, because the Pico divides by ten and the Pi never converts back. The math is consistent so nothing misbehaves, but the names are wrong.
5. **The finish turn uses an absolute yaw difference**, so overshoot and undershoot look the same and the steering sign is fixed by drive direction.
6. **The mission sequence lives in booleans**, not an enum, so the state isn't inspectable in one place.
7. **Stage 5 bay entry is fully open-loop** and shares one tuned sequence for both directions.

---

## What we'd do differently

**Suspect the sensor sooner.** We spent most of July adjusting a lap target to fit a gyro that was drifting while stationary. Four target values came and went before we replaced the sensor. A stationary-drift test takes two minutes and would have found it immediately.

**Measure the control rate before tuning the controller.** We designed the parking loops assuming 20 Hz and tuned gains for weeks before measuring the real rate at 6.8 Hz. The gains weren't the problem; the sample rate was.

**Check that a constant is actually being written.** The profile-velocity bug cost us a lot of tuning time on a servo that physically couldn't follow the commands. A constant that's defined but never used looks exactly like a constant that's set wrong.

**Write down what a change is supposed to prove.** Both of our reverted changes came from evidence that looked convincing until we asked where the numbers came from. Recording the measurement conditions alongside the value would have caught the circular measurement the first time.
