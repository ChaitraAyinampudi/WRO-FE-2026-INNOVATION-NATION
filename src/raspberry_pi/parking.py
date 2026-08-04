#!/usr/bin/env python3
import re
import sys
import threading
import time

import serial
from dynamixel_sdk import COMM_SUCCESS, PacketHandler, PortHandler

# Hardware settings
DXL_PORT = "/dev/ttyUSB0"
DXL_BAUD = 57600
PROTOCOL_VERSION = 2.0

ID_THROTTLE = 1
ID_STEERING = 2

DXL_CENTER_TICKS = 3126
ANGLE_OFFSET_DEG = 0.7

STEERING_DIRECTION = 1

MAX_THROTTLE_RAW = 120

PROFILE_ACCELERATION = 200
STEERING_PROFILE_VELOCITY = 180

PICO_PORT = "/dev/ttyACM0"
PICO_BAUD = 115200

PICO_VALUES_ARE_MM = True

PRINT_PICO_RAW_LINE = False

PICO_REQUIRED_SENSORS = {"A", "B", "C"}

TOF_PRINT_INTERVAL = 0.15

# Parking settings
APPROACH_SPEED_PERCENT = 80
FOLLOW_SPEED_PERCENT = 80
ENTRY_SPEED_PERCENT = 55

FULL_LEFT_STEER_DEG = -50.0
FULL_RIGHT_STEER_DEG = 50.0
PARKING_TURN_SPEED_PERCENT = 55
PARKING_TURN_MOTOR_DEGREES = 720.0

STEERING_SETTLE_SECONDS = 0.45
RUN_FOR_DEGREES_TIMEOUT = 15.0

SECOND_TURN_AB_TIMEOUT = 15.0

AB_BALANCE_DIFF_LEEWAY_MM = 0.1
AB_BALANCE_TIMEOUT = 15.0
AB_BALANCE_INVALID_TIMEOUT = 15.0

AB_ALIGNMENT_FORWARD_THRESHOLD_MM = 5.0

AB_BALANCE_STEER_DEG = 15.0
AB_BALANCE_SPEED_PERCENT = 35

AB_BALANCE_FORWARD_SPEED_PERCENT = 35

FRONT_STOP_MM = 5.0
WALL_TARGET_MM = 30.0
PARK_TRIGGER_MM = 15.0

C_DISTANCE_LEEWAY_MM = 0.7

BALANCE_LEEWAY_MM = 0.5

BALANCE_CONFIRM_READINGS = 2

WALL_FIND_LEEWAY_MM = 5.0

DISTANCE_CONFIRM_READINGS = 1

WALL_KP = 0.9
MAX_WALL_STEER_DEG = 30.0

WALL_STEERING_SIGN = 1

# PID settings
def K():
    return 2.0

def Td():
    return WALL_TARGET_MM

PID_KI = 0.0
PID_KD = 0.0
PID_INTEGRAL_LIMIT = 50.0
PID_STEERING_DEADBAND_DEG = 0.2

PID_STEERING_DIRECTION = WALL_STEERING_SIGN

ENABLE_PID_AUTO_BREAK = False
PID_BREAK_ERROR_TOLERANCE = 1.0
PID_BREAK_REQUIRED_COUNT = 1

VALID_STATUS_CODES = {0, 11}
USE_SENSOR_STATUS = True

APPROACH_TIMEOUT = 12.0
FOLLOW_TIMEOUT = 30.0
SENSOR_INVALID_TIMEOUT = 0.8

START_DELAY_SECONDS = 3

# DYNAMIXEL control table
ADDR_OPERATING_MODE = 11
ADDR_TORQUE_ENABLE = 64
ADDR_GOAL_VELOCITY = 104
ADDR_PROFILE_ACCELERATION = 108
ADDR_PROFILE_VELOCITY = 112
ADDR_GOAL_POSITION = 116
ADDR_PRESENT_POSITION = 132

MODE_VELOCITY = 1
MODE_POSITION = 3

TORQUE_DISABLE = 0
TORQUE_ENABLE = 1

TICKS_PER_DEGREE = 4096.0 / 360.0

ACTIVE_CAR = None

# Utility functions
def clamp(value, minimum, maximum):
    return max(minimum, min(maximum, value))

def valid_tof(value):
    return value is not None and 5.0 <= value <= 2000.0

def pico_value_to_mm(value):
    if value is None:
        return None

    return float(value)

# PID controller
class PIDController:
    def __init__(self, kp, ki, kd, integral_limit):
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.integral_limit = abs(integral_limit)

        self.integral = 0.0
        self.previous_error = None
        self.previous_time = None

    def reset(self):
        self.integral = 0.0
        self.previous_error = None
        self.previous_time = None

    def update(self, error):
        now = time.monotonic()

        if self.previous_time is None:
            dt = 0.0
        else:
            dt = now - self.previous_time

        if dt <= 0.0:
            dt = 0.001

        self.integral += error * dt

        if self.integral > self.integral_limit:
            self.integral = self.integral_limit
        elif self.integral < -self.integral_limit:
            self.integral = -self.integral_limit

        if self.previous_error is None:
            derivative = 0.0
        else:
            derivative = (error - self.previous_error) / dt

        p_term = self.kp * error
        i_term = self.ki * self.integral
        d_term = self.kd * derivative

        output = p_term + i_term + d_term

        self.previous_error = error
        self.previous_time = now

        return output, p_term, i_term, d_term, dt

def valid_sensor_status(status_value):
    if not USE_SENSOR_STATUS:
        return True

    if status_value is None:
        return True

    return status_value in VALID_STATUS_CODES

def compute_pid_equation_value(a_mm, b_mm, td_mm):
    return (
        a_mm
        - b_mm
        + ((a_mm + b_mm) / 2.0 - td_mm)
    )

def stop_drive_immediately(car):
    for _ in range(3):
        try:
            car.stop()
        except Exception:
            pass

        try:
            car.set_throttle(0)
        except Exception:
            pass

        time.sleep(0.03)

# DYNAMIXEL motor control
class DynamixelCar:
    def __init__(self):
        self.port = PortHandler(DXL_PORT)
        self.packet = PacketHandler(PROTOCOL_VERSION)
        self.opened = False

    def _check(self, result, error, action):
        if result != COMM_SUCCESS:
            raise RuntimeError(
                f"{action}: {self.packet.getTxRxResult(result)}"
            )
        if error != 0:
            raise RuntimeError(
                f"{action}: {self.packet.getRxPacketError(error)}"
            )

    def _write1(self, motor_id, address, value, action):
        result, error = self.packet.write1ByteTxRx(
            self.port, motor_id, address, value
        )
        self._check(result, error, action)

    def _write4(self, motor_id, address, value, action):
        result, error = self.packet.write4ByteTxRx(
            self.port, motor_id, address, int(value) & 0xFFFFFFFF
        )
        self._check(result, error, action)

    def _read4(self, motor_id, address, action):
        value, result, error = self.packet.read4ByteTxRx(
            self.port, motor_id, address
        )
        self._check(result, error, action)
        return int(value) & 0xFFFFFFFF

    @staticmethod
    def _position_step(previous_raw, current_raw):
        step = (current_raw - previous_raw) & 0xFFFFFFFF

        if step & 0x80000000:
            step -= 0x100000000

        return step

    def connect(self):
        if not self.port.openPort():
            raise RuntimeError(f"Could not open DYNAMIXEL port {DXL_PORT}")

        self.opened = True

        if not self.port.setBaudRate(DXL_BAUD):
            raise RuntimeError(f"Could not set DYNAMIXEL baud to {DXL_BAUD}")

        self._write1(
            ID_THROTTLE,
            ADDR_TORQUE_ENABLE,
            TORQUE_ENABLE,
            "Enable throttle torque"
        )

        self._write1(
            ID_STEERING,
            ADDR_TORQUE_ENABLE,
            TORQUE_ENABLE,
            "Enable steering torque"
        )

        self.set_throttle(0)
        time.sleep(0.2)

    def speed_percent_to_raw(self, speed_percent):
        speed_percent = clamp(float(speed_percent), -100.0, 100.0)
        return round((speed_percent / 100.0) * MAX_THROTTLE_RAW)

    def set_throttle(self, speed_percent):
        raw_velocity = self.speed_percent_to_raw(speed_percent)
        self._write4(
            ID_THROTTLE,
            ADDR_GOAL_VELOCITY,
            raw_velocity,
            "Set throttle velocity"
        )

    def steering_angle_to_ticks(self, angle_deg):
        angle_deg = clamp(float(angle_deg), -50.0, 50.0)
        corrected_angle = (
            angle_deg * STEERING_DIRECTION
            + ANGLE_OFFSET_DEG
        )
        ticks = round(
            DXL_CENTER_TICKS + corrected_angle * TICKS_PER_DEGREE
        )
        return int(clamp(ticks, 0, 4095))

    def set_steering(self, angle_deg):
        goal_ticks = self.steering_angle_to_ticks(angle_deg)
        self._write4(
            ID_STEERING,
            ADDR_GOAL_POSITION,
            goal_ticks,
            "Set steering position"
        )

    def set_motion(self, speed_percent, steering_angle):
        self.set_steering(steering_angle)
        self.set_throttle(speed_percent)

    def stop(self):
        if self.opened:
            self.set_throttle(0)

    def close(self):
        if not self.opened:
            return

        try:
            try:
                self.stop()
            except Exception as error:
                print(f"Warning: could not stop throttle: {error}")

            try:
                self.set_steering(0.0)
                time.sleep(0.2)
            except Exception as error:
                print(f"Warning: could not center steering: {error}")

        finally:
            self.port.closePort()
            self.opened = False

def _require_active_car():
    if ACTIVE_CAR is None:
        raise RuntimeError(
            "DYNAMIXEL car is not connected. ACTIVE_CAR has not been set."
        )

    return ACTIVE_CAR

def run_position(motor_id, speed, angle):
    car = _require_active_car()

    if motor_id != ID_STEERING:
        raise ValueError(
            f"run_position expects steering motor ID {ID_STEERING}"
        )

    profile_velocity = max(1, abs(int(speed)))

    car._write4(
        motor_id,
        ADDR_PROFILE_VELOCITY,
        profile_velocity,
        "Set steering profile velocity"
    )
    car.set_steering(angle)

def run_for_degrees(motor_id, speed, degrees):
    car = _require_active_car()

    speed = abs(float(speed))
    degrees = float(degrees)

    if motor_id != ID_THROTTLE:
        raise ValueError(
            f"run_for_degrees expects throttle motor ID {ID_THROTTLE}"
        )

    if speed <= 0:
        raise ValueError("run_for_degrees speed must be greater than 0")

    if degrees == 0:
        car.set_throttle(0)
        return

    target_ticks = abs(degrees) * TICKS_PER_DEGREE
    direction = 1 if degrees > 0 else -1
    commanded_speed = direction * speed

    previous_position = car._read4(
        motor_id,
        ADDR_PRESENT_POSITION,
        "Read starting throttle position"
    )

    traveled_ticks = 0.0
    start_time = time.monotonic()
    last_print_time = 0.0

    print(
        f"run_for_degrees("
        f"id={motor_id}, "
        f"speed={speed:.1f}, "
        f"degrees={degrees:.1f})"
    )

    car.set_throttle(commanded_speed)

    try:
        while traveled_ticks < target_ticks:
            if time.monotonic() - start_time >= RUN_FOR_DEGREES_TIMEOUT:
                raise RuntimeError(
                    "run_for_degrees timed out at "
                    f"{traveled_ticks / TICKS_PER_DEGREE:.1f}/"
                    f"{abs(degrees):.1f} motor degrees"
                )

            current_position = car._read4(
                motor_id,
                ADDR_PRESENT_POSITION,
                "Read throttle position"
            )

            step = car._position_step(
                previous_position,
                current_position
            )
            previous_position = current_position
            traveled_ticks += abs(step)

            now = time.monotonic()

            if now - last_print_time >= 0.15:
                print(
                    "Throttle rotation: "
                    f"{traveled_ticks / TICKS_PER_DEGREE:.1f}/"
                    f"{abs(degrees):.1f} degrees"
                )
                last_print_time = now

            time.sleep(0.01)

    finally:
        car.set_throttle(0)

    print("run_for_degrees complete.")

# Pico ToF serial reader
class PicoToF:
    SENSOR_PATTERN = re.compile(
        r"(?<!S)\b([ABCD])\s*=\s*(-?\d+(?:\.\d+)?)",
        re.IGNORECASE
    )

    STATUS_PATTERN = re.compile(
        r"\bS([ABCD])\s*=\s*(-?\d+)",
        re.IGNORECASE
    )

    def __init__(self, port=None):
        self.port_name = port or PICO_PORT
        self.serial = serial.Serial(
            self.port_name,
            PICO_BAUD,
            timeout=0.05
        )

        self.last_raw_line = ""
        self.last_values = None
        self.last_update_time = 0.0
        self.frame_number = 0

        self._lock = threading.Lock()
        self._stop_event = threading.Event()

        time.sleep(0.25)
        self.serial.reset_input_buffer()

        self._reader_thread = threading.Thread(
            target=self._reader_loop,
            name="PicoToFReader",
            daemon=True
        )
        self._reader_thread.start()

    def _parse_line(self, line):
        matches = self.SENSOR_PATTERN.findall(line)

        if not matches:
            return None

        counts = {}
        values = {}
        statuses = {}

        for sensor_name, value_text in matches:
            sensor_name = sensor_name.upper()
            counts[sensor_name] = counts.get(sensor_name, 0) + 1

            try:
                values[sensor_name] = float(value_text)
            except ValueError:
                return None

        for sensor_name, status_text in self.STATUS_PATTERN.findall(line):
            sensor_name = sensor_name.upper()

            try:
                statuses["S" + sensor_name] = int(status_text)
            except ValueError:
                return None

        if not PICO_REQUIRED_SENSORS.issubset(values):
            return None

        if any(counts.get(name, 0) != 1 for name in PICO_REQUIRED_SENSORS):
            return None

        frame = {
            sensor_name: values[sensor_name]
            for sensor_name in PICO_REQUIRED_SENSORS
        }

        for status_name in ("SA", "SB", "SC", "SD"):
            if status_name in statuses:
                frame[status_name] = statuses[status_name]

        return frame

    def _reader_loop(self):
        while not self._stop_event.is_set():
            try:
                raw = self.serial.readline()
            except serial.SerialException:
                return

            if not raw:
                continue

            line = raw.decode("utf-8", errors="ignore").strip()
            values = self._parse_line(line)

            if values is None:
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
                        "age": time.monotonic() - self.last_update_time
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
                    and all(
                        sensor in self.last_values
                        for sensor in required_sensors
                    )
                ):
                    values = dict(self.last_values)

                    return {
                        "values": values,
                        "line": self.last_raw_line,
                        "frame": self.frame_number,
                        "age": time.monotonic() - self.last_update_time
                    }

            time.sleep(0.002)

        return None

    def close(self):
        self._stop_event.set()

        if hasattr(self, "_reader_thread"):
            self._reader_thread.join(timeout=0.3)

        if self.serial.is_open:
            self.serial.close()

# Parking stages
def wait_until_front_wall(car, tof):
    c_stop_threshold = FRONT_STOP_MM + C_DISTANCE_LEEWAY_MM

    print(
        f"\nSTAGE 2: Driving straight toward the front wall"
    )
    print(
        f"C target={FRONT_STOP_MM:.1f} mm | "
        f"C leeway={C_DISTANCE_LEEWAY_MM:.1f} mm | "
        f"stop threshold={c_stop_threshold:.1f} mm"
    )

    start_time = time.monotonic()
    invalid_start = None
    confirmed = 0
    last_print_time = 0.0

    car.set_motion(APPROACH_SPEED_PERCENT, 0.0)

    while time.monotonic() - start_time < APPROACH_TIMEOUT:
        reading = tof.read_sensor("C")
        distance = None if reading is None else reading["mm"]

        if not valid_tof(distance):
            confirmed = 0

            if invalid_start is None:
                invalid_start = time.monotonic()

            if time.monotonic() - invalid_start >= SENSOR_INVALID_TIMEOUT:
                car.stop()
                raise RuntimeError("Sensor C was invalid for too long")

            continue

        invalid_start = None

        if distance <= c_stop_threshold:
            confirmed += 1
        else:
            confirmed = 0

        now = time.monotonic()

        if now - last_print_time >= TOF_PRINT_INTERVAL:
            display = (
                f"frame={reading['frame']} | "
                f"C={distance:6.1f} mm | "
                f"age={reading['age'] * 1000:5.1f} ms | "
                f"stop {confirmed}/{DISTANCE_CONFIRM_READINGS}"
            )

            if PRINT_PICO_RAW_LINE:
                display += f" | Pico: {reading['line']}"

            print(display)
            last_print_time = now

        if confirmed >= DISTANCE_CONFIRM_READINGS:
            car.stop()
            print(
                f"Front-wall position reached at C={distance:.1f} mm."
            )
            return

        car.set_motion(APPROACH_SPEED_PERCENT, 0.0)

    car.stop()
    raise RuntimeError("Front-wall approach timed out")

def turn_left_for_degrees(car, motor_degrees):
    print(
        f"\nSTAGE 1: Full-left forward turn for "
        f"{motor_degrees:.1f} motor degrees"
    )

    car.stop()

    run_position(
        ID_STEERING,
        80,
        FULL_LEFT_STEER_DEG
    )
    time.sleep(STEERING_SETTLE_SECONDS)

    try:
        run_for_degrees(
            ID_THROTTLE,
            PARKING_TURN_SPEED_PERCENT,
            abs(motor_degrees)
        )
    finally:
        car.stop()
        run_position(
            ID_STEERING,
            80,
            0
        )
        time.sleep(0.3)

    print("Forward-left turn complete.")

def turn_right_backwards_for_degrees(car, motor_degrees):
    print(
        f"\nSTAGE 3: Full-right reverse turn for "
        f"{motor_degrees:.1f} motor degrees"
    )

    car.stop()

    run_position(
        ID_STEERING,
        80,
        FULL_RIGHT_STEER_DEG
    )
    time.sleep(STEERING_SETTLE_SECONDS)

    try:
        run_for_degrees(
            ID_THROTTLE,
            PARKING_TURN_SPEED_PERCENT,
            -abs(motor_degrees)
        )
    finally:
        car.stop()
        run_position(
            ID_STEERING,
            80,
            0
        )
        time.sleep(0.2)

    print("Fixed reverse-right turn complete.")

def balance_ab_backwards(
    car,
    tof,
    difference_leeway_mm=AB_BALANCE_DIFF_LEEWAY_MM
):
    print(
        f"\nSTAGE 3B: Reverse A/B balance until "
        f"|A - B| <= {difference_leeway_mm:.1f} mm"
    )
    print(
        f"A/B balance invalid timeout: "
        f"{AB_BALANCE_INVALID_TIMEOUT:.1f} seconds"
    )
    print(
        "If alignment times out, the program continues into PID ToF_run."
    )

    car.stop()

    start_time = time.monotonic()
    invalid_start = None
    confirmed = 0
    last_print_time = 0.0

    try:
        while time.monotonic() - start_time < AB_BALANCE_TIMEOUT:
            frame_reading = tof.read_frame(("A", "B"))
            frame = None if frame_reading is None else frame_reading["values"]

            if frame is None:
                confirmed = 0

                if invalid_start is None:
                    invalid_start = time.monotonic()

                car.set_motion(-abs(AB_BALANCE_SPEED_PERCENT), 0.0)

                if time.monotonic() - invalid_start >= AB_BALANCE_INVALID_TIMEOUT:
                    stop_drive_immediately(car)
                    run_position(ID_STEERING, 80, 0)
                    print(
                        "A/B balance did not receive valid A/B frames. "
                        "Skipping alignment and starting PID ToF_run."
                    )
                    return

                continue

            a_distance = frame["A"]
            b_distance = frame["B"]

            a_valid = (
                valid_tof(a_distance)
                and valid_sensor_status(frame.get("SA"))
            )
            b_valid = (
                valid_tof(b_distance)
                and valid_sensor_status(frame.get("SB"))
            )

            if not a_valid or not b_valid:
                confirmed = 0

                if invalid_start is None:
                    invalid_start = time.monotonic()

                car.set_motion(-abs(AB_BALANCE_SPEED_PERCENT), 0.0)

                if time.monotonic() - invalid_start >= AB_BALANCE_INVALID_TIMEOUT:
                    stop_drive_immediately(car)
                    run_position(ID_STEERING, 80, 0)
                    print(
                        "A/B balance sensors were invalid for too long. "
                        "Skipping alignment and starting PID ToF_run."
                    )
                    return

                continue

            invalid_start = None
            difference = abs(a_distance - b_distance)

            if difference <= difference_leeway_mm:
                confirmed += 1
                steering = 0.0
                state = "balanced"
            else:
                confirmed = 0

                if a_distance > b_distance:
                    steering = -abs(AB_BALANCE_STEER_DEG)
                    state = "A > B, backwards left"
                else:
                    steering = abs(AB_BALANCE_STEER_DEG)
                    state = "B > A, backwards right"

            car.set_motion(-abs(AB_BALANCE_SPEED_PERCENT), steering)

            now = time.monotonic()

            if now - last_print_time >= TOF_PRINT_INTERVAL:
                print(
                    f"frame={frame_reading['frame']} | "
                    f"A={a_distance:6.1f} mm | "
                    f"B={b_distance:6.1f} mm | "
                    f"|A-B|={difference:5.1f} mm | "
                    f"{state} | steer={steering:5.1f} | "
                    f"match {confirmed}/{DISTANCE_CONFIRM_READINGS}"
                )
                last_print_time = now

            if confirmed >= DISTANCE_CONFIRM_READINGS:
                stop_drive_immediately(car)
                run_position(ID_STEERING, 80, 0)
                time.sleep(0.2)

                print(
                    f"A/B reverse balance complete at "
                    f"A={a_distance:.1f} mm, "
                    f"B={b_distance:.1f} mm, "
                    f"|A-B|={difference:.1f} mm."
                )
                return

        stop_drive_immediately(car)
        run_position(ID_STEERING, 80, 0)
        print(
            "A/B reverse balance timed out before A/B matched. "
            "Continuing into PID ToF_run."
        )
        return

    except KeyboardInterrupt:
        print("\nCtrl+C detected during A/B reverse balance.")
        stop_drive_immediately(car)

        try:
            run_position(ID_STEERING, 80, 0)
        except Exception:
            pass

        raise

def balance_ab_forward(
    car,
    tof,
    difference_leeway_mm=AB_BALANCE_DIFF_LEEWAY_MM
):
    print(
        f"\nSTAGE 4B: Forward A/B balance until "
        f"|A - B| <= {difference_leeway_mm:.1f} mm"
    )
    print(
        f"Forward A/B balance invalid timeout: "
        f"{AB_BALANCE_INVALID_TIMEOUT:.1f} seconds"
    )

    car.stop()

    start_time = time.monotonic()
    invalid_start = None
    confirmed = 0
    last_print_time = 0.0

    try:
        while time.monotonic() - start_time < AB_BALANCE_TIMEOUT:
            frame_reading = tof.read_frame(("A", "B"))
            frame = None if frame_reading is None else frame_reading["values"]

            if frame is None:
                confirmed = 0

                if invalid_start is None:
                    invalid_start = time.monotonic()

                car.set_motion(abs(AB_BALANCE_FORWARD_SPEED_PERCENT), 0.0)

                if time.monotonic() - invalid_start >= AB_BALANCE_INVALID_TIMEOUT:
                    stop_drive_immediately(car)
                    run_position(ID_STEERING, 80, 0)
                    print(
                        "Forward A/B balance did not receive valid A/B frames. "
                        "Continuing into PID ToF_run."
                    )
                    return

                continue

            a_distance = frame["A"]
            b_distance = frame["B"]

            a_valid = (
                valid_tof(a_distance)
                and valid_sensor_status(frame.get("SA"))
            )
            b_valid = (
                valid_tof(b_distance)
                and valid_sensor_status(frame.get("SB"))
            )

            if not a_valid or not b_valid:
                confirmed = 0

                if invalid_start is None:
                    invalid_start = time.monotonic()

                car.set_motion(abs(AB_BALANCE_FORWARD_SPEED_PERCENT), 0.0)

                if time.monotonic() - invalid_start >= AB_BALANCE_INVALID_TIMEOUT:
                    stop_drive_immediately(car)
                    run_position(ID_STEERING, 80, 0)
                    print(
                        "Forward A/B balance sensors were invalid for too long. "
                        "Continuing into PID ToF_run."
                    )
                    return

                continue

            invalid_start = None
            difference = abs(a_distance - b_distance)

            if difference <= difference_leeway_mm:
                confirmed += 1
                steering = 0.0
                state = "balanced"
            else:
                confirmed = 0

                if a_distance > b_distance:
                    steering = abs(AB_BALANCE_STEER_DEG)
                    state = "A > B, forward right"
                else:
                    steering = -abs(AB_BALANCE_STEER_DEG)
                    state = "B > A, forward left"

            car.set_motion(abs(AB_BALANCE_FORWARD_SPEED_PERCENT), steering)

            now = time.monotonic()

            if now - last_print_time >= TOF_PRINT_INTERVAL:
                print(
                    f"frame={frame_reading['frame']} | "
                    f"A={a_distance:6.1f} mm | "
                    f"B={b_distance:6.1f} mm | "
                    f"|A-B|={difference:5.1f} mm | "
                    f"{state} | steer={steering:5.1f} | "
                    f"match {confirmed}/{DISTANCE_CONFIRM_READINGS}"
                )
                last_print_time = now

            if confirmed >= DISTANCE_CONFIRM_READINGS:
                stop_drive_immediately(car)
                run_position(ID_STEERING, 80, 0)
                time.sleep(0.2)

                print(
                    f"Forward A/B balance complete at "
                    f"A={a_distance:.1f} mm, "
                    f"B={b_distance:.1f} mm, "
                    f"|A-B|={difference:.1f} mm."
                )
                return

        stop_drive_immediately(car)
        run_position(ID_STEERING, 80, 0)
        print(
            "Forward A/B balance timed out before A/B matched. "
            "Continuing into PID ToF_run."
        )
        return

    except KeyboardInterrupt:
        print("\nCtrl+C detected during forward A/B balance.")
        stop_drive_immediately(car)

        try:
            run_position(ID_STEERING, 80, 0)
        except Exception:
            pass

        raise

def choose_ab_alignment_after_second_turn(car, tof):
    print("\nSTAGE 3B: Checking A/B difference before alignment")
    print(
        f"Forward alignment threshold: "
        f"|A - B| > {AB_ALIGNMENT_FORWARD_THRESHOLD_MM:.1f} mm"
    )

    car.stop()
    run_position(ID_STEERING, 80, 0)
    time.sleep(0.15)

    frame_reading = tof.read_frame(("A", "B"), timeout=1.0)
    frame = None if frame_reading is None else frame_reading["values"]

    if frame is None:
        print(
            "Could not read A/B before alignment. "
            "Skipping alignment and starting PID ToF_run."
        )
        return

    a_distance = frame["A"]
    b_distance = frame["B"]

    a_valid = (
        valid_tof(a_distance)
        and valid_sensor_status(frame.get("SA"))
    )
    b_valid = (
        valid_tof(b_distance)
        and valid_sensor_status(frame.get("SB"))
    )

    if not a_valid or not b_valid:
        print(
            f"A/B pre-check invalid: "
            f"A={a_distance:.1f} valid={a_valid}, "
            f"B={b_distance:.1f} valid={b_valid}. "
            "Skipping alignment and starting PID ToF_run."
        )
        return

    difference = abs(a_distance - b_distance)

    print(
        f"A/B pre-check: A={a_distance:.1f} mm, "
        f"B={b_distance:.1f} mm, "
        f"|A-B|={difference:.1f} mm"
    )

    if difference > AB_ALIGNMENT_FORWARD_THRESHOLD_MM:
        print(
            "Difference is over 5 mm, so the program uses FORWARD alignment. "
            "Forward logic: A > B = right, B > A = left."
        )
        balance_ab_forward(car, tof)
    else:
        print(
            "Difference is 5 mm or less, so the program uses BACKWARD alignment."
        )
        balance_ab_backwards(car, tof)

def ToF_run(
    car,
    tof,
    speed,
    wall_target_mm,
    parking_trigger_mm,
    balance_leeway_mm,
    wall_find_leeway_mm
):
    td_value = Td()
    k_value = K()
    wall_found_threshold = (
        parking_trigger_mm + wall_find_leeway_mm
    )

    pid = PIDController(
        k_value,
        PID_KI,
        PID_KD,
        PID_INTEGRAL_LIMIT
    )

    start_time = time.monotonic()
    invalid_start = None
    wall_confirmed = 0
    break_count = 0
    last_print_time = 0.0

    print("\nSTAGE 4: Pure PID ToF_run using sensors A and B")
    print("Equation: A - B + (((A + B) / 2) - Td())")
    print(f"K()={k_value:.2f} | Td()={td_value:.1f} mm")
    print(f"PID gains: KP={k_value:.2f}, KI={PID_KI:.2f}, KD={PID_KD:.2f}")
    print("Pre-check chooses forward/backward alignment, then pure PID.")
    print(
        f"Parking wall target={parking_trigger_mm:.1f} mm | "
        f"wall leeway={wall_find_leeway_mm:.1f} mm | "
        f"detection threshold=B <= {wall_found_threshold:.1f} mm"
    )
    print("Press Ctrl+C to stop ToF_run immediately.")

    try:
        while time.monotonic() - start_time < FOLLOW_TIMEOUT:
            frame_reading = tof.read_frame(("A", "B"))
            frame = None if frame_reading is None else frame_reading["values"]

            if frame is None:
                pid.reset()
                wall_confirmed = 0
                break_count = 0

                if invalid_start is None:
                    invalid_start = time.monotonic()

                car.set_motion(speed, 0.0)

                if time.monotonic() - invalid_start >= SENSOR_INVALID_TIMEOUT:
                    stop_drive_immediately(car)
                    run_position(ID_STEERING, 80, 0)
                    raise RuntimeError(
                        "PID ToF_run did not receive a valid A/B frame"
                    )

                continue

            original_a = frame["A"]
            original_b = frame["B"]

            a_valid = (
                valid_tof(original_a)
                and valid_sensor_status(frame.get("SA"))
            )
            b_valid = (
                valid_tof(original_b)
                and valid_sensor_status(frame.get("SB"))
            )

            a_value = original_a if a_valid else td_value
            b_value = original_b if b_valid else td_value

            if a_valid or b_valid:
                invalid_start = None
            else:
                if invalid_start is None:
                    invalid_start = time.monotonic()

                if time.monotonic() - invalid_start >= SENSOR_INVALID_TIMEOUT:
                    stop_drive_immediately(car)
                    run_position(ID_STEERING, 80, 0)
                    raise RuntimeError(
                        "PID ToF_run A and B were invalid for too long"
                    )

            equation_value = compute_pid_equation_value(
                a_value,
                b_value,
                td_value
            )

            if (
                ENABLE_PID_AUTO_BREAK
                and abs(equation_value) <= PID_BREAK_ERROR_TOLERANCE
            ):
                break_count += 1

                if break_count >= PID_BREAK_REQUIRED_COUNT:
                    stop_drive_immediately(car)
                    run_position(ID_STEERING, 80, -1.0)
                    print(
                        f"PID break reached at error={equation_value:.2f}. "
                        "Stopped and steering set 1 degree left."
                    )
                    return
            else:
                break_count = 0

            pid_output, p_term, i_term, d_term, dt = pid.update(
                equation_value
            )

            steering = pid_output * PID_STEERING_DIRECTION

            if abs(steering) < PID_STEERING_DEADBAND_DEG:
                steering = 0.0

            steering = clamp(
                steering,
                -MAX_WALL_STEER_DEG,
                MAX_WALL_STEER_DEG
            )

            car.set_motion(speed, steering)

            if a_valid and original_a <= wall_found_threshold:
                wall_confirmed += 1
            else:
                wall_confirmed = 0

            if wall_confirmed >= DISTANCE_CONFIRM_READINGS:
                stop_drive_immediately(car)
                run_position(ID_STEERING, 80, -1.0)

                print(
                    f"Parking wall found with A={original_a:.1f} mm. "
                    "Stopped abruptly and steering set 1 degree left."
                )
                return

            now = time.monotonic()

            if now - last_print_time >= TOF_PRINT_INTERVAL:
                if steering > 0:
                    direction = "RIGHT"
                elif steering < 0:
                    direction = "LEFT"
                else:
                    direction = "STRAIGHT"

                print(
                    f"frame={frame_reading['frame']} | "
                    f"A={a_value:6.1f} mm | "
                    f"B={b_value:6.1f} mm | "
                    f"error={equation_value:7.2f} | "
                    f"P={p_term:7.2f} I={i_term:7.2f} "
                    f"D={d_term:7.2f} | "
                    f"steer={steering:6.1f} | "
                    f"{direction} | "
                    f"wall {wall_confirmed}/"
                    f"{DISTANCE_CONFIRM_READINGS}"
                )

                if not a_valid or not b_valid:
                    print(
                        "SUBSTITUTED | "
                        f"A={original_a:.1f} SA={frame.get('SA')} -> "
                        f"{a_value:.1f} | "
                        f"B={original_b:.1f} SB={frame.get('SB')} -> "
                        f"{b_value:.1f}"
                    )

                last_print_time = now

        stop_drive_immediately(car)
        run_position(ID_STEERING, 80, 0)
        raise RuntimeError("PID ToF_run timed out before finding the wall")

    except KeyboardInterrupt:
        print("\nCtrl+C detected inside PID ToF_run.")
        stop_drive_immediately(car)

        try:
            run_position(ID_STEERING, 80, 0)
        except Exception:
            pass

        print("PID ToF_run stopped throttle and is exiting.")
        raise

def parking_entry_cw(car):
    print("\nSTAGE 5: Entering the parking lot")

    car.stop()

    run_position(ID_STEERING, 50, 0)
    run_for_degrees(ID_THROTTLE, 80, 1200)
    time.sleep(0.15)
    run_position(ID_STEERING, 80, 45)
    run_for_degrees(ID_THROTTLE, 80, -760)
    time.sleep(0.15)
    run_position(ID_STEERING, 80, -6.25)
    run_for_degrees(ID_THROTTLE, 80, 200)
    time.sleep(0.15)
    run_position(ID_STEERING, 80, -50)
    run_for_degrees(ID_THROTTLE, 80, -300)
    time.sleep(0.15)
    run_position(ID_STEERING, 80, 50)
    run_for_degrees(ID_THROTTLE, 80, 180)
    time.sleep(0.15)
    run_position(ID_STEERING, 80, -50)
    run_for_degrees(ID_THROTTLE, 80, -280)
    time.sleep(0.15)
    run_position(ID_STEERING, 80, 50)
    run_for_degrees(ID_THROTTLE, 80, 150)
    time.sleep(0.15)
    run_position(ID_STEERING, 80, -50)
    run_for_degrees(ID_THROTTLE, 80, -180)

    print("Parking-entry movement complete.")

def main():
    global ACTIVE_CAR

    car = None
    tof = None

    try:
        print("Parking standalone CW test")
        print("----------------------------------------")
        print(f"U2D2 port: {DXL_PORT}")
        print(f"Pico port: {PICO_PORT}")
        print(f"Steering center: {DXL_CENTER_TICKS} ticks")
        print(f"Steering offset: {ANGLE_OFFSET_DEG} degrees")
        print("Pico distance units: millimeters, used directly")
        print("----------------------------------------")

        car = DynamixelCar()
        car.connect()
        ACTIVE_CAR = car
        print("DYNAMIXEL motors connected.")

        tof = PicoToF(PICO_PORT)
        print(f"Pico connected on {tof.port_name}.")

        print(
            f"\nThe robot will start in {START_DELAY_SECONDS} seconds. "
            "Press Ctrl+C to cancel."
        )

        for seconds_left in range(START_DELAY_SECONDS, 0, -1):
            print(seconds_left)
            time.sleep(1)

        turn_left_for_degrees(
            car,
            PARKING_TURN_MOTOR_DEGREES
        )

        wait_until_front_wall(car, tof)

        turn_right_backwards_for_degrees(
            car,
            PARKING_TURN_MOTOR_DEGREES
        )

        choose_ab_alignment_after_second_turn(
            car,
            tof
        )

        ToF_run(
            car,
            tof,
            FOLLOW_SPEED_PERCENT,
            WALL_TARGET_MM,
            PARK_TRIGGER_MM,
            BALANCE_LEEWAY_MM,
            WALL_FIND_LEEWAY_MM
        )

        parking_entry_cw(car)

        print("\nParking sequence finished successfully.")

    except KeyboardInterrupt:
        print("\nTest stopped by user.")

    except Exception as error:
        print(f"\nPARKING TEST FAILED: {error}", file=sys.stderr)

    finally:
        if car is not None:
            try:
                car.close()
            except Exception as error:
                print(f"Motor shutdown warning: {error}", file=sys.stderr)

        if tof is not None:
            try:
                tof.close()
            except Exception as error:
                print(f"Pico shutdown warning: {error}", file=sys.stderr)

if __name__ == "__main__":
    main()
