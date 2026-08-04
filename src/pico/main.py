"""
Raspberry Pi Pico sensor program.

WT901 gyro:
- I2C0
- SDA: GP4
- SCL: GP5
- Address: 0x50

VL53L0X ToF sensors:
- I2C1
- SDA: GP2
- SCL: GP3
- A XSHUT: GP18, address 0x2A
- B XSHUT: GP19, address 0x2B
- C XSHUT: GP20, address 0x2C
- D XSHUT: GP21, kept off

USB output:
Angle=X,A=X,AS=X,B=X,BS=X,C=X,CS=X
"""

import time
import struct
import _thread
import sys
import select
from machine import Pin, I2C, reset
from vl53l0x import VL53L0X

TOF_I2C_ID = 1
TOF_SDA_PIN = 2
TOF_SCL_PIN = 3

TOF_I2C_FREQ = 400_000

TOF_DEFAULT_ADDRESS = 0x29

TOF_CONFIGS = (
    ("A", 18, 0x2A),
    ("B", 19, 0x2B),
    ("C", 20, 0x2C),
)

TOF_ALL_XSHUT_PINS = (18, 19, 20, 21)

TOF_TIMEOUT_MS = 100

TOF_WAKE_DELAY_MS = 60
TOF_AFTER_ADDRESS_MS = 20

TOF_MIN_VALID_MM = 20
TOF_MAX_VALID_MM = 1500

WT_I2C_ID = 0
WT_SDA_PIN = 4
WT_SCL_PIN = 5
WT_I2C_FREQ = 100_000
WT_ADDRESS = 0x50

WT_REG_GX = 0x37
WT_REG_ROLL = 0x3D

WT_CALIBRATION_SECONDS = 3.0
WT_SAMPLE_PERIOD_MS = 10
WT_DT_MAX_SECONDS = 0.05
WT_EMA_ALPHA = 0.18
WT_GYRO_SCALE_POS = 1.00621
WT_GYRO_SCALE_NEG = 1.00007
WT_STILL_THRESHOLD_DPS = 1.0
WT_STILL_HOLD_SECONDS = 0.6
WT_BIAS_ADAPT_ALPHA = 0.015

OUTPUT_HZ = 20
OUTPUT_EVERY_TOF_FRAME = False

PRINT_STARTUP_DEBUG = True
ECHO_TO_CONSOLE = False

try:
    import usb_cdc
    USB_DATA = getattr(usb_cdc, "data", None)
except Exception:
    USB_DATA = None

print_lock = _thread.allocate_lock()

def send_line(text):

    with print_lock:
        print(text)

def debug(text):
    if PRINT_STARTUP_DEBUG:
        send_line(text)

def make_status_led():
    try:
        return Pin("LED", Pin.OUT)
    except Exception:
        return Pin(25, Pin.OUT)

led = make_status_led()
MAIN_LED_BLINK_MS = 250
_last_main_led_ms = time.ticks_ms()
control = {"gyro_reset": False}

def blink_main_led():
    global _last_main_led_ms

    now = time.ticks_ms()

    if time.ticks_diff(now, _last_main_led_ms) >= MAIN_LED_BLINK_MS:
        try:
            led.value(0 if led.value() else 1)
        except Exception:
            pass

        _last_main_led_ms = now

_command_buffer = ""
_COMMAND_BUFFER_MAX = 64

def handle_usb_command(text):
    if not text:
        return

    led.value(1)
    time.sleep_ms(40)
    led.value(0)

    if text in ("restart main", "restart_main", "main:restart", "restart", "reboot"):
        send_line("[HOST] restart_main received; rebooting Pico")
        time.sleep_ms(100)
        reset()

    if text in ("reset gyro", "reset_gyro", "gyro:reset"):
        control["gyro_reset"] = True
        send_line("[HOST] reset gyro received")

def poll_usb_command():
    global _command_buffer

    try:
        while select.select([sys.stdin], [], [], 0)[0]:
            character = sys.stdin.read(1)

            if not character:
                return

            if character in ("\r", "\n"):
                text = _command_buffer.strip().lower()
                _command_buffer = ""
                handle_usb_command(text)
            else:
                _command_buffer += character

                if len(_command_buffer) > _COMMAND_BUFFER_MAX:
                    _command_buffer = _command_buffer[-_COMMAND_BUFFER_MAX:]
    except Exception:
        pass

latest_gyro = {
    "angle_deg": 0.0,
    "wt_ok": False,
    "gz_dps": 0.0,
    "roll_deg": 0.0,
    "pitch_deg": 0.0,
    "sensor_yaw_deg": 0.0,
    "error_count": 0,
}

gyro_lock = _thread.allocate_lock()

def wt_read_signed_words(i2c, start_register, count):
    raw = i2c.readfrom_mem(WT_ADDRESS, start_register, count * 2)
    return struct.unpack("<" + ("h" * count), raw)

def wt_read_gyro_and_angles(i2c):
    gx_raw, gy_raw, gz_raw = wt_read_signed_words(i2c, WT_REG_GX, 3)
    roll_raw, pitch_raw, yaw_raw = wt_read_signed_words(i2c, WT_REG_ROLL, 3)

    gx = gx_raw / 32768.0 * 2000.0
    gy = gy_raw / 32768.0 * 2000.0
    gz = gz_raw / 32768.0 * 2000.0

    roll = roll_raw / 32768.0 * 180.0
    pitch = pitch_raw / 32768.0 * 180.0
    sensor_yaw = yaw_raw / 32768.0 * 180.0

    return gx, gy, gz, roll, pitch, sensor_yaw

def wt901_i2c_thread():
    try:
        i2c = I2C(
            WT_I2C_ID,
            sda=Pin(WT_SDA_PIN),
            scl=Pin(WT_SCL_PIN),
            freq=WT_I2C_FREQ,
        )
    except Exception as exc:
        send_line("[WT901 ERROR] I2C0 init failed: %r" % exc)
        return

    time.sleep_ms(300)

    try:
        found = i2c.scan()
    except Exception as exc:
        send_line("[WT901 ERROR] scan failed: %r" % exc)
        return

    if WT_ADDRESS not in found:
        send_line(
            "[WT901 ERROR] 0x50 not found on I2C0 GP4/GP5; scan=%s"
            % str(["0x%02X" % address for address in found])
        )
        return

    send_line("[WT901] found at 0x50 on I2C0 GP4/GP5")
    send_line("[WT901] keep robot still: 3-second gyro calibration")

    gz_bias = None
    yaw_estimate = 0.0
    last_us = None
    previous_gz = None
    filtered_gz = 0.0
    still_start_us = None
    calibration_sum = 0.0
    calibration_count = 0
    calibration_end_us = time.ticks_add(
        time.ticks_us(),
        int(WT_CALIBRATION_SECONDS * 1_000_000),
    )
    error_count = 0

    while True:
        blink_main_led()
        loop_start_ms = time.ticks_ms()

        if control.get("gyro_reset"):
            control["gyro_reset"] = False
            gz_bias = None
            yaw_estimate = 0.0
            last_us = None
            previous_gz = None
            filtered_gz = 0.0
            still_start_us = None
            calibration_sum = 0.0
            calibration_count = 0
            calibration_end_us = time.ticks_add(
                time.ticks_us(),
                int(WT_CALIBRATION_SECONDS * 1_000_000),
            )

            with gyro_lock:
                latest_gyro["angle_deg"] = 0.0
                latest_gyro["wt_ok"] = False

            send_line("[WT901] yaw reset; recalibrating for 3 seconds")

        try:
            gx, gy, gz_raw, roll, pitch, sensor_yaw = wt_read_gyro_and_angles(i2c)
            now_us = time.ticks_us()

            if gz_bias is None:
                if time.ticks_diff(now_us, calibration_end_us) < 0:
                    calibration_sum += gz_raw
                    calibration_count += 1
                else:
                    if calibration_count:
                        gz_bias = calibration_sum / calibration_count
                    else:
                        gz_bias = 0.0

                    filtered_gz = gz_raw - gz_bias
                    if filtered_gz >= 0:
                        previous_gz = filtered_gz * WT_GYRO_SCALE_POS
                    else:
                        previous_gz = filtered_gz * WT_GYRO_SCALE_NEG

                    last_us = now_us
                    send_line(
                        "[WT901] calibration complete; bias=%.4f dps"
                        % gz_bias
                    )
            else:
                demeaned_gz = gz_raw - gz_bias
                filtered_gz = (
                    WT_EMA_ALPHA * demeaned_gz
                    + (1.0 - WT_EMA_ALPHA) * filtered_gz
                )

                if filtered_gz >= 0:
                    corrected_gz = filtered_gz * WT_GYRO_SCALE_POS
                else:
                    corrected_gz = filtered_gz * WT_GYRO_SCALE_NEG

                if last_us is None:
                    last_us = now_us

                dt = time.ticks_diff(now_us, last_us) / 1_000_000.0
                last_us = now_us

                if 0 < dt <= WT_DT_MAX_SECONDS:
                    if previous_gz is None:
                        previous_gz = corrected_gz

                    yaw_estimate += 0.5 * (previous_gz + corrected_gz) * dt
                    previous_gz = corrected_gz

                if abs(filtered_gz) < WT_STILL_THRESHOLD_DPS:
                    if still_start_us is None:
                        still_start_us = now_us
                    elif (
                        time.ticks_diff(now_us, still_start_us)
                        > int(WT_STILL_HOLD_SECONDS * 1_000_000)
                    ):
                        gz_bias = (
                            (1.0 - WT_BIAS_ADAPT_ALPHA) * gz_bias
                            + WT_BIAS_ADAPT_ALPHA * gz_raw
                        )
                else:
                    still_start_us = None

            with gyro_lock:
                latest_gyro["angle_deg"] = yaw_estimate
                latest_gyro["gz_dps"] = gz_raw
                latest_gyro["roll_deg"] = roll
                latest_gyro["pitch_deg"] = pitch
                latest_gyro["sensor_yaw_deg"] = sensor_yaw
                latest_gyro["wt_ok"] = gz_bias is not None
                latest_gyro["error_count"] = error_count

        except Exception as exc:
            error_count += 1

            with gyro_lock:
                latest_gyro["error_count"] = error_count

            if error_count <= 5 or error_count % 100 == 0:
                send_line("[WT901 READ ERROR %d] %r" % (error_count, exc))

        elapsed_ms = time.ticks_diff(time.ticks_ms(), loop_start_ms)
        remaining_ms = WT_SAMPLE_PERIOD_MS - elapsed_ms

        if remaining_ms > 0:
            time.sleep_ms(remaining_ms)

def tof_scan_text(i2c):
    try:
        found = i2c.scan()
    except Exception:
        return "scan_error"

    if not found:
        return "none"

    return ",".join("0x%02X" % address for address in found)

def initialize_tof(i2c, name, xshut_pin, new_address):
    debug(
        "[TOF INIT] %s XSHUT=GP%d -> 0x%02X"
        % (name, xshut_pin, new_address)
    )

    pin = Pin(xshut_pin, Pin.OUT, value=0)
    time.sleep_ms(50)
    pin.value(1)
    time.sleep_ms(TOF_WAKE_DELAY_MS)

    if TOF_DEFAULT_ADDRESS not in i2c.scan():
        raise OSError(
            "%s did not appear at 0x29; scan=%s"
            % (name, tof_scan_text(i2c))
        )

    sensor = VL53L0X(
        i2c,
        address=TOF_DEFAULT_ADDRESS,
        timeout_ms=TOF_TIMEOUT_MS,
    )

    sensor.set_address(new_address)
    sensor.address = new_address
    time.sleep_ms(TOF_AFTER_ADDRESS_MS)

    if new_address not in i2c.scan():
        raise OSError(
            "%s address change failed; scan=%s"
            % (name, tof_scan_text(i2c))
        )

    try:
        sensor.read()
    except Exception:
        pass

    return sensor

def read_tof_cm_status(sensor):

    if sensor is None:
        return -1.0, 9

    try:
        millimetres = int(sensor.read())
    except Exception:
        return -1.0, 9

    if TOF_MIN_VALID_MM <= millimetres <= TOF_MAX_VALID_MM:
        return millimetres / 10.0, 0

    return -1.0, 9

def main():

    wt901_started = False

    for attempt in range(2):
        try:
            _thread.start_new_thread(wt901_i2c_thread, ())
            wt901_started = True
            break
        except OSError:
            send_line("[WARN] core1 unavailable on attempt %d" % (attempt + 1))
            time.sleep_ms(500)

    if not wt901_started:
        send_line("[WARN] core1 never started; streaming Angle=0.00 only")

    tof_i2c = I2C(
        TOF_I2C_ID,
        sda=Pin(TOF_SDA_PIN),
        scl=Pin(TOF_SCL_PIN),
        freq=TOF_I2C_FREQ,
    )

    xshut_pins = {}

    for pin_number in TOF_ALL_XSHUT_PINS:
        xshut_pins[pin_number] = Pin(
            pin_number,
            Pin.OUT,
            value=0,
        )

    time.sleep_ms(300)

    sensors = {}
    failed_sensors = []

    for name, xshut_pin, new_address in TOF_CONFIGS:
        try:
            sensors[name] = initialize_tof(
                tof_i2c,
                name,
                xshut_pin,
                new_address,
            )
        except Exception as exc:
            sensors[name] = None
            failed_sensors.append(name)
            send_line("[TOF WARN] %s init failed: %r" % (name, exc))

    if failed_sensors:
        send_line(
            "[TOF WARN] continuing without: %s"
            % ",".join(failed_sensors)
        )

    try:
        xshut_pins[21].value(0)
    except Exception:
        pass

    debug("[TOF] final I2C1 scan: %s" % tof_scan_text(tof_i2c))
    send_line("# FORMAT: Angle=X,A=X,AS=X,B=X,BS=X,C=X,CS=X")
    send_line("# WT901 runs core1 at ~100Hz; ToF A/B/C runs core0")

    if OUTPUT_HZ > 0:
        output_period_ms = int(1000 / OUTPUT_HZ)
    else:
        output_period_ms = 0

    next_output_ms = time.ticks_ms()

    while True:
        blink_main_led()
        poll_usb_command()

        a_cm, a_status = read_tof_cm_status(sensors["A"])
        b_cm, b_status = read_tof_cm_status(sensors["B"])
        c_cm, c_status = read_tof_cm_status(sensors["C"])

        with gyro_lock:
            angle_deg = latest_gyro["angle_deg"]

        should_print = True

        if not OUTPUT_EVERY_TOF_FRAME and output_period_ms > 0:
            now_ms = time.ticks_ms()

            if time.ticks_diff(now_ms, next_output_ms) < 0:
                should_print = False
            else:
                next_output_ms = time.ticks_add(now_ms, output_period_ms)

        if should_print:
            send_line(
                "Angle=%.2f,A=%.1f,AS=%d,B=%.1f,BS=%d,C=%.1f,CS=%d"
                % (
                    angle_deg,
                    a_cm,
                    a_status,
                    b_cm,
                    b_status,
                    c_cm,
                    c_status,
                )
            )

if __name__ == "__main__":
    main()
