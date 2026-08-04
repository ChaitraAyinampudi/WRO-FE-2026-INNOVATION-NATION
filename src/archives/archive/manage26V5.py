import os
import sys
import time
import signal
import re
from pathlib import Path
from typing import Tuple
from sense_hat import SenseHat
from time import sleep

import donkeycar as dk
import numpy as np
import pygame
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
OBSTACLE_MODEL_PATH_CW   = "~/WRO_FE_2026/models/ocwm/ocwm001-008/mypilot.h5"
OBSTACLE_MODEL_PATH_CCW  = "~/WRO_FE_2026/models/occwm/occwm001-010/mypilot.h5"

def select_model_path_with_sensehat() -> str:
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

    if drive_mode == "FCW":
        model_path = os.path.expanduser(FREERUN_MODEL_PATH_CW)
    elif drive_mode == "FCCW":
        model_path = os.path.expanduser(FREERUN_MODEL_PATH_CCW)
    elif drive_mode == "OCW":
        model_path = os.path.expanduser(OBSTACLE_MODEL_PATH_CW)
    elif drive_mode == "OCCW":
        model_path = os.path.expanduser(OBSTACLE_MODEL_PATH_CCW)
    else:
        sys.exit("Invalid DRIVE_MODE")

    print("Model:", model_path)
    return model_path

CAPTURE_W, CAPTURE_H = (176, 132)
CROP_W, CROP_H       = (160, 120)

DATA_PATH          = os.path.expanduser("~/WRO_FE_2026/data")

DRIVE_LOOP_HZ      = 20
JOYSTICK_DEADZONE  = 0.05
RECORD_THRESHOLD   = 0.05
MAX_SPEED_PERCENT  = 100
STEERING_MAX_SPEED = 100
angle_offset       = 0.7

TUB_INPUTS = [
    "cam/image_array",
    "user/angle",
    "user/throttle",
    "user/mode",
]
TUB_TYPES  = ["image_array", "float", "float", "str"]

DXL_PORT = "/dev/ttyUSB0"
DXL_BAUDRATE = 57600
DXL_PROTOCOL_VERSION = 2.0

DXL_STEER_ID = 2
DXL_THROTTLE_IDS = [1]

DXL_THROTTLE_DIRECTIONS = [1]

DXL_STEER_CENTER_TICKS = 3060
DXL_STEER_LEFT_DEG = -60
DXL_STEER_RIGHT_DEG = 60
DXL_STEER_DIRECTION = 1
DXL_TICKS_PER_DEG = 4096 / 360

DXL_VELOCITY_UNIT_RPM = 0.229
DXL_THROTTLE_MAX_RPM = 100
DXL_STEER_PROFILE_ACCEL = 200
DXL_THROTTLE_PROFILE_ACCEL = 200

ADDR_OPERATING_MODE = 11
ADDR_TORQUE_ENABLE = 64
ADDR_GOAL_VELOCITY = 104
ADDR_PROFILE_ACCEL = 108
ADDR_PROFILE_VELOCITY = 112
ADDR_GOAL_POSITION = 116

MODE_VELOCITY = 1
MODE_POSITION = 3
TORQUE_OFF = 0
TORQUE_ON = 1

def _to_uint32(value: int) -> int:
    return int(value) & 0xFFFFFFFF

def _rpm_to_velocity_lsb(rpm: float) -> int:
    return int(round(float(rpm) / DXL_VELOCITY_UNIT_RPM))

class DynamixelBus:
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

    def configure_motor(self, dxl_id, mode, profile_accel=200, profile_velocity=None):

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

CAMERA_FRAMERATE             = 30
PICAMERA_AWB_MODE            = 'off'
PICAMERA_EXPOSURE_MODE       = 'off'
PICAMERA_ISO                 = 100
PICAMERA_SHUTTER_SPEED       = 15000
PICAMERA_AWB_GAINS           = (1.5, 1.2)
PICAMERA_EXPOSURE_COMPENSATION = 0

def center_crop(img, tw=CROP_W, th=CROP_H):
    h, w = img.shape[:2]
    x0 = (w - tw) // 2
    y0 = (h - th) // 2 - 6
    return img[y0:y0 + th, x0:x0 + tw]

class DynamixelSteering:
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

        steer_deg = self.left + (angle + 1) * (self.right - self.left) / 2
        goal = DXL_STEER_CENTER_TICKS + DXL_STEER_DIRECTION * steer_deg * DXL_TICKS_PER_DEG

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
    def __init__(self, deadzone=JOYSTICK_DEADZONE):
        pygame.init(); pygame.joystick.init()
        if pygame.joystick.get_count() == 0:
            raise RuntimeError("No PS4 controller detected.")
        self.js = pygame.joystick.Joystick(0); self.js.init()
        self.dz = deadzone
        print(f"Connected joystick: {self.js.get_name()}")

    def _dz(self, v):
        return 0.0 if abs(v) < self.dz else float(v)

    def run(self) -> Tuple[float, float, str]:
        pygame.event.pump()
        angle = self._dz(self.js.get_axis(0))
        throttle = -self._dz(self.js.get_axis(4))

        if self.js.get_button(2):

            return angle, 0.0, "erase"

        if self.js.get_button(4):
            raise KeyboardInterrupt

        return angle, throttle, "user"

    def shutdown(self):
        pygame.quit()

class ConsoleDisplay:
    def __init__(self):
        self.last_t = 0
    def run(self, angle: float, throttle: float):
        t = time.monotonic()
        if t - self.last_t >= 1.0:
            print(f"Pred -> angle {angle:+.2f}  thr {throttle:+.2f}")
            self.last_t = t

def add_camera(car: dk.vehicle.Vehicle):
    from donkeycar.parts.camera import PiCamera

    cam = PiCamera(
        image_w = CAPTURE_W,
        image_h = CAPTURE_H,
        image_d = 3
    )

    time.sleep(2)

    cam.camera.framerate           = CAMERA_FRAMERATE
    cam.camera.exposure_mode       = PICAMERA_EXPOSURE_MODE
    cam.camera.awb_mode            = PICAMERA_AWB_MODE
    cam.camera.iso                 = PICAMERA_ISO
    cam.camera.shutter_speed       = PICAMERA_SHUTTER_SPEED
    cam.camera.exposure_compensation = PICAMERA_EXPOSURE_COMPENSATION
    cam.camera.awb_gains           = PICAMERA_AWB_GAINS

    car.add(cam, outputs=["cam/raw"], threaded=True)
    car.add(Lambda(center_crop), inputs=["cam/raw"], outputs=["cam/image_array"])

def build_vehicle_recording() -> dk.vehicle.Vehicle:
    car = dk.vehicle.Vehicle()
    add_camera(car)

    car.add(PS4Joystick(), outputs=["user/angle", "user/throttle", "user/mode"])
    car.add(DynamixelSteering(), inputs=["user/angle"])
    car.add(DynamixelThrottle(), inputs=["user/throttle"])

    car.add(Lambda(lambda t, mode: mode == "user" and abs(t) > RECORD_THRESHOLD),
            inputs=["user/throttle", "user/mode"], outputs=["recording"])

    tub = TubWriter(base_path=DATA_PATH, inputs=TUB_INPUTS, types=TUB_TYPES)
    car.add(tub, inputs=TUB_INPUTS, outputs=["tub/num_records"], run_condition="recording")

    class RecordCounter:
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
        def __init__(self, tub, num_records=100):
            self.tub = tub
            self.num = int(num_records)
            self.erased = False

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

                        keep_count = max(0, delete_start - start_index)
                        cat.seekable.truncate_until_end(keep_count)
                        cat.manifest.update_line_lengths(cat.seekable.line_lengths)
                        new_catalog_paths.append(rel_path)
                        cat.close()

                    else:

                        new_catalog_paths.append(rel_path)
                        cat.close()

                except Exception as e:
                    print(f"Could not rewrite catalog {catalog_path}: {e}")

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

            manifest.catalog_paths = new_catalog_paths
            manifest.current_index = delete_start
            manifest.deleted_indexes = set(i for i in manifest.deleted_indexes if i < delete_start)
            manifest._update_catalog_metadata(update=True)

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
            if mode == "erase" and not self.erased:
                deleted_records, deleted_data_files, removed_catalog_files = self._hard_delete_last_n_records()
                print(
                    f"Hard-deleted {deleted_records} records, "
                    f"{deleted_data_files} image/json files, "
                    f"and {removed_catalog_files} old catalog files."
                )
                self.erased = True

            if mode != "erase":
                self.erased = False

    wiper = PromptWiper(tub.tub, num_records=100)
    car.add(wiper, inputs=["user/mode"], outputs=[])
    return car

def build_vehicle_driving(model_path: str) -> dk.vehicle.Vehicle:
    car = dk.vehicle.Vehicle()
    add_camera(car)

    interpreter = KerasInterpreter()
    pilot = KerasLinear(interpreter=interpreter, input_shape=(CROP_H, CROP_W, 3))
    pilot.load(model_path)

    car.add(pilot, inputs=["cam/image_array"], outputs=["pilot/angle", "pilot/throttle"])

    car.add(ConsoleDisplay(), inputs=["pilot/angle", "pilot/throttle"], outputs=[])

    car.add(DynamixelSteering(), inputs=["pilot/angle"])
    car.add(DynamixelThrottle(), inputs=["pilot/throttle"])
    return car

if __name__ == "__main__":
    import argparse

    parser = argparse.ArgumentParser(description="Record data or drive autonomously")
    group = parser.add_mutually_exclusive_group(required=True)
    group.add_argument("--recording", action="store_true", help="Manual driving & data logging")
    group.add_argument("--driving",   action="store_true", help="Autonomous driving with model")
    parser.add_argument("--model", default=None, help="Optional .h5 model path for --driving mode")
    args = parser.parse_args()

    if args.recording:
        print("Recording mode selected. Skipping model selection.")
        vehicle = build_vehicle_recording()
    else:
        if args.model:
            model_path = os.path.expanduser(args.model)
            print("Driving mode selected.")
            print("Model:", model_path)
        else:
            model_path = select_model_path_with_sensehat()

        mpath = Path(model_path).expanduser()
        if not mpath.is_file():
            sys.exit(f"Model file not found: {mpath}")
        vehicle = build_vehicle_driving(str(mpath))

    def _sigterm(_s, _f):
        print("\nSIGTERM - shutting down...")
        (vehicle.shutdown() if hasattr(vehicle, "shutdown") else vehicle.stop())
        sys.exit(0)
    signal.signal(signal.SIGTERM, _sigterm)

    try:
        vehicle.start(rate_hz=DRIVE_LOOP_HZ)
    except KeyboardInterrupt:
        print("\nCtrl-C - shutting down...")
    finally:
        (vehicle.shutdown() if hasattr(vehicle, "shutdown") else vehicle.stop())
