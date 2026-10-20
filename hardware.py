#!/usr/bin/env python3
"""AeroTHON navigation controller — telemetry plus gated full-area Guided sweep.

Hardware expected:
  Raspberry Pi 5 <-> ArduPilot flight controller over serial MAVLink2
  2x VL53L1X on I2C bus 1, Adafruit CircuitPython driver
  XSHUT: BCM GPIO20 (left), BCM GPIO21 (right)
  Front TFmini connected to the flight controller and exposed as DISTANCE_SENSOR

Flight commands are disabled by default. Enable only after a complete SITL test.
Arena origin, heading, dimensions and failsafes must be verified before real flight.
"""

from __future__ import annotations

import time
import math
from dataclasses import dataclass
from typing import Optional

# ---------------- USER-CONFIGURABLE PLACEHOLDERS ----------------
# Replace only after confirming the Pi-to-flight-controller wiring.
SERIAL_DEVICE = "/dev/ttyAMA0"       # From repo config.py; verify Pi UART wiring
SERIAL_BAUD = 921600                  # From repo config.py; FC SERIAL2_BAUD=921
MAVLINK_DIALECT = "ardupilotmega"    # ArduPilot MAVLink dialect

# Raspberry Pi BCM GPIO numbering (not physical header pin numbers).
LEFT_XSHUT_BCM = 20
RIGHT_XSHUT_BCM = 21

# VL53L1X factory address is normally 0x29. Left address follows repo config.py.
VL53L1X_DEFAULT_ADDRESS = 0x29
LEFT_VL53L1X_ADDRESS = 0x2A          # From repo config.py
RIGHT_VL53L1X_ADDRESS = VL53L1X_DEFAULT_ADDRESS

TELEMETRY_PRINT_INTERVAL_S = 1.0
SENSOR_READ_INTERVAL_S = 0.2
HEARTBEAT_TIMEOUT_S = 15.0

# 30 x 40 m arena settings from Navigation-main/config.py (full sweep route).
# These geometry values are estimates and MUST be measured for the actual arena.
FIELD_HEADING_DEG = 90.0       # PLACEHOLDER: bearing from home toward arena (+x)
ARENA_X_START_M = 13.0         # repo estimate: home to arena near edge
ARENA_WIDTH_M = 40.0           # along field +x
ARENA_LENGTH_M = 30.0          # along field +y
ARENA_Y_CENTER_M = 0.0         # PLACEHOLDER: arena centerline relative to home
ARENA_MARGIN_M = 3.0           # repo sweep inset
SWEEP_LANE_SPACING_M = 5.0
SWEEP_ALTITUDE_M = 1.0       # target altitude relative to home

# Movement commands are deliberately gated. First test in SITL.
ENABLE_FLIGHT_COMMANDS = False
SITL_CONNECTION = "tcp:127.0.0.1:5762"  # common Mission Planner SITL endpoint
MOVEMENT_CONNECTION = SITL_CONNECTION  # keep SITL default; change only after validation
TAKEOFF_ALTITUDE_M = SWEEP_ALTITUDE_M  # take off to the configured sweep altitude
WAYPOINT_REACHED_RADIUS_M = 1.5
WAYPOINT_TIMEOUT_S = 180.0
WAYPOINT_HOLD_S = 1.0
TARGET_REFRESH_INTERVAL_S = 1.0

# ---------------- MAVLink telemetry ----------------
@dataclass
class Telemetry:
    mode: str = "unknown"
    armed: Optional[bool] = None
    latitude_deg: Optional[float] = None
    longitude_deg: Optional[float] = None
    relative_altitude_m: Optional[float] = None
    gps_fix_type: Optional[int] = None
    satellites_visible: Optional[int] = None
    battery_voltage_v: Optional[float] = None
    front_lidar_m: Optional[float] = None
    front_lidar_id: Optional[int] = None
    front_lidar_orientation: Optional[int] = None
    last_heartbeat: float = 0.0


def connect_mavlink():
    from pymavlink import mavutil

    endpoint = f"{SERIAL_DEVICE}"
    print(f"Connecting to flight controller: {endpoint} @ {SERIAL_BAUD} baud")
    link = mavutil.mavlink_connection(
        endpoint,
        baud=SERIAL_BAUD,
        source_system=255,
        source_component=mavutil.mavlink.MAV_COMP_ID_ONBOARD_COMPUTER,
        dialect=MAVLINK_DIALECT,
    )
    heartbeat = link.wait_heartbeat(timeout=HEARTBEAT_TIMEOUT_S)
    if heartbeat is None:
        link.close()
        raise TimeoutError(
            "No MAVLink heartbeat. Check SERIAL_DEVICE, SERIAL_BAUD, cable, "
            "FC serial protocol, and MAVLink2 configuration."
        )
    print(f"Heartbeat received (system={link.target_system}, component={link.target_component}).")
    print("Safety: telemetry-only mode; this script will not arm or command flight.")

    # Request common telemetry rates. These are requests only, not flight commands.
    requests = [
        (mavutil.mavlink.MAVLINK_MSG_ID_GLOBAL_POSITION_INT, 2.0),
        (mavutil.mavlink.MAVLINK_MSG_ID_GPS_RAW_INT, 1.0),
        (mavutil.mavlink.MAVLINK_MSG_ID_DISTANCE_SENSOR, 5.0),
        (mavutil.mavlink.MAVLINK_MSG_ID_BATTERY_STATUS, 1.0),
    ]
    for msg_id, rate_hz in requests:
        link.mav.command_long_send(
            link.target_system,
            link.target_component,
            mavutil.mavlink.MAV_CMD_SET_MESSAGE_INTERVAL,
            0,
            msg_id,
            1_000_000.0 / rate_hz,
            0, 0, 0, 0, 0,
        )
    return link, mavutil


def initialize_vl53l1x():
    """Initialize two VL53L1X devices with separate I2C addresses.

    Requires: adafruit-blinka, adafruit-circuitpython-vl53l1x.
    Uses BCM GPIO20/21 as XSHUT through CircuitPython's board.D20/D21 aliases.
    """
    import board
    import digitalio
    import adafruit_vl53l1x

    i2c = board.I2C()  # Raspberry Pi I2C bus 1 when enabled in OS configuration
    left_shutdown = digitalio.DigitalInOut(board.D20)
    right_shutdown = digitalio.DigitalInOut(board.D21)
    left_shutdown.direction = digitalio.Direction.OUTPUT
    right_shutdown.direction = digitalio.Direction.OUTPUT

    # Hold both sensors in reset, then assign a unique address to the left one.
    left_shutdown.value = False
    right_shutdown.value = False
    time.sleep(0.1)

    left_shutdown.value = True
    time.sleep(0.1)
    left = adafruit_vl53l1x.VL53L1X(i2c)
    left.set_address(LEFT_VL53L1X_ADDRESS)
    time.sleep(0.1)

    right_shutdown.value = True
    time.sleep(0.1)
    right = adafruit_vl53l1x.VL53L1X(i2c)

    # Start ranging; API is provided by the Adafruit CircuitPython driver.
    left.start_ranging()
    right.start_ranging()
    print(
        f"VL53L1X initialized: left=0x{LEFT_VL53L1X_ADDRESS:02X}, "
        f"right=0x{RIGHT_VL53L1X_ADDRESS:02X}."
    )
    return left, right, left_shutdown, right_shutdown


def read_vl53_distance_m(sensor):
    """Return distance in metres or None if the driver reports no valid reading."""
    try:
        mm = sensor.distance
        if mm is None or mm <= 0:
            return None
        return float(mm) / 1000.0
    except (OSError, RuntimeError, ValueError):
        return None


def update_telemetry(msg, state: Telemetry, mavutil):
    kind = msg.get_type()
    if kind == "HEARTBEAT":
        state.last_heartbeat = time.monotonic()
        state.mode = mavutil.mode_string_v10(msg)
        state.armed = bool(msg.base_mode & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED)
    elif kind == "GLOBAL_POSITION_INT":
        state.latitude_deg = msg.lat / 1e7
        state.longitude_deg = msg.lon / 1e7
        state.relative_altitude_m = msg.relative_alt / 1000.0
    elif kind == "GPS_RAW_INT":
        state.gps_fix_type = int(msg.fix_type)
        state.satellites_visible = int(msg.satellites_visible)
    elif kind == "DISTANCE_SENSOR":
        # MAVLink distance fields are centimetres. min/max may be 0 when unknown.
        current_cm = int(msg.current_distance)
        if current_cm > 0 and current_cm < 65535:
            state.front_lidar_m = current_cm / 100.0
            state.front_lidar_id = int(msg.id)
            state.front_lidar_orientation = int(msg.orientation)
    elif kind == "BATTERY_STATUS":
        # voltages[] is in millivolts; 65535 means unknown.
        vals = [int(v) for v in msg.voltages if int(v) not in (0, 65535)]
        if vals:
            state.battery_voltage_v = sum(vals) / 1000.0


def fmt(value, suffix=""):
    return "n/a" if value is None else f"{value:.2f}{suffix}"


def field_to_gps(fx, fy, origin_lat, origin_lon):
    """Convert field-forward/left metres to GPS using configured field heading."""
    h = math.radians(FIELD_HEADING_DEG)
    east = fx * math.sin(h) - fy * math.cos(h)
    north = fx * math.cos(h) + fy * math.sin(h)
    radius = 6_378_137.0
    lat = origin_lat + math.degrees(north / radius)
    lon = origin_lon + math.degrees(east / (radius * math.cos(math.radians(origin_lat))))
    return lat, lon


def print_arena_plan(home_lat, home_lon):
    """Print 30x40 m arena corners and vertical-lane sweep; never commands flight."""
    x0, x1 = ARENA_X_START_M, ARENA_X_START_M + ARENA_WIDTH_M
    y0 = ARENA_Y_CENTER_M - ARENA_LENGTH_M / 2
    y1 = ARENA_Y_CENTER_M + ARENA_LENGTH_M / 2
    sx0, sx1 = x0 + ARENA_MARGIN_M, x1 - ARENA_MARGIN_M
    sy0, sy1 = y0 + ARENA_MARGIN_M, y1 - ARENA_MARGIN_M
    if sx1 <= sx0 or sy1 <= sy0 or SWEEP_LANE_SPACING_M <= 0:
        raise ValueError("Invalid arena dimensions/margin/lane spacing.")
    print("\n30 x 40 m arena plan (preview only; geometry values need field verification):")
    print(f"Heading={FIELD_HEADING_DEG:.1f} deg; arena starts {ARENA_X_START_M:.1f} m from home")
    print(f"Sweep target altitude={SWEEP_ALTITUDE_M:.1f} m relative to home; lane spacing={SWEEP_LANE_SPACING_M:.1f} m")
    print("Arena corners:")
    for name, fx, fy in [("P1",x0,y0),("P2",x1,y0),("P3",x1,y1),("P4",x0,y1)]:
        lat, lon = field_to_gps(fx, fy, home_lat, home_lon)
        print(f"  {name}: {lat:.7f}, {lon:.7f} (fx={fx:.1f}, fy={fy:+.1f})")
    span = sx1 - sx0
    n_lanes = max(2, math.ceil(span / SWEEP_LANE_SPACING_M) + 1)
    xs = [sx0 + i * span / (n_lanes - 1) for i in range(n_lanes)]
    route = []
    for i, fx in enumerate(xs):
        a, b = (sy0, sy1) if i % 2 == 0 else (sy1, sy0)
        route.extend([(fx, a), (fx, b)])
    print(f"Sweep preview: {len(route)} waypoints (all target {SWEEP_ALTITUDE_M:.1f} m):")
    for i, (fx, fy) in enumerate(route, 1):
        lat, lon = field_to_gps(fx, fy, home_lat, home_lon)
        print(f"  WP{i:02d}: {lat:.7f}, {lon:.7f} (fx={fx:.1f}, fy={fy:+.1f}, alt={SWEEP_ALTITUDE_M:.1f} m)")
    print("NOTE: coordinate preview only. This script does not upload or execute this route.")



def set_mode(link, mavutil, mode_name):
    mode_id = link.mode_mapping().get(mode_name)
    if mode_id is None:
        raise RuntimeError(f"Flight controller does not expose {mode_name} mode")
    link.mav.set_mode_send(link.target_system, mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED, mode_id)


def horizontal_distance_m(lat1, lon1, lat2, lon2):
    radius = 6_378_137.0
    p1, p2 = math.radians(lat1), math.radians(lat2)
    dp = p2 - p1
    dl = math.radians(lon2 - lon1)
    a = math.sin(dp / 2)**2 + math.cos(p1) * math.cos(p2) * math.sin(dl / 2)**2
    return 2 * radius * math.asin(min(1.0, math.sqrt(a)))


def wait_for_state(link, state, mavutil, predicate, timeout_s, description):
    deadline = time.monotonic() + timeout_s
    while time.monotonic() < deadline:
        msg = link.recv_match(type=["HEARTBEAT", "GLOBAL_POSITION_INT", "GPS_RAW_INT"], blocking=True, timeout=0.5)
        if msg is not None:
            update_telemetry(msg, state, mavutil)
        if predicate(state):
            return
    raise TimeoutError(f"Timed out waiting for {description}")


def send_global_target(link, mavutil, lat, lon, altitude_m):
    # GLOBAL_RELATIVE_ALT_INT: altitude in metres above home.
    link.mav.set_position_target_global_int_send(
        0, link.target_system, link.target_component,
        mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT_INT,
        0b0000111111111000,
        int(lat * 1e7), int(lon * 1e7), float(altitude_m),
        0, 0, 0, 0, 0, 0, 0, 0,
    )


def build_sweep_route(origin_lat, origin_lon):
    """Build a complete serpentine lawnmower route for the 30 x 40 m arena."""
    x_min = ARENA_X_START_M + ARENA_MARGIN_M
    x_max = ARENA_X_START_M + ARENA_WIDTH_M - ARENA_MARGIN_M
    y_min = ARENA_Y_CENTER_M - ARENA_LENGTH_M / 2 + ARENA_MARGIN_M
    y_max = ARENA_Y_CENTER_M + ARENA_LENGTH_M / 2 - ARENA_MARGIN_M
    if x_max <= x_min or y_max <= y_min or SWEEP_LANE_SPACING_M <= 0:
        raise ValueError("Invalid arena dimensions, margin, or lane spacing")

    # Sweep along the 30 m dimension; shift sideways across the 40 m dimension.
    lane_count = max(2, math.ceil((x_max - x_min) / SWEEP_LANE_SPACING_M) + 1)
    lane_x = [x_min + i * (x_max - x_min) / (lane_count - 1) for i in range(lane_count)]
    route = []
    for lane_i, x in enumerate(lane_x):
        y_pair = (y_min, y_max) if lane_i % 2 == 0 else (y_max, y_min)
        for y in y_pair:
            lat, lon = field_to_gps(x, y, origin_lat, origin_lon)
            route.append((lat, lon, SWEEP_ALTITUDE_M, x, y, lane_i + 1))
    return route


def run_full_sweep(link, mavutil):
    """Arm, take off, and execute the complete serpentine sweep in Guided mode."""
    state = Telemetry(last_heartbeat=time.monotonic())
    wait_for_state(link, state, mavutil,
        lambda x: x.latitude_deg is not None and x.longitude_deg is not None and x.relative_altitude_m is not None,
        30, "GPS position and relative altitude")
    if state.gps_fix_type is not None and state.gps_fix_type < 3:
        raise RuntimeError(f"GPS fix type {state.gps_fix_type}; need a 3D fix")

    origin_lat, origin_lon = state.latitude_deg, state.longitude_deg
    route = build_sweep_route(origin_lat, origin_lon)
    print("\nFULL 30 x 40 m SWEEP ROUTE")
    print(f"Origin reference: {origin_lat:.7f}, {origin_lon:.7f}")
    print(f"Lane spacing: {SWEEP_LANE_SPACING_M:.1f} m | altitude: {SWEEP_ALTITUDE_M:.1f} m")
    print(f"Lanes: {len(route)//2} | movement targets: {len(route)}")
    for i, (lat, lon, alt, x, y, lane) in enumerate(route, 1):
        print(f"  WP{i:02d} lane={lane:02d} field=({x:.1f},{y:+.1f}) GPS=({lat:.7f},{lon:.7f}) alt={alt:.1f}m")
    print("\nMovement enabled: takeoff then full sweep. RTL is requested after completion or timeout.")

    set_mode(link, mavutil, "GUIDED")
    time.sleep(2)
    link.mav.command_long_send(link.target_system, link.target_component,
        mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM, 0, 1, 0, 0, 0, 0, 0, 0)
    wait_for_state(link, state, mavutil, lambda x: x.armed is True, 30, "arming")
    link.mav.command_long_send(link.target_system, link.target_component,
        mavutil.mavlink.MAV_CMD_NAV_TAKEOFF, 0, 0, 0, 0, FIELD_HEADING_DEG, 0, 0, TAKEOFF_ALTITUDE_M)
    wait_for_state(link, state, mavutil,
        lambda x: x.relative_altitude_m is not None and x.relative_altitude_m >= TAKEOFF_ALTITUDE_M * 0.9,
        90, "sweep altitude")

    for i, (lat, lon, alt, fx, fy, lane) in enumerate(route, 1):
        print(f"SEND WP{i:02d}/{len(route)} lane={lane:02d} -> field=({fx:.1f},{fy:+.1f}), alt={alt:.1f}m")
        deadline = time.monotonic() + WAYPOINT_TIMEOUT_S
        reached = False
        while time.monotonic() < deadline:
            # ArduPilot Guided position targets should be refreshed while travelling.
            send_global_target(link, mavutil, lat, lon, alt)
            msg = link.recv_match(type=["GLOBAL_POSITION_INT", "HEARTBEAT", "GPS_RAW_INT"], blocking=True, timeout=TARGET_REFRESH_INTERVAL_S)
            if msg is not None:
                update_telemetry(msg, state, mavutil)
            if state.latitude_deg is not None and state.longitude_deg is not None:
                err = horizontal_distance_m(state.latitude_deg, state.longitude_deg, lat, lon)
                alt_ok = state.relative_altitude_m is not None and abs(state.relative_altitude_m - alt) <= 1.5
                if err <= WAYPOINT_REACHED_RADIUS_M and alt_ok:
                    print(f"  REACHED WP{i:02d} (horizontal error {err:.1f}m, altitude {state.relative_altitude_m:.1f}m)")
                    reached = True
                    break
        if not reached:
            print(f"  TIMEOUT at WP{i:02d}; cancelling sweep and requesting RTL")
            try:
                set_mode(link, mavutil, "RTL")
            finally:
                return
        time.sleep(WAYPOINT_HOLD_S)

    print("FULL SWEEP COMPLETE — requesting RTL")
    set_mode(link, mavutil, "RTL")


def main():
    print("AeroTHON FULL-AREA SWEEP CONTROLLER")
    print(f"Flight commands enabled: {ENABLE_FLIGHT_COMMANDS}")
    if ENABLE_FLIGHT_COMMANDS:
        print("WARNING: movement commands will be sent. SITL test first; do not use guessed geometry on a real aircraft.")
    else:
        print("Telemetry-only. Set ENABLE_FLIGHT_COMMANDS=True to deliberately enable the full Guided sweep.\n")

    link = None
    left = right = None
    left_shutdown = right_shutdown = None
    try:
        if ENABLE_FLIGHT_COMMANDS:
            from pymavlink import mavutil as _mavutil
            endpoint = MOVEMENT_CONNECTION
            print(f"Connecting for movement test: {endpoint} @ {SERIAL_BAUD} baud")
            link = _mavutil.mavlink_connection(endpoint, baud=SERIAL_BAUD, source_system=255,
                source_component=_mavutil.mavlink.MAV_COMP_ID_ONBOARD_COMPUTER, dialect=MAVLINK_DIALECT)
            if link.wait_heartbeat(timeout=HEARTBEAT_TIMEOUT_S) is None:
                raise TimeoutError("No MAVLink heartbeat")
            mavutil = _mavutil
            run_full_sweep(link, mavutil)
            return
        link, mavutil = connect_mavlink()
        try:
            left, right, left_shutdown, right_shutdown = initialize_vl53l1x()
            sensors_ok = True
        except Exception as exc:
            sensors_ok = False
            print(f"SIDE-SENSOR INITIALIZATION FAILED: {type(exc).__name__}: {exc}")
            print("MAVLink telemetry will continue; side sensors will show n/a.")

        state = Telemetry(last_heartbeat=time.monotonic())
        next_sensor_read = 0.0
        next_print = 0.0
        left_m = right_m = None
        plan_printed = False

        while True:
            msg = link.recv_match(
                type=["HEARTBEAT", "GLOBAL_POSITION_INT", "GPS_RAW_INT", "DISTANCE_SENSOR", "BATTERY_STATUS"],
                blocking=True,
                timeout=0.2,
            )
            if msg is not None:
                update_telemetry(msg, state, mavutil)
                if (not plan_printed and state.latitude_deg is not None and state.longitude_deg is not None):
                    print_arena_plan(state.latitude_deg, state.longitude_deg)
                    plan_printed = True

            now = time.monotonic()
            if now - state.last_heartbeat > HEARTBEAT_TIMEOUT_S:
                print("WARNING: heartbeat timeout — telemetry link may be lost.")
                state.last_heartbeat = now  # avoid flooding; continue observing

            if sensors_ok and now >= next_sensor_read:
                left_m = read_vl53_distance_m(left)
                right_m = read_vl53_distance_m(right)
                next_sensor_read = now + SENSOR_READ_INTERVAL_S

            if now >= next_print:
                armed_text = "unknown" if state.armed is None else str(state.armed)
                gps_text = (
                    "n/a" if state.latitude_deg is None else
                    f"{state.latitude_deg:.7f}, {state.longitude_deg:.7f}"
                )
                print(
                    f"mode={state.mode} armed={armed_text} GPS=({gps_text}) "
                    f"fix={state.gps_fix_type} sats={state.satellites_visible} "
                    f"rel_alt={fmt(state.relative_altitude_m, ' m')} "
                    f"battery={fmt(state.battery_voltage_v, ' V')} "
                    f"front_TFmini={fmt(state.front_lidar_m, ' m')} "
                    f"(id={state.front_lidar_id}, orient={state.front_lidar_orientation}) "
                    f"left_VL53L1X={fmt(left_m, ' m')} "
                    f"right_VL53L1X={fmt(right_m, ' m')}"
                )
                next_print = now + TELEMETRY_PRINT_INTERVAL_S

    except KeyboardInterrupt:
        print("\nStopped by user.")
    finally:
        for sensor in (left, right):
            if sensor is not None:
                try:
                    sensor.stop_ranging()
                except Exception:
                    pass
        for pin in (left_shutdown, right_shutdown):
            if pin is not None:
                try:
                    pin.deinit()
                except Exception:
                    pass
        if link is not None:
            link.close()
            print("MAVLink connection closed.")


if __name__ == "__main__":
    main()
