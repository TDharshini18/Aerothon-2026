#!/usr/bin/env python3
"""
  python3 sweeper.py --corners "lat,lon;lat,lon;lat,lon;lat,lon"
        telemetry + route preview (never flies)
  python3 sweeper.py --fly --endpoint tcp:127.0.0.1:5762 --corners "..."
        arm, take off, sweep, return (SITL example)
  python3 sweeper.py --fly --airborne --endpoint ... --corners "..."
        sweep from the air (already ARMED + GUIDED)
--corners = the 4 competition corner points in PERIMETER ORDER (either direction), as
  "lat,lon;lat,lon;lat,lon;lat,lon". They are validated and re-ordered automatically.
MANUAL OVERRIDE (only while --fly is active)
  * Press ENTER or Ctrl+C at any time: the script stops commanding, switches to LOITER
    (holds position) and exits. Take over with the RC transmitter.
  * If the pilot changes mode on the RC switch, the script also stops and leaves control to the pilot.
"""
from __future__ import annotations

import argparse
import math
import signal
import sys
import threading
import time
from dataclasses import dataclass
from typing import Optional
from pymavlink import mavutil

MAVLINK_ENDPOINT = "udpin:127.0.0.1:14550"   # MAVProxy --out=udp:127.0.0.1:14550
SERIAL_BAUD = 921600                          # only used for serial endpoints
MAVLINK_DIALECT = "ardupilotmega"
SOURCE_SYSTEM = 1                             # not 255 (that is the GCS id)
SOURCE_COMPONENT = 191                        # MAV_COMP_ID_ONBOARD_COMPUTER
HEARTBEAT_TIMEOUT_S = 15.0                    # time allowed to find the autopilot at startup
HEARTBEAT_LOST_S = 3.0                        # heartbeat silence that counts as lost
POSITION_STALE_S = 3.0                        # in flight: no position for this long -> abort
TELEMETRY_PRINT_INTERVAL_S = 1.0

# ---------------- sweep ----------------
SWEEP_ALTITUDE_M = 3.0  # at 1m change to 10
SWEEP_SPEED_MPS = 1.5
SWEEP_LANE_SPACING_M = 3.0
SWEEP_MARGIN_M = 0.5
TAKEOFF_ALTITUDE_M = SWEEP_ALTITUDE_M
END_ACTION = "RTL"               # "RTL", "LAND" or "HOLD" after the last waypoint  # change when fsm
OVERRIDE_MODE = "LOITER"         # mode set when the pilot takes over manually
WAYPOINT_REACHED_RADIUS_M = 1.5
WAYPOINT_ALT_TOLERANCE_M = 1     # test
WAYPOINT_TIMEOUT_S = 180.0
WAYPOINT_HOLD_S = 0.0
WAYPOINT_ACCEPT_RADIUS_M = 1.5  
TARGET_REFRESH_INTERVAL_S = 1.0
MIN_SATELLITES = 8
ZONE_CORNERS = None              # from input

# fallback if gps fails
FIELD_HEADING_DEG = 90.0
ARENA_X_START_M = 13.0
ARENA_WIDTH_M = 40.0
ARENA_LENGTH_M = 30.0
ARENA_Y_CENTER_M = 0.0

COPTER_MODE = {"GUIDED": 4, "LOITER": 5, "RTL": 6, "LAND": 9}   # ArduCopter custom-mode numbers
ALL_TYPES = ["HEARTBEAT", "GLOBAL_POSITION_INT", "GPS_RAW_INT"]
EARTH_RADIUS_M = 6_378_137.0

# Set by ENTER, Ctrl+C, or the override listener. Checked on every control tick.
override_event = threading.Event()

class Abort(Exception):
    """Stop commanding and leave the aircraft to the pilot / FC failsafes."""

class ManualOverride(Exception):
    """Pilot requested manual control. Hold position, then hand over."""

# data from mavlink message
@dataclass
class Telemetry:
    mode: str = "unknown"
    armed: Optional[bool] = None
    latitude_deg: Optional[float] = None
    longitude_deg: Optional[float] = None
    relative_altitude_m: Optional[float] = None
    gps_fix_type: Optional[int] = None
    satellites_visible: Optional[int] = None
    last_heartbeat: float = 0.0
    last_position: float = 0.0


def is_autopilot_heartbeat(msg, target_system=None):
    """Accept only the flight controller's heartbeat."""
    if msg.type == mavutil.mavlink.MAV_TYPE_GCS:
        return False
    if msg.autopilot == mavutil.mavlink.MAV_AUTOPILOT_INVALID:
        return False
    if target_system is not None and msg.get_srcSystem() != target_system:
        return False
    return True


def connect(endpoint, baud=SERIAL_BAUD):
    print(f"Connecting: {endpoint}")
    link = mavutil.mavlink_connection(
        endpoint, baud=baud, source_system=SOURCE_SYSTEM,
        source_component=SOURCE_COMPONENT, dialect=MAVLINK_DIALECT)
    t0 = time.monotonic()
    while time.monotonic() - t0 < HEARTBEAT_TIMEOUT_S:
        hb = link.recv_match(type="HEARTBEAT", blocking=True, timeout=1)
        if hb is not None and is_autopilot_heartbeat(hb):
            link.target_system = hb.get_srcSystem()
            link.target_component = hb.get_srcComponent()
            break
    else:
        link.close()
        raise TimeoutError(
            "No autopilot heartbeat. Check the endpoint, that MAVProxy is running with a "
            "matching --out, and that no other program owns the serial port.")
    print(f"Autopilot heartbeat (system={link.target_system}, component={link.target_component}).")
    for msg_id, rate_hz in [
        (mavutil.mavlink.MAVLINK_MSG_ID_GLOBAL_POSITION_INT, 5.0),
        (mavutil.mavlink.MAVLINK_MSG_ID_GPS_RAW_INT, 1.0),
    ]:
        link.mav.command_long_send(
            link.target_system, link.target_component,
            mavutil.mavlink.MAV_CMD_SET_MESSAGE_INTERVAL, 0,
            msg_id, 1_000_000.0 / rate_hz, 0, 0, 0, 0, 0)
    return link


def update_telemetry(msg, state: Telemetry, target_system):
    if msg.get_srcSystem() != target_system:
        return
    kind = msg.get_type()
    now = time.monotonic()
    if kind == "HEARTBEAT":
        if not is_autopilot_heartbeat(msg, target_system):
            return
        state.last_heartbeat = now
        state.mode = mavutil.mode_string_v10(msg)
        state.armed = bool(msg.base_mode & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED)
    elif kind == "GLOBAL_POSITION_INT":
        state.latitude_deg = msg.lat / 1e7
        state.longitude_deg = msg.lon / 1e7
        state.relative_altitude_m = msg.relative_alt / 1000.0
        state.last_position = now
    elif kind == "GPS_RAW_INT":
        state.gps_fix_type = int(msg.fix_type)
        state.satellites_visible = int(msg.satellites_visible)


def pump(link, state, timeout=0.2):
    msg = link.recv_match(type=ALL_TYPES, blocking=True, timeout=timeout)
    if msg is not None:
        update_telemetry(msg, state, link.target_system)
    return msg


def fmt(value, suffix=""):
    return "n/a" if value is None else f"{value:.2f}{suffix}"


# ---------------- geometry ----------------
def horizontal_distance_m(lat1, lon1, lat2, lon2):
    p1, p2 = math.radians(lat1), math.radians(lat2)
    dp, dl = p2 - p1, math.radians(lon2 - lon1)
    a = math.sin(dp / 2) ** 2 + math.cos(p1) * math.cos(p2) * math.sin(dl / 2) ** 2
    return 2 * EARTH_RADIUS_M * math.asin(min(1.0, math.sqrt(a)))


def bearing_rad(c1, c2):
    """Initial bearing c1 -> c2, radians clockwise from north (what ArduPilot wants as yaw)."""
    la1, la2 = math.radians(c1[0]), math.radians(c2[0])
    dlo = math.radians(c2[1] - c1[1])
    return math.atan2(math.sin(dlo) * math.cos(la2),
                      math.cos(la1) * math.sin(la2) - math.sin(la1) * math.cos(la2) * math.cos(dlo))


def field_to_gps(fx, fy, origin_lat, origin_lon):
    """Field-forward/left metres -> GPS, using FIELD_HEADING_DEG (fallback preview only)."""
    h = math.radians(FIELD_HEADING_DEG)
    east = fx * math.sin(h) - fy * math.cos(h)
    north = fx * math.cos(h) + fy * math.sin(h)
    lat = origin_lat + math.degrees(north / EARTH_RADIUS_M)
    lon = origin_lon + math.degrees(east / (EARTH_RADIUS_M * math.cos(math.radians(origin_lat))))
    return lat, lon


def fallback_corners(origin_lat, origin_lon):
    x0, x1 = ARENA_X_START_M, ARENA_X_START_M + ARENA_WIDTH_M
    y0 = ARENA_Y_CENTER_M - ARENA_LENGTH_M / 2
    y1 = ARENA_Y_CENTER_M + ARENA_LENGTH_M / 2
    pts = [(x0, y0), (x0, y1), (x1, y1), (x1, y0)]
    return [field_to_gps(x, y, origin_lat, origin_lon) for x, y in pts]


def _local_m(c, o):
    return ((c[1] - o[1]) * math.cos(math.radians(o[0])) * 111320.0, (c[0] - o[0]) * 110540.0)


def validate_and_orient(corners, here=None):
    """Check the 4 competition corners, then reorder them so C1->C2 is the LONG side (fewer turns)
    and C1 is the corner nearest the drone. Raises ValueError with a plain reason."""
    if len(corners) != 4:
        raise ValueError("need exactly 4 corners")
    for lat, lon in corners:
        if not (-90 <= lat <= 90 and -180 <= lon <= 180):
            raise ValueError(f"corner {lat},{lon} is not a valid GPS position")
    pts = [_local_m(c, corners[0]) for c in corners]
    cross = []
    for i in range(4):
        o, a, b = pts[i], pts[(i + 1) % 4], pts[(i + 2) % 4]
        cross.append((a[0] - o[0]) * (b[1] - o[1]) - (a[1] - o[1]) * (b[0] - o[0]))
    if not (all(c > 0 for c in cross) or all(c < 0 for c in cross)):
        raise ValueError("corners are not in perimeter order (or the shape is not convex)")
    sides = [math.dist(pts[i], pts[(i + 1) % 4]) for i in range(4)]
    if min(sides) < 10 or max(sides) > 100:
        raise ValueError(f"side lengths {[round(s, 1) for s in sides]} m look wrong")
    for i in (0, 1):
        if not 0.8 <= sides[i] / sides[i + 2] <= 1.25:
            raise ValueError(f"opposite sides differ too much: {sides[i]:.1f} vs {sides[i + 2]:.1f} m")
    if here is not None:
        far = max(horizontal_distance_m(here[0], here[1], lat, lon) for lat, lon in corners)
        if far > 200:
            raise ValueError(f"a corner is {far:.0f} m from the drone -- lat/lon typo?")
    options = []
    for k in range(4):
        for d in (1, -1):
            rot = [corners[(k + d * i) % 4] for i in range(4)]
            if horizontal_distance_m(*rot[0], *rot[1]) >= horizontal_distance_m(*rot[0], *rot[3]):
                options.append(rot)
    if here is None:
        return options[0]
    return min(options, key=lambda r: horizontal_distance_m(here[0], here[1], r[0][0], r[0][1]))


def parse_corners(text):
    """'lat,lon;lat,lon;lat,lon;lat,lon' -> [(lat, lon), ...]"""
    return [tuple(float(v) for v in p.split(",")) for p in text.split(";")]


def get_corners(state: Telemetry):
    """Return (corners, source). Real, validated corners when provided, else guessed fallback (preview only)."""
    if ZONE_CORNERS is not None:
        here = ((state.latitude_deg, state.longitude_deg)
                if state.latitude_deg is not None else None)
        return validate_and_orient(list(ZONE_CORNERS), here), "provided"
    if state.latitude_deg is None or state.longitude_deg is None:
        raise RuntimeError("No GPS position for fallback geometry")
    return fallback_corners(state.latitude_deg, state.longitude_deg), "FALLBACK (guessed)"


def bilinear(corners, s, t):
    """s: 0..1 along C1->C2, t: 0..1 from the C1-C2 edge toward the C4-C3 edge."""
    (a1, o1), (a2, o2), (a3, o3), (a4, o4) = corners
    la = (1 - t) * ((1 - s) * a1 + s * a2) + t * ((1 - s) * a4 + s * a3)
    lo = (1 - t) * ((1 - s) * o1 + s * o2) + t * ((1 - s) * o4 + s * o3)
    return la, lo


def build_sweep_route(corners):
    """Serpentine lanes inside the zone. Returns [(lat, lon, alt, lane_no), ...]."""
    c1, c2, _c3, c4 = corners
    along = horizontal_distance_m(*c1, *c2)
    across = horizontal_distance_m(*c1, *c4)
    use_along = along - 2 * SWEEP_MARGIN_M
    use_across = across - 2 * SWEEP_MARGIN_M
    if use_along <= 0 or use_across <= 0 or SWEEP_LANE_SPACING_M <= 0:
        raise ValueError(f"Zone {along:.1f} x {across:.1f} m too small for margin/lane spacing")
    lanes = max(2, math.ceil(use_across / SWEEP_LANE_SPACING_M) + 1)
    s_a, s_b = SWEEP_MARGIN_M / along, 1 - SWEEP_MARGIN_M / along
    route = []
    for i in range(lanes):
        t = (SWEEP_MARGIN_M + i * use_across / (lanes - 1)) / across
        a, b = bilinear(corners, s_a, t), bilinear(corners, s_b, t)
        for lat, lon in ((a, b) if i % 2 == 0 else (b, a)):
            route.append((lat, lon, SWEEP_ALTITUDE_M, i + 1))
    return route


def route_length_m(route):
    return sum(horizontal_distance_m(route[i][0], route[i][1], route[i + 1][0], route[i + 1][1])
               for i in range(len(route) - 1))


def print_route(corners, source, route):
    c1, c2, _c3, c4 = corners
    print(f"\nSweep plan | geometry: {source}")
    if source != "provided":
        print("  WARNING: guessed arena geometry. Give the real corners with --corners. The script will NOT fly on this.")
    print(f"  Zone {horizontal_distance_m(*c1, *c2):.1f} m (lane direction) x "
          f"{horizontal_distance_m(*c1, *c4):.1f} m (step direction)")
    for name, (lat, lon) in zip(("C1", "C2", "C3", "C4"), corners):
        print(f"  {name}: {lat:.7f}, {lon:.7f}")
    length = route_length_m(route)
    print(f"  {len(route)} waypoints, {route[-1][3]} lanes @ {SWEEP_ALTITUDE_M:.0f} m, "
          f"~{length:.0f} m path, ~{length / SWEEP_SPEED_MPS / 60:.1f} min at {SWEEP_SPEED_MPS:.1f} m/s")
    print(f"  Fixed heading {math.degrees(bearing_rad(c1, c2)) % 360:.0f} deg (along C1->C2)")
    for i, (lat, lon, alt, lane) in enumerate(route, 1):
        print(f"  WP{i:02d} lane {lane:02d}: {lat:.7f}, {lon:.7f} alt={alt:.0f} m")


# ---------------- commands ----------------
def set_mode(link, name):
    link.mav.set_mode_send(
        link.target_system, mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED, COPTER_MODE[name])


def send_global_target(link, lat, lon, alt_m, yaw):
    link.mav.set_position_target_global_int_send(
        0, link.target_system, link.target_component,
        mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT_INT,
        0x09F8,                                   # position + yaw only
        int(lat * 1e7), int(lon * 1e7), float(alt_m),
        0, 0, 0, 0, 0, 0, yaw, 0)


# ---------------- manual override ----------------
def start_override_listener():
    """ENTER on stdin requests a manual override. Runs in a daemon thread."""
    def _listen():
        try:
            sys.stdin.readline()
        except Exception:
            return
        override_event.set()
    threading.Thread(target=_listen, daemon=True).start()


def install_ctrl_c_override():
    """Ctrl+C during a flight requests a manual override instead of killing the script."""
    def _handler(signum, frame):
        override_event.set()
    signal.signal(signal.SIGINT, _handler)


def guard(state: Telemetry, in_flight):
    if override_event.is_set():
        raise ManualOverride("manual override requested")
    if time.monotonic() - state.last_heartbeat > HEARTBEAT_LOST_S:
        raise Abort("autopilot heartbeat lost")
    if in_flight:
        if state.armed is False:
            raise Abort("aircraft disarmed")
        if state.mode != "GUIDED":
            raise Abort(f"pilot took over (mode is {state.mode})")
        if time.monotonic() - state.last_position > POSITION_STALE_S:
            raise Abort("GPS position stale")


def wait_for(link, state, predicate, timeout_s, description, in_flight=False):
    deadline = time.monotonic() + timeout_s
    while time.monotonic() < deadline:
        pump(link, state, 0.5)
        guard(state, in_flight)
        if predicate(state):
            return
    raise Abort(f"timed out waiting for {description}")


# ---------------- sweep ----------------
def run_full_sweep(link, airborne=False):
    state = Telemetry(last_heartbeat=time.monotonic(), last_position=time.monotonic())
    try:
        _sweep(link, state, airborne)
    except ManualOverride:
        if state.armed is not False:
            try:
                set_mode(link, OVERRIDE_MODE)
                print(f"MANUAL OVERRIDE: mode set to {OVERRIDE_MODE}. Take control with the transmitter.")
            except Exception as exc:
                print(f"MANUAL OVERRIDE: could not set {OVERRIDE_MODE} ({exc}). Take control now.")
        else:
            print("MANUAL OVERRIDE: aircraft not armed, nothing to hold.")
        raise


def _sweep(link, state, airborne):
    wait_for(link, state,
             lambda s: (s.latitude_deg is not None and s.relative_altitude_m is not None
                        and s.gps_fix_type is not None and s.armed is not None),
             30, "GPS position, fix and heartbeat")
    if state.gps_fix_type < 3:
        raise RuntimeError(f"GPS fix type {state.gps_fix_type}; need a 3D fix")
    if state.satellites_visible is not None and state.satellites_visible < MIN_SATELLITES:
        raise RuntimeError(f"Only {state.satellites_visible} satellites; need {MIN_SATELLITES}+")

    corners, source = get_corners(state)
    if source != "provided":
        raise RuntimeError("Refusing to fly: no --corners given")
    route = build_sweep_route(corners)
    print_route(corners, source, route)
    lane_yaw = bearing_rad(corners[0], corners[1])

    if airborne:
        if not (state.armed and state.mode == "GUIDED" and (state.relative_altitude_m or 0) > 2.0):
            raise RuntimeError("--airborne needs the aircraft ARMED, in GUIDED and above 2 m")
    else:
        if state.armed:
            raise RuntimeError("Aircraft is already armed; start the sweep from the ground disarmed "
                               "or use --airborne")
        set_mode(link, "GUIDED")
        wait_for(link, state, lambda s: s.mode == "GUIDED", 5, "GUIDED mode")
        link.mav.command_long_send(
            link.target_system, link.target_component,
            mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM, 0, 1, 0, 0, 0, 0, 0, 0)
        wait_for(link, state, lambda s: s.armed is True, 30, "arming (check PreArm messages)")
        link.mav.command_long_send(
            link.target_system, link.target_component,
            mavutil.mavlink.MAV_CMD_NAV_TAKEOFF, 0, 0, 0, 0, 0, 0, 0, TAKEOFF_ALTITUDE_M)
        wait_for(link, state,
                 lambda s: s.relative_altitude_m is not None
                 and s.relative_altitude_m >= TAKEOFF_ALTITUDE_M * 0.9,
                 90, "takeoff altitude", in_flight=True)

    link.mav.command_long_send(
        link.target_system, link.target_component,
        mavutil.mavlink.MAV_CMD_DO_CHANGE_SPEED, 0, 1, SWEEP_SPEED_MPS, -1, 0, 0, 0, 0)

    for i, (lat, lon, alt, lane) in enumerate(route, 1):
        print(f"SEND WP{i:02d}/{len(route)} lane {lane:02d} -> {lat:.7f}, {lon:.7f}, {alt:.0f} m")
        deadline = time.monotonic() + WAYPOINT_TIMEOUT_S
        last_send = 0.0
        while True:
            now = time.monotonic()
            if now - last_send >= TARGET_REFRESH_INTERVAL_S:
                send_global_target(link, lat, lon, alt, lane_yaw)
                last_send = now
            pump(link, state, 0.2)
            guard(state, in_flight=True)
            err = horizontal_distance_m(state.latitude_deg, state.longitude_deg, lat, lon)
            alt_ok = (state.relative_altitude_m is not None
                      and abs(state.relative_altitude_m - alt) <= WAYPOINT_ALT_TOLERANCE_M)
            is_last = (i == len(route))
            radius = WAYPOINT_REACHED_RADIUS_M if is_last else WAYPOINT_ACCEPT_RADIUS_M
            if err <= radius and alt_ok:    

                print(f"  reached WP{i:02d} (error {err:.1f} m, alt {state.relative_altitude_m:.1f} m)")
                break
            if time.monotonic() > deadline:
                timeout_reason = f"timeout at WP{i:02d} (still {err:.1f} m away)"
                print(f"  TIMEOUT: {timeout_reason}; requesting RTL")
                try:
                    set_mode(link, "RTL")
                except Exception as exc:
                    raise Abort(
                        f"{timeout_reason}; RTL mode request failed ({type(exc).__name__}: {exc}). "
                        "Pilot must take control immediately."
                    ) from exc

                rtl_deadline = time.monotonic() + 5.0
                while time.monotonic() < rtl_deadline:
                    msg = link.recv_match(type=ALL_TYPES, blocking=True, timeout=0.5)
                    if msg is not None:
                        update_telemetry(msg, state, link.target_system)
                    if state.mode == "RTL":
                        raise Abort(f"{timeout_reason}; RTL confirmed by autopilot heartbeat")

                raise Abort(
                    f"{timeout_reason}; RTL was requested but not confirmed within 5 seconds. "
                    "Check Mission Planner immediately and take manual control if needed."
                )
        time.sleep(WAYPOINT_HOLD_S)

    print(f"SWEEP COMPLETE. End action: {END_ACTION}")
    if END_ACTION in ("RTL", "LAND"):
        set_mode(link, END_ACTION)
    else:
        print("Holding position in GUIDED for the next phase.")


# ---------------- telemetry mode ----------------
def run_telemetry(link):
    state = Telemetry(last_heartbeat=time.monotonic())
    plan_printed = False
    next_print = 0.0
    while True:
        pump(link, state, 0.2)
        now = time.monotonic()
        if not plan_printed and state.latitude_deg is not None:
            try:
                corners, source = get_corners(state)
                print_route(corners, source, build_sweep_route(corners))
            except Exception as exc:
                print(f"Route preview unavailable: {exc}")
            plan_printed = True
        if now - state.last_heartbeat > HEARTBEAT_LOST_S and state.last_heartbeat:
            print("WARNING: autopilot heartbeat lost")
            state.last_heartbeat = now
        if now >= next_print:
            gps = ("n/a" if state.latitude_deg is None
                   else f"{state.latitude_deg:.7f}, {state.longitude_deg:.7f}")
            print(f"mode={state.mode} armed={state.armed} GPS=({gps}) fix={state.gps_fix_type} "
                  f"sats={state.satellites_visible} rel_alt={fmt(state.relative_altitude_m, ' m')}")
            next_print = now + TELEMETRY_PRINT_INTERVAL_S


def main():
    global ZONE_CORNERS
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawTextHelpFormatter)
    ap.add_argument("--endpoint", help=f"MAVLink endpoint (default {MAVLINK_ENDPOINT}); "
                                       "REQUIRED with --fly")
    ap.add_argument("--baud", type=int, default=SERIAL_BAUD)
    ap.add_argument("--corners", help='"lat,lon;lat,lon;lat,lon;lat,lon" in perimeter order')
    ap.add_argument("--fly", action="store_true", help="enable flight commands")
    ap.add_argument("--airborne", action="store_true",
                    help="with --fly: do not arm/take off; sweep from the current ARMED+GUIDED state")
    args = ap.parse_args()

    if args.corners:
        try:
            ZONE_CORNERS = parse_corners(args.corners)
        except ValueError:
            sys.exit('--corners must look like "lat,lon;lat,lon;lat,lon;lat,lon"')
    if args.fly and not args.endpoint:
        sys.exit("--fly needs an explicit --endpoint (e.g. tcp:127.0.0.1:5762 for SITL).")
    endpoint = args.endpoint or MAVLINK_ENDPOINT

    link = None
    try:
        link = connect(endpoint, args.baud)
        if args.fly:
            print(f"\nFLIGHT COMMANDS ENABLED on {endpoint}")
            if input("Type FLY to continue: ").strip() != "FLY":
                print("Cancelled.")
                return
            install_ctrl_c_override()
            start_override_listener()
            print("Manual override ready: press ENTER or Ctrl+C at any time.")
            run_full_sweep(link, airborne=args.airborne)
        else:
            print("Telemetry-only. Ctrl+C to stop.\n")
            run_telemetry(link)
    except ManualOverride as exc:
        print(f"\nSTOPPED: {exc}. Script is no longer commanding the aircraft.")
        return 0
    except Abort as exc:
        print(f"\nABORT: {exc}")
        print("Verify the actual flight mode in Mission Planner. If RTL is not active, take manual control.")
        return 1
    except (ValueError, RuntimeError) as exc:
        print(f"\nERROR: {exc}")
        return 1
    except KeyboardInterrupt:
        print("\nStopped by user (no mode change made).")
    finally:
        if link is not None:
            link.close()
            print("MAVLink connection closed.")
if __name__ == "__main__":
    sys.exit(main() or 0)

