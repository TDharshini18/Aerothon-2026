# import argparse
# import time

# import cv2
# from picamera2 import Picamera2
# from pymavlink import mavutil

# # Starting endpoint from your existing ToF script.
# # It must actually route to the flight controller for commands to work.
# MAVLINK = "udp:127.0.0.1:14550"

# LOWER_GREEN = (35, 60, 45)
# UPPER_GREEN = (90, 255, 255)

# MIN_AREA = 1200
# DEADBAND = 0.10
# KP = 0.10
# MAX_SPEED = 0.08  # m/s

# # Downward-facing camera: verify directions with props removed.
# RIGHT_SIGN = 1
# FORWARD_SIGN = -1


# def main():
#     parser = argparse.ArgumentParser()
#     parser.add_argument(
#         "--send",
#         action="store_true",
#         help="Send horizontal velocity commands"
#     )
#     args = parser.parse_args()

#     vehicle = None
#     camera = None
#     last_heartbeat = 0
#     latest_heartbeat = None

#     try:
#         if args.send:
#             vehicle = mavutil.mavlink_connection(MAVLINK)
#             latest_heartbeat = vehicle.wait_heartbeat(timeout=10)
#             last_heartbeat = time.monotonic()
#             print("MAVLink heartbeat received.")

#         camera = Picamera2()
#         camera.configure(
#             camera.create_video_configuration(
#                 main={"size": (640, 480), "format": "RGB888"}
#             )
#         )
#         camera.start()
#         time.sleep(2)

#         while True:
#             frame = camera.capture_array()
#             height, width = frame.shape[:2]
#             center_x, center_y = width // 2, height // 2

#             hsv = cv2.cvtColor(frame, cv2.COLOR_RGB2HSV)
#             mask = cv2.inRange(hsv, LOWER_GREEN, UPPER_GREEN)
#             mask = cv2.morphologyEx(
#                 mask, cv2.MORPH_OPEN,
#                 cv2.getStructuringElement(cv2.MORPH_RECT, (5, 5))
#             )

#             contours, _ = cv2.findContours(
#                 mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE
#             )

#             forward = 0.0
#             right = 0.0
#             detected = False
#             status = "BANNER NOT FOUND"

#             if contours:
#                 contour = max(contours, key=cv2.contourArea)

#                 if cv2.contourArea(contour) >= MIN_AREA:
#                     x, y, w, h = cv2.boundingRect(contour)
#                     bx, by = x + w // 2, y + h // 2

#                     # Normalized error: -1 to +1
#                     ex = (bx - center_x) / (width / 2)
#                     ey = (by - center_y) / (height / 2)

#                     if abs(ex) > DEADBAND:
#                         right = max(
#                             -MAX_SPEED,
#                             min(MAX_SPEED, RIGHT_SIGN * KP * ex)
#                         )

#                     if abs(ey) > DEADBAND:
#                         forward = max(
#                             -MAX_SPEED,
#                             min(MAX_SPEED, FORWARD_SIGN * KP * ey)
#                         )

#                     detected = True
#                     status = (
#                         f"DETECTED | X={ex:+.2f} Y={ey:+.2f} | "
#                         f"forward={forward:+.3f} right={right:+.3f}"
#                     )

#                     cv2.rectangle(
#                         frame, (x, y), (x + w, y + h), (0, 255, 0), 2
#                     )
#                     cv2.circle(frame, (bx, by), 5, (255, 0, 0), -1)

#             cv2.drawMarker(
#                 frame, (center_x, center_y), (255, 0, 0),
#                 cv2.MARKER_CROSS, 20, 2
#             )

#             if args.send:
#                 while True:
#                     hb = vehicle.recv_match(
#                         type="HEARTBEAT", blocking=False
#                     )
#                     if hb is None:
#                         break
#                     latest_heartbeat = hb
#                     last_heartbeat = time.monotonic()

#                 if time.monotonic() - last_heartbeat > 2:
#                     print("Heartbeat lost. Stopping controller.")
#                     break

#                 armed = bool(
#                     latest_heartbeat.base_mode &
#                     mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED
#                 )
#                 mode = mavutil.mode_string_v10(latest_heartbeat)

#                 if not armed or mode != "GUIDED":
#                     forward = right = 0.0
#                     status += " | BLOCKED: requires ARMED + GUIDED"

#                 if not detected:
#                     forward = right = 0.0

#                 vehicle.mav.set_position_target_local_ned_send(
#                     0,
#                     vehicle.target_system,
#                     vehicle.target_component,
#                     mavutil.mavlink.MAV_FRAME_BODY_NED,
#                     0b0000111111000111,
#                     0, 0, 0,
#                     forward, right, 0,
#                     0, 0, 0,
#                     0, 0
#                 )

#             print(status)
#             cv2.imshow("Green Banner Centering", frame)

#             if cv2.waitKey(1) & 0xFF in (ord("q"), 27):
#                 break

#     except KeyboardInterrupt:
#         print("Stopped.")
#     finally:
#         if vehicle is not None:
#             try:
#                 vehicle.mav.set_position_target_local_ned_send(
#                     0, vehicle.target_system, vehicle.target_component,
#                     mavutil.mavlink.MAV_FRAME_BODY_NED,
#                     0b0000111111000111,
#                     0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0
#                 )
#             except Exception:
#                 pass

#         if camera is not None:
#             camera.stop()

#         cv2.destroyAllWindows()


# if __name__ == "__main__":
#     main()
#!/usr/bin/env python3
"""Green-banner centering for a DOWNWARD-facing Pi camera (Raspberry Pi 5 + ArduPilot).

Modes
  python3 final_green.py                 preview only (no MAVLink, nothing sent)
  python3 final_green.py --send          also send horizontal velocity to the FC

Safety
  * Never arms, takes off, changes mode, or lands.
  * Velocity is sent only while ARMED + GUIDED and the autopilot heartbeat is fresh.
  * Banner lost / camera error -> zero velocity (hold).
  * Autopilot heartbeat lost (> 2 s) -> stops and sends zero velocity.
  * Ctrl+C / q -> zero velocity.
  * Flip the RC mode switch out of GUIDED to take control back instantly.

Direction check (props OFF, drone on the bench, camera top = drone nose):
  banner toward the NOSE  -> Y negative, forward positive
  banner toward the RIGHT -> X positive, right positive
  If wrong, use --rotate 90 / 180 / 270 until both are right.

Connection: let MAVProxy own the serial port and give this script a UDP output, e.g.
  mavproxy.py --master=/dev/ttyAMA0 --baudrate 921600 \
      --out=udp:<GCS_IP>:14550 --out=udp:127.0.0.1:14551
  python3 final_green.py --send --endpoint udpin:127.0.0.1:14551
Only one process may open the serial device. Use --endpoint /dev/ttyAMA0 only if
MAVProxy is NOT running.
"""
import argparse
import csv
import os
import sys
import time

import cv2
import numpy as np
from pymavlink import mavutil

LOWER_GREEN = (35, 60, 45)     # HSV (OpenCV: H 0-180)
UPPER_GREEN = (90, 255, 255)

HB_TIMEOUT_S = 2.0             # autopilot heartbeat older than this -> stop
CAMERA_FAIL_S = 1.0            # no good frame for this long -> stop
ALIGNED_FRAMES = 10            # consecutive centered frames to report "ALIGNED"
EMA = 0.5                      # error smoothing (1.0 = none)
PRINT_PERIOD_S = 0.2

# Downward-facing camera, image top = drone nose. Verify on the bench.
RIGHT_SIGN = 1.0
FORWARD_SIGN = -1.0

# Velocity-only type mask (ignore pos, accel, yaw, yaw_rate). Same as the ArduPilot docs.
VEL_MASK = 0b0000111111000111

ROT = {
    0: None,
    90: cv2.ROTATE_90_CLOCKWISE,
    180: cv2.ROTATE_180,
    270: cv2.ROTATE_90_COUNTERCLOCKWISE,
}


# ------------------------------------------------------------------ vision
def find_banner(frame_bgr, min_area):
    """Return (detected, ex, ey, bbox). ex/ey in -1..+1, relative to image centre."""
    h, w = frame_bgr.shape[:2]
    hsv = cv2.cvtColor(frame_bgr, cv2.COLOR_BGR2HSV)   # Picamera2 RGB888 arrays are BGR
    mask = cv2.inRange(hsv, np.array(LOWER_GREEN, np.uint8), np.array(UPPER_GREEN, np.uint8))
    kernel = cv2.getStructuringElement(cv2.MORPH_RECT, (5, 5))
    mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
    mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)
    contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    if not contours:
        return False, 0.0, 0.0, None
    c = max(contours, key=cv2.contourArea)
    if cv2.contourArea(c) < min_area:
        return False, 0.0, 0.0, None
    x, y, bw, bh = cv2.boundingRect(c)
    bx, by = x + bw // 2, y + bh // 2
    ex = (bx - w // 2) / (w / 2.0)
    ey = (by - h // 2) / (h / 2.0)
    return True, ex, ey, (x, y, bw, bh)


def compute_command(ex, ey, kp, deadband, max_speed):
    """Return (forward, right, centered). Velocities in m/s, body frame."""
    forward = right = 0.0
    if abs(ex) > deadband:
        right = RIGHT_SIGN * kp * ex
    if abs(ey) > deadband:
        forward = FORWARD_SIGN * kp * ey
    forward = max(-max_speed, min(max_speed, forward))
    right = max(-max_speed, min(max_speed, right))
    centered = abs(ex) <= deadband and abs(ey) <= deadband
    return forward, right, centered


# ------------------------------------------------------------------ mavlink
def is_autopilot_heartbeat(hb, target_system=None):
    if hb.type == mavutil.mavlink.MAV_TYPE_GCS:
        return False
    if hb.autopilot == mavutil.mavlink.MAV_AUTOPILOT_INVALID:
        return False
    if target_system is not None and hb.get_srcSystem() != target_system:
        return False
    return True


def connect(endpoint, baud, timeout_s=15.0):
    link = mavutil.mavlink_connection(
        endpoint, baud=baud, source_system=1,
        source_component=mavutil.mavlink.MAV_COMP_ID_ONBOARD_COMPUTER)
    t0 = time.monotonic()
    while time.monotonic() - t0 < timeout_s:
        hb = link.recv_match(type="HEARTBEAT", blocking=True, timeout=1)
        if hb is not None and is_autopilot_heartbeat(hb):
            link.target_system = hb.get_srcSystem()
            link.target_component = hb.get_srcComponent()
            return link, hb
    link.close()
    raise SystemExit(f"No autopilot heartbeat on {endpoint}. Check MAVProxy output / "
                     "endpoint / that only one program owns the serial port.")


def send_velocity(link, forward, right, down=0.0):
    link.mav.set_position_target_local_ned_send(
        0, link.target_system, link.target_component,
        mavutil.mavlink.MAV_FRAME_BODY_NED, VEL_MASK,
        0, 0, 0,
        float(forward), float(right), float(down),
        0, 0, 0,
        0, 0)


# ------------------------------------------------------------------ camera
def open_camera(lock_exposure):
    from picamera2 import Picamera2          # imported here so the file can be tested elsewhere
    cam = Picamera2()
    cam.configure(cam.create_video_configuration(
        main={"size": (640, 480), "format": "RGB888"}))
    cam.start()
    time.sleep(2)                            # let auto exposure / white balance settle
    if lock_exposure:
        md = cam.capture_metadata()
        cam.set_controls({
            "AeEnable": False,
            "ExposureTime": md["ExposureTime"],
            "AnalogueGain": md["AnalogueGain"],
            "AwbEnable": False,
            "ColourGains": md["ColourGains"],
        })
        print("Exposure and white balance locked.")
    return cam


# ------------------------------------------------------------------ main
def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--send", action="store_true", help="send horizontal velocity commands")
    ap.add_argument("--endpoint", default="udpin:127.0.0.1:14551")
    ap.add_argument("--baud", type=int, default=921600, help="only used for serial endpoints")
    ap.add_argument("--rotate", type=int, choices=list(ROT), default=0)
    ap.add_argument("--kp", type=float, default=0.10)
    ap.add_argument("--max-speed", type=float, default=0.08, help="m/s cap")
    ap.add_argument("--deadband", type=float, default=0.10)
    ap.add_argument("--min-area", type=int, default=1200)
    ap.add_argument("--headless", action="store_true", help="never open a window")
    ap.add_argument("--lock-exposure", action="store_true")
    ap.add_argument("--log", default=None, help="CSV log file")
    args = ap.parse_args()

    show = (not args.headless) and bool(os.environ.get("DISPLAY") or os.environ.get("WAYLAND_DISPLAY"))
    if not show:
        print("Headless: no window. Ctrl+C to stop.")

    link = None
    camera = None
    logf = None
    mode, armed, last_hb = "UNKNOWN", False, 0.0
    last_hb_sent = 0.0
    last_good_frame = time.monotonic()
    last_print = 0.0
    ex_f = ey_f = 0.0
    aligned_count = 0

    try:
        if args.send:
            link, hb = connect(args.endpoint, args.baud)
            mode = mavutil.mode_string_v10(hb)
            armed = bool(hb.base_mode & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED)
            last_hb = time.monotonic()
            print(f"MAVLink OK: system={link.target_system} mode={mode} armed={armed}")
        else:
            print("PREVIEW ONLY: nothing will be sent (use --send to enable).")

        camera = open_camera(args.lock_exposure)

        if args.log:
            logf = open(args.log, "w", newline="")
            log = csv.writer(logf)
            log.writerow(["t", "detected", "ex", "ey", "forward", "right", "mode", "armed", "sent"])

        while True:
            now = time.monotonic()

            # ---- camera
            frame = None
            try:
                frame = camera.capture_array()
                last_good_frame = now
                if ROT[args.rotate] is not None:
                    frame = cv2.rotate(frame, ROT[args.rotate])
            except Exception as exc:
                print("Camera error:", exc)
            if now - last_good_frame > CAMERA_FAIL_S:
                print("Camera lost for > 1 s. Stopping.")
                break

            # ---- vision
            detected, ex, ey, bbox = (False, 0.0, 0.0, None)
            if frame is not None:
                detected, ex, ey, bbox = find_banner(frame, args.min_area)
            if detected:
                ex_f = EMA * ex + (1 - EMA) * ex_f
                ey_f = EMA * ey + (1 - EMA) * ey_f
            else:
                ex_f = ey_f = 0.0

            forward = right = 0.0
            centered = False
            if detected:
                forward, right, centered = compute_command(
                    ex_f, ey_f, args.kp, args.deadband, args.max_speed)
            aligned_count = aligned_count + 1 if centered else 0
            aligned = aligned_count >= ALIGNED_FRAMES

            # ---- mavlink
            sent = False
            blocked = ""
            if args.send:
                if now - last_hb_sent >= 1.0:
                    link.mav.heartbeat_send(
                        mavutil.mavlink.MAV_TYPE_ONBOARD_CONTROLLER,
                        mavutil.mavlink.MAV_AUTOPILOT_INVALID, 0, 0, 0)
                    last_hb_sent = now
                while True:
                    hb = link.recv_match(type="HEARTBEAT", blocking=False)
                    if hb is None:
                        break
                    if not is_autopilot_heartbeat(hb, link.target_system):
                        continue
                    mode = mavutil.mode_string_v10(hb)
                    armed = bool(hb.base_mode & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED)
                    last_hb = time.monotonic()
                if time.monotonic() - last_hb > HB_TIMEOUT_S:
                    print("Autopilot heartbeat lost. Stopping.")
                    break
                if armed and mode == "GUIDED":
                    send_velocity(link, forward if detected else 0.0, right if detected else 0.0)
                    sent = True
                else:
                    blocked = f" | BLOCKED (needs ARMED + GUIDED, now {mode} armed={armed})"

            # ---- output
            if now - last_print >= PRINT_PERIOD_S:
                last_print = now
                if detected:
                    state = "ALIGNED" if aligned else ("CENTERED" if centered else "CORRECTING")
                    print(f"{state:10s} X={ex_f:+.2f} Y={ey_f:+.2f} | "
                          f"forward={forward:+.3f} right={right:+.3f} m/s{blocked}")
                else:
                    print(f"BANNER NOT FOUND (holding){blocked}")
            if logf:
                log.writerow([f"{time.time():.2f}", detected, f"{ex_f:+.3f}", f"{ey_f:+.3f}",
                              f"{forward:+.3f}", f"{right:+.3f}", mode, armed, sent])

            if show and frame is not None:
                h, w = frame.shape[:2]
                cv2.drawMarker(frame, (w // 2, h // 2), (255, 0, 0), cv2.MARKER_CROSS, 20, 2)
                if bbox:
                    x, y, bw, bh = bbox
                    cv2.rectangle(frame, (x, y), (x + bw, y + bh), (0, 255, 0), 2)
                    cv2.circle(frame, (x + bw // 2, y + bh // 2), 5, (0, 0, 255), -1)
                cv2.putText(frame, f"fwd={forward:+.2f} right={right:+.2f} "
                                   f"{'SENT' if sent else 'PREVIEW'}",
                            (10, 25), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 255), 2)
                cv2.imshow("Green Banner Centering", frame)
                if cv2.waitKey(1) & 0xFF in (ord("q"), 27):
                    break

    except KeyboardInterrupt:
        print("Stopped.")
    finally:
        if link is not None:
            try:
                send_velocity(link, 0.0, 0.0)
            except Exception:
                pass
        if camera is not None:
            try:
                camera.stop()
            except Exception:
                pass
        if logf:
            logf.close()
        if show:
            cv2.destroyAllWindows()
        print("Done. Zero velocity sent." if link is not None else "Done.")


if __name__ == "__main__":
    main()