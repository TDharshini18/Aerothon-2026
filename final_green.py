import argparse
import time

import cv2
from picamera2 import Picamera2
from pymavlink import mavutil

# Starting endpoint from your existing ToF script.
# It must actually route to the flight controller for commands to work.
MAVLINK = "udp:127.0.0.1:14550"

LOWER_GREEN = (35, 60, 45)
UPPER_GREEN = (90, 255, 255)

MIN_AREA = 1200
DEADBAND = 0.10
KP = 0.10
MAX_SPEED = 0.08  # m/s

# Downward-facing camera: verify directions with props removed.
RIGHT_SIGN = 1
FORWARD_SIGN = -1


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument(
        "--send",
        action="store_true",
        help="Send horizontal velocity commands"
    )
    args = parser.parse_args()

    vehicle = None
    camera = None
    last_heartbeat = 0
    latest_heartbeat = None

    try:
        if args.send:
            vehicle = mavutil.mavlink_connection(MAVLINK)
            latest_heartbeat = vehicle.wait_heartbeat(timeout=10)
            last_heartbeat = time.monotonic()
            print("MAVLink heartbeat received.")

        camera = Picamera2()
        camera.configure(
            camera.create_video_configuration(
                main={"size": (640, 480), "format": "RGB888"}
            )
        )
        camera.start()
        time.sleep(2)

        while True:
            frame = camera.capture_array()
            height, width = frame.shape[:2]
            center_x, center_y = width // 2, height // 2

            hsv = cv2.cvtColor(frame, cv2.COLOR_RGB2HSV)
            mask = cv2.inRange(hsv, LOWER_GREEN, UPPER_GREEN)
            mask = cv2.morphologyEx(
                mask, cv2.MORPH_OPEN,
                cv2.getStructuringElement(cv2.MORPH_RECT, (5, 5))
            )

            contours, _ = cv2.findContours(
                mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE
            )

            forward = 0.0
            right = 0.0
            detected = False
            status = "BANNER NOT FOUND"

            if contours:
                contour = max(contours, key=cv2.contourArea)

                if cv2.contourArea(contour) >= MIN_AREA:
                    x, y, w, h = cv2.boundingRect(contour)
                    bx, by = x + w // 2, y + h // 2

                    # Normalized error: -1 to +1
                    ex = (bx - center_x) / (width / 2)
                    ey = (by - center_y) / (height / 2)

                    if abs(ex) > DEADBAND:
                        right = max(
                            -MAX_SPEED,
                            min(MAX_SPEED, RIGHT_SIGN * KP * ex)
                        )

                    if abs(ey) > DEADBAND:
                        forward = max(
                            -MAX_SPEED,
                            min(MAX_SPEED, FORWARD_SIGN * KP * ey)
                        )

                    detected = True
                    status = (
                        f"DETECTED | X={ex:+.2f} Y={ey:+.2f} | "
                        f"forward={forward:+.3f} right={right:+.3f}"
                    )

                    cv2.rectangle(
                        frame, (x, y), (x + w, y + h), (0, 255, 0), 2
                    )
                    cv2.circle(frame, (bx, by), 5, (255, 0, 0), -1)

            cv2.drawMarker(
                frame, (center_x, center_y), (255, 0, 0),
                cv2.MARKER_CROSS, 20, 2
            )

            if args.send:
                while True:
                    hb = vehicle.recv_match(
                        type="HEARTBEAT", blocking=False
                    )
                    if hb is None:
                        break
                    latest_heartbeat = hb
                    last_heartbeat = time.monotonic()

                if time.monotonic() - last_heartbeat > 2:
                    print("Heartbeat lost. Stopping controller.")
                    break

                armed = bool(
                    latest_heartbeat.base_mode &
                    mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED
                )
                mode = mavutil.mode_string_v10(latest_heartbeat)

                if not armed or mode != "GUIDED":
                    forward = right = 0.0
                    status += " | BLOCKED: requires ARMED + GUIDED"

                if not detected:
                    forward = right = 0.0

                vehicle.mav.set_position_target_local_ned_send(
                    0,
                    vehicle.target_system,
                    vehicle.target_component,
                    mavutil.mavlink.MAV_FRAME_BODY_NED,
                    0b0000111111000111,
                    0, 0, 0,
                    forward, right, 0,
                    0, 0, 0,
                    0, 0
                )

            print(status)
            cv2.imshow("Green Banner Centering", frame)

            if cv2.waitKey(1) & 0xFF in (ord("q"), 27):
                break

    except KeyboardInterrupt:
        print("Stopped.")
    finally:
        if vehicle is not None:
            try:
                vehicle.mav.set_position_target_local_ned_send(
                    0, vehicle.target_system, vehicle.target_component,
                    mavutil.mavlink.MAV_FRAME_BODY_NED,
                    0b0000111111000111,
                    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0
                )
            except Exception:
                pass

        if camera is not None:
            camera.stop()

        cv2.destroyAllWindows()


if __name__ == "__main__":
    main()