from pymavlink import mavutil

ENDPOINTS = [
    "tcp:127.0.0.1:5760",
    "tcp:127.0.0.1:5762",
    "udpin:127.0.0.1:14550",
    "udpin:127.0.0.1:14551",
]

for ep in ENDPOINTS:
    print(f"Trying {ep} ...", end=" ", flush=True)
    try:
        link = mavutil.mavlink_connection(ep)
        hb = link.wait_heartbeat(timeout=5)
        if hb is None:
            print("no heartbeat")
        else:
            print(f"OK (system={link.target_system}, component={link.target_component})")
        link.close()
    except Exception as exc:
        print(f"FAILED: {type(exc).__name__}: {exc}")