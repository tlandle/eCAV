#!/usr/bin/env python3
"""Passive CARLA client that records every vehicle's ground-truth pose each
world tick, independent of the simulation pipeline.

    python scripts/diag/carla_actor_tap.py --out actor_tap.csv [--port 2000] [--minutes 30]

Columns: wall, frame, elapsed_s, actor_id, type, x, y, z, yaw, speed_mps.
Reconnects when the server restarts; stops after --minutes or when the file
named by --stop exists.
"""
import argparse, csv, os, sys, time
sys.path.insert(0, os.path.expanduser("~/carla-0.9.15/PythonAPI/carla"))
import carla  # noqa: E402


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--out", required=True)
    ap.add_argument("--port", type=int, default=2000)
    ap.add_argument("--minutes", type=float, default=30)
    ap.add_argument("--stop", default=None)
    a = ap.parse_args()
    t_end = time.time() + 60 * a.minutes
    fh = open(a.out, "w", newline="")
    w = csv.writer(fh)
    w.writerow(["wall", "frame", "elapsed_s", "actor_id", "type", "x", "y", "z", "yaw", "speed_mps"])
    epoch = 0
    while time.time() < t_end and not (a.stop and os.path.exists(a.stop)):
        try:
            client = carla.Client("localhost", a.port)
            client.set_timeout(10.0)
            world = client.get_world()
            epoch += 1
            print(f"connected (epoch {epoch}) map={world.get_map().name}", flush=True)
            while time.time() < t_end and not (a.stop and os.path.exists(a.stop)):
                snap = world.wait_for_tick(seconds=20.0)
                ts = snap.timestamp
                for act in world.get_actors().filter("vehicle.*"):
                    tf = act.get_transform()
                    v = act.get_velocity()
                    w.writerow([f"{time.time():.3f}", snap.frame, f"{ts.elapsed_seconds:.3f}", act.id, act.type_id,
                                f"{tf.location.x:.2f}", f"{tf.location.y:.2f}", f"{tf.location.z:.2f}",
                                f"{tf.rotation.yaw:.1f}", f"{(v.x**2 + v.y**2) ** 0.5:.2f}"])
                fh.flush()
        except (RuntimeError, OSError) as e:
            print(f"server unavailable ({e}); retry in 5 s", flush=True)
            time.sleep(5)
    fh.close()
    return 0


if __name__ == "__main__":
    sys.exit(main())
