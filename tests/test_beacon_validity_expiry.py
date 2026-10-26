"""ValidBeacon with the j4 thresholds: a hard-braking CAV's beacon (12 m/s2) is accepted under the
15 m/s2 bound; a beacon that jumps 4 m off the predicted position is rejected by the displacement
gate; after the 1 s recency bound the record expires and the next beacon re-bootstraps instead of
being rejected again (no cascade). Runs standalone."""
import os
import sys
sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
import numpy as np  # noqa: E402
from ecav.core.application.edge.beacon_id_manager import BeaconIdManager  # noqa: E402


def main():
    m = BeaconIdManager(validity_checks=True, world_dt=0.05)
    cid = 7
    # bootstrap at tick 0, 13 m/s along +x
    ok, why = m.validate_beacon(cid, np.array([0.0, 0.0, 0.0]), 0, yaw=0.0, velocity=np.array([13.0, 0.0, 0.0]))
    assert ok and why == "bootstrap", why
    m.get_temp_id(cid, np.array([0.0, 0.0, 0.0]), 0, velocity=np.array([13.0, 0.0, 0.0]))
    # one tick later, braking at 12 m/s2: v = 12.4 m/s, moved 0.65 m
    ok, why = m.validate_beacon(cid, np.array([0.65, 0.0, 0.0]), 1, yaw=0.0, velocity=np.array([12.4, 0.0, 0.0]))
    assert ok, ("12 m/s2 braking must pass the 15 m/s2 bound", why)
    m.get_temp_id(cid, np.array([0.65, 0.0, 0.0]), 1, velocity=np.array([12.4, 0.0, 0.0]))
    # a 6 m jump off the predicted position is rejected by the displacement gate (5 m floor)
    ok, why = m.validate_beacon(cid, np.array([1.3 + 6.0, 0.0, 0.0]), 2, yaw=0.0, velocity=np.array([12.0, 0.0, 0.0]))
    assert (not ok) and why == "displacement", why
    # 21 ticks after the last accepted beacon the record has expired: re-bootstrap, not another rejection
    ok, why = m.validate_beacon(cid, np.array([30.0, 0.0, 0.0]), 23, yaw=0.0, velocity=np.array([12.0, 0.0, 0.0]))
    assert ok and why == "bootstrap", ("expired record must re-bootstrap", ok, why)
    print("ok")


if __name__ == "__main__":
    main()
