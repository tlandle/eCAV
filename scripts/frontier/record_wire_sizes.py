#!/usr/bin/env python3
"""Record sizes as the campaign lineage puts them on the wire: for each transferred-state
arm of the state frontier, the pickle blob (payload.serialize(), what [OBSTACLE_HANDOFF]
and [XFERROW] report and what the parametric link charges) and the gRPC TransferChunk that
wraps that blob on the TRANSFER_MODE=grpc path (T8 / netem blocks). Run inside the campaign
worktree with the overlay applied (scripts/diag/run_in_overlay_wt.sh), so payload/factories
are the campaign's (payload_wired.py, factories_1l.py). Does not insert the main checkout on
sys.path: the ecav package resolves through PYTHONPATH to the worktree."""
import os, sys, json
import numpy as np
from ecav.core.application.edge.migration.harness import TRAJECTORIES, _tracker_cfg
from ecav.core.application.edge.migration.factories import latent_from_tracklet
from ecav.core.application.edge.migration.payload import KFState, MigrationPayload
from ecav.core.tracking.mamba3dmot.tracker import Mamba3DTracker
import ecav.core.application.edge.migration.payload as _pl
sys.path.insert(0, "ecav/protos")
import migration_pb2 as mpb
import torch
DT = 0.05; HANDOFF = 10
device = "cuda" if torch.cuda.is_available() else "cpu"
cfg = _tracker_cfg("ecav/core/tracking/mamba3dmot/mamba3dmot_weights.pth")
traj = TRAJECTORIES["straight"](HANDOFF + 5)
edge = Mamba3DTracker(cfg, device=device)
for det in traj[:HANDOFF]: edge.update(det.reshape(1, -1), np.array([0.95], dtype=np.float32))
src = edge.tracked_tracklets[0]
m = np.asarray(src.memo_bank, dtype=np.float32); k = max(len(m) - 1, 1)
vel = np.array([(m[-1, 0] - m[0, 0]) / (k * DT), (m[-1, 1] - m[0, 1]) / (k * DT)], dtype=np.float32)
arms = {"position": (1, None, False), "pos_vel": (1, vel, False), "kalman_state": (1, vel, True), "hist2": (2, None, False), "hist5": (5, None, False), "hist10": (None, None, False)}
out = {"payload_module": os.path.relpath(_pl.__file__), "schema_version": None, "arms": {}}
for name, (depth, v, kf) in arms.items():
    lat = latent_from_tracklet(src, persistent_vehicle_id=42, last_observation_t=0.5, history_depth=depth, vel_mps=v)
    if kf:
        x = np.zeros(10, dtype=np.float32); x[:7] = m[-1]; x[7] = vel[0] * DT; x[8] = vel[1] * DT
        lat.kf_state = KFState(state_vector=x, covariance=np.eye(10, dtype=np.float32), hits=10, anchoring_age=0)
    pay = MigrationPayload(source_locale_id="A", destination_locale_id="B", trigger_time_s=HANDOFF * DT, tracks=[lat])
    blob = pay.serialize(); out["schema_version"] = getattr(pay, "schema_version", None)
    chunk = mpb.TransferChunk(payload=blob, actor_id=42, epoch=1, schema_version=int(getattr(pay, "schema_version", 1) or 1))
    declared = pay.payload_bytes() if hasattr(pay, "payload_bytes") else None
    out["arms"][name] = {"pickle_bytes": len(blob), "grpc_transferchunk_bytes": chunk.ByteSize(), "declared_payload_bytes": declared,
                         "memo_frames": int(np.asarray(lat.memo_bank).shape[0]) if lat.memo_bank is not None else 0, "has_vel": v is not None, "has_kf_state": kf}
    print(f"{name:13s} pickle {len(blob):5d} B  grpc TransferChunk {chunk.ByteSize():5d} B  declared {declared}  memo {out['arms'][name]['memo_frames']}")
json.dump(out, open(sys.argv[1] if len(sys.argv) > 1 else "/tmp/record_wire_sizes.json", "w"), indent=1); print("payload module:", out["payload_module"])
