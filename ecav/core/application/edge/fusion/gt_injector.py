"""Ground-truth detection injection for closed-loop evaluation.

Bypasses the perception model output and feeds simulator GT actor poses
into the AB3DMOT-format detection stream the edge manager downstream
consumes. Used for paper closed-loop runs where we want to isolate the
controller/scheduler under realistic AoI without the perception backbone
as a confound. The model is still run on each tick (its compute time and
ns-3 payload bytes contribute to AoI) — only the output boxes are
replaced with GT.

Output format (KITTI-swap convention, matching `late_fusion_backend`,
`IntermediateFusionBackend._to_ab3dmot_format`, and the WorldFusion edge
manager's now-fixed `_to_ab3dmot_format`):
  dets:   (N, 8) float32 with columns [h, w, l, x_world, height, y_world, yaw, score].
          Col 4 is the actor's CARLA z (height) and col 5 is CARLA y (ground
          axis 2). This matches AB3DMOT's KITTI-camera assumption where
          `Box3D.array2bbox_raw` maps col 4 -> bbox.y (vertical) and col 5
          -> bbox.z (forward). NMS / dup-detection use (bbox.x, bbox.z) for
          ground-plane distance, which under this convention is correctly
          (world_x, world_y).
  info:   (N, 3) int64 with col 0 = frame_idx, col 1 = det_index (GUID),
          col 2 = CARLA actor id (CID). Setting col 2 to the real actor id
          is critical: AB3DMOT propagates it as `trk.carla_id`, and the
          downstream behavior agent uses `carla_id != -1` to distinguish
          beacon-identified tracks from anonymous (model-derived) ones.
          With CID=-1, every track gets flagged as a "potential ghost"
          and the planner discards the predictions.
  scores: (N,)   float32, all 1.0 for GT.
  frame:  int    frame_idx (the downstream self-detection filter expects this).
"""
from __future__ import annotations

from typing import Iterable, Sequence
import math
import numpy as np


def build_gt_dets(carla_world,
                  world_anchor: Sequence[float],
                  frame_idx: int,
                  exclude_actor_ids: Iterable[int] = (),
                  max_range_m: float = 70.0) -> dict:
    """Build a GT detection batch in the same shape as the model output.

    Args:
        carla_world: live `carla.World` handle.
        world_anchor: [x, y, z, roll, yaw_deg, pitch] — the edge's anchor
            pose. yaw_deg is used to convert actor world yaw → anchor-
            frame yaw to match the post_process output convention.
        frame_idx: tick index, written into info col 0.
        exclude_actor_ids: CARLA actor IDs to skip (typically the ego
            CAVs being managed by this edge).
        max_range_m: skip actors further than this from the anchor xy.

    Returns:
        dict with 'dets' (N, 8) float32 and 'info' (N, 3) int64.
    """
    excl = set(int(i) for i in exclude_actor_ids)
    anchor_x, anchor_y, anchor_z = float(world_anchor[0]), float(world_anchor[1]), float(world_anchor[2])
    anchor_yaw_rad = math.radians(float(world_anchor[4]))

    rows = []
    actor_ids = []
    actor_types = []
    for actor in carla_world.get_actors().filter('vehicle.*'):
        if int(actor.id) in excl:
            continue
        tf = actor.get_transform()
        loc = tf.location
        dx = loc.x - anchor_x
        dy = loc.y - anchor_y
        rng = math.hypot(dx, dy)
        if rng > max_range_m:
            continue
        actor_types.append(actor.type_id)

        ext = actor.bounding_box.extent  # half-extents (CARLA convention)
        l_full = 2.0 * ext.x
        w_full = 2.0 * ext.y
        h_full = 2.0 * ext.z

        actor_yaw_rad = math.radians(tf.rotation.yaw)
        # Match post-process output: yaw in anchor frame
        yaw_anchor_rad = actor_yaw_rad - anchor_yaw_rad

        # 8-col layout: KITTI-swap convention so AB3DMOT NMS/dup-detection
        # treat (col 3, col 5) = (world_x, world_y) as the ground plane.
        #   [h, w, l, x_world, height, y_world, yaw, score]
        rows.append((h_full, w_full, l_full,
                     loc.x, loc.z, loc.y,
                     yaw_anchor_rad, 1.0))
        actor_ids.append(int(actor.id))

    if not rows:
        return {
            'dets':   np.empty((0, 8), dtype=np.float32),
            'info':   np.empty((0, 3), dtype=np.int64),
            'scores': np.empty(0, dtype=np.float32),
            'frame':  frame_idx,
        }

    dets = np.array(rows, dtype=np.float32)
    n = len(rows)
    info = np.empty((n, 3), dtype=np.int64)
    info[:, 0] = frame_idx
    info[:, 1] = np.arange(n)
    info[:, 2] = np.array(actor_ids, dtype=np.int64)
    scores = np.ones(n, dtype=np.float32)

    # Debug: per-tick pose of every Tesla in the GT injection set. The
    # cross-traffic Tesla in scenario_3 starts near (-35, 127.7) and moves
    # in -x. CARLA actor ids change across runs, so match by type.
    for i, atype in enumerate(actor_types):
        if 'tesla' in atype.lower():
            print(f"[GT INJECT DBG] frame={frame_idx} aid={actor_ids[i]} "
                  f"type={atype} world=({rows[i][3]:.2f},{rows[i][5]:.2f}) "
                  f"yaw_anchor_rad={rows[i][6]:.3f}")

    return {'dets': dets, 'info': info, 'scores': scores, 'frame': frame_idx}
