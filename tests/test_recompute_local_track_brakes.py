"""A brake on the CAV's own local forecast (0.5 s horizon) is counted apart and
enters neither S_ghost nor S_fp; brakes on shared (edge, 3 s) tracks still do."""
import os
import sys

sys.path.insert(0, os.path.join(os.path.dirname(os.path.dirname(os.path.abspath(__file__))), "scripts"))
from recompute_metrics import compute_run_metrics, _is_local_track  # noqa: E402


def _attr(tick, track, horizon, cls, matched, obs, ego=(0.0, 10.0)):
    return {"trigger_tick": tick, "trigger_track_id": track, "prediction_horizon_s": horizon,
            "gt_brake_class": cls, "gt_matched_actor_id": matched, "gt_provenance": "x",
            "obs_x": obs[0], "obs_y": obs[1], "ego_x": ego[0], "ego_y": ego[1],
            "delta_use_ticks": 0 if horizon < 1.0 else 6, "is_ego_ghost": cls == "self_ghost",
            "obstacle_speed": 12.0, "ttc": 0.3}


def _run(attrs):
    return {"vehicles": {"109": {"safety": {"collision_count": 0, "stuck": False},
                                 "brake_attributions": attrs, "avg_speed_mps": 10.0}},
            "edges": {"e": {"total_frames": 250}}, "config_max_step": 300,
            "config_manager_type": "late_fusion", "config_anchoring": True, "config_latency_ms": 300}


def test_local_track_flag():
    assert _is_local_track({"prediction_horizon_s": 0.5})
    assert not _is_local_track({"prediction_horizon_s": 3.0})
    assert not _is_local_track({})


def test_local_brakes_do_not_fail_the_run():
    # the parked-truck column's near face, tracked on board at the ego's speed, 2.5 m beside it
    attrs = [_attr(t, 42, 0.5, "other_fp", 113, (-2.5, 10.0)) for t in (184, 185, 186)]
    # a local track the run-time labeller tagged self_ghost (in the corridor)
    attrs.append(_attr(200, 50, 0.5, "self_ghost", 109, (0.0, 8.5)))
    m = compute_run_metrics(_run(attrs))
    assert m["s_fp"] == 1 and m["s_ghost"] == 1 and m["s_op"] == 1
    assert m["focal_local_episodes"] == 2 and m["focal_local_ticks"] == 4
    assert m["focal_fp_episodes"] == 0 and m["focal_ghost_episodes"] == 0


def test_shared_self_ghost_still_fails():
    attrs = [_attr(t, 42, 0.5, "other_fp", 113, (-2.5, 10.0)) for t in (184, 185)]
    # an anonymous edge track 2 m behind the ego in its corridor: a self-ghost of the shared state
    attrs += [_attr(t, 89, 3.0, "self_ghost", 109, (0.0, 8.0)) for t in (280, 281, 282)]
    m = compute_run_metrics(_run(attrs))
    assert m["s_ghost"] == 0 and m["s_op"] == 0
    assert m["focal_ghost_episodes"] == 1 and m["focal_local_episodes"] == 1 and m["s_fp"] == 1


def test_shared_false_brake_still_fails():
    attrs = [_attr(98, 33, 3.0, "other_fp", 115, (3.5, 12.0))]
    m = compute_run_metrics(_run(attrs))
    assert m["s_fp"] == 0 and m["s_op"] == 0 and m["focal_local_episodes"] == 0


def main():
    for name, fn in sorted(globals().items()):
        if name.startswith("test_"):
            fn()
            print("ok", name)


if __name__ == "__main__":
    main()
