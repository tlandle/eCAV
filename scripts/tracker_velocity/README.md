Offline replay of recorded detections through the tracker's velocity rules (2026-11-12).

    det_parse.py <run_dir> <tick_lo> <tick_hi> <out.csv>   [DET] lines of the conflict actor against ground truth
    kf_replay.py <dets.csv> first|mean|front <lo> <hi> <lane_change_tick>      Kalman tuning variants
    edge_replay.py <dets.csv> <lo> <hi> <lane_change_tick>                     published velocity from box edges (current rule) and a rigid-motion rule
    centre_replay.py <dets.csv> <lo> <hi> <lane_change_tick>                   least-squares slope of the box centre, window variants
    hm_brk.py <stamp> <manager> <anchoring>      highway merge: first held brake against the start of the lane change
    trk_speed.py <run_dir>                       published speed of the conflict actor against ground truth

Run from the repository root. Inputs are runner_stdout.txt and evaluation_output/per_tick_age.csv of existing runs.
