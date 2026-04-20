# Edge Pipeline Scaling: Methodology

## Overview

This document describes the methodology for characterizing how the edge cooperative perception pipeline (WorldFusion + AB3DMOT + SMART) scales with the number of cooperative agents N. We combine three categories of data: real measurements from simulation runs, GPU-timed synthetic benchmarks through actual model weights, and analytical models for network behavior.

## Data Categories

### Category 1: Real Measurements (Ground Truth)

Source: `experiment_results/` simulation runs on the eCAV simulator with CARLA.

| N egos | Detections/frame | Tracking time (ms) | Source |
|--------|-----------------|-------------------|--------|
| 1 | 5.8 | 7.3 | 47 runs, simulation_metrics.json |
| 2 | 6.7 | 7.5 | 4 runs |
| 4 | 14.9 | 20.1 | 2 runs |
| 8 | 26.1 | 34.4 | 10 runs |
| 16 | 65.0 | 91.4 | 4 runs |

These are end-to-end measurements from live CARLA simulations running the full perception-tracking-prediction pipeline. Detection counts include ego vehicles, other traffic, and parked vehicles visible in the scene.

### Category 2: Synthetic GPU Benchmarks (Extrapolated Input, Real Compute)

For N > 16 (and for controlled comparison at all N), we construct synthetic inputs and run them through the actual trained model weights on GPU. This measures real inference time for inputs at scales we have not run in the full simulator.

**WorldFusion fusion benchmark:**
- Synthetic `spatial_features` tensors of shape `[N, 64, 200, 200]` (matching V2XSim voxel grid)
- Identity pairwise transform matrices (all agents at world origin, worst case for attention)
- Runs through: per-agent backbone, per-agent detection heads, Where2Comm attention fusion, final detection heads
- GPU: NVIDIA GeForce RTX (16 GB), measured with `torch.cuda.synchronize()` barriers
- 3 warmup iterations discarded, 10 timed iterations averaged

**AB3DMOT tracking benchmark:**
- Synthetic 3D bounding box detections: `n_dets = 4 * N` (calibrated from real data: ~4 detections per ego vehicle)
- Detections distributed in a circle with small per-frame motion to simulate moving vehicles
- 9 frames of warmup to establish tracks (min_hits=3), 10th frame timed
- CPU-only (AB3DMOT uses NumPy, not GPU)

**SMART prediction benchmark:**
- Analytical model calibrated to 0.3 ms observed at N~6 tracks
- Decomposed into: O(N * 2048) tokenization (matching against codebook) + O(N^2) graph attention
- If SMART model loads successfully, replaced with real GPU timing
- Otherwise analytical scaling used with calibration point noted

**What this captures:** Real compute time for each component at input sizes corresponding to N agents. The synthetic inputs have the correct tensor shapes and reasonable value distributions but do not capture scene-specific spatial correlations.

**What this does not capture:** Data loading overhead, inter-stage serialization, GPU memory contention from concurrent stages, or CARLA simulation overhead.

### Category 3: Analytical Models (Network Layer)

**SB-SPS MAC model (V2V sidelink):**
- Standard: C-V2X PC5 Mode 4 (3GPP TS 36.321)
- PRR formula: `PRR = ((M-1)/M)^(N-1) * (1 - p_loss)`
  - M = 20 subchannels (C-V2X default)
  - p_loss = 0.02 (fading baseline)
- Payload adjustment: intermediate fusion features are ~200 KB. At 6 Mbps effective rate, this requires ceil(200000 / 750) = 267 TTIs. All TTIs must succeed, so `PRR_payload = PRR^267`.
- Channel busy ratio: `CBR = 1 - ((M-1)/M)^N`
- Source: Validated against ns-3 5G-LENA NR V2X simulations (scripts/validate_mac_vs_ns3.py)

**Edge backhaul model:**
- Radio uplink: 15 ms mean (from merged_latency.csv field measurements)
- Backhaul queuing: `(N * feature_bytes * 8) / (backhaul_bw * 1e6) * 1000 * 0.5` ms
  - 100 Mbps backhaul (conservative MEC assumption)
  - Factor 0.5: M/D/1 queue approximation

## Scene Complexity Scaling

From real simulation data, we observe approximately linear scaling of detections with ego count:

- Each ego vehicle sees itself + other egos + background traffic + parked vehicles
- Empirical fit: `detections_per_frame ~ 4.0 * N`
- Track confirmation ratio: ~70% of detections become confirmed tracks (min_hits=3 gate)
- This linear relationship holds because in the CARLA intersection scenarios, additional ego vehicles both contribute detections and appear as detectable objects to other egos

At N=64: ~256 detections/frame, ~179 confirmed tracks. This exceeds the MTR hard limit of 100 objects, which would require truncation or batching in a real deployment.

## Assumptions and Limitations

1. **Synthetic inputs are spatially uniform.** Real scenes have non-uniform object distributions (clusters near intersections). This may underestimate AB3DMOT's Hungarian matching cost for clustered detections.

2. **Where2Comm attention with identity transforms.** Using identity pairwise transforms means no spatial warping cost. Real scenes require non-trivial affine warps, adding ~1-2 ms overhead that scales with N.

3. **SMART analytical model.** The O(N^2) attention scaling is based on the model architecture (graph neural network). Actual GPU utilization may differ due to batching efficiency and memory bandwidth. Real measurements replace this if the model loads.

4. **Network model is analytical.** The SB-SPS PRR formula assumes uniform random subchannel selection. Real C-V2X has sensing-based selection that improves PRR at low N but converges to the analytical model at high N. Our ns-3 validation shows the analytical model is conservative (underestimates PRR by 2-5%).

5. **Single GPU, sequential stages.** The benchmark runs each stage sequentially on one GPU. A pipelined implementation could overlap stages, reducing total latency by up to the duration of the shortest stage.

## Reproducing

```bash
cd ecav/worldfusion
conda run -n opencda310 python ../../paper2_edge_scaling_benchmark.py
# Or skip GPU fusion benchmark:
conda run -n opencda310 python ../../paper2_edge_scaling_benchmark.py --skip-fusion
```

Outputs in `paper2_figures/`:
- `benchmark_timing.csv`: raw timing data for all N values
- `fig_edge_stage_latency.pdf`: stacked per-stage latency vs N with 100ms deadline
- `fig_v2v_prr_vs_n.pdf`: V2V PRR collapse with payload adjustment
- `fig_edge_vs_v2v_e2e.pdf`: edge vs V2V effective E2E latency
- `fig_scene_complexity.pdf`: detection and track counts vs N
