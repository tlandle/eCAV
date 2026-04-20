# Supported Models and Baselines

eCAV supports multiple cooperative perception, prediction, and communication models for closed-loop evaluation in CARLA. Each model uses the **original authors' code and pretrained weights** for faithful reproduction.

## Cooperative Perception

### Supported Models
- [x] [CMP (CoBEVT + MTR) [RA-L2025]](https://arxiv.org/abs/2403.17916) - Cooperative perception and prediction
- [x] [BM2CP [CoRL2023]](https://arxiv.org/abs/2310.14702) - Multi-modal intermediate fusion
- [x] [AutoCast [MobiSys2022]](https://arxiv.org/abs/2112.14947) - Early fusion with MCKP scheduling
- [x] Late Fusion (YOLO + LiDAR) - Detected object sharing
- [x] [WorldFusion](https://github.com/) - BEV feature fusion (eCAV)
- [x] Oracle (ground truth) - Upper bound reference

### Planned
- [ ] [Where2comm [NeurIPS2022]](https://arxiv.org/abs/2209.12836) - Spatial confidence maps
- [ ] [V2X-ViT [ECCV2022]](https://arxiv.org/abs/2203.10638) - Vision Transformer V2X fusion
- [ ] [EMP [MobiCom2021]](https://dl.acm.org/doi/10.1145/3447993.3483242) - Edge-assisted multi-vehicle perception

### OPV2V 3D Detection Benchmark

| Method | Backbone | Fusion | Compression | Payload | AP@0.5 | AP@0.7 | Pretrained |
|--------|----------|--------|-------------|---------|--------|--------|------------|
| No Cooperation ([SinBEVT](https://arxiv.org/abs/2403.17916)) | PointPillar | None | N/A | 0 | 0.79 | 0.65 | [weights](ecav/core/application/v2v/baselines/cmp/CMP/pretrained/opv2v/point_pillar_sinbevt/) |
| [V2VNet](https://arxiv.org/abs/2004.00371) | PointPillar | Intermediate | None | 82.5 MB/s | 0.83 | 0.66 | [weights](ecav/core/application/v2v/baselines/cmp/CMP/pretrained/opv2v/point_pillar_v2vnet_multiego/) |
| [CoBEVT/CMP](https://arxiv.org/abs/2403.17916) | PointPillar | Intermediate | None | 82.5 MB/s | **0.93** | 0.81 | [weights](ecav/core/application/v2v/baselines/cmp/CMP/pretrained/opv2v/corpbevtlidar_delay_1_frame_aug/) |
| [CoBEVT/CMP](https://arxiv.org/abs/2403.17916) | PointPillar | Intermediate | **256x** | **0.32 MB/s** | 0.92 | **0.82** | [weights](ecav/core/application/v2v/baselines/cmp/CMP/pretrained/opv2v/corpbevtlidar_delay_1_frame_aug_c256/) |
| [BM2CP](https://arxiv.org/abs/2310.14702) | PointPillar | Intermediate | None | ~200KB/msg | - | - | [weights](ecav/worldfusion/opencood/logs/v2xsim_bm2cp_ego_baseline_2026_01_02_20_01_42/) |

*100ms communication delay. Results on OPV2V test set.*

### OPV2V Multi-Object Tracking

| Method | Compression | AMOTA | MOTA | IDS |
|--------|-------------|-------|------|-----|
| No Cooperation | N/A | 20.19 | 44.87 | - |
| CoBEVT/CMP | None | **38.76** | **62.02** | - |
| CoBEVT/CMP | 256x | 37.98 | 60.76 | 3 |

### OPV2V Cooperative Motion Prediction

| Method | Cooperation | minADE@1s | minADE@3s | minADE@5s | minFDE@5s | Pretrained |
|--------|-------------|-----------|-----------|-----------|-----------|------------|
| SinBEVT | None | 0.4099 | 1.1573 | 2.2217 | 5.1853 | - |
| V2VNet | Perception + Prediction | 0.4065 | 1.1127 | 2.1174 | 4.9037 | [weights](ecav/core/application/v2v/baselines/cmp/CMP/MTR/output/opv2v_multiego_v2vnet/) |
| CMP | Perception only | 0.3543 | 1.0222 | 1.9616 | 4.5383 | - |
| **CMP** | **Perception + Prediction** | **0.3429** | **0.9785** | **1.8578** | **4.1628** | [weights](ecav/core/application/v2v/baselines/cmp/CMP/MTR/output/opv2v_multiego_cobevt_c256/) |

## Prediction Models

- [x] Linear Predictor - Constant velocity extrapolation
- [x] AB3DMOT - Kalman filter tracking ([Weng et al.](https://arxiv.org/abs/2008.08063))
- [x] MTR - Motion Transformer ([Shi et al., NeurIPS2022](https://arxiv.org/abs/2209.13508)) - CMP baseline
- [ ] SMART - Discrete tokenized prediction (edge pipeline, training in progress)

## Perception Backends

Set `backend` in the vehicle/RSU perception YAML config:

| Backend | Model | Sensors | Config Key |
|---------|-------|---------|------------|
| `default` | YOLO + LiDAR frustum | 4 cameras + LiDAR | `backend: default` |
| `bm2cp` | PointPillar + BM2CP | LiDAR + 4 cameras | `backend: bm2cp` |
| `cobevt` | PointPillar + CoBEVT 256x | LiDAR only | `backend: cobevt` |
| `worldfusion` | PointPillar + WorldFusion | LiDAR + 4 cameras | `backend: worldfusion` |

## Network Models

- [x] **ns-3 NR V2X Mode 2** - Full 5G-LENA sidelink MAC simulation via shared-memory co-sim
- [x] **Analytical SB-SPS** - Python fallback (WINNER+ B1 + SB-SPS collision model)
- [x] **SEE-V2X Hybrid** - C-V2X trace-driven latency (213K real samples, 3 intersections)

| Engine | Interface | Scheduling | Config |
|--------|-----------|------------|--------|
| ns-3 5G-LENA | V2V sidelink | NR Mode 2 (distributed) | `network_engine: ns3` |
| ns-3 5G-LENA | Uu uplink | gNB-scheduled (centralized) | `network_engine: ns3, mode: uu_uplink` |
| Analytical | V2V sidelink | SB-SPS model | `network_engine: analytical` |

## Datasets

| Dataset | Scenarios | Agents | Usage | Source |
|---------|-----------|--------|-------|--------|
| [OPV2V](https://arxiv.org/abs/2109.07644) | 43 train / 16 test | 2-7 CAVs | CMP, CoBEVT, V2VNet training/eval | [UCLA Mobility Lab](https://mobility-lab.seas.ucla.edu/opv2v/) |
| [V2V4Real](https://arxiv.org/abs/2312.09329) | Real-world | 2 CAVs | CMP real-world eval | [UCLA Mobility Lab](https://mobility-lab.seas.ucla.edu/v2v4real/) |
| [V2X-Sim 2.0](https://arxiv.org/abs/2202.08449) | Multi-agent sim | 2-5 CAVs | BM2CP, WorldFusion training | [AI4CE Lab](https://ai4ce.github.io/V2X-Sim/) |
| [SEE-V2X](https://github.com/) | 3 intersections | C-V2X traces | Latency modeling (213K samples) | Bundled |

## Quick Start

```bash
# V2V with CMP baseline (4 cooperative vehicles)
python test_runner.py --scenario-yaml \
  ecav/scenario_testing/config_yaml/openscenario_3_v2v_cmp_4ego.yaml

# V2V with BM2CP intermediate fusion (4 vehicles)
python test_runner.py --scenario-yaml \
  ecav/scenario_testing/config_yaml/openscenario_3_v2v_intermediate_4ego.yaml

# Edge-assisted late fusion (4 vehicles)
python test_runner.py --scenario-yaml \
  ecav/scenario_testing/config_yaml/openscenario_3_edge_late_fusion_4ego.yaml

# N-sweep characterization (all fusion levels)
python -m ecav.scenario_testing.v2v_n_sweep \
  --fusion-level all --n-values 4 8 16 \
  --output-dir evaluation_outputs/v2v_n_sweep/
```
