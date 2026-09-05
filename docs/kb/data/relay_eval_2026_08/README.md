# relay_eval_2026_08: Khonsu closed-loop data

One CSV row per run. Binary outcome fields: `collided` (any ego contact), `completed`. Report collided-or-not and completed-without-collision only; never mean episodes.

| File | What | Harness (develop) | Status |
|---|---|---|---|
| design_sweep_v3_rows.csv | six flow arms x 10 + burst arms x 5 on the corrected scenario (ONCOMING_SPEED=12, TRIGGER_DIST=300 as runner defaults) | scripts/khonsu_rebaseline_arms.sh | citable re-baseline |
| q5_scale_rows.csv | oncoming density N=2/4/8 (FLOW_N releases the first N of 8 parked oncoming vehicles at scenario start; all N cross the boundary; unconnected actors, one migration per crossing), arms cold / edgewarp (at-crossing one-frame snapshot, paper name "handover snapshot") / warm (forecast trigger 1 s, full history), 5 runs per cell | scripts/khonsu_q5_scale.sh; FLOW_N knob in ecav/scenario_testing/scenarios/scenario_1.py | citable (collided field); completion-field format under confirmation |
| design_sweep_v1_rows.csv | 150-run uniform design sweep on the unvalidated geometry | scripts/khonsu_design_sweep.sh | relative findings only, not citable absolutely |
| fault_injection_results.csv | eight ownership-epoch scenarios through the per-track state machine | unit harness | citable |
| canvas_latency.csv | fusion latency and memory vs BEV canvas side, 8 contributors | see current_state.md 2026-08-31 | citable (Fig. 1) |
| q3_lookahead_sweep.csv, q7_locale_sizing.csv | August pilots | superseded by s5 lookahead redo (scripts/khonsu_s5_lookahead_pinned.sh, worktree khonsu_v1_wt at 9d5e1883) and the sizing study | do not cite |

Retired: q4_flow_table.csv (canonical warm arm reproduces 4/4 under the env contract, but the reactive and edgewarp arms never transferred). Provenance history in docs/kb/wiki/current_state.md (2026-08-31, 2026-09-03, 2026-09-04) and raw/sessions.

## Radio plane (platform note, 2026-09-05)
The default flow/accel runners use the ANALYTICAL C-V2X plane: SEE-V2X
measured latency traces (HybridModel: real C-V2X RTT + backhaul lognormal +
base_ms) plus SB-SPS PC5 Mode-4 contention (SbSpsMac, M=20 resources). This
plane governs the UPLINK only (sensor -> edge, via the jitter buffer). The
edge -> ego forecast DOWNLINK is instantaneous, gated only by
downlink_packet_loss_pct (no delay stamp). Age at use is therefore carried by
uplink staleness. Measured SEE-V2X RTT p50/p95: L 11.8/22.3, M 12.3/23.3,
H 18.7/23.8 ms. The 5G-LENA ns-3 co-sim (ecav/core/networking/ns3_cosim) is a
separate, unwired path. The paper's platform sentence must match whichever
T12 age source Tyler selects.

## Visible-row tag justification (2026-09-05)
5.3 visible cells (flow_visible, accel_visible) are labeled eval_tag=
freeze-1c+vis, NOT re-run under a single tag with the occluded cells. Justified:
the visible scenario files (scenario_1_flow_visible.xml, scenario_1_accel_
visible.xml + their configs/runners) are ADDITIVE - new files added after
freeze-1c. The occluded-cell arms never reference them (they load
scenario_1_flow.xml / scenario_1_accel.xml), so occluded-arm behavior is
byte-identical with or without the visible files present. The 2x2 figure
therefore pools freeze-1c (occluded) + freeze-1c+vis (visible) rows validly.

## CORRECTION to the radio-plane note (2026-09-05)
The earlier "downlink instantaneous, loss only" note was WRONG for the flow
runner. It described the BASE pluggable manager (_advance_vehicles). The flow
runner's manager (WorldFusionAdaptiveEdge -> WorldFusionEdge, the
linear_predictor) has use_ns3_lut DEFAULT TRUE and applies BOTH uplink and
downlink ns-3 LUT latency: UL sampled per-CAV (max over N) into the jitter
buffer, DL sampled (n_cav, dl_bytes) plus compute_ms into an outbound queue
delivered at deliver_tick. So realized age at use is already ns-3-LUT-derived
(ns3_uplink_lut.csv / ns3_downlink_lut.csv, N in {4,8,16,24,31} x payload,
bilinear interp). ns-3 IS part of this platform. LUT ranges: UL p50 9.6-37.8,
DL p50 5.3-304, DL p95 up to 952 ms at N=31 / 16.9KB. T12 (freeze-1e) sweeps
NS3_LUT_N over the LUT range; MAC_BG_SENDERS dropped. The paper's platform
sentence: ns-3-derived C-V2X Uu latency (payload/N-aware LUT), UL+DL.

## AGEROW availability + adaptive-lineage LUT (2026-09-05, restart-4 rescinded)
CONFIRMED (peer + code): edge_manager_worldfusion_ab3dmot_mtr_adaptive.run_step
calls super().run_step at line 171; neither the adaptive nor the mamba
subclass overrides delivery/latency, so WorldFusionEdge's ns-3 LUT sampling
(UL at ingest, DL into the latest-wins outbound queue) is LIVE in every
Khonsu run including frozen1c. The adaptive lineage was created after the
Apr-30 LUT commit but inherits it; the earlier "grep per file missed
inheritance" scare (restart 4) was wrong and RESCINDED - no code change, batch
not stopped (beyond an accidental kill I immediately resumed).
frozen1c rows: NO AGEROW (realized age at use not extractable for the headline
batch); the extractable freshness fields are HANDOFFROW warm_before_first_use
and bytes. AGEROW (realized age = delivery_tick - source_frame_tick, ms) is
added at freeze-1e (outbound drain) and carried by T12 and all later tags.

## Medium mismatch: inter-locale transfer accounted as radio (2026-09-05)
frozen1c/1e: InterLocaleLink(edge.latency_model) sampled the transfer's
network cost from the C-V2X RADIO model, but inter-locale transfer is WIRED
backhaul between edge servers. SCOPE: accounting only (link.py never delays
the mechanism) + the computed-trigger EMA seed (40ms was a radio number).
The LEADROW/XFERROW transfer times and computed-lead EMA are affected; the
lead EFFECT is <0.05s, and warm/predictive/mtr/oracle arms do not use the
EMA, so frozen1c is NOT invalidated (Tyler's ruling). FIX (develop, for
freeze-2/T8/later): InterLocaleLink wired model = base 2ms + payload/1Gbps +
0.5x queueing; computed EMA reseeded to ~2ms. TRANSFER_MEDIUM=wired default.
NETEM matrix HELD until citable 5G-MOBIX wired-backhaul figures arrive.

## Wired backhaul figures: 5G-MOBIX D5.2 (2026-09-05)
Source: 5G-MOBIX D5.2 v3.0 "Report on technical evaluation" section 4.5.4,
CS_14 inter-MEC exchange. Table 30 (MEC-to-MEC over fibre, 100km ES-PT
corridor): one-way network latency avg 1.8-3.6 ms, median 1.6-3.6, p95
2.4-3.7, max 3.6-5.2, stdev 0.1-0.8 ms; 11km NL federation added no
measurable delay; executive-summary bullet: direct inter-MEC over leased line
across operators adds 15-20 ms per hop; wired packet loss zero (TCP).
InterLocaleLink wired model uses base 3 ms + 0.5 ms jitter (stdev) +
serialization + payload/1Gbps + 0.5x queue; computed-trigger EMA reseeded
~3 ms. netem matrix (Table 31 equivalent): (delay,jitter) = (0,0),(3,0.5),
(20,1),(50,2) ms x loss {0,0.1,1}%. 3ms=fibre op-point, 20ms=leased-line
hop (both MEASURED); 50ms and 1% loss = stress beyond measurement (labeled
_stress in row tags); 0.1% = nominal wired loss.

## Age-at-use floor is a result (2026-09-05)
Delta_use (t_c - t_o, total age of the consumed forecast's newest observation
at use) has a MINIMUM of ~300 ms at the lightest load, quantized in ~150 ms
steps, set by the edge cadence (edge_dt=0.2s) + jitter buffer + DL queue, NOT
network. The freshness axis starts at this floor; tau(u) is defined on total
Delta_use (AGEROW realized_age_ms). network_age_ms (UL+DL LUT only, added
freeze-1f=7c28b753) is the radio contribution per load for the load-to-age
table. T12 reruns on 1f (N=4 on 1e was pilot). 50ms bins kept; levels land on
300,450,600,... (sparse bins expected).
