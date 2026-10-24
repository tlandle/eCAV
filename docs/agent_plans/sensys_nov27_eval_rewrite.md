# SenSys Nov 27 resubmission: evaluation rewrite plan (proposed 2026-10-24 19:55, awaiting Tyler)

Target file: paper repo contents/evaluation4.tex (Section 5). Structure is kept: 5.1 Methodology (7 sub-sub-sections), 5.2 Results (roadmap + six sub-sub-sections, Takeaways 1-6), 5.3 Overhead (Takeaway 7). Edits are paragraph-local so Overleaf comments and the \ad/\KR/\tl markers survive.

## Data source per slot (one code version per figure: j3 = f59b5987 / d770d40d)

| Slot | Current float | j3 replacement |
|---|---|---|
| 5.2.1 envelope | arch_safety_scale (oracle, contract on/off, local) | new: P(S_op) vs measured age, oracle / naive / ETSI / contract, LT and SCP at the reference speed, Wilson bars |
| 5.2.2 situation | situation_sensitivity (a,b) | fig_situation_j3 (a) oracle per geometry, (b) tau_max vs speed oracle+contract bootstrap, (c) stop vs ttc_graded; plus Table tau_max at 0.8/0.9/0.95 |
| 5.2.3 landing | arch_landing, breakdown, payload table, safety_margin | unchanged except safety_margin rebuilt with j3 tau_max |
| 5.2.4 failure channel | arch_failure_decomp N=4 | new: classes collision / self-ghost / false brake / stall per arm at N=4 (scaling + landing stages) + overtake panel (oracle 1.5-2 s boundary, contract late completion) |
| 5.2.5 MAC | fig19_mac_safety (old run set) | decision: rerun MAC points on j3 (~80 runs) or cut to the ns-3 age table |
| 5.2.6 ablation | sba_ablation (VIPS +/- contract N=8; loss/GNSS) | new: (a) merge-point arms naive / ETSI / Mahalanobis gate / VIPS / SBA on measured age, (b) faults: beacon loss, delay, clock offset, position noise x contract+naive / ETSI / contract+ValidBeacon at SCP 200 ms and LT 400 ms; ValidBeacon threshold table |
| 5.3 overhead | tbl-bw-cost | unchanged; retrodiction cost table goes to the response appendix unless Tyler wants it here |

## Methodology edits
- Metrics: age measured per tick at the planner (per_tick_age), Wilson on P(S_op), bootstrap on age statistics, tau_max at 0.8/0.9/0.95.
- Evaluated systems: Oracle redefined as the exact-state reference (not an upper bound); add ETSI self-announcement, Mahalanobis gate (3.75), VIPS rectification as merge-point arms.
- Measurement studies: situation study = 3 geometries x 6 speeds x 2 control responses; faults study added.
- Latency models: replace the 0.75/1.0/1.25 multiplier paragraph with the six speeds and two controllers.
- Scenarios: two -> three geometries; 20 trials -> 10 seeds; fix the NHTSA code (straight crossing is SCP; the overtake needs its own code, verify against the typology).
- Participant scaling: closed-loop N up to 16 (LT), N=4 (SCP); landing study to 32 stays.

## Numbers that change (tau_max at the 0.8 threshold, ms)
oracle LT 38:1625 50:1125 53:1625 63:1625 70:475 88:375; SCP 38:1125 50:825 53:625 63:625 70:525 88:325; overtake 43:1625.
contract LT 38/53/63 censored >3 s, 70:425, 88:125; SCP 50/70 censored at sweep end, 53/63:575, 88:175; overtake 825.
Old text: oracle ~350-400 ms, budgets 220-450 ms, 2-4x the 100 ms figure. All replaced.

## New sentences
- 5.2.1: exact-state reference wording (2-3 sentences; threshold mechanism verified with HAZARD_TRACE replay).
- 5.2.2: controller paragraph for panel (c).
- 5.2.4: overtake late completion (contract 0.7-0.9 beyond the oracle boundary by S_prog).
- 5.2.6: ETSI/gate/VIPS paragraph; faults paragraph with ValidBeacon cost.

## Tyler's calls
1. Oracle-row wording (exact-state reference). 2. Overtake framing (physics + late completion). 3. MAC subsection: rerun or cut. 4. Overtake NHTSA code. 5. Retrodiction table placement.

## Builders still to write
envelope figure, failure decomposition, ablation + faults figure, safety_margin rebuild, tau table export. fig_situation exists (safety_envelope_paper/analysis/fig_situation_j3).
