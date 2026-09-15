# ns-3 5G-LENA co-simulation on the edge path: validation block (design, 2026-09-14)

Tyler's ruling (radio option 2): the paper keeps the co-simulation wording and
earns it. Build the validation block first, report the true per-run wall time
and the agreement with the lookup table before any rerun is committed to the
co-simulated path. Nothing in the paper claims a co-simulated number until the
reruns land; the wording is ahead of the data.

## What exists

- Bridge: ecav/core/networking/ns3_cosim/ns3_bridge.py (NS3Bridge over POSIX
  shared memory; mode "uu_uplink" = NR Uu to the gNB; compute_tick(positions,
  occlusion_info, tick, payload_bytes) -> [LinkResult(tx_id, rx_id, delivered,
  sinr_db, delay_ms)], one per UE in Uu mode, DL rows by sentinel as the LUT
  builder reads them; get_last_timing() gives the per-tick wall time).
- Server: nr_v2x_cosim_server.cc (5G-LENA nr module, UMi street canyon,
  40 MHz at 5.9 GHz, 23 dBm, numerology 0), binary on atlas only:
  /home/atlas/ns3_cv2x/ns-3-dev-5glena/build/scratch/ns3-dev-nr_v2x_cosim_server-default.
- Edge path today (code_overlay/edge_manager_merged_1l_epoch.py): UL stamped
  per fusion tick from the LUT (max over the live uploaders of
  sample_ms(n_cell, ul_bytes, 'ul'), arrival = frame + ceil(ul_ms / dt), lines
  440-473); DL stamped at publish from the LUT (sample_ms(n, dl_bytes, 'dl'),
  deliver_tick = tick + ceil((compute_ms + dl_ms) / dt), lines 912-928);
  use_ns3_lut false falls to the parametric HybridModel. The bridge has no
  call site on this path.

## Wiring (overlay change behind NS3_COSIM=1, default off; the LUT path is
byte-identical when off)

- One bridge per edge (per-locale cell): NS3Bridge(mode="uu_uplink",
  max_vehicles=8, tick_duration_s=dt), started lazily at the edge's first
  fusion tick (bring-up about 75 s per bridge, logged), the cell's gNB at the
  locale's RSU: positions fed as [x - rsu_x, y - rsu_y, z] for the uploading
  agents of that fusion tick (RSU excluded: it is the gNB side), payload_bytes
  = ul_bytes as the LUT path computes it.
- UL: per-agent delay_ms from the results; ul_ms = max over agents (the same
  order statistic as the LUT path), arrival = frame + ceil(ul_ms / dt);
  delivered=False is logged and, for parity with the landed path, not applied
  as loss (the landed runs model delay only). A second column carries the LUT
  draw for the same tick and agents, so both are on every row.
- DL: at publish, the DL delay of the recipient CAV from the same tick's
  results (the Uu server writes DL rows), the LUT draw beside it.
- Marker per fusion tick per edge: [RADIOROW] tick edge n_ul ul_live_ms
  ul_lut_ms dl_live_ms dl_lut_ms delivered_frac ns3_wall_ms; the existing
  [AGEROW] and the age-at-use accounting are unchanged, so the forecast age at
  use is comparable between the live and the LUT runs of the same cell.
- Shutdown of the bridges at scenario end (the scenario manager's cleanup).

## Validation block (atlas only; the binary is there)

- Cells: one flow cell (openscenario_1_flow_gt, warm arm, seed 1, 12 m/s) and
  one corridor route (openscenario_1_corridor_gt, warm arm, one seed), each
  run twice: NS3_COSIM=1 (live) and the LUT (landed path), same seed, same
  everything else, under the 1q predictor once ruled (the radio path does not
  touch the predictor, so the smoke of the wiring can run under 1p; the
  numbers that go to the paper wait for 1q).
- Reported: per-run wall time live vs LUT (and the bridge's own per-tick
  wall time distribution, bring-up separated); the per-message UL and DL
  latency distributions (p50 / p95 / p99 and PRR from delivered) live vs LUT
  on the same ticks; the forecast age at use (the [AGEROW] population) live vs
  LUT; the arms' outcome columns for the record.
- Agreement rule (paper side, 2026-09-14: the network sentence promises that
  radio load sets the TAIL of the sensing-to-planner age): compare p50, p95
  and p99 of the per-message UL and DL latencies and of the forecast age at
  use, live against LUT on the same cells. The LUT draw is a Gaussian around
  a cell's mean and p95; the live bridge produces the true per-tick
  distribution, so a tail disagreement is the expected finding, not a defect.
  Rule: p50 within the LUT cell's sigma; p95 and p99 reported with their
  ratio live/LUT; if the age-at-use tail disagrees beyond one edge cycle at
  p95, the paper's sentence changes to what the co-simulation shows, not the
  rule.

## Invalidation conditions

- If the bridge's per-tick wall time makes a flow run exceed the 900 s timeout,
  the block reports the time and the per-tick cost; the timeout is raised only
  as a stated change.
- If the bridge returns fewer results than uploaders on any tick, or a
  delivered fraction below the LUT cell's PRR by more than 0.1 at the same
  count, the geometry or payload mapping is wrong; stop and check the position
  frame before any number is reported.
- If live UL latency depends on the locale's RSU position (three cells) in a
  way the ring-topology LUT cannot show, that is a finding, not an error, and
  is reported per locale.

## Cost and hosts

Three bridges per run (one per edge) each with a ~75 s bring-up, and a
blocking call per fusion tick per edge: the per-run wall time is the number
this block measures; the KB's prior estimate is 2 to 5x the LUT's 3.4 to 5
minutes. Atlas only until an ns-3 build exists on cetus (no build there;
scripts/setup_ns3_cv2x.sh is the recipe, about half a day).

## Order

Design (this note) -> wiring in the edge overlay behind NS3_COSIM (LUT path
unchanged) -> wiring smoke under 1p on atlas (one flow cell live, markers
present, bring-up and per-tick times logged) -> the four validation runs under
1q -> report -> Tyler decides whether the reruns move to the co-simulated
path.
