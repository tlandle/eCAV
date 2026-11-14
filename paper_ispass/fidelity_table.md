# Fidelity ablation, straight crossing (stamp ispass_fid10)

Ten seeds per cell, 70 km/h, one connected vehicle and two roadside units, table radio load 24 senders. Source: paper_ispass/fidelity.csv.

## Fraction of runs completed without a collision, by added delay (ms)

| architecture | condition | 0 | 800 | 1200 | 1500 | 1800 | 2100 | 2400 |
|---|---|---|---|---|---|---|---|---|
| I2V object sharing | A idealized | 1.00 | 1.00 | 1.00 | 1.00 | 1.00 | 1.00 | 0.00 |
| I2V object sharing | B compute | 1.00 | 1.00 | 1.00 | 1.00 | 1.00 | 0.10 | 0.00 |
| I2V object sharing | C network | 1.00 | 1.00 | 1.00 | 1.00 | 1.00 | 0.00 | 0.00 |
| I2V object sharing | D full | 1.00 | 1.00 | 1.00 | 1.00 | 1.00 | 0.00 | 0.00 |
| V2X2V prediction | A idealized | 1.00 | 1.00 | 1.00 | 1.00 | 1.00 | 0.40 | 0.00 |
| V2X2V prediction | B compute | 1.00 | 1.00 | 1.00 | 1.00 | 1.00 | 0.10 | 0.00 |
| V2X2V prediction | C network | 1.00 | 1.00 | 1.00 | 1.00 | 1.00 | 0.10 | 0.00 |
| V2X2V prediction | D full | 1.00 | 1.00 | 1.00 | 1.00 | 1.00 | 0.50 | 0.00 |
| intermediate fusion | A idealized | 1.00 | 1.00 | 1.00 | 1.00 | 1.00 | 1.00 | 0.30 |
| intermediate fusion | B compute | 1.00 | 1.00 | 1.00 | 0.80 | 0.40 | 0.00 | 0.00 |
| intermediate fusion | C network | 1.00 | 1.00 | 1.00 | 1.00 | 0.00 | 0.10 | 0.00 |
| intermediate fusion | D full | 1.00 | 1.00 | 0.80 | 0.20 | 0.00 | 0.00 | 0.00 |

## Fraction of runs that fail on a false brake without a collision

A false brake is a sustained brake that starts on a track the labeler does not match to the crossing car, or a brake on the crossing car's forecast after the car has passed.

| architecture | condition | 0 | 800 | 1200 | 1500 | 1800 | 2100 | 2400 |
|---|---|---|---|---|---|---|---|---|
| I2V object sharing | A idealized | 0.00 | 0.00 | 0.00 | 0.00 | 0.00 | 0.00 | 0.00 |
| I2V object sharing | B compute | 0.00 | 0.00 | 0.00 | 0.00 | 0.00 | 0.00 | 0.00 |
| I2V object sharing | C network | 0.00 | 0.00 | 0.00 | 0.00 | 0.00 | 0.00 | 0.00 |
| I2V object sharing | D full | 0.10 | 0.00 | 0.00 | 0.00 | 0.00 | 0.00 | 0.00 |
| V2X2V prediction | A idealized | 0.00 | 0.00 | 0.00 | 0.00 | 0.00 | 0.00 | 0.00 |
| V2X2V prediction | B compute | 0.00 | 0.00 | 0.10 | 0.10 | 0.00 | 0.00 | 0.00 |
| V2X2V prediction | C network | 0.00 | 0.00 | 0.00 | 0.00 | 0.00 | 0.10 | 0.00 |
| V2X2V prediction | D full | 0.00 | 0.00 | 0.00 | 0.10 | 0.00 | 0.00 | 0.00 |
| intermediate fusion | A idealized | 0.00 | 0.00 | 0.00 | 0.00 | 0.00 | 0.00 | 0.30 |
| intermediate fusion | B compute | 0.00 | 0.00 | 0.00 | 0.00 | 0.10 | 0.00 | 0.00 |
| intermediate fusion | C network | 0.00 | 0.00 | 0.30 | 0.10 | 0.00 | 0.10 | 0.00 |
| intermediate fusion | D full | 0.00 | 0.10 | 0.20 | 0.00 | 0.00 | 0.00 | 0.00 |

## Boundary

Last cell with the rate at or above 0.90 and first cell at or below 0.40, as added delay and as the mean forecast age at the brake in that cell (ms).

| architecture | condition | last >= 0.90: added delay | forecast age | success | first <= 0.40: added delay | forecast age | success |
|---|---|---|---|---|---|---|---|
| I2V object sharing | A idealized | 2100 | 2100 | 1.00 | 2400 | 2400 | 0.00 |
| I2V object sharing | B compute | 1800 | 1850 | 1.00 | 2100 | 2150 | 0.10 |
| I2V object sharing | C network | 1800 | 1850 | 1.00 | 2100 | 2150 | 0.00 |
| I2V object sharing | D full | 1800 | 1855 | 1.00 | 2100 | 2150 | 0.00 |
| V2X2V prediction | A idealized | 1800 | 1800 | 1.00 | 2100 | 2100 | 0.40 |
| V2X2V prediction | B compute | 1800 | 1850 | 1.00 | 2100 | 2150 | 0.10 |
| V2X2V prediction | C network | 1800 | 1850 | 1.00 | 2100 | 2150 | 0.10 |
| V2X2V prediction | D full | 1800 | 1855 | 1.00 | 2400 | 2450 | 0.00 |
| intermediate fusion | A idealized | 2100 | 2150 | 1.00 | 2400 | 2400 | 0.30 |
| intermediate fusion | B compute | 1200 | 1445 | 1.00 | 1800 | 2145 | 0.40 |
| intermediate fusion | C network | 1500 | 1700 | 1.00 | 1800 | 2150 | 0.00 |
| intermediate fusion | D full | 800 | 1285 | 1.00 | 1500 | 2130 | 0.20 |

## Delay terms

Mean over the cells of each condition (ms). Compute is the measured service time charged to each forecast; uplink and downlink are the table radio delays at 24 senders.

| architecture | host and GPU | compute (B, D) | uplink (C, D) | downlink (C, D) |
|---|---|---|---|---|
| I2V object sharing | atlas, RTX 4080 Super | 20 (20 to 21) | 0 (0 to 0) | 6 (6 to 6) |
| V2X2V prediction | atlas, RTX 4080 Super | 22 (21 to 22) | 10 (10 to 10) | 6 (6 to 6) |
| intermediate fusion | cetus, RTX 3080 Ti | 145 (143 to 150) | 41 (41 to 41) | 223 (217 to 229) |

## Notes

- Code versions. Every cell with a composed delay (conditions B, C and D of all three architectures) ran at develop bcea7b2d or later (object-level arms at fd75bed5), which makes a recipient keep the newest forecast by source tick. Before it, an older forecast with a longer downlink sample could be delivered after a newer one and replace it (10.4 % of arrivals in intermediate fusion C). The earlier runs are in experiment_results/_prefix_reorder/ and _prefix_reorder_obj/. Condition A has no composed delay and is from the campaign before that commit; intermediate fusion B is also from before it.
- Evidence that the fix does not act on intermediate fusion A and B (one run each at bcea7b2d against the campaign run of the same seed; sensor data is not repeatable across simulator sessions, so the comparison is on arrivals, control and onset): A at 1800 ms has the same forecast arrival sequence, the same control and ego state on all 346 ticks and the same onset (+3550 ms), with 0 forecasts superseded; B at 1500 ms has 27 arrivals in both, no arrival out of order, 0 superseded, onset +3400 against +3450 ms.
- False brakes are listed separately from collisions. In intermediate fusion C at 1200 ms (3 of 10 runs) and 1500 ms (1 of 10) the brake is on a parked car's track beside the lane.
