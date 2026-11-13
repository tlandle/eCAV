# Fidelity ablation gate on the straight crossing (stamp ispass_fid10gate)

Geometry: straight crossing (scp_edge_late_fusion, Scenario_3, 70 km/h, crossing trigger 70:5.12 s), one connected vehicle and two roadside units, 3 seeds per cell, table radio load 24 senders. Object-level arms ran on atlas (RTX 4080 Super), intermediate fusion on cetus (RTX 3080 Ti, service occupancy on in conditions B and D). Runs at develop db12304b; reduced with the false-brake rule below.

Columns: success (no collision and no false brake), collided, false brake, mean age at the brake (ms), onset of the forecast brake after the crossing car's launch (median and range, ms), minimum separation between the two vehicles (median and range, m), fraction of runs in which a brake the conflict track did not cause came first, coverage, mean compute, uplink and downlink (ms).

| architecture | cond | added delay (ms) | n | success | collided | false brake | brake age | forecast brake onset | min separation | other brake first | coverage | compute | UL | DL |
|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|
| no cooperation | A | 0 | 3 | 0.00 | 1.00 | 0.33 | 0 | nan [nan, nan] | 3.0 [3.0, 3.0] | 0.33 | 0.81 | 0 | 0 | 0 |
| I2V object sharing | A | 0 | 3 | 1.00 | 0.00 | 0.00 | 0 | 2150 [2150, 2150] | 22.2 [22.1, 22.2] | 0.00 | 1.00 | 0 | 0 | 0 |
| I2V object sharing | A | 200 | 3 | 1.00 | 0.00 | 0.00 | 200 | 2100 [2100, 2100] | 23.2 [23.2, 23.2] | 0.00 | 1.00 | 0 | 0 | 0 |
| I2V object sharing | A | 400 | 3 | 1.00 | 0.00 | 0.00 | 400 | 2300 [2300, 2300] | 19.1 [19.1, 19.1] | 0.00 | 1.00 | 0 | 0 | 0 |
| I2V object sharing | A | 600 | 3 | 1.00 | 0.00 | 0.00 | 600 | 2100 [2100, 2100] | 23.2 [23.2, 23.2] | 0.00 | 1.00 | 0 | 0 | 0 |
| I2V object sharing | A | 800 | 3 | 1.00 | 0.00 | 0.00 | 800 | 2300 [2250, 2300] | 19.0 [19.0, 20.2] | 0.00 | 1.00 | 0 | 0 | 0 |
| I2V object sharing | A | 1000 | 3 | 1.00 | 0.00 | 0.00 | 1000 | 2450 [2450, 2450] | 18.5 [18.5, 18.5] | 0.00 | 1.00 | 0 | 0 | 0 |
| I2V object sharing | A | 1300 | 3 | 1.00 | 0.00 | 0.00 | 1300 | 2750 [2750, 2750] | 12.5 [12.5, 12.5] | 0.00 | 1.00 | 0 | 0 | 0 |
| I2V object sharing | A | 1600 | 3 | 1.00 | 0.00 | 0.00 | 1600 | 3100 [3100, 3100] | 7.1 [7.1, 7.1] | 0.00 | 1.00 | 0 | 0 | 0 |
| I2V object sharing | A | 1900 | 3 | 1.00 | 0.00 | 0.00 | 1900 | 3400 [3350, 3400] | 5.1 [5.1, 5.6] | 0.00 | 0.99 | 0 | 0 | 0 |
| I2V object sharing | A | 2200 | 3 | 0.00 | 1.00 | 0.00 | 2200 | 3650 [3650, 3650] | 3.8 [3.8, 3.9] | 0.00 | 0.99 | 0 | 0 | 0 |
| I2V object sharing | A | 2500 | 3 | 0.00 | 1.00 | 0.00 | 2500 | 3950 [3950, 3950] | 3.1 [3.1, 3.1] | 0.00 | 0.98 | 0 | 0 | 0 |
| V2X2V prediction | A | 0 | 3 | 1.00 | 0.00 | 0.00 | 0 | 2150 [2050, 2150] | 22.4 [22.1, 23.3] | 0.00 | 1.00 | 0 | 0 | 0 |
| V2X2V prediction | A | 200 | 3 | 1.00 | 0.00 | 0.00 | 200 | 2100 [2100, 2150] | 23.2 [23.2, 23.3] | 0.00 | 1.00 | 0 | 0 | 0 |
| V2X2V prediction | A | 400 | 3 | 1.00 | 0.00 | 0.00 | 400 | 2300 [2300, 2350] | 19.2 [19.1, 20.1] | 0.00 | 1.00 | 0 | 0 | 0 |
| V2X2V prediction | A | 600 | 3 | 1.00 | 0.00 | 0.00 | 600 | 2100 [2100, 2100] | 23.2 [23.2, 23.2] | 0.00 | 1.00 | 0 | 0 | 0 |
| V2X2V prediction | A | 800 | 3 | 1.00 | 0.00 | 0.00 | 800 | 2300 [2300, 2300] | 19.1 [19.1, 19.1] | 0.00 | 1.00 | 0 | 0 | 0 |
| V2X2V prediction | A | 1000 | 3 | 1.00 | 0.00 | 0.00 | 1000 | 2500 [2500, 2500] | 17.1 [17.1, 17.1] | 0.00 | 1.00 | 0 | 0 | 0 |
| V2X2V prediction | A | 1300 | 3 | 1.00 | 0.00 | 0.00 | 1300 | 2800 [2800, 2800] | 11.6 [11.5, 11.6] | 0.00 | 1.00 | 0 | 0 | 0 |
| V2X2V prediction | A | 1600 | 3 | 1.00 | 0.00 | 0.00 | 1600 | 3100 [3100, 3100] | 7.1 [7.0, 7.1] | 0.00 | 1.00 | 0 | 0 | 0 |
| V2X2V prediction | A | 1900 | 3 | 0.67 | 0.00 | 0.33 | 1900 | 3400 [3350, 3650] | 5.9 [5.1, 23.3] | 0.33 | 1.00 | 0 | 0 | 0 |
| V2X2V prediction | A | 2200 | 3 | 0.33 | 0.67 | 0.00 | 2200 | 3650 [3650, 3700] | 3.9 [3.6, 4.5] | 0.00 | 1.00 | 0 | 0 | 0 |
| V2X2V prediction | A | 2500 | 3 | 0.00 | 1.00 | 0.00 | 2500 | 3950 [3950, 4000] | 3.2 [3.1, 3.4] | 0.00 | 0.99 | 0 | 0 | 0 |
| intermediate fusion | A | 0 | 3 | 1.00 | 0.00 | 0.00 | 0 | 2050 [2050, 2050] | 24.8 [24.8, 24.8] | 0.00 | 0.93 | 0 | 0 | 0 |
| intermediate fusion | A | 200 | 3 | 1.00 | 0.00 | 0.00 | 200 | 2050 [2050, 2050] | 24.7 [24.7, 24.7] | 0.00 | 0.95 | 0 | 0 | 0 |
| intermediate fusion | A | 400 | 3 | 1.00 | 0.00 | 0.00 | 400 | 2150 [2150, 2150] | 22.5 [22.5, 22.5] | 0.00 | 0.97 | 0 | 0 | 0 |
| intermediate fusion | A | 600 | 3 | 1.00 | 0.00 | 0.00 | 600 | 2400 [2400, 2400] | 19.1 [19.1, 19.1] | 0.00 | 1.00 | 0 | 0 | 0 |
| intermediate fusion | A | 800 | 3 | 1.00 | 0.00 | 0.00 | 800 | 2650 [2650, 2650] | 15.4 [15.4, 15.4] | 0.00 | 1.00 | 0 | 0 | 0 |
| intermediate fusion | A | 1000 | 3 | 1.00 | 0.00 | 0.00 | 1000 | 2750 [2750, 2750] | 13.0 [13.0, 13.0] | 0.00 | 1.00 | 0 | 0 | 0 |
| intermediate fusion | A | 1300 | 3 | 1.00 | 0.00 | 0.00 | 1300 | 3050 [3050, 3050] | 8.1 [8.1, 8.1] | 0.00 | 1.00 | 0 | 0 | 0 |
| intermediate fusion | A | 1600 | 3 | 1.00 | 0.00 | 0.00 | 1600 | 3450 [3450, 3450] | 5.4 [5.4, 5.4] | 0.00 | 1.00 | 0 | 0 | 0 |
| intermediate fusion | A | 1900 | 3 | 1.00 | 0.00 | 0.00 | 1950 | 3600 [3600, 3600] | 4.7 [4.7, 4.7] | 0.00 | 1.00 | 0 | 0 | 0 |
| intermediate fusion | A | 2200 | 3 | 0.00 | 1.00 | 1.00 | 2200 | 4200 [4200, 4200] | 3.2 [3.2, 3.2] | 0.00 | 0.98 | 0 | 0 | 0 |
| intermediate fusion | B | 0 | 3 | 1.00 | 0.00 | 0.00 | 267 | 2150 [2150, 2200] | 22.5 [21.1, 23.1] | 0.00 | 0.98 | 148 | 0 | 0 |
| intermediate fusion | B | 400 | 3 | 1.00 | 0.00 | 0.00 | 617 | 2500 [2450, 2500] | 18.2 [18.1, 18.7] | 0.00 | 1.00 | 144 | 0 | 0 |
| intermediate fusion | B | 800 | 3 | 1.00 | 0.00 | 0.00 | 1000 | 2900 [2750, 3250] | 9.9 [5.3, 11.5] | 0.00 | 1.00 | 142 | 0 | 0 |
| intermediate fusion | B | 1200 | 3 | 1.00 | 0.00 | 0.00 | 1450 | 3150 [3100, 3200] | 6.7 [6.7, 6.9] | 0.00 | 1.00 | 143 | 0 | 0 |
| intermediate fusion | B | 1600 | 3 | 0.33 | 0.67 | 0.00 | 1833 | 3700 [3650, 3800] | 3.8 [3.6, 4.5] | 0.00 | 0.99 | 145 | 0 | 0 |
| intermediate fusion | D | 0 | 3 | 1.00 | 0.00 | 0.00 | 467 | 2200 [2150, 2250] | 21.1 [20.8, 22.6] | 0.00 | 0.99 | 149 | 41 | 222 |
| intermediate fusion | D | 400 | 3 | 1.00 | 0.00 | 0.00 | 900 | 2650 [2600, 2700] | 14.5 [14.5, 15.7] | 0.00 | 1.00 | 146 | 41 | 221 |
| intermediate fusion | D | 800 | 3 | 1.00 | 0.00 | 0.00 | 1350 | 3050 [3000, 3100] | 7.6 [7.6, 8.9] | 0.00 | 1.00 | 145 | 41 | 226 |
| intermediate fusion | D | 1200 | 3 | 0.67 | 0.00 | 0.33 | 1733 | 3300 [3150, 3550] | 6.3 [4.3, 6.8] | 0.00 | 1.00 | 144 | 41 | 218 |
| intermediate fusion | D | 1600 | 3 | 0.00 | 1.00 | 1.00 | 2133 | 4400 [4050, 4400] | 3.1 [3.1, 3.3] | 0.00 | 0.98 | 143 | 41 | 223 |

## Acceptance

a. No cooperation collides in 3 of 3 runs (minimum separation 3.0 m, no forecast brake). Every arm completes at 0 ms in condition A with a minimum separation of 22.2 to 24.8 m, about 19 m clear of contact.

b. In condition A success does not increase with delay in any arm, and every arm reaches collision inside the grid: I2V and intermediate fusion between 1900 and 2200 ms, V2X2V between 1900 and 2500 ms.

c. The first brake is the forecast brake on the crossing car's track in every run but one. No curve speed limit occurs (the connected vehicle drives straight).
   - Ruling 1: below about 800 ms of added delay the minimum separation of the object-level arms does not fall with age (19.0 to 23.2 m). The brake there fires when the conflict enters the planner's comparison window (time to collision 2.2 to 2.5 s at the brake), not when the forecast arrives. From 1000 ms upward the separation falls monotonically to the collision boundary. The monotonic requirement applies from the knee upward.
   - Ruling 2: in V2X2V, condition A, 1900 ms, run 3, a hazard on a track that the labeler matches to a parked car beside the lane (not the crossing car) started a stop hold at +2000 ms. The reducer counts a run whose sustained brake starts on a track that is not the conflict actor's, before the forecast brake or without one, as a false brake and as a failure. That cell is 2 of 3. The same rule marks one no-cooperation run (a hazard after contact); its success was already 0.

d. Intermediate fusion with measured compute (B) and with compute and radio (D) brakes within one to two publish periods (150 to 200 ms each) of condition A at the same forecast age, with the same outcomes away from the boundary.
   - B at 1600 ms (forecast age 1750 to 1800 ms) brakes at +3650 to +3800 ms and collides in 2 of 3 runs. Condition A brakes once it holds a forecast generated about 1700 to 1850 ms after the launch (forecast speed 12.3 to 12.9 m/s). In B forecasts are generated every 150 to 200 ms, and the forecast that triggered was generated at +1900 to +2050 ms.
   - D at 1200 ms (forecast age 1550 to 1650 ms at the brake) brakes at +3150, +3300 and +3550 ms, around condition A at 1600 ms (+3450 ms). Its one false brake (run 3) is two hazard events on the crossing car's own track at +4700 and +4750 ms, after the car had passed: the stale forecast still placed the car in the conflict.

## Grid for the campaign

Added delay 0, 800, 1200, 1500, 1800, 2100, 2400 ms for every arm and condition (stamp ispass_fid10, 10 seeds, conditions A B C D, table radio load 24 senders, intermediate fusion with service occupancy on).
