# Fidelity ablation, straight crossing (stamp ispass_fid10)

Ten seeds per cell, 70 km/h, one connected vehicle and two roadside units, table radio load 24 senders. Success is a run without a collision and without a false brake. Source: paper_ispass/fidelity.csv.

## Operational success by added delay (ms)

| architecture | condition | 0 | 800 | 1200 | 1500 | 1800 | 2100 | 2400 |
|---|---|---|---|---|---|---|---|---|
| I2V object sharing | A idealized | 1.00 | 1.00 | 1.00 | 1.00 | 1.00 | 1.00 | 0.00 |
| I2V object sharing | B compute | 1.00 | 1.00 | 1.00 | 1.00 | 1.00 | 0.00 | 0.00 |
| I2V object sharing | C network | 1.00 | 1.00 | 1.00 | 1.00 | 1.00 | 0.00 | 0.00 |
| I2V object sharing | D full | 0.90 | 1.00 | 1.00 | 1.00 | 1.00 | 0.00 | 0.00 |
| V2X2V prediction | A idealized | 1.00 | 1.00 | 1.00 | 1.00 | 1.00 | 0.40 | 0.00 |
| V2X2V prediction | B compute | 1.00 | 1.00 | 1.00 | 1.00 | 0.90 | 0.40 | 0.00 |
| V2X2V prediction | C network | 1.00 | 1.00 | 1.00 | 1.00 | 1.00 | 0.00 | 0.00 |
| V2X2V prediction | D full | 1.00 | 1.00 | 1.00 | 0.90 | 0.90 | 0.10 | 0.00 |
| intermediate fusion | A idealized | 1.00 | 1.00 | 1.00 | 1.00 | 1.00 | 1.00 | 0.00 |
| intermediate fusion | B compute | 1.00 | 1.00 | 1.00 | 0.80 | 0.30 | 0.00 | 0.00 |
| intermediate fusion | C network | 1.00 | 1.00 | 1.00 | 1.00 | 0.00 | 0.00 | 0.00 |
| intermediate fusion | D full | 1.00 | 0.90 | 0.40 | 0.10 | 0.00 | 0.00 | 0.00 |

## Boundary

Last cell with success at or above 0.90 and first cell at or below 0.40, as added delay and as the mean forecast age at the brake in that cell (ms).

| architecture | condition | last >= 0.90: added delay | forecast age | success | first <= 0.40: added delay | forecast age | success |
|---|---|---|---|---|---|---|---|
| I2V object sharing | A idealized | 2100 | 2100 | 1.00 | 2400 | 2400 | 0.00 |
| I2V object sharing | B compute | 1800 | 1850 | 1.00 | 2100 | 2150 | 0.00 |
| I2V object sharing | C network | 1800 | 1850 | 1.00 | 2100 | 2150 | 0.00 |
| I2V object sharing | D full | 1800 | 1850 | 1.00 | 2100 | 2150 | 0.00 |
| V2X2V prediction | A idealized | 1800 | 1800 | 1.00 | 2100 | 2100 | 0.40 |
| V2X2V prediction | B compute | 1800 | 1855 | 0.90 | 2100 | 2150 | 0.40 |
| V2X2V prediction | C network | 1800 | 1850 | 1.00 | 2100 | 2150 | 0.00 |
| V2X2V prediction | D full | 1800 | 1850 | 0.90 | 2100 | 2150 | 0.10 |
| intermediate fusion | A idealized | 2100 | 2150 | 1.00 | 2400 | 2400 | 0.00 |
| intermediate fusion | B compute | 1200 | 1445 | 1.00 | 1800 | 2145 | 0.30 |
| intermediate fusion | C network | 1500 | 1700 | 1.00 | 1800 | 2200 | 0.00 |
| intermediate fusion | D full | 800 | 1295 | 0.90 | 1200 | 1710 | 0.40 |

## Delay terms

Mean over the cells of each condition (ms). Compute is the measured service time charged to each forecast; uplink and downlink are the table radio delays at 24 senders.

| architecture | host and GPU | compute (B, D) | uplink (C, D) | downlink (C, D) |
|---|---|---|---|---|
| I2V object sharing | atlas, RTX 4080 Super | 20 (19 to 20) | 0 (0 to 0) | 6 (6 to 6) |
| V2X2V prediction | atlas, RTX 4080 Super | 21 (21 to 22) | 10 (10 to 10) | 6 (6 to 6) |
| intermediate fusion | cetus, RTX 3080 Ti | 145 (143 to 150) | 41 (41 to 41) | 223 (219 to 225) |
