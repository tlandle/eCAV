# Fidelity ablation, straight crossing (stamp ispass_fid11)

Ten seeds per cell, 70 km/h, one connected vehicle and two roadside units, 24 senders on the radio. I2V object sharing and V2X2V prediction resolve the radio from the sampled tables. Intermediate fusion (16.9 KB messages) resolves it with the live ns-3 process, once with the connected vehicle in a sender slot that the scheduler serves in full and once in a slot that receives about half of its downlink messages. Its full condition replays the execution times recorded in the compute condition of the same delay and repetition. Source: paper_ispass/fidelity.csv.

## Fraction of runs completed without a collision, by added delay (ms)

| architecture | condition | 0 | 800 | 1200 | 1500 | 1800 | 2100 | 2400 |
|---|---|---|---|---|---|---|---|---|
| I2V object sharing | A idealized | 1.00 | 1.00 | 1.00 | 1.00 | 1.00 | 0.30 | 0.00 |
| I2V object sharing | B compute | 1.00 | 1.00 | 1.00 | 1.00 | 1.00 | 0.00 | 0.00 |
| I2V object sharing | C network | 1.00 | 1.00 | 1.00 | 1.00 | 1.00 | 0.00 | 0.00 |
| I2V object sharing | D full | 1.00 | 1.00 | 1.00 | 1.00 | 1.00 | 0.00 | 0.00 |
| V2X2V prediction | A idealized | 1.00 | 1.00 | 1.00 | 1.00 | 1.00 | 0.40 | 0.00 |
| V2X2V prediction | B compute | 1.00 | 1.00 | 1.00 | 1.00 | 1.00 | 0.00 | 0.00 |
| V2X2V prediction | C network | 1.00 | 1.00 | 1.00 | 1.00 | 1.00 | 0.20 | 0.10 |
| V2X2V prediction | D full | 1.00 | 1.00 | 1.00 | 1.00 | 1.00 | 0.20 | 0.00 |
| intermediate fusion | A idealized | 1.00 | 1.00 | 1.00 | 1.00 | 1.00 | 0.00 | 0.10 |
| intermediate fusion | B compute | 1.00 | 1.00 | 1.00 | 1.00 | 0.50 | 0.10 | 0.00 |
| intermediate fusion, slot 0 (100% delivered) | C network | 1.00 | 1.00 | 1.00 | 1.00 | 1.00 | 0.00 | 0.00 |
| intermediate fusion, slot 0 (100% delivered) | D full | 1.00 | 1.00 | 1.00 | 0.60 | 0.10 | 0.00 | 0.00 |
| intermediate fusion, slot 21 (52% delivered) | C network | 1.00 | 1.00 | 1.00 | 0.00 | 0.00 | 0.00 | 0.00 |
| intermediate fusion, slot 21 (52% delivered) | D full | 1.00 | 1.00 | 0.60 | 0.00 | 0.00 | 0.00 | 0.10 |

## Fraction of runs that fail on a false brake without a collision

A false brake is a sustained brake that starts on a track the labeler does not match to the crossing car, or a brake on the crossing car's forecast after the car has passed.

| architecture | condition | 0 | 800 | 1200 | 1500 | 1800 | 2100 | 2400 |
|---|---|---|---|---|---|---|---|---|
| I2V object sharing | A idealized | 0.10 | 0.00 | 0.00 | 0.00 | 0.00 | 0.00 | 0.00 |
| I2V object sharing | B compute | 0.00 | 0.00 | 0.00 | 0.00 | 0.00 | 0.00 | 0.00 |
| I2V object sharing | C network | 0.00 | 0.00 | 0.00 | 0.00 | 0.00 | 0.00 | 0.00 |
| I2V object sharing | D full | 0.10 | 0.00 | 0.00 | 0.00 | 0.00 | 0.00 | 0.00 |
| V2X2V prediction | A idealized | 0.00 | 0.00 | 0.00 | 0.00 | 0.00 | 0.00 | 0.00 |
| V2X2V prediction | B compute | 0.10 | 0.00 | 0.00 | 0.00 | 0.00 | 0.00 | 0.00 |
| V2X2V prediction | C network | 0.00 | 0.00 | 0.00 | 0.00 | 0.00 | 0.00 | 0.10 |
| V2X2V prediction | D full | 0.10 | 0.00 | 0.00 | 0.00 | 0.00 | 0.00 | 0.00 |
| intermediate fusion | A idealized | 0.00 | 0.00 | 0.00 | 0.00 | 0.00 | 0.00 | 0.10 |
| intermediate fusion | B compute | 0.00 | 0.20 | 0.10 | 0.10 | 0.00 | 0.00 | 0.00 |
| intermediate fusion, slot 0 (100% delivered) | C network | 0.00 | 0.00 | 0.00 | 0.00 | 0.00 | 0.00 | 0.00 |
| intermediate fusion, slot 0 (100% delivered) | D full | 0.00 | 0.00 | 0.00 | 0.10 | 0.00 | 0.00 | 0.00 |
| intermediate fusion, slot 21 (52% delivered) | C network | 0.00 | 0.00 | 0.00 | 0.00 | 0.00 | 0.00 | 0.00 |
| intermediate fusion, slot 21 (52% delivered) | D full | 0.00 | 0.00 | 0.00 | 0.00 | 0.00 | 0.00 | 0.10 |

## Mean forecast age at the brake (ms), by added delay (ms)

| architecture | condition | 0 | 800 | 1200 | 1500 | 1800 | 2100 | 2400 |
|---|---|---|---|---|---|---|---|---|
| I2V object sharing | A idealized | 0 | 800 | 1200 | 1500 | 1800 | 2100 | 2400 |
| I2V object sharing | B compute | 50 | 850 | 1250 | 1550 | 1850 | 2150 | 2450 |
| I2V object sharing | C network | 50 | 850 | 1250 | 1550 | 1850 | 2150 | 2450 |
| I2V object sharing | D full | 50 | 850 | 1250 | 1550 | 1850 | 2150 | 2455 |
| V2X2V prediction | A idealized | 0 | 800 | 1200 | 1500 | 1800 | 2100 | 2400 |
| V2X2V prediction | B compute | 50 | 850 | 1250 | 1550 | 1850 | 2150 | 2450 |
| V2X2V prediction | C network | 50 | 850 | 1250 | 1550 | 1850 | 2150 | 2450 |
| V2X2V prediction | D full | 50 | 850 | 1255 | 1550 | 1850 | 2150 | 2455 |
| intermediate fusion | A idealized | 0 | 800 | 1200 | 1550 | 1800 | 2150 | 2400 |
| intermediate fusion | B compute | 140 | 1000 | 1375 | 1665 | 1945 | 2285 | 2570 |
| intermediate fusion, slot 0 (100% delivered) | C network | 100 | 900 | 1300 | 1600 | 1950 | 2200 | 2550 |
| intermediate fusion, slot 0 (100% delivered) | D full | 300 | 1055 | 1465 | 1795 | 2075 | 2360 | 2675 |
| intermediate fusion, slot 21 (52% delivered) | C network | 460 | 1225 | 1590 | 1950 | 2245 | 2500 | 2900 |
| intermediate fusion, slot 21 (52% delivered) | D full | 545 | 1355 | 1750 | 2045 | 2350 | 2615 | 2960 |

## Boundary

Last cell with the rate at or above 0.90 and first cell at or below 0.40, as added delay and as the mean forecast age at the brake in that cell (ms).

| architecture | condition | last >= 0.90: added delay | forecast age | success | first <= 0.40: added delay | forecast age | success |
|---|---|---|---|---|---|---|---|
| I2V object sharing | A idealized | 1800 | 1800 | 1.00 | 2100 | 2100 | 0.30 |
| I2V object sharing | B compute | 1800 | 1850 | 1.00 | 2100 | 2150 | 0.00 |
| I2V object sharing | C network | 1800 | 1850 | 1.00 | 2100 | 2150 | 0.00 |
| I2V object sharing | D full | 1800 | 1850 | 1.00 | 2100 | 2150 | 0.00 |
| V2X2V prediction | A idealized | 1800 | 1800 | 1.00 | 2100 | 2100 | 0.40 |
| V2X2V prediction | B compute | 1800 | 1850 | 1.00 | 2100 | 2150 | 0.00 |
| V2X2V prediction | C network | 1800 | 1850 | 1.00 | 2100 | 2150 | 0.20 |
| V2X2V prediction | D full | 1800 | 1850 | 1.00 | 2100 | 2150 | 0.20 |
| intermediate fusion | A idealized | 1800 | 1800 | 1.00 | 2100 | 2150 | 0.00 |
| intermediate fusion | B compute | 1500 | 1665 | 1.00 | 2100 | 2285 | 0.10 |
| intermediate fusion, slot 0 (100% delivered) | C network | 1800 | 1950 | 1.00 | 2100 | 2200 | 0.00 |
| intermediate fusion, slot 0 (100% delivered) | D full | 1200 | 1465 | 1.00 | 1800 | 2075 | 0.10 |
| intermediate fusion, slot 21 (52% delivered) | C network | 1200 | 1590 | 1.00 | 1500 | 1950 | 0.00 |
| intermediate fusion, slot 21 (52% delivered) | D full | 800 | 1355 | 1.00 | 1500 | 2045 | 0.00 |

## Delay terms

Mean over the cells of each condition (ms), range over the cells in parentheses. Compute is the execution time charged to each forecast. Uplink and downlink are the radio delays of delivered messages at 24 senders, and dropped is the fraction of downlink messages not delivered.

| architecture | host and GPU | compute (B, D) | uplink (C, D) | downlink mean (C, D) | downlink p95 (C, D) | dropped (C, D) |
|---|---|---|---|---|---|---|
| I2V object sharing | atlas, RTX 4080 Super | 18 (17 to 18) | 0 (0 to 0) | 6 (6 to 6) | 7 (7 to 9) | 0.00 (0.00 to 0.00) |
| V2X2V prediction | atlas, RTX 4080 Super | 20 (19 to 20) | 11 (10 to 11) | 8 (7 to 8) | 16 (13 to 18) | 0.00 (0.00 to 0.00) |
| intermediate fusion, slot 0 (100% delivered) | atlas, RTX 4080 Super | 96 (95 to 97) | 41 (41 to 41) | 48 (48 to 48) | 49 (49 to 49) | 0.00 (0.00 to 0.00) |
| intermediate fusion, slot 21 (52% delivered) | atlas, RTX 4080 Super | 96 (95 to 97) | 41 (41 to 41) | 140 (138 to 143) | 289 (289 to 289) | 0.48 (0.47 to 0.49) |

## Notes

- One code version for every cell (develop cc8a5523, chain of Nov 20 to 22 on atlas).
- The live ns-3 scheduler serves senders by slot and the pattern repeats across seeds. Each repetition uses its own ns-3 stream.
- False brakes are listed separately from collisions.
