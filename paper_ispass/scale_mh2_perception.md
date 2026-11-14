# Perception rows with sensor frame synchronisation (stamp mh2, branch sensor-sync at 87c20fa6)

Every client waits for the current tick's frame from each camera and its LiDAR before detection (frame age 0).
Four cameras 640x480 and one LiDAR per vehicle, 20 Hz, scale_town06 dense, 300 ticks up to 32 vehicles and 100 above.
Mean tick wall time in ms.

| deployment | 8 | 16 | 32 | 64 | 128 |
|---|---|---|---|---|---|
| OpenCDA 0.1.3, one host (median; does not wait for the tick's frames) | 1596 | 3323 | 7213 | 15180 | does not start |
| eCAV, one host, service on the A10 | 436 | 806 | 1568 | no first tick in two attempts | not run |
| eCAV, three hosts, 1 replica on NDm | 367 | 639 | 1214 | 2499 | failed twice |
| eCAV, three hosts, 2 replicas | 364 | 637 | 1214 | 2453 | failed twice |
| eCAV, three hosts, 4 replicas | 367 | 632 | 1218 | 2432 | 4490 |
| eCAV, three hosts, 8 replicas | 360 | 638 | 1223 | 2445 | failed twice |

- With the wait in place the replica count does not change the tick. The tick is bounded by the simulator delivering the tick's sensor frames (mean wait 438 to 646 ms at 32 vehicles).
- Three hosts against one host: 1.19, 1.26 and 1.29 times at 8, 16 and 32 vehicles.
- 128 vehicles is the sensor memory limit of the A10 (22 of 24 GB). One of four configurations started within two attempts each.
- The mh1 perception rows (scale_mh1.md) were measured before the fix, when detection used frames one or two ticks old, and are superseded by this table.
