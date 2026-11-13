Median tick wall time (ms) against the number of vehicles, no perception.

| layout | system | 8 | 16 | 32 | 64 | 128 | 256 | 512 |
|---|---|---|---|---|---|---|---|---|
| dense | OpenCDA | 160 | 380 | 919 | 2151 | 4888 | 13954 | - |
| dense | OpenCDA, map manager off | 97 | 248 | 619 | 1439 | 3210 | 9827 | - |
| dense | eCAV, one host | 60 | 89 | 159 | 389 | 1078 | 2398 | - |
| dense | eCAV, three hosts | - | - | - | 278 | 712 | 1547 | 4150 |
| spread | OpenCDA | 287 | 513 | 1051 | 2566 | 5759 | 15273 | - |
| spread | OpenCDA, map manager off | 150 | 349 | 722 | 1862 | 4028 | 10975 | - |
| spread | eCAV, one host | 53 | 80 | 144 | 347 | 1039 | 2910 | - |
| spread | eCAV, three hosts | - | - | - | 276 | 733 | 1870 | 4780 |

Median tick wall time (ms) against the number of vehicles, perception on (four cameras and one LiDAR per vehicle, dense layout). OpenCDA at 64 vehicles: 40 measured ticks. OpenCDA at 128 vehicles does not start (the simulator ends with a segmentation fault at 22.1 GB of GPU memory).

| system | 8 | 16 | 32 | 64 | 128 |
|---|---|---|---|---|---|
| OpenCDA | 1596 | 3323 | 7213 | 15180 | - |
| eCAV, one host | 462 | 827 | 1616 | - | - |
| eCAV, distributed, 1 replica | 364 | 629 | 1245 | 2534 | 4233 |
| eCAV, distributed, 2 replicas | 247 | 441 | 858 | 1879 | 4232 |
| eCAV, distributed, 4 replicas | 244 | 440 | 859 | 1787 | 4156 |
| eCAV, distributed, 8 replicas | 245 | 442 | 870 | 1806 | 4202 |
