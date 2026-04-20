# Network-Aware Cooperative Intelligence for Connected Autonomous Vehicles
## Proposed Paper: V2V vs Edge Communication Architecture with ns-3 Co-Simulation

---

## Slide 1: The Problem (Set the stage)

**Title:** Why autonomous vehicles cannot act alone

- A single autonomous vehicle has blind spots: buildings, trucks, and other cars block its view of hazards
- At a four-way intersection, an occluded left-turn scenario causes roughly 40% of urban collisions (NHTSA pre-crash typology)
- The vehicle's onboard sensors (LiDAR, camera, radar) cannot see around corners
- Solution: **cooperative intelligence**. Vehicles and roadside units (RSUs) share observations so that an occluded hazard detected by one agent reaches every other agent's planner

**Speaker notes:**
- Frame the problem without assuming AV knowledge
- One vehicle = limited view; many vehicles + roadside cameras = complete view
- The question this presentation answers: *how should that sharing happen?*

---

## Slide 2: The Cooperative Intelligence Pipeline

**Title:** What "cooperative intelligence" actually means

The full stack that an autonomous vehicle needs:

1. **Perception**: detect objects in the scene (3D bounding boxes)
2. **Tracking**: maintain persistent identities over time (which box is the same car?)
3. **Prediction**: forecast where each object will be 2-5 seconds from now
4. **Planning**: decide steering, throttle, brake based on predicted future

*Prediction is the hardest part.* It requires history, context, and multi-agent reasoning. On-vehicle compute cannot run state-of-the-art predictors at 10 Hz over dozens of agents.

**Key insight:** if all agents share data with a central point, one compute node can run the expensive prediction once for the whole scene.

---

## Slide 3: Two Architectures for Sharing

**Title:** Where should the cooperative compute happen?

Two fundamentally different designs:

### Option A: Direct V2V (Vehicle-to-Vehicle)
- Cars talk directly to each other over wireless radio (no infrastructure needed)
- Each car receives data from nearby cars and runs its own fusion locally
- Examples: AutoCast (MobiSys 2022), F-Cooper (SEC 2019)

### Option B: Edge-Hosted (Vehicle-to-Infrastructure)
- Cars upload data to a server at the nearest cellular base station
- Server fuses everything, runs prediction, sends results back
- Examples: EMP (MobiCom 2021), Harbor (SenSys 2024)

**Both exist in the literature. Neither is definitively better. Why?**

---

## Slide 4: The Communication Layer Matters

**Title:** The problem is the wireless link

Both architectures depend on wireless V2X communication, but use it differently:

### V2V uses **PC5 sidelink**
- Direct car-to-car radio (3GPP 5G NR Release 16)
- Contention-based: cars compete for airtime, no central coordinator
- Low latency, but reliability degrades as more cars transmit

### Edge uses **Uu uplink**
- Standard cellular uplink (car to base station)
- Scheduled: base station allocates resources, no contention
- Higher bandwidth ceiling, but adds base station processing delay

**The choice of architecture is inseparable from the choice of link.**

---

## Slide 5: What Data Needs to Travel?

**Title:** Not all cooperative data is the same size

| Payload Type | Size | What it is |
|--------------|------|------------|
| Beacon | ~300 B | Vehicle's own position and velocity |
| Object list | 1-5 KB | Detected bounding boxes after local perception |
| Intermediate feature | 17-200 KB | Raw neural network BEV feature maps (compressed) |
| Raw sensor data | 1-10 MB | LiDAR point cloud, camera frames |

**Key observation:** intermediate feature sharing gives the best perception quality but is 100x larger than object lists. The wireless link has to carry this load.

**Figure placeholder:** Bar chart showing payload sizes on log scale

---

## Slide 6: The Gap in Prior Work

**Title:** Nobody has actually measured which architecture wins

Current state of the literature:

- **V2V papers** (AutoCast, F-Cooper) assume idealized sidelink or evaluate at 3-7 cars on a testbed. They do not characterize degradation at urban density.
- **Edge papers** (EMP, Harbor) assume a working uplink. EMP reports 93ms end-to-end at 3 vehicles. Harbor tests 3 Lincoln MKZ cars at Mcity.
- **Cooperative perception papers** (V2X-ViT, CoBEVT, Where2Comm) evaluate *detection accuracy* offline and ignore the network entirely.

**No paper has systematically measured:**
- V2V sidelink vs edge uplink feasibility as a function of payload size and agent density
- Where each architecture's breaking point is
- Which payload types should use which link

*This is our paper.*

---

## Slide 7: What is ns-3? (For systems readers unfamiliar)

**Title:** The tool we need: a realistic network simulator

**ns-3** is the standard discrete-event network simulator used across networking research (NSDI, SIGCOMM, SenSys):

- Open source, C++ core
- Models layer 1 through layer 7 of the network stack
- Simulates wireless physics (SINR, fading, path loss)
- Models MAC protocols, scheduling, packet queuing

**5G-LENA NR V2X** is the module inside ns-3 that implements 3GPP 5G NR:

- PC5 sidelink Mode 2 (sensing-based semi-persistent scheduling)
- Uu uplink with gNB scheduling
- Real radio propagation models

**Why ns-3 vs real hardware:** real 5G NR V2X testbeds at 30+ cars do not exist. Simulation is the only way to characterize dense-scale behavior.

---

## Slide 8: Our Approach: CARLA + ns-3 Co-Simulation

**Title:** How we integrate the network simulator with our CAV platform

### CARLA
- Open-source 3D driving simulator
- Simulates physics, sensors, traffic, road topology
- Produces ground-truth vehicle positions, LiDAR, camera

### eCAV (our platform)
- Built on CARLA
- Runs realistic autonomous driving stack (perception, tracking, prediction, planning)
- Distributed: each vehicle is a separate process

### ns-3 co-simulation bridge
- Per tick, CARLA publishes vehicle positions to ns-3 via shared memory
- ns-3 computes wireless link quality for every possible V2V and V2I pair
- ns-3 returns: which messages were delivered, their latency, their SINR
- eCAV drops or delays packets based on ns-3's decisions

**This is the integration we are building. No public CAV platform has it.**

**Figure placeholder:** Architecture diagram (CARLA ↔ eCAV ↔ ns-3 via shared memory)

---

## Slide 9: What the Paper Will Measure

**Title:** The experiments we will run

### Experiment 1: Feasibility vs payload size and N
- Sweep: N = 4, 8, 16, 32, 64 agents
- Payloads: 300 B, 5 KB, 17 KB, 200 KB
- Links: PC5 sidelink, Uu uplink, both
- Output: Packet Reception Ratio (PRR) heatmap

### Experiment 2: End-to-End Age of Information (AoI)
- Measure: vehicle sensor capture → edge fusion → result back at planner
- Breakdown: encoding + network + edge compute + decoding
- Compare: V2V direct vs edge-hosted for each payload type

### Experiment 3: Detection quality as a function of link reliability
- How does PRR degradation propagate into cooperative perception AP?
- Which payloads tolerate packet loss? (object lists yes, BEV features no)

### Experiment 4: Hybrid link design
- Architecture that routes small payloads over sidelink, large over Uu
- Compare against single-link baselines

---

## Slide 10: Expected Contributions

**Title:** What this paper adds to the literature

1. **The first systematic characterization** of V2V sidelink vs edge Uu uplink feasibility for cooperative perception at 4 to 64 agent density, using realistic 5G NR V2X simulation.

2. **An open-source CARLA + ns-3 co-simulation platform** that couples driving simulation with a standards-compliant wireless network simulator. No such platform exists publicly.

3. **A hybrid communication architecture** that assigns cooperative payloads to the appropriate link based on payload size and channel state, connecting the network layer to the edge compute deadline budget.

4. **Empirical evidence for the architectural choice** between V2V and edge-hosted cooperative intelligence, grounded in measured PRR, latency, and resulting perception quality.

---

## Slide 11: Why This Matters Beyond CAVs

**Title:** Connections to systems research

This paper touches several areas a systems audience cares about:

- **Edge computing resource allocation:** the edge has a compute budget. How much of that budget goes to network versus computation?
- **Co-simulation infrastructure:** bridging simulators is a classic systems problem (gem5, PANDA, shared-memory IPC). Our approach is reusable for other CPS domains.
- **ML serving under deadlines:** cooperative perception is a pipeline of neural network inferences with a hard real-time deadline. Same structure as Clockwork, INFaaS.
- **Distributed state consistency:** the edge maintains shared state (tracked objects, predicted trajectories) across mobile clients with unreliable links.

---

## Slide 12: Timeline and Deliverables

**Title:** Where we are and what's next

### Completed
- CARLA simulation platform (eCAV)
- Shared memory co-simulation bridge between CARLA and ns-3
- 5G-LENA NR V2X module integrated
- Analytical C-V2X channel model as fallback

### In progress
- Debugging the ns-3 subprocess crash under Uu uplink configuration
- PRR sweep across payload sizes and agent counts

### Next steps
- Full experiment sweeps (Experiments 1-4 above)
- Paper draft targeting SenSys 2027 or NSDI 2027
- Open source release of the co-simulation bridge

**Target venue:** SenSys 2027 (networked sensing + systems) or NSDI 2027 (networked systems)

---

## Slide 13: Questions / Discussion

**Title:** Open questions for discussion

- Is the CARLA + ns-3 co-simulation a standalone contribution, or should it be evaluation infrastructure for a bigger systems claim?
- Which venue best fits: networking (NSDI, SIGCOMM) or systems (SenSys, MobiSys)?
- Should we include real 5G hardware measurements as validation, or is simulation sufficient?
- What is the right baseline: existing V2V papers (AutoCast) or ideal-network cooperative perception (V2X-ViT)?
