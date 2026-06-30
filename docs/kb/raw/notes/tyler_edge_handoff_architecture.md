# Notes: Tyler - Architecture Of Edge State Hand-Off

**Date:** 2026.06.07
**Source**: Weekly Meeting

---

## State Transfer And Storage

For distributed scenarios, the C++ services-layer server will store the current state for all vehicles in a buffer. For sequential scenarios, the `ecav.py` process itself will store current state in a `sim_api.py` dict. It is not required for an initial implementation for these two data sinks to be kept in sync in any way. But we do not want to preclude that the root `ecav.py` process might also wish to query for vehicle state for research purposes in a distributed sim. However, such an endpoint should necessarily exist since state retrieval is a requirement for individual edges.

The associated gRPC message should support storage of either a full or partial state snapshot. It is fine to store these state objects as is - `protobuf` objects.

Assume for `v1` that we simply store a full snapshot for every vehicle every tick.

Edges should be able to retrieve all state on the subsequent tick (single barrier), though a double-barrier design where every edge retrieves all state at the end of a tick is also possible.

The driving rationale here is that management of state storage is an artifact of simulator design rather than being representative of real world implementation. The goal is to separate out the actual simulated components - from a research perspective - from the simulator itself.

---

## Edge-To-Edge Communications

The actual latency and bandwidth costs associated with such transfer and storage are representative of simulator - not simulation - architecture. The actual simulation architecture of edge-to-edge communication can be trivially simple - a basic ping/ack message is all that is required for vehicle hand-off - from a simulation perspective while still being modeled for rich complexity in terms of network latency and serialization costs. Even if the receiving edge has the full vehicle state, we can still model both the serialization/deserialization costs irrespective of whether or not they are _required_ from the standpoint of the simulation itself.

Nothing about the absence of needing to send the underlying data - and modeling that in a specific way - needs to affect the actual simulation implementation of such transfer and vice versa.

This is most clearly obvious in a sequential simulation where all memory can be shared instantly. We can still layer latency and serialization modeling costs on top of this instant transfer.

The actual cost within the simulation to serialize-transfer-deserialize the data must not be conflated with the actual simulated measurement of that same process. In the former case, the actual location - physically - of edge nodes will be the dominant factor; whereas in the simulation it will be the simulated location of edge nodes that will be the dominant factor. 

---

## Implementation 

### Sequential Simulation

- vehicle state snapshots stored in dict in `sim_api.py`
- vehicle state snapshot stored each tick
- individual edges can pull instantly
- edge-to-edge transfer does not need to be an RPC; can just be a simple instance member-function call
	- state transfer is a simple ping - ack: "edge_0 transferring ownership of vehicle_0 to edge-1" : "edge_1 acknowledges vehicle_0 ownership edge_0"
- assume pickled state object
- ability to measure and store simulated serialization and networking costs of state transfer

### Distributed Edge-Only `-eo` Simulation

- same as sequential simulation but serialized array of vehicle state snapshot(s) included with every do tick message from `sim_api.py` to individual edges
- edge-to-edge transfer now must be an RPC, but no more complicated than before
	- transfer message can be expanded to included simulation processing characteristics: latency, etc

### Fully Distributed Simulation

- transfer and retrieval of state snapshots moved to C++ server