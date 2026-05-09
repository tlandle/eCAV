# Project Architecture & Notes
## ecloudsim_distributed_sandbox

### Architecture Overview
ecloudsim_distributed_sandbox is a distributed extension of eCAV designed for scalable autonomous vehicle simulation.

Key Architectural Insights:
1. **Distributed Execution**: Moves from a single-process model to a distributed architecture where each vehicle or RSU can run in its own process or Docker container. Communication is handled via gRPC (defined in `ecloud.proto`).
2. **Orchestration**: `ecav.py` acts as either the central server (orchestrator) or a client. The server manages the simulation clock (`broadcast_tick`) and scenario state, while clients handle local perception and planning.
3. **Edge Computing & Fusion**: The system supports 'Edge' nodes that can aggregate data from multiple vehicles for collaborative perception (e.g., BM2CP, WorldFusion). This is reflected in the complex gRPC messages for intermediate features and fusion results.
4. **Cloud-Native Design**: Includes Ansible playbooks and Dockerfiles for deploying the simulation across a cluster of machines, likely for large-scale experiments.
5. **Hybrid Simulation**: Supports both sequential (legacy eCAV) and distributed modes, allowing for comparison and easier debugging of new algorithms.

### Key Files:
- `ecav.py`: Primary entry point for both the orchestrator (server) and vehicle clients in distributed mode.
- `ecav/protos/ecloud.proto`: Defines the gRPC communication interface between orchestrator, edge nodes, and vehicle clients.
- `ecav/scenario_testing/`: Contains scripts for running scenarios (e.g., `ecloud_4lane_scenario_dist_16_car.py`) and setting up distributed vehicle managers via `ScenarioManager`.

## Technical Writing Styles (OSDI/SOSP/SenSys/MobiCom/MobiSys)
- **Tone**: Formal, precise, objective, and neutral. It reads as a technical review or research paper.
- **Perspective**: Action-oriented but typically passive voice or first-person plural ("We evaluate", "Our system").
- **Structure**: Clear problem statements and distinct contributions. Strict categorization of topics: Abstract, Introduction, Background, System Design, Implementation, Evaluation, Discussion, Related Work, Conclusion.
- **Language Rules**:
  - Avoid hyperbole and qualitative descriptors (e.g., "significantly reduces"). Use quantitative results ("reduces latency by 45%").
  - Do not use sentences with a dangling 'this' (e.g., use 'this methodology' or 'this observation').
  - Minimize the "it's not this, it's that" phrasing.
  - Avoid overusing semicolons, colons, parentheses, or formatting (bold/italics/emphasis).
  - Concise language focused entirely on technical intent, rationale, and evaluation metrics.
- **Context Handling**: Always ground statements in collected data or reproducible steps, providing empirical evidence or clear methodology when verifying or defending claims.
