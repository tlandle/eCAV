# Azure Distributed Deployment Plan

**Status:** WIP / Exploratory  
**Branch:** `distributed-integration`  
**Created:** 2026-04-06

---

## Context

eCAV has previously been deployed to Azure (commented-out IPs in `cloud_config.yaml` confirm this), but deployment was manual and monolithic — a single `start_actors.sh` ran on one machine and spawned everything locally or into Docker containers with `--network=host`. The new topology separates five concerns across five node types, requiring a proper multi-node launch orchestration layer. This plan captures the architectural analysis, the (minimal) code change needed, and the proposed Ansible-based deployment structure.

---

## Hypothesis

The codebase already has most of the multi-node plumbing in place:
- `sim_api.py:580` already skips spawning `ecloud_server` when `ECLOUD_IP != 'localhost'`
- `cloud_config.yaml` already carries IPs for all roles
- `ecloud_actor_client.py` and `edge_process.py` both read `ECLOUD_IP`/`EDGE_IP` from `cloud_config.yaml`

**One code gap:** `ecav.py:43` hardcodes `ECLOUD_SERVER_ADDRESS = "localhost:50051"` — used by actor containers when they call `fetch_scenario_from_server()`. Node-3 and node-4 actors won't be able to reach node-1 unless this reads from config.

Everything else is a deployment / scripting problem.

---

## Target Topology

```
node-0   CARLA + ecav.py (scenario driver, no GPU required if headless)
           - Starts CARLA (optionally headless)
           - Runs: python ecav.py -t <scenario> -d [--apply_ml]
           - Connects to ecloud_server at node-1:50051
           - Connects to CARLA at localhost:2000

node-1   ecloud C++ orchestrator
           - Runs: ./ecav/ecloud_server/ecloud_server --minloglevel=1
           - Listens: :50051 (gRPC, actor registration + tick coordination)
           - Pushes ticks TO actors on node-3/node-4 via their push ports (50101+)
           - No CARLA dependency; no GPU required

node-2   LitServe inference server
           - Runs: worldfusion_grpc_server.py (:18002) + yolo_grpc_server.py (:18001)
           - Optionally: litserve_models.py (:18000) for HTTP fallback
           - GPU required (inference)
           - Accessed by actor processes on node-3

node-3   GPU actors: ego vehicles, RSUs
           - Runs Docker containers: ecav.py -d -i <index> --apply_ml [-l]
           - Connects to ecloud_server at node-1:50051
           - Connects to ML inference at node-2:18001 / node-2:18002
           - Exposes push ports 50101+ to node-1

node-4   CPU actors: non-ego vehicles, edges without GPU
           - Runs Docker containers: ecav.py -d -i -1 | edge_process.py -e <idx>
           - Connects to ecloud_server at node-1:50051
           - Exposes push ports 50101+ to node-1
```

---

## Azure Networking

### VNet / Subnet
All nodes on same Azure VNet (e.g., `10.0.0.0/16`), single subnet or segmented by role.
Private IPs stable (static or reserved DHCP). Public IPs for SSH only.

### NSG Inbound Rules per Node

| Node | Ports | From |
|------|-------|------|
| node-0 | 2000 (CARLA Python API), 22 | VNet + admin IP |
| node-1 | 50051 (orchestrator gRPC), 22 | VNet |
| node-2 | 18000 (LitServe), 18001 (YOLO gRPC), 18002 (WF gRPC), 22 | VNet |
| node-3 | 50101–50200 (actor push servers), 8000–8099 (actor ports), 22 | VNet |
| node-4 | 50101–50200 (actor push servers), 8100–8199 (non-ego + edge), 50054–50060 (edge gRPC), 22 | VNet |

### Recommended VM SKUs
- node-0: `Standard_NV6ads_A10_v5` (GPU for CARLA display) or any DS-series if headless
- node-1: `Standard_D4s_v3` (CPU only, low cost)
- node-2: `Standard_NC4as_T4_v3` (T4 GPU, cost-efficient for inference)
- node-3: `Standard_NC4as_T4_v3` (GPU for ego/RSU perception)
- node-4: `Standard_D4s_v3` (CPU only)

---

## Code Changes Required

### 1. `ecav/ecav.py:43` — Fix hardcoded ECLOUD_SERVER_ADDRESS

```python
# Before:
ECLOUD_SERVER_ADDRESS = "localhost:50051"

# After:
import yaml as _yaml
_cloud_cfg = _yaml.safe_load(open('cloud_config.yaml')) if os.path.exists('cloud_config.yaml') else {}
ECLOUD_SERVER_ADDRESS = f"{_cloud_cfg.get('ecloud_server_public_ip', 'localhost')}:50051"
```

This is the only code change needed. Everything else is already wired for multi-node.

---

## Deployment Structure

```
deploy/
  ansible/
    inventory/
      azure.ini.template     # template with node role groups
    group_vars/
      all.yml               # port constants, conda env name, repo path
    roles/
      common/               # git pull, conda env check, shared setup
      carla/                # start/stop CARLA
      ecloud_server/        # start/stop ecloud_server daemon
      inference/            # start/stop worldfusion + yolo gRPC servers
      gpu_actors/           # launch ego/RSU Docker containers
      cpu_actors/           # launch non-ego/edge Docker containers
    site.yml                # master playbook: role assignments per host group
    start.yml               # convenience: full cluster start
    stop.yml                # convenience: full cluster stop
  scripts/
    node-0-start.sh         # CARLA + ecav.py (extracted from start_actors.sh)
    node-1-start.sh         # standalone ecloud_server
    node-2-start.sh         # inference servers
    node-3-start.sh         # GPU actor containers
    node-4-start.sh         # CPU actor containers
    cluster-stop.sh         # calls stop on all nodes via SSH
  cloud_config.azure.yaml.j2   # Jinja2 template rendered per-node by Ansible
```

### Inventory template (`ansible/inventory/azure.ini.template`)
```ini
[carla_node]
node-0 ansible_host={{ node0_ip }}

[orchestrator]
node-1 ansible_host={{ node1_ip }}

[inference]
node-2 ansible_host={{ node2_ip }}

[gpu_actors]
node-3 ansible_host={{ node3_ip }}

[cpu_actors]
node-4 ansible_host={{ node4_ip }}
```

### `ansible/group_vars/all.yml`
Carries: repo path, conda env name, port constants, cloud_config.yaml template variables.
Ansible renders `cloud_config.yaml` per-node from a Jinja2 template:
- `node-0`: carla_ip=localhost, ecloud_ip=node-1's private IP
- `node-1`: ecloud_server itself doesn't need cloud_config
- `node-2`: standalone inference servers, cloud_config not required
- `node-3/4`: carla_ip=node-0, ecloud_ip=node-1, vehicle_client_ip=this node's private IP

### Role responsibilities

**`common`**: `git pull`, check conda env, ensure repo root is CWD.

**`carla`**:
- Start: `./CarlaUE4.sh -RenderOffScreen -carla-rpc-port=2000`
- Stop: `pkill CarlaUE4`

**`ecloud_server`**:
- Start: `./ecav/ecloud_server/ecloud_server --minloglevel=1 &`
- Health check: `nc -z localhost 50051`
- Stop: `pkill ecloud_server`

**`inference`**:
- Start: `worldfusion_grpc_server.py` + `yolo_grpc_server.py` (readiness check from `start_actors.sh`)

**`gpu_actors`**:
- Reads actor count from scenario or env var
- Launches Docker containers per ego/RSU:
  ```
  docker run --network=host --runtime=nvidia ...
    python3.10 ecav.py --apply_ml -l -d -i <idx>
  ```
- Waits for `"Registered with"` in docker logs

**`cpu_actors`**:
- Non-ego vehicles: `ecav.py -d -i -1`
- Edge processes: `edge_process.py -e <idx> -P <port> -O <node-1-ip>`

---

## Per-Node Scripts (shape, not final)

**`node-1-start.sh`**:
```bash
#!/bin/bash
cd /path/to/ecav
./ecav/ecloud_server/ecloud_server --minloglevel=1 &
echo $! > /tmp/ecloud_server.pid
```

**`node-2-start.sh`**:
```bash
#!/bin/bash
cd /path/to/ecav
conda run -n opencda python ecav/ml_manager/worldfusion_grpc_server.py &> /tmp/wf_grpc.log &
conda run -n opencda python ecav/ml_manager/yolo_grpc_server.py &> /tmp/yolo_grpc.log &
# readiness check logic from start_actors.sh
```

---

## Configuration Management

`cloud_config.yaml` is the single source of truth for IPs. Ansible generates it per-node from `cloud_config.azure.yaml.j2`. For manual runs, the commented-out Azure IPs already in `cloud_config.yaml` serve as the reference pattern. No structural changes to the config format.

---

## Open Questions

1. **Model weights / checkpoints**: local copy per GPU node, or shared Azure Files (NFS)? Shared is simpler operationally but adds a network dependency on every inference call startup.

2. **Log aggregation**: per-node `/tmp` logs sufficient for now, or centralized collection from the start (Azure Monitor, or periodic rsync to node-0)?

3. **CARLA TrafficManager port (`-T`)**: when actors run on node-3 and CARLA is on node-0, the TM client connects to `carla_server_public_ip:8000`. Needs verification that this is wired correctly vs. requiring a separate config knob.

---

## Checklist

### Phase 1: Code Fix
- [ ] Fix `ecav.py:43` — read `ECLOUD_SERVER_ADDRESS` from `cloud_config.yaml`
- [ ] Smoke-test locally: `cloud_config.yaml` with localhost still works

### Phase 2: Per-Node Scripts
- [ ] Create `deploy/scripts/node-0-start.sh` (CARLA + ecav.py)
- [ ] Create `deploy/scripts/node-1-start.sh` (standalone ecloud_server)
- [ ] Create `deploy/scripts/node-2-start.sh` (inference servers + readiness check)
- [ ] Create `deploy/scripts/node-3-start.sh` (GPU actor containers)
- [ ] Create `deploy/scripts/node-4-start.sh` (CPU actor containers)
- [ ] Create `deploy/scripts/cluster-stop.sh`

### Phase 3: Ansible Skeleton
- [ ] Create `deploy/ansible/inventory/azure.ini.template`
- [ ] Create `deploy/ansible/group_vars/all.yml`
- [ ] Create `deploy/ansible/roles/` (common, carla, ecloud_server, inference, gpu_actors, cpu_actors)
- [ ] Create `deploy/ansible/site.yml` + `start.yml` + `stop.yml`
- [ ] Create `deploy/cloud_config.azure.yaml.j2`

### Phase 4: Verification
- [ ] Validate on local multi-process setup (simulate nodes via different ports)
- [ ] Deploy to Azure — confirm each node starts independently
- [ ] Run `openscenario_3_edge_worldfusion -d` across 5 nodes end-to-end
- [ ] Confirm metrics collection works end-to-end

---

## Verification Plan

1. **Local smoke test (Phase 1)**: Set `cloud_config.yaml` to a non-localhost ECLOUD_IP → confirm `sim_api.py` skips spawning ecloud_server, `ecav.py` actor path uses the right address.

2. **Two-machine test**: node-0 + node-1 on two local VMs. Confirm ecloud_server on node-1 receives registrations from ecav.py on node-0.

3. **Full 5-node Azure run**: Deploy via Ansible, run scenario, check per-node logs. Measure tick latency vs. single-machine baseline.

---

## Related Files
- `cloud_config.yaml` — existing multi-IP pattern (commented-out Azure IPs)
- `start_actors.sh` — source of readiness-check logic to extract into node scripts
- `ecav/scenario_testing/utils/sim_api.py:580` — existing remote orchestrator guard
- `ecav.py:43` — the one hardcoded address that needs fixing
