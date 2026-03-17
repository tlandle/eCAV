# Design Description for Deployment of Multi-vehicle Simulation

![Design document](https://github.com/tlandle/eCAV/blob/dist/deployment/deployment.png)

## Virtual Machines

The deployment of a multi-vehicle simulation will involve several VM types:
1. Carla server - will need to be installed on a virtual machine with a GPU. This VM is running the Carla server and doing simulation rendering.
2. eCAV - the eCAV framework needs to be installed on a VM, which is the master for the simulation. This VM is responsible for initiating the simulation, connecting to each simulation vehicle to pass simulation parameters, and for the "tick" of the simulation to cause it to advance a single time step.
3. Worker nodes - we will use multiple worker nodes that can host one or more Docker containers for the simulated vehicle. Simulation Vehicle can use a neural network for doing perception of other vehicles, therefore worker nodes will need to be built on a GPU VM.

We will manually create a VM for the Carla server and eCAV. We will create an image of a worker node to encapsulate our desired worker node installation. Simulation Vehicle can run in two modes - *apply_ml=true* uses a neural network (Yolov5) to perform perception of other vehicles, *apply_ml=false* reads information directly from the Carla server to find information on other vehicles. We will want two images - a GPU based image when *apply_ml* is true, and a non-GPU image when *apply_ml* is false. Given heavy restrictions on access to GPUs within Azure for research grants currently, I'd recommend supporting non-GPU based simulations also.

We will use docker_machine to create new workers from our images, to deploy our Docker container image to the worker, as well as starting/stopping the containers when we want to run a simulation. 

## eCAV Master

The eCAV framework is installed on a VM. This can be installed on the same VM as the Carla server, or separately. The eCAV framework (ecav.py) reads a deployment.yml configuration file that provides parameters connection information for each Simulation Vehicle Docker container.

eCAV uses a REQ/RESP pattern from 0MQ to connect to each Simulation Vehicle, pass parameters for the simulation run, and make RPC calls to the vehicle to control the simulation. At the end of the simulation, it is responsible for evaluating and reporting out on the success of the simulation run.

## Simulation Vehicle

An eCAV vehicle that runs in a separate process (sim_vehicle.py), and has been containerised. We can deploy multiple containers to a VM, and determine how many containers can be supported by a single vehicle while still getting good performance, and also deploy containers to new VMs to scale further. 

sim_vehicle.py has a single command line parameter to set the listening port (-p). The default port to listen on is 5555. When the simulation vehicle starts, it immediately starts listening for the eCAV master to connect to it. When the master connects, it sends a START command, along with the parameters for this simulation run. Parameters include:

* test scenario - the yaml configuration file for this simulation to be run. Note initially we assume the yaml file exists on the container, but an improvement would be to pass the contents of the yaml file in another RPC call so that the simulation can be easily changed without modifying the container.
* Vehicle index - the identifying in the simulation yaml file for this simulation vehicle to read. This configuration determines the start spawn location and behavior for this simulated vehicle
* Version - which version of Carla we are running. Currently eCAV supports 0.9.11 and 0.9.12.
* Application - an array of applications that this vehicle should use. For example, "single", "platoon" and "edge".

## Logging

We will use Docker Logs to see real-time stdout and stderr content from terminals on the eCAV VM. We may want to explore using a framework for automatic logging collection and analysis. For example, Graylog and Fluentd seem to be popular frameworks used with Docker.

## Orchestration

Docker has a built in orchestration mechanism called Docker Swarm. If orchestration of the simulation vehicles proves to be difficult to manage, we could look into Docker Swarm as a way to manage large scale deployments of containers.

# Discovery

we could investigate using Zookeeper as a centralized way for Simulation Vehicles to register themselves when they start, so that the eCAV master can automatically find each container and we can avoid having to modify the deployment.yml file with new IP addresses each time the simulation is started or stopped. If we have fairly static IP addresses betweek simulation runs, this might not be needed.
