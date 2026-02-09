/****************************************************************************
 Copyright (c) 2023 Georgia Institute of Technology

 Permission is hereby granted, free of charge, to any person obtaining a copy
 of this software and associated documentation files (the "Software"), to deal
 in the Software without restriction, including without limitation the rights to
 use, copy, modify, merge, publish, distribute, sublicense, and/or sell copies
 of the Software, and to permit persons to whom the Software is furnished to do so,
 subject to the following conditions:

 The above copyright notice and this permission notice shall be included in
 all copies or substantial portions of the Software.

 THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
 THE SOFTWARE.
****************************************************************************/


#include <iostream>
#include <memory>
#include <string>
#include <atomic>
#include <mutex>
#include <condition_variable>
#include <thread>
#include <cassert>
#include <stdexcept>
#include <errno.h>
#include <csignal>
#include <unistd.h>
#include <chrono>
#include <map>
#include <vector>

#include "absl/flags/flag.h"
#include "absl/flags/parse.h"
#include "absl/strings/str_format.h"
#include "absl/log/log.h"
#include "absl/log/flags.h"
#include "absl/log/initialize.h"
#include "absl/log/globals.h"

#include <grpcpp/ext/proto_server_reflection_plugin.h>
#include <grpcpp/grpcpp.h>
#include <grpcpp/health_check_service_interface.h>

#include <google/protobuf/util/time_util.h>
#include <google/protobuf/util/json_util.h>

#include "ecloud.grpc.pb.h"
#include "ecloud.pb.h"

//#include <glog/logging.h>

#define WORLD_TICK_DEFAULT_MS 50
#define SLOW_CAR_COUNT 0
#define SPECTATOR_INDEX 0
#define VERBOSE_PRINT_COUNT 5
#define MAX_CARS 512
#define INVALID_TIME 0
#define TICK_ID_INVALID -1
#define VEHICLE_UPDATE_BATCH_SIZE 32

#define ECLOUD_PUSH_BASE_PORT 50101
#define ECLOUD_PUSH_API_PORT 50061

ABSL_FLAG(uint16_t, port, 50051, "Sim API server port for the service");
ABSL_FLAG(uint16_t, minloglevel, static_cast<uint16_t>(absl::LogSeverityAtLeast::kInfo),
          "Messages logged at a lower level than this don't actually "
          "get logged anywhere");

using google::protobuf::util::TimeUtil;

using grpc::CallbackServerContext;
using grpc::Server;
using grpc::ServerBuilder;
using grpc::ServerUnaryReactor;
using grpc::Status;

using ecloud::Ecloud;
using ecloud::EcloudResponse;
using ecloud::VehicleUpdate;
using ecloud::Empty;
using ecloud::Tick;
using ecloud::Command;
using ecloud::VehicleState;
using ecloud::SimulationInfo;
using ecloud::RegistrationInfo;
using ecloud::WaypointBuffer;
using ecloud::Waypoint;
using ecloud::Transform;
using ecloud::Location;
using ecloud::Rotation;
using ecloud::LocDebugHelper;
using ecloud::AgentDebugHelper;
using ecloud::PlanerDebugHelper;
using ecloud::ClientDebugHelper;
using ecloud::Timestamps;
using ecloud::WaypointRequest;
using ecloud::EdgeWaypoints;
using ecloud::EdgeObjects;
using ecloud::ObjectBuffer;
using ecloud::EdgeObstacleObject;
using ecloud::ObjectRequest;
using ecloud::ActorType;
using ecloud::ScenarioRequest;
using ecloud::EdgeRegistrationInfo;
using ecloud::EdgeScenarioConfig;
using ecloud::EdgeTickComplete;
using ecloud::EdgeTick;
using ecloud::EdgeIndex;
using ecloud::ActorConnectionInfo;
using ecloud::EdgeMapping;
using ecloud::EdgeMappingSetup;

std::atomic<int16_t> numCompletedVehicles_;
std::atomic<int16_t> numRepliedVehicles_;
std::atomic<int32_t> tickId_;
std::atomic<bool> pushedTick_;

bool repliedCars_[MAX_CARS];
bool repliedRsus[MAX_CARS];
std::string carNames_[MAX_CARS];

bool init_;
bool isEdge_;
int16_t numCars_;
std::string configYaml_;
std::string application_;
std::string version_;

std::string simIP_;

VehicleState vehState_;
Command command_;

std::vector<std::pair<int16_t, std::string>> serializedEdgeWaypoints_; // vehicleIdx, serializedWPBuffer
std::vector<std::pair<int16_t, std::string>> serializedEdgeObjects_; // vehicleIdx, serializedObjBuffer

// Edge architecture state
struct EdgeInfo {
    int32_t edge_index;
    std::string edge_ip;
    int32_t edge_port;
    int32_t num_vehicles;
    int32_t num_rsus;
    std::string container_name;
    std::string edge_config_yaml;  // JSON of edge-specific YAML section
    std::vector<int32_t> vehicle_indices;  // Global vehicle indices owned by this edge
    std::vector<int32_t> rsu_indices;      // Global RSU indices owned by this edge
};

std::vector<EdgeInfo> edgeInfos_;  // Registered edges
std::map<int32_t, int32_t> vehicleToEdgeMapping_;  // vehicle_index -> edge_index
std::map<int32_t, int32_t> rsuToEdgeMapping_;      // rsu_index -> edge_index
std::atomic<int16_t> numRegisteredEdges_;
std::atomic<int16_t> numCompletedEdges_;
bool hasEdges_;  // True if scenario has edges (from sim_api.py)
int16_t numExpectedEdges_;  // Expected number of edges to register

absl::Mutex mu_;

volatile std::atomic<int16_t> numRegisteredVehicles_ ABSL_GUARDED_BY(mu_);
std::vector<std::string> pendingReplies_ ABSL_GUARDED_BY(mu_); // TODO: Move to a hashmap serialized protobuf allows differing message types in same vector

class PushClient
{
    public:
        explicit PushClient( std::shared_ptr<grpc::Channel> channel, std::string connection ) :
                            stub_(Ecloud::NewStub(channel)), connection_(connection) {}

        bool PushTick(int32_t tickId, Command command, int64_t lastClientDurationNS)
        {
            Tick tick;
            tick.set_tick_id(tickId);
            tick.set_command(command);

            LOG_IF(INFO, command == Command::END) << "pushing END";

            tick.set_last_client_duration_ns(lastClientDurationNS);
            
            grpc::ClientContext context;
            Empty empty;

            // The actual RPC.
            std::mutex mu;
            std::condition_variable cv;
            bool done = false;
            Status status;
            stub_->async()->PushTick(&context, &tick, &empty,
                            [&mu, &cv, &done, &status](Status s) {
                            status = std::move(s);
                            std::lock_guard<std::mutex> lock(mu);
                            done = true;
                            cv.notify_one();
                            });

            std::unique_lock<std::mutex> lock(mu);
            while (!done) {
                cv.wait(lock);
            }

            // Act upon its status.
            if (status.ok()) {
                return true;
            } else {
                LOG(ERROR) << status.error_code() << ": " << status.error_message();
                return false;
            }
        }

    private:
        std::unique_ptr<Ecloud::Stub> stub_;
        std::string connection_;
};

// Logic and data behind the server's behavior.
class EcloudServiceImpl final : public Ecloud::CallbackService {
public:
    explicit EcloudServiceImpl() {
        if ( !init_ )
        {
            numCompletedVehicles_.store(0);
            numRepliedVehicles_.store(0);
            numRegisteredVehicles_.store(0);
            tickId_.store(0);

            vehState_ = VehicleState::REGISTERING;
            command_ = Command::TICK;

            numCars_ = 0;
            configYaml_ = "";
            isEdge_ = false;

            // Edge architecture initialization
            numRegisteredEdges_.store(0);
            numCompletedEdges_.store(0);
            hasEdges_ = false;
            numExpectedEdges_ = 0;
            edgeInfos_.clear();
            vehicleToEdgeMapping_.clear();
            rsuToEdgeMapping_.clear();

            simIP_ = "localhost";

            const std::string connection = absl::StrFormat("%s:%d", simIP_, ECLOUD_PUSH_API_PORT );
            simAPIClient_ = new PushClient(grpc::CreateChannel(connection, grpc::InsecureChannelCredentials()), connection);

            vehicleClients_.clear();
            edgeClients_.clear();
            pendingReplies_.clear();

            init_ = true;
        }
    }

    ServerUnaryReactor* Server_GetVehicleUpdates(CallbackServerContext* context,
                               const Empty* empty,
                               EcloudResponse* reply) override {

        DLOG(INFO) << "Server_GetVehicleUpdates - deserializing updates.";

        const int16_t replies = pendingReplies_.size();
        for ( int i = 0; i < replies; i++ )
        {
            VehicleUpdate *update = reply->add_vehicle_update();
            const std::string msg = pendingReplies_.back();
            pendingReplies_.pop_back();
            update->ParseFromString(msg);
            LOG(INFO) << "update: vehicle_index - " << update->vehicle_index();

            if ( i == VEHICLE_UPDATE_BATCH_SIZE ) // keep from exhausting resources
                break;
        }

        DLOG(INFO) << "Server_GetVehicleUpdates - updates deserialized.";

        if ( pendingReplies_.size() == 0 )
            numRepliedVehicles_ = 0;
    
        ServerUnaryReactor* reactor = context->DefaultReactor();
        reactor->Finish(Status::OK);
        return reactor;
    }

    ServerUnaryReactor* Client_SendUpdate(CallbackServerContext* context,
                               const VehicleUpdate* request,
                               Empty* empty) override {

        if ( isEdge_ || request->vehicle_index() == SPECTATOR_INDEX || request->vehicle_state() == VehicleState::TICK_DONE || request->vehicle_state() == VehicleState::DEBUG_INFO_UPDATE )
        {
            std::string msg;
            request->SerializeToString(&msg);
            if ( isEdge_ || request->vehicle_state() == VehicleState::TICK_DONE || request->vehicle_state() == VehicleState::DEBUG_INFO_UPDATE )
            {
                // TODO: hashmap
                mu_.Lock();
                pendingReplies_.push_back(msg);
                mu_.Unlock();
            }
            else
            {
                assert( request->vehicle_index() == SPECTATOR_INDEX );
                pendingReplies_.push_back(msg);
            }
        }

        if ( request->actor_type() == ActorType::VEHICLE )
            repliedCars_[request->vehicle_index()] = true; // DEBUGGING
        else
            repliedRsus[request->vehicle_index()] = true; // DEBUGGING

        if ( request->vehicle_state() == VehicleState::TICK_DONE )
        {
            numCompletedVehicles_++;
            DLOG(INFO) << "Client_SendUpdate - TICK_DONE - tick id: " << request->tick_id() << " vehicle id: " << request->vehicle_index();
        }
        else if ( request->vehicle_state() == VehicleState::TICK_OK )
        {
            numRepliedVehicles_++;
            DLOG(INFO) << "Client_SendUpdate - TICK_OK - tick id: " << request->tick_id() << " vehicle id: " << request->vehicle_index();
        }
        else if ( request->vehicle_state() == VehicleState::DEBUG_INFO_UPDATE )
        {
            numCompletedVehicles_++;
            DLOG(INFO) << "Client_SendUpdate - DEBUG_INFO_UPDATE - tick id: " << request->tick_id() << " vehicle id: " << request->vehicle_index();
        }
    
        const bool complete = ( numRepliedVehicles_.load() + numCompletedVehicles_.load() ) == numCars_;
        if ( complete && !pushedTick_ )
        {
            pushedTick_ = true;
            const int64_t lastClientDurationNS = request->duration_ns();
            simAPIClient_->PushTick( request->tick_id(), command_, lastClientDurationNS );
            LOG(INFO) << "tick " << request->tick_id() << " COMPLETE";
        }
        
        ServerUnaryReactor* reactor = context->DefaultReactor();
        reactor->Finish(Status::OK);
        return reactor;
    }

    // server can push WP *before* ticking world and client can fetch them before it ticks
    ServerUnaryReactor* Client_GetWaypoints(CallbackServerContext* context,
                               const WaypointRequest* request,
                               WaypointBuffer* buffer) override {

        for ( int i = 0; i < serializedEdgeObjects_.size(); i++ )
        {
            const std::pair<int16_t, std::string > wpPair = serializedEdgeWaypoints_[i];
            if ( wpPair.first == request->vehicle_index() )
            {
                buffer->set_vehicle_index(request->vehicle_index());
                WaypointBuffer waypointBuf;
                LOG(INFO) << "Requesting vehicle " << request->vehicle_index() << " waypoints starting parse";
                const std::string buf = wpPair.second;
                waypointBuf.ParseFromString(buf);
                LOG(INFO) << "Requesting vehicle " << request->vehicle_index() << " waypoints parsed";
                for ( Waypoint wp : waypointBuf.waypoint_buffer())
                {
                    Waypoint *p = buffer->add_waypoint_buffer();
                    p->CopyFrom(wp);
                    LOG(INFO) << "Requesting vehicle " << request->vehicle_index() << " single waypoint copied";
                }
                LOG(INFO) << "Requesting vehicle " << request->vehicle_index() << " all waypoints copied";
                break;
            }
        }
        LOG(INFO) << "vehicle " << request->vehicle_index() << " waypoints sent";


        ServerUnaryReactor* reactor = context->DefaultReactor();
        reactor->Finish(Status::OK);
        return reactor;
    }

    //
    // Clients grab objects before tick. 
    //
    ServerUnaryReactor* Client_GetObjects(CallbackServerContext* context,
                               const ObjectRequest* request,
                               ObjectBuffer* buffer) override {

        for ( int i = 0; i < serializedEdgeObjects_.size(); i++ )
        {
            const std::pair<int16_t, std::string > objPair = serializedEdgeObjects_[i];
            if ( objPair.first == request->vehicle_index() )
            {
                buffer->set_vehicle_id(request->vehicle_index());
                ObjectBuffer objBuf;
                LOG(INFO) << "Requesting vehicle " << request->vehicle_index() << " objects starting parse";
                const std::string buf = objPair.second;
                objBuf.ParseFromString(buf);
                LOG(INFO) << "Requesting vehicle " << request->vehicle_index() << " objects parsed";
                buffer->set_pickled_edge_predictions(objBuf.pickled_edge_predictions());
                LOG(INFO) << "Requesting vehicle " << request->vehicle_index() << " all objects copied";
                break;
            }
        }
        LOG(INFO) << "vehicle " << request->vehicle_index() << " objects sent";


        ServerUnaryReactor* reactor = context->DefaultReactor();
        reactor->Finish(Status::OK);
        return reactor;
    } 

    ServerUnaryReactor* Client_RegisterVehicle(CallbackServerContext* context,
                               const RegistrationInfo* request,
                               SimulationInfo* reply) override {

        assert( configYaml_ != "" );

        if ( request->vehicle_state() == VehicleState::REGISTERING )
        {
            DLOG(INFO) << "got a registration update";

            mu_.Lock();
            const int16_t vIdx = numRegisteredVehicles_.load();
            reply->set_vehicle_index(vIdx);
            const std::string connection = absl::StrFormat("%s:%d", request->vehicle_ip(), request->vehicle_port());
            PushClient *vehicleClient = new PushClient(grpc::CreateChannel(connection, grpc::InsecureChannelCredentials()), connection);
            vehicleClients_.push_back(std::move(vehicleClient));
            numRegisteredVehicles_++;
            mu_.Unlock();

            reply->set_test_scenario(configYaml_);
            reply->set_application(application_);
            reply->set_version(version_);

            DLOG(INFO) << "RegisterVehicle - REGISTERING - container " << request->container_name() << " got vehicle id: " << reply->vehicle_index();

            carNames_[reply->vehicle_index()] = request->container_name();
        }
        else if ( request->vehicle_state() == VehicleState::CARLA_UPDATE )
        {
            const int16_t vIdx = request->vehicle_index();
            reply->set_vehicle_index(vIdx);

            DLOG(INFO) << "RegisterVehicle - CARLA_UPDATE - vehicle_index: " << vIdx << " | actor_id: " << request->actor_id() << " | vid: " << request->vid();

            // TODO: Hashmap
            mu_.Lock();
            std::string msg;
            request->SerializeToString(&msg);
            pendingReplies_.push_back(msg);
            numRepliedVehicles_++;
            mu_.Unlock();
        }
        else
        {
            assert(false);
        }

        const int16_t replies = numRepliedVehicles_.load();
        LOG(INFO) << "received " << numRegisteredVehicles_.load() << " registrations";
        LOG(INFO) << "received " << replies << " replies with Carla data";
        const bool complete = ( replies == numCars_ );

        LOG_IF(INFO, complete ) << "REGISTRATION COMPLETE";
        if ( complete )
        {
            assert( vehState_ == VehicleState::REGISTERING && replies == pendingReplies_.size() );
            simAPIClient_->PushTick( TICK_ID_INVALID, command_, INVALID_TIME );
        }

        ServerUnaryReactor* reactor = context->DefaultReactor();
        reactor->Finish(Status::OK);
        return reactor;
    }

    ServerUnaryReactor* Client_GetScenario(CallbackServerContext* context,
                               const ScenarioRequest* request,
                               SimulationInfo* reply) override {

        LOG(INFO) << "Client_GetScenario - request from vehicle_index: " << request->vehicle_index();

        reply->set_test_scenario(configYaml_);
        reply->set_application(application_);
        reply->set_version(version_);
        reply->set_is_edge(isEdge_);
        reply->set_carla_ip(simIP_);

        DLOG(INFO) << "Client_GetScenario - returning scenario: " << configYaml_;

        ServerUnaryReactor* reactor = context->DefaultReactor();
        reactor->Finish(Status::OK);
        return reactor;
    }

    ServerUnaryReactor* Server_DoTick(CallbackServerContext* context,
                               const Tick* request,
                               Empty* empty) override {
        for ( int i = 0; i < numCars_; i++ )
        {
            repliedCars_[i] = false;
            repliedRsus[i] = false;
        }

        pushedTick_ = false;
        numRepliedVehicles_ = 0;
        numCompletedEdges_ = 0;  // Reset edge completion counter
        assert(tickId_ == request->tick_id() - 1);
        tickId_++;
        command_ = request->command();

        const auto now = std::chrono::system_clock::now();
        DLOG(INFO) << "received new tick " << request->tick_id() << " at " << std::chrono::duration_cast<std::chrono::milliseconds>(
            now.time_since_epoch()).count();

        const int32_t tickId = request->tick_id();

        // If we have edges, push ticks to edges instead of individual vehicles
        if (hasEdges_ && edgeClients_.size() > 0) {
            LOG(INFO) << "pushing tick " << tickId << " to " << edgeClients_.size() << " edges";
            for ( int i = 0; i < edgeClients_.size(); i++ )
            {
                PushClient *e = edgeClients_[i];
                std::thread t( &PushClient::PushTick, e, tickId, command_, INVALID_TIME );
                t.detach();
            }
        } else {
            // No edges, push directly to vehicles (existing behavior)
            for ( int i = 0; i < vehicleClients_.size(); i++ )
            {
                PushClient *v = vehicleClients_[i];
                std::thread t( &PushClient::PushTick, v, tickId, command_, INVALID_TIME );
                t.detach();
            }
        }

        ServerUnaryReactor* reactor = context->DefaultReactor();
        reactor->Finish(Status::OK);
        return reactor;
    }


    ServerUnaryReactor* Server_PushEdgeWaypoints(CallbackServerContext* context,
                               const EdgeWaypoints* edgeWaypoints,
                               Empty* empty) override {
        serializedEdgeWaypoints_.clear();

        LOG(INFO) << "updated waypoints received";
        for ( WaypointBuffer wpBuf : edgeWaypoints->all_waypoint_buffers() )
        {   
            std::string serializedWPs;
            wpBuf.SerializeToString(&serializedWPs);
            const std::pair< int16_t, std::string > wpPair = std::make_pair( wpBuf.vehicle_index(), serializedWPs );
            serializedEdgeWaypoints_.push_back(wpPair);
            LOG(INFO) << "updated waypoints for vehicle index " << wpBuf.vehicle_index();
        }
        LOG(INFO) << "updated waypoints processed";

        ServerUnaryReactor* reactor = context->DefaultReactor();
        reactor->Finish(Status::OK);
        return reactor;
    }

    ServerUnaryReactor* Server_PushEdgeObjects(CallbackServerContext* context,
                               const EdgeObjects* edgeObjects,
                               Empty* empty) override {
        serializedEdgeObjects_.clear();

        LOG(INFO) << "updated edge objects received";
        for ( ObjectBuffer objBuf : edgeObjects->all_object_buffers() )
        {   
            std::string serializedObjs;
            objBuf.SerializeToString(&serializedObjs);
            const std::pair< int16_t, std::string > objPair = std::make_pair( objBuf.vehicle_id(), serializedObjs );
            serializedEdgeObjects_.push_back(objPair);
            LOG(INFO) << "updated generated predictions for vehicle index " << objBuf.vehicle_id();
        }
        LOG(INFO) << "updated edge objects processed";

        ServerUnaryReactor* reactor = context->DefaultReactor();
        reactor->Finish(Status::OK);
        return reactor;
    }

    // Set up edge mappings before edges/actors start
    ServerUnaryReactor* Server_SetEdgeMappings(CallbackServerContext* context,
                               const EdgeMappingSetup* request,
                               Empty* empty) override {

        LOG(INFO) << "Server_SetEdgeMappings - setting up " << request->num_edges() << " edges";

        mu_.Lock();

        // Clear existing mappings
        vehicleToEdgeMapping_.clear();
        rsuToEdgeMapping_.clear();
        numExpectedEdges_ = request->num_edges();
        hasEdges_ = (numExpectedEdges_ > 0);

        // Process each edge mapping
        for (const EdgeMapping& mapping : request->mappings()) {
            const int32_t edge_index = mapping.edge_index();

            // Map vehicles to this edge
            for (int32_t vehicle_idx : mapping.vehicle_indices()) {
                vehicleToEdgeMapping_[vehicle_idx] = edge_index;
                LOG(INFO) << "  vehicle " << vehicle_idx << " -> edge " << edge_index;
            }

            // Map RSUs to this edge
            for (int32_t rsu_idx : mapping.rsu_indices()) {
                rsuToEdgeMapping_[rsu_idx] = edge_index;
                LOG(INFO) << "  rsu " << rsu_idx << " -> edge " << edge_index;
            }
        }

        mu_.Unlock();

        LOG(INFO) << "Server_SetEdgeMappings - configured " << vehicleToEdgeMapping_.size()
                  << " vehicle mappings and " << rsuToEdgeMapping_.size() << " rsu mappings";

        ServerUnaryReactor* reactor = context->DefaultReactor();
        reactor->Finish(Status::OK);
        return reactor;
    }

    ServerUnaryReactor* Server_StartScenario(CallbackServerContext* context,
                               const SimulationInfo* request,
                               Empty* empty) override {
        vehState_ = VehicleState::REGISTERING;

        configYaml_ = request->test_scenario();
        application_ = request->application();
        version_ = request->version();
        numCars_ = request->vehicle_index(); // bit of a hack to use vindex as count
        isEdge_ = request->is_edge();
        // TODO: simIP_ = // always localhost for now

        assert( numCars_ <= MAX_CARS );
        DLOG(INFO) << "numCars_: " << numCars_;

        ServerUnaryReactor* reactor = context->DefaultReactor();
        reactor->Finish(Status::OK);
        return reactor;
    }

    // ============================================
    // Edge Architecture RPCs
    // ============================================

    // Actor asks orchestrator where to connect (edge or orchestrator)
    ServerUnaryReactor* Client_GetConnectionInfo(CallbackServerContext* context,
                               const RegistrationInfo* request,
                               ActorConnectionInfo* reply) override {

        const int32_t vehicle_index = request->vehicle_index();
        LOG(INFO) << "Client_GetConnectionInfo - request for vehicle_index: " << vehicle_index;

        // Check if this vehicle belongs to an edge
        auto it = vehicleToEdgeMapping_.find(vehicle_index);
        if (hasEdges_ && it != vehicleToEdgeMapping_.end()) {
            const int32_t edge_index = it->second;
            // Find the edge info
            for (const EdgeInfo& edge : edgeInfos_) {
                if (edge.edge_index == edge_index) {
                    reply->set_has_edge(true);
                    reply->set_edge_ip(edge.edge_ip);
                    reply->set_edge_port(edge.edge_port);
                    reply->set_edge_index(edge_index);
                    reply->set_vehicle_index(vehicle_index);
                    LOG(INFO) << "Client_GetConnectionInfo - vehicle " << vehicle_index
                              << " -> edge " << edge_index << " at " << edge.edge_ip << ":" << edge.edge_port;
                    break;
                }
            }
        } else {
            // No edge, actor connects directly to orchestrator
            reply->set_has_edge(false);
            reply->set_vehicle_index(vehicle_index);
            LOG(INFO) << "Client_GetConnectionInfo - vehicle " << vehicle_index << " -> orchestrator (no edge)";
        }

        ServerUnaryReactor* reactor = context->DefaultReactor();
        reactor->Finish(Status::OK);
        return reactor;
    }

    // Edge registers with orchestrator, receives its scenario configuration
    ServerUnaryReactor* Edge_Register(CallbackServerContext* context,
                               const EdgeRegistrationInfo* request,
                               EdgeScenarioConfig* reply) override {

        LOG(INFO) << "Edge_Register - edge_index: " << request->edge_index()
                  << " at " << request->edge_ip() << ":" << request->edge_port();

        mu_.Lock();

        // Store edge info
        EdgeInfo edgeInfo;
        edgeInfo.edge_index = request->edge_index();
        edgeInfo.edge_ip = request->edge_ip();
        edgeInfo.edge_port = request->edge_port();
        edgeInfo.num_vehicles = request->num_vehicles();
        edgeInfo.num_rsus = request->num_rsus();
        edgeInfo.container_name = request->container_name();

        // Find edge config from stored mappings (set by sim_api.py via Server_StartScenario extended)
        // For now, we'll populate these during registration processing
        // The vehicle_indices and rsu_indices are set by sim_api.py before actors/edges start

        // Create push client for this edge
        const std::string connection = absl::StrFormat("%s:%d", request->edge_ip(), request->edge_port());
        PushClient *edgeClient = new PushClient(grpc::CreateChannel(connection, grpc::InsecureChannelCredentials()), connection);
        edgeClients_.push_back(std::move(edgeClient));
        edgeInfos_.push_back(edgeInfo);

        numRegisteredEdges_++;
        mu_.Unlock();

        // Build response with edge-specific config
        reply->set_edge_index(request->edge_index());
        reply->set_edge_config_yaml(configYaml_);  // Full config for now, edge can parse its section
        reply->set_carla_ip(simIP_);
        reply->set_carla_port(2000);  // Default CARLA port
        reply->set_application(application_);
        reply->set_version(version_);

        // Copy vehicle and RSU indices if they've been set and count them
        int32_t numVehicles = 0;
        int32_t numRsus = 0;
        for (const auto& mapping : vehicleToEdgeMapping_) {
            if (mapping.second == request->edge_index()) {
                reply->add_vehicle_indices(mapping.first);
                numVehicles++;
            }
        }
        for (const auto& mapping : rsuToEdgeMapping_) {
            if (mapping.second == request->edge_index()) {
                reply->add_rsu_indices(mapping.first);
                numRsus++;
            }
        }
        reply->set_num_vehicles(numVehicles);
        reply->set_num_rsus(numRsus);

        LOG(INFO) << "Edge_Register - registered edge " << request->edge_index()
                  << " (" << numRegisteredEdges_.load() << "/" << numExpectedEdges_ << ")";

        // Check if all edges are registered
        if (numRegisteredEdges_.load() == numExpectedEdges_) {
            LOG(INFO) << "EDGE REGISTRATION COMPLETE - all " << numExpectedEdges_ << " edges registered";
            // Notify sim_api.py that edges are ready
            simAPIClient_->PushTick(TICK_ID_INVALID, Command::TICK, INVALID_TIME);
        }

        ServerUnaryReactor* reactor = context->DefaultReactor();
        reactor->Finish(Status::OK);
        return reactor;
    }

    // Edge notifies orchestrator it completed processing for a tick
    ServerUnaryReactor* Edge_TickComplete(CallbackServerContext* context,
                               const EdgeTickComplete* request,
                               Empty* reply) override {

        LOG(INFO) << "Edge_TickComplete - edge " << request->edge_index()
                  << " tick " << request->tick_id() << " (" << request->num_actors_processed() << " actors)";

        numCompletedEdges_++;

        const bool allEdgesComplete = (numCompletedEdges_.load() == numExpectedEdges_);
        if (allEdgesComplete && !pushedTick_) {
            pushedTick_ = true;
            simAPIClient_->PushTick(request->tick_id(), command_, INVALID_TIME);
            LOG(INFO) << "tick " << request->tick_id() << " COMPLETE (all edges reported)";
        }

        ServerUnaryReactor* reactor = context->DefaultReactor();
        reactor->Finish(Status::OK);
        return reactor;
    }

    // Orchestrator pushes tick to edge (called internally, not directly via RPC)
    // This is implemented as a client call from Server_DoTick
    // The Edge_PushTick RPC is handled by the edge process, not here

    ServerUnaryReactor* Server_EndScenario(CallbackServerContext* context,
                               const Empty* request,
                               Empty* reply) override {
        command_ = Command::END;

        LOG(INFO) << "pushing END";

        // If we have edges, push END to edges instead of individual vehicles
        if (hasEdges_ && edgeClients_.size() > 0) {
            LOG(INFO) << "pushing END to " << edgeClients_.size() << " edges";
            for ( int i = 0; i < edgeClients_.size(); i++ )
                edgeClients_[i]->PushTick(TICK_ID_INVALID, Command::END, INVALID_TIME); // don't thread --> block
        } else {
            // No edges, push directly to vehicles (existing behavior)
            for ( int i = 0; i < vehicleClients_.size(); i++ )
                vehicleClients_[i]->PushTick(TICK_ID_INVALID, Command::END, INVALID_TIME); // don't thread --> block
        }

        ServerUnaryReactor* reactor = context->DefaultReactor();
        reactor->Finish(Status::OK);
        return reactor;
    }

    private:

        std::vector< PushClient * > vehicleClients_;
        std::vector< PushClient * > edgeClients_;  // Clients for pushing ticks to edges
        PushClient * simAPIClient_;
};

void RunServer(uint16_t port) {
    EcloudServiceImpl service;

    grpc::EnableDefaultHealthCheckService(true);
    grpc::reflection::InitProtoReflectionServerBuilderPlugin();
    ServerBuilder builder;
    // Listen on the given address without any authentication mechanism.
    const std::string server_address = absl::StrFormat("0.0.0.0:%d", port );
    builder.AddListeningPort(server_address, grpc::InsecureServerCredentials());
    // Allow large messages for intermediate feature tensors (WorldFusion/BM2CP)
    builder.SetMaxReceiveMessageSize(200 * 1024 * 1024);  // 200 MB
    builder.SetMaxSendMessageSize(200 * 1024 * 1024);     // 200 MB
    LOG(INFO) << "server listening on port " << port << std::endl;
    // Register "service" as the instance through which we'll communicate with
    // clients. In this case it corresponds to an *synchronous* service.
    builder.RegisterService(&service);
    // Sample way of setting keepalive arguments on the server. Here, we are
    // configuring the server to send keepalive pings at a period of 10 minutes
    // with a timeout of 20 seconds. Additionally, pings will be sent even if
    // there are no calls in flight on an active HTTP2 connection. When receiving
    // pings, the server will permit pings at an interval of 10 seconds.
    builder.AddChannelArgument(GRPC_ARG_KEEPALIVE_TIME_MS,
                                10 * 60 * 1000 /*10 min*/);
    builder.AddChannelArgument(GRPC_ARG_KEEPALIVE_TIMEOUT_MS,
                                20 * 1000 /*20 sec*/);
    builder.AddChannelArgument(GRPC_ARG_KEEPALIVE_PERMIT_WITHOUT_CALLS, 1);
    builder.AddChannelArgument(
        GRPC_ARG_HTTP2_MIN_RECV_PING_INTERVAL_WITHOUT_DATA_MS,
        10 * 1000 /*10 sec*/);
    // Finally assemble the server.
    std::unique_ptr<Server> server(builder.BuildAndStart());

    // Wait for the server to shutdown. Note that some other thread must be
    // responsible for shutting down the server for this call to ever return.
    server->Wait();
}

int main(int argc, char* argv[]) {

    // 2 - std::cout << "ABSL: ERROR - " << static_cast<uint16_t>(absl::LogSeverityAtLeast::kError) << std::endl;
    // 1 - std::cout << "ABSL: WARNING - " << static_cast<uint16_t>(absl::LogSeverityAtLeast::kWarning) << std::endl;
    // 0 - std::cout << "ABSL: INFO - " << static_cast<uint16_t>(absl::LogSeverityAtLeast::kInfo) << std::endl;

    absl::ParseCommandLine(argc, argv);
    //absl::InitializeLog();
    absl::SetMinLogLevel(static_cast<absl::LogSeverityAtLeast>(absl::GetFlag(FLAGS_minloglevel)));

    std::thread server = std::thread(&RunServer,absl::GetFlag(FLAGS_port));

    server.join();

    return 0;
}
