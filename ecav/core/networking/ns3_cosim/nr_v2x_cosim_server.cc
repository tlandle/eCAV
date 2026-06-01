/* -*- Mode:C++; c-file-style:"gnu"; indent-tabs-mode:nil; -*- */
/*
 * NR V2X Co-Simulation Server
 * Author: Tyler Landle <tlandle3@gatech.edu>
 *
 * Persistent process for CARLA co-simulation. Two operating modes:
 *
 *   Mode 0 (V2V_SIDELINK): NR V2X Mode 2 sidelink. Vehicles communicate
 *       directly over PC5 with autonomous resource selection. This is the
 *       distributed V2V path. PRR degrades with N due to MAC collisions.
 *
 *   Mode 1 (UU_UPLINK): NR Uu uplink to gNB. Vehicles upload features
 *       to the edge server via centralized scheduling. No MAC collisions
 *       (gNB controls resource allocation). This is the edge path.
 *
 * Per-tick operation (shared memory IPC, no serialization):
 *   1. Python writes vehicle positions to shared memory
 *   2. Python sets state = PYTHON_READY
 *   3. C++ reads positions, runs NR V2X MAC for one tick
 *   4. C++ writes per-link delivery results, sets state = NS3_DONE
 *   5. Python reads results
 *
 * Build: symlink or copy to ns-3-dev-5glena/scratch/ and run ./ns3 build
 *
 * Compile-time options:
 *   USE_NS3_FULL: enable full ns-3 NR V2X stack (requires 5G-LENA)
 *   (default): analytical SB-SPS model for development/testing
 */

#include "ns3/core-module.h"
#include "ns3/network-module.h"
#include "ns3/internet-module.h"
#include "ns3/mobility-module.h"
#include "ns3/applications-module.h"
#include "ns3/point-to-point-module.h"

#ifdef USE_NS3_FULL
#include "ns3/nr-module.h"
#include "ns3/lte-module.h"
#include "ns3/antenna-module.h"
#include "ns3/config-store.h"
#include <ns3/buildings-helper.h>
#endif

#include <sys/mman.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <unistd.h>
#include <cstring>
#include <cstdint>
#include <chrono>
#include <iostream>
#include <vector>
#include <map>
#include <set>
#include <cmath>
#include <algorithm>

using namespace ns3;

NS_LOG_COMPONENT_DEFINE("NrV2xCosimServer");

// ===================================================================
//  Shared memory layout (must match shm_layout.py exactly)
// ===================================================================
#pragma pack(push, 1)
struct ShmHeader {
    uint32_t tick;
    uint16_t n_vehicles;
    uint16_t max_vehicles;
    uint8_t  state;         // 0=idle, 1=python_ready, 2=ns3_done, 255=shutdown
    uint8_t  mode;          // 0=V2V sidelink, 1=Uu uplink
    uint16_t payload_bytes;
    float    sim_time_s;
    uint8_t  _pad[48];
};

struct VehicleEntry {
    float x, y, z, heading_rad;
    uint8_t tx_flag;
    uint8_t _pad[3];
};

struct ShmLinkResult {
    uint16_t tx_id;
    uint16_t rx_id;
    uint8_t  delivered;
    uint8_t  _pad;
    int16_t  sinr_db_x10;
    uint16_t delay_ms_x10;  // one-way delay * 10, 0 if undelivered
};

struct ResultHeader {
    uint32_t n_results;
    float    prr;
    float    mean_sinr_db;
    float    channel_time_ms;
};
#pragma pack(pop)

static_assert(sizeof(ShmHeader) == 64, "Header must be 64 bytes");
static_assert(sizeof(VehicleEntry) == 20, "VehicleEntry must be 20 bytes");
static_assert(sizeof(ShmLinkResult) == 10, "ShmLinkResult must be 10 bytes");
static_assert(sizeof(ResultHeader) == 16, "ResultHeader must be 16 bytes");

// ===================================================================
//  Constants
// ===================================================================
constexpr uint8_t STATE_IDLE = 0;
constexpr uint8_t STATE_PYTHON_READY = 1;
constexpr uint8_t STATE_NS3_DONE = 2;
constexpr uint8_t STATE_SHUTDOWN = 255;

// ===================================================================
//  Global state
// ===================================================================
uint8_t* g_shm = nullptr;
uint32_t g_maxVehicles = 128;
size_t   g_shmSize = 0;

// ===================================================================
//  SHM pointer helpers
// ===================================================================
ShmHeader* getHeader() {
    return reinterpret_cast<ShmHeader*>(g_shm);
}

VehicleEntry* getVehicle(int i) {
    return reinterpret_cast<VehicleEntry*>(
        g_shm + sizeof(ShmHeader) + i * sizeof(VehicleEntry));
}

ShmLinkResult* getLinkResult(int i) {
    return reinterpret_cast<ShmLinkResult*>(
        g_shm + sizeof(ShmHeader) + g_maxVehicles * sizeof(VehicleEntry)
        + i * sizeof(ShmLinkResult));
}

ResultHeader* getResultHeader() {
    return reinterpret_cast<ResultHeader*>(
        g_shm + sizeof(ShmHeader) + g_maxVehicles * sizeof(VehicleEntry)
        + g_maxVehicles * g_maxVehicles * sizeof(ShmLinkResult));
}

// ===================================================================
//  Delivery tracker for per-tick result collection
// ===================================================================
struct DeliveryRecord {
    bool  delivered;
    float sinr_db;
    float delay_ms;  // 0 on failure
};

struct DeliveryTracker {
    // (tx_node_id, rx_node_id) -> record
    std::map<std::pair<uint32_t, uint32_t>, DeliveryRecord> results;

    void RecordDelivery(uint32_t txNode, uint32_t rxNode,
                        float sinrDb, float delayMs) {
        results[{txNode, rxNode}] = {true, sinrDb, delayMs};
    }

    void RecordFailure(uint32_t txNode, uint32_t rxNode, float sinrDb) {
        auto key = std::make_pair(txNode, rxNode);
        if (results.find(key) == results.end()) {
            results[key] = {false, sinrDb, 0.0f};
        }
    }

    void Clear() { results.clear(); }
};

DeliveryTracker g_tracker;

#ifdef USE_NS3_FULL
// ===================================================================
//  Full ns-3 NR V2X Mode 2 integration
// ===================================================================

// IMSI/RNTI to node-index mapping
std::map<uint64_t, uint32_t> g_imsiToNodeIdx;
std::map<uint16_t, uint32_t> g_rntiToNodeIdx;

// Node container and mobility models (persistent across ticks)
NodeContainer g_ueNodes;
std::vector<Ptr<ConstantPositionMobilityModel>> g_mobilityModels;

// RLC RX trace callback: fires when a PDU is successfully received
void OnRlcRxPdu(uint64_t imsi, uint16_t rnti, uint16_t txRnti,
                uint8_t lcid, uint32_t rxPduSize, double delay)
{
    // Map IMSI to node index (IMSI is 1-based, node index is 0-based)
    uint32_t rxIdx = (imsi > 0) ? (imsi - 1) : 0;

    // Map TX RNTI to node index
    auto it = g_rntiToNodeIdx.find(txRnti);
    uint32_t txIdx = (it != g_rntiToNodeIdx.end()) ? it->second : 0;

    if (rxIdx < g_maxVehicles && txIdx < g_maxVehicles && txIdx != rxIdx) {
        // delay is in seconds (ns-3 convention); store as ms.
        g_tracker.RecordDelivery(txIdx, rxIdx, 10.0f,
                                 static_cast<float>(delay * 1000.0));
    }
}

// Application-layer TX/RX callback for packet tracking
void OnAppRx(Ptr<Node> node, Ptr<const Packet> pkt,
             const Address& srcAddr, const Address& dstAddr)
{
    uint32_t rxIdx = node->GetId();
    // Extract source node from packet tag or address mapping
    // For broadcast sidelink, we track at RLC level instead
}

// Build RNTI-to-nodeIndex map after device installation
void BuildRntiMap(NetDeviceContainer& devices) {
    for (uint32_t i = 0; i < devices.GetN(); i++) {
        Ptr<NrUeNetDevice> ueDev = DynamicCast<NrUeNetDevice>(devices.Get(i));
        if (ueDev) {
            uint64_t imsi = ueDev->GetImsi();
            g_imsiToNodeIdx[imsi] = i;
            // RNTI is assigned after connection; we build the map
            // using IMSI for now (IMSI = nodeIdx + 1)
        }
    }
}

// Get sidelink bitmap from string (from highway example)
void GetSlBitmapFromString(std::string slBitMapString,
                           std::vector<std::bitset<1>>& slBitMapVector)
{
    std::stringstream ss(slBitMapString);
    std::string token;
    while (std::getline(ss, token, '|')) {
        slBitMapVector.push_back(std::bitset<1>(token == "1" ? 1 : 0));
    }
}

void SetupSidelinkTopology(uint32_t maxVeh, double carrierGhz,
                            double bandwidthMhz, double txPowerDbm,
                            uint16_t numerology, uint16_t payloadBytes)
{
    // Create nodes with constant-position mobility
    g_ueNodes.Create(maxVeh);

    MobilityHelper mobility;
    mobility.SetMobilityModel("ns3::ConstantPositionMobilityModel");
    mobility.Install(g_ueNodes);

    // Store mobility model pointers for fast position updates
    g_mobilityModels.resize(maxVeh);
    for (uint32_t i = 0; i < maxVeh; i++) {
        g_mobilityModels[i] = g_ueNodes.Get(i)->GetObject<ConstantPositionMobilityModel>();
        // Place inactive nodes far away
        g_mobilityModels[i]->SetPosition(Vector(10000.0 + i * 100.0, 10000.0, 0.0));
    }

    // NR / EPC helpers
    Ptr<NrPointToPointEpcHelper> epcHelper = CreateObject<NrPointToPointEpcHelper>();
    Ptr<NrHelper> nrHelper = CreateObject<NrHelper>();
    nrHelper->SetEpcHelper(epcHelper);

    // Spectrum: single band, single CC, single BWP
    uint16_t bandwidthBand = static_cast<uint16_t>(bandwidthMhz * 10); // x100kHz
    CcBwpCreator ccBwpCreator;
    CcBwpCreator::SimpleOperationBandConf bandConf(
        carrierGhz * 1e9, bandwidthBand, 1,
        BandwidthPartInfo::V2V_Highway);
    OperationBandInfo band = ccBwpCreator.CreateOperationBandContiguousCc(bandConf);

    // Channel: enable randomness for realistic results
    Config::SetDefault("ns3::ThreeGppChannelModel::UpdatePeriod",
                       TimeValue(MilliSeconds(100)));
    nrHelper->SetChannelConditionModelAttribute("UpdatePeriod",
                                                TimeValue(MilliSeconds(100)));
    nrHelper->SetPathlossAttribute("ShadowingEnabled", BooleanValue(true));

    nrHelper->InitializeOperationBand(&band);
    BandwidthPartInfoPtrVector allBwps = CcBwpCreator::GetAllBwps({band});

    // UE antenna: quasi-omnidirectional (1x2 array, isotropic elements)
    nrHelper->SetUeAntennaAttribute("NumRows", UintegerValue(1));
    nrHelper->SetUeAntennaAttribute("NumColumns", UintegerValue(2));
    nrHelper->SetUeAntennaAttribute("AntennaElement",
        PointerValue(CreateObject<IsotropicAntennaModel>()));

    nrHelper->SetUePhyAttribute("TxPower", DoubleValue(txPowerDbm));

    // MAC: NR Mode 2 autonomous resource selection
    nrHelper->SetUeMacAttribute("EnableSensing", BooleanValue(false));
    nrHelper->SetUeMacAttribute("T1", UintegerValue(2));
    nrHelper->SetUeMacAttribute("T2", UintegerValue(33));
    nrHelper->SetUeMacAttribute("ActivePoolId", UintegerValue(0));
    nrHelper->SetUeMacAttribute("ReservationPeriod",
                                TimeValue(MilliSeconds(100)));
    nrHelper->SetUeMacAttribute("NumSidelinkProcess", UintegerValue(4));
    nrHelper->SetUeMacAttribute("EnableBlindReTx", BooleanValue(true));
    nrHelper->SetUeMacAttribute("SlThresPsschRsrp", IntegerValue(-128));

    uint8_t bwpId = 0;
    nrHelper->SetBwpManagerTypeId(TypeId::LookupByName("ns3::NrSlBwpManagerUe"));
    nrHelper->SetUeBwpManagerAlgorithmAttribute("GBR_MC_PUSH_TO_TALK",
                                                UintegerValue(bwpId));

    // Install UE devices
    NetDeviceContainer ueDevices = nrHelper->InstallUeDevice(g_ueNodes, allBwps);
    for (auto it = ueDevices.Begin(); it != ueDevices.End(); ++it) {
        DynamicCast<NrUeNetDevice>(*it)->UpdateConfig();
    }

    // Sidelink helper
    Ptr<NrSlHelper> nrSlHelper = CreateObject<NrSlHelper>();
    nrSlHelper->SetEpcHelper(epcHelper);
    std::string errorModel = "ns3::NrEesmIrT1";
    nrSlHelper->SetSlErrorModel(errorModel);
    nrSlHelper->SetUeSlAmcAttribute("AmcModel", EnumValue(NrAmc::ErrorModel));
    nrSlHelper->SetNrSlSchedulerTypeId(NrSlUeMacSchedulerSimple::GetTypeId());
    nrSlHelper->SetUeSlSchedulerAttribute("FixNrSlMcs", BooleanValue(true));
    nrSlHelper->SetUeSlSchedulerAttribute("InitialNrSlMcs", UintegerValue(14));

    std::set<uint8_t> bwpIdContainer;
    bwpIdContainer.insert(bwpId);
    nrSlHelper->PrepareUeForSidelink(ueDevices, bwpIdContainer);

    // Sidelink resource pool
    Ptr<NrSlCommPreconfigResourcePoolFactory> poolFactory =
        Create<NrSlCommPreconfigResourcePoolFactory>();
    std::string slBitMap = "1|1|1|1|1|1|0|0|0|1|1|1";
    std::vector<std::bitset<1>> slBitMapVector;
    GetSlBitmapFromString(slBitMap, slBitMapVector);
    poolFactory->SetSlTimeResources(slBitMapVector);
    poolFactory->SetSlSensingWindow(100);
    poolFactory->SetSlSelectionWindow(5);
    poolFactory->SetSlFreqResourcePscch(10);
    poolFactory->SetSlSubchannelSize(10);
    poolFactory->SetSlMaxNumPerReserve(3);

    LteRrcSap::SlResourcePoolNr pool = poolFactory->CreatePool();

    LteRrcSap::SlResourcePoolConfigNr poolCfg;
    poolCfg.haveSlResourcePoolConfigNr = true;
    LteRrcSap::SlResourcePoolIdNr poolId;
    poolId.id = 0;
    poolCfg.slResourcePoolId = poolId;
    poolCfg.slResourcePool = pool;

    LteRrcSap::SlBwpPoolConfigCommonNr bwpPoolCfg;
    bwpPoolCfg.slTxPoolSelectedNormal[0] = poolCfg;

    LteRrcSap::Bwp bwpIe;
    bwpIe.numerology = numerology;
    bwpIe.symbolsPerSlots = 14;
    bwpIe.rbPerRbg = 1;
    bwpIe.bandwidth = bandwidthBand;

    LteRrcSap::SlBwpGeneric slBwpGeneric;
    slBwpGeneric.bwp = bwpIe;
    slBwpGeneric.slLengthSymbols = LteRrcSap::GetSlLengthSymbolsEnum(14);
    slBwpGeneric.slStartSymbol = LteRrcSap::GetSlStartSymbolEnum(0);

    LteRrcSap::SlBwpConfigCommonNr slBwpCfg;
    slBwpCfg.haveSlBwpGeneric = true;
    slBwpCfg.slBwpGeneric = slBwpGeneric;
    slBwpCfg.haveSlBwpPoolConfigCommonNr = true;
    slBwpCfg.slBwpPoolConfigCommonNr = bwpPoolCfg;

    LteRrcSap::SlFreqConfigCommonNr slFreqCfg;
    slFreqCfg.slBwpList[bwpId] = slBwpCfg;

    std::string tddPattern = "DL|DL|DL|F|UL|UL|UL|UL|UL|UL|";
    LteRrcSap::TddUlDlConfigCommon tddCfg;
    tddCfg.tddPattern = tddPattern;

    LteRrcSap::SlPreconfigGeneralNr slGeneral;
    slGeneral.slTddConfig = tddCfg;

    LteRrcSap::SlUeSelectedConfig slUeSelCfg;
    slUeSelCfg.slProbResourceKeep = 0.0;
    LteRrcSap::SlPsschTxParameters psschParams;
    psschParams.slMaxTxTransNumPssch = 5;
    LteRrcSap::SlPsschTxConfigList psschCfgList;
    psschCfgList.slPsschTxParameters[0] = psschParams;
    slUeSelCfg.slPsschTxConfigList = psschCfgList;

    LteRrcSap::SidelinkPreconfigNr slPreCfg;
    slPreCfg.slPreconfigGeneral = slGeneral;
    slPreCfg.slUeSelectedPreConfig = slUeSelCfg;
    slPreCfg.slPreconfigFreqInfoList[0] = slFreqCfg;

    nrSlHelper->InstallNrSlPreConfiguration(ueDevices, slPreCfg);

    // Random streams
    int64_t stream = 1;
    stream += nrHelper->AssignStreams(ueDevices, stream);
    stream += nrSlHelper->AssignStreams(ueDevices, stream);

    // Internet stack + IP
    InternetStackHelper internet;
    internet.Install(g_ueNodes);
    uint32_t dstL2Id = 255;
    Ipv4Address groupAddr("225.0.0.0");
    uint16_t port = 8000;

    Ipv4InterfaceContainer ueIpIface =
        epcHelper->AssignUeIpv4Address(ueDevices);

    Ipv4StaticRoutingHelper ipv4RoutingHelper;
    for (uint32_t u = 0; u < g_ueNodes.GetN(); u++) {
        Ptr<Ipv4StaticRouting> ueRouting =
            ipv4RoutingHelper.GetStaticRouting(
                g_ueNodes.Get(u)->GetObject<Ipv4>());
        ueRouting->SetDefaultRoute(
            epcHelper->GetUeDefaultGatewayAddress(), 1);
    }

    // Sidelink bearers (all nodes TX and RX)
    Ptr<LteSlTft> tftTx = Create<LteSlTft>(
        LteSlTft::Direction::TRANSMIT,
        LteSlTft::CommType::GroupCast,
        groupAddr, dstL2Id);
    nrSlHelper->ActivateNrSlBearer(Seconds(2.0), ueDevices, tftTx);

    Ptr<LteSlTft> tftRx = Create<LteSlTft>(
        LteSlTft::Direction::RECEIVE,
        LteSlTft::CommType::GroupCast,
        groupAddr, dstL2Id);
    nrSlHelper->ActivateNrSlBearer(Seconds(2.0), ueDevices, tftRx);

    // Applications: OnOff client (TX) + PacketSink (RX) on every node
    Address remoteAddr = InetSocketAddress(groupAddr, port);
    Address localAddr = InetSocketAddress(Ipv4Address::GetAny(), port);

    // Data rate: payload * 10 Hz * 8 bits / 1000 = kbps
    uint32_t dataRateKbps = (payloadBytes * 10 * 8) / 1000;
    if (dataRateKbps == 0) dataRateKbps = 1;
    std::string dataRateStr = std::to_string(dataRateKbps) + "kb/s";

    OnOffHelper client("ns3::UdpSocketFactory", remoteAddr);
    client.SetAttribute("EnableSeqTsSizeHeader", BooleanValue(true));
    client.SetConstantRate(DataRate(dataRateStr), payloadBytes);

    PacketSinkHelper sink("ns3::UdpSocketFactory", localAddr);
    sink.SetAttribute("EnableSeqTsSizeHeader", BooleanValue(true));

    for (uint32_t i = 0; i < maxVeh; i++) {
        ApplicationContainer clientApp = client.Install(g_ueNodes.Get(i));
        clientApp.Start(Seconds(2.1 + 0.001 * i));
        clientApp.Stop(Seconds(86400.0)); // effectively never

        ApplicationContainer sinkApp = sink.Install(g_ueNodes.Get(i));
        sinkApp.Start(Seconds(0.0));
    }

    // Connect RLC RX trace for delivery tracking
    Config::ConnectWithoutContext(
        "/NodeList/*/DeviceList/*/$ns3::NrUeNetDevice/"
        "ComponentCarrierMapUe/*/NrUeMac/RxRlcPduWithTxRnti",
        MakeCallback(&OnRlcRxPdu));

    // Build RNTI map
    BuildRntiMap(ueDevices);

    // Run initial setup phase (bearer activation at t=2s)
    Simulator::Stop(Seconds(2.5));
    Simulator::Run();

    std::cout << "NR V2X sidelink topology initialized ("
              << maxVeh << " nodes, payload=" << payloadBytes
              << "B, rate=" << dataRateStr << ")" << std::endl;
}

void UpdatePositions(uint16_t N) {
    for (uint32_t i = 0; i < g_maxVehicles; i++) {
        if (i < N) {
            VehicleEntry* ve = getVehicle(i);
            g_mobilityModels[i]->SetPosition(
                Vector(ve->x, ve->y, ve->z));
        } else {
            // Move inactive nodes far away
            g_mobilityModels[i]->SetPosition(
                Vector(10000.0 + i * 100.0, 10000.0, 0.0));
        }
    }
}

void AdvanceOneTick(double tickDurationS) {
    g_tracker.Clear();
    Simulator::Stop(Seconds(tickDurationS));
    Simulator::Run();
}

// ===================================================================
//  Uu uplink: gNB + EPC + remote host + per-UE UDP uplink
// ===================================================================
//
// Structured after 5G-LENA's cttc-nr-cc-bwp-demo.cc. Single gNB co-located
// with the RSU at the world origin; UEs are the SHM vehicles. A P2P link
// connects the PGW to one synthetic remote host that represents the edge.
// Per tick we call Send() on each active UE's socket, tag with a SeqTsHeader,
// and on the remote host's PacketSink Rx trace compute delay = now - ts.

// Persistent Uu state (lifetime = server process).
NodeContainer g_gnbNodes;
NodeContainer g_remoteHostContainer;
std::vector<Ptr<Socket>> g_ueUpSockets;   // one per UE (UL: UE -> edge)
std::vector<Ptr<Socket>> g_edgeDnSockets; // one per UE (DL: edge -> UE)
std::vector<Ipv4Address> g_ueIpAddrs;     // target address for DL per UE
Ipv4Address g_remoteHostAddr;
uint16_t g_uuUplinkBasePort   = 2000;  // UL per-UE port = base + ueIdx
uint16_t g_uuDownlinkBasePort = 3000;  // DL per-UE port = base + ueIdx
uint32_t g_gnbNodeIdx = 0xFFFFFFFF;
// NR helpers and EPC helper MUST persist for the lifetime of the process.
// Destroying them between ticks breaks multi-Run() because NR's scheduler
// and EPC state reach into helper internals. Cosim originally had these as
// locals in SetupUuTopology, which caused silent crashes on the 2nd Run().
Ptr<NrPointToPointEpcHelper> g_epcHelper;
Ptr<IdealBeamformingHelper>  g_beamHelper;
Ptr<NrHelper>                g_nrHelper;

static void OnUuPacketRxPerUe(uint32_t txId, Ptr<const Packet> pkt)
{
    // Uplink Rx at edge (UdpServer on remote host). Chunked sends: last
    // chunk has bit 31 of seq set. Record delivery only on last chunk so
    // the recorded delay is full-message completion time.
    SeqTsHeader seqTs;
    Ptr<Packet> copy = pkt->Copy();
    if (copy->PeekHeader(seqTs) == 0) return;
    uint32_t seq = seqTs.GetSeq();
    if ((seq & 0x80000000u) == 0) return;
    double delayMs =
        (Simulator::Now() - seqTs.GetTs()).GetNanoSeconds() / 1e6;
    if (txId < g_maxVehicles) {
        g_tracker.RecordDelivery(txId, g_gnbNodeIdx, 10.0f,
                                 static_cast<float>(delayMs));
    }
}

// Separate tracker for downlink (edge -> UE). rx_id = UE index, tx_id = 0
// (the edge). Uses the same RecordDelivery semantics — we repurpose the
// result struct by writing DL records with rx_id set to the UE.
static std::map<uint32_t, DeliveryRecord> g_dlTracker;

static void OnUuPacketRxDownlink(uint32_t ueId, Ptr<const Packet> pkt)
{
    // Downlink Rx at UE (UdpServer on each UE).
    SeqTsHeader seqTs;
    Ptr<Packet> copy = pkt->Copy();
    if (copy->PeekHeader(seqTs) == 0) return;
    uint32_t seq = seqTs.GetSeq();
    if ((seq & 0x80000000u) == 0) return;
    double delayMs =
        (Simulator::Now() - seqTs.GetTs()).GetNanoSeconds() / 1e6;
    if (ueId < g_maxVehicles) {
        g_dlTracker[ueId] = {true, 10.0f, static_cast<float>(delayMs)};
    }
}

void SetupUuTopology(uint32_t maxVeh, double carrierGhz,
                     double bandwidthMhz, double txPowerDbm,
                     uint16_t numerology, uint16_t /*payloadBytes*/)
{
    // Bump RLC UM TX buffer so large multi-chunk payloads (e.g. 17 KB
    // cooperative-perception features) don't overflow the default 10 KB
    // buffer. We chunk at ~1.4 KB; 64 KB covers CoBEVT-256x (~17 KB) plus
    // a safety margin for burst contention at high N.
    Config::SetDefault("ns3::LteRlcUm::MaxTxBufferSize",
                       UintegerValue(64 * 1024));

    // Create UE + gNB nodes. UEs persist like sidelink path; gNB is fixed.
    g_ueNodes.Create(maxVeh);
    g_gnbNodes.Create(1);
    g_remoteHostContainer.Create(1);

    MobilityHelper mobility;
    mobility.SetMobilityModel("ns3::ConstantPositionMobilityModel");
    mobility.Install(g_ueNodes);
    mobility.Install(g_gnbNodes);
    mobility.Install(g_remoteHostContainer);

    g_mobilityModels.resize(maxVeh);
    // Initialize UEs spread around the cell at mid-range so NR attach +
    // beamforming initialization have sensible pathloss. Actual positions
    // get updated per tick for active UEs; inactive ones stay here.
    for (uint32_t i = 0; i < maxVeh; i++) {
        g_mobilityModels[i] = g_ueNodes.Get(i)->GetObject<ConstantPositionMobilityModel>();
        double angle = 2.0 * M_PI * i / maxVeh;
        double r = 40.0;
        g_mobilityModels[i]->SetPosition(Vector(r * cos(angle), r * sin(angle), 1.5));
    }
    // gNB co-located with RSU at world origin, 4.5 m mast height (matches
    // eCAV RSU LiDAR placement).
    g_gnbNodes.Get(0)->GetObject<ConstantPositionMobilityModel>()
        ->SetPosition(Vector(0.0, 0.0, 4.5));

    // NR + EPC helpers — store globally so they outlive this function.
    // Losing these Ptrs between ticks breaks NR's multi-Run behavior.
    g_epcHelper  = CreateObject<NrPointToPointEpcHelper>();
    g_beamHelper = CreateObject<IdealBeamformingHelper>();
    g_nrHelper   = CreateObject<NrHelper>();
    auto& epcHelper = g_epcHelper;
    auto& beam = g_beamHelper;
    auto& nrHelper = g_nrHelper;
    nrHelper->SetBeamformingHelper(beam);
    nrHelper->SetEpcHelper(epcHelper);
    beam->SetAttribute("BeamformingMethod",
                       TypeIdValue(DirectPathBeamforming::GetTypeId()));

    // Spectrum: single band, single CC, single BWP. bandwidthMhz MHz at
    // carrierGhz GHz. SimpleOperationBandConf wants Hz (double), not the
    // 100-kHz uint16 that sidelink's LteRrcSap::Bwp attribute uses.
    double bandwidthHz = bandwidthMhz * 1e6;
    CcBwpCreator ccBwpCreator;
    CcBwpCreator::SimpleOperationBandConf bandConf(
        carrierGhz * 1e9, bandwidthHz, 1,
        BandwidthPartInfo::UMi_StreetCanyon_LoS);
    bandConf.m_numBwp = 1;
    OperationBandInfo band = ccBwpCreator.CreateOperationBandContiguousCc(bandConf);
    nrHelper->SetPathlossAttribute("ShadowingEnabled", BooleanValue(false));
    // S1u backhaul: gNB -> PGW tunneling delay. Typical MEC edge: 1-5 ms
    // fronthaul; non-MEC (regional) edge: 5-20 ms. Set via --s1u-delay-ms.
    double s1uDelayMs = 2.0;
    if (const char* s = std::getenv("UU_S1U_DELAY_MS")) s1uDelayMs = std::atof(s);
    epcHelper->SetAttribute("S1uLinkDelay",
                            TimeValue(MilliSeconds((uint32_t)s1uDelayMs)));
    // Reduce ns-3 event rate for faster wall-clock. Channel model updates
    // every 500ms (default is 100ms) — we use static UE positions per tick
    // so frequent channel updates buy nothing.
    Config::SetDefault("ns3::ThreeGppChannelModel::UpdatePeriod",
                       TimeValue(MilliSeconds(500)));
    nrHelper->SetChannelConditionModelAttribute("UpdatePeriod",
                                                TimeValue(MilliSeconds(500)));
    nrHelper->SetSchedulerTypeId(TypeId::LookupByName("ns3::NrMacSchedulerTdmaRR"));
    nrHelper->InitializeOperationBand(&band);
    BandwidthPartInfoPtrVector allBwps = CcBwpCreator::GetAllBwps({band});

    // Antennas
    nrHelper->SetUeAntennaAttribute("NumRows", UintegerValue(1));
    nrHelper->SetUeAntennaAttribute("NumColumns", UintegerValue(2));
    nrHelper->SetUeAntennaAttribute("AntennaElement",
        PointerValue(CreateObject<IsotropicAntennaModel>()));
    nrHelper->SetGnbAntennaAttribute("NumRows", UintegerValue(4));
    nrHelper->SetGnbAntennaAttribute("NumColumns", UintegerValue(8));
    nrHelper->SetGnbAntennaAttribute("AntennaElement",
        PointerValue(CreateObject<IsotropicAntennaModel>()));

    nrHelper->SetUePhyAttribute("TxPower", DoubleValue(txPowerDbm));

    NetDeviceContainer gnbDev = nrHelper->InstallGnbDevice(g_gnbNodes, allBwps);
    NetDeviceContainer ueDev  = nrHelper->InstallUeDevice(g_ueNodes, allBwps);

    nrHelper->GetGnbPhy(gnbDev.Get(0), 0)
        ->SetAttribute("Numerology", UintegerValue(numerology));
    nrHelper->GetGnbPhy(gnbDev.Get(0), 0)
        ->SetAttribute("TxPower", DoubleValue(txPowerDbm));
    // All-flexible TDD pattern (matches cc-bwp-demo). The scheduler picks
    // UL vs DL per slot based on pending traffic. Explicit fixed TDD
    // patterns starve whichever direction has more pending bytes; flex
    // lets the scheduler adapt to our per-tick UL-heavy (features) +
    // smaller DL (predictions) bursts.
    nrHelper->GetGnbPhy(gnbDev.Get(0), 0)
        ->SetAttribute("Pattern", StringValue(
            "F|F|F|F|F|F|F|F|F|F|"));

    for (auto it = gnbDev.Begin(); it != gnbDev.End(); ++it) {
        DynamicCast<NrGnbNetDevice>(*it)->UpdateConfig();
    }
    for (auto it = ueDev.Begin(); it != ueDev.End(); ++it) {
        DynamicCast<NrUeNetDevice>(*it)->UpdateConfig();
    }

    // Internet on remote host; P2P link PGW <-> remote.
    Ptr<Node> pgw = epcHelper->GetPgwNode();
    Ptr<Node> remoteHost = g_remoteHostContainer.Get(0);
    InternetStackHelper internet;
    internet.Install(g_remoteHostContainer);
    PointToPointHelper p2ph;
    p2ph.SetDeviceAttribute("DataRate", DataRateValue(DataRate("100Gb/s")));
    p2ph.SetDeviceAttribute("Mtu", UintegerValue(2500));
    p2ph.SetChannelAttribute("Delay", TimeValue(Seconds(0.0)));
    NetDeviceContainer internetDevs = p2ph.Install(pgw, remoteHost);
    Ipv4AddressHelper ipv4h;
    ipv4h.SetBase("1.0.0.0", "255.0.0.0");
    Ipv4InterfaceContainer ipIfs = ipv4h.Assign(internetDevs);
    Ipv4StaticRoutingHelper ipv4Rh;
    Ptr<Ipv4StaticRouting> rhStatic =
        ipv4Rh.GetStaticRouting(remoteHost->GetObject<Ipv4>());
    rhStatic->AddNetworkRouteTo(Ipv4Address("7.0.0.0"),
                                Ipv4Mask("255.0.0.0"), 1);
    internet.Install(g_ueNodes);
    Ipv4InterfaceContainer ueIpIfs =
        epcHelper->AssignUeIpv4Address(NetDeviceContainer(ueDev));
    g_remoteHostAddr = ipIfs.GetAddress(1);
    g_gnbNodeIdx = g_gnbNodes.Get(0)->GetId();

    for (uint32_t u = 0; u < g_ueNodes.GetN(); u++) {
        Ptr<Ipv4StaticRouting> ueSr =
            ipv4Rh.GetStaticRouting(g_ueNodes.Get(u)->GetObject<Ipv4>());
        ueSr->SetDefaultRoute(epcHelper->GetUeDefaultGatewayAddress(), 1);
    }
    nrHelper->AttachToClosestEnb(ueDev, gnbDev);

    // Per-UE UL + DL. UL: UdpServer on remote host (edge), UE-side raw
    // socket as client. DL: UdpServer on UE, edge-side raw socket as
    // client. Each direction has its own port and EpcTft filter so PGW
    // routing disambiguates bearers.
    g_ueUpSockets.resize(g_ueNodes.GetN());
    g_edgeDnSockets.resize(g_ueNodes.GetN());
    g_ueIpAddrs.resize(g_ueNodes.GetN());
    for (uint32_t u = 0; u < g_ueNodes.GetN(); u++) {
        Ipv4Address ueAddr = ueIpIfs.GetAddress(u);
        g_ueIpAddrs[u] = ueAddr;
        uint16_t ulPort = g_uuUplinkBasePort + u;
        uint16_t dlPort = g_uuDownlinkBasePort + u;

        // UL: edge-side server + UE-side client socket
        UdpServerHelper ulSrv(ulPort);
        ApplicationContainer ulSa = ulSrv.Install(remoteHost);
        ulSa.Start(Seconds(0.0));
        DynamicCast<UdpServer>(ulSa.Get(0))->TraceConnectWithoutContext(
            "Rx", MakeBoundCallback(&OnUuPacketRxPerUe, u));
        Ptr<Socket> uls = Socket::CreateSocket(g_ueNodes.Get(u),
            TypeId::LookupByName("ns3::UdpSocketFactory"));
        uls->Bind();
        uls->Connect(InetSocketAddress(g_remoteHostAddr, ulPort));
        g_ueUpSockets[u] = uls;

        // DL: UE-side server + edge-side client socket
        UdpServerHelper dlSrv(dlPort);
        ApplicationContainer dlSa = dlSrv.Install(g_ueNodes.Get(u));
        dlSa.Start(Seconds(0.0));
        DynamicCast<UdpServer>(dlSa.Get(0))->TraceConnectWithoutContext(
            "Rx", MakeBoundCallback(&OnUuPacketRxDownlink, u));
        Ptr<Socket> dls = Socket::CreateSocket(remoteHost,
            TypeId::LookupByName("ns3::UdpSocketFactory"));
        dls->Bind();
        dls->Connect(InetSocketAddress(ueAddr, dlPort));
        g_edgeDnSockets[u] = dls;

        // Dedicated bearer covering both directions (UL + DL for this UE).
        Ptr<EpcTft> tft = Create<EpcTft>();
        EpcTft::PacketFilter ulpf;
        ulpf.remotePortStart = ulPort;
        ulpf.remotePortEnd   = ulPort;
        tft->Add(ulpf);
        EpcTft::PacketFilter dlpf;
        dlpf.localPortStart = dlPort;
        dlpf.localPortEnd   = dlPort;
        tft->Add(dlpf);
        EpsBearer bearer(EpsBearer::NGBR_LOW_LAT_EMBB);
        nrHelper->ActivateDedicatedEpsBearer(ueDev.Get(u), bearer, tft);
    }

    std::cout << "NR Uu uplink topology initialized (" << maxVeh
              << " UEs, gNB at origin, remote=" << g_remoteHostAddr << ")"
              << std::endl;
}

static bool g_uuFirstTick = true;

// Payload chunking: keep each UDP datagram below typical MTU so IP doesn't
// fragment and overflow the NR RLC buffer. Large application payloads (e.g.
// 17KB compressed feature maps) are split into CHUNK_PAYLOAD-byte chunks at
// Send time. All chunks carry the same SeqTsHeader timestamp, so the delay
// measured on the LAST chunk is the time-to-fully-deliver the logical
// message. This mirrors what PDCP/RLC does at L2 in real 5G.
constexpr uint16_t CHUNK_PAYLOAD = 1400;

static void SendChunkedOn(Ptr<Socket> socket, uint32_t senderTag,
                          uint32_t totalBytes);

static void SendChunkedDownlink(uint32_t ueIdx, uint32_t totalBytes)
{
    SendChunkedOn(g_edgeDnSockets[ueIdx], ueIdx, totalBytes);
}

static void SendChunkedUplink(uint32_t ueIdx, uint32_t totalBytes)
{
    SendChunkedOn(g_ueUpSockets[ueIdx], ueIdx, totalBytes);
}

static void SendChunkedOn(Ptr<Socket> socket, uint32_t senderTag,
                          uint32_t totalBytes)
{
    // Split payload into <=CHUNK_PAYLOAD byte UDP datagrams so IP doesn't
    // fragment. Last chunk has bit 31 of seq set so the receiver records
    // delay once per logical message. senderTag identifies the UE index
    // (so both UL and DL rx traces can demux per-UE).
    const uint32_t headerOverhead = 12;
    const uint32_t chunkBody = CHUNK_PAYLOAD - headerOverhead;
    uint32_t remaining = totalBytes;
    uint32_t chunkIdx = 0;
    uint32_t totalChunks = (totalBytes + chunkBody - 1) / chunkBody;
    while (remaining > 0) {
        uint32_t thisChunk = std::min(chunkBody, remaining);
        SeqTsHeader seqTs;
        uint32_t seq = senderTag;
        if (chunkIdx == totalChunks - 1) seq |= 0x80000000u;
        seqTs.SetSeq(seq);
        Ptr<Packet> pkt = Create<Packet>(thisChunk);
        pkt->AddHeader(seqTs);
        socket->Send(pkt);
        remaining -= thisChunk;
        chunkIdx++;
    }
}

void AdvanceOneTickUu(uint16_t N, uint16_t payloadBytes,
                      double tickDurationS)
{
    g_tracker.Clear();
    g_dlTracker.clear();
    // DL payload: separate size for edge -> UE direction (predictions are
    // typically comparable to UL feature payload). Override via env var.
    uint32_t dlPayload = payloadBytes;
    if (const char* s = std::getenv("UU_DL_PAYLOAD_BYTES")) {
        dlPayload = static_cast<uint32_t>(std::atoi(s));
    }
    // First tick folds in the RRC/bearer warmup (~1.5s in cc-bwp-demo).
    double runFor = g_uuFirstTick ? (1.5 + tickDurationS) : tickDurationS;
    for (uint16_t i = 0; i < N; i++) {
        VehicleEntry* ve = getVehicle(i);
        if (!ve->tx_flag) continue;
        g_mobilityModels[i]->SetPosition(Vector(ve->x, ve->y, ve->z));
        if (g_uuFirstTick) {
            uint32_t ueIdx = i;
            uint32_t pb = payloadBytes;
            uint32_t dpb = dlPayload;
            Simulator::Schedule(Seconds(1.5), [ueIdx, pb, dpb](){
                SendChunkedUplink(ueIdx, pb);
                SendChunkedDownlink(ueIdx, dpb);
            });
        } else {
            SendChunkedUplink(i, payloadBytes);
            SendChunkedDownlink(i, dlPayload);
        }
    }
    Simulator::Stop(Seconds(runFor));
    Simulator::Run();
    g_uuFirstTick = false;
}

void WriteUuResults(uint16_t N) {
    // Two rows per active UE: UL (tx=UE, rx=0xFFFF) and DL (tx=0xFFFE, rx=UE).
    // Python side can demux by tx_id/rx_id to get UL and DL delay per UE.
    uint32_t resultIdx = 0;
    for (uint16_t u = 0; u < N; u++) {
        VehicleEntry* ve = getVehicle(u);
        if (!ve->tx_flag) continue;

        // UL row
        auto ulIt = g_tracker.results.find({(uint32_t)u, g_gnbNodeIdx});
        bool ulOk = ulIt != g_tracker.results.end() && ulIt->second.delivered;
        float ulDelay = ulOk ? ulIt->second.delay_ms : 0.0f;
        ShmLinkResult* lr = getLinkResult(resultIdx++);
        lr->tx_id = u;
        lr->rx_id = 0xFFFF;  // edge
        lr->delivered = ulOk ? 1 : 0;
        lr->sinr_db_x10 = static_cast<int16_t>(10);
        lr->delay_ms_x10 = static_cast<uint16_t>(
            std::min(static_cast<double>(ulDelay) * 10.0, 65535.0));

        // DL row
        auto dlIt = g_dlTracker.find(u);
        bool dlOk = dlIt != g_dlTracker.end() && dlIt->second.delivered;
        float dlDelay = dlOk ? dlIt->second.delay_ms : 0.0f;
        lr = getLinkResult(resultIdx++);
        lr->tx_id = 0xFFFE;  // edge -> UE marker
        lr->rx_id = u;
        lr->delivered = dlOk ? 1 : 0;
        lr->sinr_db_x10 = static_cast<int16_t>(10);
        lr->delay_ms_x10 = static_cast<uint16_t>(
            std::min(static_cast<double>(dlDelay) * 10.0, 65535.0));
    }
    ResultHeader* rh = getResultHeader();
    rh->n_results = resultIdx;
    uint32_t delivered_count = 0;
    for (uint32_t i = 0; i < resultIdx; i++) {
        if (getLinkResult(i)->delivered) delivered_count++;
    }
    rh->prr = resultIdx > 0 ? (float)delivered_count / resultIdx : 0;
    rh->mean_sinr_db = 0;
}

#endif // USE_NS3_FULL

// ===================================================================
//  Analytical SB-SPS fallback (for development without 5G-LENA)
// ===================================================================

void RunAnalyticalTick(uint16_t N, uint16_t payloadBytes,
                       double carrierGhz, double txPowerDbm)
{
    std::vector<Vector> positions(N);
    for (int i = 0; i < N; i++) {
        VehicleEntry* ve = getVehicle(i);
        positions[i] = Vector(ve->x, ve->y, ve->z);
    }

    // Subchannel count depends on bandwidth and subchannel size
    // 40 MHz / (10 RBs * 180 kHz/RB) = ~22 subchannels
    uint32_t numSubchannels = 20;

    // Number of subchannels needed per TB depends on payload
    // At MCS 14 with 10 RBs/subchannel: ~840 bytes per subchannel
    // Rule of thumb: ceil(payload / 840) subchannels per vehicle
    uint32_t subchPerVeh = std::max(1u,
        (uint32_t)std::ceil(payloadBytes / 840.0));

    // If a single vehicle needs more subchannels than available,
    // the payload cannot be delivered on sidelink
    bool payloadExceedsCapacity = (subchPerVeh > numSubchannels);

    uint32_t resultIdx = 0;

    for (int rx = 0; rx < N; rx++) {
        for (int tx = 0; tx < N; tx++) {
            if (tx == rx) continue;

            if (payloadExceedsCapacity) {
                ShmLinkResult* lr = getLinkResult(resultIdx);
                lr->tx_id = tx;
                lr->rx_id = rx;
                lr->delivered = 0;
                lr->sinr_db_x10 = -100;
                lr->delay_ms_x10 = 0;
                resultIdx++;
                continue;
            }

            double dx = positions[tx].x - positions[rx].x;
            double dy = positions[tx].y - positions[rx].y;
            double dz = positions[tx].z - positions[rx].z;
            double dist = std::sqrt(dx*dx + dy*dy + dz*dz);

            // WINNER+ B1 LOS path loss (3GPP TR 36.885)
            double d = std::max(dist, 3.0);
            double pl = 22.7 * std::log10(d)
                      + 27.0 + 20.0 * std::log10(carrierGhz);
            double rxPower = txPowerDbm - pl;
            double noiseFloor = -95.0;
            double sinr = rxPower - noiseFloor;

            // SB-SPS collision probability per subchannel
            // With subchPerVeh subchannels per vehicle, collision if
            // any other vehicle selects an overlapping subchannel set.
            // P(no collision on one subchannel) = ((S - subchPerVeh) / S)^(N-1)
            // P(no collision on all subchPerVeh) = P(one)^subchPerVeh
            double effectiveSubch = std::max(1.0,
                (double)(numSubchannels - subchPerVeh + 1));
            double pNoCollision = std::pow(
                (effectiveSubch - 1.0) / effectiveSubch, N - 1);
            pNoCollision = std::pow(pNoCollision, subchPerVeh);
            bool collision = (std::rand() / (double)RAND_MAX) > pNoCollision;

            bool delivered = !collision && sinr > 0.0;

            // Analytical sidelink delay synth: one selection-window (0.1ms
            // granularity) + payload transmission at MCS 14 rate (~4 Mbps
            // per subchannel). Order-of-magnitude only; real numbers come
            // from the USE_NS3_FULL path.
            double delayMs = 0.0;
            if (delivered) {
                double txMs = (payloadBytes * 8.0) / (4e6) * 1000.0;
                delayMs = 1.0 + txMs;  // 1 ms MAC + transmission
            }

            ShmLinkResult* lr = getLinkResult(resultIdx);
            lr->tx_id = tx;
            lr->rx_id = rx;
            lr->delivered = delivered ? 1 : 0;
            lr->sinr_db_x10 = static_cast<int16_t>(sinr * 10);
            lr->delay_ms_x10 = static_cast<uint16_t>(
                std::min(delayMs * 10.0, 65535.0));
            resultIdx++;
        }
    }

    // Write result header
    ResultHeader* rh = getResultHeader();
    rh->n_results = resultIdx;
    uint32_t delivered_count = 0;
    double sinr_sum = 0;
    for (uint32_t i = 0; i < resultIdx; i++) {
        ShmLinkResult* lr = getLinkResult(i);
        if (lr->delivered) delivered_count++;
        sinr_sum += lr->sinr_db_x10 / 10.0;
    }
    rh->prr = resultIdx > 0 ? (float)delivered_count / resultIdx : 0;
    rh->mean_sinr_db = resultIdx > 0 ? (float)(sinr_sum / resultIdx) : 0;
}

#ifdef USE_NS3_FULL
void WriteNs3Results(uint16_t N) {
    uint32_t resultIdx = 0;

    for (int rx = 0; rx < N; rx++) {
        for (int tx = 0; tx < N; tx++) {
            if (tx == rx) continue;

            auto it = g_tracker.results.find({(uint32_t)tx, (uint32_t)rx});
            bool delivered = false;
            float sinr = -10.0f;
            float delayMs = 0.0f;
            if (it != g_tracker.results.end()) {
                delivered = it->second.delivered;
                sinr = it->second.sinr_db;
                delayMs = it->second.delay_ms;
            }

            ShmLinkResult* lr = getLinkResult(resultIdx);
            lr->tx_id = tx;
            lr->rx_id = rx;
            lr->delivered = delivered ? 1 : 0;
            lr->sinr_db_x10 = static_cast<int16_t>(sinr * 10);
            lr->delay_ms_x10 = static_cast<uint16_t>(
                std::min(static_cast<double>(delayMs) * 10.0, 65535.0));
            resultIdx++;
        }
    }

    ResultHeader* rh = getResultHeader();
    rh->n_results = resultIdx;
    uint32_t delivered_count = 0;
    double sinr_sum = 0;
    for (uint32_t i = 0; i < resultIdx; i++) {
        ShmLinkResult* lr = getLinkResult(i);
        if (lr->delivered) delivered_count++;
        sinr_sum += lr->sinr_db_x10 / 10.0;
    }
    rh->prr = resultIdx > 0 ? (float)delivered_count / resultIdx : 0;
    rh->mean_sinr_db = resultIdx > 0 ? (float)(sinr_sum / resultIdx) : 0;
}
#endif

// ===================================================================
//  Main
// ===================================================================
int main(int argc, char* argv[])
{
    std::string shmPath;
    uint32_t maxVehicles = 128;
    double carrierGhz = 5.9;
    double bandwidthMhz = 40.0;
    double txPowerDbm = 23.0;
    uint16_t numerology = 0;
    uint8_t mode = 0;
    double tickDurationS = 0.05; // 50ms CARLA tick

    CommandLine cmd;
    cmd.AddValue("shm-path", "Shared memory file path", shmPath);
    cmd.AddValue("max-vehicles", "Maximum vehicles", maxVehicles);
    cmd.AddValue("carrier-ghz", "Carrier frequency", carrierGhz);
    cmd.AddValue("bandwidth-mhz", "Bandwidth", bandwidthMhz);
    cmd.AddValue("tx-power-dbm", "TX power", txPowerDbm);
    cmd.AddValue("numerology", "NR numerology", numerology);
    cmd.AddValue("mode", "0=V2V sidelink 1=Uu uplink", mode);
    cmd.AddValue("tick-duration", "Co-sim tick in seconds", tickDurationS);
    cmd.Parse(argc, argv);

    g_maxVehicles = maxVehicles;

    // Open shared memory
    int fd = open(shmPath.c_str(), O_RDWR);
    if (fd < 0) {
        std::cerr << "Failed to open shared memory: " << shmPath << std::endl;
        return 1;
    }

    g_shmSize = sizeof(ShmHeader)
              + maxVehicles * sizeof(VehicleEntry)
              + maxVehicles * maxVehicles * sizeof(ShmLinkResult)
              + sizeof(ResultHeader);

    g_shm = static_cast<uint8_t*>(
        mmap(nullptr, g_shmSize, PROT_READ | PROT_WRITE, MAP_SHARED, fd, 0));
    if (g_shm == MAP_FAILED) {
        std::cerr << "mmap failed" << std::endl;
        return 1;
    }

    std::cout << "NR V2X co-sim server started. SHM=" << shmPath
              << " maxVeh=" << maxVehicles
              << " mode=" << (mode == 0 ? "V2V_SIDELINK" : "UU_UPLINK")
              << std::endl;

#ifdef USE_NS3_FULL
    bool useNs3 = true;
    std::cout << "Using full ns-3 NR V2X stack (5G-LENA)" << std::endl;
#else
    std::cout << "Using analytical SB-SPS model (development mode)" << std::endl;
    std::cout << "Build with -DUSE_NS3_FULL for full ns-3 integration" << std::endl;
#endif

    // Signal ready
    getHeader()->state = STATE_IDLE;

    // Wait for first tick to get initial N and payload_bytes
    std::cout << "Waiting for first tick..." << std::endl;
    while (getHeader()->state != STATE_PYTHON_READY) {
        if (getHeader()->state == STATE_SHUTDOWN) {
            munmap(g_shm, g_shmSize);
            close(fd);
            return 0;
        }
        usleep(100);
    }

    uint16_t initialPayload = getHeader()->payload_bytes;
    uint16_t initialN = getHeader()->n_vehicles;
    std::cout << "First tick: N=" << initialN
              << " payload=" << initialPayload << "B" << std::endl;

#ifdef USE_NS3_FULL
    if (useNs3 && mode == 0) {
        SetupSidelinkTopology(maxVehicles, carrierGhz, bandwidthMhz,
                              txPowerDbm, numerology, initialPayload);
    } else if (useNs3 && mode == 1) {
        SetupUuTopology(maxVehicles, carrierGhz, bandwidthMhz,
                        txPowerDbm, numerology, initialPayload);
    }
#endif

    // Process the first tick that's already waiting
    goto process_tick;

    // Main co-simulation loop
    while (true) {
        // Wait for Python to signal
        while (getHeader()->state != STATE_PYTHON_READY) {
            if (getHeader()->state == STATE_SHUTDOWN) {
                std::cout << "Shutdown signal received" << std::endl;
#ifdef USE_NS3_FULL
                if (useNs3) Simulator::Destroy();
#endif
                munmap(g_shm, g_shmSize);
                close(fd);
                return 0;
            }
            usleep(100);
        }

    process_tick:
        auto t0 = std::chrono::high_resolution_clock::now();

        ShmHeader* hdr = getHeader();
        uint16_t N = hdr->n_vehicles;
        uint16_t payloadBytes = hdr->payload_bytes;
        uint32_t tick = hdr->tick;

#ifdef USE_NS3_FULL
        if (useNs3 && mode == 0) {
            UpdatePositions(N);
            AdvanceOneTick(tickDurationS);
            WriteNs3Results(N);
        } else if (useNs3 && mode == 1) {
            AdvanceOneTickUu(N, payloadBytes, tickDurationS);
            WriteUuResults(N);
        } else
#endif
        {
            RunAnalyticalTick(N, payloadBytes, carrierGhz, txPowerDbm);
        }

        auto t1 = std::chrono::high_resolution_clock::now();
        double elapsed_ms = std::chrono::duration<double, std::milli>(t1 - t0).count();

        getResultHeader()->channel_time_ms = elapsed_ms;

        if (tick % 100 == 0) {
            std::cout << "tick=" << tick << " N=" << N
                      << " payload=" << payloadBytes
                      << " PRR=" << getResultHeader()->prr
                      << " time=" << elapsed_ms << "ms" << std::endl;
        }

        // Signal Python
        hdr->state = STATE_NS3_DONE;
    }
}
