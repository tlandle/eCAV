/**
 * channel_engine.h
 * Author: Tyler Landle <tlandle3@gatech.edu>
 *
 * Standalone C++ channel engine for C-V2X Mode 4 / NR-V2X sidelink.
 * Extracts WINNER+ B1 (3GPP TR 36.885) propagation from ns-3 CNI model
 * and implements SB-SPS MAC with per-link SINR and capture effect.
 *
 * Compiled as pybind11 shared library. No ns-3 dependency at runtime.
 */

#ifndef CHANNEL_ENGINE_H
#define CHANNEL_ENGINE_H

#include <array>
#include <cstdint>
#include <random>
#include <vector>

namespace v2x {

struct LinkResult {
    int tx_id;
    int rx_id;
    bool delivered;
    float sinr_db;
    float delay_ms;
};

struct OcclusionInfo {
    bool building_blocked;
    int num_vehicles_blocking;
    float extra_loss_db;
};

class ChannelEngine {
public:
    ChannelEngine(float carrier_ghz = 5.9f,
                  float bw_mhz = 10.0f,
                  float tx_power_dbm = 23.0f,
                  int num_subchannels = 20,
                  int rc_min = 5,
                  int rc_max = 15,
                  float p_keep = 0.0f,
                  float sinr_thresh_db = 0.0f,
                  float antenna_h = 1.5f,
                  float noise_floor_dbm = -95.0f,
                  float shadow_std_los = 3.0f,
                  float shadow_std_nlos = 4.0f,
                  float rician_k_db = 3.0f);

    std::vector<LinkResult> compute_tick(
        const std::vector<std::array<float, 3>>& positions,
        const std::vector<std::vector<OcclusionInfo>>& occlusion,
        int tick);

    std::vector<int> get_subchannel_assignments() const;
    void reset();

    static float backhaul_queue_delay_ms(int n_vehicles,
                                         int feature_bytes = 175000,
                                         float backhaul_bw_mbps = 100.0f);

private:
    float carrier_ghz_, bw_mhz_, tx_power_dbm_, antenna_h_;
    float noise_floor_dbm_, sinr_thresh_db_;
    float shadow_std_los_, shadow_std_nlos_, rician_k_linear_;
    int num_subchannels_, rc_min_, rc_max_;
    float p_keep_;

    std::vector<int> subchannel_;
    std::vector<int> reselect_counter_;

    std::mt19937 rng_;
    std::normal_distribution<float> normal_dist_;
    std::uniform_real_distribution<float> uniform_01_;
    std::exponential_distribution<float> rayleigh_dist_;

    float path_loss_los(float dist_m) const;
    float path_loss_nlos(float dist_m) const;
    float path_loss_freespace(float dist_m) const;
    float compute_path_loss(float dist_m, const OcclusionInfo& occ);
    float sample_fading(bool los);
    float sample_shadow(bool los);
    void update_sps(int n_vehicles);
    static float distance(const std::array<float, 3>& a,
                          const std::array<float, 3>& b);
};

}  // namespace v2x

#endif  // CHANNEL_ENGINE_H
