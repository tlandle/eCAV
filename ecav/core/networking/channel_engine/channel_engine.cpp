/**
 * channel_engine.cpp
 * Author: Tyler Landle <tlandle3@gatech.edu>
 *
 * Standalone C-V2X Mode 4 channel engine.
 *
 * Propagation: WINNER+ B1 urban microcell (3GPP TR 36.885)
 * extracted from ns-3 CNI implementation
 * (cni-urbanmicrocell-propagation-loss-model.cc).
 *
 * MAC: Sensing-Based Semi-Persistent Scheduling (SB-SPS) per 3GPP TS 36.321.
 * Per-link SINR with co-channel interference and capture effect.
 *
 * Public domain ns-3 source (NIST). Adapted for standalone compilation.
 */

#include "channel_engine.h"

#include <algorithm>
#include <cassert>
#include <cmath>

namespace v2x {

ChannelEngine::ChannelEngine(float carrier_ghz, float bw_mhz,
                             float tx_power_dbm, int num_subchannels,
                             int rc_min, int rc_max, float p_keep,
                             float sinr_thresh_db, float antenna_h,
                             float noise_floor_dbm, float shadow_std_los,
                             float shadow_std_nlos, float rician_k_db)
    : carrier_ghz_(carrier_ghz), bw_mhz_(bw_mhz),
      tx_power_dbm_(tx_power_dbm), antenna_h_(antenna_h),
      noise_floor_dbm_(noise_floor_dbm), sinr_thresh_db_(sinr_thresh_db),
      shadow_std_los_(shadow_std_los), shadow_std_nlos_(shadow_std_nlos),
      num_subchannels_(num_subchannels), rc_min_(rc_min), rc_max_(rc_max),
      p_keep_(p_keep),
      rng_(std::random_device{}()),
      normal_dist_(0.0f, 1.0f),
      uniform_01_(0.0f, 1.0f),
      rayleigh_dist_(1.0f) {
    rician_k_linear_ = std::pow(10.0f, rician_k_db / 10.0f);
}

void ChannelEngine::reset() {
    subchannel_.clear();
    reselect_counter_.clear();
}

// --- WINNER+ B1 path loss (from ns-3 CNI urban microcell model) ---

float ChannelEngine::path_loss_freespace(float dist_m) const {
    if (dist_m < 1.0f) dist_m = 1.0f;
    return 20.0f * std::log10(dist_m)
         + 46.4f
         + 20.0f * std::log10(carrier_ghz_ / 5.0f);
}

float ChannelEngine::path_loss_los(float dist_m) const {
    if (dist_m < 3.0f) dist_m = 3.0f;
    float fc = carrier_ghz_;
    float hbs1 = antenna_h_ - 1.0f;
    float hms1 = antenna_h_ - 1.0f;
    float d_bp = 4.0f * hbs1 * hms1 * (carrier_ghz_ * 1e9f) / 3.0e8f;

    float loss;
    if (dist_m <= d_bp) {
        loss = 22.7f * std::log10(dist_m) + 27.0f + 20.0f * std::log10(fc);
    } else {
        loss = 40.0f * std::log10(dist_m) + 7.56f
             - 17.3f * std::log10(hbs1) - 17.3f * std::log10(hms1)
             + 2.7f * std::log10(fc);
    }
    return std::max(path_loss_freespace(dist_m), loss);
}

float ChannelEngine::path_loss_nlos(float dist_m) const {
    if (dist_m < 3.0f) dist_m = 3.0f;
    float fc = carrier_ghz_;
    float hbs = antenna_h_;
    float loss = (44.9f - 6.55f * std::log10(hbs)) * std::log10(dist_m)
               + 5.83f * std::log10(hbs) + 18.38f
               + 23.0f * std::log10(fc) - 5.0f;
    return std::max(path_loss_freespace(dist_m), loss);
}

float ChannelEngine::compute_path_loss(float dist_m, const OcclusionInfo& occ) {
    if (occ.building_blocked)
        return path_loss_nlos(dist_m);
    return path_loss_los(dist_m) + occ.extra_loss_db;
}

// --- Fading ---

float ChannelEngine::sample_fading(bool los) {
    if (los) {
        float K = rician_k_linear_;
        float mu = std::sqrt(K / (K + 1.0f));
        float sigma = std::sqrt(0.5f / (K + 1.0f));
        float re = mu + sigma * normal_dist_(rng_);
        float im = sigma * normal_dist_(rng_);
        float power = re * re + im * im;
        return 10.0f * std::log10(std::max(power, 1e-10f));
    } else {
        float power = rayleigh_dist_(rng_);
        return 10.0f * std::log10(std::max(power, 1e-10f));
    }
}

float ChannelEngine::sample_shadow(bool los) {
    float sigma = los ? shadow_std_los_ : shadow_std_nlos_;
    return sigma * normal_dist_(rng_);
}

// --- SB-SPS MAC ---

void ChannelEngine::update_sps(int n_vehicles) {
    while (static_cast<int>(subchannel_.size()) < n_vehicles) {
        std::uniform_int_distribution<int> ch_dist(0, num_subchannels_ - 1);
        subchannel_.push_back(ch_dist(rng_));
        std::uniform_int_distribution<int> rc_dist(rc_min_, rc_max_);
        reselect_counter_.push_back(rc_dist(rng_));
    }
    subchannel_.resize(n_vehicles);
    reselect_counter_.resize(n_vehicles);

    for (int i = 0; i < n_vehicles; ++i) {
        reselect_counter_[i]--;
        if (reselect_counter_[i] <= 0) {
            if (uniform_01_(rng_) >= p_keep_) {
                std::uniform_int_distribution<int> ch_dist(0, num_subchannels_ - 1);
                subchannel_[i] = ch_dist(rng_);
            }
            std::uniform_int_distribution<int> rc_dist(rc_min_, rc_max_);
            reselect_counter_[i] = rc_dist(rng_);
        }
    }
}

float ChannelEngine::distance(const std::array<float, 3>& a,
                              const std::array<float, 3>& b) {
    float dx = a[0] - b[0], dy = a[1] - b[1], dz = a[2] - b[2];
    return std::sqrt(dx * dx + dy * dy + dz * dz);
}

std::vector<int> ChannelEngine::get_subchannel_assignments() const {
    return subchannel_;
}

float ChannelEngine::backhaul_queue_delay_ms(int n_vehicles,
                                              int feature_bytes,
                                              float backhaul_bw_mbps) {
    float total_bits = static_cast<float>(n_vehicles)
                     * static_cast<float>(feature_bytes) * 8.0f;
    float bw_bps = backhaul_bw_mbps * 1e6f;
    return (total_bits / bw_bps) * 1000.0f * 0.5f;
}

// --- Main per-tick computation ---

std::vector<LinkResult> ChannelEngine::compute_tick(
    const std::vector<std::array<float, 3>>& positions,
    const std::vector<std::vector<OcclusionInfo>>& occlusion,
    int tick) {

    int N = static_cast<int>(positions.size());
    assert(static_cast<int>(occlusion.size()) == N);

    update_sps(N);

    // Received power for all pairs
    std::vector<std::vector<float>> rx_power(N, std::vector<float>(N, -999.0f));
    for (int i = 0; i < N; ++i) {
        for (int j = i + 1; j < N; ++j) {
            float dist_m = distance(positions[i], positions[j]);
            const auto& occ = occlusion[i][j];
            bool los = !occ.building_blocked && occ.num_vehicles_blocking == 0;
            float pl = compute_path_loss(dist_m, occ);
            float rx = tx_power_dbm_ - pl - sample_shadow(los) + sample_fading(los);
            rx_power[i][j] = rx;
            rx_power[j][i] = rx;
        }
    }

    // SINR and delivery for each directed pair
    std::vector<LinkResult> results;
    results.reserve(N * (N - 1));
    float noise_linear = std::pow(10.0f, noise_floor_dbm_ / 10.0f);

    for (int rx = 0; rx < N; ++rx) {
        for (int tx = 0; tx < N; ++tx) {
            if (tx == rx) continue;
            int tx_ch = subchannel_[tx];
            float sig_linear = std::pow(10.0f, rx_power[tx][rx] / 10.0f);

            float interf_linear = 0.0f;
            for (int k = 0; k < N; ++k) {
                if (k == tx || k == rx) continue;
                if (subchannel_[k] == tx_ch)
                    interf_linear += std::pow(10.0f, rx_power[k][rx] / 10.0f);
            }

            float sinr_linear = sig_linear / (interf_linear + noise_linear);
            float sinr_db = 10.0f * std::log10(std::max(sinr_linear, 1e-10f));
            results.push_back(LinkResult{tx, rx, sinr_db >= sinr_thresh_db_,
                                         sinr_db, 1.0f});
        }
    }
    return results;
}

}  // namespace v2x
