/**
 * bindings.cpp -- pybind11 bindings for the C-V2X channel engine.
 * Author: Tyler Landle <tlandle3@gatech.edu>
 */

#include <pybind11/pybind11.h>
#include <pybind11/stl.h>
#include "channel_engine.h"

namespace py = pybind11;

PYBIND11_MODULE(channel_engine, m) {
    m.doc() = "C-V2X Mode 4 channel engine with WINNER+ B1 and SB-SPS MAC";

    py::class_<v2x::OcclusionInfo>(m, "OcclusionInfo")
        .def(py::init([](bool bb, int nv, float db) {
            return v2x::OcclusionInfo{bb, nv, db};
        }), py::arg("building_blocked"), py::arg("num_vehicles_blocking"),
           py::arg("extra_loss_db"))
        .def_readwrite("building_blocked", &v2x::OcclusionInfo::building_blocked)
        .def_readwrite("num_vehicles_blocking", &v2x::OcclusionInfo::num_vehicles_blocking)
        .def_readwrite("extra_loss_db", &v2x::OcclusionInfo::extra_loss_db);

    py::class_<v2x::LinkResult>(m, "LinkResult")
        .def_readonly("tx_id", &v2x::LinkResult::tx_id)
        .def_readonly("rx_id", &v2x::LinkResult::rx_id)
        .def_readonly("delivered", &v2x::LinkResult::delivered)
        .def_readonly("sinr_db", &v2x::LinkResult::sinr_db)
        .def_readonly("delay_ms", &v2x::LinkResult::delay_ms);

    py::class_<v2x::ChannelEngine>(m, "ChannelEngine")
        .def(py::init<float,float,float,int,int,int,float,float,float,float,float,float,float>(),
             py::arg("carrier_ghz")=5.9f, py::arg("bw_mhz")=10.0f,
             py::arg("tx_power_dbm")=23.0f, py::arg("num_subchannels")=20,
             py::arg("rc_min")=5, py::arg("rc_max")=15, py::arg("p_keep")=0.0f,
             py::arg("sinr_thresh_db")=0.0f, py::arg("antenna_h")=1.5f,
             py::arg("noise_floor_dbm")=-95.0f, py::arg("shadow_std_los")=3.0f,
             py::arg("shadow_std_nlos")=4.0f, py::arg("rician_k_db")=3.0f)
        .def("compute_tick", &v2x::ChannelEngine::compute_tick)
        .def("get_subchannel_assignments", &v2x::ChannelEngine::get_subchannel_assignments)
        .def("reset", &v2x::ChannelEngine::reset)
        .def_static("backhaul_queue_delay_ms", &v2x::ChannelEngine::backhaul_queue_delay_ms,
             py::arg("n_vehicles"), py::arg("feature_bytes")=175000,
             py::arg("backhaul_bw_mbps")=100.0f);
}
