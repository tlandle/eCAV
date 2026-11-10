# Scenario status

Generated 2026-11-10 from a static check of every scenario on `develop`. A scenario passes when both files exist, the YAML parses and has a `scenario_runner` section, the CARLA scenario files it names exist, and every `manager_type` is registered. Passing does not mean the scenario was run. The families that ran in 2026 campaigns are listed as Supported in the README.

## Passes static checks

| Scenario | Edge manager |
|---|---|
| `blind_overtake_edge_late_fusion` | late_fusion |
| `blind_overtake_edge_late_fusion_2ego` | late_fusion |
| `blind_overtake_edge_late_fusion_mixed_2ego` | late_fusion |
| `highway_merge_edge_late_fusion` | late_fusion |
| `mv2x_datadump_town01` | no edge |
| `openscenario_1_accel_gt` | worldfusion_mamba_adaptive |
| `openscenario_1_accel_visible_gt` | worldfusion_mamba_adaptive |
| `openscenario_1_accel_worldfusion` | worldfusion_mamba_adaptive |
| `openscenario_1_burst_gt` | worldfusion_mamba_adaptive |
| `openscenario_1_corridor_gt` | worldfusion_mamba_adaptive |
| `openscenario_1_e6a_oracle` | worldfusion_mamba_adaptive |
| `openscenario_1_e6clat_gt_latency` | worldfusion_mamba_adaptive |
| `openscenario_1_edge` | late_fusion |
| `openscenario_1_edge_worldfusion` | worldfusion_mamba_adaptive |
| `openscenario_1_flow_gt` | worldfusion_mamba_adaptive |
| `openscenario_1_flow_nohandoff_gt` | worldfusion_mamba_adaptive |
| `openscenario_1_flow_single` | worldfusion_mamba_adaptive |
| `openscenario_1_flow_visible_gt` | worldfusion_mamba_adaptive |
| `openscenario_1_two_locale_worldfusion` | worldfusion_mamba_adaptive |
| `openscenario_20_edge_late_fusion_12cav` | late_fusion |
| `openscenario_20_edge_late_fusion_4ego` | late_fusion |
| `openscenario_20_edge_worldfusion_12cav` | worldfusion_adaptive |
| `openscenario_20_edge_worldfusion_4ego` | worldfusion_adaptive |
| `openscenario_21_edge_late_fusion_4ego` | late_fusion |
| `openscenario_21_edge_worldfusion_4ego` | worldfusion_adaptive |
| `openscenario_22_edge_late_fusion_4ego` | late_fusion |
| `openscenario_22_edge_worldfusion_4ego` | worldfusion_adaptive |
| `openscenario_23_edge_late_fusion_4ego` | late_fusion |
| `openscenario_23_edge_worldfusion_4ego` | worldfusion_adaptive |
| `openscenario_3` | no edge |
| `openscenario_3_edge_cip_smoke` | cip |
| `openscenario_3_edge_infra_only_1rsu_smoke` | infra_only |
| `openscenario_3_edge_infra_only_smoke` | infra_only |
| `openscenario_3_edge_late_fusion` | oracle |
| `openscenario_3_edge_late_fusion_16ego` | late_fusion |
| `openscenario_3_edge_late_fusion_2ego` | late_fusion |
| `openscenario_3_edge_late_fusion_4ego` | cip |
| `openscenario_3_edge_late_fusion_8ego` | late_fusion |
| `openscenario_3_edge_late_fusion_boundary` | late_fusion |
| `openscenario_3_edge_late_fusion_etsi_smoke` | late_fusion_etsi_self_announcement |
| `openscenario_3_edge_late_fusion_mixed_4ego` | late_fusion |
| `openscenario_3_edge_late_fusion_smoke` | late_fusion |
| `openscenario_3_edge_perception_2rsu_smoke` | perception |
| `openscenario_3_edge_route` | worldfusion |
| `openscenario_3_edge_worldfusion` | worldfusion |
| `openscenario_3_edge_worldfusion_4ego` | worldfusion_adaptive |
| `openscenario_3_edge_worldfusion_mtr_live` | worldfusion_adaptive |
| `openscenario_3_edge_worldfusion_smoke` | worldfusion_adaptive |
| `openscenario_3_multi_edge_late_fusion` | late_fusion |
| `openscenario_3_multi_edge_mamba` | sota |
| `openscenario_3_sota_late_fusion` | sota_edge |
| `openscenario_3_sota_worldfusion` | sota_edge |
| `openscenario_multi_edge_right_merge` | late_fusion |
| `openscenario_multi_edge_right_merge_worldfusion` | worldfusion_mamba_adaptive |
| `scp_edge_late_fusion` | late_fusion |
| `scp_edge_late_fusion_16ego` | late_fusion |
| `scp_edge_late_fusion_2ego` | late_fusion |
| `scp_edge_late_fusion_4ego` | late_fusion |
| `scp_edge_late_fusion_8ego` | late_fusion |
| `scp_edge_late_fusion_mixed_4ego` | late_fusion |

## Deprecated (does not start)

| Scenario | Reason |
|---|---|
| `ecloud_4lane_8car_scenario` | no YAML |
| `ecloud_4lane_dist_128_car` | no scenario_runner section |
| `ecloud_4lane_dist_16_car` | no scenario_runner section |
| `ecloud_4lane_dist_24_car` | no scenario_runner section |
| `ecloud_4lane_dist_2_car` | no scenario_runner section |
| `ecloud_4lane_dist_4_car` | no scenario_runner section |
| `ecloud_4lane_scenario` | no scenario_runner section |
| `ecloud_4lane_scenario_dist` | no scenario_runner section |
| `ecloud_4lane_scenario_dist_128_car` | YAML does not parse |
| `ecloud_4lane_scenario_dist_128_car_new_spec` | no runner .py |
| `ecloud_4lane_scenario_dist_16_car` | no scenario_runner section |
| `ecloud_4lane_scenario_dist_16_car_perception` | no scenario_runner section |
| `ecloud_4lane_scenario_dist_32_car` | no scenario_runner section |
| `ecloud_4lane_scenario_dist_32_car_perception` | no scenario_runner section |
| `ecloud_4lane_scenario_dist_48_car` | no scenario_runner section |
| `ecloud_4lane_scenario_dist_48_car_perception` | no scenario_runner section |
| `ecloud_4lane_scenario_dist_60_car` | no scenario_runner section |
| `ecloud_4lane_scenario_dist_64_car` | no scenario_runner section |
| `ecloud_4lane_scenario_dist_8_car` | YAML does not parse |
| `ecloud_4lane_scenario_dist_8_car_2` | no scenario_runner section |
| `ecloud_4lane_scenario_dist_8_car_perception` | no scenario_runner section |
| `ecloud_4lane_scenario_dist_config` | no scenario_runner section |
| `ecloud_edge_10_car_1_rsu_comparison_srunner` | no scenario_runner section |
| `ecloud_edge_10_car_1_rsu_srunner` | no scenario_runner section |
| `ecloud_edge_10_car_comparison_srunner` | no scenario_runner section |
| `ecloud_edge_16_car` | no scenario_runner section |
| `ecloud_edge_1_car_left_turn` | no scenario_runner section |
| `ecloud_edge_1_car_left_turn_comparison` | no scenario_runner section |
| `ecloud_edge_1_car_left_turn_perception_no_traffic` | no scenario_runner section |
| `ecloud_edge_4_car` | no scenario_runner section |
| `ecloud_edge_4_car_comparison` | no scenario_runner section |
| `ecloud_edge_4_car_comparison_srunner` | no scenario_runner section |
| `ecloud_edge_8_car` | no scenario_runner section |
| `ecloud_edge_scenario` | no scenario_runner section |
| `ecloud_perception_non_perception` | no scenario_runner section |
| `ecloud_template_4lane_scenario_dist` | no scenario_runner section |
| `metro_scale_scenario` | no runner .py |
| `multi_2lanefree_carla` | no scenario_runner section |
| `openscenario_1` | scenario file path does not exist (configFile, additionalScenario) |
| `openscenario_11` | scenario file path does not exist (configFile, additionalScenario) |
| `openscenario_12` | scenario file path does not exist (configFile, additionalScenario) |
| `openscenario_13` | scenario file path does not exist (configFile, additionalScenario) |
| `openscenario_13_edge` | scenario file path does not exist (configFile, additionalScenario) |
| `openscenario_14` | scenario file path does not exist (configFile, additionalScenario) |
| `openscenario_15` | scenario file path does not exist (configFile, additionalScenario) |
| `openscenario_16` | scenario file path does not exist (configFile, additionalScenario) |
| `openscenario_17` | scenario file path does not exist (configFile, additionalScenario) |
| `openscenario_18` | scenario file path does not exist (configFile, additionalScenario) |
| `openscenario_1_e6b_ego_local` | scenario file path does not exist (configFile, additionalScenario) |
| `openscenario_2` | scenario file path does not exist (configFile, additionalScenario) |
| `openscenario_20_deoc_edge_late_fusion_12cav` | no runner .py |
| `openscenario_20_deoc_edge_late_fusion_4ego` | no runner .py |
| `openscenario_20_deoc_edge_worldfusion_12cav` | no runner .py |
| `openscenario_20_deoc_edge_worldfusion_4ego` | no runner .py |
| `openscenario_20_deoc_edge_worldfusion_static_12cav` | no runner .py |
| `openscenario_20_deoc_edge_worldfusion_static_4ego` | no runner .py |
| `openscenario_20_edge_worldfusion_static_12cav` | no runner .py |
| `openscenario_20_edge_worldfusion_static_4ego` | no runner .py |
| `openscenario_3_edge` | manager_type not registered (bm2cp) |
| `openscenario_3_edge_worldfusion_mtr` | no runner .py |
| `openscenario_3_fog_100` | no runner .py |
| `openscenario_3_fog_25` | no runner .py |
| `openscenario_3_fog_50` | no runner .py |
| `openscenario_3_fog_75` | no runner .py |
| `openscenario_3_harbor` | manager_type not registered (harbor) |
| `openscenario_3_light_100` | no runner .py |
| `openscenario_3_light_25` | no runner .py |
| `openscenario_3_light_50` | no runner .py |
| `openscenario_3_light_75` | no runner .py |
| `openscenario_3_rain_100` | no runner .py |
| `openscenario_3_rain_25` | no runner .py |
| `openscenario_3_rain_50` | no runner .py |
| `openscenario_3_rain_75` | no runner .py |
| `openscenario_3_v2v_autocast` | manager_type not registered (v2v_autocast) |
| `openscenario_3_v2v_cmp` | manager_type not registered (cmp) |
| `openscenario_3_v2v_cmp_16ego` | manager_type not registered (cmp) |
| `openscenario_3_v2v_cmp_2ego` | manager_type not registered (cmp) |
| `openscenario_3_v2v_cmp_4ego` | manager_type not registered (cmp) |
| `openscenario_3_v2v_cmp_8ego` | manager_type not registered (cmp) |
| `openscenario_3_v2v_coop` | manager_type not registered (v2v_coop) |
| `openscenario_3_v2v_intermediate` | manager_type not registered (v2v_coop) |
| `openscenario_3_v2v_intermediate_16ego` | manager_type not registered (v2v_coop) |
| `openscenario_3_v2v_intermediate_2ego` | manager_type not registered (v2v_coop) |
| `openscenario_3_v2v_intermediate_4ego` | manager_type not registered (v2v_coop) |
| `openscenario_3_v2v_intermediate_8ego` | manager_type not registered (v2v_coop) |
| `openscenario_3_v2v_late_fusion` | manager_type not registered (v2v_autocast) |
| `openscenario_3_v2v_late_fusion_16ego` | manager_type not registered (v2v_autocast) |
| `openscenario_3_v2v_late_fusion_2ego` | manager_type not registered (v2v_autocast) |
| `openscenario_3_v2v_late_fusion_4ego` | manager_type not registered (v2v_autocast) |
| `openscenario_3_v2v_late_fusion_8ego` | manager_type not registered (v2v_autocast) |
| `openscenario_4` | scenario file path does not exist (configFile, additionalScenario) |
| `openscenario_5` | scenario file path does not exist (configFile, additionalScenario) |
| `openscenario_6` | scenario file path does not exist (configFile, additionalScenario) |
| `openscenario_7` | scenario file path does not exist (configFile, additionalScenario) |
| `openscenario_8` | scenario file path does not exist (configFile, additionalScenario) |
| `openscenario_9` | scenario file path does not exist (configFile, additionalScenario) |
| `openscenario_9_edge` | scenario file path does not exist (configFile, additionalScenario) |
| `platoon_joining_2lanefree_carla` | no scenario_runner section |
| `platoon_joining_2lanefree_cosim` | no scenario_runner section |
| `platoon_joining_town06_carla` | no scenario_runner section |
| `platoon_stability_2lanefree_carla` | no scenario_runner section |
| `scp_edge` | no runner .py |
| `scp_non_edge` | no runner .py |
| `single_2lanefree_carla` | no scenario_runner section |
| `single_2lanefree_cosim` | no scenario_runner section |
| `single_intersection_town06_carla` | no scenario_runner section |
| `single_town05_cosim` | no scenario_runner section |
| `single_town06_carla` | no scenario_runner section |
| `single_town06_cosim` | no scenario_runner section |
| `v2xp_datadump_town06_carla` | no scenario_runner section |

## Helper scripts in the scenario directory (not scenarios)

- `generate_v2v_figures.py`
- `generate_v2v_scenarios.py`
- `v2v_n_sweep.py`
