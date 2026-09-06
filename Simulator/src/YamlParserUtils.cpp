#include <Simulator/YamlParserUtils.h>
#include <UserCommon/ConfigParserUtils.h>
#include <UserCommon/TimeUtils.h>
#include <Common/Units.h>
#include <TinyNPY.h>

#include <algorithm>
#include <fstream>
#include <iostream>

using namespace common;
using namespace user_common_330371063_324976703;

namespace simulator {

namespace {
    common::types::DroneConfigData parseDroneConfig(const std::filesystem::path& path) {
        YAML::Node config = YAML::LoadFile(path.string());
        if (!config["drone_config"]) throw std::invalid_argument("Missing drone_config");
        YAML::Node dc = config["drone_config"];

        common::types::DroneConfigData drone;
        drone.radius = (get_with_check<double>(dc, "dimensions_cm", 30.0, [](const double& v){ return v > 0.0; }, "dimensions_cm must be > 0") / 2.0) * cm;
        drone.max_rotate = get_with_check<double>(dc, "max_rotate_deg", 90.0, [](const double& v){ return v > 0.0 && v <= 360.0; }, "max_rotate_deg must be in (0, 360]") * horizontal_angle[deg];
        drone.max_advance = get_with_check<double>(dc, "max_advance_cm", 100.0, [](const double& v){ return v > 0.0; }, "max_advance_cm must be > 0") * cm;
        drone.max_elevate = get_with_check<double>(dc, "max_elevate_cm", 200.0, [](const double& v){ return v > 0.0; }, "max_elevate_cm must be > 0") * cm;
        return drone;
    }

    common::types::LidarConfigData parseLidarConfig(const std::filesystem::path& path) {
        YAML::Node config = YAML::LoadFile(path.string());
        if (!config["lidar_config"]) throw std::invalid_argument("Missing lidar_config");
        YAML::Node lc = config["lidar_config"];

        common::types::LidarConfigData lidar;
        double z_max = get_with_default<double>(lc, "z_max_cm", 200.0);
        double z_min = get_with_check<double>(lc, "z_min_cm", 0.0, [](const double& v){ return v >= 0.0; }, "z_min_cm must be >= 0");
        if (z_min > z_max) throw std::invalid_argument("z_max_cm must be >= z_min_cm");
        
        lidar.z_min = z_min * cm;
        lidar.z_max = z_max * cm;
        lidar.d = get_with_check<double>(lc, "d_cm", 1.0, [](const double& v){ return v > 0.0; }, "d_cm must be > 0") * cm;
        lidar.fov_circles = get_with_check<std::size_t>(lc, "fov_circles", 2, [](const std::size_t& v){ return true; }, ""); // can be 0
        return lidar;
    }

    common::types::MissionConfigData parseMissionConfig(const std::filesystem::path& path, const common::types::MappingBounds& default_bounds) {
        YAML::Node config = YAML::LoadFile(path.string());
        if (!config["mission_config"]) throw std::invalid_argument("Missing mission_config");
        YAML::Node mc = config["mission_config"];

        common::types::MissionConfigData mission;
        mission.max_steps = get_with_check<std::size_t>(mc, "max_steps", 2500, [](const std::size_t& v){ return true; }, "");
        mission.gps_resolution = get_with_check<double>(mc, "gps_resolution_cm", 1.0, [](const double& v){ return v > 0.0; }, "gps_resolution_cm must be > 0") * cm;
        mission.output_mapping_resolution_factor = get_with_default<double>(mc, "output_mapping_resolution_factor", 1.0);

        // Bounds
        double min_x = default_bounds.min_x.numerical_value_in(cm);
        double max_x = default_bounds.max_x.numerical_value_in(cm);
        double min_y = default_bounds.min_y.numerical_value_in(cm);
        double max_y = default_bounds.max_y.numerical_value_in(cm);
        double min_z = default_bounds.min_height.numerical_value_in(cm);
        double max_z = default_bounds.max_height.numerical_value_in(cm);

        if (mc["boundaries"]) {
            YAML::Node b = mc["boundaries"];
            if (b["x_boundary"]) {
                min_x = get_with_default<double>(b["x_boundary"], "min_cm", 0.0) + default_bounds.min_x.numerical_value_in(cm);
                max_x = get_with_default<double>(b["x_boundary"], "max_cm", default_bounds.max_x.numerical_value_in(cm) - default_bounds.min_x.numerical_value_in(cm)) + default_bounds.min_x.numerical_value_in(cm);
            }
            if (b["y_boundary"]) {
                min_y = get_with_default<double>(b["y_boundary"], "min_cm", 0.0) + default_bounds.min_y.numerical_value_in(cm);
                max_y = get_with_default<double>(b["y_boundary"], "max_cm", default_bounds.max_y.numerical_value_in(cm) - default_bounds.min_y.numerical_value_in(cm)) + default_bounds.min_y.numerical_value_in(cm);
            }
            if (b["height_boundary"]) {
                min_z = get_with_default<double>(b["height_boundary"], "min_cm", 0.0) + default_bounds.min_height.numerical_value_in(cm);
                max_z = get_with_default<double>(b["height_boundary"], "max_cm", default_bounds.max_height.numerical_value_in(cm) - default_bounds.min_height.numerical_value_in(cm)) + default_bounds.min_height.numerical_value_in(cm);
            }
        }

        if (min_x > max_x) throw std::invalid_argument("min_x must be <= max_x");
        if (min_y > max_y) throw std::invalid_argument("min_y must be <= max_y");
        if (min_z > max_z) throw std::invalid_argument("min_height must be <= max_height");

        mission.mission_bounds.min_x = min_x * x_extent[cm];
        mission.mission_bounds.max_x = max_x * x_extent[cm];
        mission.mission_bounds.min_y = min_y * y_extent[cm];
        mission.mission_bounds.max_y = max_y * y_extent[cm];
        mission.mission_bounds.min_height = min_z * z_extent[cm];
        mission.mission_bounds.max_height = max_z * z_extent[cm];

        return mission;
    }

    types::SimulationConfigData parseSimulationConfig(const std::filesystem::path& path) {
        YAML::Node config = YAML::LoadFile(path.string());
        if (!config["simulation_config"]) throw std::invalid_argument("Missing simulation_config");
        YAML::Node node = config["simulation_config"];
        
        types::SimulationConfigData sim;
        sim.map_filename = get_with_check<std::string>(node, "map_filename", "", [](const std::string& v){ return !v.empty(); }, "Missing or empty 'map_filename'");
        sim.map_resolution = get_with_check<double>(node, "map_resolution_cm", 1.0, [](const double& v){ return v > 0.0; }, "map_resolution_cm must be > 0") * cm;
        
        double offset_x = 0.0, offset_y = 0.0, offset_z = 0.0;
        if (node["map_axes_offset"]) {
            YAML::Node o = node["map_axes_offset"];
            offset_x = get_with_default<double>(o, "x_offset", 0.0);
            offset_y = get_with_default<double>(o, "y_offset", 0.0);
            offset_z = get_with_default<double>(o, "height_offset", 0.0);
        }
        sim.map_offset = common::Position3D{offset_x * x_extent[cm], offset_y * y_extent[cm], offset_z * z_extent[cm]};

        double init_x = 0.0, init_y = 0.0, init_z = 0.0;
        if (node["initial_drone_position"]) {
            YAML::Node i = node["initial_drone_position"];
            init_x = get_with_default<double>(i, "x_cm", 0.0);
            init_y = get_with_default<double>(i, "y_cm", 0.0);
            init_z = get_with_default<double>(i, "height_cm", 0.0);
        }
        sim.initial_drone_position = common::Position3D{(init_x + offset_x) * x_extent[cm], (init_y + offset_y) * y_extent[cm], (init_z + offset_z) * z_extent[cm]};
        sim.initial_angle = get_with_default<double>(node, "initial_angle_deg", 0.0) * horizontal_angle[deg];

        return sim;
    }
} // namespace

types::SimulationCompositionData YamlParserUtils::parseCompositions(const std::filesystem::path& sim_path) {
    YAML::Node config = YAML::LoadFile(sim_path.string());
    types::SimulationCompositionData composition;
    composition.composition_file = sim_path;

    std::filesystem::path base_dir = sim_path.parent_path();
    
    if (config.IsNull() || !config["simulation_compositions"]) {
        throw std::invalid_argument("Missing simulation_compositions key");
    }
    YAML::Node comp_yaml = config["simulation_compositions"];
    
    if (!comp_yaml["simulations"]) throw std::invalid_argument("Missing simulations");
    if (!comp_yaml["drone_configs"]) throw std::invalid_argument("Missing drone_configs");
    if (!comp_yaml["lidar_configs"]) throw std::invalid_argument("Missing lidar_configs");
    
    if (comp_yaml["simulations"]) {
        for (const auto& sim_node : comp_yaml["simulations"]) {
            std::filesystem::path sim_config_path = base_dir / sim_node["simulation_config"].as<std::string>();
            types::SimulationConfigData sim = parseSimulationConfig(sim_config_path);
            if (!sim.map_filename.is_absolute()) {
                sim.map_filename = std::filesystem::weakly_canonical(base_dir / sim.map_filename);
            }
            
            common::types::MappingBounds default_bounds;
            std::filesystem::path npy_path = sim.map_filename;
            try {
                auto arr = std::make_shared<NpyArray>();
                const char* err = arr->LoadNPY(npy_path.string().c_str());
                if (err == nullptr && arr->Shape().size() == 3) {
                    const auto& shape = arr->Shape();
                    double res_cm = sim.map_resolution.numerical_value_in(cm);
                    
                    default_bounds.min_x = sim.map_offset.x;
                    default_bounds.min_y = sim.map_offset.y;
                    default_bounds.min_height = sim.map_offset.z;
                    default_bounds.max_x = sim.map_offset.x + (static_cast<double>(shape[0]) * res_cm * x_extent[cm]);
                    default_bounds.max_y = sim.map_offset.y + (static_cast<double>(shape[1]) * res_cm * y_extent[cm]);
                    default_bounds.max_height = sim.map_offset.z + (static_cast<double>(shape[2]) * res_cm * z_extent[cm]);
                }
            } catch (...) {
                // Ignore load errors here
            }

            std::vector<common::types::MissionConfigData> missions_for_sim;
            if (sim_node["mission_configs"]) {
                for (const auto& mission_path_node : sim_node["mission_configs"]) {
                    std::filesystem::path mission_path = base_dir / mission_path_node.as<std::string>();
                    common::types::MissionConfigData mission = parseMissionConfig(mission_path, default_bounds);
                    missions_for_sim.push_back(mission);
                }
            }

            composition.simulation_mission_groups.push_back({sim, missions_for_sim});
        }
    }

    if (comp_yaml["drone_configs"]) {
        for (const auto& drone_path_node : comp_yaml["drone_configs"]) {
            std::filesystem::path drone_path = base_dir / drone_path_node.as<std::string>();
            common::types::DroneConfigData drone = parseDroneConfig(drone_path);
            composition.drone_configs.push_back(drone);
        }
    }
    if (comp_yaml["lidar_configs"]) {
        for (const auto& lidar_path_node : comp_yaml["lidar_configs"]) {
            std::filesystem::path lidar_path = base_dir / lidar_path_node.as<std::string>();
            common::types::LidarConfigData lidar = parseLidarConfig(lidar_path);
            composition.lidar_configs.push_back(lidar);
        }
    }

    return composition;
}

void YamlParserUtils::writeComparativeReport(
    const std::string& composition_filename,
    const std::string& folder_name,
    const std::map<std::string, types::SimulationManagerReport>& mission_reports,
    const std::filesystem::path& output_path,
    const std::vector<std::string>& failed_plugins
) {
    YAML::Emitter out;
    out << YAML::BeginMap;
    out << YAML::Key << "comparative_report";
    out << YAML::BeginMap;
    out << YAML::Key << "composition_file" << YAML::Value << composition_filename;
    out << YAML::Key << "mission_control_folder" << YAML::Value << folder_name;

    // Get current time for generated_at_utc
    out << YAML::Key << "generated_at_utc" << YAML::Value << TimeUtils::generate_iso_timestamp();

    struct ResultBucket {
        double total_score = 0;
        int total_steps = 0;
        std::vector<std::string> managers;
    };
    std::vector<ResultBucket> buckets;
    std::vector<std::string> all_failed_plugins = failed_plugins;

    for (const auto& [mc_name, report] : mission_reports) {
        if (report.runs.empty()) {
            all_failed_plugins.push_back(mc_name + ".so");
            continue;
        }

        double total_score = 0.0;
        int total_steps = 0;
        bool all_failed = true;
        for (const auto& run : report.runs) {
            if (run.mission_score != -1.0) {
                total_score += run.mission_score;
                if (!run.mission_results.empty()) {
                    total_steps += run.mission_results.front().steps;
                }
                all_failed = false;
            }
        }

        if (all_failed) {
            all_failed_plugins.push_back(mc_name + ".so");
            continue;
        }

        bool found = false;
        for (auto& bucket : buckets) {
            if (std::abs(bucket.total_score - total_score) < 1e-5 && bucket.total_steps == total_steps) {
                bucket.managers.push_back(mc_name + ".so");
                found = true;
                break;
            }
        }
        if (!found) {
            buckets.push_back({total_score, total_steps, {mc_name + ".so"}});
        }
    }

    // sort buckets by number of agreeing managers descending
    std::sort(buckets.begin(), buckets.end(), [](const ResultBucket& a, const ResultBucket& b) {
        return a.managers.size() > b.managers.size();
    });

    out << YAML::Key << "results_summary";
    out << YAML::BeginSeq;
    for (const auto& bucket : buckets) {
        out << YAML::BeginMap;
        out << YAML::Key << "same_results" << YAML::Flow << YAML::BeginSeq;
        for (const auto& mgr : bucket.managers) {
            out << mgr;
        }
        out << YAML::EndSeq;
        out << YAML::Key << "total_score" << YAML::Value << bucket.total_score;
        out << YAML::Key << "total_steps" << YAML::Value << bucket.total_steps;
        out << YAML::EndMap;
    }
    out << YAML::EndSeq;

    out << YAML::Key << "errors";
    YAML::Node errors_node(YAML::NodeType::Sequence);
    for (const auto& plugin : all_failed_plugins) {
        errors_node.push_back(plugin);
    }
    out << YAML::Flow << errors_node;

    out << YAML::EndMap;
    out << YAML::EndMap;

    std::filesystem::path output_file = output_path / "comparative_report.yaml";
    std::ofstream file(output_file);
    if (file) {
        file << out.c_str() << "\n";
    }
}

void YamlParserUtils::writeCompetitiveReport(
    const std::string& composition_filename,
    const std::string& mission_control_name,
    const std::map<std::string, types::SimulationManagerReport>& algo_reports,
    const std::filesystem::path& output_path,
    const std::vector<std::string>& failed_plugins
) {
    YAML::Emitter out;
    out << YAML::BeginMap;
    out << YAML::Key << "competitive_report";
    out << YAML::BeginMap;
    out << YAML::Key << "composition_file" << YAML::Value << composition_filename;
    out << YAML::Key << "mission_control" << YAML::Value << mission_control_name;
    
    // Get current time for generated_at_utc
    out << YAML::Key << "generated_at_utc" << YAML::Value << TimeUtils::generate_iso_timestamp();

    struct AlgoResult {
        std::string algo_so;
        double total_score = 0;
        int total_steps = 0;
    };
    std::vector<AlgoResult> algo_results;
    std::vector<std::string> all_failed_plugins = failed_plugins;

    for (const auto& [algo_name, report] : algo_reports) {
        if (report.runs.empty()) {
            all_failed_plugins.push_back(algo_name + ".so");
            continue;
        }

        double total_score = 0.0;
        int total_steps = 0;
        bool all_failed = true;
        for (const auto& run : report.runs) {
            if (run.mission_score != -1.0) {
                total_score += run.mission_score;
                if (!run.mission_results.empty()) {
                    total_steps += run.mission_results.front().steps;
                }
                all_failed = false;
            }
        }
        
        if (all_failed) {
            all_failed_plugins.push_back(algo_name + ".so");
        } else {
            algo_results.push_back({algo_name + ".so", total_score, total_steps});
        }
    }

    std::sort(algo_results.begin(), algo_results.end(), [](const AlgoResult& a, const AlgoResult& b) {
        if (a.total_score != b.total_score) {
            return a.total_score > b.total_score;
        }
        return a.total_steps < b.total_steps;
    });

    out << YAML::Key << "results_summary";
    out << YAML::BeginSeq;
    for (const auto& result : algo_results) {
        out << YAML::BeginMap;
        out << YAML::Key << "algorithm" << YAML::Value << result.algo_so;
        out << YAML::Key << "total_score" << YAML::Value << result.total_score;
        out << YAML::Key << "total_steps" << YAML::Value << result.total_steps;
        out << YAML::EndMap;
    }
    out << YAML::EndSeq;

    out << YAML::Key << "errors";
    YAML::Node errors_node(YAML::NodeType::Sequence);
    for (const auto& plugin : all_failed_plugins) {
        errors_node.push_back(plugin);
    }
    out << YAML::Flow << errors_node;

    out << YAML::EndMap;
    out << YAML::EndMap;

    std::filesystem::path output_file = output_path / "competitive_report.yaml";
    std::ofstream file(output_file);
    if (file) {
        file << out.c_str() << "\n";
    }
}

void YamlParserUtils::writeSimulationOutput(
    const types::SimulationManagerReport& report,
    const YAML::Node& comp_yaml,
    const std::string& composition_filename,
    const std::filesystem::path& output_path,
    const std::string& extra_suffix
) {
    YAML::Emitter out;
    out << YAML::BeginMap; // root
    out << YAML::Key << "score_report";
    out << YAML::BeginMap;
    out << YAML::Key << "composition_file" << YAML::Value << composition_filename;

    out << YAML::Key << "simulations";
    out << YAML::BeginSeq;

    std::size_t run_idx = 0;
    if (comp_yaml["simulation_compositions"] && comp_yaml["simulation_compositions"]["simulations"]) {
        for (const auto& sim_node : comp_yaml["simulation_compositions"]["simulations"]) {
            out << YAML::BeginMap;
            out << YAML::Key << "simulation";
            out << YAML::BeginMap;
            out << YAML::Key << "simulation_config" << YAML::Value << sim_node["simulation_config"].as<std::string>();

            out << YAML::Key << "missions";
            out << YAML::BeginSeq;

            if (sim_node["mission_configs"]) {
                for (const auto& mission_node : sim_node["mission_configs"]) {
                    out << YAML::BeginMap;
                    out << YAML::Key << "mission";
                    out << YAML::BeginMap;
                    out << YAML::Key << "mission_config" << YAML::Value << mission_node.as<std::string>();
                    
                    out << YAML::Key << "runs";
                    out << YAML::BeginSeq;

                    auto drone_configs = comp_yaml["simulation_compositions"]["drone_configs"];
                    auto lidar_configs = comp_yaml["simulation_compositions"]["lidar_configs"];

                    if (drone_configs && lidar_configs) {
                        for (const auto& drone_node : drone_configs) {
                            for (const auto& lidar_node : lidar_configs) {
                                if (run_idx >= report.runs.size()) break;
                                const auto& run = report.runs[run_idx++];
                                
                                out << YAML::BeginMap;
                                out << YAML::Key << "drone_config" << YAML::Value << drone_node.as<std::string>();
                                out << YAML::Key << "lidar_config" << YAML::Value << lidar_node.as<std::string>();

                                if (!run.mission_results.empty()) {
                                    const auto& mr = run.mission_results.front();
                                    std::string status;
                                    switch (mr.status) {
                                        case common::types::MissionRunStatus::Completed: status = "completed"; break;
                                        case common::types::MissionRunStatus::MaxSteps: status = "max_steps"; break;
                                        case common::types::MissionRunStatus::Error: status = "error"; break;
                                    }
                                    out << YAML::Key << "status" << YAML::Value << status;
                                    out << YAML::Key << "steps" << YAML::Value << static_cast<int>(mr.steps);
                                    
                                    out << YAML::Key << "score" << YAML::Value << run.mission_score;

                                    if (!mr.errors.empty()) {
                                        out << YAML::Key << "error_ref" << YAML::Value << YAML::BeginMap;
                                        out << YAML::Key << "code" << YAML::Value << mr.errors[0].code;
                                        out << YAML::EndMap;
                                    }
                                } else {
                                    out << YAML::Key << "status" << YAML::Value << "error";
                                    out << YAML::Key << "steps" << YAML::Value << 0;
                                    out << YAML::Key << "score" << YAML::Value << run.mission_score;
                                }

                                out << YAML::EndMap; // run
                            }
                        }
                    }
                    out << YAML::EndSeq; // runs
                    out << YAML::EndMap; // mission (inner)
                    out << YAML::EndMap; // mission (outer)
                }
            }
            out << YAML::EndSeq; // missions
            out << YAML::EndMap; // simulation (inner)
            out << YAML::EndMap; // simulation (outer)
        }
    }
    out << YAML::EndSeq; // simulations
    out << YAML::EndMap; // score_report
    out << YAML::EndMap; // root

    std::string filename = "simulation_output" + (extra_suffix.empty() ? "" : "_" + extra_suffix) + ".yaml";
    std::filesystem::path output_file = output_path / filename;
    std::ofstream file(output_file);
    if (file) {
        file << out.c_str() << "\n";
    }
}

} // namespace simulator
