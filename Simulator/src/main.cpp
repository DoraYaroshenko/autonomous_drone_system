#include "PluginLoader.h"
#include "MappingAlgorithmRegistrar.h"
#include "MissionControlRegistrar.h"
#include <Simulator/SimulationRunFactoryImpl.h>
#include <Simulator/YamlParserUtils.h>
#include <UserCommon/TimeUtils.h>

#include <UserCommon/ErrorCodes.h>
#include <iostream>
#include <string>
#include <vector>
#include <filesystem>
#include <fstream>
#include <map>
#include <chrono>
#include <ctime>
#include <iomanip>
#include <sstream>
#include <cstdlib>
#include <algorithm>
#include <thread>
#include <atomic>
#include <memory>
#include <mutex>

void print_usage_and_exit(const std::string& error_msg) { //explains the user how to write command correctly
    std::cerr << "Error: " << error_msg << "\n";
    std::cerr << "Usage:\n";
    std::cerr << "  Comparative run: simulator_<submitter_ids> -comparative simulation=<sim.yaml> mission_control_folder=<folder> algorithm=<algo.so> [num_threads=<num>] [-verbose]\n";
    std::cerr << "  Competition run: simulator_<submitter_ids> -competition simulation=<sim.yaml> mission_control=<mc.so> algorithms_folder=<folder> [num_threads=<num>] [-verbose]\n";
    std::exit(1);
}

// TimeUtils used for timestamps

struct PluginRun {
    std::string plugin_name;
    std::unique_ptr<simulator::SimulationRunFactoryImpl> factory;
    std::vector<simulator::types::SimulationResult> results;
    std::filesystem::path run_out_dir;
};

struct GlobalRunTask {
    PluginRun* plugin_run;
    size_t result_index;
    const simulator::types::SimulationConfigData* simulation;
    const simulator::types::MissionConfigData* mission;
    const simulator::types::DroneConfigData* drone;
    const simulator::types::LidarConfigData* lidar;
    std::filesystem::path run_output;
};

struct ParsedArgs {
    bool is_comparative = false;
    bool is_competition = false;
    bool is_verbose = false;
    int num_threads = 1;
    std::filesystem::path sim_path;
    std::vector<std::string> mc_plugins_to_load;
    std::vector<std::string> algo_plugins_to_load;
    std::filesystem::path output_base_dir;
    std::map<std::string, std::string> raw_args;
};

ParsedArgs parse_arguments(int argc, char** argv) {
    ParsedArgs parsed;
    std::vector<std::string> unsupported_args;

    for (int i = 1; i < argc; ++i) {
        std::string arg = argv[i];
        if (arg == "-comparative") {
            if (parsed.is_comparative) unsupported_args.push_back(arg);
            parsed.is_comparative = true;
        } else if (arg == "-competition") {
            if (parsed.is_competition) unsupported_args.push_back(arg);
            parsed.is_competition = true;
        } else if (arg == "-verbose") {
            if (parsed.is_verbose) unsupported_args.push_back(arg);
            parsed.is_verbose = true;
        } else {
            auto pos = arg.find('=');
            if (pos != std::string::npos && pos > 0 && pos < arg.size() - 1) {
                std::string key = arg.substr(0, pos);
                std::string value = arg.substr(pos + 1);
                
                if (key == "simulation" || key == "mission_control_folder" || 
                    key == "algorithm" || key == "mission_control" || 
                    key == "algorithms_folder" || key == "num_threads") {
                    if (parsed.raw_args.find(key) != parsed.raw_args.end()) {
                        unsupported_args.push_back("Duplicate arg: " + key);
                    } else {
                        parsed.raw_args[key] = value;
                    }
                } else {
                    unsupported_args.push_back(arg);
                }
            } else {
                unsupported_args.push_back(arg);
            }
        }
    }

    if (!unsupported_args.empty()) {
        std::string msg = "Unsupported arguments provided: ";
        for (const auto& a : unsupported_args) msg += a + " ";
        print_usage_and_exit(msg);
    }

    if (parsed.is_comparative && parsed.is_competition) {
        print_usage_and_exit("Cannot specify both -comparative and -competition modes.");
    }
    
    if (!parsed.is_comparative && !parsed.is_competition) {
        print_usage_and_exit("Missing mode argument (-comparative or -competition).");
    }

    if (parsed.raw_args.find("simulation") == parsed.raw_args.end()) {
        print_usage_and_exit("Missing mandatory argument: simulation=<...>");
    }

    parsed.sim_path = parsed.raw_args["simulation"];
    if (!std::filesystem::exists(parsed.sim_path) || !std::filesystem::is_regular_file(parsed.sim_path)) {
        print_usage_and_exit("Simulation file does not exist or is not a regular file: " + parsed.sim_path.string());
    }

    if (parsed.raw_args.find("num_threads") != parsed.raw_args.end()) {
        try {
            size_t pos = 0;
            int num = std::stoi(parsed.raw_args["num_threads"], &pos);
            if (pos != parsed.raw_args["num_threads"].size() || num < 1) {
                print_usage_and_exit("Invalid value for num_threads: " + parsed.raw_args["num_threads"]);
            }
            parsed.num_threads = num;
        } catch (...) {
            print_usage_and_exit("Invalid value for num_threads: " + parsed.raw_args["num_threads"]);
        }
    }

    if (parsed.is_comparative) {
        if (parsed.raw_args.count("mission_control") || parsed.raw_args.count("algorithms_folder")) {
            print_usage_and_exit("Unsupported arguments for comparative mode.");
        }
        if (parsed.raw_args.find("mission_control_folder") == parsed.raw_args.end()) print_usage_and_exit("Missing mandatory argument for comparative mode: mission_control_folder=<...>");
        if (parsed.raw_args.find("algorithm") == parsed.raw_args.end()) print_usage_and_exit("Missing mandatory argument for comparative mode: algorithm=<...>");

        std::filesystem::path mc_folder = parsed.raw_args["mission_control_folder"];
        std::filesystem::path algo_file = parsed.raw_args["algorithm"];

        if (!std::filesystem::exists(mc_folder) || !std::filesystem::is_directory(mc_folder)) {
            print_usage_and_exit("mission_control_folder does not exist or is not a directory: " + mc_folder.string());
        }
        if (!std::filesystem::exists(algo_file) || !std::filesystem::is_regular_file(algo_file)) {
            print_usage_and_exit("algorithm file does not exist or is not a regular file: " + algo_file.string());
        }

        parsed.algo_plugins_to_load.push_back(std::filesystem::canonical(algo_file).string());
        for (const auto& entry : std::filesystem::directory_iterator(mc_folder)) {
            if (entry.is_regular_file() && entry.path().extension() == ".so") {
                parsed.mc_plugins_to_load.push_back(std::filesystem::canonical(entry.path()).string());
            }
        }
        if (parsed.mc_plugins_to_load.empty()) {
            print_usage_and_exit("No .so files found in mission_control_folder: " + mc_folder.string());
        }
        
        parsed.output_base_dir = mc_folder / ("comparative_results_" + user_common_330371063_324976703::TimeUtils::generate_folder_timestamp());
    } else {
        if (parsed.raw_args.count("mission_control_folder") || parsed.raw_args.count("algorithm")) {
            print_usage_and_exit("Unsupported arguments for competition mode.");
        }
        if (parsed.raw_args.find("algorithms_folder") == parsed.raw_args.end()) print_usage_and_exit("Missing mandatory argument for competition mode: algorithms_folder=<...>");
        if (parsed.raw_args.find("mission_control") == parsed.raw_args.end()) print_usage_and_exit("Missing mandatory argument for competition mode: mission_control=<...>");

        std::filesystem::path algo_folder = parsed.raw_args["algorithms_folder"];
        std::filesystem::path mc_file = parsed.raw_args["mission_control"];

        if (!std::filesystem::exists(algo_folder) || !std::filesystem::is_directory(algo_folder)) {
            print_usage_and_exit("algorithms_folder does not exist or is not a directory: " + algo_folder.string());
        }
        if (!std::filesystem::exists(mc_file) || !std::filesystem::is_regular_file(mc_file)) {
            print_usage_and_exit("mission_control file does not exist or is not a regular file: " + mc_file.string());
        }

        parsed.mc_plugins_to_load.push_back(std::filesystem::canonical(mc_file).string());
        for (const auto& entry : std::filesystem::directory_iterator(algo_folder)) {
            if (entry.is_regular_file() && entry.path().extension() == ".so") {
                parsed.algo_plugins_to_load.push_back(std::filesystem::canonical(entry.path()).string());
            }
        }
        if (parsed.algo_plugins_to_load.empty()) {
            print_usage_and_exit("No .so files found in algorithms_folder: " + algo_folder.string());
        }

        parsed.output_base_dir = algo_folder / ("competition_" + user_common_330371063_324976703::TimeUtils::generate_folder_timestamp());
    }

    return parsed;
}

void load_comparative_plugins(simulator::PluginLoader& loader, 
                              const ParsedArgs& parsed_args, 
                              std::vector<PluginRun>& plugin_runs, 
                              std::vector<std::string>& failed_plugins) {
    try {
        loader.loadLibrary(parsed_args.algo_plugins_to_load[0]);
    } catch (const std::exception& e) {
        print_usage_and_exit(std::string("Failed to load algorithm library: ") + e.what());
    }
    auto algoFactories = simulator::MappingAlgorithmRegistrar::getInstance().getFactories();
    std::string algo_name = std::filesystem::path(parsed_args.algo_plugins_to_load[0]).filename().stem().string();
    if (algoFactories.empty()) {
        print_usage_and_exit("Algorithm failed to register.");
    }
    auto algoFactory = algoFactories[0];
    simulator::MappingAlgorithmRegistrar::getInstance().clear();
    
    try {
        std::filesystem::create_directories(parsed_args.output_base_dir);
        std::ofstream(parsed_args.output_base_dir / "error_log.txt");
    } catch (const std::exception& e) {
        print_usage_and_exit("Failed to create output directory: " + parsed_args.output_base_dir.string() + " - " + e.what());
    }
    
    std::atomic<size_t> next_plugin{0};
    std::mutex out_mtx;

    auto worker = [&]() {
        std::vector<PluginRun> local_runs;
        std::vector<std::string> local_failed;
        while (true) {
            size_t idx = next_plugin.fetch_add(1, std::memory_order_relaxed);
            if (idx >= parsed_args.mc_plugins_to_load.size()) break;
            
            const auto& mc_path = parsed_args.mc_plugins_to_load[idx];
            std::string mc_name = std::filesystem::path(mc_path).filename().stem().string();
            std::string mc_filename = std::filesystem::path(mc_path).filename().string();
            
            try {
                loader.loadLibrary(mc_path);
            } catch (const std::exception& e) {
                std::cerr << "Warning: Mission Control " << mc_path << " failed to load: " << e.what() << "\n";
                {
                    std::lock_guard<std::mutex> lock(out_mtx);
                    std::ofstream err_file(parsed_args.output_base_dir / "error_log.txt", std::ios::app);
                    if (err_file) err_file << "Failed to load mission control: " << mc_filename << std::endl;
                }
                local_failed.push_back(mc_filename);
                continue;
            }
            
            auto mcFactories = simulator::MissionControlRegistrar::getInstance().getFactories();
            if (mcFactories.empty()) {
                std::cerr << "Warning: Mission Control " << mc_path << " failed to register.\n";
                {
                    std::lock_guard<std::mutex> lock(out_mtx);
                    std::ofstream err_file(parsed_args.output_base_dir / "error_log.txt", std::ios::app);
                    if (err_file) err_file << "Failed to register mission control: " << mc_filename << std::endl;
                }
                local_failed.push_back(mc_filename);
                simulator::MissionControlRegistrar::getInstance().clear();
                continue;
            }
            auto mcFactory = mcFactories[0];
            simulator::MissionControlRegistrar::getInstance().clear(); //in registrar the key is thread_id and the value is the factories of the libraries it loaded
            
            local_runs.push_back({
                mc_name,
                std::make_unique<simulator::SimulationRunFactoryImpl>(algoFactory, mcFactory, parsed_args.is_verbose),
                {},
                parsed_args.output_base_dir
            });
        }
        std::lock_guard<std::mutex> lock(out_mtx); //unlocks automatically when worker ends
        for (auto& r : local_runs) plugin_runs.push_back(std::move(r));
        for (auto& f : local_failed) failed_plugins.push_back(std::move(f));
    };

    if (parsed_args.num_threads <= 1) {
        worker();
    } else {
        int num_extra = std::min<int>(parsed_args.num_threads, static_cast<int>(parsed_args.mc_plugins_to_load.size()));
        std::vector<std::jthread> threads; //join happens when threads vector goes out of scope
        for (int i = 0; i < num_extra; ++i) {
            threads.emplace_back(worker); //construct and append in place
        }
    }
}

void load_competition_plugins(simulator::PluginLoader& loader, 
                              const ParsedArgs& parsed_args, 
                              std::vector<PluginRun>& plugin_runs, 
                              std::vector<std::string>& failed_plugins) {
    try {
        loader.loadLibrary(parsed_args.mc_plugins_to_load[0]);
    } catch (const std::exception& e) {
        print_usage_and_exit(std::string("Failed to load mission control library: ") + e.what());
    }
    auto mcFactories = simulator::MissionControlRegistrar::getInstance().getFactories();
    std::string mc_name = std::filesystem::path(parsed_args.mc_plugins_to_load[0]).filename().stem().string();
    if (mcFactories.empty()) {
        print_usage_and_exit("Mission Control failed to register.");
    }
    auto mcFactory = mcFactories[0];
    simulator::MissionControlRegistrar::getInstance().clear();
    
    try {
        std::filesystem::create_directories(parsed_args.output_base_dir);
        std::ofstream(parsed_args.output_base_dir / "error_log.txt");
    } catch (const std::exception& e) {
        print_usage_and_exit("Failed to create output directory: " + parsed_args.output_base_dir.string() + " - " + e.what());
    }
    
    std::atomic<size_t> next_plugin{0};
    std::mutex out_mtx;

    auto worker = [&]() {
        std::vector<PluginRun> local_runs;
        std::vector<std::string> local_failed;
        while (true) {
            size_t idx = next_plugin.fetch_add(1, std::memory_order_relaxed);
            if (idx >= parsed_args.algo_plugins_to_load.size()) break; //the loop ends when there are no more plugins to load
            
            const auto& algo_path = parsed_args.algo_plugins_to_load[idx];
            std::string algo_name = std::filesystem::path(algo_path).filename().stem().string();
            std::string algo_filename = std::filesystem::path(algo_path).filename().string();
            
            try {
                loader.loadLibrary(algo_path);
            } catch (const std::exception& e) {
                std::cerr << "Warning: Algorithm " << algo_path << " failed to load: " << e.what() << "\n";
                {
                    std::lock_guard<std::mutex> lock(out_mtx);
                    std::ofstream err_file(parsed_args.output_base_dir / "error_log.txt", std::ios::app);
                    if (err_file) err_file << "Failed to load algorithm: " << algo_filename << std::endl;
                }
                local_failed.push_back(algo_filename);
                continue;
            }
            
            auto algoFactories = simulator::MappingAlgorithmRegistrar::getInstance().getFactories();
            if (algoFactories.empty()) {
                std::cerr << "Warning: Algorithm " << algo_path << " failed to register.\n";
                {
                    std::lock_guard<std::mutex> lock(out_mtx);
                    std::ofstream err_file(parsed_args.output_base_dir / "error_log.txt", std::ios::app);
                    if (err_file) err_file << "Failed to register algorithm: " << algo_filename << std::endl;
                }
                local_failed.push_back(algo_filename);
                simulator::MappingAlgorithmRegistrar::getInstance().clear();
                continue;
            }
            auto algoFactory = algoFactories[0];
            simulator::MappingAlgorithmRegistrar::getInstance().clear();
            
            local_runs.push_back({
                algo_name,
                std::make_unique<simulator::SimulationRunFactoryImpl>(algoFactory, mcFactory, parsed_args.is_verbose),
                {},
                parsed_args.output_base_dir
            });
        }
        std::lock_guard<std::mutex> lock(out_mtx);
        for (auto& r : local_runs) plugin_runs.push_back(std::move(r));
        for (auto& f : local_failed) failed_plugins.push_back(std::move(f));
    };

    if (parsed_args.num_threads <= 1) {
        worker();
    } else {
        int num_extra = std::min<int>(parsed_args.num_threads, static_cast<int>(parsed_args.algo_plugins_to_load.size()));
        std::vector<std::jthread> threads;
        for (int i = 0; i < num_extra; ++i) {
            threads.emplace_back(worker);
        }
    }
}

std::vector<GlobalRunTask> generate_global_tasks(std::vector<PluginRun>& plugin_runs, 
                                                 const simulator::types::SimulationCompositionData& composition,
                                                 const YAML::Node& config) {
    std::vector<GlobalRunTask> global_tasks;
    YAML::Node comp_yaml = config["simulation_compositions"];
    
    for (auto& pr : plugin_runs) {
        size_t run_index = 0;
        
        size_t sim_idx = 0;
        for (const auto& [simulation, missions] : composition.simulation_mission_groups) {
            auto sim_node = comp_yaml["simulations"][sim_idx++];
            std::string sim_name = std::filesystem::path(sim_node["simulation_config"].as<std::string>()).stem().string();
            
            size_t mission_idx = 0;
            for (const auto& mission : missions) {
                auto mission_node = sim_node["mission_configs"][mission_idx++];
                std::string mission_name = std::filesystem::path(mission_node.as<std::string>()).stem().string();
                
                size_t drone_idx = 0;
                for (const auto& drone : composition.drone_configs) {
                    auto drone_node = comp_yaml["drone_configs"][drone_idx++];
                    std::string drone_name = std::filesystem::path(drone_node.as<std::string>()).stem().string();
                    
                    size_t lidar_idx = 0;
                    for (const auto& lidar : composition.lidar_configs) {
                        auto lidar_node = comp_yaml["lidar_configs"][lidar_idx++];
                        std::string lidar_name = std::filesystem::path(lidar_node.as<std::string>()).stem().string();
                        
                        std::string unique_name = pr.plugin_name + "_" + sim_name + "_" + mission_name + "_" + drone_name + "_" + lidar_name + ".npy";
                        std::filesystem::path run_output = pr.run_out_dir / unique_name;
                        
                        global_tasks.push_back({&pr, run_index, &simulation, &mission, &drone, &lidar, run_output});
                        run_index++;
                    }
                }
            }
        }
        pr.results.resize(run_index);
    }
    return global_tasks;
}

void execute_tasks(std::vector<GlobalRunTask>& global_tasks, int num_threads) {
    std::atomic<size_t> next_task{0};
    auto worker = [&]() {
        while (true) {
            size_t idx = next_task.fetch_add(1, std::memory_order_relaxed);
            if (idx >= global_tasks.size()) break;

            const auto& task = global_tasks[idx];
            try {
                std::unique_ptr<simulator::ISimulationRun> sim_run = 
                    task.plugin_run->factory->create(*(task.simulation), *(task.mission), *(task.drone), *(task.lidar), task.run_output);
                task.plugin_run->results[task.result_index] = sim_run->run();
            } catch (const std::exception& e) {
                std::filesystem::path err_path = task.run_output;
                err_path.replace_extension(".error");
                std::ofstream err_file(err_path);
                if (err_file) {
                    err_file << "Simulation Run Error: " << e.what() << "\n";
                }
                simulator::types::SimulationResult error_result;
                error_result.simulation_config = *(task.simulation);
                error_result.mission_config = *(task.mission);
                error_result.mission_score = -1.0;
                error_result.mission_results.push_back(simulator::types::MissionRunResult{
                    simulator::types::MissionRunStatus::Error,
                    0,
                    {simulator::types::ErrorRef{std::string(::user_common_330371063_324976703::ErrorCodes::SIMULATION_RUN_ERROR), e.what()}}
                });
                task.plugin_run->results[task.result_index] = std::move(error_result);
            }
        }
    };

    if (num_threads <= 1) {
        worker();
    } else {
        int num_extra_threads = std::min<int>(num_threads, static_cast<int>(global_tasks.size()));
        std::vector<std::jthread> threads;
        for (int i = 0; i < num_extra_threads; ++i) {
            threads.emplace_back(worker);
        }
        // Main thread waits automatically as jthreads join upon destruction
    }
}

void generate_reports(std::vector<PluginRun>& plugin_runs, 
                      const simulator::types::SimulationCompositionData& composition,
                      const YAML::Node& config,
                      const ParsedArgs& parsed_args,
                      const std::vector<std::string>& failed_plugins) {
    std::string run_time = user_common_330371063_324976703::TimeUtils::generate_iso_timestamp();
    std::map<std::string, simulator::types::SimulationManagerReport> reports;

    for (auto& pr : plugin_runs) {
        simulator::types::SimulationManagerReport report{
            composition.composition_file,
            run_time,
            "output_map_accuracy",
            {0.0, 100.0},
            -1,
            std::move(pr.results)
        };
        simulator::YamlParserUtils::writeSimulationOutput(report, config, parsed_args.sim_path.filename().string(), pr.run_out_dir, pr.plugin_name);
        reports[pr.plugin_name] = std::move(report);
    }
    
    if (parsed_args.is_comparative) {
        std::string folder_name = std::filesystem::path(parsed_args.raw_args.at("mission_control_folder")).filename().string();
        simulator::YamlParserUtils::writeComparativeReport(parsed_args.sim_path.filename().string(), folder_name, reports, parsed_args.output_base_dir, failed_plugins);
    } else {
        std::string mc_filename = std::filesystem::path(parsed_args.raw_args.at("mission_control")).filename().string();
        simulator::YamlParserUtils::writeCompetitiveReport(parsed_args.sim_path.filename().string(), mc_filename, reports, parsed_args.output_base_dir, failed_plugins);
    }
}

int main(int argc, char** argv) {
    ParsedArgs parsed_args = parse_arguments(argc, argv);

    simulator::types::SimulationCompositionData composition;
    try {
        composition = simulator::YamlParserUtils::parseCompositions(parsed_args.sim_path);
    } catch (const std::exception& e) {
        print_usage_and_exit("Failed to parse simulation configuration: " + std::string(e.what()));
    }

    YAML::Node config = YAML::LoadFile(parsed_args.sim_path.string());

    std::cout << "Starting Simulation...\n";
    std::cout << "Command line parsing successful.\n";
    std::cout << "Mode: " << (parsed_args.is_comparative ? "Comparative" : "Competitive") << "\n";
    std::cout << "Simulation file: " << parsed_args.sim_path << "\n";
    std::cout << "Num threads requested: " << parsed_args.num_threads << "\n";
    std::cout << "Algorithms to load: " << parsed_args.algo_plugins_to_load.size() << "\n";
    std::cout << "Mission controls to load: " << parsed_args.mc_plugins_to_load.size() << "\n";
    std::cout << "Output directory: " << parsed_args.output_base_dir << "\n";

    std::vector<std::string> failed_plugins;
    simulator::PluginLoader loader;
    std::vector<PluginRun> plugin_runs;

    if (parsed_args.is_comparative) {
        load_comparative_plugins(loader, parsed_args, plugin_runs, failed_plugins);
    } else {
        load_competition_plugins(loader, parsed_args, plugin_runs, failed_plugins);
    }

    std::vector<GlobalRunTask> global_tasks = generate_global_tasks(plugin_runs, composition, config);
    execute_tasks(global_tasks, parsed_args.num_threads);
    generate_reports(plugin_runs, composition, config, parsed_args, failed_plugins);
    
    return 0;
}
