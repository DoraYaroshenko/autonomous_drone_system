#include <Simulator/SimulationRunImpl.h>

#include <Simulator/MapsComparison.h>
#include <UserCommon/ErrorCodes.h>
#include <iostream>

#include <stdexcept>
#include <utility>
#include <vector>
#include <fstream>




namespace simulator {
namespace types {
using namespace common::types;
using namespace simulator::types;
}
using namespace common;
namespace user_common_330371063_324976703 {}
using namespace user_common_330371063_324976703;

SimulationRunImpl::SimulationRunImpl(std::unique_ptr<const IMap3D> hidden_map,
                                     std::unique_ptr<IMutableMap3D> output_map,
                                     std::unique_ptr<IGPS> gps,
                                     std::unique_ptr<IDroneMovement> movement,
                                     std::unique_ptr<ILidar> lidar,
                                     std::unique_ptr<IMappingAlgorithm> mapping_algorithm,
                                     std::unique_ptr<IMissionControl> mission_control,
                                     types::SimulationConfigData simulation_config,
                                     types::MissionConfigData mission_config,
                                     std::filesystem::path output_map_file)
    : hidden_map_(std::move(hidden_map)),
      output_map_(std::move(output_map)),
      gps_(std::move(gps)),
      movement_(std::move(movement)),
      lidar_(std::move(lidar)),
      mapping_algorithm_(std::move(mapping_algorithm)),
      mission_control_(std::move(mission_control)),
      simulation_config_(std::move(simulation_config)),
      mission_config_(std::move(mission_config)),
      output_map_file_(std::move(output_map_file)) {
    if (!hidden_map_ ||
        !output_map_ ||
        !gps_ ||
        !movement_ ||
        !lidar_ ||
        !mapping_algorithm_ ||
        !mission_control_) {
        throw std::invalid_argument("SimulationRunImpl requires injected dependencies.");
    }
}

types::SimulationResult SimulationRunImpl::run() {
    types::SimulationResult result;
    result.simulation_config = simulation_config_;
    result.mission_config = mission_config_;
    result.output_map_file = output_map_file_;

    // Determine resolution request status.
    // For now, we always use the GPS resolution (factor=1 or default).
    double factor = mission_config_.output_mapping_resolution_factor;
    if (factor <= 0.0 || factor < 1.0) {
        result.resolution_request_status = types::ResolutionRequestStatus::IgnoredTooSmall;
    } else if (factor == 1.0) {
        result.resolution_request_status = types::ResolutionRequestStatus::Accepted;
    } else {
        // For now, we don't support non-1 resolution factors.
        result.resolution_request_status = types::ResolutionRequestStatus::Ignored;
    }

    // Run the mission.
    try {
        types::MissionRunResult mission_result = mission_control_->runMission();
        result.mission_results.push_back(std::move(mission_result));
    } catch (const std::exception& e) {
        std::filesystem::path err_path = output_map_file_;
        err_path.replace_extension(".error");
        std::ofstream err_file(err_path, std::ios::app);
        if (err_file) {
            err_file << "Mission Exception: " << e.what() << "\n";
        }
        
        types::MissionRunResult error_result;
        error_result.status = types::MissionRunStatus::Error;
        error_result.errors.push_back(types::ErrorRef{std::string(::user_common_330371063_324976703::ErrorCodes::MISSION_EXCEPTION), e.what()});
        result.mission_results.push_back(std::move(error_result));
    }

    // Compute score by comparing hidden map with output map.
    try {
        std::vector<IMap3D*> targets = {output_map_.get()};
        std::vector<double> scores = MapsComparison::compare(*hidden_map_, targets);
        result.mission_score = scores.empty() ? -1.0 : scores[0];
    } catch (const std::exception& e) {
        std::filesystem::path err_path = output_map_file_;
        err_path.replace_extension(".error");
        std::ofstream err_file(err_path, std::ios::app);
        if (err_file) {
            err_file << "MapsComparison Exception: " << e.what() << "\n";
        }
        std::cerr << "MapsComparison Exception: " << e.what() << std::endl;
        result.mission_score = -1.0;
    }

    // Explicitly enforce -1.0 score if the mission run ended in an error
    if (!result.mission_results.empty() && 
        result.mission_results.back().status == types::MissionRunStatus::Error) {
        result.mission_score = -1.0;
    }

    // Store output map config.
    result.output_map_config = output_map_->getMapConfig();

    try {
        output_map_->save(output_map_file_);
    } catch (const std::exception& e) {
        std::filesystem::path err_path = output_map_file_;
        err_path.replace_extension(".error");
        std::ofstream err_file(err_path, std::ios::app);
        if (err_file) {
            err_file << "Failed to save output map: " << e.what() << "\n";
        }
    }

    return result;
}




} // namespace simulator
