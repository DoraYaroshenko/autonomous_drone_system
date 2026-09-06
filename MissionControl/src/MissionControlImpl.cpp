#include "MissionControl/MissionControlImpl.h"
#include <UserCommon/CollisionUtils.h>
#include <UserCommon/Logger.h>
#include <Common/MissionControlRegistration.h>
#include <UserCommon/ErrorCodes.h>

#include <fstream>
#include <chrono>
#include <utility>

#include "MissionControl/DroneControlImpl.h"

namespace mission_control_330371063_324976703 {
using namespace common;

using namespace user_common_330371063_324976703;

MissionControlImpl_330371063_324976703::MissionControlImpl_330371063_324976703(common::MissionControlDependencies deps)
    : mission_(deps.mission_config),
      drone_(deps.drone_config),
      output_map_(deps.output_map),
      output_map_file_(deps.output_map_file),
      verbose_(deps.verbose) {
      drone_control_ = std::make_unique<DroneControlImpl>(
          deps.drone_config, deps.lidar, deps.gps, deps.movement,
          deps.output_map, deps.mapping_algorithm);
}

void MissionControlImpl_330371063_324976703::recordError(const std::string& errorCode, const std::string& message, common::types::MissionRunResult& result, const std::string& logMessage) const {
    std::filesystem::path error_log_path = output_map_file_;
    error_log_path.replace_extension(".error");
    std::ofstream error_log(error_log_path, std::ios::app);
    if (error_log) {
        error_log << logMessage << "\n";
    }

    result.status = common::types::MissionRunStatus::Error;
    result.errors.push_back(common::types::ErrorRef{errorCode, message});
}

void MissionControlImpl_330371063_324976703::executeMissionLoop(common::types::MissionRunResult& result) {
    while (drone_control_->state().step_index < mission_.max_steps) {
        std::size_t step = drone_control_->state().step_index;
        try {
            common::types::DroneStepResult step_result = drone_control_->step();

            if (step_result.status == common::types::DroneStepStatus::Completed) {
                result.status = common::types::MissionRunStatus::Completed;
                result.steps = step + 1;
                break;
            }

            if (step_result.status == common::types::DroneStepStatus::Error) {
                result.steps = step + 1;
                recordError(std::string(user_common_330371063_324976703::ErrorCodes::DRONE_STEP_ERROR), 
                            step_result.message, 
                            result, 
                            "Step " + std::to_string(step) + ": ERROR - " + step_result.message);
                break;
            }

            // DroneStepStatus::Continue — keep going.
            result.steps = step + 1;

            // Verify legality of the drone's position after movement
            common::types::DroneState current_state = drone_control_->state();
            
            // 1. Check if the drone went out of mission boundaries
            if (!CollisionUtils::isDroneFullyInBounds(output_map_, current_state.position, drone_.radius)) {
                recordError(std::string(user_common_330371063_324976703::ErrorCodes::OUT_OF_BOUNDS), 
                            "Drone's physical body is outside the configured mission boundaries.", 
                            result, 
                            "Step " + std::to_string(step) + ": ERROR - Drone flew completely out of the mission boundaries!");
                break;
            }

        } catch (const std::exception& e) {
            result.steps = step + 1;
            recordError(std::string(user_common_330371063_324976703::ErrorCodes::DRONE_EXCEPTION), 
                        e.what(), 
                        result, 
                        "Step " + std::to_string(step) + ": EXCEPTION - " + std::string(e.what()));
            break;
        }
    }
}

void MissionControlImpl_330371063_324976703::finalizeMissionResult(common::types::MissionRunResult& result) {
    // If we exhausted max_steps without completing or erroring.
    if (result.status == common::types::MissionRunStatus::Completed && result.steps >= mission_.max_steps) {
        result.status = common::types::MissionRunStatus::MaxSteps;
        result.steps = mission_.max_steps;
    }
}

void MissionControlImpl_330371063_324976703::saveMap(common::types::MissionRunResult& result) {
    // Save the output map.
    try {
        output_map_.save(output_map_file_);
    } catch (const std::exception& e) {
        recordError(std::string(user_common_330371063_324976703::ErrorCodes::MAP_SAVE_ERROR), 
                    e.what(), 
                    result, 
                    "SAVE ERROR - " + std::string(e.what()));
    }
}

common::types::MissionRunResult MissionControlImpl_330371063_324976703::runMission() {
    common::types::MissionRunResult result;
    result.status = common::types::MissionRunStatus::Completed;

    // Initialize the logger for this mission run if verbose
    if (verbose_) {
        Logger::init((output_map_file_.parent_path() / (output_map_file_.stem().string() + "_drone_logs.jsonl")).string());
    }

    executeMissionLoop(result);
    finalizeMissionResult(result);
    saveMap(result);

    if (verbose_) {
        Logger::close();
    }
    return result;
}

REGISTER_MISSION_CONTROL(MissionControlImpl_330371063_324976703);

} // namespace mission_control_330371063_324976703
