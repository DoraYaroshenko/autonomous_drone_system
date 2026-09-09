#include "MissionControl/DroneControlImpl.h"
#include <UserCommon/ScanResultToVoxels.h>
#include <UserCommon/Logger.h>

#include <utility>
#include <string>

namespace mission_control_330371063_324976703 {
using namespace common;

using namespace user_common_330371063_324976703;

/*
 * Responsibility Division between Algorithm and Drone Control:
 * 
 * Algorithm (IMappingAlgorithm):
 * - High-level decision making and path planning.
 * - Determines the next desired movement or scan based on its internal state machine (e.g. Frontier Exploration).
 * - Can issue "macro" commands (e.g., "Advance 50cm") without needing to manage the drone's per-step physical limitations.
 * - Note: While an optimized algorithm might clamp its own steps for precise map updating, the interface contract allows it to request large movements.
 * 
 * Drone Control (DroneControlImpl):
 * - Execution engine and safety wrapper.
 * - Chunking: Breaks down large commands from the algorithm into smaller "chunks" that respect the drone's physical limits (max_advance, max_elevate, max_rotate) per simulation step.
 * - State Persistence: Uses `pending_command_` to persist the remaining chunks of a large movement across multiple simulation steps without querying the algorithm again.
 * - Execution Ordering: Ensures that if a command contains both a movement and a scan, the scan is ONLY executed on the final chunk, after the drone has reached the exact destination requested.
 * - Map Integration: Translates raw lidar scan results into voxels and updates the global output map.
 * - Error Handling & Logging: Catches movement exceptions (like wall collisions) to halt the simulation safely, and logs all movements and scans.
 */
DroneControlImpl::DroneControlImpl(common::types::DroneConfigData drone,
                                   ILidar& lidar,
                                   IGPS& gps,
                                   IDroneMovement& movement,
                                   IMutableMap3D& output_map,
                                   IMappingAlgorithm& mapping_algorithm)
    : drone_(std::move(drone)),
      lidar_(lidar),
      gps_(gps),
      movement_(movement),
      output_map_(output_map),
      mapping_algorithm_(mapping_algorithm) {}

std::optional<common::types::DroneStepResult> DroneControlImpl::fetchCommand(const common::types::DroneState& current_state, common::types::MappingStepCommand& out_cmd) {
    if (pending_command_.has_value()) {
        out_cmd = pending_command_.value();
    } else {
        out_cmd = mapping_algorithm_.nextStep(current_state, last_scan_ptr_);
        last_scan_ptr_ = nullptr; // Reset scan pointer

        if (out_cmd.status == common::types::AlgorithmStatus::Finished ||
            out_cmd.status == common::types::AlgorithmStatus::FinishedWithUnmappableVoxels) {
            return common::types::DroneStepResult{common::types::DroneStepStatus::Completed, "Mapping complete."};
        }
    }
    return std::nullopt;
}

std::optional<common::types::DroneStepResult> DroneControlImpl::executeMovementChunk(common::types::MovementCommand& move, bool& out_is_final_chunk) {
    common::types::MovementResult result{true, {}};

    try {
        switch (move.type) {
            case common::types::MovementCommandType::Rotate: {
                double max_rot = drone_.max_rotate.numerical_value_in(deg);
                double cmd_angle = move.angle.numerical_value_in(deg);
                double chunk = std::min(std::abs(cmd_angle), max_rot);
                if (cmd_angle < 0) chunk = -chunk;

                result = movement_.rotate(move.rotation, HorizontalAngle(chunk * deg));
                
                double remaining = cmd_angle - chunk;
                if (std::abs(remaining) > 0.001) {
                    move.angle = HorizontalAngle(remaining * deg);
                    out_is_final_chunk = false;
                }
                break;
            }
            case common::types::MovementCommandType::Advance: {
                double max_adv = drone_.max_advance.numerical_value_in(cm);
                double cmd_dist = move.distance.numerical_value_in(cm);
                double chunk = std::min(cmd_dist, max_adv);

                result = movement_.advance(PhysicalLength(chunk * cm));
                
                double remaining = cmd_dist - chunk;
                if (remaining > 0.001) {
                    move.distance = PhysicalLength(remaining * cm);
                    out_is_final_chunk = false;
                }
                break;
            }
            case common::types::MovementCommandType::Elevate: {
                double max_elev = drone_.max_elevate.numerical_value_in(cm);
                double cmd_dist = move.distance.numerical_value_in(cm);
                double chunk = std::min(std::abs(cmd_dist), max_elev);
                if (cmd_dist < 0) chunk = -chunk;

                result = movement_.elevate(PhysicalLength(chunk * cm));
                
                double remaining = cmd_dist - chunk;
                if (std::abs(remaining) > 0.001) {
                    move.distance = PhysicalLength(remaining * cm);
                    out_is_final_chunk = false;
                }
                break;
            }
            case common::types::MovementCommandType::Hover:
                break;
        }
    } catch (const std::runtime_error& e) {
        return common::types::DroneStepResult{
            common::types::DroneStepStatus::Error,
            "Movement exception caught: " + std::string(e.what())
        };
    }

    if (!result) {
        return common::types::DroneStepResult{
            common::types::DroneStepStatus::Error,
            "Movement failed: " + result.message
        };
    }
    
    return std::nullopt;
}

void DroneControlImpl::executeScan(const common::types::MappingStepCommand& cmd) {
    if (cmd.scan_orientation.has_value()) {
        const Orientation& scan_orient = cmd.scan_orientation.value();
        last_scan_storage_ = lidar_.scan(scan_orient);
        last_scan_ptr_ = &last_scan_storage_;

        common::types::LidarConfigData lidar_config = lidar_.config();
        ScanResultToVoxels::applyToMap(
            output_map_,
            gps_.position(),
            gps_.heading(),
            last_scan_storage_,
            lidar_config
        );
    }
}

void DroneControlImpl::logActivity(const common::types::MappingStepCommand& cmd, bool is_final_chunk) {
    // Log the movement
    std::string action_str = "hover";
    if (cmd.movement.has_value()) {
        switch (cmd.movement.value().type) {
            case common::types::MovementCommandType::Rotate:  action_str = "rotate"; break;
            case common::types::MovementCommandType::Advance: action_str = "advance"; break;
            case common::types::MovementCommandType::Elevate: action_str = "elevate"; break;
            case common::types::MovementCommandType::Hover:   action_str = "hover"; break;
        }
    }
    
    common::types::DroneState end_state = state();
    Logger::logMovement(
        end_state.position.x.numerical_value_in(cm),
        end_state.position.y.numerical_value_in(cm),
        end_state.position.z.numerical_value_in(cm),
        end_state.heading.horizontal.numerical_value_in(deg),
        end_state.heading.altitude.numerical_value_in(deg),
        action_str
    );

    // If we performed a scan, log the scan rays
    if (is_final_chunk && cmd.scan_orientation.has_value() && last_scan_ptr_ != nullptr) {
        for (const auto& hit : *last_scan_ptr_) {
            Logger::logScan(
                hit.angle.horizontal.numerical_value_in(deg),
                hit.angle.altitude.numerical_value_in(deg)
            );
        }
    }
}

common::types::DroneStepResult DroneControlImpl::step() {
    Logger::setStep(step_index_);

    common::types::DroneState current_state = state();

    common::types::MappingStepCommand cmd;
    if (auto early_result = fetchCommand(current_state, cmd)) {
        return early_result.value();
    }

    bool is_final_chunk = true;

    if (cmd.movement.has_value()) {
        if (auto error_result = executeMovementChunk(cmd.movement.value(), is_final_chunk)) {
            pending_command_.reset();
            return error_result.value();
        }
    }

    if (is_final_chunk) {
        executeScan(cmd);
        pending_command_.reset();
    } else {
        pending_command_ = cmd;
    }

    logActivity(cmd, is_final_chunk);

    ++step_index_;

    return common::types::DroneStepResult{common::types::DroneStepStatus::Continue, {}};
}

common::types::DroneState DroneControlImpl::state() const {
    return common::types::DroneState{gps_.position(), gps_.heading(), step_index_};
}

} // namespace mission_control_330371063_324976703
