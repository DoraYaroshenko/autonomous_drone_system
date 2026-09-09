#pragma once

#include <MissionControl/IDroneControl.h>
#include <Common/IDroneMovement.h>
#include <Common/IGPS.h>
#include <Common/ILidar.h>
#include <Common/IMappingAlgorithm.h>
#include <Common/IMutableMap3D.h>

namespace mission_control_330371063_324976703 {
using namespace common;

class DroneControlImpl final : public mission_control::IDroneControl {
public:
    DroneControlImpl(common::types::DroneConfigData drone,
                     ILidar& lidar,
                     IGPS& gps,
                     IDroneMovement& movement,
                     IMutableMap3D& output_map,
                     IMappingAlgorithm& mapping_algorithm);

    [[nodiscard]] common::types::DroneStepResult step() override;
    [[nodiscard]] common::types::DroneState state() const override;

private:
    std::optional<common::types::DroneStepResult> fetchCommand(const common::types::DroneState& current_state, common::types::MappingStepCommand& out_cmd);
    std::optional<common::types::DroneStepResult> executeMovementChunk(common::types::MovementCommand& move, bool& out_is_final_chunk);
    void executeScan(const common::types::MappingStepCommand& cmd);
    void logActivity(const common::types::MappingStepCommand& cmd, bool is_final_chunk);

    common::types::DroneConfigData drone_;
    ILidar& lidar_;
    IGPS& gps_;
    IDroneMovement& movement_;
    IMutableMap3D& output_map_;
    IMappingAlgorithm& mapping_algorithm_;
    std::size_t step_index_ = 0;
    
    // State machine for splitting large commands (e.g. drone movements bigger than maximum)
    std::optional<common::types::MappingStepCommand> pending_command_;

    const common::types::LidarScanResult* last_scan_ptr_ = nullptr; //compliance with IMappingAlgorithm.h. Also, the object can't be modified through that pointer
    common::types::LidarScanResult last_scan_storage_;
};

} // namespace mission_control_330371063_324976703
