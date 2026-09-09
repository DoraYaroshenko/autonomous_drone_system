#pragma once

#include <MissionControl/IDroneControl.h>
#include <Common/IMap3D.h>
#include <Common/IMissionControl.h>
#include <Common/IMutableMap3D.h>
#include <Common/MissionControlFactory.h>

#include <filesystem>

namespace mission_control_330371063_324976703 {
using namespace common;

class MissionControlImpl_330371063_324976703 final : public IMissionControl {
public:
    explicit MissionControlImpl_330371063_324976703(common::MissionControlDependencies deps);

    [[nodiscard]] common::types::MissionRunResult runMission() override;

private:
    void recordError(const std::string& errorCode, const std::string& message, common::types::MissionRunResult& result, const std::string& logPrefix = "") const;
    void executeMissionLoop(common::types::MissionRunResult& result);
    void finalizeMissionResult(common::types::MissionRunResult& result);
    void saveMap(common::types::MissionRunResult& result);

    common::types::MissionConfigData mission_;
    common::types::DroneConfigData drone_;
    IMutableMap3D& output_map_;
    std::unique_ptr<mission_control::IDroneControl> drone_control_;
    std::filesystem::path output_map_file_;
    bool verbose_;
};

} // namespace mission_control_330371063_324976703
