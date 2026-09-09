#pragma once

#include <MissionControl/IDroneControl.h>
#include <Common/IDroneMovement.h>
#include <Common/IGPS.h>
#include <Common/ILidar.h>
#include <Common/IMap3D.h>
#include <Common/IMappingAlgorithm.h>
#include <Common/IMissionControl.h>
#include <Common/IMutableMap3D.h>
#include <Simulator/ISimulationRun.h>

#include <filesystem>
#include <memory>




namespace simulator {
namespace types {
using namespace common::types;
using namespace simulator::types;
}
using namespace common;
namespace user_common_330371063_324976703 {}
using namespace user_common_330371063_324976703;

class SimulationRunImpl final : public ISimulationRun {
public:
    SimulationRunImpl(std::unique_ptr<const IMap3D> hidden_map,
                      std::unique_ptr<IMutableMap3D> output_map,
                      std::unique_ptr<IGPS> gps,
                      std::unique_ptr<IDroneMovement> movement,
                      std::unique_ptr<ILidar> lidar,
                      std::unique_ptr<IMappingAlgorithm> mapping_algorithm,
                      std::unique_ptr<IMissionControl> mission_control,
                      // Changed: stores run metadata needed to build SimulationResult.
                      types::SimulationConfigData simulation_config,
                      types::MissionConfigData mission_config,
                      std::filesystem::path output_map_file);
    [[nodiscard]] types::SimulationResult run() override;

private:
    std::unique_ptr<const IMap3D> hidden_map_;
    std::unique_ptr<IMutableMap3D> output_map_;
    std::unique_ptr<IGPS> gps_;
    std::unique_ptr<IDroneMovement> movement_;
    std::unique_ptr<ILidar> lidar_;
    std::unique_ptr<IMappingAlgorithm> mapping_algorithm_;
    std::unique_ptr<IMissionControl> mission_control_;
    types::SimulationConfigData simulation_config_;
    types::MissionConfigData mission_config_;
    std::filesystem::path output_map_file_;
};




} // namespace simulator
