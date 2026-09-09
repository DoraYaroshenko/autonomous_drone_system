#pragma once

#include <Simulator/ISimulationRunFactory.h>
#include <Common/MappingAlgorithmFactory.h>
#include <Common/MissionControlFactory.h>




namespace simulator {
namespace types {
using namespace common::types;
using namespace simulator::types;
}
using namespace common;
namespace user_common_330371063_324976703 {}
using namespace user_common_330371063_324976703;

class SimulationRunFactoryImpl final : public ISimulationRunFactory {
public:
    SimulationRunFactoryImpl(
        common::MappingAlgorithmFactory algo_factory,
        common::MissionControlFactory mc_factory,
        bool is_verbose = false);

    [[nodiscard]] std::unique_ptr<ISimulationRun>
    create(const types::SimulationConfigData& simulation,
           const types::MissionConfigData& mission,
           const types::DroneConfigData& drone,
           const types::LidarConfigData& lidar,
           const std::filesystem::path& output_path) override;

private:
    common::MappingAlgorithmFactory algo_factory_;
    common::MissionControlFactory mc_factory_;
    bool is_verbose_;
};




} // namespace simulator
