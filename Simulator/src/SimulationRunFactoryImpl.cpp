#include <Simulator/SimulationRunFactoryImpl.h>
#include <Simulator/SimulationRunImpl.h>
#include <Simulator/Map3DImpl.h>
#include <Simulator/MockGPS.h>
#include <Simulator/MockLidar.h>
#include <Simulator/MockMovement.h>

#include <stdexcept>
#include <utility>
#include <iostream>

namespace simulator {
namespace types {
using namespace common::types;
using namespace simulator::types;
}
using namespace common;

SimulationRunFactoryImpl::SimulationRunFactoryImpl(
    common::MappingAlgorithmFactory algo_factory,
    common::MissionControlFactory mc_factory,
    bool is_verbose)
    : algo_factory_(std::move(algo_factory)),
      mc_factory_(std::move(mc_factory)),
      is_verbose_(is_verbose) {
}

std::unique_ptr<ISimulationRun>
SimulationRunFactoryImpl::create(const types::SimulationConfigData& simulation,
                                 const types::MissionConfigData& mission,
                                 const types::DroneConfigData& drone,
                                 const types::LidarConfigData& lidar,
                                 const std::filesystem::path& output_path) {
                                     
    auto hidden_npy = std::make_shared<NpyArray>();
    const char* err = hidden_npy->LoadNPY(simulation.map_filename.string());
    if (err != nullptr) {
        throw std::runtime_error("Failed to load map: " + simulation.map_filename.string() + " error: " + std::string(err));
    }
    auto hidden_map = std::make_unique<Map3DImpl>(
        hidden_npy,
        types::MapConfig{
            mission.mission_bounds,
            common::Position3D{0.0 * common::cm, 0.0 * common::cm, 0.0 * common::cm},
            simulation.map_resolution
        }
    );
    
    // Create output map with same physical size but possibly different resolution.
    PhysicalLength res = simulation.map_resolution * mission.output_mapping_resolution_factor;
    auto sx = static_cast<std::size_t>(std::ceil((mission.mission_bounds.max_x - mission.mission_bounds.min_x).numerical_value_in(cm) / res.numerical_value_in(cm)));
    auto sy = static_cast<std::size_t>(std::ceil((mission.mission_bounds.max_y - mission.mission_bounds.min_y).numerical_value_in(cm) / res.numerical_value_in(cm)));
    auto sz = static_cast<std::size_t>(std::ceil((mission.mission_bounds.max_height - mission.mission_bounds.min_height).numerical_value_in(cm) / res.numerical_value_in(cm)));

    std::vector<std::size_t> shape{sx, sy, sz};
    auto output_npy = std::make_shared<NpyArray>(shape, sizeof(char), NpyArray::GetTypeChar(typeid(char)), false);
    output_npy->Allocate(); //allocates memory for a map
    char* data = output_npy->Data<char>();
    std::fill(data, data + output_npy->NumValue(), static_cast<char>(types::VoxelOccupancy::Unmapped));

    types::MapConfig out_config{
        mission.mission_bounds,
        Position3D{mission.mission_bounds.min_x, mission.mission_bounds.min_y, mission.mission_bounds.min_height},
        res
    };
    auto output_map = std::make_unique<Map3DImpl>(output_npy, out_config);

    auto gps = std::make_unique<MockGPS>(simulation.initial_drone_position, Orientation{simulation.initial_angle, AltitudeAngle(0.0 * deg)}, mission.gps_resolution);
    auto movement = std::make_unique<MockMovement>(*gps, *hidden_map, drone.radius);
    auto lidar_sensor = std::make_unique<MockLidar>(lidar, *hidden_map, *gps);

    common::MappingAlgorithmDependencies algo_deps{
        mission,
        lidar,
        drone,
        *output_map
    };
    auto mapping_algorithm = algo_factory_(algo_deps);

    common::MissionControlDependencies mc_deps{
        mission,
        drone,
        *lidar_sensor,
        *gps,
        *movement,
        *output_map,
        *mapping_algorithm,
        output_path,
        is_verbose_
    };
    auto mission_control = mc_factory_(mc_deps);

    return std::make_unique<SimulationRunImpl>(
        std::move(hidden_map),
        std::move(output_map),
        std::move(gps),
        std::move(movement),
        std::move(lidar_sensor),
        std::move(mapping_algorithm),
        std::move(mission_control),
        simulation,
        mission,
        output_path
    );
}

} // namespace simulator
