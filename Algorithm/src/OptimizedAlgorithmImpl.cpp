#include "Algorithm/OptimizedAlgorithmImpl.h"
#include <Common/Units.h>
#include <Common/IMap3D.h>
#include <UserCommon/CollisionUtils.h>
#include <Common/MappingAlgorithmRegistration.h>
#include <UserCommon/OrientationUtils.h>

#include <mp-units/systems/si/math.h>

#include <algorithm> //provides std::sort, std::find, std::max
#include <cmath> //provides std::sqrt, std::cos, std::pow
#include <queue>
#include <vector>
#include <unordered_map>
#include <unordered_set>
#include <optional>
#include <functional>
#include <array>
#include <limits> //provides std::numeric_limits - infinity for a double, maximum possible int
#include <tuple>

namespace algorithm_330371063_324976703 {

using namespace user_common_330371063_324976703;

namespace {

//in cpp, class and struct have the same functionality, different default visibility
// Voxel index in integer grid coordinates (one voxel per resolution unit).
//here we use struct and not a class, because fields don't need encapsulation or protection
struct VoxelIndex {
    int x;
    int y;
    int z;

    VoxelIndex operator+(const VoxelIndex& other) const {
        return {x + other.x, y + other.y, z + other.z};
    }

    VoxelIndex operator-(const VoxelIndex& other) const {
        return {x - other.x, y - other.y, z - other.z};
    }

    bool operator==(const VoxelIndex& other) const {
        return x == other.x && y == other.y && z == other.z;
    }
};

// Convert a world-space Position3D to a voxel index given a resolution. Each voxel spans [i*res, (i+1)*res) in each axis.
[[nodiscard]] inline VoxelIndex positionToVoxelIndex(const Position3D& pos, PhysicalLength resolution) {
    double res = resolution.force_numerical_value_in(cm);
    return {
        //static cast ensures the conversion is logical at compile time
        static_cast<int>(std::floor(pos.x.force_numerical_value_in(cm) / res)),
        static_cast<int>(std::floor(pos.y.force_numerical_value_in(cm) / res)),
        static_cast<int>(std::floor(pos.z.force_numerical_value_in(cm) / res))
    };
}

// Convert a voxel index center back to a world-space Position3D.
[[nodiscard]] inline Position3D voxelIndexToPosition(const VoxelIndex& vi, PhysicalLength resolution) {
    double res = resolution.force_numerical_value_in(cm);
    return {
        XLength((vi.x + 0.5) * res * x_extent[cm]),
        YLength((vi.y + 0.5) * res * y_extent[cm]),
        ZLength((vi.z + 0.5) * res * z_extent[cm])
    };
}

// ---------------------------------------------------------------------------
// Helpers: VoxelIndex hashing for use in unordered containers
// ---------------------------------------------------------------------------
struct VoxelIndexHash {
    std::size_t operator()(const VoxelIndex& v) const noexcept {
        auto h1 = std::hash<int>{}(v.x); //{} creates an instance of a hasher object
        auto h2 = std::hash<int>{}(v.y);
        auto h3 = std::hash<int>{}(v.z);
        return h1 ^ (h2 * 2654435761u) ^ (h3 * 40503u); //xor with large prime numbers
    }
};

using VoxelSet = std::unordered_set<VoxelIndex, VoxelIndexHash>;
using VoxelMap = std::unordered_map<VoxelIndex, VoxelIndex, VoxelIndexHash>;

// All the neighbours of a specific voxel
constexpr std::array<VoxelIndex, 6> kNeighbourOffsets = {{
    { 1,  0,  0},
    {-1,  0,  0},
    { 0,  1,  0},
    { 0, -1,  0},
    { 0,  0,  1},
    { 0,  0, -1},
}};

// ---------------------------------------------------------------------------
// Collision-safe voxel queries
// ---------------------------------------------------------------------------

/// Check whether a voxel is "passable" for the drone — i.e. the drone can potentially move there. We require:
///   - Drone is fully in bounds
///   - No voxel inside the drone sphere is Occupied
/// Unmapped voxels are allowed (we'll scan before actually entering).
//checks that the drone center can be safely positioned inside the voxel's center
bool isVoxelPassable(const VoxelIndex& vi,
                     PhysicalLength resolution,
                     PhysicalLength radius,
                     const IMap3D& map) {
    Position3D centre = voxelIndexToPosition(vi, resolution);

    if (!CollisionUtils::isDroneFullyInBounds(map, centre, radius)) {
        return false;
    }

    bool isPassable = true;
    CollisionUtils::forEachVoxelInDroneSphere(centre, radius, resolution, [&](const Position3D& p) {
        if (!map.isInBounds(p)) {
            isPassable = false;
            return false;
        }
        auto v = map.atVoxel(p);
        if (v == common::types::VoxelOccupancy::Occupied || v == common::types::VoxelOccupancy::PotentiallyOccupied) {
            isPassable = false;
            return false;
        }
        return true;
    });

    return isPassable;
}

/// Stricter check: all voxels inside the drone sphere must be Empty.
/// Used before actually moving into a voxel.
bool isVoxelKnownSafe(const VoxelIndex& vi,
                      PhysicalLength resolution,
                      PhysicalLength radius,
                      const IMap3D& map) {
    Position3D centre = voxelIndexToPosition(vi, resolution);

    if (!CollisionUtils::isDroneFullyInBounds(map, centre, radius)) {
        return false;
    }

    bool isSafe = true;
    CollisionUtils::forEachVoxelInDroneSphere(centre, radius, resolution, [&](const Position3D& voxelPosition) {
        if (!map.isInBounds(voxelPosition)) {
            isSafe = false;
            return false;
        }
        auto occupancy = map.atVoxel(voxelPosition);
        if (occupancy != common::types::VoxelOccupancy::Empty) {
            isSafe = false;
            return false;
        }
        return true;
    });

    return isSafe;
}

/// Gather all unmapped/potentially-occupied voxel positions inside the drone
/// sphere at a given voxel position. Returns voxel indices that need scanning.
std::vector<VoxelIndex> getUnknownVoxelsInSphere(const VoxelIndex& vi,
                                                  PhysicalLength resolution,
                                                  PhysicalLength radius,
                                                  const IMap3D& map) {
    std::vector<VoxelIndex> unknowns;
    Position3D centre = voxelIndexToPosition(vi, resolution);
    VoxelSet seen;

    CollisionUtils::forEachVoxelInDroneSphere(centre, radius, resolution, [&](const Position3D& voxelPosition) {
        if (!map.isInBounds(voxelPosition)) return true;
        
        auto occupancy = map.atVoxel(voxelPosition);
        if (occupancy == common::types::VoxelOccupancy::Unmapped) {
            VoxelIndex currentVoxelIndex = positionToVoxelIndex(voxelPosition, resolution);
            if (!seen.contains(currentVoxelIndex)) {
                seen.insert(currentVoxelIndex);
                unknowns.push_back(currentVoxelIndex);
            }
        }
        return true;
    });

    return unknowns;
}

// ---------------------------------------------------------------------------
// Compute scan orientations for unmapped voxels near a position
// ---------------------------------------------------------------------------
//generate scan orientation for a single voxel
Orientation scanOrientationToVoxel(const Position3D& drone_pos,
                                   const Orientation& drone_heading,
                                   const VoxelIndex& target_voxel,
                                   PhysicalLength resolution) {
    Position3D target = voxelIndexToPosition(target_voxel, resolution);

    double dx = (target.x - drone_pos.x).numerical_value_in(cm);
    double dy = (target.y - drone_pos.y).numerical_value_in(cm);
    double dz = (target.z - drone_pos.z).numerical_value_in(cm);

    double horiz = std::sqrt(dx * dx + dy * dy);

    HorizontalAngle abs_h(si::atan2(dy * cm, dx * cm));
    AltitudeAngle abs_v(si::atan2(dz * cm, horiz * cm));

    HorizontalAngle h_diff = abs_h - drone_heading.horizontal; //difference between absolute angle of the target and drone's heading
    h_diff = user_common_330371063_324976703::OrientationUtils::normalizeAngle180(h_diff);

    AltitudeAngle v_diff = abs_v - drone_heading.altitude;
    v_diff = user_common_330371063_324976703::OrientationUtils::normalizeAngle180(v_diff);

    return Orientation{h_diff, v_diff};
}

/// Generate scan orientations for all unmapped voxels within a given range (in voxel units) of the drone
std::vector<Orientation> generateScansForUnmapped(const Position3D& drone_pos,
                                                   const Orientation& drone_heading,
                                                   PhysicalLength resolution,
                                                   const IMap3D& map,
                                                   int scan_range) { //the radius of searched box in voxel units
    std::vector<Orientation> scans;
    VoxelIndex drone_vi = positionToVoxelIndex(drone_pos, resolution);

    for (int dx = -scan_range; dx <= scan_range; ++dx) {
        for (int dy = -scan_range; dy <= scan_range; ++dy) {
            for (int dz = -scan_range; dz <= scan_range; ++dz) {
                VoxelIndex target{drone_vi.x + dx, drone_vi.y + dy, drone_vi.z + dz};
                Position3D p = voxelIndexToPosition(target, resolution);
                if (!map.isInBounds(p)) continue;
                auto v = map.atVoxel(p);
                if (v == common::types::VoxelOccupancy::Unmapped) {
                    Orientation orient = scanOrientationToVoxel(drone_pos, drone_heading, target, resolution);
                    bool exists = false;
                    for (const auto& existing : scans) { //Optimization step: the drone has field of view, so if we have two very close voxels, scanning one of them almost certainly will reveal the other one too
                        if (std::abs((existing.horizontal - orient.horizontal).numerical_value_in(deg)) < 1.0 &&
                            std::abs((existing.altitude - orient.altitude).numerical_value_in(deg)) < 1.0) {
                            exists = true;
                            break;
                        }
                    }
                    if (!exists) {
                        scans.push_back(orient);
                    }
                }
            }
        }
    }
    return scans;
}

// ---------------------------------------------------------------------------
// BFS frontier search using passable (not strictly known-safe) expansion.
// A "frontier" is a passable voxel that is adjacent to an unmapped voxel
// and is NOT the start voxel (so the drone actually has to move).
// If no moveable frontier is found but the start itself has unmapped
// neighbours, we return the start as a fallback.
// ---------------------------------------------------------------------------
struct FrontierResult {
    bool found = false;
    std::vector<VoxelIndex> path;
};

FrontierResult findFrontier(const Position3D& drone_pos,
                            PhysicalLength resolution,
                            PhysicalLength radius,
                            const IMap3D& map) {
    VoxelIndex start = positionToVoxelIndex(drone_pos, resolution);

    std::queue<VoxelIndex> bfs_queue;
    VoxelSet visited;
    VoxelMap came_from;

    bfs_queue.push(start);
    visited.insert(start);

    bool start_is_frontier = false;

    while (!bfs_queue.empty()) {
        VoxelIndex current = bfs_queue.front();
        bfs_queue.pop();

        // Check if this voxel has any unmapped neighbours. Returns it if true
        bool has_unmapped_neighbour = false;
        for (const auto& offset : kNeighbourOffsets) {
            VoxelIndex nb = current + offset;
            Position3D nb_pos = voxelIndexToPosition(nb, resolution);
            if (!map.isInBounds(nb_pos)) continue;

            auto v = map.atVoxel(nb_pos);
            if (v == common::types::VoxelOccupancy::Unmapped) {
                has_unmapped_neighbour = true;
                break;
            }
        }

        if (has_unmapped_neighbour) {
            if (current == start) {
                // Remember that start is a frontier, but keep looking for one that requires movement
                start_is_frontier = true;
            } else {
                std::vector<VoxelIndex> path;
                VoxelIndex node = current;
                while (!(node == start)) {
                    path.push_back(node);
                    node = came_from[node];
                }
                path.push_back(start);
                std::reverse(path.begin(), path.end());
                return {true, path};
            }
        }

        // looks at the 6 neighbors again, uses isVoxelPassable() to check if the drone can physically fit inside that neighbor. If it is safe, and we haven't visited it yet, it pushes that neighbor onto the bfs_queue so the algorithm can step into it and repeat the whole process on the next cycle.
        for (const auto& offset : kNeighbourOffsets) {
            VoxelIndex nb = current + offset;
            if (visited.count(nb)) continue;
            visited.insert(nb);

            if (isVoxelPassable(nb, resolution, radius, map)) {
                came_from[nb] = current;
                bfs_queue.push(nb); //note we only add passable voxels to the queue
            }
        }
    }

    // If we found no remote frontier but start itself is one, return it
    if (start_is_frontier) {
        return {true, {start}};
    }

    return {false, {}};
}



} // namespace

// ---------------------------------------------------------------------------
// State machine for the Frontier Exploration algorithm
// ---------------------------------------------------------------------------

enum class SweepPhase {
    ScanSurroundings,  // Generate scans for nearby unmapped voxels
    ExecuteScans,      // Drain pending scan queue
    FindFrontier,      // BFS to find nearest unmapped frontier
    FollowPath,        // Execute movement commands along path
    ScanBeforeMove,    // Scan unknown voxels in drone sphere at next path step
    Finished,
};

struct SweepState {
    SweepPhase phase = SweepPhase::ScanSurroundings;
    std::vector<Orientation> pending_scans;
    std::vector<VoxelIndex> planned_path;
    std::size_t path_index = 0;
    VoxelIndex frontier_target{0, 0, 0};
    int stall_count = 0; //optimization: counts the number of times we get frontier_target==last_frontier. If we get more than 50 stalls, be stop
    VoxelIndex last_frontier{-999, -999, -999};
    SweepPhase return_after_scans = SweepPhase::FindFrontier; // where to go after ExecuteScans
    bool initial_scan_done = false;
};

// ---------------------------------------------------------------------------
// Phase handlers
// ---------------------------------------------------------------------------
//if there were many states, we would create an interface for state with method to handle it. Every instanse is a state. Automatic transition between states. We have a few states, so it is ok
std::optional<common::types::MappingStepCommand> handleScanSurroundings(
    SweepState& sweep,
    const Position3D& pos,
    const Orientation& heading,
    PhysicalLength resolution,
    const IMap3D& map) {

    // On first call, do a broad scan (range 5) to map nearby space. On subsequent calls, use a smaller range.
    int range = sweep.initial_scan_done ? 3 : 5;
    sweep.initial_scan_done = true;

    sweep.pending_scans = generateScansForUnmapped(pos, heading, resolution, map, range);

    if (!sweep.pending_scans.empty()) { 
        sweep.return_after_scans = SweepPhase::FindFrontier;
        sweep.phase = SweepPhase::ExecuteScans;
    } else { //meaning no unmapped voxels in a given range
        sweep.phase = SweepPhase::FindFrontier;
    }
    return std::nullopt; //for uniform interface for all handlers and to prevent the main loop from stopping
}

std::optional<common::types::MappingStepCommand> handleExecuteScans(SweepState& sweep) {
    if (!sweep.pending_scans.empty()) {
        common::types::MappingStepCommand cmd;
        cmd.scan_orientation = sweep.pending_scans.back();
        sweep.pending_scans.pop_back();
        cmd.status = common::types::AlgorithmStatus::Working;
        return cmd;
    }
    sweep.phase = sweep.return_after_scans;
    return std::nullopt;
}

std::optional<common::types::MappingStepCommand> handleFindFrontier(
    SweepState& sweep,
    const Position3D& pos,
    PhysicalLength resolution,
    PhysicalLength radius,
    const IMap3D& map) {

    auto result = findFrontier(pos, resolution, radius, map);
    if (result.found) {
        sweep.frontier_target = result.path.back();

        // Stall detection
        if (sweep.frontier_target == sweep.last_frontier) {
            sweep.stall_count++;
        } else {
            sweep.stall_count = 0;
            sweep.last_frontier = sweep.frontier_target;
        }
        if (sweep.stall_count > 50) {
            sweep.phase = SweepPhase::Finished;
            return std::nullopt;
        }

        if (result.path.size() <= 1) {
            // We're at the frontier; scan again and the map should update,
            // then next time findFrontier will find a remote one
            sweep.phase = SweepPhase::ScanSurroundings;
        } else {
            sweep.planned_path = std::move(result.path);
            sweep.path_index = 1; // Skip start
            sweep.phase = SweepPhase::FollowPath;
        }
    } else {
        sweep.phase = SweepPhase::Finished;
    }
    return std::nullopt;
}

/// Before moving to a path step, check if we need to scan unknown voxels
/// in the drone sphere at that position. If so, scan from current position
/// toward those unknowns.
std::optional<common::types::MappingStepCommand> handleScanBeforeMove(
    SweepState& sweep,
    const Position3D& pos,
    const Orientation& heading,
    PhysicalLength resolution,
    PhysicalLength radius,
    const IMap3D& map) {

    if (sweep.path_index >= sweep.planned_path.size()) {
        sweep.phase = SweepPhase::FollowPath;
        return std::nullopt;
    }

    VoxelIndex next_vi = sweep.planned_path[sweep.path_index];
    auto unknowns = getUnknownVoxelsInSphere(next_vi, resolution, radius, map);

    if (unknowns.empty()) {
        // All clear — proceed to move
        sweep.phase = SweepPhase::FollowPath;
        return std::nullopt;
    }

    // Generate scans toward the unknown voxels
    sweep.pending_scans.clear();
    for (const auto& unk : unknowns) {
        Orientation orient = scanOrientationToVoxel(pos, heading, unk, resolution);
        bool exists = false;
        for (const auto& existing : sweep.pending_scans) {
            if (std::abs((existing.horizontal - orient.horizontal).numerical_value_in(deg)) < 1.0 &&
                std::abs((existing.altitude - orient.altitude).numerical_value_in(deg)) < 1.0) {
                exists = true;
                break;
            }
        }
        if (!exists) {
            sweep.pending_scans.push_back(orient);
        }
    }

    if (!sweep.pending_scans.empty()) {
        sweep.return_after_scans = SweepPhase::FollowPath;
        sweep.phase = SweepPhase::ExecuteScans;
    } else {
        sweep.phase = SweepPhase::FollowPath;
    }
    return std::nullopt;
}

bool checkPathStatusAndSafety(SweepState& sweep,
                              PhysicalLength resolution,
                              PhysicalLength radius,
                              const IMap3D& map) {
    if (sweep.path_index >= sweep.planned_path.size()) {
        sweep.phase = SweepPhase::ScanSurroundings;
        return true;
    }

    VoxelIndex voxel_index = sweep.planned_path[sweep.path_index];

    if (!isVoxelKnownSafe(voxel_index, resolution, radius, map)) {
        if (!isVoxelPassable(voxel_index, resolution, radius, map)) {
            sweep.phase = SweepPhase::ScanSurroundings;
            return true;
        }
        sweep.phase = SweepPhase::ScanBeforeMove;
        return true;
    }
    return false;
}

std::optional<common::types::MappingStepCommand> handleElevationStep(
    SweepState& sweep, double dx, double dy, double dz, 
    PhysicalLength resolution, const common::types::DroneConfigData& drone_config) {
    
    if (std::abs(dz) <= 0.01) return std::nullopt;

    common::types::MappingStepCommand cmd;
    cmd.status = common::types::AlgorithmStatus::Working;

    PhysicalLength abs_dist(std::abs(dz) * cm);
    PhysicalLength step = (abs_dist < drone_config.max_elevate) ? abs_dist : drone_config.max_elevate;
    
    PhysicalLength max_safe_step = (abs_dist < resolution) ? abs_dist : resolution;
    if (step > max_safe_step) {
        step = max_safe_step;
    }

    cmd.movement = common::types::MovementCommand{
        common::types::MovementCommandType::Elevate, {}, {}, (dz > 0) ? step : -step
    };
    if (abs_dist <= drone_config.max_elevate + 0.001 * cm && std::abs(dx) < 0.01 && std::abs(dy) < 0.01) {
        sweep.path_index++;
    }
    return cmd;
}

std::optional<common::types::MappingStepCommand> handleRotationStep(
    double dx, double dy, const Orientation& heading, 
    const common::types::DroneConfigData& drone_config) {
    
    HorizontalAngle desired_h(si::atan2(dy * cm, dx * cm));
    HorizontalAngle current_h = heading.horizontal;
    HorizontalAngle h_diff = desired_h - current_h;
    h_diff = user_common_330371063_324976703::OrientationUtils::normalizeAngle180(h_diff);

    double abs_diff = std::abs(h_diff.numerical_value_in(deg));

    if (abs_diff > 0.5) {
        common::types::MappingStepCommand cmd;
        cmd.status = common::types::AlgorithmStatus::Working;
        HorizontalAngle max_rot(drone_config.max_rotate.numerical_value_in(deg) * horizontal_angle[deg]);
        HorizontalAngle abs_angle(abs_diff * horizontal_angle[deg]);
        HorizontalAngle step_angle = (abs_angle < max_rot) ? abs_angle : max_rot;

        cmd.movement = common::types::MovementCommand{
            common::types::MovementCommandType::Rotate,
            h_diff.numerical_value_in(deg) > 0 ? common::types::RotationDirection::Left : common::types::RotationDirection::Right,
            step_angle,
            {}
        };
        return cmd;
    }
    return std::nullopt;
}

std::optional<common::types::MappingStepCommand> handleAdvanceStep(
    SweepState& sweep, double horiz_dist, 
    PhysicalLength resolution, const common::types::DroneConfigData& drone_config) {
    
    common::types::MappingStepCommand cmd;
    cmd.status = common::types::AlgorithmStatus::Working;
    
    PhysicalLength adv_dist(horiz_dist * cm);
    PhysicalLength step = (adv_dist < drone_config.max_advance) ? adv_dist : drone_config.max_advance;

    PhysicalLength max_safe_step = (adv_dist < resolution) ? adv_dist : resolution;
    if (step > max_safe_step) {
        step = max_safe_step;
    }

    cmd.movement = common::types::MovementCommand{
        common::types::MovementCommandType::Advance, {}, {}, step
    };
    if (adv_dist <= drone_config.max_advance + 0.001 * cm) {
        sweep.path_index++;
    }
    return cmd;
}

std::optional<common::types::MappingStepCommand> handleFollowPath(
    SweepState& sweep,
    const Position3D& pos,
    const Orientation& heading,
    PhysicalLength resolution,
    PhysicalLength radius,
    const IMap3D& map,
    const common::types::DroneConfigData& drone_config) {

    if (checkPathStatusAndSafety(sweep, resolution, radius, map)) {
        return std::nullopt;
    }

    VoxelIndex voxel_index = sweep.planned_path[sweep.path_index];
    Position3D target_pos = voxelIndexToPosition(voxel_index, resolution);

    double dx = (target_pos.x - pos.x).numerical_value_in(cm);
    double dy = (target_pos.y - pos.y).numerical_value_in(cm);
    double dz = (target_pos.z - pos.z).numerical_value_in(cm);

    if (auto elevateCmd = handleElevationStep(sweep, dx, dy, dz, resolution, drone_config)) {
        return elevateCmd;
    }

    double horiz_dist = std::sqrt(dx * dx + dy * dy);
    if (horiz_dist > 0.01) {
        if (auto rotateCmd = handleRotationStep(dx, dy, heading, drone_config)) {
            return rotateCmd;
        }
        if (auto advanceCmd = handleAdvanceStep(sweep, horiz_dist, resolution, drone_config)) {
            return advanceCmd;
        }
    }

    sweep.path_index++;
    return std::nullopt;
}

OptimizedAlgorithmImpl_330371063_324976703::~OptimizedAlgorithmImpl_330371063_324976703() {
}

common::types::MappingStepCommand OptimizedAlgorithmImpl_330371063_324976703::nextStep(const common::types::DroneState& state,
                                                         const common::types::LidarScanResult* /*latest_scan*/) {
    if (!sweep_state_) {
        sweep_state_ = std::make_unique<SweepState>();
    }
    auto& sweep = *sweep_state_;
    const Position3D& pos = state.position;
    const Orientation& heading = state.heading;
    PhysicalLength resolution = output_map_.getMapConfig().resolution;
    PhysicalLength radius = drone_config_.radius;

    while (true) {
        std::optional<common::types::MappingStepCommand> cmd_opt;

        switch (sweep.phase) {
            case SweepPhase::ScanSurroundings:
                cmd_opt = handleScanSurroundings(sweep, pos, heading, resolution, output_map_);
                break;
            case SweepPhase::ExecuteScans:
                cmd_opt = handleExecuteScans(sweep);
                break;
            case SweepPhase::FindFrontier:
                cmd_opt = handleFindFrontier(sweep, pos, resolution, radius, output_map_);
                break;
            case SweepPhase::FollowPath:
                cmd_opt = handleFollowPath(sweep, pos, heading, resolution, radius, output_map_, drone_config_);
                break;
            case SweepPhase::ScanBeforeMove:
                cmd_opt = handleScanBeforeMove(sweep, pos, heading, resolution, radius, output_map_);
                break;
            case SweepPhase::Finished: {
                common::types::MappingStepCommand cmd;
                cmd.status = common::types::AlgorithmStatus::Finished;
                return cmd;
            }
        }

        if (cmd_opt) { //if a handler returned a command, this step is over and we return it. If it didn't, we go back to the top
            return *cmd_opt;
        }
    }
}

REGISTER_MAPPING_ALGORITHM(OptimizedAlgorithmImpl_330371063_324976703);

} // namespace algorithm_330371063_324976703
