#include <UserCommon/ScanResultToVoxels.h>
#include <UserCommon/OrientationUtils.h>

#include <mp-units/systems/si/math.h>

#include <limits>


namespace user_common_330371063_324976703 {
using namespace common;


namespace {

[[nodiscard]] bool isZeroDistance(PhysicalLength distance) {
    return distance == 0.0 * cm;
}

//did lidar hit no obstacles at max possible distance
[[nodiscard]] bool isMissDistance(PhysicalLength distance) {
    return distance.force_numerical_value_in(cm) == std::numeric_limits<double>::max();
}

// LidarHit angles are relative to the drone's heading. The map update needs a
// world-facing beam direction, so we add the drone heading.
[[nodiscard]] Orientation absoluteBeamOrientation(const Orientation& drone_heading,
                                                  const Orientation& relative_beam) {
    return OrientationUtils::addOrientations(relative_beam, drone_heading);
}

[[nodiscard]] Position3D pointAlongBeam(const Position3D& origin,
                                        const Orientation& beam_orientation,
                                        PhysicalLength distance) {
    Direction3D dir = OrientationUtils::getBeamDirection(beam_orientation);
    return OrientationUtils::pointAlongBeam(origin, dir, distance);
}

// Evidence strength for conflicting writes to the same voxel.
// Occupied is a measured hit, Empty is proven free space, and
// PotentiallyOccupied is only uncertainty from a too-close hit.
[[nodiscard]] int occupancyPriority(common::types::VoxelOccupancy occupancy) {
    switch (occupancy) {
    case common::types::VoxelOccupancy::Occupied:
        return 3;
    case common::types::VoxelOccupancy::Empty:
        return 2;
    case common::types::VoxelOccupancy::PotentiallyOccupied:
        return 1;
    case common::types::VoxelOccupancy::Unmapped:
    case common::types::VoxelOccupancy::OutOfBounds:
        return 0;
    }

    return 0;
}

void setIfStronger(IMutableMap3D& output_map,
                   const Position3D& position,
                   common::types::VoxelOccupancy value) {
    if (!output_map.isInBounds(position)) {
        return;
    }

    const common::types::VoxelOccupancy current_value = output_map.atVoxel(position);
    if (occupancyPriority(value) > occupancyPriority(current_value)) {
        output_map.set(position, value);
    }
}

// Walks a beam segment and applies the same observation to each sampled point.
//if lidar hit something, need to mark the voxels before the hit as empty. if lidar returned 0, need to mark all voxels between 0 and z_min potentially occupied
void markBeamSegment(IMutableMap3D& output_map,
                     const Position3D& scan_origin,
                     const Orientation& beam_orientation,
                     PhysicalLength start_distance,
                     PhysicalLength end_distance,
                     PhysicalLength step,
                     common::types::VoxelOccupancy value) {
    for (PhysicalLength distance = start_distance; distance <= end_distance; distance += step) {
        const Position3D current_point = pointAlongBeam(scan_origin, beam_orientation, distance);
        if (!output_map.isInBounds(current_point)) {
            break;
        }
        setIfStronger(output_map, current_point, value);
    }
}

} // anonynous namespace, that allows all the functions above be private for this file

void ScanResultToVoxels::applyToMap(IMutableMap3D& output_map,
                                    const Position3D& scan_origin,
                                    const Orientation& drone_heading,
                                    const common::types::LidarScanResult& scan,
                                    const common::types::LidarConfigData& lidar_config) {
    if (!output_map.isInBounds(scan_origin)) {
        return;
    }

    // Use a sub-voxel step like MockLidar so we do not skip thin voxels along diagonal rays.
    const PhysicalLength step = 0.1 * output_map.getMapConfig().resolution;
    if (step <= 0.0 * cm) {
        return;
    }

    for (const common::types::LidarHit& hit : scan) {
        const Orientation beam_orientation = absoluteBeamOrientation(drone_heading, hit.angle);

        if (isZeroDistance(hit.distance)) {
            // Distance 0 means the hit happened before z_min. The exact voxel is unknown, so mark the near segment as uncertain.
            markBeamSegment(output_map,
                            scan_origin,
                            beam_orientation,
                            0.0 * cm,
                            lidar_config.z_min,
                            step,
                            common::types::VoxelOccupancy::PotentiallyOccupied);
            continue;
        }

        if (isMissDistance(hit.distance)) {
            // A miss means the beam found no obstacle through the measurable
            // range, so sampled points along the ray are empty.
            markBeamSegment(output_map,
                            scan_origin,
                            beam_orientation,
                            0.0 * cm,
                            lidar_config.z_max,
                            step,
                            common::types::VoxelOccupancy::Empty);
            continue;
        }

        if (hit.distance > 0.0 * cm) {
            // A normal hit proves the path before the hit is empty and the hit position itself is occupied.
            markBeamSegment(output_map,
                            scan_origin,
                            beam_orientation,
                            0.0 * cm,
                            hit.distance,
                            step,
                            common::types::VoxelOccupancy::Empty);
            setIfStronger(output_map,
                          pointAlongBeam(scan_origin, beam_orientation, hit.distance),
                          common::types::VoxelOccupancy::Occupied);
        }
    }
}



} // namespace user_common_330371063_324976703
