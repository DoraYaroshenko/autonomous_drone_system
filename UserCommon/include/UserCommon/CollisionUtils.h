#pragma once

#include <Common/IMap3D.h>
#include <Common/Types.h>
#include <algorithm>

namespace user_common_330371063_324976703 {
using namespace common;

class CollisionUtils {
public:
    /**
     * @brief Checks if a drone at the given position with the given radius 
     *        collides with any occupied voxels in the map.
     * 
     * @param map The map to check against
     * @param pos The center position of the drone
     * @param radius The collision radius of the drone
     * @return true if collision detected (intersects with Occupied voxel), false otherwise
     */
    static bool isDroneColliding(const IMap3D& map, const Position3D& pos, PhysicalLength radius);

    /**
     * @brief Checks if a drone's entire bounding box is within the map's boundaries.
     * 
     * @param map The map to check against
     * @param pos The center position of the drone
     * @param radius The collision radius of the drone
     * @return true if the drone is fully inside the map boundaries, false otherwise
     */
    static bool isDroneFullyInBounds(const IMap3D& map, const Position3D& pos, PhysicalLength radius);

    /**
     * @brief Iterates over every sample point inside a sphere defined by the drone's position and radius.
     *        The step size is calculated based on the given resolution.
     * 
     * @param center The center position of the sphere
     * @param radius The radius of the sphere
     * @param resolution The resolution of the map (used to compute step size)
     * @param callback A function taking `const Position3D&` and returning a `bool`.
     *                 Return `false` from the callback to abort iteration early, `true` to continue.
     */
    template <typename Func>
    static void forEachVoxelInDroneSphere(const Position3D& center, 
                                          PhysicalLength radius, 
                                          PhysicalLength resolution, 
                                          Func callback) {
        double droneRadiusCm = radius.numerical_value_in(cm);
        double resolutionCm = resolution.numerical_value_in(cm);
        double stepSizeCm = std::max(1.0, resolutionCm / 2.0);

        for (double deltaXCm = -droneRadiusCm; deltaXCm <= droneRadiusCm; deltaXCm += stepSizeCm) {
            for (double deltaYCm = -droneRadiusCm; deltaYCm <= droneRadiusCm; deltaYCm += stepSizeCm) {
                for (double deltaZCm = -droneRadiusCm; deltaZCm <= droneRadiusCm; deltaZCm += stepSizeCm) {
                    if ((deltaXCm * deltaXCm + deltaYCm * deltaYCm + deltaZCm * deltaZCm) > (droneRadiusCm * droneRadiusCm)) continue;
                    Position3D voxelPosition{
                        center.x + deltaXCm * x_extent[cm],
                        center.y + deltaYCm * y_extent[cm],
                        center.z + deltaZCm * z_extent[cm]
                    };
                    
                    if (!callback(voxelPosition)) {
                        return;
                    }
                }
            }
        }
    }
};

} // namespace user_common_330371063_324976703
