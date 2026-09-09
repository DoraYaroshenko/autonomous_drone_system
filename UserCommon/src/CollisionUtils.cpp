#include <UserCommon/CollisionUtils.h>

#include <algorithm>


namespace user_common_330371063_324976703 {
using namespace common;



bool CollisionUtils::isDroneColliding(const IMap3D& map, const Position3D& pos, PhysicalLength radius) {
    // Quick check to make sure the BB center itself isn't occupied
    if (map.isInBounds(pos) && map.atVoxel(pos) == common::types::VoxelOccupancy::Occupied) {
        return true;
    }
    
    // We get the map resolution by retrieving its config.
    auto mapConfig = map.getMapConfig();
    PhysicalLength mapResolution = mapConfig.resolution;
    
    bool hasCollided = false;
    forEachVoxelInDroneSphere(pos, radius, mapResolution, [&](const Position3D& voxelPosition) {
        if (map.isInBounds(voxelPosition) && map.atVoxel(voxelPosition) == common::types::VoxelOccupancy::Occupied) {
            hasCollided = true;
            return false; // Stop iteration
        }
        return true; // Continue iteration
    });
    
    return hasCollided;
}

bool CollisionUtils::isDroneFullyInBounds(const IMap3D& map, const Position3D& pos, PhysicalLength radius) {
    auto bounds = map.getMapConfig().boundaries;
    
    double posXCm = pos.x.numerical_value_in(cm);
    double posYCm = pos.y.numerical_value_in(cm);
    double posZCm = pos.z.numerical_value_in(cm);
    double radiusCm = radius.numerical_value_in(cm);
    
    double minXCm = bounds.min_x.numerical_value_in(cm);
    double maxXCm = bounds.max_x.numerical_value_in(cm);
    double minYCm = bounds.min_y.numerical_value_in(cm);
    double maxYCm = bounds.max_y.numerical_value_in(cm);
    double minZCm = bounds.min_height.numerical_value_in(cm);
    double maxZCm = bounds.max_height.numerical_value_in(cm);

    //evaluated at compilation time
    constexpr double EPSILON = 1e-4;

    bool isWithinBounds = (posXCm - radiusCm >= minXCm - EPSILON && posXCm + radiusCm <= maxXCm + EPSILON &&
                           posYCm - radiusCm >= minYCm - EPSILON && posYCm + radiusCm <= maxYCm + EPSILON &&
                           posZCm - radiusCm >= minZCm - EPSILON && posZCm + radiusCm <= maxZCm + EPSILON);

    return isWithinBounds;
}



} // namespace user_common_330371063_324976703
