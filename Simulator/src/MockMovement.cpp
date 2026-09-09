#include <Simulator/MockMovement.h>

#include <mp-units/systems/si/math.h>

#include <cmath>
#include <stdexcept>
#include <algorithm>
#include <UserCommon/CollisionUtils.h>

namespace {
using namespace common;

void checkMoveSafety(const IMap3D& map, const Position3D& pos, PhysicalLength radius) {
    if (!user_common_330371063_324976703::CollisionUtils::isDroneFullyInBounds(map, pos, radius)) {
        throw std::runtime_error("Collision detected: Drone went out of bounds");
    }
    
    PhysicalLength res = map.getMapConfig().resolution;

    bool collided = false;
    user_common_330371063_324976703::CollisionUtils::forEachVoxelInDroneSphere(pos, radius, res, [&](const Position3D& p) {
        if (map.isInBounds(p)) {
            auto v = map.atVoxel(p);
            if (v == common::types::VoxelOccupancy::Occupied || 
                v == common::types::VoxelOccupancy::PotentiallyOccupied) {
                collided = true;
                return false; // Stop iteration
            }
        }
        return true; // Continue iteration
    });

    if (collided) {
        throw std::runtime_error("Collision detected: Drone hit a wall");
    }
}
} // namespace




namespace simulator {
namespace types {
using namespace common::types;
using namespace simulator::types;
}
using namespace common;
namespace user_common_330371063_324976703 {}
using namespace user_common_330371063_324976703;

MockMovement::MockMovement(MockGPS& gps, const common::IMap3D& hidden_map, common::PhysicalLength drone_radius) 
    : gps_(gps), hidden_map_(hidden_map), drone_radius_(drone_radius) {}

types::MovementResult MockMovement::rotate(types::RotationDirection direction, HorizontalAngle angle) {
    const Orientation current = gps_.heading();
    // Left = positive (counter-clockwise), Right = negative (clockwise).
    const HorizontalAngle signed_angle =
        (direction == types::RotationDirection::Left) ? angle : -angle;
    gps_.setHeading(Orientation{current.horizontal + signed_angle, current.altitude});
    return types::MovementResult{true, {}};
}

types::MovementResult MockMovement::advance(PhysicalLength distance) {
    // Port from ex1's MockMovementDriver::advance.
    // Moves the drone in the direction it is currently facing (horizontal plane).
    const Position3D position = gps_.position();
    const Orientation orientation = gps_.heading();

    auto dx = si::cos(orientation.horizontal);
    auto dy = si::sin(orientation.horizontal);

    XLength new_x = position.x + XLength((distance * dx).numerical_value_in(cm) * x_extent[cm]);
    YLength new_y = position.y + YLength((distance * dy).numerical_value_in(cm) * y_extent[cm]);
    Position3D new_pos{new_x, new_y, position.z};

    checkMoveSafety(hidden_map_, new_pos, drone_radius_);
    gps_.setPosition(new_pos);
    return types::MovementResult{true, {}};
}

types::MovementResult MockMovement::elevate(PhysicalLength distance) {
    // Port from ex1's MockMovementDriver::elevate.
    // Moves the drone vertically (z axis). Negative distance = descend.
    const Position3D position = gps_.position();
    ZLength new_z = position.z + ZLength(distance);
    Position3D new_pos{position.x, position.y, new_z};

    checkMoveSafety(hidden_map_, new_pos, drone_radius_);
    gps_.setPosition(new_pos);
    return types::MovementResult{true, {}};
}




} // namespace simulator
