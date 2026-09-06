#include <UserCommon/OrientationUtils.h>
#include <mp-units/systems/si/math.h>

namespace user_common_330371063_324976703 {

using namespace common;

Direction3D OrientationUtils::getBeamDirection(const common::Orientation& orientation) {
    const auto cos_altitude = si::cos(orientation.altitude);
    const auto dx = cos_altitude * si::cos(orientation.horizontal);
    const auto dy = cos_altitude * si::sin(orientation.horizontal);
    const auto dz = si::sin(orientation.altitude);

    return Direction3D{
        dx.force_numerical_value_in(mp::one),
        dy.force_numerical_value_in(mp::one),
        dz.force_numerical_value_in(mp::one)
    };
}

common::Position3D OrientationUtils::pointAlongBeam(const common::Position3D& origin, 
                                                    const Direction3D& dir, 
                                                    common::PhysicalLength distance) {
    const double distance_cm = distance.force_numerical_value_in(cm);
    return common::Position3D{
        origin.x + XLength(dir.dx * distance_cm * x_extent[cm]),
        origin.y + YLength(dir.dy * distance_cm * y_extent[cm]),
        origin.z + ZLength(dir.dz * distance_cm * z_extent[cm])
    };
}

common::Orientation OrientationUtils::addOrientations(const common::Orientation& a, 
                                                      const common::Orientation& b) {
    auto normalize_angle = [](double angle) {
        double normalized = std::fmod(angle, 360.0);
        if (normalized < 0.0) {
            normalized += 360.0;
        }
        return normalized;
    };

    return common::Orientation{
        HorizontalAngle{normalize_angle((a.horizontal + b.horizontal).numerical_value_in(deg)) * deg},
        AltitudeAngle{normalize_angle((a.altitude + b.altitude).numerical_value_in(deg)) * deg}
    };
}

} // namespace user_common_330371063_324976703
