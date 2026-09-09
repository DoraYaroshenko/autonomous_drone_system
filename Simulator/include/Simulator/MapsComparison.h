#pragma once

#include <Common/IMap3D.h>
#include <Common/Types.h>




namespace simulator {
namespace types {
using namespace common::types;
using namespace simulator::types;
}
using namespace common;
namespace user_common_330371063_324976703 {}
using namespace user_common_330371063_324976703;

class MapsComparison {
public:
    [[nodiscard]] static std::vector<double> compare(const IMap3D& origin,
                                                     const std::vector<IMap3D*> targets);
};




} // namespace simulator
