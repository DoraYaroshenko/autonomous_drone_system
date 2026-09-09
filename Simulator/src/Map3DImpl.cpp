#include <Simulator/Map3DImpl.h>
#include <UserCommon/Logger.h>

#include <cmath>
#include <fstream>
#include <iostream>
#include <stdexcept>
#include <utility>




namespace simulator {
namespace types {
using namespace common::types;
using namespace simulator::types;
}
using namespace common;
namespace user_common_330371063_324976703 {}
using namespace user_common_330371063_324976703;

namespace {

// Convert a world-space position to linear index in the NPY array.
// NPY layout: [X, Y, Z] in row-major order (shape[0]=X, shape[1]=Y, shape[2]=Z).
// Returns (size_t)-1 if out of bounds.
std::size_t posToLinear(const Position3D& pos,
                        const types::MapConfig& config,
                        const std::vector<std::size_t>& shape,
                        bool colMajor) {
    double res = config.resolution.force_numerical_value_in(cm);
    if (res <= 0.0) {
        return static_cast<std::size_t>(-1);
    }

    // World position to array index: subtract offset, divide by resolution.
    double fx = (pos.x - config.offset.x).numerical_value_in(cm) / res;
    double fy = (pos.y - config.offset.y).numerical_value_in(cm) / res;
    double fz = (pos.z - config.offset.z).numerical_value_in(cm) / res;

    auto ix = static_cast<int>(std::floor(fx));
    auto iy = static_cast<int>(std::floor(fy));
    auto iz = static_cast<int>(std::floor(fz));

    auto sx = static_cast<std::size_t>(shape[0]);
    auto sy = static_cast<std::size_t>(shape[1]);
    auto sz = static_cast<std::size_t>(shape[2]);

    if (ix < 0 || static_cast<std::size_t>(ix) >= sx || 
        iy < 0 || static_cast<std::size_t>(iy) >= sy || 
        iz < 0 || static_cast<std::size_t>(iz) >= sz) {
        return static_cast<std::size_t>(-1);
    }

    if (colMajor) {
        return static_cast<std::size_t>(ix)
             + static_cast<std::size_t>(iy) * sx
             + static_cast<std::size_t>(iz) * sx * sy;
    } else {
        return static_cast<std::size_t>(ix) * sy * sz
             + static_cast<std::size_t>(iy) * sz
             + static_cast<std::size_t>(iz);
    }
}

} // namespace

Map3DImpl::Map3DImpl(std::shared_ptr<NpyArray> map_ptr)
    : Map3DImpl(std::move(map_ptr), types::MapConfig{}) {}

Map3DImpl::Map3DImpl(std::shared_ptr<NpyArray> map_ptr, const types::MapConfig map_config)
    : map_(std::move(map_ptr)),
      config_(map_config) {
    if (!map_) {
        throw std::invalid_argument("Map3DImpl requires a valid map pointer.");
    }
}

types::VoxelOccupancy Map3DImpl::atVoxel(const Position3D& pos) const {
    const auto& shape = map_->Shape();
    if (shape.size() != 3) {
        return types::VoxelOccupancy::OutOfBounds;
    }

    std::size_t idx = posToLinear(pos, config_, shape, map_->ColMajor());
    if (idx == static_cast<std::size_t>(-1)) {
        return types::VoxelOccupancy::OutOfBounds;
    }

    // Read the value from the NPY array (supports int8, uint8, int32, and char).
    int raw_value = 0;
    if (map_->ValueType() == typeid(std::int8_t)) {
        raw_value = static_cast<int>(map_->Data<std::int8_t>()[idx]);
    } else if (map_->ValueType() == typeid(std::uint8_t)) {
        raw_value = static_cast<int>(map_->Data<std::uint8_t>()[idx]);
    } else if (map_->ValueType() == typeid(char)) {
        raw_value = static_cast<int>(map_->Data<char>()[idx]);
    } else if (map_->ValueType() == typeid(int)) {
        raw_value = map_->Data<int>()[idx];
    } else {
        std::cerr << "UNKNOWN VALUETYPE: " << map_->ValueType().name() << "\n";
        return types::VoxelOccupancy::OutOfBounds;
    }

    if (raw_value > 0) {
        return types::VoxelOccupancy::Occupied;
    }
    return static_cast<types::VoxelOccupancy>(raw_value);
}

types::MapConfig Map3DImpl::getMapConfig() const {
    return config_;
}

bool Map3DImpl::isInBounds(const Position3D& pos) const {
    const auto& shape = map_->Shape();
    if (shape.size() != 3) {
        return false;
    }

    // Check against configured boundaries if they are set (non-zero range).
    const auto& bounds = config_.boundaries;
    bool has_bounds = (bounds.max_x > bounds.min_x) || (bounds.max_y > bounds.min_y) || (bounds.max_height > bounds.min_height);
    if (has_bounds) {
        if (pos.x < bounds.min_x || pos.x >= bounds.max_x ||
            pos.y < bounds.min_y || pos.y >= bounds.max_y ||
            pos.z < bounds.min_height || pos.z >= bounds.max_height) {
            return false;
        }
    }

    // Also check against array dimensions.
    std::size_t idx = posToLinear(pos, config_, shape, map_->ColMajor());
    return idx != static_cast<std::size_t>(-1);
}

void Map3DImpl::set(const Position3D& pos, types::VoxelOccupancy value) { //modifies the map in memory during the simulation
    const auto& shape = map_->Shape();
    if (shape.size() != 3) {
        return;
    }

    std::size_t idx = posToLinear(pos, config_, shape, map_->ColMajor());
    if (idx == static_cast<std::size_t>(-1)) {
        return;
    }

    int raw_value = static_cast<int>(value);

    // Calculate grid coordinates for logging
    int ix, iy, iz;
    if (map_->ColMajor()) {
        std::size_t sx = shape[0];
        std::size_t sy = shape[1];
        ix = static_cast<int>(idx % sx);
        iy = static_cast<int>((idx / sx) % sy);
        iz = static_cast<int>(idx / (sx * sy));
    } else {
        std::size_t sz = shape[2];
        std::size_t sy = shape[1];
        iz = static_cast<int>(idx % sz);
        iy = static_cast<int>((idx / sz) % sy);
        ix = static_cast<int>(idx / (sy * sz));
    }

    auto logIfChanged = [ix, iy, iz, value, raw_value](int current_val) {
        if (current_val != raw_value) {
            ::user_common_330371063_324976703::Logger::logVoxel(ix, iy, iz, value);
        }
    };

    // Write the value to the NPY array.
    if (map_->ValueType() == typeid(std::int8_t)) {
        auto* ptr = const_cast<std::int8_t*>(map_->Data<std::int8_t>());
        logIfChanged(ptr[idx]);
        ptr[idx] = static_cast<std::int8_t>(raw_value);
    } else if (map_->ValueType() == typeid(std::uint8_t)) {
        auto* ptr = const_cast<std::uint8_t*>(map_->Data<std::uint8_t>());
        logIfChanged(ptr[idx]);
        ptr[idx] = static_cast<std::uint8_t>(raw_value);
    } else if (map_->ValueType() == typeid(char)) {
        auto* ptr = const_cast<char*>(map_->Data<char>());
        logIfChanged(ptr[idx]);
        ptr[idx] = static_cast<char>(raw_value);
    } else if (map_->ValueType() == typeid(int)) {
        auto* ptr = const_cast<int*>(map_->Data<int>());
        logIfChanged(ptr[idx]);
        ptr[idx] = raw_value;
    }
}

void Map3DImpl::save(const std::filesystem::path& path) const { //saves the map to disk
    const auto& shape = map_->Shape();
    if (shape.size() != 3) {
        throw std::runtime_error("Cannot save map: invalid shape.");
    }

    std::size_t total = shape[0] * shape[1] * shape[2];
    std::vector<char> data(total);
    if (map_->ValueType() == typeid(char)) {
        const auto* src = map_->Data<char>();
        std::copy(src, src + total, data.begin());
    } else if (map_->ValueType() == typeid(std::int8_t)) {
        const auto* src = map_->Data<std::int8_t>();
        for (std::size_t i = 0; i < total; ++i) {
            data[i] = static_cast<char>(src[i]);
        }
    } else if (map_->ValueType() == typeid(std::uint8_t)) {
        const auto* src = map_->Data<std::uint8_t>();
        for (std::size_t i = 0; i < total; ++i) {
            data[i] = static_cast<char>(src[i]);
        }
    } else if (map_->ValueType() == typeid(int)) {
        const auto* src = map_->Data<int>();
        for (std::size_t i = 0; i < total; ++i) {
            data[i] = static_cast<char>(src[i]);
        }
    }

    std::vector<std::size_t> save_shape = {shape[0], shape[1], shape[2]};
    const char* err = NpyArray::SaveNPY<char>(path.string(), data, save_shape, false);
    if (err != nullptr) {
        throw std::runtime_error(std::string("Failed to save NPY file: ") + err);
    }
}




} // namespace simulator
