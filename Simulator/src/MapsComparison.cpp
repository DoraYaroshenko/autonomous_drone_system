#include <Simulator/MapsComparison.h>

#include <cmath>
#include <iostream>




namespace simulator {
namespace types {
using namespace common::types;
using namespace simulator::types;
}
using namespace common;
namespace user_common_330371063_324976703 {}
using namespace user_common_330371063_324976703;

std::vector<double> MapsComparison::compare(const IMap3D& original,
                                             const std::vector<IMap3D*> targets) {
    std::vector<double> scores;

    const types::MapConfig orig_config = original.getMapConfig();
    const auto& orig_bounds = orig_config.boundaries;
    PhysicalLength res = orig_config.resolution;

    if (res <= 0.0 * cm) {
        // Invalid resolution — return -1 for all targets.
        for (std::size_t i = 0; i < targets.size(); ++i) {
            scores.push_back(-1.0);
        }
        return scores;
    }
    ZLength res_z(res.numerical_value_in(cm) * z_extent[cm]);
    YLength res_y(res.numerical_value_in(cm) * y_extent[cm]);
    XLength res_x(res.numerical_value_in(cm) * x_extent[cm]);
    
    auto get_voxel_count = [](auto min_val, auto max_val, auto res_axis) { //counts the number of voxels along one axis
        double diff_cm = (max_val - min_val).numerical_value_in(cm);
        double res_cm = res_axis.numerical_value_in(cm);
        return std::round(diff_cm / res_cm);
    };

    double orig_nx = get_voxel_count(orig_bounds.min_x, orig_bounds.max_x, res_x);
    double orig_ny = get_voxel_count(orig_bounds.min_y, orig_bounds.max_y, res_y);
    double orig_nz = get_voxel_count(orig_bounds.min_height, orig_bounds.max_height, res_z);

    for (const IMap3D* target : targets) {
        if (target == nullptr) {
            scores.push_back(-1.0);
            continue;
        }

        const types::MapConfig target_config = target->getMapConfig();
        const auto& target_bounds = target_config.boundaries;
        
        double target_nx = get_voxel_count(target_bounds.min_x, target_bounds.max_x, res_x);
        double target_ny = get_voxel_count(target_bounds.min_y, target_bounds.max_y, res_y);
        double target_nz = get_voxel_count(target_bounds.min_height, target_bounds.max_height, res_z);

        const double epsilon = 1e-5;
        if (std::abs(orig_nx - target_nx) > epsilon ||
            std::abs(orig_ny - target_ny) > epsilon ||
            std::abs(orig_nz - target_nz) > epsilon) {
            scores.push_back(-1.0);
            continue;
        }

        double score = 0.0;
        double max_score = 0.0;
        bool error_occurred = false;

        for (int z_idx = 0; z_idx < static_cast<int>(orig_nz); ++z_idx) {
            for (int y_idx = 0; y_idx < static_cast<int>(orig_ny); ++y_idx) {
                for (int x_idx = 0; x_idx < static_cast<int>(orig_nx); ++x_idx) {
                    
                    Position3D orig_pos {
                        orig_bounds.min_x + res_x * (x_idx + 0.5),
                        orig_bounds.min_y + res_y * (y_idx + 0.5),
                        orig_bounds.min_height + res_z * (z_idx + 0.5)
                    };
                    
                    Position3D target_pos {
                        target_bounds.min_x + res_x * (x_idx + 0.5),
                        target_bounds.min_y + res_y * (y_idx + 0.5),
                        target_bounds.min_height + res_z * (z_idx + 0.5)
                    };

                    types::VoxelOccupancy orig_val = original.atVoxel(orig_pos);
                    types::VoxelOccupancy target_val = target->atVoxel(target_pos);
                    
                    if (orig_val == types::VoxelOccupancy::OutOfBounds || target_val == types::VoxelOccupancy::OutOfBounds) {
                        error_occurred = true;
                        break;
                    }

                    max_score += 1.0;

                    if (orig_val == target_val) {
                        // Exact match.
                        score += 1.0;
                    } else if (target_val == types::VoxelOccupancy::Unmapped) {
                        // Unmapped (-1) in the target = unreachable.
                        // Partial credit (same as ex1's -1 unreachable penalty).
                        score += 0.25;
                    } else if (target_val == types::VoxelOccupancy::PotentiallyOccupied) {
                        // PotentiallyOccupied (-3) = lidar sensed something too close.
                        // Give partial credit — better than a complete mismatch.
                        score += 0.25;
                    }
                    // else: occupied/free mismatch → score += 0.0
                }
                if (error_occurred) break;
            }
            if (error_occurred) break;
        }

        if (error_occurred) {
            scores.push_back(-1.0);
        } else if (max_score == 0.0) {
            scores.push_back(100.0); // No comparable voxels → perfect score.
        } else {
            scores.push_back((score / max_score) * 100.0); // Scale to 0-100.
        }
    }

    return scores;
}




} // namespace simulator
