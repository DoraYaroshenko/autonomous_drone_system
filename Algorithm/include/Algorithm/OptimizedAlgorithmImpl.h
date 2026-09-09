#pragma once

#include <Common/IMappingAlgorithm.h>
#include <memory>

namespace algorithm_330371063_324976703 {
using namespace common;

struct SweepState;

//final means no class can inherit from it
class OptimizedAlgorithmImpl_330371063_324976703 final : public IMappingAlgorithm {
public:
    //inherits constructor from the base class
    using IMappingAlgorithm::IMappingAlgorithm;

    //override tells the compiler we intend to override a virtual method from the base class, if not it will give an error. So if we make a typo in the signature, we don't accidentally create a new method 
    ~OptimizedAlgorithmImpl_330371063_324976703() override;
    [[nodiscard]] common::types::MappingStepCommand nextStep(const common::types::DroneState& state,
                                                     const common::types::LidarScanResult* latest_scan) override;
private:
    std::unique_ptr<SweepState> sweep_state_;
};

} // namespace algorithm_330371063_324976703
