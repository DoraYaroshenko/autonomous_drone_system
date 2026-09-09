#pragma once

#include <Common/MappingAlgorithmFactory.h>
#include <vector>
#include <string>
#include <mutex>
#include <unordered_map>
#include <thread>

namespace simulator {

class MappingAlgorithmRegistrar {
public:
    static MappingAlgorithmRegistrar& getInstance() {
        static MappingAlgorithmRegistrar instance;
        return instance;
    }

    void registerFactory(common::MappingAlgorithmFactory factory) {
        std::lock_guard<std::mutex> lock(mtx_);
        factories_[std::this_thread::get_id()].push_back(std::move(factory));
    }

    std::vector<common::MappingAlgorithmFactory> getFactories() {
        std::lock_guard<std::mutex> lock(mtx_);
        return factories_[std::this_thread::get_id()];
    }

    void clear() {
        std::lock_guard<std::mutex> lock(mtx_);
        factories_.erase(std::this_thread::get_id());
    }

private:
    MappingAlgorithmRegistrar() = default;
    ~MappingAlgorithmRegistrar() = default;
    MappingAlgorithmRegistrar(const MappingAlgorithmRegistrar&) = delete;
    MappingAlgorithmRegistrar& operator=(const MappingAlgorithmRegistrar&) = delete;

    std::mutex mtx_;
    std::unordered_map<std::thread::id, std::vector<common::MappingAlgorithmFactory>> factories_; //in registrar the key is thread_id and the value is the factories of the libraries it loaded
};

} // namespace simulator
