#pragma once

#include <Common/MissionControlFactory.h>
#include <vector>
#include <mutex>
#include <unordered_map>
#include <thread>

namespace simulator {

class MissionControlRegistrar {
public:
    static MissionControlRegistrar& getInstance() {
        static MissionControlRegistrar instance;
        return instance;
    }

    void registerFactory(common::MissionControlFactory factory) {
        std::lock_guard<std::mutex> lock(mtx_);
        factories_[std::this_thread::get_id()].push_back(std::move(factory));
    }

    std::vector<common::MissionControlFactory> getFactories() {
        std::lock_guard<std::mutex> lock(mtx_);
        return factories_[std::this_thread::get_id()];
    }

    void clear() {
        std::lock_guard<std::mutex> lock(mtx_);
        factories_.erase(std::this_thread::get_id());
    }

private:
    MissionControlRegistrar() = default;
    ~MissionControlRegistrar() = default;
    MissionControlRegistrar(const MissionControlRegistrar&) = delete;
    MissionControlRegistrar& operator=(const MissionControlRegistrar&) = delete;

    std::mutex mtx_;
    std::unordered_map<std::thread::id, std::vector<common::MissionControlFactory>> factories_;
};

} // namespace simulator
