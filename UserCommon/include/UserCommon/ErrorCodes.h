#pragma once

#include <string_view>

namespace user_common_330371063_324976703 {
namespace ErrorCodes {

constexpr std::string_view DRONE_STEP_ERROR = "DRONE_STEP_ERROR"; //pointer to start of chracter array and size, doesn't allocate memory
constexpr std::string_view OUT_OF_BOUNDS = "ILLEGAL_MOVEMENT_OUT_OF_BOUNDS";
constexpr std::string_view DRONE_EXCEPTION = "DRONE_EXCEPTION";
constexpr std::string_view MAP_SAVE_ERROR = "MAP_SAVE_ERROR";
constexpr std::string_view MISSION_EXCEPTION = "MISSION_EXCEPTION";
constexpr std::string_view SIMULATION_RUN_ERROR = "SIMULATION_RUN_ERROR";

} // namespace ErrorCodes
} // namespace user_common_330371063_324976703
