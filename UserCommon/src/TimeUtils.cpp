#include <UserCommon/TimeUtils.h>
#include <chrono>
#include <ctime>
#include <iomanip>
#include <sstream>

namespace user_common_330371063_324976703 {

std::string TimeUtils::generate_iso_timestamp() {
    auto now = std::chrono::system_clock::now();
    auto time_t_now = std::chrono::system_clock::to_time_t(now);
    std::tm utc_tm{};
#if defined(_MSC_VER)
    gmtime_s(&utc_tm, &time_t_now);
#else
    gmtime_r(&time_t_now, &utc_tm);
#endif
    std::ostringstream time_ss;
    time_ss << std::put_time(&utc_tm, "%Y-%m-%dT%H:%M:%SZ");
    return time_ss.str();
}

std::string TimeUtils::generate_folder_timestamp() {
    auto now = std::chrono::system_clock::now();
    auto time_t_now = std::chrono::system_clock::to_time_t(now);
    auto ts = std::chrono::duration_cast<std::chrono::nanoseconds>(now.time_since_epoch()).count();
    constexpr size_t NUM_DIGITS = 9;
    size_t NUM_DIGITS_P = 1000000000; // 10^9

    std::tm utc_tm{};
#if defined(_MSC_VER)
    gmtime_s(&utc_tm, &time_t_now);
#else
    gmtime_r(&time_t_now, &utc_tm);
#endif
    std::ostringstream time_ss;
    time_ss << std::put_time(&utc_tm, "%Y%m%d%H%M%S") 
            << std::setfill('0') << std::setw(NUM_DIGITS) << (ts % NUM_DIGITS_P);
    return time_ss.str();
}

} // namespace user_common_330371063_324976703
