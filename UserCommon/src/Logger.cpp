#include <UserCommon/Logger.h>
#include <iostream>


namespace user_common_330371063_324976703 {
using namespace common;



thread_local std::ofstream Logger::out_stream_;
thread_local std::size_t Logger::current_step_id_ = 0;

void Logger::init(const std::string& path) {
    if (out_stream_.is_open()) {
        out_stream_.close();
    }
    out_stream_.open(path, std::ios::out | std::ios::trunc);
    if (!out_stream_) {
        std::cerr << "Logger error: Failed to open " << path << std::endl;
    }
    current_step_id_ = 0;
}

void Logger::close() {
    if (out_stream_.is_open()) {
        out_stream_.close();
    }
}

void Logger::setStep(std::size_t step_id) {
    current_step_id_ = step_id;
}

void Logger::logMovement(double x_cm, double y_cm, double z_cm, 
                         double h_angle_deg, double v_angle_deg, 
                         const std::string& action) {
    if (!out_stream_.is_open()) return;

    out_stream_ << "{\"type\": \"movement\", \"step_id\": " << current_step_id_ 
                << ", \"x_cm\": " << x_cm << ", \"y_cm\": " << y_cm << ", \"z_cm\": " << z_cm
                << ", \"h_angle_deg\": " << h_angle_deg << ", \"v_angle_deg\": " << v_angle_deg
                << ", \"action\": \"" << action << "\"}\n";
}

void Logger::logScan(double relative_h_angle_deg, double relative_v_angle_deg) {
    if (!out_stream_.is_open()) return;

    out_stream_ << "{\"type\": \"scan\", \"step_id\": " << current_step_id_ 
                << ", \"relative_h_angle_deg\": " << relative_h_angle_deg 
                << ", \"relative_v_angle_deg\": " << relative_v_angle_deg << "}\n";
}

void Logger::logVoxel(int x, int y, int z, common::types::VoxelOccupancy occupancy) {
    if (!out_stream_.is_open()) return;

    out_stream_ << "{\"type\": \"voxel\", \"step_id\": " << current_step_id_ 
                << ", \"x\": " << x << ", \"y\": " << y << ", \"z\": " << z 
                << ", \"value\": " << static_cast<int>(occupancy) << "}\n";
}



} // namespace user_common_330371063_324976703
