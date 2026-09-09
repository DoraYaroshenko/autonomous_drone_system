#pragma once

#include <string>
#include <fstream>
#include <mutex>
#include <Common/Types.h>


namespace user_common_330371063_324976703 {
using namespace common;



/**
 * @brief A lightweight streaming logger for simulation visualizations.
 * Outputs to a JSON Lines (.jsonl) file directly to avoid large memory overhead.
 */
class Logger {
public:
    static void init(const std::string& path);
    static void close();
    static void setStep(std::size_t step_id); //sets current_step_id_
    
    static void logMovement(double x_cm, double y_cm, double z_cm, 
                            double h_angle_deg, double v_angle_deg, 
                            const std::string& action);
                            
    static void logScan(double relative_h_angle_deg, double relative_v_angle_deg);
    
    static void logVoxel(int x, int y, int z, common::types::VoxelOccupancy occupancy);

private:
    static thread_local std::ofstream out_stream_; //output file stream
    static thread_local std::size_t current_step_id_;
};

//atomic - for primitive typesone shared variable. thread_local - each thread gets a copy, for big objects.

} // namespace user_common_330371063_324976703
