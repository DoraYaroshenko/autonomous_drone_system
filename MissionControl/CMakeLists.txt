add_library(MissionControl_330371063_324976703 SHARED 
    src/MissionControlImpl.cpp
    src/DroneControlImpl.cpp
    ../UserCommon/src/CollisionUtils.cpp
    ../UserCommon/src/ScanResultToVoxels.cpp
    ../UserCommon/src/Logger.cpp
    ../UserCommon/src/TimeUtils.cpp
    ../UserCommon/src/OrientationUtils.cpp
)
target_include_directories(MissionControl_330371063_324976703 PRIVATE include common_mission_control/include ../UserCommon/include)
target_link_libraries(MissionControl_330371063_324976703 common::common)
set_target_properties(MissionControl_330371063_324976703 PROPERTIES PREFIX "")