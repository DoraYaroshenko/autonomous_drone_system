add_library(Algorithm_330371063_324976703 SHARED 
    src/OptimizedAlgorithmImpl.cpp
    ../UserCommon/src/CollisionUtils.cpp
    ../UserCommon/src/ScanResultToVoxels.cpp
    ../UserCommon/src/Logger.cpp
    ../UserCommon/src/TimeUtils.cpp
    ../UserCommon/src/OrientationUtils.cpp
)
target_include_directories(Algorithm_330371063_324976703 PRIVATE include ../UserCommon/include)
target_link_libraries(Algorithm_330371063_324976703 common::common)
set_target_properties(Algorithm_330371063_324976703 PROPERTIES PREFIX "")
