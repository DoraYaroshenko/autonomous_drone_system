#!/bin/bash

# test_exceptions.sh
# Tests that MockMovement exceptions (like collisions) are properly caught
# and isolated when multiple algorithms run in competition.

# Find the simulator executable
SIM=$(ls build/default/Simulator/simulator_* 2>/dev/null | head -n 1)
if [ -z "$SIM" ]; then
    echo "Simulator executable not found! Make sure you compiled the project (including Algorithm_Faulty)."
    exit 1
fi
SIM="./$SIM"

echo "Using simulator: $SIM"

# Create dummy test environment
TEST_DIR="test_env_exceptions"
rm -rf "$TEST_DIR"
mkdir -p "$TEST_DIR"

# # 1. Provide a simple simulation composition
# cat <<EOF > "$TEST_DIR/sim.yaml"
# SimulationConfig:
#   max_simulation_steps: 50

# MapConfig:
#   resolution: 5
#   x_extent: 100
#   y_extent: 100
#   z_extent: 100
#   drone_start_position:
#     x: 15
#     y: 15
#     z: 15

# DroneConfig:
#   radius: 1
#   max_rotate: 45
#   max_advance: 20
#   max_elevate: 20

# MissionConfig:
#   output_mapping_resolution_factor: 1

# LidarConfig:
#   max_range: 30
#   fov_horizontal: 120
#   fov_vertical: 60
#   horizontal_resolution: 5
#   vertical_resolution: 5
# EOF

# 2. Gather algorithms (we need the faulty one and the optimized one)
mkdir -p "$TEST_DIR/algos"
# Copy optimized algorithm (known good)
cp build/default/Algorithm/Algorithm_Optimized_*.so "$TEST_DIR/algos/" 2>/dev/null
# Copy faulty algorithm
cp build/default/Algorithm/Algorithm_Faulty_*.so "$TEST_DIR/algos/" 2>/dev/null

if [ -z "$(ls -A $TEST_DIR/algos/Algorithm_Faulty_*.so 2>/dev/null)" ]; then
    echo "Algorithm_Faulty shared library not found. Please compile it first."
    rm -rf "$TEST_DIR"
    exit 1
fi

if [ -z "$(ls -A $TEST_DIR/algos/Algorithm_Optimized_*.so 2>/dev/null)" ]; then
    echo "Algorithm_Optimized shared library not found. Using only Faulty."
fi

# 3. Get mission control
MC=$(ls build/default/MissionControl/MissionControl_*.so 2>/dev/null | head -n 1)
if [ -z "$MC" ]; then
    echo "MissionControl shared library not found. Please compile it first."
    rm -rf "$TEST_DIR"
    exit 1
fi

echo "Running competition test..."
echo "Command: $SIM -competition simulation=$TEST_DIR/sim.yaml mission_control=$MC algorithms_folder=$TEST_DIR/algos"
$SIM -competition simulation="./inputs/test_threads_sim.yaml" mission_control="$MC" algorithms_folder="$TEST_DIR/algos"
EXIT_CODE=$?

echo "Simulator Exit Code: $EXIT_CODE"

if [ $EXIT_CODE -ne 0 ]; then
    echo "FAIL: Simulator crashed! Exit code was $EXIT_CODE."
    exit 1
fi

echo "Simulator finished successfully. Checking results..."

# We expect output directories in the algos folder: competition_<time>
COMP_DIR=$(ls -d "$TEST_DIR/algos/competition_"* 2>/dev/null | head -n 1)
if [ -z "$COMP_DIR" ]; then
    echo "FAIL: Could not find competition output directory."
    exit 1
fi

# The global unified result YAML should exist
UNIFIED_YAML=$(ls "$COMP_DIR"/*yaml 2>/dev/null | head -n 1)
if [ ! -f "$UNIFIED_YAML" ]; then
    echo "FAIL: Unified YAML file not found in $COMP_DIR"
    exit 1
fi

echo "Unified Result YAML:"
cat "$UNIFIED_YAML"
echo "----------------------"

# We check if Faulty scored 0 (since YamlParserUtils ignores -1.0 scores and leaves total as 0)
if grep -q "total_score: 0" "$UNIFIED_YAML" && grep -q "Algorithm_Faulty" "$UNIFIED_YAML"; then
    echo "SUCCESS: Faulty algorithm received a score of 0."
else
    echo "FAIL: Faulty algorithm did not receive a score of 0 in the unified output."
fi

# We check if the error was logged in the Faulty algorithm's error_log.txt
FAULTY_ERR_LOG="$COMP_DIR/Algorithm_Faulty_330371063_324976703/run_0/error_log.txt"
if [ -f "$FAULTY_ERR_LOG" ]; then
    if grep -q "Collision detected:" "$FAULTY_ERR_LOG"; then
        echo "SUCCESS: 'Collision detected:' found in Faulty algorithm's error_log.txt."
    else
        echo "FAIL: 'Collision detected:' NOT found in Faulty algorithm's error_log.txt."
        cat "$FAULTY_ERR_LOG"
    fi
else
    echo "FAIL: Faulty algorithm error_log.txt not found at $FAULTY_ERR_LOG"
fi

# We check if Optimized algorithm completed without error log (if it exists)
OPT_ERR_LOG="$COMP_DIR/Algorithm_Optimized_330371063_324976703/run_0/error_log.txt"
if [ -d "$COMP_DIR/Algorithm_Optimized_330371063_324976703" ]; then
    if [ -f "$OPT_ERR_LOG" ]; then
        if grep -q "Collision detected:" "$OPT_ERR_LOG"; then
            echo "FAIL: Optimized algorithm unexpectedly threw a collision exception!"
            cat "$OPT_ERR_LOG"
        else
            echo "SUCCESS: Optimized algorithm log exists but no collision exception found."
        fi
    else
        echo "SUCCESS: Optimized algorithm ran cleanly with no error_log.txt."
    fi
fi

# Cleanup
echo "Cleaning up..."
rm -rf "$TEST_DIR"
echo "Test script finished."
