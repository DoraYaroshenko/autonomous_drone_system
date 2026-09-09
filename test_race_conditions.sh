#!/bin/bash

# test_race_conditions.sh
# Tests logical thread safety by comparing the outputs of a single-threaded run
# against a multi-threaded run. If thread-safety is handled correctly, the outputs
# should be 100% identical.

set -e

# Default simulation file if not provided
SIM_FILE=${1:-inputs/simulation.yaml}
ALGO_SRC="build/default/Algorithm"
MC_SO="build/default/MissionControl/MissionControl_330371063_324976703.so"
SIM_EXE="./build/default/Simulator/simulator_330371063_324976703"

TEST_DIR="test_algos_race"
rm -rf $TEST_DIR
mkdir -p $TEST_DIR

# Use the pre-built algorithms
cp $ALGO_SRC/Algorithm_*.so $TEST_DIR/

echo "============================================================"
echo "Phase 2: Deterministic Stress Testing"
echo "Simulation: $SIM_FILE"
echo "============================================================"

# Helper function to find the most recent output directory
get_latest_output_dir() {
    ls -td $TEST_DIR/competition_* | head -1
}

echo "[1/3] Running Golden Baseline (1 Thread)..."
$SIM_EXE -competition simulation=$SIM_FILE mission_control=$MC_SO algorithms_folder=$TEST_DIR num_threads=1 > /dev/null
BASELINE_DIR=$(get_latest_output_dir)
# Rename it so the next run doesn't get confused
mv $BASELINE_DIR $TEST_DIR/baseline_output
BASELINE_DIR="$TEST_DIR/baseline_output"

echo "[2/3] Running Stress Test (50 Threads)..."
# Using 50 threads to guarantee massive context switching and race potential
$SIM_EXE -competition simulation=$SIM_FILE mission_control=$MC_SO algorithms_folder=$TEST_DIR num_threads=50 > /dev/null
STRESS_DIR=$(get_latest_output_dir)

echo "[3/3] Comparing Outputs..."
echo "Diffing $BASELINE_DIR against $STRESS_DIR"
echo "------------------------------------------------------------"

# We exclude the generated folder name timestamp from diff if possible, 
# but since diff compares the contents directly when passing two directories, it's fine.
# We just need to diff the contents.

if diff -r -u -I "^Time" -I "^time" -I "time_str" -I "Timestamp" -I "[0-9][0-9][0-9][0-9]-[0-9][0-9]-[0-9][0-9]" "$BASELINE_DIR" "$STRESS_DIR" > /tmp/race_diff.txt; then
    echo "SUCCESS: No logical race conditions detected! The outputs are perfectly deterministic."
    exit_code=0
else
    echo "FAILURE: The outputs differ! There is a logical race condition (or timestamp mismatch)."
    echo "Here is the summary of the differences:"
    echo "------------------------------------------------------------"
    cat /tmp/race_diff.txt | head -n 30
    echo "------------------------------------------------------------"
    echo "Outputs preserved in $TEST_DIR for manual inspection."
    exit_code=1
fi

echo "============================================================"
# Clean up only if successful
if [ $exit_code -eq 0 ]; then
    rm -rf $TEST_DIR
fi

exit $exit_code
