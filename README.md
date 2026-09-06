## File Tree (Overview)

```text
.
├── Algorithm/                 # Implementation of mapping algorithm
│   ├── CMakeLists.txt
│   ├── include/Algorithm/
│   └── src/
├── MissionControl/            # Implementation of the drone mission control logic
│   ├── CMakeLists.txt
│   ├── common_mission_control/
│   ├── include/MissionControl/
│   └── src/
├── Simulator/                 # Simulation engine, multi-threading, and mocked hardware
│   ├── CMakeLists.txt
│   ├── common_simulator/
│   ├── include/Simulator/
│   └── src/
├── common/                    # Shared interfaces across the core skeleton
│   ├── CMakeLists.txt
│   └── include/Common/
├── UserCommon/                # Student-defined common utilities (logging, collisions, voxels)
│   ├── include/UserCommon/
│   └── src/
├── CMakeLists.txt             # Root CMake config
├── CMakePresets.json
├── README.md
├── students.txt               # Student details
├── vcpkg-configuration.json   # vcpkg dependencies
└── vcpkg.json
```

## Build Instructions

You can build the entire project at once or build each component individually using CMake presets from the **root directory**.

Because the `CMakePresets.json` file is located at the root of the project (and configuring vcpkg correctly relies on it), you should run all commands from the root folder. To build individual components, you simply specify the `--target` flag instead of navigating into their subfolders.

**1. Configure the Project (Run once from root)**
```bash
cmake --preset default
```

**2. Build All Components**
```bash
cmake --build --preset default
```

**3. Build Simulator Only**
```bash
cmake --build --preset default --target simulator_330371063_324976703
```

**4. Build Algorithm Only**
```bash
cmake --build --preset default --target Algorithm_330371063_324976703
```

**5. Build Mission Control Only**
```bash
cmake --build --preset default --target MissionControl_330371063_324976703
```

## Running the Simulator
The simulator supports two modes, which dynamically load the `MissionControl` and `Algorithm` `.so` plugins at runtime:

**Comparative Run**
```bash
./build/default/Simulator/simulator_330371063_324976703 -comparative simulation=<sim.yaml> mission_control_folder=<folder> algorithm=<algo.so> [num_threads=<num>] [-verbose]
```

**Competition Run**
```bash
./build/default/Simulator/simulator_330371063_324976703 -competition simulation=<sim.yaml> mission_control=<mc.so> algorithms_folder=<folder> [num_threads=<num>] [-verbose]
```

The `-verbose` flag will cause the Mission Control to output drone movement log files named `<mission_details>_drone_logs.jsonl` under the results folder next to the output `.npy` files and `.error` logs. These can be used with our visualizations (submitted in previous assignments).
