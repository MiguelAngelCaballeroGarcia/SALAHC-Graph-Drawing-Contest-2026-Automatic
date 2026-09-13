# SALAHC - Graph Drawing Contest 2026 Live Challenge (Automatic Category)

A C++20 graph-drawing optimizer developed for the **Graph Drawing Contest 2026 Live Challenge, Automatic Category**.

The team **SALAHC** finished **6th** in the competition. This repository contains the contest implementation, its tests, benchmark utilities, representative instances, and the analysis material used during development.

## Project Context

The project was developed by **Miguel Ángel Caballero García** while enrolled in the Double BSc in Mathematics and Computer Science at the **University of Barcelona**, during a full-year Erasmus at **TUM**. The project was introduced through **Johannes Zink**, whose connection to the graph-drawing community led to the collaboration.

This repository is preserved as a finished contest project and portfolio artifact. The algorithm is not presented as a general-purpose graph-drawing library: it is the implementation submitted and tuned for the 2026 challenge.

## Approach

The optimizer works on straight-line drawings of graphs and uses a staged, time-budgeted pipeline:

1. Parse each input graph and initialize its geometry.
2. Apply a force-directed layout to improve the initial drawing.
3. Optimize the drawing with Late Acceptance Hill Climbing (LAHC).
4. Apply Simulated Annealing (SA) for further exploration.
5. Legalize hard geometric constraints before writing the result.
6. Evaluate crossings with incremental updates and an LBVH broad phase.

The objective is lexicographic and prioritizes the contest's crossing-related quality measures. The executable can process one or more input files concurrently and allocates worker threads across graphs.

## Repository Layout

- `src/`: production C++ implementation.
- `tests/`: GoogleTest unit and integration tests.
- `data/`: optional local input and output directory; contest instances are intentionally not included in this public repository.
- `bench-ab/`: focused A/B benchmark harness and optimized geometry experiments.
- `scripts/`: PowerShell scripts for telemetry, sweeps, and result analysis.
- `CMakeLists.txt`, `CMakePresets.json`: portable build configuration.

Generated build directories, executables, telemetry output, and local benchmark artifacts are intentionally ignored by Git.

## Requirements

- CMake 3.20 or newer.
- A C++20 compiler.
- AVX2 support at runtime and compiler support for AVX2 instructions.
- PowerShell is only required for the optional benchmark and analysis scripts.
- GoogleTest is downloaded automatically by CMake when it is not already available through a CMake package.

## Build and Test

From the repository root:

```powershell
cmake -S . -B build-release -DCMAKE_BUILD_TYPE=Release
cmake --build build-release --config Release --parallel
ctest --test-dir build-release -C Release --output-on-failure
```

On a multi-configuration generator such as Visual Studio, `--config Release` selects the Release binaries. On a single-configuration generator, `CMAKE_BUILD_TYPE=Release` selects the configuration.

The main executable is written to the generator-specific Release output directory, for example:

```text
build-release/Release/SALAHC.exe
```

## Running the Optimizer

If you provide local contest or test instances under `data/`, the executable processes all `.json` and `.txt` files directly under that directory and writes matching output files under `data/solutions/`:

```powershell
build-release/Release/SALAHC.exe
```

Specific input files can be supplied explicitly:

```powershell
build-release/Release/GDContestAI.exe data/Automatic-10.txt
```

Useful environment variables:

| Variable | Purpose |
| --- | --- |
| `GDCONTESTAI_THREADS` | Maximum worker threads to use. |
| `GDCONTESTAI_TIME_LIMIT_MS` | Override the global time limit in milliseconds. |
| `GDCONTESTAI_NO_TIME_LIMIT` | Set to `1` to disable the global time limit. |
| `GDCONTESTAI_SEED` | Set the base random seed for reproducible runs. |
| `GDCONTESTAI_DISABLE_FINAL_POLISH` | Set to `1` to disable final polishing. |
| `GDCONTESTAI_EXPERIMENTAL_ESCAPE` | Set to `1` to enable the experimental escape behavior. |

The optimizer writes solutions using the input filename. Files under `data/solutions/` may be overwritten. The public repository intentionally omits contest instances and generated solutions; supply authorized local inputs before running it without explicit arguments.

## Reproducibility

For comparable runs, use a Release build, set `GDCONTESTAI_THREADS` explicitly, set `GDCONTESTAI_SEED`, and record the compiler, CPU, input instance, time limit, and enabled pipeline options. Runtime and optimization results can vary with hardware, thread scheduling, and the available time budget.

## Attribution

- Team: SALAHC
- Contest: Graph Drawing Contest 2026 Live Challenge, Automatic Category
- Result: 6th place
- Author: Miguel Ángel Caballero García
- Academic context: Double BSc in Mathematics and Computer Science, University of Barcelona; full-year Erasmus at TUM
- Project supervision: Johannes Zink

## License and Contest Materials

The original implementation in this repository is released under the [MIT License](LICENSE).

Contest instances and any other third-party materials are intentionally excluded from this repository and remain subject to their original terms.
