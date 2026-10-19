# Micrmouse Simulator

This folder contains a modular C++17 micromouse simulator that keeps the algorithm isolated behind a `MouseAPI` interface.

## Features

- Configurable dynamic maze size using `std::vector`
- File-based maze loading with support for multiple goal cells
- Default generated maze when no file is supplied
- Step-by-step simulator loop with move validation
- ASCII visualization of the maze and live mouse position
- Sample algorithms: right-hand rule, random walk, and flood-fill
- Extension-friendly maze source abstraction for future image-based loaders

## Build

```bash
make
```

This creates the `micrmouse_simulator` executable.

You can also build with CMake:

```bash
cmake -S . -B build
cmake --build build
```

## Run

Generated maze:

```bash
./micrmouse_simulator --width 8 --height 8 --start 0 0 --direction east --goal 7 7 --algorithm right-hand --max-steps 150
```

Maze from file:

```bash
./micrmouse_simulator --maze mazes/example.maze --algorithm right-hand --max-steps 200
```

Flood-fill on a loaded maze:

```bash
./micrmouse_simulator --maze mazes/example.maze --algorithm flood-fill --render final
```

Interactive stepping:

```bash
./micrmouse_simulator --maze mazes/example.maze --interactive
```

Quiet summary-only run:

```bash
./micrmouse_simulator --maze mazes/example.maze --render none
```

Deterministic random walk:

```bash
./micrmouse_simulator --width 8 --height 8 --algorithm random --seed 42 --render final
```

Generated maze with flood-fill:

```bash
./micrmouse_simulator --width 8 --height 8 --start 0 0 --direction east --goal 7 7 --algorithm flood-fill --render final
```

## Maze File Format

```text
WIDTH <width> <height>
START <x> <y>
GOAL <x> <y>
GOAL <x> <y>
WALL <x> <y> <N|E|S|W>
OPEN <x> <y> <N|E|S|W>
```

Notes:

- Use one `WIDTH` line first.
- Use at least one `GOAL` line.
- Outer boundary walls are created automatically.
- `WALL x y DIR` sets the wall on one side of cell `(x, y)`.
- `OPEN x y DIR` is included so future loaders can explicitly clear walls if needed.
- `--direction` controls the mouse heading before the first step.
- `--render none` suppresses maze drawing and prints only the final summary.
- `--render final` prints the initial and final states without the full step trace.
- `--seed` makes random-walk runs reproducible.
- `flood-fill` keeps an internal discovered wall map and replans toward the goal after each sensor update.

## Future Extension

The `IMazeSource` interface separates maze acquisition from simulation. A later `ImageMazeSource` can be added to:

1. Load an image with OpenCV.
2. Threshold it into a binary wall map.
3. Detect wall segments.
4. Convert them into the existing `Maze` structure.
