#ifndef MICRMOUSE_SIMULATOR_MAZE_GENERATOR_H
#define MICRMOUSE_SIMULATOR_MAZE_GENERATOR_H

#include <vector>

#include "Maze.h"

class MazeGenerator {
public:
    static Maze buildDefault(int width, int height, Position start, const std::vector<Position>& goals);
};

#endif
