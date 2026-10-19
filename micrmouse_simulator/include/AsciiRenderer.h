#ifndef MICRMOUSE_SIMULATOR_ASCII_RENDERER_H
#define MICRMOUSE_SIMULATOR_ASCII_RENDERER_H

#include <string>

#include "Maze.h"

class AsciiRenderer {
public:
    static std::string render(const Maze& maze, const Position& mousePosition, Direction mouseDirection);
};

#endif
