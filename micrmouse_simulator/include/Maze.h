#ifndef MICRMOUSE_SIMULATOR_MAZE_H
#define MICRMOUSE_SIMULATOR_MAZE_H

#include <string>
#include <vector>

#include "Direction.h"

class Maze {
public:
    Maze(int width, int height);

    int width() const;
    int height() const;

    bool inBounds(const Position& position) const;
    bool hasWall(const Position& cell, Direction direction) const;
    void setWall(const Position& cell, Direction direction, bool exists);

    void setStart(const Position& start);
    const Position& start() const;

    void addGoal(const Position& goal);
    void clearGoals();
    const std::vector<Position>& goals() const;
    bool isGoal(const Position& position) const;

private:
    int width_;
    int height_;
    Position start_;
    std::vector<Position> goals_;
    std::vector<std::vector<bool>> verticalWalls_;
    std::vector<std::vector<bool>> horizontalWalls_;

    void validateCell(const Position& cell) const;
};

#endif
