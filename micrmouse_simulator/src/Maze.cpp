#include "Maze.h"

#include <stdexcept>

Maze::Maze(int width, int height)
    : width_(width),
      height_(height),
      start_{0, 0},
      verticalWalls_(height, std::vector<bool>(width + 1, false)),
      horizontalWalls_(height + 1, std::vector<bool>(width, false)) {
    if (width <= 0 || height <= 0) {
        throw std::invalid_argument("Maze dimensions must be positive");
    }

    for (int y = 0; y < height_; ++y) {
        verticalWalls_[y][0] = true;
        verticalWalls_[y][width_] = true;
    }

    for (int x = 0; x < width_; ++x) {
        horizontalWalls_[0][x] = true;
        horizontalWalls_[height_][x] = true;
    }
}

int Maze::width() const {
    return width_;
}

int Maze::height() const {
    return height_;
}

bool Maze::inBounds(const Position& position) const {
    return position.x >= 0 && position.x < width_ && position.y >= 0 && position.y < height_;
}

bool Maze::hasWall(const Position& cell, Direction direction) const {
    validateCell(cell);

    switch (direction) {
        case Direction::North:
            return horizontalWalls_[cell.y][cell.x];
        case Direction::East:
            return verticalWalls_[cell.y][cell.x + 1];
        case Direction::South:
            return horizontalWalls_[cell.y + 1][cell.x];
        case Direction::West:
            return verticalWalls_[cell.y][cell.x];
    }

    return true;
}

void Maze::setWall(const Position& cell, Direction direction, bool exists) {
    validateCell(cell);

    switch (direction) {
        case Direction::North:
            horizontalWalls_[cell.y][cell.x] = exists;
            break;
        case Direction::East:
            verticalWalls_[cell.y][cell.x + 1] = exists;
            break;
        case Direction::South:
            horizontalWalls_[cell.y + 1][cell.x] = exists;
            break;
        case Direction::West:
            verticalWalls_[cell.y][cell.x] = exists;
            break;
    }
}

void Maze::setStart(const Position& start) {
    validateCell(start);
    start_ = start;
}

const Position& Maze::start() const {
    return start_;
}

void Maze::addGoal(const Position& goal) {
    validateCell(goal);
    goals_.push_back(goal);
}

void Maze::clearGoals() {
    goals_.clear();
}

const std::vector<Position>& Maze::goals() const {
    return goals_;
}

bool Maze::isGoal(const Position& position) const {
    for (const Position& goal : goals_) {
        if (goal == position) {
            return true;
        }
    }

    return false;
}

void Maze::validateCell(const Position& cell) const {
    if (!inBounds(cell)) {
        throw std::out_of_range("Cell position is outside the maze bounds");
    }
}
