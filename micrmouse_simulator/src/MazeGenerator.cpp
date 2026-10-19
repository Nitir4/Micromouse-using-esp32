#include "MazeGenerator.h"

#include <stdexcept>

Maze MazeGenerator::buildDefault(int width, int height, Position start, const std::vector<Position>& goals) {
    Maze maze(width, height);
    maze.setStart(start);

    if (goals.empty()) {
        throw std::invalid_argument("At least one goal cell is required");
    }

    maze.clearGoals();
    for (const Position& goal : goals) {
        maze.addGoal(goal);
    }

    // Add a small central obstacle while leaving the outer perimeter open so
    // simple wall-following algorithms still have a guaranteed route.
    if (width >= 6 && height >= 6) {
        const int left = width / 2 - 1;
        const int right = width / 2;
        const int top = height / 2 - 1;
        const int bottom = height / 2;

        maze.setWall({left, top}, Direction::North, true);
        maze.setWall({right, top}, Direction::North, true);
        maze.setWall({left, bottom}, Direction::South, true);
        maze.setWall({right, bottom}, Direction::South, true);
        maze.setWall({left, top}, Direction::West, true);
        maze.setWall({left, bottom}, Direction::West, true);
        maze.setWall({right, top}, Direction::East, true);
        maze.setWall({right, bottom}, Direction::East, true);
    }

    return maze;
}
