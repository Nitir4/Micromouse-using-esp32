#ifndef MICRMOUSE_SIMULATOR_MAZE_LOADER_H
#define MICRMOUSE_SIMULATOR_MAZE_LOADER_H

#include <memory>
#include <string>
#include <vector>

#include "Maze.h"

class IMazeSource {
public:
    virtual ~IMazeSource() = default;
    virtual Maze load() const = 0;
};

class FileMazeSource : public IMazeSource {
public:
    explicit FileMazeSource(std::string filePath);

    Maze load() const override;

private:
    std::string filePath_;
};

class GeneratedMazeSource : public IMazeSource {
public:
    GeneratedMazeSource(int width, int height, Position start, std::vector<Position> goals);

    Maze load() const override;

private:
    int width_;
    int height_;
    Position start_;
    std::vector<Position> goals_;
};

#endif
