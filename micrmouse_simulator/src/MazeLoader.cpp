#include "MazeLoader.h"

#include <fstream>
#include <sstream>
#include <stdexcept>

#include "MazeGenerator.h"

namespace {

Direction parseDirection(const std::string& token) {
    if (token == "N" || token == "NORTH") {
        return Direction::North;
    }
    if (token == "E" || token == "EAST") {
        return Direction::East;
    }
    if (token == "S" || token == "SOUTH") {
        return Direction::South;
    }
    if (token == "W" || token == "WEST") {
        return Direction::West;
    }

    throw std::runtime_error("Unknown wall direction token: " + token);
}

std::string stripComment(const std::string& line) {
    const std::size_t commentIndex = line.find('#');
    if (commentIndex == std::string::npos) {
        return line;
    }
    return line.substr(0, commentIndex);
}

}  // namespace

FileMazeSource::FileMazeSource(std::string filePath)
    : filePath_(std::move(filePath)) {
}

Maze FileMazeSource::load() const {
    std::ifstream input(filePath_);
    if (!input) {
        throw std::runtime_error("Unable to open maze file: " + filePath_);
    }

    std::unique_ptr<Maze> maze;
    bool hasStart = false;
    std::vector<Position> goals;

    std::string line;
    std::size_t lineNumber = 0;
    while (std::getline(input, line)) {
        ++lineNumber;

        std::string cleaned = stripComment(line);
        std::istringstream stream(cleaned);

        std::string keyword;
        if (!(stream >> keyword)) {
            continue;
        }

        if (keyword == "WIDTH") {
            int width = 0;
            int height = 0;
            if (!(stream >> width >> height)) {
                throw std::runtime_error("Invalid WIDTH declaration on line " + std::to_string(lineNumber));
            }
            maze = std::make_unique<Maze>(width, height);
            continue;
        }

        if (!maze) {
            throw std::runtime_error("Maze dimensions must be declared before line " + std::to_string(lineNumber));
        }

        if (keyword == "START") {
            Position start{};
            if (!(stream >> start.x >> start.y)) {
                throw std::runtime_error("Invalid START declaration on line " + std::to_string(lineNumber));
            }
            maze->setStart(start);
            hasStart = true;
        } else if (keyword == "GOAL") {
            Position goal{};
            if (!(stream >> goal.x >> goal.y)) {
                throw std::runtime_error("Invalid GOAL declaration on line " + std::to_string(lineNumber));
            }
            goals.push_back(goal);
        } else if (keyword == "WALL") {
            Position cell{};
            std::string directionToken;
            if (!(stream >> cell.x >> cell.y >> directionToken)) {
                throw std::runtime_error("Invalid WALL declaration on line " + std::to_string(lineNumber));
            }
            maze->setWall(cell, parseDirection(directionToken), true);
        } else if (keyword == "OPEN") {
            Position cell{};
            std::string directionToken;
            if (!(stream >> cell.x >> cell.y >> directionToken)) {
                throw std::runtime_error("Invalid OPEN declaration on line " + std::to_string(lineNumber));
            }
            maze->setWall(cell, parseDirection(directionToken), false);
        } else {
            throw std::runtime_error("Unknown keyword '" + keyword + "' on line " + std::to_string(lineNumber));
        }
    }

    if (!maze) {
        throw std::runtime_error("Maze file is missing WIDTH declaration");
    }
    if (!hasStart) {
        throw std::runtime_error("Maze file is missing START declaration");
    }
    if (goals.empty()) {
        throw std::runtime_error("Maze file must declare at least one GOAL");
    }

    maze->clearGoals();
    for (const Position& goal : goals) {
        maze->addGoal(goal);
    }

    return *maze;
}

GeneratedMazeSource::GeneratedMazeSource(int width, int height, Position start, std::vector<Position> goals)
    : width_(width),
      height_(height),
      start_(start),
      goals_(std::move(goals)) {
}

Maze GeneratedMazeSource::load() const {
    return MazeGenerator::buildDefault(width_, height_, start_, goals_);
}
