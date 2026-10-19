#include "FloodFillAlgorithm.h"

#include <limits>
#include <queue>
#include <stdexcept>

namespace {

Direction opposite(Direction direction) {
    return turnAround(direction);
}

}  // namespace

FloodFillAlgorithm::FloodFillAlgorithm(int width,
                                       int height,
                                       Position start,
                                       std::vector<Position> goals,
                                       Direction initialDirection)
    : width_(width),
      height_(height),
      current_(start),
      direction_(initialDirection),
      goals_(std::move(goals)),
      knowledge_(height, std::vector<CellKnowledge>(width)) {
    for (int y = 0; y < height_; ++y) {
        recordWall({0, y}, Direction::West, true);
        recordWall({width_ - 1, y}, Direction::East, true);
    }

    for (int x = 0; x < width_; ++x) {
        recordWall({x, 0}, Direction::North, true);
        recordWall({x, height_ - 1}, Direction::South, true);
    }
}

std::string FloodFillAlgorithm::name() const {
    return "flood-fill";
}

void FloodFillAlgorithm::step(MouseAPI& api) {
    if (queuedActions_.empty()) {
        senseCurrentCell(api);
        std::vector<std::vector<int>> distances = computeDistances();
        queueMove(chooseBestDirection(distances));
    }

    if (!queuedActions_.empty()) {
        executeNext(api);
    }
    api.step();
}

bool FloodFillAlgorithm::inBounds(const Position& position) const {
    return position.x >= 0 && position.x < width_ && position.y >= 0 && position.y < height_;
}

int FloodFillAlgorithm::directionIndex(Direction direction) {
    return static_cast<int>(direction);
}

void FloodFillAlgorithm::recordWall(const Position& cell, Direction direction, bool exists) {
    if (!inBounds(cell)) {
        return;
    }

    CellKnowledge& currentCell = knowledge_[cell.y][cell.x];
    const int index = directionIndex(direction);
    currentCell.known[index] = true;
    currentCell.wall[index] = exists;

    const Position delta = deltaFor(direction);
    const Position neighbor{cell.x + delta.x, cell.y + delta.y};
    if (!inBounds(neighbor)) {
        return;
    }

    CellKnowledge& neighborCell = knowledge_[neighbor.y][neighbor.x];
    const int oppositeIndex = directionIndex(opposite(direction));
    neighborCell.known[oppositeIndex] = true;
    neighborCell.wall[oppositeIndex] = exists;
}

bool FloodFillAlgorithm::isKnownWall(const Position& cell, Direction direction) const {
    const CellKnowledge& currentCell = knowledge_[cell.y][cell.x];
    const int index = directionIndex(direction);
    return currentCell.known[index] && currentCell.wall[index];
}

bool FloodFillAlgorithm::canTraverse(const Position& cell, Direction direction) const {
    if (!inBounds(cell)) {
        return false;
    }

    const Position delta = deltaFor(direction);
    const Position next{cell.x + delta.x, cell.y + delta.y};
    if (!inBounds(next)) {
        return false;
    }

    return !isKnownWall(cell, direction);
}

void FloodFillAlgorithm::senseCurrentCell(MouseAPI& api) {
    recordWall(current_, direction_, api.wallFront());
    recordWall(current_, turnLeft(direction_), api.wallLeft());
    recordWall(current_, turnRight(direction_), api.wallRight());
}

std::vector<std::vector<int>> FloodFillAlgorithm::computeDistances() const {
    const int unreachable = std::numeric_limits<int>::max();
    std::vector<std::vector<int>> distances(height_, std::vector<int>(width_, unreachable));
    std::queue<Position> frontier;

    for (const Position& goal : goals_) {
        if (!inBounds(goal)) {
            continue;
        }
        distances[goal.y][goal.x] = 0;
        frontier.push(goal);
    }

    while (!frontier.empty()) {
        const Position cell = frontier.front();
        frontier.pop();

        for (Direction direction : {Direction::North, Direction::East, Direction::South, Direction::West}) {
            if (!canTraverse(cell, direction)) {
                continue;
            }

            const Position delta = deltaFor(direction);
            const Position next{cell.x + delta.x, cell.y + delta.y};
            if (distances[next.y][next.x] <= distances[cell.y][cell.x] + 1) {
                continue;
            }

            distances[next.y][next.x] = distances[cell.y][cell.x] + 1;
            frontier.push(next);
        }
    }

    return distances;
}

Direction FloodFillAlgorithm::chooseBestDirection(const std::vector<std::vector<int>>& distances) const {
    const int unreachable = std::numeric_limits<int>::max();
    Direction bestDirection = direction_;
    int bestDistance = unreachable;
    int bestTurnPenalty = std::numeric_limits<int>::max();

    for (Direction candidate : {direction_, turnLeft(direction_), turnRight(direction_), turnAround(direction_)}) {
        if (!canTraverse(current_, candidate)) {
            continue;
        }

        const Position delta = deltaFor(candidate);
        const Position next{current_.x + delta.x, current_.y + delta.y};
        const int distance = distances[next.y][next.x];
        if (distance == unreachable) {
            continue;
        }

        int turnPenalty = 0;
        if (candidate == turnAround(direction_)) {
            turnPenalty = 2;
        } else if (candidate != direction_) {
            turnPenalty = 1;
        }

        if (distance < bestDistance || (distance == bestDistance && turnPenalty < bestTurnPenalty)) {
            bestDistance = distance;
            bestTurnPenalty = turnPenalty;
            bestDirection = candidate;
        }
    }

    if (bestDistance == unreachable) {
        throw std::runtime_error("Flood fill could not find a route to any goal");
    }

    return bestDirection;
}

void FloodFillAlgorithm::queueMove(Direction targetDirection) {
    if (targetDirection == turnLeft(direction_)) {
        queuedActions_.push_back(Action::TurnLeft);
    } else if (targetDirection == turnRight(direction_)) {
        queuedActions_.push_back(Action::TurnRight);
    } else if (targetDirection == turnAround(direction_)) {
        queuedActions_.push_back(Action::TurnRight);
        queuedActions_.push_back(Action::TurnRight);
    }

    queuedActions_.push_back(Action::MoveForward);
}

void FloodFillAlgorithm::executeNext(MouseAPI& api) {
    const Action action = queuedActions_.front();
    queuedActions_.pop_front();

    switch (action) {
        case Action::MoveForward:
            api.moveForward();
            break;
        case Action::TurnLeft:
            api.turnLeft();
            break;
        case Action::TurnRight:
            api.turnRight();
            break;
    }

    applyActionToInternalState(action);
}

void FloodFillAlgorithm::applyActionToInternalState(Action action) {
    switch (action) {
        case Action::MoveForward: {
            const Position delta = deltaFor(direction_);
            const Position previous = current_;
            current_ = {current_.x + delta.x, current_.y + delta.y};
            recordWall(previous, direction_, false);
            break;
        }
        case Action::TurnLeft:
            direction_ = turnLeft(direction_);
            break;
        case Action::TurnRight:
            direction_ = turnRight(direction_);
            break;
    }
}
