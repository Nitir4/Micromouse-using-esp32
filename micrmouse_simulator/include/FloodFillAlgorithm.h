#ifndef MICRMOUSE_SIMULATOR_FLOOD_FILL_ALGORITHM_H
#define MICRMOUSE_SIMULATOR_FLOOD_FILL_ALGORITHM_H

#include <deque>
#include <vector>

#include "Algorithm.h"
#include "Direction.h"

class FloodFillAlgorithm : public IMouseAlgorithm {
public:
    FloodFillAlgorithm(int width,
                       int height,
                       Position start,
                       std::vector<Position> goals,
                       Direction initialDirection);

    std::string name() const override;
    void step(MouseAPI& api) override;

private:
    struct CellKnowledge {
        bool known[4] = {false, false, false, false};
        bool wall[4] = {false, false, false, false};
    };

    enum class Action {
        MoveForward,
        TurnLeft,
        TurnRight
    };

    int width_;
    int height_;
    Position current_;
    Direction direction_;
    std::vector<Position> goals_;
    std::vector<std::vector<CellKnowledge>> knowledge_;
    std::deque<Action> queuedActions_;

    bool inBounds(const Position& position) const;
    static int directionIndex(Direction direction);
    void recordWall(const Position& cell, Direction direction, bool exists);
    bool isKnownWall(const Position& cell, Direction direction) const;
    bool canTraverse(const Position& cell, Direction direction) const;
    void senseCurrentCell(MouseAPI& api);
    std::vector<std::vector<int>> computeDistances() const;
    Direction chooseBestDirection(const std::vector<std::vector<int>>& distances) const;
    void queueMove(Direction targetDirection);
    void executeNext(MouseAPI& api);
    void applyActionToInternalState(Action action);
};

#endif
