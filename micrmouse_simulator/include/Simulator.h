#ifndef MICRMOUSE_SIMULATOR_SIMULATOR_H
#define MICRMOUSE_SIMULATOR_SIMULATOR_H

#include <cstddef>
#include <string>

#include "Algorithm.h"
#include "Maze.h"
#include "MouseAPI.h"

enum class RenderMode {
    AllSteps,
    FinalOnly,
    None
};

struct SimulationResult {
    bool success = false;
    std::size_t stepsTaken = 0;
    Position finalPosition{};
    Direction finalDirection = Direction::North;
    std::string statusMessage;
};

class Simulator {
public:
    Simulator(Maze maze, Direction initialDirection, std::size_t maxSteps, bool interactive, RenderMode renderMode);

    SimulationResult run(IMouseAlgorithm& algorithm);

private:
    class SimulatorMouseAPI : public MouseAPI {
    public:
        explicit SimulatorMouseAPI(Simulator& simulator);

        bool wallFront() override;
        bool wallLeft() override;
        bool wallRight() override;
        void moveForward() override;
        void turnLeft() override;
        void turnRight() override;
        void step() override;

        void beginAlgorithmStep();
        bool stepRequested() const;

    private:
        Simulator& simulator_;
        bool actionTaken_ = false;
        bool stepRequested_ = false;

        void ensureActionSlot();
    };

    Maze maze_;
    Position mousePosition_;
    Direction mouseDirection_;
    std::size_t maxSteps_;
    bool interactive_;
    RenderMode renderMode_;

    bool isWallRelative(Direction relativeDirection) const;
    void moveForward();
    void render(std::size_t stepNumber, const IMouseAlgorithm& algorithm, bool force) const;
};

#endif
