#include "Simulator.h"

#include <iostream>
#include <stdexcept>

#include "AsciiRenderer.h"
#include "Direction.h"

Simulator::Simulator(Maze maze,
                     Direction initialDirection,
                     std::size_t maxSteps,
                     bool interactive,
                     RenderMode renderMode)
    : maze_(std::move(maze)),
      mousePosition_(maze_.start()),
      mouseDirection_(initialDirection),
      maxSteps_(maxSteps),
      interactive_(interactive),
      renderMode_(renderMode) {
}

SimulationResult Simulator::run(IMouseAlgorithm& algorithm) {
    SimulatorMouseAPI api(*this);
    std::size_t stepsCompleted = 0;

    render(0, algorithm, false);
    if (maze_.isGoal(mousePosition_)) {
        render(0, algorithm, true);
        return {true, 0, mousePosition_, mouseDirection_, "Mouse starts on a goal cell"};
    }

    try {
        for (std::size_t stepNumber = 1; stepNumber <= maxSteps_; ++stepNumber) {
            api.beginAlgorithmStep();
            algorithm.step(api);

            if (!api.stepRequested()) {
                throw std::runtime_error("Algorithm did not call MouseAPI::step()");
            }

            render(stepNumber, algorithm, false);
            stepsCompleted = stepNumber;

            if (maze_.isGoal(mousePosition_)) {
                render(stepNumber, algorithm, true);
                return {true, stepNumber, mousePosition_, mouseDirection_, "Goal reached"};
            }
        }
    } catch (const std::exception& error) {
        render(stepsCompleted, algorithm, true);
        return {false, stepsCompleted, mousePosition_, mouseDirection_, error.what()};
    }

    render(maxSteps_, algorithm, true);
    return {false, maxSteps_, mousePosition_, mouseDirection_, "Maximum step limit reached"};
}

bool Simulator::isWallRelative(Direction relativeDirection) const {
    Direction absoluteDirection = relativeDirection;
    if (relativeDirection == Direction::North) {
        absoluteDirection = mouseDirection_;
    } else if (relativeDirection == Direction::West) {
        absoluteDirection = turnLeft(mouseDirection_);
    } else if (relativeDirection == Direction::East) {
        absoluteDirection = turnRight(mouseDirection_);
    }
    return maze_.hasWall(mousePosition_, absoluteDirection);
}

void Simulator::moveForward() {
    if (maze_.hasWall(mousePosition_, mouseDirection_)) {
        throw std::runtime_error("Illegal move attempted through a wall");
    }

    const Position delta = deltaFor(mouseDirection_);
    Position next{mousePosition_.x + delta.x, mousePosition_.y + delta.y};
    if (!maze_.inBounds(next)) {
        throw std::runtime_error("Illegal move attempted outside the maze");
    }

    mousePosition_ = next;
}

void Simulator::render(std::size_t stepNumber, const IMouseAlgorithm& algorithm, bool force) const {
    if (renderMode_ == RenderMode::None) {
        return;
    }

    if (!force) {
        if (renderMode_ == RenderMode::FinalOnly && stepNumber != 0) {
            return;
        }
    }

    std::cout << "Algorithm: " << algorithm.name()
              << " | Step: " << stepNumber
              << " | Position: (" << mousePosition_.x << ", " << mousePosition_.y << ")\n";
    std::cout << AsciiRenderer::render(maze_, mousePosition_, mouseDirection_) << std::flush;

    if (interactive_) {
        std::cout << "Press Enter for the next step..." << std::flush;
        std::string line;
        std::getline(std::cin, line);
    }
}

Simulator::SimulatorMouseAPI::SimulatorMouseAPI(Simulator& simulator)
    : simulator_(simulator) {
}

bool Simulator::SimulatorMouseAPI::wallFront() {
    return simulator_.maze_.hasWall(simulator_.mousePosition_, simulator_.mouseDirection_);
}

bool Simulator::SimulatorMouseAPI::wallLeft() {
    return simulator_.maze_.hasWall(simulator_.mousePosition_, ::turnLeft(simulator_.mouseDirection_));
}

bool Simulator::SimulatorMouseAPI::wallRight() {
    return simulator_.maze_.hasWall(simulator_.mousePosition_, ::turnRight(simulator_.mouseDirection_));
}

void Simulator::SimulatorMouseAPI::moveForward() {
    ensureActionSlot();
    simulator_.moveForward();
}

void Simulator::SimulatorMouseAPI::turnLeft() {
    ensureActionSlot();
    simulator_.mouseDirection_ = ::turnLeft(simulator_.mouseDirection_);
}

void Simulator::SimulatorMouseAPI::turnRight() {
    ensureActionSlot();
    simulator_.mouseDirection_ = ::turnRight(simulator_.mouseDirection_);
}

void Simulator::SimulatorMouseAPI::step() {
    stepRequested_ = true;
}

void Simulator::SimulatorMouseAPI::beginAlgorithmStep() {
    actionTaken_ = false;
    stepRequested_ = false;
}

bool Simulator::SimulatorMouseAPI::stepRequested() const {
    return stepRequested_;
}

void Simulator::SimulatorMouseAPI::ensureActionSlot() {
    if (actionTaken_) {
        throw std::runtime_error("Only one move or turn is allowed per simulator step");
    }
    actionTaken_ = true;
}
