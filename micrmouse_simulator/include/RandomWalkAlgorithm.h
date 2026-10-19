#ifndef MICRMOUSE_SIMULATOR_RANDOM_WALK_ALGORITHM_H
#define MICRMOUSE_SIMULATOR_RANDOM_WALK_ALGORITHM_H

#include <cstdint>
#include <deque>
#include <random>

#include "Algorithm.h"

class RandomWalkAlgorithm : public IMouseAlgorithm {
public:
    RandomWalkAlgorithm();
    explicit RandomWalkAlgorithm(std::uint32_t seed);

    std::string name() const override;
    void step(MouseAPI& api) override;

private:
    enum class Action {
        MoveForward,
        TurnLeft,
        TurnRight
    };

    std::deque<Action> queuedActions_;
    std::mt19937 generator_;

    void executeNext(MouseAPI& api);
};

#endif
