#ifndef MICRMOUSE_SIMULATOR_RIGHT_HAND_RULE_ALGORITHM_H
#define MICRMOUSE_SIMULATOR_RIGHT_HAND_RULE_ALGORITHM_H

#include <deque>

#include "Algorithm.h"

class RightHandRuleAlgorithm : public IMouseAlgorithm {
public:
    std::string name() const override;
    void step(MouseAPI& api) override;

private:
    enum class Action {
        MoveForward,
        TurnLeft,
        TurnRight
    };

    std::deque<Action> queuedActions_;

    void executeNext(MouseAPI& api);
};

#endif
