#include "RightHandRuleAlgorithm.h"

std::string RightHandRuleAlgorithm::name() const {
    return "right-hand";
}

void RightHandRuleAlgorithm::step(MouseAPI& api) {
    if (queuedActions_.empty()) {
        if (!api.wallRight()) {
            queuedActions_.push_back(Action::TurnRight);
            queuedActions_.push_back(Action::MoveForward);
        } else if (!api.wallFront()) {
            queuedActions_.push_back(Action::MoveForward);
        } else if (!api.wallLeft()) {
            queuedActions_.push_back(Action::TurnLeft);
            queuedActions_.push_back(Action::MoveForward);
        } else {
            queuedActions_.push_back(Action::TurnRight);
            queuedActions_.push_back(Action::TurnRight);
            queuedActions_.push_back(Action::MoveForward);
        }
    }

    executeNext(api);
    api.step();
}

void RightHandRuleAlgorithm::executeNext(MouseAPI& api) {
    Action action = queuedActions_.front();
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
}
