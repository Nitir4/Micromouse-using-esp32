#include "RandomWalkAlgorithm.h"

#include <vector>

RandomWalkAlgorithm::RandomWalkAlgorithm()
    : generator_(std::random_device{}()) {
}

RandomWalkAlgorithm::RandomWalkAlgorithm(std::uint32_t seed)
    : generator_(seed) {
}

std::string RandomWalkAlgorithm::name() const {
    return "random";
}

void RandomWalkAlgorithm::step(MouseAPI& api) {
    if (queuedActions_.empty()) {
        std::vector<std::deque<Action>> options;

        if (!api.wallFront()) {
            options.push_back({Action::MoveForward});
        }
        if (!api.wallLeft()) {
            options.push_back({Action::TurnLeft, Action::MoveForward});
        }
        if (!api.wallRight()) {
            options.push_back({Action::TurnRight, Action::MoveForward});
        }

        options.push_back({Action::TurnRight, Action::TurnRight, Action::MoveForward});

        std::uniform_int_distribution<std::size_t> distribution(0, options.size() - 1);
        queuedActions_ = options[distribution(generator_)];
    }

    executeNext(api);
    api.step();
}

void RandomWalkAlgorithm::executeNext(MouseAPI& api) {
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
