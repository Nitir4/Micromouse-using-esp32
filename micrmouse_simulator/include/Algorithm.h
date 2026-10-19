#ifndef MICRMOUSE_SIMULATOR_ALGORITHM_H
#define MICRMOUSE_SIMULATOR_ALGORITHM_H

#include <string>

#include "MouseAPI.h"

class IMouseAlgorithm {
public:
    virtual ~IMouseAlgorithm() = default;

    virtual std::string name() const = 0;
    virtual void step(MouseAPI& api) = 0;
};

#endif
