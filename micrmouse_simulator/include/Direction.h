#ifndef MICRMOUSE_SIMULATOR_DIRECTION_H
#define MICRMOUSE_SIMULATOR_DIRECTION_H

#include <stdexcept>

enum class Direction {
    North = 0,
    East = 1,
    South = 2,
    West = 3
};

struct Position {
    int x = 0;
    int y = 0;

    bool operator==(const Position& other) const {
        return x == other.x && y == other.y;
    }
};

inline Direction turnLeft(Direction direction) {
    return static_cast<Direction>((static_cast<int>(direction) + 3) % 4);
}

inline Direction turnRight(Direction direction) {
    return static_cast<Direction>((static_cast<int>(direction) + 1) % 4);
}

inline Direction turnAround(Direction direction) {
    return static_cast<Direction>((static_cast<int>(direction) + 2) % 4);
}

inline Position deltaFor(Direction direction) {
    switch (direction) {
        case Direction::North:
            return {0, -1};
        case Direction::East:
            return {1, 0};
        case Direction::South:
            return {0, 1};
        case Direction::West:
            return {-1, 0};
    }

    throw std::runtime_error("Unknown direction");
}

inline char directionSymbol(Direction direction) {
    switch (direction) {
        case Direction::North:
            return '^';
        case Direction::East:
            return '>';
        case Direction::South:
            return 'v';
        case Direction::West:
            return '<';
    }

    return '?';
}

#endif
