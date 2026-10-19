#ifndef MICRMOUSE_SIMULATOR_MOUSE_API_H
#define MICRMOUSE_SIMULATOR_MOUSE_API_H

class MouseAPI {
public:
    virtual ~MouseAPI() = default;

    virtual bool wallFront() = 0;
    virtual bool wallLeft() = 0;
    virtual bool wallRight() = 0;

    virtual void moveForward() = 0;
    virtual void turnLeft() = 0;
    virtual void turnRight() = 0;

    virtual void step() = 0;
};

#endif
