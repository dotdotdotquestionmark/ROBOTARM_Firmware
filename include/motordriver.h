#pragma once

#include <avr/io.h>
#include <util/delay.h>

// four inputs, motorID, direction, steps, and speed
struct parameter {
    char motorID;
    int speed;
    int direction;
    int distance;
};

class Motion {
    public:
    int engageStepper(int direction, int speed);
    int engageServo(int direction, int speed);
};

class Stasis {
    public:
    int holdStepper();
    int holdServo();
};