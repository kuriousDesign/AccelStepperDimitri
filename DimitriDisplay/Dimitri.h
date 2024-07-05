#ifndef DIMITRI_H
#define DIMITRI_H


class Dimitri
{
public:
    // Game::Modes, these should match Game::Modes enum in the outerspace
    enum Modes : int32_t
    {
        ABORTING = -3,
        KILLED = -2,
        INACTIVE = 0,
        RESETTING = 50,
        IDLE = 100,
        HOMING = 200,
        RUNNING = 500,
        ERROR = 911,
        MANUAL = 1100,

    };

    enum Errors : int32_t
    {
        NONE = 0,
        POS_LIM_SW_ON_WHEN_NOT_AT_GEAR_12 = 1,
        CAM_SW_NOT_DETECTED_WHEN_AT_POS = 2,
        HOMING_FAILED = 3,
    };
};

#endif