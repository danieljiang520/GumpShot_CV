/*
 * manual.c
 *
 *  Created on: Apr 11, 2023
 *      Author: danieljiang
 */

#include "manual.h"


/************************************** Function definitions **************************************/

/**
 * Create new ControllerState
 */

void runManualMode(ControllerState *controllerState, GameConfig *gameConfig)
{
    // if user presses button to increase motor speed
    if (controllerState->speedControl == 1 && gameConfig->speed <= (SPEED_MAX - SPEED_STEP))
        gameConfig->speed += SPEED_STEP;
    // if user presses button to decrease motor speed
    else if (controllerState->speedControl == -1 && gameConfig->speed >= SPEED_STEP)
        gameConfig->speed -= SPEED_STEP;

    LauncherMotors(gameConfig->speed);

    // if user presses button to increase rotation (to the right)
    if (controllerState->direction == 1 && gameConfig->direction <= (DIRECTION_MAX - DIRECTION_STEP))
        gameConfig->direction += DIRECTION_STEP;
    // if user presses button to decrease rotation (to the left)
    else if (controllerState->direction && gameConfig->direction >= DIRECTION_STEP)
        gameConfig->direction -= DIRECTION_STEP;


    Rotate(gameConfig->direction);

    // if user presses button to launch ping pong ball
    if (controllerState->launch == 1)
        LockingServo();

}
