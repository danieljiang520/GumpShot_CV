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

void runEasyMode(GameConfig *gameConfig)
{
	if (gameConfig->launcher_timer >= gameConfig->launcher_period) {
	  gameConfig->launcher_timer = 0;
	  LockingServo();
	}
	else {
	  gameConfig->launcher_timer++;

	}



//	get paddle location (depth) from pi
//	calculate motor speed
//	Set launcher motors to proper power
	LauncherMotors(gameConfig->speed);

//	get paddle location (side to side) from pi
//	calculate rotational degrees
//	Rotate launcher to proper location
//	Rotate(degrees);
//	if (!HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_13)) {
//	  LockingServo();
//	}

	  	  // Display to LCD

}


