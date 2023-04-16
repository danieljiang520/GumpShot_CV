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

    // if user presses button to increase rotation (to the right)
    if (controllerState->direction == 1 && gameConfig->direction <= (DIRECTION_MAX - DIRECTION_STEP))
        gameConfig->direction += DIRECTION_STEP;
    // if user presses button to decrease rotation (to the left)
    else if (controllerState->direction == -1 && gameConfig->direction >= (DIRECTION_MIN + DIRECTION_STEP))
        gameConfig->direction -= DIRECTION_STEP;


    // if user presses button to launch ping pong ball
    if (controllerState->launch == 1)
        LockingServo();


    Rotate(gameConfig->direction);


}

void readUART(GameConfig *gameConfig, char data[]) {
	uint16_t tempDegree = 0;
	int exponent = 0;
	//read from least significant bit
	for (int i = 5; i >= 0; --i) {
		char c = data[i];
		if (c == 'f') {
			continue;
		}

		else if (c == 's'){
			break;
		}

		else if ('0' <= c && c <= '9') {
		    uint16_t base = c - '0';
		    tempDegree += base * pow(10, exponent);
		    exponent++;
		}
	}

	// update gameConfig only if tempDegree != 0 or 999 (no bounding box)
	if (tempDegree >= DIRECTION_MIN && tempDegree <= DIRECTION_MAX) {
		gameConfig->direction = tempDegree;
	}
}

void runEasyMode(GameConfig *gameConfig)
{



//	  Rotate(gameConfig->direction);

}


