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
	if ((tempDegree >= DIRECTION_MIN && tempDegree <= DIRECTION_MAX) || tempDegree == 999) {
		gameConfig->direction = tempDegree;
	}
}

//void runEasyMode(GameConfig *gameConfig)
//{
//
//
//
////	  Rotate(gameConfig->direction);
//
//}
uint16_t positions[5] = {75, 82, 90, 98, 105};


void calcRandDirection(GameConfig *gameConfig) {
	uint8_t offset_idx = 1 + rand() % 4; // 1-4

	uint8_t idx = 0;
	for (int i=0; i < 5; ++i) {
		if (positions[i] == gameConfig->direction) {
			idx = i;
			break;
		}
	}
	offset_idx = (offset_idx + idx) % 5;
	gameConfig->direction = positions[offset_idx]; ;

}


