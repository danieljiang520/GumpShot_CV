/*
 * controller.h
 *
 *  Created on: Apr 10, 2023
 *      Author: danieljiang
 */

#ifndef CONTROLLER_H_
#define CONTROLLER_H_

#include "stm32f4xx_hal.h"
#include "main.h"

extern SPI_HandleTypeDef hspi2;

#define PERIOD_STEP 20
#define DIRECTION_MAX 135
#define DIRECTION_STEP 3
#define SPEED_MAX 100
#define SPEED_STEP 5
/************************************** controller typedefs **************************************/

typedef struct {
	/* All buttons are active low */
	uint8_t left;
	uint8_t down;
	uint8_t right;
	uint8_t up;

	uint8_t start;
	uint8_t R3;
	uint8_t L3;
	uint8_t select;

	uint8_t square;
	uint8_t X;
	uint8_t O;
	uint8_t triangle;

	uint8_t R1;
	uint8_t L1;
	uint8_t R2;
	uint8_t L2;
} ButtonState;


typedef struct {
	int direction; // -1 = left; 0 = not pressed; 1 = right
	int freqControl; // -1 = down; 0 = not pressed; 1 = up
	int speedControl;
	int changeMode; // 0 = select not pressed; 1 = select pressed
	int launch; // 0 = not pressed; 1 = square pressed
	int stop; // 0 = not pressed; 1 = L1, L2, R1, R2 all pressed
} ControllerState;


/************************************** Public functions **************************************/

ButtonState buttonCreate(void);
ControllerState controllerStateCreate(void);
void controllerInit(void);
void controllerRead(ButtonState *buttonState, ControllerState *controllerState, GameConfig *gameConfig);
void updateGameConfig(ControllerState *controllerState, GameConfig *gameConfig);


#endif /* CONTROLLER_H_ */
