/*
 * controller.h
 *
 */

#ifndef CONTROLLER_H_
#define CONTROLLER_H_

#include "stm32f4xx_hal.h"
#include "main.h"

extern SPI_HandleTypeDef hspi2;


/************************************** LCD typedefs **************************************/

typedef struct {
	/* All buttons are active low */
	uint8_t left;
	uint8_t down;
	uint8_t right;
	uint8_t up;

	uint8_t start;
//	uint8_t R3;
//	uint8_t L3;
	uint8_t select;

//	uint8_t square;
//	uint8_t X;
//	uint8_t O;
//	uint8_t triangle;

	uint8_t R1;
	uint8_t L1;
	uint8_t R2;
	uint8_t L2;
} ControllerState;

typedef struct {
	int direction; // -1 = left; 0 = not pressed; 1 = right
	int freqControl; // -1 = down; 0 = not pressed; 1 = up
	int changeMode; // 0 = select not pressed; 1 = select pressed
	int stop; // 0 = not pressed; 1 = L1, L2, R1, R2 all pressed
} ManualState;


/************************************** Public functions **************************************/
ControllerState controllerCreate(void);
ManualState manualStateCreate(void);
void controllerInit(void);
void controllerRead(ControllerState *controllerState, ManualState *manualState);


#endif /* CONTROLLER_H_ */
