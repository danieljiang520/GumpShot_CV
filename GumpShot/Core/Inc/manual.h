/*
 * manual.h
 *
 *  Created on: Apr 11, 2023
 *      Author: danieljiang
 */

#ifndef INC_MANUAL_H_
#define INC_MANUAL_H_

#include "stm32f4xx_hal.h"
#include "main.h"
#include "controller.h"
#include "lcd_i2c.h"


/************************************** manual typedefs **************************************/



/************************************** Public functions **************************************/

void runManualMode(ControllerState *controllerState, GameConfig *gameConfig);
void runEasyMode(GameConfig *gameConfig);


#endif /* INC_MANUAL_H_ */
