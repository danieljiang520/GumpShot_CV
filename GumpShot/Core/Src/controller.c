/*
 * controller.h
 *
 *  Created on: Apr 10, 2023
 *      Author: danieljiang
 */

#include "controller.h"


/************************************** Function definitions **************************************/

/**
 * Create new ButtonState and initialize the PS1 controller
 */
ButtonState buttonCreate(void)
{
	// Set all button to not pressed
	ButtonState buttonState = {1, 1, 1, 1,
							   1, 1, 1, 1,
							   1, 1, 1, 1};

	controllerInit();

	return buttonState;
}


ControllerState controllerStateCreate(void)
{
	ControllerState controllerState = {0, 0, 0, 0, 0, 0};
	return controllerState;
}


/**
 * Initialize Controller by sending SPI commands
 */
void controllerInit(void)
{
	uint8_t data[21];

	// Poll once just for fun
	uint8_t pullOnce[5] = {0x01, 0x42, 0x00, 0xFF, 0xFF};
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_6, GPIO_PIN_RESET);
	HAL_SPI_TransmitReceive(&hspi2, (uint8_t *)pullOnce, (uint8_t *)data, 5, 100);
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_6, GPIO_PIN_SET);

	// Go into configuration mode
	uint8_t enterConfig[5] = {0x01, 0x43, 0x00, 0x01, 0x00};
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_6, GPIO_PIN_RESET);
	HAL_SPI_TransmitReceive(&hspi2, (uint8_t *)enterConfig, (uint8_t *)data, 5, 100);
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_6, GPIO_PIN_SET);

	// Check Status
	uint8_t checkStatus[9] = {0x01, 0x45, 0x00, 0x5A, 0x5A, 0x5A, 0x5A, 0x5A, 0x5A};
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_6, GPIO_PIN_RESET);
	HAL_SPI_TransmitReceive(&hspi2, (uint8_t *)checkStatus, (uint8_t *)data, 9, 100);
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_6, GPIO_PIN_SET);

	// Turn on analog mode
//	uint8_t analogMode[9] = {0x01, 0x44, 0x00, 0x01, 0x03, 0x00, 0x00, 0x00, 0x00};
//	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_6, GPIO_PIN_RESET);
//	HAL_SPI_TransmitReceive(&hspi2, (uint8_t *)analogMode, (uint8_t *)data, 9, 100);
//	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_6, GPIO_PIN_SET);

	// Setup motor command mapping
//	uint8_t motorMap[9] = {0x01, 0x4D, 0x00, 0x00, 0x01, 0xFF, 0xFF, 0xFF, 0xFF};
//	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_6, GPIO_PIN_RESET);
//	HAL_SPI_TransmitReceive(&hspi2, (uint8_t *)motorMap, (uint8_t *)data, 9, 100);
//	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_6, GPIO_PIN_SET);
//
//	// Config ontroller to return all pressure values
//	uint8_t pressureConfig[9] = {0x01, 0x4F, 0x00, 0xFF, 0xFF, 0x03, 0x00, 0x00, 0x00};
//	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_6, GPIO_PIN_RESET);
//	HAL_SPI_TransmitReceive(&hspi2, (uint8_t *)motorMap, (uint8_t *)data, 9, 100);
//	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_6, GPIO_PIN_SET);

	// Exit config mode
	uint8_t exitConfig[9] = {0x01, 0x43, 0x00, 0x00, 0x00};
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_6, GPIO_PIN_RESET);
	HAL_SPI_TransmitReceive(&hspi2, (uint8_t *)exitConfig, (uint8_t *)data, 9, 100);
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_6, GPIO_PIN_SET);

	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_6, GPIO_PIN_RESET);
	HAL_SPI_TransmitReceive(&hspi2, (uint8_t *)pullOnce, (uint8_t *)data, 5, 100);
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_6, GPIO_PIN_SET);
}


/**
 * Read Controller by sending SPI pulling command
 */
void controllerRead(ButtonState *buttonState, ControllerState *controllerState, GameConfig *gameConfig)
{
	/* Send pulling command to controller */
	uint8_t data[5];
	uint8_t command[5] = {0x01, 0x42, 0x00, 0x00, 0x00};
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_6, GPIO_PIN_RESET);
	HAL_SPI_TransmitReceive(&hspi2, (uint8_t *)command, (uint8_t *)data, 5, 100);
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_6, GPIO_PIN_SET);

	/* All buttons are active low */
	uint8_t left = (data[3] >> 7) & 0b1;
	uint8_t down = (data[3] >> 6) & 0b1;
	uint8_t right = (data[3] >> 5) & 0b1;
	uint8_t up = (data[3] >> 4) & 0b1;

	uint8_t start = (data[3] >> 3) & 0b1;
	uint8_t R3 = (data[3] >> 2) & 0b1;
	uint8_t L3 = (data[3] >> 1) & 0b1;
	uint8_t select = data[3] & 0b1;

	uint8_t square = (data[4] >> 7) & 0b1;
	uint8_t X = (data[4] >> 6) & 0b1;
	uint8_t O = (data[4] >> 5) & 0b1;
	uint8_t triangle = (data[4] >> 4) & 0b1;

	uint8_t R1 = (data[4] >> 3) & 0b1;
	uint8_t L1 = (data[4] >> 2) & 0b1;
	uint8_t R2 = (data[4] >> 1) & 0b1;
	uint8_t L2 = data[4] & 0b1;

	/* Update manual state only when button is released: 0->1 */
	uint8_t releasedLeft = !buttonState->left && left;
	uint8_t releasedDown = !buttonState->down && down;
	uint8_t releasedRight = !buttonState->right && right;
	uint8_t releasedUp = !buttonState->up && up;

	uint8_t releasedStart = !buttonState->start && start;
	uint8_t releasedR3 = !buttonState->R3 && R3;
	uint8_t releasedL3 = !buttonState->L3 && L3;
	uint8_t releasedSelect = !buttonState->select && select;

	uint8_t releasedSquare = !buttonState->square && square;
	uint8_t releasedX = !buttonState->X && X;
	uint8_t releasedO = !buttonState->O && O;
	uint8_t releasedTriangle = !buttonState->triangle && triangle;

	uint8_t releasedR1 = !buttonState->R1 && R1;
	uint8_t releasedL1 = !buttonState->L1 && L1;
	uint8_t releasedR2 = !buttonState->R2 && R2;
	uint8_t releasedL2 = !buttonState->L2 && L2;

	/* Update manual mode state */
    controllerState->direction = (releasedLeft ? 1 : (releasedRight ? -1 : 0));
    controllerState->freqControl = (releasedTriangle ? 1 : (releasedX ? -1 : 0));
    controllerState->speedControl = (releasedUp ? 1 : (releasedDown ? -1 : 0));
    controllerState->changeMode = (releasedSelect ? 1 : 0);
    controllerState->launch = (releasedSquare ? 1 : 0);
    controllerState->stop = (releasedO ? 1 : 0);

    buttonState->left = left;
    buttonState->down = down;
    buttonState->right = right;
    buttonState->up = up;

    buttonState->start = start;
    buttonState->R3 = R3;
    buttonState->L3 = L3;
    buttonState->select = select;

    buttonState->R1 = R1;
    buttonState->L1 = L1;
    buttonState->R2 = R2;
    buttonState->L2 = L2;

    buttonState->square = square;
    buttonState->X = X;
    buttonState->O = O;
    buttonState->triangle = triangle;

    updateGameConfig(controllerState, gameConfig);
}


void updateGameConfig(ControllerState *controllerState, GameConfig *gameConfig) {
	// Update mode
	if (controllerState->changeMode == 1) {
		if (gameConfig->mode >= 2) {
			gameConfig->mode = 0;
		} else {
			++gameConfig->mode;
		}
	}

	// if user presses button to increase motor speed
	if (controllerState->speedControl == 1 && gameConfig->speed <= (SPEED_MAX - SPEED_STEP))
		gameConfig->speed += SPEED_STEP;
	// if user presses button to decrease motor speed
	else if (controllerState->speedControl == -1 && gameConfig->speed >= SPEED_STEP)
		gameConfig->speed -= SPEED_STEP;

	// Update frequency
	if (controllerState->freqControl == 1 && controllerState->freqControl < PERIOD_MAX) {
		gameConfig->launcher_period += PERIOD_STEP;
		TIM3->ARR += 5000;
		TIM3->CCR1 += 5000;
		TIM3->CNT = 0;
	}
	else if (controllerState->freqControl == -1 && gameConfig->launcher_period > PERIOD_MIN) {
		gameConfig->launcher_period -= PERIOD_STEP;
		TIM3->ARR -= 5000;
		TIM3->CCR1 -= 5000;
		TIM3->CNT = 0;
	}

	// Update start
	if (controllerState->stop == 1) {
		// not stop
		if (gameConfig->stop == 0)
			gameConfig->stop = 1;
		else
			gameConfig->stop = 0;
	}

}

