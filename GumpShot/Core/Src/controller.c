/*
 * controller.c
 *
 */

#include "controller.h"


/************************************** Function definitions **************************************/

/**
 * Create new Lcd_HandleTypeDef and initialize the Lcd
 */
ControllerState controllerCreate()
{
	// Set all button to not pressed
	ControllerState controllerState = {1, 1, 1, 1, 1, 1, 1, 1, 1, 1};

	initController();

	return controllerState;
}


/**
 * Initialize Controller by sending SPI commands
 */
void initController(void)
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
void controllerRead(ControllerState *controllerState, ManualState *manualState)
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
//	uint8_t R3 = (data[3] >> 2) & 0b1;
//	uint8_t L3 = (data[3] >> 1) & 0b1;
	uint8_t select = data[3] & 0b1;

//	uint8_t square = (data[4] >> 7) & 0b1;
//	uint8_t X = (data[4] >> 6) & 0b1;
//	uint8_t O = (data[4] >> 5) & 0b1;
//	uint8_t triangle = (data[4] >> 4) & 0b1;

	uint8_t R1 = (data[4] >> 3) & 0b1;
	uint8_t L1 = (data[4] >> 2) & 0b1;
	uint8_t R2 = (data[4] >> 1) & 0b1;
	uint8_t L2 = data[4] & 0b1;

	/* Update manual state only when button is released: 0->1 */
	uint8_t releasedLeft = !controllerState->left && left;
	uint8_t releasedDown = !controllerState->down && down;
	uint8_t releasedRight = !controllerState->right && right;
	uint8_t releasedUp = !controllerState->up && up;

	uint8_t releasedStart = !controllerState->start && start;
//	uint8_t releasedR3 = !controllerState->R3 && R3;
//	uint8_t releasedL3 = !controllerState->L3 && L3;
	uint8_t releasedSelect = !controllerState->select && select;

//	uint8_t releasedSquare = !controllerState->square && square;
//	uint8_t releasedX = !controllerState->X && X;
//	uint8_t releasedO = !controllerState->O && O;
//	uint8_t releasedTriangle = !controllerState->triangle && triangle;

	uint8_t releasedR1 = !controllerState->R1 && R1;
	uint8_t releasedL1 = !controllerState->L1 && L1;
	uint8_t releasedR2 = !controllerState->R2 && R2;
	uint8_t releasedL2 = !controllerState->L2 && L2;

	/* Update manual mode state */
    manualState->direction = (releasedRight ? 1 : (releasedLeft ? -1 : 0));
    manualState->freqControl = (releasedUp ? 1 : (releasedDown ? -1 : 0));
    manualState->changeMode = (releasedSelect ? 1 : 0);
    manualState->stop = (releasedL1 && releasedL2 && releasedR1 && releasedR2 ? 1 : 0);

    controllerState->left = left;
    controllerState->down = down;
    controllerState->right = right;
    controllerState->up = up;

    controllerState->start = start;
    controllerState->select = select;

    controllerState->R1 = R1;
    controllerState->L1 = L1;
    controllerState->R2 = R2;
    controllerState->L2 = L2;
}

