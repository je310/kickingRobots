
#include "urobotMain.h"

void urobotMain() {
	const uint32_t delay = 666;

	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_4, GPIO_PIN_SET); // aux power

	int color = 0;
	for (
		; true
		; color = (color + 1) % 3
		) {

		HAL_GPIO_TogglePin(GPIOB, GPIO_PIN_3);


		HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0, GPIO_PIN_SET); // red
		HAL_GPIO_WritePin(GPIOA, GPIO_PIN_8, GPIO_PIN_SET); // blue
		HAL_GPIO_WritePin(GPIOA, GPIO_PIN_7, GPIO_PIN_SET); // green

		switch(color){
		case 0:
			HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0, GPIO_PIN_RESET); // red
			break;
		case 1:
			HAL_GPIO_WritePin(GPIOA, GPIO_PIN_8, GPIO_PIN_RESET); // blue
			break;
		case 2:
			HAL_GPIO_WritePin(GPIOA, GPIO_PIN_7, GPIO_PIN_RESET); // green
			break;
		}

		HAL_Delay(delay);


	}
}
