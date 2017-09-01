#pragma once

#include "stm32f0xx_hal.h"
#include "NEC_Decode.h"
#include "main.h"

typedef struct hardware{
	ADC_HandleTypeDef* hadc;
	RTC_HandleTypeDef* hrtc;
	TIM_HandleTypeDef* htim1;
	TIM_HandleTypeDef* htim2;
	TIM_HandleTypeDef* htim3;
	TIM_HandleTypeDef* htim16;
	TIM_HandleTypeDef* htim17;
	TIM_HandleTypeDef* htim14;
}hdw;




void millisecondCallback();

hdw hardware;
void NECHandler();
void parseNECTimes();
void auxOn();
void auxOff();
void transmitIRburst();
void readAnalog();
void lightLED(int colour,int value);
void LEDoff();
void activateMotors();
void deactivateMotors();
void stepMotors();
void IRLED(int on);
void motorsMiddle();
void delayTime(unsigned int loops);
void StandbyMode_Measure(void);
static void SYSCLKConfig_STOP(void);
static void RTC_AlarmConfig(unsigned int ms);
void StopMode_Measure(void);
void displayBattVoltage();
void followLine();

unsigned ADC_raw;
unsigned int Vdd;
NEC nec;
float voltage;
unsigned int rightIR;
unsigned int leftIR;
unsigned int backIR;
 int rightAcc;
 int leftAcc;
 int backAcc;
 int backAccSaved;
unsigned int emittedBit;
int cal;

void urobotMain();
