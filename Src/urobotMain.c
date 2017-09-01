#include "urobotMain.h"

#include "stm32f0xx_hal.h"


//This is the primary file for the robot firmware. As so far as convenient all user written functions should be declared in
// urobotMain.h with the implementation at the bottom of this file.
// The Auto generated files should be left as un-edited as possible, apart from things required to enter this section of the code.
// hardware contains all of the handles required. HAL functions should also be inside user written functions with robot specific names such that the
// main loop is self documenting.


int leftMotorPeriod = 5;
int rightMotorPeriod = 5;
const int microstepDiv = 2;
int defaultSpeed = 1;
int slowMult = 3;


uint8_t thisCmd = 0;

enum Direction{
	dForward = 1,
	dStopped = 0,
	dBackward = -1,
};

enum Motor{
	mLeft,
	mRight,
};

enum bool{
	False,
	True,
};
enum Direction leftMotorDirn = dStopped;
enum Direction rightMotorDirn = dStopped;

unsigned int timeSinceLastReset = 0;



const unsigned int KEEP_ALIVE = 240;
float arr[256];


//lsfr code
const size_t bits = 7;
size_t XorFrom[] = {7,6};
const float framerate = 1000;

//unsigned int emittedBit = 0;
#define GOLD_SZ 5
enum bool emittCode = False;

enum bool showSensor;

void step_lfsr(uint8_t* lfsr, uint8_t const* const taps, size_t taps_sz) {
	// TODO: downshift instead?
	uint8_t scratch = 0x0;
	int i = 0;
	for (i = 0; i < taps_sz; ++i) {
		scratch ^= ((*lfsr) >> taps[i]);
	}
	*lfsr = ((*lfsr) << 1) | (scratch & 0x1);
	*lfsr &= 0x1f;  // technically, not needed
}

void step_GC(){
	const uint8_t seed = 0x8;  // 00010 reversed

	static uint8_t lfsr1 = 0x1 << (GOLD_SZ - 1);  // top bit set
	static uint8_t lfsr2 = 0x8;
	const char taps1[] = {1, 2, 3, 4};  // -1
	const uint8_t taps2[] = {2, 4};     // -1
	const size_t taps_sz1 = sizeof(taps1) / sizeof(taps1[0]);
	const size_t taps_sz2 = sizeof(taps2) / sizeof(taps2[0]);
	static uint8_t xored_lfsr = 0;

	step_lfsr(&lfsr1, taps1, taps_sz1);
	step_lfsr(&lfsr2, taps2, taps_sz2);
	xored_lfsr = lfsr1 ^ lfsr2;

	emittedBit = (xored_lfsr & (1 << (GOLD_SZ - 1))) ;
	emittedBit = emittedBit >> (GOLD_SZ - 1);
}

//void  lsfr() {
//
////	    static uint16_t lfsr = 1;
////	    uint16_t bit;                    /* Must be 16bit to allow bit<<15 later in the code */
////
////		/* taps: 16 14 13 11; feedback polynomial: x^16 + x^14 + x^13 + x^11 + 1 */
////		bit  = ((lfsr >> 0) ^ (lfsr >> 2) ^ (lfsr >> 3) ^ (lfsr >> 5) ) & 1;
////		lfsr =  (lfsr >> 1) | (bit << 15);
////		if (bit == 0) emittedBit = 0;
////		else emittedBit = 1;
//	static int state = 0;
//	state = 1-state;
//	emittedBit = state;
//
//
//}



enum bool checkBatteryState = True;
enum bool shouldCal = False;
int kickCount = 0;
void millisecondCallback(){



	static unsigned int msCounter = 0;
	msCounter += 1;
	timeSinceLastReset += 1;

	if (msCounter % leftMotorPeriod == 0){
		StepMotor(mLeft, leftMotorDirn);
	}


	if (msCounter % rightMotorPeriod == 0){
		StepMotor(mRight, rightMotorDirn);
	}

	if (timeSinceLastReset > KEEP_ALIVE){
		leftMotorDirn = dStopped;
		rightMotorDirn = dStopped;
		deactivateMotors();

	}



	if (emittCode){
		step_GC();
		emittedBit ? IRLED(1):IRLED(0);
		HAL_ADC_Start_IT(hardware.hadc);

	}
	else IRLED(0);

	if(msCounter%((1 << GOLD_SZ) - 1 )==0){
		if (shouldCal){
			shouldCal = False;
			cal =  (leftAcc - rightAcc);
		}
		int diff = leftAcc - cal - rightAcc;
		if(showSensor)
			(diff > 0) ? lightLED(2,1800): lightLED(2,2001);
		leftAcc = 0;
		rightAcc =0;
		backAccSaved = backAcc;
		backAcc = 0;
	}


	if (checkBatteryState == True){
		//		readAnalog();
		HAL_ADC_Start_IT(hardware.hadc);
		CheckBattery(msCounter);
	}
	kickCount ++;
	if(kickCount > 100){
		HAL_GPIO_WritePin(GPIOF, GPIO_PIN_1, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(GPIOA, GPIO_PIN_12, GPIO_PIN_RESET);
	}
	ButtonPressed(thisCmd);

}

enum CheckBatteryState{
	cbsReadAnalog,
	cbsTurnOnLed,
	cbsWaitingForReset,
};

void CheckBattery(unsigned int time){
	const unsigned int now = time % 1050;

	static CheckBatteryState = cbsReadAnalog;

	if (CheckBatteryState == cbsReadAnalog && now < 45){
		LEDoff();
		CheckBatteryState = cbsTurnOnLed;
	} else if(CheckBatteryState == cbsTurnOnLed && now >= 50 && now < 100 ){
		displayBattVoltage();
		CheckBatteryState = cbsWaitingForReset;
	} else if (CheckBatteryState == cbsWaitingForReset && now >= 100){
		LEDoff();
		CheckBatteryState = cbsReadAnalog;
	}
}


const int sins[] = {100.0, 109.0, 119.0, 129.0, 138.0, 147.0, 155.0, 163.0, 170.0, 177.0, 183.0, 188.0,
		192.0, 195.0, 198.0, 199.0, 200.0, 199.0, 198.0, 195.0, 192.0, 188.0, 183.0, 177.0, 170.0,
		163.0, 155.0, 147.0, 138.0, 129.0, 119.0, 109.0, 100.0, 90.0, 80.0, 70.0, 61.0, 52.0, 44.0,
		36.0, 29.0, 22.0, 16.0, 11.0, 7.0, 4.0, 1.0, 0.0, 0.0, 0.0, 1.0, 4.0, 7.0, 11.0, 16.0, 22.0,
		29.0, 36.0, 44.0, 52.0, 61.0, 70.0, 80.0, 90.0};

//int sins[] = {200, 200, 200, 200, 200, 200, 200, 200, 200, 200, 200, 200, 200, 200, 200,     200, 200, 200, 200, 200, 200, 200, 200, 200, 200, 200, 200, 200, 200, 200,     200, 200, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,     0, 0, 0, 0, 0, 0, 0, 0, 0, 0};

void StepMotor(enum Motor m, enum Direction d) {
	const int power = 1;
	if (d != dStopped)
		activateMotors();
	//__HAL_TIM_GetCounter(hardware.htim14);
	switch (m) {

	case mLeft: {
		static unsigned int lefti = 0;
		lefti += d;
		static unsigned int highLow = 0;
		if(d == dForward){
			HAL_GPIO_WritePin(GPIOA, GPIO_PIN_1, GPIO_PIN_SET);
		}
		if(d == dBackward){
			HAL_GPIO_WritePin(GPIOA, GPIO_PIN_1, GPIO_PIN_RESET);
		}
		if(highLow){
			HAL_GPIO_WritePin(GPIOA, GPIO_PIN_0, GPIO_PIN_SET);
			highLow = 0;
		}
		else{
			HAL_GPIO_WritePin(GPIOA, GPIO_PIN_0, GPIO_PIN_RESET);
			highLow = 1;
		}



		break;
	}

	case mRight: {
		static unsigned int righti = 0;
		righti += d;
		static unsigned int highLow = 0;
		if(d == dForward){
			HAL_GPIO_WritePin(GPIOA, GPIO_PIN_3, GPIO_PIN_SET);
		}
		if(d == dBackward){
			HAL_GPIO_WritePin(GPIOA, GPIO_PIN_3, GPIO_PIN_RESET);
		}
		if(highLow){
			HAL_GPIO_WritePin(GPIOA, GPIO_PIN_2, GPIO_PIN_SET);
			highLow = 0;
		}
		else{
			HAL_GPIO_WritePin(GPIOA, GPIO_PIN_2, GPIO_PIN_RESET);
			highLow = 1;
		}
		break;

	}
	}

	//	if (HAL_TIM_PWM_Start(hardware.htim2, TIM_CHANNEL_ALL) != HAL_OK) {
	//		Error_Handler();
	//	}
}




enum Button{
	bUp = 157,
	bLeft = 221,
	bRight = 61,
	bDown = 87,
	bOk = 253,
	b1 = 151,
	b2 = 103,
	b3 = 79,
	b4 = 207,
	b5 = 231,
	b6 = 133,
	b7 = 239,
	b8 = 199,
	b9 = 165,
	b0 = 181,
	bStar = 189,
	bHash = 173,
	bRepeat = 255,
	bNull = 1,
};


void urobotMain() {

	HAL_Delay(3000); // recommended as driving motors immediately can make it such that getting a connection to STLink
	// is unreliable due to electrical noise.
	HAL_TIM_Base_Init(hardware.htim14);
	HAL_TIM_Base_Start(hardware.htim14);

	HAL_TIM_Base_Init(hardware.htim16);
	HAL_TIM_Base_Start(hardware.htim16);
	HAL_TIM_Base_Start_IT(hardware.htim16);

	//motor timer start
	if (HAL_TIM_Base_Start(hardware.htim2) != HAL_OK) {
		Error_Handler();
	}
	if (HAL_TIM_PWM_Start(hardware.htim2, TIM_CHANNEL_1) != HAL_OK) {
		Error_Handler();
	}
	if (HAL_TIM_PWM_Start(hardware.htim2, TIM_CHANNEL_2) != HAL_OK) {
		Error_Handler();
	}
	if (HAL_TIM_PWM_Start(hardware.htim2, TIM_CHANNEL_3) != HAL_OK) {
		Error_Handler();
	}
	if (HAL_TIM_PWM_Start(hardware.htim2, TIM_CHANNEL_4) != HAL_OK) {
		Error_Handler();
	}

	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_4, GPIO_PIN_SET); // aux power (needed all the time for IR rec.
	IRLED(0); // turn off IR sensing LEDs

	while (1) {

	}

}

void ButtonPressed(int Cmd){
	static int lastCmd = 0;

	switch (Cmd){

	case bRepeat:
		ButtonPressed(lastCmd);
		//LEDoff();
		break;

	case bUp:{
		timeSinceLastReset = 0;
		thisCmd = bNull;
		leftMotorDirn = dForward;
		rightMotorDirn = dForward;
		leftMotorPeriod = defaultSpeed;
		rightMotorPeriod = defaultSpeed;

		break;
	}
	case bLeft:{
		timeSinceLastReset = 0;
		thisCmd = bNull;
		leftMotorDirn = dForward;
		rightMotorDirn = dBackward;
		leftMotorPeriod = defaultSpeed;
		rightMotorPeriod = defaultSpeed;

		break;
	}
	case bRight:{
		timeSinceLastReset = 0;
		thisCmd = bNull;
		leftMotorDirn = dBackward;
		rightMotorDirn = dForward;
		leftMotorPeriod = defaultSpeed;
		rightMotorPeriod = defaultSpeed;

		break;
	}
	case bDown:{
		timeSinceLastReset = 0;
		thisCmd = bNull;
		leftMotorDirn = dBackward;
		rightMotorDirn = dBackward;
		leftMotorPeriod = defaultSpeed;
		rightMotorPeriod = defaultSpeed;

		break;
	}
	case bOk:
		break;
	case b1:
		shouldCal = True;
		thisCmd = bNull;
		break;
	case b2:
		timeSinceLastReset = 0;
		thisCmd = bNull;
		leftMotorDirn = dForward;
		rightMotorDirn = dForward;
		leftMotorPeriod = defaultSpeed*slowMult;
		rightMotorPeriod = defaultSpeed*slowMult;

		break;
	case b3:
	case b4:
		timeSinceLastReset = 0;
		thisCmd = bNull;
		leftMotorDirn = dForward;
		rightMotorDirn = dBackward;
		leftMotorPeriod = defaultSpeed*slowMult;
		rightMotorPeriod = defaultSpeed*slowMult;
		break;
	case b5:

		HAL_GPIO_WritePin(GPIOF, GPIO_PIN_1, GPIO_PIN_SET);
		HAL_GPIO_WritePin(GPIOA, GPIO_PIN_12, GPIO_PIN_SET);
		kickCount = 0;


		thisCmd = bNull;
		break;
	case b6:
		timeSinceLastReset = 0;
		thisCmd = bNull;
		leftMotorDirn = dBackward;
		rightMotorDirn = dForward;
		leftMotorPeriod = defaultSpeed*slowMult;
		rightMotorPeriod = defaultSpeed*slowMult;
		break;
	case b7:
	case b8:
		timeSinceLastReset = 0;
		thisCmd = bNull;
		leftMotorDirn = dBackward;
		rightMotorDirn = dBackward;
		leftMotorPeriod = defaultSpeed*slowMult;
		rightMotorPeriod = defaultSpeed*slowMult;

		break;
	case b9:
		break;
	case b0:
		checkBatteryState = 1-checkBatteryState;
		showSensor = 1 - showSensor;
		if(showSensor) emittCode = True;
		else emittCode = False;
	case bStar:
		shouldCal = True;
		thisCmd = bNull;
		break;
	case bHash:
		emittCode = True;
		followLine();
		break;

	default:

		break;
	case bNull:
		break;
	};


	if(Cmd != bRepeat && Cmd != bNull)
		lastCmd = Cmd;

}

void followLine(){
	const int first = 500;
	const int second = 1000;
	const int third = 1500;
	int lineThresh = -1370;
	//activateMotors();
	//while(thisCmd== b2 || thisCmd == bNull){
	timeSinceLastReset = 0;
	if(backAccSaved >lineThresh + third){
		leftMotorDirn = dForward;
		rightMotorDirn = dBackward;
		leftMotorPeriod = defaultSpeed*slowMult;
		rightMotorPeriod = defaultSpeed*slowMult;
	}
	else if(backAccSaved >lineThresh + second && backAccSaved < lineThresh + third){
		leftMotorDirn = dForward;
		rightMotorDirn = dBackward;
		leftMotorPeriod = defaultSpeed*slowMult;
		rightMotorPeriod = defaultSpeed*slowMult*10;
	}
	else if(backAccSaved >lineThresh +first && backAccSaved < lineThresh + second) {
		leftMotorDirn = dForward;
		rightMotorDirn = dForward;
		leftMotorPeriod = defaultSpeed*slowMult;
		rightMotorPeriod = defaultSpeed*slowMult;
	}
	else if(backAccSaved >lineThresh -first && backAccSaved < lineThresh + first) {
		leftMotorDirn = dForward;
		rightMotorDirn = dForward;
		leftMotorPeriod = defaultSpeed*slowMult;
		rightMotorPeriod = defaultSpeed*slowMult;
	}
	else if(backAccSaved >lineThresh -second && backAccSaved < lineThresh - first) {
		leftMotorDirn = dForward;
		rightMotorDirn = dForward;
		leftMotorPeriod = defaultSpeed*slowMult;
		rightMotorPeriod = defaultSpeed*slowMult;
	}
	else if(backAccSaved >lineThresh - third && backAccSaved < lineThresh - second) {
		leftMotorDirn = dBackward;
		rightMotorDirn = dForward;
		leftMotorPeriod = defaultSpeed*slowMult*10;
		rightMotorPeriod = defaultSpeed*slowMult;
	}
	else if(backAccSaved <lineThresh -third) {
		leftMotorDirn = dBackward;
		rightMotorDirn = dForward;
		leftMotorPeriod = defaultSpeed*slowMult;
		rightMotorPeriod = defaultSpeed*slowMult;
	}
	//HAL_Delay(150);
	//}
	return;
}


void TableDistance(){
	//star button check back sensor.
	//lightLED(0,500);
	auxOn();
	IRLED(1);
	readAnalog(); // read analogue ready for battery demo.
	HAL_Delay(1);
	IRLED(0);
	LEDoff();
	if(backIR > 3700)
		lightLED(2,1900);
	else
		lightLED(1,1900);
	//lightLED(2,500);


}

/////////// Start of function implimentations.

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin){
	if(GPIO_Pin == GPIO_PIN_10){
		NECHandler();
	}

}
int NECcode[33]; // first is the Auto gain calibration period.
const int necBoundary = 1750;
const int minAG = 10000;
const int repeatBoundary = 12000;
const int maxAG = 15000;

void NECHandler(){

	static int lastDown = 0;
	static int count  = 0;

	int currenttime =  __HAL_TIM_GetCounter(hardware.htim14);

	int thisTime = currenttime-lastDown;

	if(thisTime < 0) thisTime += 0xffff;

	if(count == 0){
		if(thisTime > minAG && thisTime < maxAG){

			if (thisTime < repeatBoundary ){
				thisCmd = bRepeat;
				return;
			} else {
				NECcode[count] = thisTime;
				count++;
			}
		}
	} else {
		if( thisTime < minAG){
			NECcode[count] = thisTime;
			count++;
			if(count == 33){
				parseNECTimes();
				count = 0;
			}
		}
		else{
			count = 0;
		}
	}
	lastDown = currenttime;
}

//void NECHandler(){
//	static int isFalling = 0;
//	static int lastDown = 0;
//	static int count  = 0;
//	if(HAL_GPIO_ReadPin(GPIOA,GPIO_PIN_10)){
//		isFalling = 1;
//		lastDown = __HAL_TIM_GetCounter(hardware.htim14);
//	}
//	else{
//		isFalling = 0;
//		int thisTime = __HAL_TIM_GetCounter(hardware.htim14)-lastDown;
//
//		if(thisTime < 0) thisTime += 0xffff;
//
//		if(count == 0){
//			if (thisTime < necBoundary){
//				thisCmd = bRepeat;
//				return;
//			}
//			if(thisTime > minAG && thisTime < maxAG){
//				NECcode[count] = thisTime;
//				count++;
//			}
//		} else {
//			if( thisTime < minAG){
//				NECcode[count] = thisTime;
//				count++;
//				if(count == 33){
//					parseNECTimes();
//					count = 0;
//				}
//			}
//			else{
//				count = 0;
//			}
//		}
//	}
//}

union message{
	int full;
	char arr[4];
}thisMes;
void parseNECTimes(){
	thisMes.full = 0;
	unsigned int mask = 0x80000000;
	int i = 0 ;
	for(i = 1; i < 33; i++){
		if(NECcode[i] > necBoundary){
			thisMes.full = thisMes.full | mask;

		}
		mask = mask >> 1;
	}
	int valid = 1;
	if(thisMes.arr[0]== ~thisMes.arr[1]){
		valid = 0;
	}
	if(thisMes.arr[2]== ~thisMes.arr[3]){
		valid = 0;
	}
	thisCmd = thisMes.arr[0];
}


void transmitIRburst(){
	//demo writing to transmit led
	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_9, GPIO_PIN_SET);
	HAL_Delay(500);
	HAL_TIM_PWM_Start(hardware.htim3, TIM_CHANNEL_2);
	__HAL_TIM_SetCompare(hardware.htim3, TIM_CHANNEL_2,63);
	HAL_Delay(500);
	__HAL_TIM_SetCompare(hardware.htim3, TIM_CHANNEL_2,127);
	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_9, GPIO_PIN_RESET);
}

void readAnalog(){
	HAL_ADC_Start_IT(hardware.hadc);
}
// colour 2 is blue out of 2000
//colour 1 is green out of 2000
//colour 0 is ret out of 128
void lightLED(int colour,int value){
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_4, GPIO_PIN_SET); // aux power
	if(colour == 0){
		HAL_TIM_PWM_Start(hardware.htim3,TIM_CHANNEL_3);
		__HAL_TIM_SetCompare(hardware.htim3, TIM_CHANNEL_3,value);
	}
	if(colour == 1){
		HAL_TIM_PWM_Start(hardware.htim17,TIM_CHANNEL_1);
		__HAL_TIM_SetCompare(hardware.htim17, TIM_CHANNEL_1,value);
	}
	if(colour == 2){
		HAL_TIM_PWM_Start(hardware.htim1,TIM_CHANNEL_1);
		__HAL_TIM_SetCompare(hardware.htim1, TIM_CHANNEL_1,value);
	}
	HAL_TIM_IC_Start_IT(hardware.htim1, TIM_CHANNEL_3);
}

void LEDoff(){
	//HAL_GPIO_WritePin(GPIOB, GPIO_PIN_4, GPIO_PIN_RESET); // aux power
	__HAL_TIM_SetCompare(hardware.htim3, TIM_CHANNEL_3,127);
	__HAL_TIM_SetCompare(hardware.htim17, TIM_CHANNEL_1,2001);
	__HAL_TIM_SetCompare(hardware.htim1, TIM_CHANNEL_1,2001);
}

//auxiliary power.
void auxOn(){
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_4, GPIO_PIN_SET); // aux power
}

void auxOff(){
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_4, GPIO_PIN_RESET); // aux power
}


void motorsMiddle(){

//	__HAL_TIM_SetCompare(hardware.htim2, TIM_CHANNEL_1,100);
//	__HAL_TIM_SetCompare(hardware.htim2, TIM_CHANNEL_2,100);
//	__HAL_TIM_SetCompare(hardware.htim2, TIM_CHANNEL_3,100);
//	__HAL_TIM_SetCompare(hardware.htim2, TIM_CHANNEL_4,100);
}

int motorsAreActive = 0;

void activateMotors(){
	//	if (motorsAreActive == 1)
	//		return;
	//
	//	motorsAreActive = 1;
	//
	//	//start PWM on all channels
	//	HAL_TIM_PWM_Start(hardware.htim2,TIM_CHANNEL_1);
	//	HAL_TIM_PWM_Start(hardware.htim2,TIM_CHANNEL_2);
	//	HAL_TIM_PWM_Start(hardware.htim2,TIM_CHANNEL_3);
	//	HAL_TIM_PWM_Start(hardware.htim2,TIM_CHANNEL_4);
	HAL_GPIO_WritePin(GPIOF, GPIO_PIN_1, GPIO_PIN_SET); // motor power pin
}
void deactivateMotors(){
	//	if (motorsAreActive == 0)
	//		return;
	//
	//	motorsAreActive = 0;
	//	//start PWM on all channels
	//	HAL_TIM_PWM_Stop(hardware.htim2,TIM_CHANNEL_1);
	//	HAL_TIM_PWM_Stop(hardware.htim2,TIM_CHANNEL_2);
	//	HAL_TIM_PWM_Stop(hardware.htim2,TIM_CHANNEL_3);
	//	HAL_TIM_PWM_Stop(hardware.htim2,TIM_CHANNEL_4);
	//motorsMiddle();
	HAL_GPIO_WritePin(GPIOF, GPIO_PIN_1, GPIO_PIN_RESET); // motor power pin
}

void IRLED(int on){
	if(on){
		HAL_GPIO_WritePin(GPIOB, GPIO_PIN_3, GPIO_PIN_RESET); // motor power pin
	}
	else{
		HAL_GPIO_WritePin(GPIOB, GPIO_PIN_3, GPIO_PIN_SET); // motor power pin
	}
}

void delayTime(unsigned int loops){
	int i = 0;
	for(i = 0; i < loops; i++){

	}
}

void displayBattVoltage(){
	if(!HAL_GPIO_ReadPin(GPIOA,GPIO_PIN_15)){
		lightLED(1,1950);
		lightLED(2,1950);
		lightLED(0,115);
		int i = 0;
		for(i = 0; i < 50000; i++){}
		lightLED(1,2001);
		lightLED(2,2001);
		lightLED(0,130);
		//return;
	}
	if(voltage > 8.0){
		lightLED(1,1900);
		return;
	}
	else if(voltage > 7.6){
		lightLED(2,1900);
		return;

	}
	else {
		lightLED(0,110);
		return;
	}
}

void StandbyMode_Measure(void)
{
	/* Enable Power Clock*/
	__HAL_RCC_PWR_CLK_ENABLE();

	/* Allow access to Backup */
	HAL_PWR_EnableBkUpAccess();

	/* Reset RTC Domain */
	__HAL_RCC_BACKUPRESET_FORCE();
	__HAL_RCC_BACKUPRESET_RELEASE();

	/* The Following Wakeup sequence is highly recommended prior to each Standby mode entry
     mainly  when using more than one wakeup source this is to not miss any wakeup event.
       - Disable all used wakeup sources,
       - Clear all related wakeup flags,
       - Re-enable all used wakeup sources,
       - Enter the Standby mode.
	 */

	/*#### Disable all used wakeup sources: WKUP pin ###########################*/
	HAL_PWR_DisableWakeUpPin(PWR_WAKEUP_PIN1);


	/*#### Clear all related wakeup flags ######################################*/
	/* Clear PWR wake up Flag */
	__HAL_PWR_CLEAR_FLAG(PWR_FLAG_WU);

	/* Enable WKUP pin, for now we will rely on RTC
  //HAL_PWR_EnableWakeUpPin(PWR_WAKEUP_PIN1);

  /* Request to enter STANDBY mode */
	HAL_PWR_EnterSTANDBYMode();
}

void StopMode_Measure(void)
{
	GPIO_InitTypeDef GPIO_InitStruct;

	/* Configure all GPIO as analog to reduce current consumption on non used IOs */
	/* Warning : Reconfiguring all GPIO will close the connexion with the debugger */
	/* Enable GPIOs clock */

	__HAL_RCC_GPIOA_CLK_ENABLE();
	__HAL_RCC_GPIOB_CLK_ENABLE();
	__HAL_RCC_GPIOC_CLK_ENABLE();

	__HAL_RCC_GPIOF_CLK_ENABLE();


	GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Pin = GPIO_PIN_All;
	HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);
	HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);
	HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);
	HAL_GPIO_Init(GPIOF, &GPIO_InitStruct);


	/* Disable GPIOs clock */
	__HAL_RCC_GPIOA_CLK_DISABLE();
	__HAL_RCC_GPIOB_CLK_DISABLE();
	__HAL_RCC_GPIOC_CLK_DISABLE();

	__HAL_RCC_GPIOF_CLK_DISABLE();





	/* Enter Stop Mode */
	HAL_PWR_EnterSTOPMode(PWR_LOWPOWERREGULATOR_ON, PWR_STOPENTRY_WFE);

	/* Configures system clock after wake-up from STOP: enable HSI and PLL with HSI as source*/
	SYSCLKConfig_STOP();



	/* Inserted Delay */


}
static void SYSCLKConfig_STOP(void)
{
	RCC_ClkInitTypeDef RCC_ClkInitStruct;
	RCC_OscInitTypeDef RCC_OscInitStruct;
	uint32_t pFLatency = 0;

	/* Get the Oscillators configuration according to the internal RCC registers */
	HAL_RCC_GetOscConfig(&RCC_OscInitStruct);

	/* Activate PLL with HSI as source */
	RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_NONE;
	RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
	RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
	RCC_OscInitStruct.PLL.PREDIV = RCC_PREDIV_DIV1;
	RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL12;
	if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
	{
		Error_Handler();
	}

	/* Get the Clocks configuration according to the internal RCC registers */
	HAL_RCC_GetClockConfig(&RCC_ClkInitStruct, &pFLatency);

	/* Select PLL as system clock source and configure the HCLK and PCLK1 clocks dividers */
	RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_SYSCLK;
	RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
	if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, pFLatency) != HAL_OK)
	{
		Error_Handler();
	}


}

static void RTC_AlarmConfig(unsigned int ms)
{

	unsigned int seconds = ms/1000;
	unsigned int mSeconds = 0; //currently will only do round times as I would need to measure the current subseconds feild, as it is not writable.
	RTC_DateTypeDef  sdatestructure;
	RTC_TimeTypeDef  stimestructure;
	RTC_AlarmTypeDef salarmstructure;

	/*##-1- Configure the Date #################################################*/
	/* Set Date: Tuesday February 18th 2014 */
	sdatestructure.Year = 0x14;
	sdatestructure.Month = RTC_MONTH_FEBRUARY;
	sdatestructure.Date = 0x18;
	sdatestructure.WeekDay = RTC_WEEKDAY_TUESDAY;

	if(HAL_RTC_SetDate(hardware.hrtc,&sdatestructure,RTC_FORMAT_BCD) != HAL_OK)
	{
		/* Initialization Error */
		Error_Handler();
	}

	/*##-2- Configure the Time #################################################*/
	/* Set Time: 02:20:00 */
	stimestructure.Hours = 0x02;
	stimestructure.Minutes = 0x20;
	stimestructure.Seconds = 0x00;
	stimestructure.TimeFormat = RTC_HOURFORMAT12_AM;
	stimestructure.DayLightSaving = RTC_DAYLIGHTSAVING_NONE ;
	stimestructure.StoreOperation = RTC_STOREOPERATION_RESET;

	if(HAL_RTC_SetTime(hardware.hrtc,&stimestructure,RTC_FORMAT_BCD) != HAL_OK)
	{
		/* Initialization Error */
		Error_Handler();
	}

	/*##-3- Configure the RTC Alarm peripheral #################################*/
	/* Set Alarm to 02:20:30
     RTC Alarm Generation: Alarm on Hours, Minutes and Seconds */
	salarmstructure.Alarm = RTC_ALARM_A;
	salarmstructure.AlarmDateWeekDay = RTC_WEEKDAY_MONDAY;
	salarmstructure.AlarmDateWeekDaySel = RTC_ALARMDATEWEEKDAYSEL_DATE;
	salarmstructure.AlarmMask = RTC_ALARMMASK_DATEWEEKDAY;
	salarmstructure.AlarmSubSecondMask = RTC_ALARMSUBSECONDMASK_NONE;
	salarmstructure.AlarmTime.TimeFormat = RTC_HOURFORMAT12_AM;
	salarmstructure.AlarmTime.Hours = 0x02;
	salarmstructure.AlarmTime.Minutes = 0x20;
	salarmstructure.AlarmTime.Seconds = seconds;
	salarmstructure.AlarmTime.SubSeconds = 0x56;

	if(HAL_RTC_SetAlarm_IT(hardware.hrtc,&salarmstructure,RTC_FORMAT_BCD) != HAL_OK)
	{
		/* Initialization Error */
		Error_Handler();
	}
}
