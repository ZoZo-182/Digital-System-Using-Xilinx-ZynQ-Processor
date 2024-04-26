/*
 * main.c
 *
 *  Created on: Apr 18, 2018
 *      Author: arthur
 */

//#include "PWM.h"
#include "xsysmon.h"
#include "xparameters.h"
#include "sleep.h"
#include "stdio.h"
#include "xgpio.h"
#include "xil_types.h"
#include "xil_exception.h"
#include "xbasic_types.h"
#include "string.h"
#include "xtmrctr.h"
#include "stdlib.h"

#ifdef XPAR_INTC_0_DEVICE_ID
 #include "xintc.h"
 #include <stdio.h>
#else
 #include "xscugic.h"
 #include "xil_printf.h"
#include "stdbool.h"
#endif

#define LED_ON_DUTY 0x3FFF
#define LED_OFF_DUTY 0x3000
#define XADC_DEVICE_ID XPAR_XADC_WIZ_0_DEVICE_ID
#define BTN_DEVICE_ID XPAR_AXI_GPIO_0_DEVICE_ID
#define XADC_SEQ_CHANNELS 0x0420000
#define XADC_CHANNELS 0x0420000

#define Test_Bit(VEC,BIT) ((VEC&(1<<BIT))!=0)

/************************** Constant Definitions (Ultra sonic) *****************************/
/*
 * The following constants map to the XPAR parameters created in the
 * xparameters.h file. They are only defined here such that a user can easily
 * change all the needed parameters in one place.
 */
#define TMRCTR_DEVICE_ID        XPAR_TMRCTR_0_DEVICE_ID
#define TMRCTR_1_DEVICE_ID        XPAR_TMRCTR_1_DEVICE_ID


#define TMRCTR_INTERRUPT_ID     XPAR_FABRIC_TMRCTR_0_VEC_ID
#define TMRCTR_1_INTERRUPT_ID     XPAR_FABRIC_TMRCTR_1_VEC_ID


#ifdef XPAR_INTC_0_DEVICE_ID
#define INTC_DEVICE_ID          XPAR_INTC_0_DEVICE_ID
#define INTC                    XIntc
#define INTC_HANDLER            XIntc_InterruptHandler
#else
#define INTC_DEVICE_ID          XPAR_SCUGIC_SINGLE_DEVICE_ID
#define INTC                    XScuGic
#define INTC_HANDLER            XScuGic_InterruptHandler
#endif /* XPAR_INTC_0_DEVICE_ID */

#define CLK_PERIOD              20             /* System clock period in ns */
#define PWM_PERIOD              20000000       /* PWM period in ns */  //change to 1000000000 to slow down
#define DUTY_PERIOD             15000          /* DUTY period in ns */
#define TMRCTR_0_0                0            /* Timer 0 ID */
#define TMRCTR_0_1                1            /* Timer 1 ID */
#define TMRCTR_1_0                0            /* Timer 0 ID */
#define TMRCTR_1_1                1            /* Timer 1 ID */
/*******************************************************************************************/
#define CLK_PERIOD              20    /* system clock period in ( ms) */
#define PWM_PERIOD_1            20000000    /* PWM period in (20 ms) */
#define DUTY_PERIOD             15000    /* DUTY period in ( ms) */
#define TMRCTR_0_0                0            /* Timer 0 ID */
#define TMRCTR_0_1                1            /* Timer 1 ID */
#define TMRCTR_1_0                0            /* Timer 0 ID */
#define TMRCTR_1_1                1            /* Timer 1 ID */

///************************** PWM OUT *****************************/
#define TMRCTR_DEVICE_ID_1        XPAR_TMRCTR_2_DEVICE_ID
#define TMRCTR_DEVICE_ID_PMOD	  XPAR_TMRCTR_4_DEVICE_ID
//
//
//
//
//#define INTC_DEVICE_ID          XPAR_SCUGIC_SINGLE_DEVICE_ID
//#define INTC                    XScuGic
//#define INTC_HANDLER            XScuGic_InterruptHandler
//
//#define PWM_PERIOD              5000000   /* PWM period in (50 ms) */ //(why 50?)
#define TMRCTR_0                0            /* Timer 0 ID */
#define TMRCTR_1                1            /* Timer 1 ID */
//#define CYCLE_PER_DUTYCYCLE     10           /* Clock cycles per duty cycle */
//#define MAX_DUTYCYCLE           100          /* Max duty cycle */
//#define DUTYCYCLE_DIVISOR       5            /* Duty cycle Divisor */
//#define WAIT_COUNT              PWM_PERIOD   /* Interrupt wait counter */
//

/************************** Constant Definitions *****************************/
#ifndef TESTAPP_GEN

#define GPIO_DEVICE_ID XPAR_GPIO_0_DEVICE_ID
#define GPIO_CHANNEL1 1

#ifdef XPAR_INTC_0_DEVICE_ID
 #define INTC_GPIO_INTERRUPT_ID XPAR_INTC_0_GPIO_0_VEC_ID
 #define INTC_DEVICE_ID XPAR_INTC_0_DEVICE_ID
#else
 #define INTC_GPIO_INTERRUPT_ID XPAR_FABRIC_AXI_GPIO_0_IP2INTC_IRPT_INTR
 #define INTC_DEVICE_ID XPAR_SCUGIC_SINGLE_DEVICE_ID
#endif /* XPAR_INTC_0_DEVICE_ID */


#define GPIO_ALL_LEDS 0xFFFF
#define GPIO_ALL_BUTTONS 0xFFFF


#define BUTTON_CHANNEL 1 /* Channel 1 of the GPIO Device */
#define LED_CHANNEL 2 /* Channel 2 of the GPIO Device */
#define BUTTON_INTERRUPT XGPIO_IR_CH1_MASK  /* Channel 1 Interrupt Mask */


#define INTERRUPT_CONTROL_VALUE 0x7

#define LED_DELAY 1000000

#endif /* TESTAPP_GEN */

#define INTR_DELAY 0x0FFFFFFF

#ifdef XPAR_INTC_0_DEVICE_ID
 #define INTC_DEVICE_ID XPAR_INTC_0_DEVICE_ID
 #define INTC XIntc
 #define INTC_HANDLER XIntc_InterruptHandler
#else
 #define INTC_DEVICE_ID XPAR_SCUGIC_SINGLE_DEVICE_ID
 #define INTC XScuGic
 #define INTC_HANDLER XScuGic_InterruptHandler
#endif /* XPAR_INTC_0_DEVICE_ID */

/************************** Function Prototypes ******************************/
void buzzer(float distance, int enable);

void GpioHandler(void *CallBackRef);

int GpioIntrExample(INTC *IntcInstancePtr, XGpio *InstancePtr,
u16 DeviceId, u16 IntrId,
u16 IntrMask, u32 *DataRead);

int GpioSetupIntrSystem(INTC *IntcInstancePtr, XGpio *InstancePtr,
u16 DeviceId, u16 IntrId, u16 IntrMask);

void GpioDisableIntr(INTC *IntcInstancePtr, XGpio *InstancePtr,
u16 IntrId, u16 IntrMask);

///************************** Function Prototypes PWM ******************************/
int TmrCtrPwmMotor(XTmrCtr *InstancePtr, u16 DeviceId);
//u16 IntrId, u32 HighTime);
static void TimerCounterHandler(void *CallBackRef, u8 TmrCtrNumber);
//static int TmrCtrSetupIntrSystem(INTC *IntcInstancePtr, XTmrCtr *InstancePtr,
// u16 DeviceId, u16 IntrId);
//static void TmrCtrDisableIntr(INTC *IntcInstancePtr, u16 IntrId);

/**************************** Type Definitions *******************************/

/***************** Macros (Inline Functions) Definitions *********************/

/************************** Function Prototypes(ultra) ******************************/
int TmrCtrCapture(INTC *IntcInstancePtr, XTmrCtr *InstancePtr, u16 DeviceId,
u16 IntrId);
static void TimerCounterHandler_0(void *CallBackRef, u8 TmrCtrNumber);
static void TimerCounterHandler_1(void *CallBackRef, u8 TmrCtrNumber);
static int TmrCtrSetupIntrSystem(INTC *IntcInstancePtr, XTmrCtr *InstancePtr,
u16 DeviceId, u16 IntrId);
int TmrCtrPwmExample(INTC *IntcInstancePtr, XTmrCtr *InstancePtr, u16 DeviceId,
u16 IntrId);
/************************** Variable Definitions (ultra) *****************************/
INTC InterruptController;  /* The instance of the Interrupt Controller */
XTmrCtr TimerCounterInst;  /* The instance of the Timer Counter */
XTmrCtr TimerCaptureInst;  /* The instance of the Timer Counter */
/*
 * The following variables are shared between non-interrupt processing and
 * interrupt processing such that they must be global.
 */
static int   PeriodTimerHit = FALSE;
static int   HighTimerHit = FALSE;
static u32   capture0, capture1, pulsewidth;
float distance;
static u32 myRawDataPot;
static u32 myRawDataPhoto;
/************************** Variable Definitions *****************************/
//(PWM STUFF)
XTmrCtr TimerCounterInstMotor;  /* The instance of the Timer Counter */
//static int   PeriodTimerHit = FALSE;
//static int   HighTimerHit = FALSE;
XGpio Gpio; /* The Instance of the GPIO Driver */
INTC Intc; /* The Instance of the Interrupt Controller Driver */
static u16 GlobalIntrMask; /* GPIO channel mask that is needed by * the Interrupt Handler */
static volatile u32 IntrFlag = 0; /* Interrupt Handler Flag */
u32 count = 1;
u32 udata;
Xuint32 *baseaddr_p = (Xuint32 *)XPAR_MYLCD_0_S00_AXI_BASEADDR;


void Xadc_Init(XSysMon *InstancePtr, u32 DeviceId) {
XSysMon_Config *ConfigPtr;
ConfigPtr = XSysMon_LookupConfig(DeviceId);
XSysMon_CfgInitialize(InstancePtr, ConfigPtr, ConfigPtr->BaseAddress);
// Disable the Channel Sequencer before configuring the Sequence registers.
XSysMon_SetSequencerMode(InstancePtr, XSM_SEQ_MODE_SAFE);
// Disable all alarms
XSysMon_SetAlarmEnables(InstancePtr, 0x0);
// Set averaging for all channels to 16 samples
XSysMon_SetAvg(InstancePtr, XSM_AVG_16_SAMPLES);
// Set differential input mode for all channels
// XSysMon_SetSeqInputMode(InstancePtr, XADC_SEQ_CHANNELS);
// Set 6ADCCLK acquisition time in all channels
XSysMon_SetSeqAcqTime(InstancePtr, XADC_SEQ_CHANNELS);
// Disable averaging in all channels
XSysMon_SetSeqAvgEnables(InstancePtr, XADC_SEQ_CHANNELS);
// Enable all channels
XSysMon_SetSeqChEnables(InstancePtr, XADC_SEQ_CHANNELS);
// Set the ADCCLK frequency equal to 1/32 of System clock
XSysMon_SetAdcClkDivisor(InstancePtr, 32);
// Enable Calibration
XSysMon_SetCalibEnables(InstancePtr, XSM_CFR1_CAL_PS_GAIN_OFFSET_MASK | XSM_CFR1_CAL_ADC_GAIN_OFFSET_MASK);
// Enable the Channel Sequencer in continuous sequencer cycling mode
XSysMon_SetSequencerMode(InstancePtr, XSM_SEQ_MODE_CONTINPASS);
// Clear the old status
// XSysMon_GetStatus(InstancePtr);
}

#define READDATA_DBG 0

#ifndef TESTAPP_GEN
int main () {
XSysMon Xadc;
XGpio Btn;
XGpio Led;
u32 data;
u32 time_count = 0;
int Status;
int Status2;
u32 DataRead;
int systemEnabled = 1;
int reset = 0;
int inWhileLoop = 1;
bool prevButtonState = false; // Variable to track previous button state
bool prevSystemEnabledState = false;

// Initialize the GPIO for the button
   Status = XGpio_Initialize(&Btn, BTN_DEVICE_ID);
   if (Status != XST_SUCCESS) {
       //print("Error initializing GPIO for button\r\n");
       return XST_FAILURE;
   }

Status = GpioIntrExample(&Intc, &Gpio,
  GPIO_DEVICE_ID,
  INTC_GPIO_INTERRUPT_ID,
  GPIO_CHANNEL1, &DataRead);


Xadc_Init(&Xadc, XADC_DEVICE_ID);

/* Run the Timer Counter PWM example to setup the Trigger on the sensor */
Status = TmrCtrPwmExample(&Intc, &TimerCounterInst,
 TMRCTR_1_DEVICE_ID, TMRCTR_1_INTERRUPT_ID);

/* Run the Timer Counter PWM example to setup the Trigger on the sensor */
Status = TmrCtrCapture(&Intc, &TimerCaptureInst,
 TMRCTR_DEVICE_ID, TMRCTR_INTERRUPT_ID);
//u32 myRawData = XSysMon_GetAdcData(&Xadc, currentChannel);



while(1) {
        if (IntrFlag) {
            // Clear the interrupt flag
            IntrFlag = 0;
        }


data = XGpio_DiscreteRead(&Gpio, 1);

// toggle between the two channels
static u8 currentChannel = 17;
if (data & 0x2) {
if (!prevButtonState ) {
currentChannel = (currentChannel == 17) ? 22 : 17; // Toggle between channels
   if (currentChannel == 22) {
       //strncat(str, "22", 2);
	   LCD_WriteLine2("Photoresistor   ");
   } else {
       //strncat(str, "17", 2);
	   LCD_WriteLine2("Potentiometer   ");
   }
}
prevButtonState = true;
} else {
prevButtonState = false;
 }

if (data & 0x1) { // btn 0
// reset the system at any point in time
reset = !reset;
if (reset) {

}
}

if (data & 0x4) { // btn 2
if (!prevSystemEnabledState) {
// select between system enable and disable
systemEnabled = !systemEnabled; // Toggle system state
if (systemEnabled) {
enableSystem();
inWhileLoop = 1;


} else {
   LCD_WriteLine1("System Disabled ");
   inWhileLoop = 0;
}
}
prevSystemEnabledState = true;
} else {
prevSystemEnabledState = false;
}

if (inWhileLoop) {
time_count ++;
if (time_count == 100000) { // print channel reading approx. 10x per second
time_count = 0;


if (currentChannel == 17){
 myRawDataPot = XSysMon_GetAdcData(&Xadc, 17);
}
else if (currentChannel == 22)
 myRawDataPhoto = XSysMon_GetAdcData(&Xadc, 22);

// //Run the Timer Counter PWM example
Status = TmrCtrPwmMotor(&TimerCounterInst,
TMRCTR_DEVICE_ID_1);


Status2 = TmrCtrPwmMotor(&TimerCounterInst,
		TMRCTR_DEVICE_ID_PMOD);


distance = (float)pulsewidth*0.020/148; // distance in inches
       printf("Distance %5.2F inches \r\n", distance);

//servo
Xuint32 *baseaddr_ps = (Xuint32 *)XPAR_PWMSERVO_0_S00_AXI_BASEADDR;
*baseaddr_ps = myRawDataPot;
}


int lcd_distance = (int)distance;
	char lcd_num[3];
	itoa(lcd_distance, lcd_num,10);
	char string[16] = "Distance: ";
	strcat(string, lcd_num);
	strcat(string, "in");


if(distance > 24){
				LCD_WriteLine1("System Enabled  ");
				XGpio_DiscreteWrite(&Gpio, 2, 0x2);
				//buzzer code
}
			else if(distance > 12){
				LCD_WriteLine1(string);
				XGpio_DiscreteWrite(&Gpio, 2, 0x6);
			}
				//buzzer code
			else{
				LCD_WriteLine1(string);
				XGpio_DiscreteWrite(&Gpio, 2, 0x4);
			}
				//buzzer code

//----------------------------------------------------------------
//Buzzer

buzzer(distance, systemEnabled);

//----------------------------------------------------------------
}
usleep(1);
}


if (Status == 0 ){
//if(DataRead == 0)
//print("No button pressed. \r\n");
//else
//print("Successfully ran Gpio Interrupt Tapp Example\r\n");
} else {
//print("Gpio Interrupt Tapp Example Failed.\r\n");
return XST_FAILURE;
}

return XST_SUCCESS;
}
#endif



void buzzer(float distance, int enable)
{
	u32 pwmmasks;
	float HighTime;
	float Period;
	float DutyCycle;
	float d;
	if (enable == 1){
		if (distance >= 24){
			d = 0;
		}
		else{
			d = ((distance - 24) * -1) * 1000;

		}
		Period = 5000000 / ((d/300) + 20);   //derek math
//		printf("D: ", d);
		HighTime = Period/2;
		DutyCycle = HighTime / Period;

		Xil_Out32(0x42830000 + 4, Period/20);
		Xil_Out32(0x42830000 + 4, HighTime/20);


		pwmmasks = XTC_CSR_ENABLE_INT_MASK | XTC_CSR_DOWN_COUNT_MASK | XTC_CSR_AUTO_RELOAD_MASK;
		pwmmasks |= XTC_CSR_ENABLE_PWM_MASK | XTC_CSR_EXT_GENERATE_MASK|XTC_CSR_ENABLE_ALL_MASK;
		Xil_Out32(0x42830000, pwmmasks);
		Xil_Out32(0x42830000, pwmmasks);
	}
	else{
		Period = 0;
		HighTime = Period/2;
		DutyCycle = HighTime / Period;

		Xil_Out32(0x42830000 + 4, Period/20);
		Xil_Out32(0x42830000 + 4, HighTime/20);

		pwmmasks = XTC_CSR_ENABLE_INT_MASK | XTC_CSR_DOWN_COUNT_MASK | XTC_CSR_AUTO_RELOAD_MASK;
		pwmmasks |= XTC_CSR_ENABLE_PWM_MASK | XTC_CSR_EXT_GENERATE_MASK|XTC_CSR_ENABLE_ALL_MASK;
		Xil_Out32(0x42830000, pwmmasks);
		Xil_Out32(0x42830000, pwmmasks);

	}
}



void enableSystem() {
XSysMon Xadc;
XGpio Btn;
int Status;
u32 DataRead;
// Initialize the GPIO for the button
Status = XGpio_Initialize(&Btn, BTN_DEVICE_ID);
if (Status != XST_SUCCESS) {
//print("Error initializing GPIO for button\r\n");
return XST_FAILURE;
}

Status = GpioIntrExample(&Intc, &Gpio,
GPIO_DEVICE_ID,
  INTC_GPIO_INTERRUPT_ID,
  GPIO_CHANNEL1, &DataRead);

Xadc_Init(&Xadc, XADC_DEVICE_ID);
return;
}


float pwmServo(u32 rawData) {
 return ((((0.025 * PWM_PERIOD_1) + ((0.10 * PWM_PERIOD_1)/65000)) * rawData) * 2.5);
}

int GpioIntrExample(INTC *IntcInstancePtr, XGpio* InstancePtr, u16 DeviceId,
u16 IntrId, u16 IntrMask, u32 *DataRead)
{
int Status;
u32 delay;

/* Initialize the GPIO driver. If an error occurs then exit */
Status = XGpio_Initialize(InstancePtr, DeviceId);
if (Status != XST_SUCCESS) {
return XST_FAILURE;
}

Status = GpioSetupIntrSystem(IntcInstancePtr, InstancePtr, DeviceId,
IntrId, IntrMask);
if (Status != XST_SUCCESS) {
return XST_FAILURE;
}

IntrFlag = 0;
delay = 0;

return Status;
}


int GpioSetupIntrSystem(INTC *IntcInstancePtr, XGpio *InstancePtr,
u16 DeviceId, u16 IntrId, u16 IntrMask)
{
int Result;

GlobalIntrMask = IntrMask;

#ifdef XPAR_INTC_0_DEVICE_ID

#else /* !XPAR_INTC_0_DEVICE_ID */

#ifndef TESTAPP_GEN
XScuGic_Config *IntcConfig;

/*
* Initialize the interrupt controller driver so that it is ready to
* use.
*/
IntcConfig = XScuGic_LookupConfig(INTC_DEVICE_ID);
if (NULL == IntcConfig) {
return XST_FAILURE;
}

Result = XScuGic_CfgInitialize(IntcInstancePtr, IntcConfig,
IntcConfig->CpuBaseAddress);
if (Result != XST_SUCCESS) {
return XST_FAILURE;
}
#endif /* TESTAPP_GEN */

XScuGic_SetPriorityTriggerType(IntcInstancePtr, IntrId,
0xA0, 0x3);

/*
* Connect the interrupt handler that will be called when an
* interrupt occurs for the device.
*/
Result = XScuGic_Connect(IntcInstancePtr, IntrId,
(Xil_ExceptionHandler)GpioHandler, InstancePtr);
if (Result != XST_SUCCESS) {
return Result;
}

/* Enable the interrupt for the GPIO device.*/
XScuGic_Enable(IntcInstancePtr, IntrId);
#endif /* XPAR_INTC_0_DEVICE_ID */


XGpio_InterruptEnable(InstancePtr, IntrMask);
XGpio_InterruptGlobalEnable(InstancePtr);


Xil_ExceptionInit();

Xil_ExceptionRegisterHandler(XIL_EXCEPTION_ID_INT,
(Xil_ExceptionHandler)INTC_HANDLER, IntcInstancePtr);

/* Enable non-critical exceptions */
Xil_ExceptionEnable();

return XST_SUCCESS;
}


void GpioHandler(void *CallbackRef)
{
XGpio *GpioPtr = (XGpio *)CallbackRef;
count++;
u32 data = XGpio_DiscreteRead(GpioPtr, 1);
IntrFlag = 1;

/* Clear the Interrupt */
XGpio_InterruptClear(GpioPtr, GlobalIntrMask);
}


void GpioDisableIntr(INTC *IntcInstancePtr, XGpio *InstancePtr,
u16 IntrId, u16 IntrMask)
{
XGpio_InterruptDisable(InstancePtr, IntrMask);
#ifdef XPAR_INTC_0_DEVICE_ID
XIntc_Disable(IntcInstancePtr, IntrId);
#else
/* Disconnect the interrupt */
XScuGic_Disable(IntcInstancePtr, IntrId);
XScuGic_Disconnect(IntcInstancePtr, IntrId);
#endif
return;
}

void LCD_WriteLine1(const char str[16]) {
u32 EQ = 0;
EQ = (str[0] << 24 | 0) | (str[1] << 16 | 0) | (str[2] << 8 | 0) | (str[3] | 0);
    *(baseaddr_p+0) = EQ;
EQ = (str[4] << 24 | 0) | (str[5] << 16 | 0) | (str[6] << 8 | 0) | (str[7] | 0);
    *(baseaddr_p+1) = EQ;
EQ = (str[8] << 24 | 0) | (str[9] << 16 | 0) | (str[10] << 8 | 0) | (str[11] | 0);
    *(baseaddr_p+2) = EQ;
EQ = (str[12] << 24 | 0) | (str[13] << 16 | 0) | (str[14] << 8 | 0) | (str[15] | 0);
    *(baseaddr_p+3) = EQ;
}

void LCD_WriteLine2(const char str[16]) {
u32 EQ = 0;
EQ = (str[0] << 24 | 0) | (str[1] << 16 | 0) | (str[2] << 8 | 0) | (str[3] | 0);
    *(baseaddr_p+4) = EQ;
EQ = (str[4] << 24 | 0) | (str[5] << 16 | 0) | (str[6] << 8 | 0) | (str[7] | 0);
    *(baseaddr_p+5) = EQ;
EQ = (str[8] << 24 | 0) | (str[9] << 16 | 0) | (str[10] << 8 | 0) | (str[11] | 0);
    *(baseaddr_p+6) = EQ;
EQ = (str[12] << 24 | 0) | (str[13] << 16 | 0) | (str[14] << 8 | 0) | (str[15] | 0);
    *(baseaddr_p+7) = EQ;
}


///****************************************************************** PWM FUNCTIONS **************************************************************************************/
int TmrCtrPwmMotor(XTmrCtr *TmrCtrInstancePtr, u16 DeviceId)
{
// u8  DutyCycle;
// float DutyCycle_percent;
// u8  NoOfCycles;
float  Div;
float  Div2;
u32 Period;
u32 HighTime;
u32 Period2;
u32 HighTime2;
// u64 WaitCount;
int Status;

/*
* Initialize the timer counter so that it's ready to use,
* specify the device ID that is generated in xparameters.h
*/
Status = XTmrCtr_Initialize(TmrCtrInstancePtr, DeviceId);
if (Status != XST_SUCCESS) {
return XST_FAILURE;
}

/*
* Perform a self-test to ensure that the hardware was built
* correctly. Timer0 is used for self test
*/
Status = XTmrCtr_SelfTest(TmrCtrInstancePtr, TMRCTR_0);
if (Status != XST_SUCCESS) {
return XST_FAILURE;
}


/*
* Setup the handler for the timer counter that will be called from the
* interrupt context when the timer expires
*/
XTmrCtr_SetHandler(TmrCtrInstancePtr, TimerCounterHandler,
TmrCtrInstancePtr);

u32 pwmmasks = XTC_CSR_ENABLE_INT_MASK | XTC_CSR_DOWN_COUNT_MASK
| XTC_CSR_AUTO_RELOAD_MASK;

Xil_Out32(0x42820000, 0); //Turn off all fields.
Xil_Out32(0x42820010, 0); //Turn off all fields.

Xil_Out32(0x42820000, pwmmasks); //Start timer0 with count down/auto reload capability and interrupts.
Xil_Out32(0x42820010, pwmmasks); //Start timer1 with count down/auto reload capability and interrupts.

Xil_Out32(0x42840000, 0); //Turn off all fields.
Xil_Out32(0x42840010, 0); //Turn off all fields.

Xil_Out32(0x42840000, pwmmasks); //Start timer0 with count down/auto reload capability and interrupts.
Xil_Out32(0x42840010, pwmmasks); //Start timer1 with count down/auto reload capability and interrupts.

/*
* We start with the fixed divisor and after every CYCLE_PER_DUTYCYCLE
* decrement the divisor by 1, as a result Duty cycle increases
* proportionally. This is done until duty cycle is reached upto
* MAX_DUTYCYCLE
*/
Div = 66000/(float)myRawDataPhoto;
Div2 = 66000/(float)myRawDataPot;


/* Configure PWM */
Period = 200000; //PWM_PERIOD;
HighTime = Period/Div;

Period2 = 200000; //PWM_PERIOD;
HighTime2 = Period2/Div2;

// DutyCycle_percent = (float)HighTime/(float)Period*100;
Xil_Out32(0x42820000+4, Period);
Xil_Out32(0x42820010+4, HighTime);


Xil_Out32(0x42840000+4, Period2);
Xil_Out32(0x42840010+4, HighTime2);



/* Enable PWM */
pwmmasks |= XTC_CSR_ENABLE_PWM_MASK | XTC_CSR_EXT_GENERATE_MASK|XTC_CSR_ENABLE_ALL_MASK;
Xil_Out32(0x42820000, pwmmasks);
Xil_Out32(0x42820010, pwmmasks);

Xil_Out32(0x42840000, pwmmasks);
Xil_Out32(0x42840010, pwmmasks);

return Status;
}


static void TimerCounterHandler(void *CallBackRef, u8 TmrCtrNumber)
{
/* Mark if period timer expired */
if (TmrCtrNumber == TMRCTR_0) {
PeriodTimerHit = TRUE;
}

/* Mark if high time timer expired */
if (TmrCtrNumber == TMRCTR_1) {
HighTimerHit = TRUE;
}
}



static int TmrCtrSetupIntrSystem(INTC *IntcInstancePtr,
XTmrCtr *TmrCtrInstancePtr, u16 DeviceId, u16 IntrId)
{
int Status;

#ifdef XPAR_INTC_0_DEVICE_ID

#else
XScuGic_Config *IntcConfig;

/*
* Initialize the interrupt controller driver so that it is ready to
* use
*/
IntcConfig = XScuGic_LookupConfig(INTC_DEVICE_ID);
if (NULL == IntcConfig) {
return XST_FAILURE;
}

Status = XScuGic_CfgInitialize(IntcInstancePtr, IntcConfig,
IntcConfig->CpuBaseAddress);
if (Status != XST_SUCCESS) {
return XST_FAILURE;
}

XScuGic_SetPriorityTriggerType(IntcInstancePtr, IntrId,
0xA0, 0x3);

/*
* Connect the interrupt handler that will be called when an
* interrupt occurs for the device.
*/
Status = XScuGic_Connect(IntcInstancePtr, IntrId,
(Xil_ExceptionHandler)XTmrCtr_InterruptHandler,
TmrCtrInstancePtr);
if (Status != XST_SUCCESS) {
return Status;
}

/* Enable the interrupt for the Timer device */
XScuGic_Enable(IntcInstancePtr, IntrId);
#endif /* XPAR_INTC_0_DEVICE_ID */

/* Initialize the exception table */
Xil_ExceptionInit();

/* Register the interrupt controller handler with the exception table */
Xil_ExceptionRegisterHandler(XIL_EXCEPTION_ID_INT,
(Xil_ExceptionHandler)
INTC_HANDLER,
IntcInstancePtr);

/* Enable non-critical exceptions */
Xil_ExceptionEnable();

return XST_SUCCESS;
}


void TmrCtrDisableIntr(INTC *IntcInstancePtr, u16 IntrId)
{
/* Disconnect the interrupt for the timer counter */
#ifdef XPAR_INTC_0_DEVICE_ID
XIntc_Disconnect(IntcInstancePtr, IntrId);
#else
XScuGic_Disconnect(IntcInstancePtr, IntrId);
#endif
}


//-----------------------------------------UltraSonic Sensor SECTION------------------------------------------------------
int TmrCtrPwmExample(INTC *IntcInstancePtr, XTmrCtr *TmrCtrInstancePtr,
u16 DeviceId, u16 IntrId)
{

int Status;

/*
* Initialize the timer counter so that it's ready to use,
* specify the device ID that is generated in xparameters.h
*/
Status = XTmrCtr_Initialize(TmrCtrInstancePtr, DeviceId);
if (Status != XST_SUCCESS) {
return XST_FAILURE;
}


u32 pwmmasks = XTC_CSR_ENABLE_INT_MASK | XTC_CSR_DOWN_COUNT_MASK
| XTC_CSR_AUTO_RELOAD_MASK;

Xil_Out32(0x42810000, 0); //Turn off all fields.
Xil_Out32(0x42810010, 0); //Turn off all fields.

Xil_Out32(0x42810000, 0x52); //Start timer0 with count down/auto reload capability and interrupts.
Xil_Out32(0x42810010, 0x52); //Start timer1 with count down/auto reload capability and interrupts.


// DutyCycle_percent = (float)HighTime/(float)PWM_PERIOD*100;
Xil_Out32(0x42810000+4, PWM_PERIOD/CLK_PERIOD);
Xil_Out32(0x42810010+4, DUTY_PERIOD/CLK_PERIOD);


// printf("PWM Configured for Duty Cycle = %5.2F\r\n", DutyCycle_percent);

/* Enable PWM */
// XTmrCtr_PwmEnable(TmrCtrInstancePtr);
pwmmasks |= XTC_CSR_ENABLE_PWM_MASK | XTC_CSR_EXT_GENERATE_MASK | XTC_CSR_ENABLE_ALL_MASK;
Xil_Out32(0x42810000, 0x52|0x604); //0x656); //pwmmasks);
Xil_Out32(0x42810010, 0x52|0x604); //0x656); //pwmmasks);
return Status;
}


int TmrCtrCapture(INTC *IntcInstancePtr, XTmrCtr *TmrCtrInstancePtr,
u16 DeviceId, u16 IntrId)
{

int Status;

/*
* Initialize the timer counter so that it's ready to use,
* specify the device ID that is generated in xparameters.h
*/
Status = XTmrCtr_Initialize(TmrCtrInstancePtr, DeviceId);
if (Status != XST_SUCCESS) {
return XST_FAILURE;
}

/*
* Perform a self-test to ensure that the hardware was built
* correctly. Timer0 is used for self test
*/
Status = XTmrCtr_SelfTest(TmrCtrInstancePtr, TMRCTR_0_0);
if (Status != XST_SUCCESS) {
return XST_FAILURE;
}

/*
* Connect the timer counter to the interrupt subsystem such that
* interrupts can occur
*/
Status = TmrCtrSetupIntrSystem(IntcInstancePtr, TmrCtrInstancePtr,
DeviceId, IntrId);
if (Status != XST_SUCCESS) {
return XST_FAILURE;
}

/*
* Setup the handler for the timer counter that will be called from the
* interrupt context when the timer expires
*/
XTmrCtr_SetHandler(TmrCtrInstancePtr, TimerCounterHandler_0,
TmrCtrInstancePtr);

u32 masks = XTC_CSR_ENABLE_ALL_MASK | XTC_CSR_ENABLE_INT_MASK | XTC_CSR_AUTO_RELOAD_MASK |
XTC_CSR_EXT_CAPTURE_MASK | XTC_CSR_CAPTURE_MODE_MASK;
Xil_Out32(0x42800000, 0x459); //Start timer0 with Capture capability and interrupts.
Xil_Out32(0x42800010, 0x459); //Start timer1 with Capture capability and interrupts.

return Status;
}

/*****************************************************************************/
/**
* This function is the handler which performs processing for the timer counter.
* It is called from an interrupt context.
*
* @param CallBackRef is a pointer to the callback function
* @param TmrCtrNumber is the number of the timer to which this
* handler is associated with.
*
* @return None.
*
* @note None.
*
/*****************************Capture Handler******************************/
static void TimerCounterHandler_0(void *CallBackRef, u8 TmrCtrNumber)
{

// XTmrCtr *InstancePtr = (XTmrCtr *)CallBackRef;

/* Mark if period timer expired */
if (TmrCtrNumber == TMRCTR_0_0) {
capture0 = XTmrCtr_GetCaptureValue(&TimerCaptureInst, TMRCTR_0_0);
// capture0 =  Xil_In32(0x42800004);
// xil_printf("\r\nIn Tmrctr interrupt handler - TMRCTR_0 %u\r\n", capture0);
PeriodTimerHit = TRUE;
}

/* Mark if high time timer expired */
if (TmrCtrNumber == TMRCTR_0_1) {
capture1 = XTmrCtr_GetCaptureValue(&TimerCaptureInst, TMRCTR_0_1);
// capture1 =  Xil_In32(0x42800014);
// xil_printf("\r\nIn Tmrctr interrupt handler - TMRCTR_1 %u\r\n", capture1);
HighTimerHit = TRUE;

if(capture1 > capture0){
pulsewidth = capture1 - capture0;
}
   else {
   pulsewidth = capture1 - capture0 + 0xFFFFFFFF + 1;
}

}

}

/*****************************************************************************/
/**
* This function setups the interrupt system such that interrupts can occur
* for the timer counter. This function is application specific since the actual
* system may or may not have an interrupt controller.  The timer counter could
* be directly connected to a processor without an interrupt controller.  The
* user should modify this function to fit the application.
*
* @param IntcInstancePtr is a pointer to the Interrupt Controller
* driver Instance.
* @param TmrCtrInstancePtr is a pointer to the XTmrCtr driver Instance.
* @param DeviceId is the XPAR_<TmrCtr_instance>_DEVICE_ID value from
* xparameters.h.
* @param IntrId is XPAR_<INTC_instance>_<TmrCtr_instance>_VEC_ID
* value from xparameters.h.
*
* @return XST_SUCCESS if the Test is successful, otherwise XST_FAILURE.
*
* @note none.
*
******************************************************************************/

