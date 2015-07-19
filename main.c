/*
 *  blinky.c for the Teensy 3.1 board (K20 MCU, 16 MHz crystal)
 *
 *  This code will blink the Teensy's LED.  Each "blink" is
 *  really a set of eight pulses.  These pulses give the actual
 *  system clock in Mhz, starting with the MSB.  A pulse is
 *  narrow for a 0-bit and wide for a 1-bit.
 *
 *  For a system clock of 72 MHz, blinks will read 0x48.
 *  For a system clock of 48 MHz, blinks will read 0x30.
 */

#include "common.h"
#include "string.h"

__inline__ void dumbdelay_ms(uint32_t ms)
{
	uint32_t index;
	uint32_t loops = ms * ( (uint32_t)mcg_clk_hz / 10000 );

	for (index=0; index<loops; index++)  ;	// dumb delay
}

void HardFault_Handler()
{
	int n, byIndex;

	PORTC_PCR5 = PORT_PCR_MUX(0x1); // LED is on PC5 (pin 13), config as GPIO (alt = 1)
	GPIOC_PDDR = (1<<5);			// make this an output pin

	// This guy is called when memset et al is called...
	while(1)
	{
		// Do the "hard fault" dance
		for ( byIndex = 0; byIndex < 4; byIndex ++ )
		{
			GPIOC_PSOR = ( 1 << 5 );
			dumbdelay_ms( 50 );
			GPIOC_PCOR = ( 1 << 5 );
			dumbdelay_ms( 50 );
		}
	}
}

#define FTM_FC_PS_DIV_1 0
#define FTM_FC_PS_DIV_2 1
#define FTM_FC_PS_DIV_4 2
#define FTM_FC_PS_DIV_8 3
#define FTM_FC_PS_DIV_16 4
#define FTM_FC_PS_DIV_32 5
#define FTM_FC_PS_DIV_64 6
#define FTM_FC_PS_DIV_128 7

#define PERIOD_1MS 0x4650
#define PERIOD_2MS 0x8CA0
#define PERIOD_2_5MS 0xAFC8


void initPWM(void)
{
   //SIM_SCGC5 |= SIM_SCGC5_PORTC_MASK;

   //PORTC_PCR4 = (0|PORT_PCR_MUX(4)); /* FTM0_CH0 enable on PTC3 */


	/* SIM_SCGC6: FTM0=1 - Set this otherwise hard fault!*/
	SIM_SCGC6 |= SIM_SCGC6_FTM0_MASK;

	/* FTM0_MODE: ??=0,??=0,??=0,??=0,??=0,??=0,??=0,??=0,??=0,??=0,??=0,??=0,??=0,??=0,??=0,??=0,??=0,??=0,??=0,??=0,??=0,??=0,??=0,??=0,FAULTIE=0,FAULTM=0,CAPTEST=0,PWMSYNC=0,WPDIS=1,INIT=0,FTMEN=0 */
	FTM0_MODE = (FTM_MODE_FAULTM(0x00) | FTM_MODE_WPDIS_MASK); /* Set up mode register */

	/* FTM0_SC: ??=0,??=0,??=0,??=0,??=0,??=0,??=0,??=0,??=0,??=0,??=0,??=0,??=0,??=0,??=0,??=0,??=0,??=0,??=0,??=0,??=0,??=0,??=0,??=0,TOF=0,TOIE=0,CPWMS=0,CLKS=0,PS=0 */
	FTM0_SC = (FTM_SC_CLKS(0x00) | FTM_SC_PS(0x00)); /* Clear status and control register */

	FTM0_CNTIN = FTM_CNTIN_INIT(0x00);   /* Clear counter initial register */

	FTM0_MOD = FTM_MOD_MOD( PERIOD_2_5MS );      /* Set up modulo register */
	FTM0_C0SC = (FTM_CnSC_MSB_MASK | FTM_CnSC_ELSB_MASK); /* Set up channel status and control register */
	FTM0_C0V = FTM_CnV_VAL( PERIOD_1MS );      /* Set up channel value register */

	FTM0_OUTINIT = FTM_OUTINIT_CH0OI_MASK; /* Set up Initial State for Channel Output register */
	 /* FTM0_MODE: ??=0,??=0,??=0,??=0,??=0,??=0,??=0,??=0,??=0,??=0,??=0,??=0,??=0,??=0,??=0,??=0,??=0,??=0,??=0,??=0,??=0,??=0,??=0,??=0,FAULTIE=0,FAULTM=0,CAPTEST=0,PWMSYNC=0,WPDIS=1,INIT=1,FTMEN=0 */
	 FTM0_MODE = (FTM_MODE_FAULTM(0x00) | FTM_MODE_WPDIS_MASK | FTM_MODE_INIT_MASK); /* Initialize the Output Channels */
	 /* PORTC_PCR1: ISF=0,MUX=4 */
	 PORTC_PCR1 = (uint32_t)((PORTC_PCR1 & (uint32_t)~(uint32_t)(
				   PORT_PCR_ISF_MASK |
				   PORT_PCR_MUX(0x03)
				  )) | (uint32_t)(
				   PORT_PCR_MUX(0x04)
				  ));
	 /* FTM0_SC: ??=0,??=0,??=0,??=0,??=0,??=0,??=0,??=0,??=0,??=0,??=0,??=0,??=0,??=0,??=0,??=0,??=0,??=0,??=0,??=0,??=0,??=0,??=0,??=0,TOF=0,TOIE=0,CPWMS=0,CLKS=1,PS=0 */
	 FTM0_SC = ( FTM_SC_CLKS( 0x01 ) | FTM_SC_PS( FTM_FC_PS_DIV_4 ) ); /* Set up status and control register */
}

#if 0
void initFreeRTOS()
{
	if( xQueue != NULL )
	{
		/* Start the two tasks as described in the comments at the top of this
		file. */
		xTaskCreate( prvQueueReceiveTask, "Rx", configMINIMAL_STACK_SIZE, NULL, mainQUEUE_RECEIVE_TASK_PRIORITY, NULL );
		xTaskCreate( prvQueueSendTask, "TX", configMINIMAL_STACK_SIZE, NULL, mainQUEUE_SEND_TASK_PRIORITY, NULL );

		/* Create the software timer that is responsible for turning off the LED
		if the button is not pushed within 5000ms, as described at the top of
		this file. */
		xButtonLEDTimer = xTimerCreate( "ButtonLEDTimer", 			/* A text name, purely to help debugging. */
									mainBUTTON_LED_TIMER_PERIOD_MS,	/* The timer period, in this case 5000ms (5s). */
									pdFALSE,						/* This is a one shot timer, so xAutoReload is set to pdFALSE. */
									( void * ) 0,					/* The ID is not used, so can be set to anything. */
									prvButtonLEDTimerCallback		/* The callback function that switches the LED off. */
								);

		/* Start the tasks and timer running. */
		vTaskStartScheduler();
	}
}
#endif

/* ************************************************************************** */
int main( void )
/*
** Entry point to program
*/
{
	uint8_t byIndex;
	uint16_t wDuty;

	wDuty = 0;

	// Initialize GPIO pin
#if 0
	PORTC_PCR5 = PORT_PCR_MUX( 0x1 );	// LED is on PC5 (pin 13), config as GPIO (alt = 1)
	GPIOC_PDDR = ( 1 << 5 );			// make this an output pin
	GPIOC_PCOR = ( 1 << 5 );			// start with LED off
#endif

	/* Configure pin as output */
	/* GPIOC_PDDR: PDD|=0x20 */
	GPIOC_PDDR |= GPIO_PDDR_PDD(0x20);

	/* Set initialization value */
	/* GPIOC_PDOR: PDO&=~0x20 */
	GPIOC_PDOR &= (uint32_t)~(uint32_t)(GPIO_PDOR_PDO(0x20));

	/* Initialization of Port Control register */
	/* PORTC_PCR5: ISF=0,MUX=1 */
	PORTC_PCR5 = (uint32_t)((PORTC_PCR5 & (uint32_t)~(uint32_t)(
		PORT_PCR_ISF_MASK |
		PORT_PCR_MUX(0x06)
	   )) | (uint32_t)(
		PORT_PCR_MUX(0x01)
	   ));

	// Flash the led on and off a couple of times to begin with...
	for ( byIndex = 0; byIndex < 3; byIndex ++ )
	{
		GPIOC_PSOR = ( 1 << 5 );
		dumbdelay_ms( 100 );
		GPIOC_PCOR = ( 1 << 5 );
		dumbdelay_ms( 100 );
	}

	// Wait for it...
	dumbdelay_ms(500);

	// Init the PWM
	initPWM();

	// Init freertos
	//initFreeRTOS();

	// Flash slowly!
	while ( 1 )
	{
		GPIOC_PSOR = ( 1 << 5 );
		dumbdelay_ms ( 500 );
		GPIOC_PCOR = ( 1 << 5 );
		dumbdelay_ms( 500 );

		// Modify the PWM duty
		FTM0_C0V = FTM_CnV_VAL(wDuty+=0x1000);
	}

	return  0;						// should never get here!
}
