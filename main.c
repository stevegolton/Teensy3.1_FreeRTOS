#include "common.h"
#include "string.h"

#include "FreeRTOS.h"
#include "timers.h"

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

__inline__ void dumbdelay_ms( const uint32_t ms );
void HardFault_Handler();
void initPWM( void );
void initFreeRTOS();

//#define configASSERT( x )     if( ( x ) == 0 ) { taskDISABLE_INTERRUPTS(); for( ;; ); }

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
	initFreeRTOS();

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

/* ************************************************************************** */
/*!
 * \brief		Delay using a loop.
 *
 * \param[in]	ms		Delay in ms.
 */
__inline__ void dumbdelay_ms( const uint32_t ms )
{
	uint32_t loops;
	uint32_t index;

	// Calc delay in clock cycles
	loops = ms * ( (uint32_t)mcg_clk_hz / 10000 );

	// Dumb delay
	for ( index = 0; index < loops; index++ );
}

/* ************************************************************************** */
/*!
 * \brief	Called by the system when a hard fault is encountered.
 * 			Flashes our led at 10hz indefinitely.
 */
void HardFault_Handler()
{
	int index;

	// Configure "panic" pin
	PORTC_PCR5 = PORT_PCR_MUX(0x1); // LED is on PC5 (pin 13), config as GPIO (alt = 1)
	GPIOC_PDDR = (1<<5);			// make this an output pin

	while( 1 )
	{
		// Do the "hard fault panic" dance
		for ( index = 0; index < 4; index ++ )
		{
			GPIOC_PSOR = ( 1 << 5 );
			dumbdelay_ms( 50 );
			GPIOC_PCOR = ( 1 << 5 );
			dumbdelay_ms( 50 );
		}
	}
}

/* ************************************************************************** */
/*!
 * \brief		Init PWM.
 *
 * Initialises the PWM on pin 22 of the Teensy3.1 board with the config:
 * - Period = 2.5ms
 * - Duty = 1ms high
 * - FTM 0
 */
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
/* ************************************************************************** */
/*!
 * \brief		Initialises FreeRTOS.
 */
void initFreeRTOS()
{
	// Nothing for now...
}

#endif

#if 1

#define mainBUTTON_LED_TIMER_PERIOD_MS		( 500UL / portTICK_PERIOD_MS )

static TimerHandle_t xButtonLEDTimer = NULL;

static int i = 0;

static void prvButtonLEDTimerCallback( TimerHandle_t xTimer )
{
	if ( 1 == i )
	{
		i = 0;
		GPIOC_PSOR = ( 1 << 5 );
	}
	else
	{
		i = 1;
		GPIOC_PCOR = ( 1 << 5 );
	}
}

static void sometask( void *pvParameters )
{
	int index;

	xTimerStart( xButtonLEDTimer, 0 );

	for (;;)
	{
		//GPIOC_PSOR = ( 1 << 5 );
		//dumbdelay_ms( 500 );
		//GPIOC_PCOR = ( 1 << 5 );
		//dumbdelay_ms( 500 );
	}
}

/* ************************************************************************** */
/*!
 * \brief		Initializes FreeRTOS.
 */
void initFreeRTOS()
{
	// Configure "LED" pin
	PORTC_PCR5 = PORT_PCR_MUX(0x1); // LED is on PC5 (pin 13), config as GPIO (alt = 1)
	GPIOC_PDDR = (1<<5);			// make this an output pin

	xButtonLEDTimer = xTimerCreate( "LEDtmr", 						/* A text name, purely to help debugging. */
									mainBUTTON_LED_TIMER_PERIOD_MS,	/* The timer period, in this case 5000ms (5s). */
									pdTRUE,						/* This is a one shot timer, so xAutoReload is set to pdFALSE. */
									( void * ) 0,					/* The ID is not used, so can be set to anything. */
									prvButtonLEDTimerCallback		/* The callback function that switches the LED off. */
							);

	xTaskCreate( sometask, "Task", configMINIMAL_STACK_SIZE, NULL, 0, NULL );

	/* Start the tasks and timer running. */
	vTaskStartScheduler();
}

/*
** ===================================================================
**     Event       :  FRTOS1_vApplicationStackOverflowHook (module Events)
**
**     Component   :  FRTOS1 [FreeRTOS]
**     Description :
**         if enabled, this hook will be called in case of a stack
**         overflow.
**     Parameters  :
**         NAME            - DESCRIPTION
**         pxTask          - Task handle
**       * pcTaskName      - Pointer to task name
**     Returns     : Nothing
** ===================================================================
*/
void vApplicationStackOverflowHook(xTaskHandle pxTask, char *pcTaskName)
{
  /* This will get called if a stack overflow is detected during the context
     switch.  Set configCHECK_FOR_STACK_OVERFLOWS to 2 to also check for stack
     problems within nested interrupts, but only do this for debug purposes as
     it will increase the context switch time. */
  (void)pxTask;
  (void)pcTaskName;
  taskDISABLE_INTERRUPTS();
  /* Write your code here ... */
  for(;;) {}
}

/*
** ===================================================================
**     Event       :  FRTOS1_vApplicationTickHook (module Events)
**
**     Component   :  FRTOS1 [FreeRTOS]
**     Description :
**         If enabled, this hook will be called by the RTOS for every
**         tick increment.
**     Parameters  : None
**     Returns     : Nothing
** ===================================================================
*/
void vApplicationTickHook(void)
{
  /* Called for every RTOS tick. */
  /* Write your code here ... */
}

/*
** ===================================================================
**     Event       :  FRTOS1_vApplicationIdleHook (module Events)
**
**     Component   :  FRTOS1 [FreeRTOS]
**     Description :
**         If enabled, this hook will be called when the RTOS is idle.
**         This might be a good place to go into low power mode.
**     Parameters  : None
**     Returns     : Nothing
** ===================================================================
*/
void vApplicationIdleHook(void)
{
  /* Called whenever the RTOS is idle (from the IDLE task).
     Here would be a good place to put the CPU into low power mode. */
  /* Write your code here ... */
}

/*
** ===================================================================
**     Event       :  FRTOS1_vApplicationMallocFailedHook (module Events)
**
**     Component   :  FRTOS1 [FreeRTOS]
**     Description :
**         If enabled, the RTOS will call this hook in case memory
**         allocation failed.
**     Parameters  : None
**     Returns     : Nothing
** ===================================================================
*/
void vApplicationMallocFailedHook(void)
{
  /* Called if a call to pvPortMalloc() fails because there is insufficient
     free memory available in the FreeRTOS heap.  pvPortMalloc() is called
     internally by FreeRTOS API functions that create tasks, queues, software
     timers, and semaphores.  The size of the FreeRTOS heap is set by the
     configTOTAL_HEAP_SIZE configuration constant in FreeRTOSConfig.h. */
  taskDISABLE_INTERRUPTS();
  /* Write your code here ... */
  for(;;) {}
}

#endif
