#include <asf.h>
#include "conf_board.h"

#include "gfx_mono_ug_2832hsweg04.h"
#include "gfx_mono_text.h"
#include "sysfont.h"

/* Botao da placa */
#define BUT_PIO     PIOA
#define BUT_PIO_ID  ID_PIOA
#define BUT_PIO_PIN 11
#define BUT_PIO_PIN_MASK (1 << BUT_PIO_PIN)

#define ECHO_PIO PIOD
#define ECHO_PIO_ID ID_PIOD
#define ECHO_PIO_PIN 30
#define ECHO_PIO_PIN_MASK (1 << ECHO_PIO_PIN)

#define TRIG_PIO PIOC
#define TRIG_PIO_ID ID_PIOC
#define TRIG_PIO_PIN 13
#define TRIG_PIO_PIN_MASK (1 << TRIG_PIO_PIN)

/** RTOS  */
#define TASK_OLED_STACK_SIZE                (1024*6/sizeof(portSTACK_TYPE))
#define TASK_OLED_STACK_PRIORITY            (tskIDLE_PRIORITY)

extern void vApplicationStackOverflowHook(xTaskHandle *pxTask,  signed char *pcTaskName);
extern void vApplicationIdleHook(void);
extern void vApplicationTickHook(void);
extern void vApplicationMallocFailedHook(void);
extern void xPortSysTickHandler(void);

/** prototypes */
void but_callback(void);
void echo_callback(void);
static void BUT_init(void);
static void ECHO_init(void);
static void TRIG_init(void);
static void RTT_init(float freqPrescale, uint32_t IrqNPulses, uint32_t rttIRQSource);

/**Queue**/
QueueHandle_t xQueueECHO;

/************************************************************************/
/* RTOS application funcs                                               */
/************************************************************************/

extern void vApplicationStackOverflowHook(xTaskHandle *pxTask, signed char *pcTaskName) {
	printf("stack overflow %x %s\r\n", pxTask, (portCHAR *)pcTaskName);
	for (;;) {	}
}

extern void vApplicationIdleHook(void) { }

extern void vApplicationTickHook(void) { }

extern void vApplicationMallocFailedHook(void) {
	configASSERT( ( volatile void * ) NULL );
}

/************************************************************************/
/* handlers / callbacks                                                 */
/************************************************************************/

void but_callback(void) {
	pio_set(TRIG_PIO, TRIG_PIO_PIN_MASK);
	delay_ms(1000);
	pio_clear(TRIG_PIO, TRIG_PIO_PIN_MASK);
}

void echo_callback(void) {
	if (pio_get(ECHO_PIO, PIO_INPUT, ECHO_PIO_PIN_MASK)) {
		RTT_init(8621, 0, 0);
	} else {
		uint32_t time = rtt_read_timer_value(RTT);
		xQueueSendFromISR(xQueueECHO, &time, 0);
	}
}
/************************************************************************/
/* TASKS                                                                */
/************************************************************************/

static void task_oled(void *pvParameters) {
	gfx_mono_ssd1306_init();
  gfx_mono_draw_string("Exemplo RTOS", 0, 0, &sysfont);
  gfx_mono_draw_string("oii", 0, 20, &sysfont);

	for (;;)  {
		if(xQueueReceive(xQueueECHO, &time, 0)) {
			printf("time: %d\n", time);
		}
		else
	}
}

/************************************************************************/
/* funcoes                                                              */
/************************************************************************/

static void configure_console(void) {
	const usart_serial_options_t uart_serial_options = {
		.baudrate = CONF_UART_BAUDRATE,
		.charlength = CONF_UART_CHAR_LENGTH,
		.paritytype = CONF_UART_PARITY,
		.stopbits = CONF_UART_STOP_BITS,
	};

	/* Configure console UART. */
	stdio_serial_init(CONF_UART, &uart_serial_options);

	/* Specify that stdout should not be buffered. */
	setbuf(stdout, NULL);
}

static void BUT_init(void) {
	NVIC_EnableIRQ(BUT_PIO_ID);
	NVIC_SetPriority(BUT_PIO_ID, 4);

	pio_configure(BUT_PIO, PIO_INPUT, BUT_PIO_PIN_MASK, PIO_PULLUP | PIO_DEBOUNCE);
	pio_set_debounce_filter(BUT_PIO, BUT_PIO_PIN_MASK, 60);
	pio_enable_interrupt(BUT_PIO, BUT_PIO_PIN_MASK);
	pio_handler_set(BUT_PIO, BUT_PIO_ID, BUT_PIO_PIN_MASK, PIO_IT_FALL_EDGE , but_callback);
}

static void ECHO_init(void) {
	NVIC_EnableIRQ(ECHO_PIO_ID);
	NVIC_SetPriority(ECHO_PIO_ID, 4);

	pio_configure(ECHO_PIO, PIO_INPUT, ECHO_PIO_PIN_MASK, PIO_PULLUP | PIO_DEBOUNCE);
	pio_set_debounce_filter(ECHO_PIO, ECHO_PIO_PIN_MASK, 60);
	pio_enable_interrupt(ECHO_PIO, ECHO_PIO_PIN_MASK);
	pio_handler_set(ECHO_PIO, ECHO_PIO_ID, ECHO_PIO_PIN_MASK, PIO_IT_FALL_EDGE , echo_callback);
}

static void TRIG_init(void) {
	pmc_enable_periph_clk(TRIG_PIO_ID);
	pio_configure(TRIG_PIO, PIO_OUTPUT_0, TRIG_PIO_PIN_MASK, PIO_DEFAULT);
}

static void RTT_init(float freqPrescale, uint32_t IrqNPulses, uint32_t rttIRQSource) {

  uint16_t pllPreScale = (int) (((float) 32768) / freqPrescale);
	
  rtt_sel_source(RTT, false);
  rtt_init(RTT, pllPreScale);
  
  if (rttIRQSource & RTT_MR_ALMIEN) {
	uint32_t ul_previous_time;
  	ul_previous_time = rtt_read_timer_value(RTT);
  	while (ul_previous_time == rtt_read_timer_value(RTT));
  	rtt_write_alarm_time(RTT, IrqNPulses+ul_previous_time);
  }

  /* config NVIC */
  NVIC_DisableIRQ(RTT_IRQn);
  NVIC_ClearPendingIRQ(RTT_IRQn);
  NVIC_SetPriority(RTT_IRQn, 4);
  NVIC_EnableIRQ(RTT_IRQn);

  /* Enable RTT interrupt */
  if (rttIRQSource & (RTT_MR_RTTINCIEN | RTT_MR_ALMIEN))
	rtt_enable_interrupt(RTT, rttIRQSource);
  else
	rtt_disable_interrupt(RTT, RTT_MR_RTTINCIEN | RTT_MR_ALMIEN);
		  
}

/************************************************************************/
/* main                                                                 */
/************************************************************************/
int main(void) {
	/* Initialize the SAM system */
	sysclk_init();
	board_init();

	/* Initialize the console uart */
	configure_console();

	/* Create task to control oled */
	if (xTaskCreate(task_oled, "oled", TASK_OLED_STACK_SIZE, NULL, TASK_OLED_STACK_PRIORITY, NULL) != pdPASS) {
	  printf("Failed to create oled task\r\n");
	}


	/* Start the scheduler. */
	vTaskStartScheduler();

  /* RTOS n�o deve chegar aqui !! */
	while(1){}

	/* Will only get here if there was insufficient memory to create the idle task. */
	return 0;
}