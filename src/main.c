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

#define TRIG_PIO     PIOC
#define TRIG_PIO_ID  ID_PIOC
#define TRIG_PIO_PIN 13
#define TRIG_PIO_PIN_MASK (1 << TRIG_PIO_PIN)

#define ECHO_PIO     PIOD
#define ECHO_PIO_ID  ID_PIOD
#define ECHO_PIO_PIN 30
#define ECHO_PIO_PIN_MASK (1 << ECHO_PIO_PIN)

#define ALARM_PIO     PIOA
#define ALARM_PIO_ID  ID_PIOA
#define ALARM_PIO_PIN 6
#define ALARM_PIO_PIN_MASK (1 << ALARM_PIO_PIN)

/** RTOS  */
#define TASK_OLED_STACK_SIZE                (1024*6/sizeof(portSTACK_TYPE))
#define TASK_OLED_STACK_PRIORITY            (tskIDLE_PRIORITY)

#define TASK_ALARM_STACK_SIZE                (1024*6/sizeof(portSTACK_TYPE))
#define TASK_ALARM_STACK_PRIORITY            (tskIDLE_PRIORITY)

SemaphoreHandle_t xSemaphoreEcho;
SemaphoreHandle_t xSemaphoreAlarmEnable;
SemaphoreHandle_t xSemaphoreAlarmDisable;

QueueHandle_t xQueueEcho;

extern void vApplicationStackOverflowHook(xTaskHandle *pxTask,  signed char *pcTaskName);
extern void vApplicationIdleHook(void);
extern void vApplicationTickHook(void);
extern void vApplicationMallocFailedHook(void);
extern void xPortSysTickHandler(void);

/** prototypes */
void but_callback(void);
void echo_callback(void);
static void BUT_init(void);
static void TRIG_init(void);
static void ECHO_init(void);
static void RTT_init(float freqPrescale, uint32_t IrqNPulses, uint32_t rttIRQSource);
static void ALARM_init(void);


/*************************/
/* RTOS application funcs*/
/*************************/

extern void vApplicationStackOverflowHook(xTaskHandle *pxTask, signed char *pcTaskName) {
	printf("stack overflow %x %s\r\n", pxTask, (portCHAR *)pcTaskName);
	for (;;) {	}
}

extern void vApplicationIdleHook(void) { }

extern void vApplicationTickHook(void) { }

extern void vApplicationMallocFailedHook(void) {
	configASSERT( ( volatile void * ) NULL );
}

/************************/
/* handlers / callbacks */
/************************/

void but_callback(void) {
	BaseType_t xHigherPriorityTaskWoken = pdFALSE;
	xSemaphoreGiveFromISR(xSemaphoreEcho, &xHigherPriorityTaskWoken);
}

void echo_callback(void) {
	if (pio_get(ECHO_PIO, PIO_INPUT, ECHO_PIO_PIN_MASK)){
		RTT_init(32000, 0, 0);
	}
	else {
		uint32_t tempo = rtt_read_timer_value(RTT);
		xQueueSendFromISR(xQueueEcho, &tempo, 0);
	}
}

/************************/
/* TASK                 */
/************************/

static void task_alarm(void *pvParameters){
	ALARM_init();
	char status = 0;
	int freq = 244;
	int periodo = (1000/(freq))/2;

	for (;;) {
		if (xSemaphoreTake(xSemaphoreAlarmEnable, (TickType_t) 500) == pdTRUE) {
			status = 1;
		}
		if (xSemaphoreTake(xSemaphoreAlarmDisable, (TickType_t) 500) == pdTRUE) {
			status = 0;
		}
		if (status == 1){
			pio_set(ALARM_PIO, ALARM_PIO_PIN_MASK);
			delay_ms(periodo);
			pio_clear(ALARM_PIO, ALARM_PIO_PIN_MASK);
			delay_ms(periodo);
		}
	}
}

static void task_oled(void *pvParameters) {
	uint32_t ticks;
	char str[32];
	int distancia;
	gfx_mono_ssd1306_init();
	BaseType_t xHigherPriorityTaskWoken = pdFALSE;
	
	BUT_init();
	TRIG_init();
	ECHO_init();

	for (;;)  {
		if (xSemaphoreTake(xSemaphoreEcho, (TickType_t) 500) == pdTRUE) {
			pio_set(TRIG_PIO, TRIG_PIO_PIN_MASK);
			delay_us(10);
			pio_clear(TRIG_PIO, TRIG_PIO_PIN_MASK);
		
			if (xQueueReceive(xQueueEcho, &ticks, 5000)) {
				double tempo = (double)ticks/32000;
				distancia = ((tempo * 343)/2)*100;

				if (distancia >= 200){
					xSemaphoreGiveFromISR(xSemaphoreAlarmEnable, &xHigherPriorityTaskWoken);
				} 
				else {
					xSemaphoreGiveFromISR(xSemaphoreAlarmDisable, &xHigherPriorityTaskWoken);
				}

				sprintf(str, "%6d", distancia);
				printf(str);
				gfx_mono_draw_string(str, 25, 12, &sysfont);
				gfx_mono_draw_string(" cm", 65, 12, &sysfont);			
			}
		}
	}
}

/************************/
/* funcoes              */
/************************/

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

static void BUT_init(void) {
	/* configura prioridae */
	NVIC_EnableIRQ(BUT_PIO_ID);
	NVIC_SetPriority(BUT_PIO_ID, 4);

	/* conf botão como entrada */
	pio_configure(BUT_PIO, PIO_INPUT, BUT_PIO_PIN_MASK, PIO_PULLUP | PIO_DEBOUNCE);
	pio_set_debounce_filter(BUT_PIO, BUT_PIO_PIN_MASK, 60);
	pio_enable_interrupt(BUT_PIO, BUT_PIO_PIN_MASK);
	pio_handler_set(BUT_PIO, BUT_PIO_ID, BUT_PIO_PIN_MASK, PIO_IT_FALL_EDGE , but_callback);
}

static void TRIG_init(void) {
	pmc_enable_periph_clk(TRIG_PIO_ID);
	pio_configure(TRIG_PIO, PIO_OUTPUT_1, TRIG_PIO_PIN_MASK, PIO_DEFAULT);
}

static void ECHO_init(void) {
	/* configura prioridae */
	NVIC_EnableIRQ(ECHO_PIO_ID);
	NVIC_SetPriority(ECHO_PIO_ID, 4);

	/* conf echo como entrada */
	pio_configure(ECHO_PIO, PIO_INPUT, ECHO_PIO_PIN_MASK, PIO_DEFAULT);
	pio_enable_interrupt(ECHO_PIO, ECHO_PIO_PIN_MASK);
	pio_handler_set(ECHO_PIO, ECHO_PIO_ID, ECHO_PIO_PIN_MASK, PIO_IT_EDGE, echo_callback);
}

static void ALARM_init(void) {
	pmc_enable_periph_clk(ALARM_PIO_ID);
	pio_set_output(ALARM_PIO, ALARM_PIO_PIN_MASK, 0, 0, 0);
}

/************************/
/* main                 */
/************************/


int main(void) {
	sysclk_init();
	board_init();
	
	xQueueEcho = xQueueCreate(32, sizeof(uint32_t));

	xSemaphoreEcho = xSemaphoreCreateBinary();
	if (xSemaphoreEcho == NULL)
   		printf("falha em criar o semaforo \n");

	xSemaphoreAlarmEnable = xSemaphoreCreateBinary();
	if (xSemaphoreAlarmEnable == NULL)
   		printf("falha em criar o semaforo \n");

	xSemaphoreAlarmDisable = xSemaphoreCreateBinary();
	if (xSemaphoreAlarmDisable == NULL)
   		printf("falha em criar o semaforo \n");

	/* Initialize the console uart */
	configure_console();

	/* Create task to control oled */
	if (xTaskCreate(task_oled, "oled", TASK_OLED_STACK_SIZE, NULL, TASK_OLED_STACK_PRIORITY, NULL) != pdPASS) {
		printf("Failed to create oled task\r\n");
	}

	if (xTaskCreate(task_alarm, "alarm", TASK_ALARM_STACK_SIZE, NULL, TASK_ALARM_STACK_PRIORITY, NULL) != pdPASS) {
		printf("Failed to create alarm  task\r\n");
	}

	/* Start the scheduler. */
	vTaskStartScheduler();

	/* RTOS não deve chegar aqui !! */
	while(1){}

	/* Will only get here if there was insufficient memory to create the idle task. */
	return 0;
}