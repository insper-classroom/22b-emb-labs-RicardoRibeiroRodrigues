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

// Trigger
#define TRIG_PIO PIOD
#define TRIG_PIO_ID ID_PIOD
#define TRIG_PIO_IDX 30
#define TRIG_PIO_IDX_MASK (1u << TRIG_PIO_IDX)

// Echo
#define ECHO_PIO PIOA
#define ECHO_PIO_ID ID_PIOA
#define ECHO_PIO_IDX 6
#define ECHO_PIO_IDX_MASK (1u << ECHO_PIO_IDX)

#define V_SOM 340
#define WIDTH_OLED 128
#define HEIGHT_OLED 32



/** RTOS  */
#define TASK_OLED_STACK_SIZE                (1024*6/sizeof(portSTACK_TYPE))
#define TASK_OLED_STACK_PRIORITY            (tskIDLE_PRIORITY)

extern void vApplicationStackOverflowHook(xTaskHandle *pxTask,  signed char *pcTaskName);
extern void vApplicationIdleHook(void);
extern void vApplicationTickHook(void);
extern void vApplicationMallocFailedHook(void);
extern void xPortSysTickHandler(void);

xQueueHandle xQueueEcho;

/** prototypes */
static void io_init(void);
static void RTT_init(float freqPrescale, uint32_t IrqNPulses, uint32_t rttIRQSource);
void send_trig(void);
void gfx_clear(void);
void oled_dist(double dist);


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

void echo_callback(void) {
	// Para borda de subida
	if (pio_get(ECHO_PIO, PIO_INPUT, ECHO_PIO_IDX_MASK)) {
		printf("Borda de subida\n");
		RTT_init(40000, 0, 0);
	} else {
		printf("Borda de descida\n");
		uint32_t timer_value = rtt_read_timer_value(RTT);
		xQueueSendFromISR(xQueueEcho, &timer_value, 0);
	}
}

/************************************************************************/
/* TASKS                                                                */
/************************************************************************/

static void task_oled(void *pvParameters) {
	gfx_mono_ssd1306_init();
	gfx_mono_draw_string("Distancia atual: ", 0, 0, &sysfont);
	
	send_trig();
	uint32_t dT;

	for (;;)  {
		if (xQueueReceive(xQueueEcho, &dT, 0)) {
			printf("Entra no receive\n");
			printf("%u\n", dT);
			float dt_s = dT / 40000;
			float dist = (dt_s * V_SOM) / 2;
			printf("dist = %f\n", dist);
			gfx_clear();
			gfx_mono_draw_string("Distancia atual: ", 0, 0, &sysfont);
			oled_dist(dist);
			send_trig();
		}
	}
}

/************************************************************************/
/* funcoes                                                              */
/************************************************************************/

void oled_dist(double dist) {
	char str[128];
	sprintf(str, "%.6lf m", dist);
	gfx_mono_draw_string(str, 0, 16, &sysfont);
}

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

void gfx_clear(void) {
	for (int x = 0; x < WIDTH_OLED; x++) {
		for (int y = 0; y < HEIGHT_OLED; y++) {
			gfx_mono_draw_pixel(x, y, GFX_PIXEL_CLR);
		}
	}
}

/** 
 * Configura RTT
 *
 * arg0 pllPreScale  : Frequência na qual o contador irá incrementar
 * arg1 IrqNPulses   : Valor do alarme 
 * arg2 rttIRQSource : Pode ser uma 
 *     - 0: 
 *     - RTT_MR_RTTINCIEN: Interrupção por incremento (pllPreScale)
 *     - RTT_MR_ALMIEN : Interrupção por alarme
 */
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

static void io_init(void) {
	// init clocks
	pmc_enable_periph_clk(ECHO_PIO_ID);
	pmc_enable_periph_clk(TRIG_PIO_ID);

	// echo pin
	pio_configure(ECHO_PIO, PIO_INPUT, ECHO_PIO_IDX_MASK, PIO_DEFAULT);
	pio_set_debounce_filter(ECHO_PIO, ECHO_PIO_IDX_MASK, 60);
	pio_handler_set(ECHO_PIO,
		ECHO_PIO_ID,
		ECHO_PIO_IDX_MASK,
		PIO_IT_EDGE,
		echo_callback);
		
	pio_enable_interrupt(ECHO_PIO, ECHO_PIO_IDX_MASK);
	pio_get_interrupt_status(ECHO_PIO);
	
	NVIC_EnableIRQ(ECHO_PIO_ID);
	NVIC_SetPriority(ECHO_PIO_ID, 5);
	
	// trigger pin
	pio_set_output(TRIG_PIO, TRIG_PIO_IDX_MASK, 0, 0, 1);
}

void send_trig(void) {
	pio_set(TRIG_PIO, TRIG_PIO_IDX_MASK);
	delay_us(12);
	pio_clear(TRIG_PIO, TRIG_PIO_IDX_MASK);
}

/************************************************************************/
/* main                                                                 */
/************************************************************************/


int main(void) {
	/* Initialize the SAM system */
	sysclk_init();
	board_init();
	io_init();

	/* Initialize the console uart */
	configure_console();

	/* Create task to control oled */
	if (xTaskCreate(task_oled, "oled", TASK_OLED_STACK_SIZE, NULL, TASK_OLED_STACK_PRIORITY, NULL) != pdPASS) {
	  printf("Failed to create oled task\r\n");
	}
	
	xQueueEcho = xQueueCreate(32, sizeof(uint32_t));
	if (xQueueEcho == NULL)
		printf("Falha ao criar Queue\n");
		
	/* Start the scheduler. */
	vTaskStartScheduler();

  /* RTOS não deve chegar aqui !! */
	while(1){}

	/* Will only get here if there was insufficient memory to create the idle task. */
	return 0;
}
