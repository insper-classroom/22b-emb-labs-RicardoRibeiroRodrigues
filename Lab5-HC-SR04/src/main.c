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

#define V_SOM 340.0
#define WIDTH_OLED 128
#define HEIGHT_OLED 32
// 1 / tempo mínimo que echo pode ficar em 1
#define FREQ 1 / (2 * 0.000058)



/** RTOS  */
#define TASK_OLED_STACK_SIZE                (1024*6/sizeof(portSTACK_TYPE))
#define TASK_OLED_STACK_PRIORITY            (tskIDLE_PRIORITY)

extern void vApplicationStackOverflowHook(xTaskHandle *pxTask,  signed char *pcTaskName);
extern void vApplicationIdleHook(void);
extern void vApplicationTickHook(void);
extern void vApplicationMallocFailedHook(void);
extern void xPortSysTickHandler(void);

xQueueHandle xQueueEcho;

// Para atualizar o Oled a cada 1 segundo
xTimerHandle xTimerOled;
xSemaphoreHandle xSemaphoreOled;

/** prototypes */
static void io_init(void);
static void RTT_init(float freqPrescale, uint32_t IrqNPulses, uint32_t rttIRQSource);
void send_trig(void);
void gfx_clear(void);
void oled_dist(double dist);
void desenha_grafico_e_numero(float dist, int *x_atual);


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
	if (pio_get(ECHO_PIO, PIO_INPUT, ECHO_PIO_IDX_MASK)) {
		printf("Borda de subida\n");
		// Alarme no 204 -> O dt já é maior que o máximo possível.
		RTT_init(FREQ, 204, RTT_MR_ALMIEN);
	} else {
		printf("Borda de descida\n");
		uint32_t timer_value = rtt_read_timer_value(RTT);
		printf("Timer value = %u\n", timer_value);
		xQueueSendFromISR(xQueueEcho, &timer_value, 0);
	}
}

void RTT_Handler(void) {
	uint32_t ul_status;
	ul_status = rtt_get_status(RTT);

	/* IRQ due to Alarm */
	if ((ul_status & RTT_SR_ALMS) == RTT_SR_ALMS) {
		printf("Estorou o alarme!");
		uint32_t timer_value = rtt_read_timer_value(RTT);
		xQueueSendFromISR(xQueueEcho, &timer_value, 0);
	}
}

void vTimerOledCallback(TimerHandle_t xTimer) {
	xSemaphoreGiveFromISR(xSemaphoreOled, 0);
}

/************************************************************************/
/* TASKS                                                                */
/************************************************************************/

static void task_oled(void *pvParameters) {
	gfx_mono_ssd1306_init();
	gfx_mono_draw_string("Distancia atual: ", 0, 0, &sysfont);
	float dist = 0;
	int x_atual = 0;
	
	send_trig();
	uint32_t dT;
	char erro = 0;
	
	 xTimerOled = xTimerCreate(
		 "TimerOled",
		 250 / portTICK_PERIOD_MS, // 1/4 segundo
		 pdTRUE,
		 (void *) 0,
		 vTimerOledCallback
	 );
	 xTimerStart(xTimerOled, 0);

	for (;;)  {
		if (xQueueReceive(xQueueEcho, &dT, 0)) {
			printf("dt = %lu\n", dT);
			dist = (V_SOM * dT) / (2 * FREQ) * 100; // * 100 para converter para centimetros
			
			if (dist > 400) {
				// Erro de espaco aberto
				x_atual = 0;
				erro = 'a';
			} else if(dist < 2) {
				erro = 'm';
			} else {
				erro = 0;
			}
			printf("%.6lf\n", dist);
			// oled_dist(dist);
			send_trig();
		}
		// Esse semaforo serve para atualizar o OLED um pouco mais lento, para nao fazer a atualizacao rapido demais para dar para ver
		if (xSemaphoreTake(xSemaphoreOled, 0)) {
			if (!erro) {
				// oled_dist(dist);
				desenha_grafico_e_numero(dist, &x_atual);
			} else if (erro == 'a') {
				gfx_clear();
				gfx_mono_draw_string("Espaco aberto", 0, 0, &sysfont);
			} else if (erro == 'm') {
				gfx_clear();
				gfx_mono_draw_string("Mau contato", 0, 0, &sysfont);
			}
		}
	}
}

/************************************************************************/
/* funcoes                                                              */
/************************************************************************/

void oled_dist(double dist) {
	gfx_clear();		
	gfx_mono_draw_string("Distancia atual: ", 0, 0, &sysfont);
	char str[128];
	sprintf(str, "%.2lf cm", dist);
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

void gfx_clear_half(void) {
	for (int x = 0; x < WIDTH_OLED; x++) {
		for (int y = 0; y < HEIGHT_OLED / 2; y++) {
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
	NVIC_SetPriority(ECHO_PIO_ID, 4);
	
	// trigger pin
	pio_set_output(TRIG_PIO, TRIG_PIO_IDX_MASK, 0, 0, 1);
}

void send_trig(void) {
	pio_set(TRIG_PIO, TRIG_PIO_IDX_MASK);
	delay_us(10);
	pio_clear(TRIG_PIO, TRIG_PIO_IDX_MASK);
}

void desenha_grafico_e_numero(float dist, int *x_atual)
{
	char str[20];
	sprintf(str, "%.1f cm", dist);
	gfx_clear_half();
	gfx_mono_draw_string(str, 0, 0, &sysfont);
	
	

	if (*x_atual == 0)
	gfx_mono_generic_draw_filled_rect(70, 0, 74, 31, GFX_PIXEL_CLR);
	
	int dist_max = 400; // cm
	int dist_min = 2; // cm
	int diff = dist_max - dist_min;
	
	// Comecando em y = 1 do oled e terminado em y = 30 -> 30 valores
	// Usa o 29 e 30.5 no primeiro e no segundo por conta da divisao inteira ser floor
	int pos = ((-29 * dist) + (diff * 30.5)) / (diff);
	gfx_mono_generic_draw_horizontal_line(*x_atual + 70, pos, 1, GFX_PIXEL_SET);
	*x_atual = *x_atual >= 58 ? 0 : *x_atual + 1;
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
	
	xSemaphoreOled = xSemaphoreCreateBinary();
	if (xSemaphoreOled == NULL)
		printf("Falha ao criar semaforo\n");
		
	/* Start the scheduler. */
	vTaskStartScheduler();

  /* RTOS não deve chegar aqui !! */
	while(1){}

	/* Will only get here if there was insufficient memory to create the idle task. */
	return 0;
}
