/************************************************************************/
/* includes                                                             */
/************************************************************************/

#include <asf.h>
#include <string.h>
#include "ili9341.h"
#include "lvgl.h"
#include "touch/touch.h"
#include "clock24.h"
#include "hot_steam.h"

LV_FONT_DECLARE(dseg70);
LV_FONT_DECLARE(dseg50);
LV_FONT_DECLARE(dseg35);
LV_FONT_DECLARE(dseg40);

typedef struct  {
	uint32_t year;
	uint32_t month;
	uint32_t day;
	uint32_t week;
	uint32_t hour;
	uint32_t minute;
	uint32_t second;
} calendar;

#define AFEC_POT AFEC1
#define AFEC_POT_ID ID_AFEC1
#define AFEC_POT_CHANNEL 6 // Canal do pino PD30


// Prototypes
void RTC_init(Rtc *rtc, uint32_t id_rtc, calendar t, uint32_t irq_type);


/************************************************************************/
/* LCD / LVGL                                                           */
/************************************************************************/

#define LV_HOR_RES_MAX          (320)
#define LV_VER_RES_MAX          (240)

/*A static or global variable to store the buffers*/
static lv_disp_draw_buf_t disp_buf;

/*Static or global buffer(s). The second buffer is optional*/
static lv_color_t buf_1[LV_HOR_RES_MAX * LV_VER_RES_MAX];
static lv_disp_drv_t disp_drv;          /*A variable to hold the drivers. Must be static or global.*/
static lv_indev_drv_t indev_drv;

/************************************************************************/
/* RTOS                                                                 */
/************************************************************************/

#define TASK_LCD_STACK_SIZE                (1024*6/sizeof(portSTACK_TYPE))
#define TASK_LCD_STACK_PRIORITY            (tskIDLE_PRIORITY)

#define TASK_RTC_STACK_SIZE                (1024*6/sizeof(portSTACK_TYPE))
#define TASK_RTC_STACK_PRIORITY            (tskIDLE_PRIORITY)

extern void vApplicationStackOverflowHook(xTaskHandle *pxTask,  signed char *pcTaskName);
extern void vApplicationIdleHook(void);
extern void vApplicationTickHook(void);
extern void vApplicationMallocFailedHook(void);
extern void xPortSysTickHandler(void);

extern void vApplicationStackOverflowHook(xTaskHandle *pxTask, signed char *pcTaskName) {
	printf("stack overflow %x %s\r\n", pxTask, (portCHAR *)pcTaskName);
	for (;;) {	}
}

extern void vApplicationIdleHook(void) { }

extern void vApplicationTickHook(void) { }

extern void vApplicationMallocFailedHook(void) {
	configASSERT( ( volatile void * ) NULL );
}

SemaphoreHandle_t xSemaphoreHorario;
QueueHandle_t xQueueTemp;
volatile char is_clock_clicked = 0;
volatile char is_display_off = 0;


/**
 * \brief Interrupt handler for the RTC. 
 */
void RTC_Handler(void) {
    uint32_t ul_status = rtc_get_status(RTC);
	
    /* seccond tick */
    if ((ul_status & RTC_SR_SEC) == RTC_SR_SEC) {	
		// o código para irq de segundo vem aqui
		xSemaphoreGiveFromISR(xSemaphoreHorario, 0);
		afec_channel_enable(AFEC_POT, AFEC_POT_CHANNEL);
		afec_start_software_conversion(AFEC_POT);
    }
	
    /* Time or date alarm */
    if ((ul_status & RTC_SR_ALARM) == RTC_SR_ALARM) {
    	// o código para irq de alame vem aqui
	
    }
	
	rtc_clear_status(RTC, RTC_SCCR_SECCLR);
    rtc_clear_status(RTC, RTC_SCCR_ALRCLR);
    rtc_clear_status(RTC, RTC_SCCR_ACKCLR);
    rtc_clear_status(RTC, RTC_SCCR_TIMCLR);
    rtc_clear_status(RTC, RTC_SCCR_CALCLR);
    rtc_clear_status(RTC, RTC_SCCR_TDERRCLR);
}

static void AFEC_pot_Callback(void){
	 uint16_t g_ul_value = afec_channel_get_value(AFEC_POT, AFEC_POT_CHANNEL);
	 xQueueSendFromISR(xQueueTemp, &g_ul_value, 0);
	 afec_channel_disable(AFEC_POT, AFEC_POT_CHANNEL);	
}

/************************************************************************/
/* lvgl                                                                 */
/************************************************************************/
static lv_obj_t * labelBtn1;
static lv_obj_t * labelBtnMenu;
static lv_obj_t * labelBtnClock;
static lv_obj_t * labelBtnUp;
static lv_obj_t * labelBtnDown;
static lv_obj_t * labelFloor;
static lv_obj_t * labelFloorDecimal;
static lv_obj_t * labelFloorDegrees;
static lv_obj_t * labelClock;
static lv_obj_t * labelSetValue;
static lv_obj_t * labelSetDegrees;
static lv_obj_t * labelDayWeek;
static lv_obj_t * labelSet;
static lv_obj_t * labelBtnHome;
static lv_obj_t * labelFloorTemp;
static lv_obj_t * labelSetConfig;


static void event_handler(lv_event_t * e) {
	lv_event_code_t code = lv_event_get_code(e);

	if(code == LV_EVENT_CLICKED) {
		LV_LOG_USER("Clicked");
	}
	else if(code == LV_EVENT_VALUE_CHANGED) {
		LV_LOG_USER("Toggled");
	}
}

void menu_handler(lv_event_t *e) {
	lv_event_code_t code = lv_event_get_code(e);
	
	if(code == LV_EVENT_CLICKED) {
		LV_LOG_USER("Clicked");
	}
	else if(code == LV_EVENT_VALUE_CHANGED) {
		LV_LOG_USER("Toggled");
	}
}

void power_handler(lv_event_t *e) {
		lv_event_code_t code = lv_event_get_code(e);
		
		if(code == LV_EVENT_CLICKED) {
			printf("Clicka no power\n");
			if (!is_display_off) {
				is_display_off = 1;
				lv_obj_clean(lv_scr_act());
			}
			
		}
		else if(code == LV_EVENT_VALUE_CHANGED) {
			LV_LOG_USER("Toggled");
		}
}

void home_handler(lv_event_t *e) {
	lv_event_code_t code = lv_event_get_code(e);
	
	if(code == LV_EVENT_CLICKED) {
		LV_LOG_USER("Clicked");
	}
	else if(code == LV_EVENT_VALUE_CHANGED) {
		LV_LOG_USER("Toggled");
	}
}

void clk_handler(lv_event_t *e) {
	lv_event_code_t code = lv_event_get_code(e);
	
	if(code == LV_EVENT_CLICKED) {
		LV_LOG_USER("Clicked");
		is_clock_clicked = !is_clock_clicked;
		printf("Click! clock\n");
	}
	else if(code == LV_EVENT_VALUE_CHANGED) {
		LV_LOG_USER("Toggled");
	}
}

void down_handler(lv_event_t *e) {
	lv_event_code_t code = lv_event_get_code(e);
	char *c;
	int temp;
	if(code == LV_EVENT_CLICKED) {
		printf("Clicked down\n");
		LV_LOG_USER("Clicked");
		if (!is_clock_clicked) {
			c = lv_label_get_text(labelSetValue);
			temp = atoi(c);
			lv_label_set_text_fmt(labelSetValue, "%02d", temp - 1);
		} else {
			uint32_t current_hour, current_min, current_sec;
			rtc_get_time(RTC, &current_hour, &current_min, &current_sec);
			current_min--;
			if (current_min < 0) {
				(current_hour > 0) && (current_hour--);
				current_min = 0;
			}
			rtc_set_time(RTC, current_hour, current_min, current_sec);
			lv_label_set_text_fmt(labelClock, "%02d:%02d", current_hour, current_min);
		}
	} else if(code == LV_EVENT_VALUE_CHANGED) {
		LV_LOG_USER("Toggled");
	}
}

void up_handler(lv_event_t *e) {
	lv_event_code_t code = lv_event_get_code(e);
	char *c;
	int temp;
	if(code == LV_EVENT_CLICKED) {
		printf("Clicked up\n");
		if (!is_clock_clicked) {
			c = lv_label_get_text(labelSetValue);
			temp = atoi(c);
			lv_label_set_text_fmt(labelSetValue, "%02d", temp + 1);
		} else {
			uint32_t current_hour, current_min, current_sec;
			rtc_get_time(RTC, &current_hour, &current_min, &current_sec);
			current_min++;
			if (current_min >= 60) {
				current_hour++;
				current_min -= 60;
			}
			rtc_set_time(RTC, current_hour, current_min, current_sec);
			lv_label_set_text_fmt(labelClock, "%02d:%02d", current_hour, current_min);
		}
	}
}

void lv_ex_btn_1(void) {
	lv_obj_t * label;

	lv_obj_t * btn1 = lv_btn_create(lv_scr_act());
	lv_obj_add_event_cb(btn1, event_handler, LV_EVENT_ALL, NULL);
	lv_obj_align(btn1, LV_ALIGN_CENTER, 0, -40);

	label = lv_label_create(btn1);
	lv_label_set_text(label, "Corsi");
	lv_obj_center(label);

	lv_obj_t * btn2 = lv_btn_create(lv_scr_act());
	lv_obj_add_event_cb(btn2, event_handler, LV_EVENT_ALL, NULL);
	lv_obj_align(btn2, LV_ALIGN_CENTER, 0, 40);
	lv_obj_add_flag(btn2, LV_OBJ_FLAG_CHECKABLE);
	lv_obj_set_height(btn2, LV_SIZE_CONTENT);

	label = lv_label_create(btn2);
	lv_label_set_text(label, "Toggle");
	lv_obj_center(label);
}

void lv_termostato(void) {
	
	lv_obj_clear_flag(lv_scr_act(), LV_OBJ_FLAG_SCROLLABLE);
	
	// Estilo -> bg preto e texto branco.
	static lv_style_t style;
	lv_style_init(&style);
	lv_style_set_bg_color(&style, lv_color_black());
	lv_style_set_border_color(&style, lv_color_white());
	lv_style_set_border_width(&style, 0);
	lv_style_set_text_color(&style, lv_color_white());
	
	// Botao power
	lv_obj_t * btn1 = lv_btn_create(lv_scr_act());
	lv_obj_add_event_cb(btn1, power_handler, LV_EVENT_ALL, NULL);
	lv_obj_align(btn1, LV_ALIGN_BOTTOM_LEFT, 20, -30);
	lv_obj_add_style(btn1, &style, 0);
	
	labelBtn1 = lv_label_create(btn1);
	lv_label_set_text(labelBtn1, "[ " LV_SYMBOL_POWER);
	lv_obj_center(labelBtn1);
	
	// Botao menu
	lv_obj_t *btnMenu = lv_btn_create(lv_scr_act());
	lv_obj_add_event_cb(btnMenu, menu_handler, LV_EVENT_ALL, NULL);
	lv_obj_align_to(btnMenu, btn1, LV_ALIGN_OUT_RIGHT_MID, -8, -10);
	lv_obj_add_style(btnMenu, &style, 0);

	labelBtnMenu = lv_label_create(btnMenu);
	lv_label_set_text(labelBtnMenu, "| M |" );
	lv_obj_center(labelBtnMenu);
	
	// Botao clock
	
	static lv_style_t style_def;

	lv_style_init(&style_def); /*Darken the button when pressed and make it wider*/
	static lv_style_t style_pr;
	lv_style_init(&style_pr);
	lv_style_set_img_recolor_opa(&style_pr, LV_OPA_30);
	lv_style_set_img_recolor(&style_pr, lv_color_white());
	
	lv_obj_t *btnClk = lv_imgbtn_create(lv_scr_act());
	lv_imgbtn_set_src(btnClk, LV_IMGBTN_STATE_RELEASED, &clock24, NULL, NULL);
    lv_obj_add_style(btnClk, &style_def, 0);
	lv_obj_add_style(btnClk, &style_pr, LV_STATE_PRESSED);
	lv_obj_add_event_cb(btnClk, clk_handler, LV_EVENT_ALL, NULL);
	lv_obj_align(btnClk, LV_ALIGN_CENTER, 30, 125); 

	labelBtnClock = lv_label_create(lv_scr_act());
	lv_obj_add_style(labelBtnClock, &style, 0);
	lv_obj_align_to(labelBtnClock, btn1, LV_ALIGN_OUT_RIGHT_MID, 83, 0);
	lv_label_set_text_fmt(labelBtnClock, "]");
	
	
	// Seta para baixo
	lv_obj_t * btnDown = lv_btn_create(lv_scr_act());
	lv_obj_add_event_cb(btnDown, down_handler, LV_EVENT_ALL, NULL);
	lv_obj_align(btnDown, LV_ALIGN_BOTTOM_RIGHT, -5, -30);
	lv_obj_add_style(btnDown, &style, 0);
	
	labelBtnDown = lv_label_create(btnDown);
	lv_label_set_text(labelBtnDown, LV_SYMBOL_DOWN " ]");
	lv_obj_center(labelBtnDown);

	// Seta para cima
	lv_obj_t * btnUp = lv_btn_create(lv_scr_act());
	lv_obj_add_event_cb(btnUp, up_handler, LV_EVENT_ALL, NULL);
	lv_obj_align_to(btnUp, btnDown, LV_ALIGN_OUT_LEFT_MID, -40, -10);
	lv_obj_add_style(btnUp, &style, 0);
	
	labelBtnUp = lv_label_create(btnUp);
	lv_label_set_text(labelBtnUp, "[ " LV_SYMBOL_UP);
	lv_obj_center(labelBtnUp);
	
	// Icon casa
	lv_obj_t *btnHome = lv_btn_create(lv_scr_act());
	lv_obj_add_event_cb(btnHome, home_handler, LV_EVENT_ALL, NULL);
	lv_obj_align_to(btnHome, btnUp, LV_ALIGN_OUT_LEFT_TOP, -10, -32);
	lv_obj_add_style(btnHome, &style, 0);
	
	labelBtnHome = lv_label_create(btnHome);
	lv_label_set_text(labelBtnHome, LV_SYMBOL_HOME);
	lv_obj_center(labelBtnHome);
	
	
	// -------------------------------- Labels --------------------------------
	
	static lv_style_t sevenSegStyle;
	lv_style_init(&sevenSegStyle);
	lv_style_set_border_width(&sevenSegStyle, 0);
	lv_style_set_text_color(&sevenSegStyle, lv_color_white());

	// Label temperatura
	labelFloor = lv_label_create(lv_scr_act());
	lv_obj_align(labelFloor, LV_ALIGN_LEFT_MID, 55, -20);
	lv_obj_add_style(labelFloor, &sevenSegStyle, 0);
	lv_obj_set_style_text_font(labelFloor, &dseg70, LV_STATE_DEFAULT);
	lv_label_set_text_fmt(labelFloor, "%02d", 23);
	
	// Label decimal temp
	labelFloorDecimal = lv_label_create(lv_scr_act());
	lv_obj_align_to(labelFloorDecimal, labelFloor, LV_ALIGN_OUT_RIGHT_MID, 6, 10);
	lv_obj_add_style(labelFloorDecimal, &sevenSegStyle, 0);
	lv_obj_set_style_text_font(labelFloorDecimal, &dseg35, LV_STATE_DEFAULT);
	lv_label_set_text_fmt(labelFloorDecimal, ".%d", 4);
	
	// °C -> floor
	labelFloorDegrees = lv_label_create(lv_scr_act());
	lv_obj_align_to(labelFloorDegrees, labelFloor, LV_ALIGN_OUT_RIGHT_MID, 8, -22);
	lv_obj_add_style(labelFloorDegrees, &sevenSegStyle, 0);
	lv_label_set_text(labelFloorDegrees, "°C");
	
	// Label floor temp
	labelFloorTemp = lv_label_create(lv_scr_act());
	lv_obj_add_style(labelFloorTemp, &sevenSegStyle, 0);
	lv_obj_align_to(labelFloorTemp, labelFloor, LV_ALIGN_OUT_LEFT_MID, 1, 0);
	lv_label_set_text(labelFloorTemp, "FLOOR\nTEMP");
	lv_obj_set_style_text_font(labelFloorTemp, &lv_font_montserrat_14, LV_STATE_DEFAULT);
	
	// Day of the week
	labelDayWeek = lv_label_create(lv_scr_act());
	lv_obj_align_to(labelDayWeek, labelFloor, LV_ALIGN_OUT_TOP_LEFT, 10, -10);
	lv_obj_add_style(labelDayWeek, &sevenSegStyle, 0);
	lv_label_set_text(labelDayWeek, "MON");
	
	
	// Label temp config
	labelSetValue = lv_label_create(lv_scr_act());
	lv_obj_align(labelSetValue, LV_ALIGN_RIGHT_MID, -15, -30);
	lv_obj_add_style(labelSetValue, &sevenSegStyle, 0);
	lv_obj_set_style_text_font(labelSetValue, &dseg40, LV_STATE_DEFAULT);
	lv_label_set_text_fmt(labelSetValue, "%02d", 22);
	
	// °C -> set
	labelSetDegrees = lv_label_create(lv_scr_act());
	lv_obj_align_to(labelSetDegrees, labelSetValue, LV_ALIGN_OUT_RIGHT_MID, 2, -8);
	lv_obj_add_style(labelSetDegrees, &sevenSegStyle, 0);
	lv_obj_set_style_text_font(labelSetDegrees, &lv_font_montserrat_14, LV_STATE_DEFAULT);
	lv_label_set_text(labelSetDegrees, "°C");
	
	// Set
	labelSet = lv_label_create(lv_scr_act());
	lv_obj_align_to(labelSet, labelSetValue, LV_ALIGN_OUT_LEFT_MID, 20, 20);
	lv_obj_add_style(labelSet, &sevenSegStyle, 0);
	lv_obj_set_style_text_font(labelSet, &lv_font_montserrat_14, LV_STATE_DEFAULT);
	lv_label_set_text(labelSet, "SET");
	
	// Set config
	labelSetConfig = lv_label_create(lv_scr_act());
	lv_obj_align_to(labelSetConfig, labelSetValue, LV_ALIGN_OUT_LEFT_MID, 20, -10);
	lv_obj_add_style(labelSetConfig, &sevenSegStyle, 0);
	lv_obj_set_style_text_font(labelSet, &lv_font_montserrat_14, LV_STATE_DEFAULT);
	lv_label_set_text(labelSetConfig, LV_SYMBOL_SETTINGS);
	
	// Set steam
	lv_obj_t *steam_icon = lv_imgbtn_create(lv_scr_act());
	lv_obj_add_event_cb(steam_icon, event_handler, LV_EVENT_ALL, NULL);
	lv_imgbtn_set_src(steam_icon, LV_IMGBTN_STATE_RELEASED, &hot_steam, NULL, NULL);
	lv_obj_align_to(steam_icon, labelSetValue, LV_ALIGN_OUT_BOTTOM_RIGHT, 105, 5);
	
	// Set clock
	lv_obj_t *btnSetClock = lv_imgbtn_create(lv_scr_act());
	lv_imgbtn_set_src(btnSetClock, LV_IMGBTN_STATE_RELEASED, &clock24, NULL, NULL);
	lv_obj_add_event_cb(btnSetClock, event_handler, LV_EVENT_ALL, NULL);
	lv_obj_align_to(btnSetClock, labelSetValue, LV_ALIGN_OUT_BOTTOM_RIGHT, 70, 10);
	
	
	// Label clock
	labelClock = lv_label_create(lv_scr_act());
	lv_obj_align(labelClock, LV_ALIGN_TOP_RIGHT, -10, 10);
	lv_obj_add_style(labelClock, &sevenSegStyle, 0);
	lv_obj_set_style_text_font(labelClock, &dseg35, LV_STATE_DEFAULT);
	lv_label_set_text_fmt(labelClock, "%02d:%02d", 17, 46);
}

/************************************************************************/
/* TASKS                                                                */
/************************************************************************/

static void task_lcd(void *pvParameters) {
	int px, py;

	// lv_ex_btn_1();
	lv_termostato();
	printf("Comeca\n");
	
	
	for (;;)  {
		lv_tick_inc(50);
		lv_task_handler();
		vTaskDelay(50);
	}
}

static void task_rtc(void *pvParameters) {
	
	/** Configura RTC -> Usa o RTC para atualizar o horario e temperatura todo segundo.**/
	calendar rtc_initial = {2022, 11, 19, 12, 14, 01, 1};
	RTC_init(RTC, ID_RTC, rtc_initial, RTC_IER_SECEN);
	
	uint32_t current_hour, current_min, current_sec;
	char str[128];
	uint16_t temp_brute;
			
	
	while(1) {
		if ((xSemaphoreTake(xSemaphoreHorario, 0) == pdTRUE) && (!is_display_off)) {
			rtc_get_time(RTC, &current_hour, &current_min, &current_sec);
			lv_label_set_text_fmt(labelClock, "%02d:%02d", current_hour, current_min);
		}
		if (xQueueReceive(xQueueTemp, &temp_brute, 0) && (!is_display_off)) {
			float temp = (float)temp_brute / 80.0;
			int temp_int = (int) temp;
			uint decimal = ((float) temp - temp_int) * 10;
			lv_label_set_text_fmt(labelFloor, "%02d", temp_int);
			lv_label_set_text_fmt(labelFloorDecimal, ".%01d", decimal);
		}
	}
}


/************************************************************************/
/* configs                                                              */
/************************************************************************/

static void configure_lcd(void) {
	/**LCD pin configure on SPI*/
	pio_configure_pin(LCD_SPI_MISO_PIO, LCD_SPI_MISO_FLAGS);  //
	pio_configure_pin(LCD_SPI_MOSI_PIO, LCD_SPI_MOSI_FLAGS);
	pio_configure_pin(LCD_SPI_SPCK_PIO, LCD_SPI_SPCK_FLAGS);
	pio_configure_pin(LCD_SPI_NPCS_PIO, LCD_SPI_NPCS_FLAGS);
	pio_configure_pin(LCD_SPI_RESET_PIO, LCD_SPI_RESET_FLAGS);
	pio_configure_pin(LCD_SPI_CDS_PIO, LCD_SPI_CDS_FLAGS);
	
	ili9341_init();
	ili9341_backlight_on();
}

static void configure_console(void) {
	const usart_serial_options_t uart_serial_options = {
		.baudrate = USART_SERIAL_EXAMPLE_BAUDRATE,
		.charlength = USART_SERIAL_CHAR_LENGTH,
		.paritytype = USART_SERIAL_PARITY,
		.stopbits = USART_SERIAL_STOP_BIT,
	};

	/* Configure console UART. */
	stdio_serial_init(CONSOLE_UART, &uart_serial_options);

	/* Specify that stdout should not be buffered. */
	setbuf(stdout, NULL);
}

/**
* Configura o RTC para funcionar com interrupcao de alarme
*/
void RTC_init(Rtc *rtc, uint32_t id_rtc, calendar t, uint32_t irq_type) {
	/* Configura o PMC */
	pmc_enable_periph_clk(ID_RTC);

	/* Default RTC configuration, 24-hour mode */
	rtc_set_hour_mode(rtc, 0);

	/* Configura data e hora manualmente */
	rtc_set_date(rtc, t.year, t.month, t.day, t.week);
	rtc_set_time(rtc, t.hour, t.minute, t.second);

	/* Configure RTC interrupts */
	NVIC_DisableIRQ(id_rtc);
	NVIC_ClearPendingIRQ(id_rtc);
	NVIC_SetPriority(id_rtc, 4);
	NVIC_EnableIRQ(id_rtc);

	/* Ativa interrupcao via alarme */
	rtc_enable_interrupt(rtc,  irq_type);
}

static void config_AFEC_pot(Afec *afec, uint32_t afec_id, uint32_t afec_channel, afec_callback_t callback){
  /*************************************
  * Ativa e configura AFEC
  *************************************/
  /* Ativa AFEC - 0 */
  afec_enable(afec);

  /* struct de configuracao do AFEC */
  struct afec_config afec_cfg;

  /* Carrega parametros padrao */
  afec_get_config_defaults(&afec_cfg);

  /* Configura AFEC */
  afec_init(afec, &afec_cfg);

  /* Configura trigger por software */
  afec_set_trigger(afec, AFEC_TRIG_SW);

  /*** Configuracao específica do canal AFEC ***/
  struct afec_ch_config afec_ch_cfg;
  afec_ch_get_config_defaults(&afec_ch_cfg);
  afec_ch_cfg.gain = AFEC_GAINVALUE_0;
  afec_ch_set_config(afec, afec_channel, &afec_ch_cfg);

  /*
  * Calibracao:
  * Because the internal ADC offset is 0x200, it should cancel it and shift
  down to 0.
  */
  afec_channel_set_analog_offset(afec, afec_channel, 0x200);

  /***  Configura sensor de temperatura ***/
  struct afec_temp_sensor_config afec_temp_sensor_cfg;

  afec_temp_sensor_get_config_defaults(&afec_temp_sensor_cfg);
  afec_temp_sensor_set_config(afec, &afec_temp_sensor_cfg);
  
  /* configura IRQ */
  afec_set_callback(afec, afec_channel,	callback, 1);
  NVIC_SetPriority(afec_id, 4);
  NVIC_EnableIRQ(afec_id);
}


/**
* @Brief Inverte o valor do pino 0->1/ 1->0
*/
void pin_toggle(Pio *pio, uint32_t mask) {
	if(pio_get_output_data_status(pio, mask))
	pio_clear(pio, mask);
	else
	pio_set(pio,mask);
}


/************************************************************************/
/* port lvgl                                                            */
/************************************************************************/

void my_flush_cb(lv_disp_drv_t * disp_drv, const lv_area_t * area, lv_color_t * color_p) {
	ili9341_set_top_left_limit(area->x1, area->y1);   
	ili9341_set_bottom_right_limit(area->x2, area->y2);
	ili9341_copy_pixels_to_screen(color_p,  (area->x2 + 1 - area->x1) * (area->y2 + 1 - area->y1));
	
	/* IMPORTANT!!!
	* Inform the graphics library that you are ready with the flushing*/
	lv_disp_flush_ready(disp_drv);
}

void my_input_read(lv_indev_drv_t * drv, lv_indev_data_t*data) {
	int px, py, pressed;
	
	if (readPoint(&px, &py)) {
		data->state = LV_INDEV_STATE_PRESSED;
		if (is_display_off) {
			is_display_off = 0;
			lv_termostato();
		}
	}
	else
		data->state = LV_INDEV_STATE_RELEASED; 
	
	data->point.x = px;
	data->point.y = py;
}



void configure_lvgl(void) {
	lv_init();
	lv_disp_draw_buf_init(&disp_buf, buf_1, NULL, LV_HOR_RES_MAX * LV_VER_RES_MAX);
	
	lv_disp_drv_init(&disp_drv);            /*Basic initialization*/
	disp_drv.draw_buf = &disp_buf;          /*Set an initialized buffer*/
	disp_drv.flush_cb = my_flush_cb;        /*Set a flush callback to draw to the display*/
	disp_drv.hor_res = LV_HOR_RES_MAX;      /*Set the horizontal resolution in pixels*/
	disp_drv.ver_res = LV_VER_RES_MAX;      /*Set the vertical resolution in pixels*/

	lv_disp_t * disp;
	disp = lv_disp_drv_register(&disp_drv); /*Register the driver and save the created display objects*/
	
	/* Init input on LVGL */
	lv_indev_drv_init(&indev_drv);
	indev_drv.type = LV_INDEV_TYPE_POINTER;
	indev_drv.read_cb = my_input_read;
	lv_indev_t * my_indev = lv_indev_drv_register(&indev_drv);
}

/************************************************************************/
/* main                                                                 */
/************************************************************************/
int main(void) {
	/* board and sys init */
	board_init();
	sysclk_init();
	configure_console();

	/* LCd, touch and lvgl init*/
	configure_lcd();
	configure_touch();
	configure_lvgl();
	// Canal do pino PC31
	config_AFEC_pot(AFEC_POT, AFEC_POT_ID, AFEC_POT_CHANNEL, AFEC_pot_Callback);
	
	xSemaphoreHorario = xSemaphoreCreateBinary();
	if (xSemaphoreHorario == NULL) {
		printf("Failed to create semaphore \n");
	}
	xQueueTemp = xQueueCreate(32, sizeof(uint16_t));
	if (xQueueTemp == NULL) {
	  printf("falha em criar a fila \n");
	}

	/* Create task to control oled */
	if (xTaskCreate(task_lcd, "LCD", TASK_LCD_STACK_SIZE, NULL, TASK_LCD_STACK_PRIORITY, NULL) != pdPASS) {
		printf("Failed to create lcd task\r\n");
	}
	if (xTaskCreate(task_rtc, "RTC", TASK_RTC_STACK_SIZE, NULL, TASK_RTC_STACK_PRIORITY, NULL) != pdPASS) {
		printf("Failed to create lcd task\r\n");
	}
	
	/* Start the scheduler. */
	vTaskStartScheduler();

	while(1){ }
}
