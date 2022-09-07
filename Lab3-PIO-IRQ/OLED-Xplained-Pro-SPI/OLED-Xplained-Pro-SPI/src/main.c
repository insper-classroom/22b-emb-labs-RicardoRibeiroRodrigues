#include <asf.h>

#include "gfx_mono_ug_2832hsweg04.h"
#include "gfx_mono_text.h"
#include "sysfont.h"

/************************************************************************/
/* defines                                                              */
/************************************************************************/
// Configurações botao 1 da placa OLED
#define BUT_1_PIO PIOD
#define BUT_1_PIO_ID ID_PIOD
#define BUT_1_IDX 28
#define BUT_1_IDX_MASK (1u << BUT_1_IDX)

// Configurações botao 2 da placa OLED
#define BUT_2_PIO PIOC
#define BUT_2_PIO_ID ID_PIOC
#define BUT_2_IDX 31
#define BUT_2_IDX_MASK (1u << BUT_2_IDX)

// Configurações botao 3 da placa do OLED
#define BUT_3_PIO PIOA
#define BUT_3_PIO_ID ID_PIOA
#define BUT_3_IDX 19
#define BUT_3_IDX_MASK (1u << BUT_3_IDX)


// Configurações LED 1
#define LED_1_PIO PIOA
#define LED_1_PIO_ID ID_PIOA
#define LED_1_IDX 0
#define LED_1_IDX_MASK (1 << LED_1_IDX)

// Configurações LED 2
#define LED_2_PIO PIOC
#define LED_2_PIO_ID ID_PIOC
#define LED_2_IDX 30
#define LED_2_IDX_MASK (1 << LED_2_IDX)

// Numero de piscadas padrao LED
#define PISCADAS_PADRAO_LED 30


/************************************************************************/
/* variaveis globais                                                    */
/************************************************************************/
volatile char but_1_flag;
volatile char but_2_flag;
volatile char but_3_flag;
volatile int delay;

/************************************************************************/
/* prototype                                                            */
/************************************************************************/
void io_init(void);
void pisca_led(int n, int t);

/************************************************************************/
/* handler / callbacks                                                  */
/************************************************************************/

/*
 * Exemplo de callback para o botao, sempre que acontecer
 * ira piscar o led por 5 vezes
 *
 * !! Isso é um exemplo ruim, nao deve ser feito na pratica, !!
 * !! pois nao se deve usar delays dentro de interrupcoes    !!
 */
void but_1_callback(void)
{
	but_1_flag = 1;
}

void but_2_callback(void) 
{
	but_2_flag = 1;
}

void but_3_callback(void)
{
	but_3_flag = 1;
}


/************************************************************************/
/* funções                                                              */
/************************************************************************/

// pisca led N vez no periodo T
void pisca_led(int n, int t){
	// Desenha retangulo para o progresso
	gfx_mono_draw_rect(0, 16, 30, 15, GFX_PIXEL_SET);
	
	for (int i = 0; i < n; i++){
		if (but_2_flag) {
			but_2_flag = 0;
			break;
		}
		// Desenha o progresso do pisca led
		gfx_mono_draw_filled_rect(0, 16, 30 * i / n, 15, GFX_PIXEL_SET);
		
		pio_clear(LED_2_PIO, LED_2_IDX_MASK);
		delay_ms(t);
		pio_set(LED_2_PIO, LED_2_IDX_MASK);
		delay_ms(t);
	}
}

// Recebe o delay em ms e printa no OLED a frequencia
void printa_freq(int delay_arg) {
	// Print freq OLED
	char str[128];
	double delay_secs = (double) delay_arg / 1000;
	double freq = 1.0 / delay_secs;
	sprintf(str, "%lf hz", freq);
	gfx_mono_draw_string(str, 0, 0, &sysfont);
}

// Funcao para barra de progresso apos seu final
void progress_clear(void) {
	for (int x = 0; x < 40; x++) {
		for (int y = 16; y < 32; y++) {
			gfx_mono_draw_pixel(x, y, GFX_PIXEL_CLR);
		}
	}
}


void io_init(void) {
	// Configura led do OLED
	pmc_enable_periph_clk(LED_2_PIO_ID);
	pio_set_output(LED_2_PIO, LED_2_IDX_MASK, 1, 0, 0);
	
	// Inicializa PIO do botao 1 do OLED
	pmc_enable_periph_clk(BUT_1_PIO_ID);
	pio_set_input(BUT_1_PIO, BUT_1_IDX_MASK, PIO_DEFAULT | PIO_DEBOUNCE);
	pio_pull_up(BUT_1_PIO, BUT_1_IDX_MASK, 1);
	
	// Inicializa PIO do botao 2 do OLED
	pmc_enable_periph_clk(BUT_2_PIO_ID);
	pio_set_input(BUT_2_PIO, BUT_2_IDX_MASK, PIO_DEFAULT | PIO_DEBOUNCE);
	pio_pull_up(BUT_2_PIO, BUT_2_IDX_MASK, 1);
	
	 // Inicializa PIO do botao 3 do OLED
	 pmc_enable_periph_clk(BUT_3_PIO_ID);
	 pio_set_input(BUT_3_PIO, BUT_3_IDX_MASK, PIO_DEFAULT | PIO_DEBOUNCE);
	 pio_pull_up(BUT_3_PIO, BUT_3_IDX_MASK, 1);
	
	// Configura handler para o botao 1 para interrupcao
	pio_handler_set(BUT_1_PIO,
		BUT_1_PIO_ID,
		BUT_1_IDX_MASK,
		PIO_IT_FALL_EDGE,
		but_1_callback);
		
	// Configura handler para o botao 2 para interrupcao
	pio_handler_set(BUT_2_PIO,
		BUT_2_PIO_ID,
		BUT_2_IDX_MASK,
		PIO_IT_FALL_EDGE,
		but_2_callback);
		
	// Configura handler para o botao 3 para interrupcao
	pio_handler_set(BUT_3_PIO,
		BUT_3_PIO_ID,
		BUT_3_IDX_MASK,
		PIO_IT_FALL_EDGE,
		but_3_callback);	
	
	// Ativa interrupção e limpa primeira IRQ do botao 1 gerada na ativacao
	pio_enable_interrupt(BUT_1_PIO, BUT_1_IDX_MASK);
	pio_get_interrupt_status(BUT_1_PIO);
	
	// Ativa interrupção e limpa primeira IRQ do botao 2 gerada na ativacao
	pio_enable_interrupt(BUT_2_PIO, BUT_2_IDX_MASK);
	pio_get_interrupt_status(BUT_2_PIO);
	
	// Ativa interrupção e limpa primeira IRQ do botao 3 gerada na ativacao
	pio_enable_interrupt(BUT_3_PIO, BUT_3_IDX_MASK);
	pio_get_interrupt_status(BUT_3_PIO);
	
	
	// Configura NVIC para receber interrupcoes do PIO do botao 1
	// com prioridade 4 (quanto mais próximo de 0 maior)
	NVIC_EnableIRQ(BUT_1_PIO_ID);
	NVIC_SetPriority(BUT_1_PIO_ID, 4);
	
	// Configura NVIC para receber interrupcoes do PIO do botao 2
	// com prioridade 4 (quanto mais próximo de 0 maior)
	NVIC_EnableIRQ(BUT_2_PIO_ID);
	NVIC_SetPriority(BUT_2_PIO_ID, 4);
	
	// Configura NVIC para receber interrupcoes do PIO do botao 3
	// com prioridade 4 (quanto mais próximo de 0 maior)
	NVIC_EnableIRQ(BUT_3_PIO_ID);
	NVIC_SetPriority(BUT_3_PIO_ID, 4);
}


int main (void)
{
	board_init();
	sysclk_init();
	delay_init();
	
	
	// Desativa watchdog
	WDT->WDT_MR = WDT_MR_WDDIS;
	
	// Configura os LEDS e Botoes
	io_init();

	// Init OLED
	gfx_mono_ssd1306_init();
	
	delay = 800;
	printa_freq(delay);

	/* Insert application code here, after the board has been initialized. */
	while(1) {
	
			// Pisca o led no delay global
			if (but_1_flag | but_3_flag) {
				for (int i = 0; i < 1000000; i++) {
					// Se o soltar o botao e tiver passado menos de 20 iteracoes (2 segundos) -> aperta e solta.
					if (but_1_flag && (pio_get(BUT_1_PIO, PIO_INPUT, BUT_1_IDX_MASK) && (i < 20))) {
						delay -= 100;
						printa_freq(delay);
						pisca_led(PISCADAS_PADRAO_LED, delay);
						break;
					}
					// Aperto longo do botao 1 -> 2 segundos ou mais apertando o botao
					if (but_1_flag && (pio_get(BUT_1_PIO, PIO_INPUT, BUT_1_IDX_MASK) && (i >= 20))) {
						delay += 100;
						printa_freq(delay);
						pisca_led(PISCADAS_PADRAO_LED, delay);
						break;
					}
					// Soltando o botao 3
					if (but_3_flag && (pio_get(BUT_3_PIO, PIO_INPUT, BUT_3_IDX_MASK))) {
						// So diminui a freq se for um aperto curto
						if (i < 20) {
							delay += 100;
							printa_freq(delay);
						}
						pisca_led(PISCADAS_PADRAO_LED, delay);
						break;
					}
					delay_ms(100);
				}
				but_1_flag = 0;
				but_3_flag = 0;
				progress_clear();
			}
			
			
			// Entra em sleep mode
			pmc_sleep(SAM_PM_SMODE_SLEEP_WFI);
	}
}
