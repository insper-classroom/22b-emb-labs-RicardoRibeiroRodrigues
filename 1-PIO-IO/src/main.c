/**
 * 5 semestre - Eng. da Computação - Insper
 * Rafael Corsi - rafael.corsi@insper.edu.br
 *
 * Projeto 0 para a placa SAME70-XPLD
 *
 * Objetivo :
 *  - Introduzir ASF e HAL
 *  - Configuracao de clock
 *  - Configuracao pino In/Out
 *
 * Material :
 *  - Kit: ATMEL SAME70-XPLD - ARM CORTEX M7
 */

/************************************************************************/
/* includes                                                             */
/************************************************************************/

#include "asf.h"

/************************************************************************/
/* defines                                                              */
/************************************************************************/

#define LED_PIO      PIOC     
#define LED_PIO_ID    ID_PIOC
#define LED_PIO_IDX   8    
#define LED_PIO_IDX_MASK  (1 << LED_PIO_IDX)

// Configuracoes do botao placa
#define BUT_PIO   PIOA
#define BUT_PIO_ID ID_PIOA
#define BUT_PIO_IDX 11
#define BUT_PIO_IDX_MASK (1u << BUT_PIO_IDX) // esse já está pronto.

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

// Configurações LED 3
#define LED_3_PIO PIOB
#define LED_3_PIO_ID ID_PIOB
#define LED_3_IDX 2
#define LED_3_IDX_MASK (1 << LED_3_IDX)

// Configurações botao 1
#define BUT_1_PIO PIOD
#define BUT_1_PIO_ID ID_PIOD
#define BUT_1_IDX 28
#define BUT_1_IDX_MASK (1u << BUT_1_IDX)

// Configurações botao 2
#define BUT_2_PIO PIOC
#define BUT_2_PIO_ID ID_PIOC
#define BUT_2_IDX 31
#define BUT_2_IDX_MASK (1u << BUT_2_IDX)

// Configurações botao 3
#define BUT_3_PIO PIOA
#define BUT_3_PIO_ID ID_PIOA
#define BUT_3_IDX 19
#define BUT_3_IDX_MASK (1u << BUT_3_IDX)


/************************************************************************/
/* constants                                                            */
/************************************************************************/

/************************************************************************/
/* variaveis globais                                                    */
/************************************************************************/

/************************************************************************/
/* prototypes                                                           */
/************************************************************************/

void init(void);

/************************************************************************/
/* interrupcoes                                                         */
/************************************************************************/

/************************************************************************/
/* funcoes                                                              */
/************************************************************************/

// Função de inicialização do uC
void init(void)
{
	 // Initialize the board clock
	 sysclk_init();

	 // Desativa WatchDog Timer
	 WDT->WDT_MR = WDT_MR_WDDIS;
	 
	 // Ativa o PIO na qual o LED foi conectado
	 // para que possamos controlar o LED.
	 pmc_enable_periph_clk(LED_PIO_ID);
	 
	 pio_set_output(LED_PIO, LED_PIO_IDX_MASK, 0, 0, 0);
	 
	 // Ativa o PIO do led 1
	 pmc_enable_periph_clk(LED_1_PIO_ID);
	 
	 pio_set_output(LED_1_PIO, LED_1_IDX_MASK, 0, 0, 0);
	 
	 // Ativa o PIO do led 2
	 pmc_enable_periph_clk(LED_2_PIO_ID);
	 
	 pio_set_output(LED_2_PIO, LED_2_IDX_MASK, 0, 0, 0);
	 
	 // Ativa o PIO do led 3
	 pmc_enable_periph_clk(LED_3_PIO_ID);
	 
	 pio_set_output(LED_3_PIO, LED_3_IDX_MASK, 0, 0, 0);
	 
	 // Inicializa PIO do botao
	 pmc_enable_periph_clk(BUT_PIO_ID);
	
	 pio_set_input(BUT_PIO, BUT_PIO_IDX_MASK, PIO_DEFAULT);

	 pio_pull_up(BUT_PIO, BUT_PIO_IDX_MASK, 1);
	 
	 // Inicializa PIO do botao 1
	 pmc_enable_periph_clk(BUT_1_PIO_ID);
	 
	 pio_set_input(BUT_1_PIO, BUT_1_IDX_MASK, PIO_DEFAULT);

	 pio_pull_up(BUT_1_PIO, BUT_1_IDX_MASK, 1);
	 
	 // Inicializa PIO do botao 2
	 pmc_enable_periph_clk(BUT_2_PIO_ID);
	 
	 pio_set_input(BUT_2_PIO, BUT_2_IDX_MASK, PIO_DEFAULT);

	 pio_pull_up(BUT_2_PIO, BUT_2_IDX_MASK, 1);
	 
	 // Inicializa PIO do botao 3
	 pmc_enable_periph_clk(BUT_3_PIO_ID);
	 
	 pio_set_input(BUT_3_PIO, BUT_3_IDX_MASK, PIO_DEFAULT);

	 pio_pull_up(BUT_3_PIO, BUT_3_IDX_MASK, 1);
}

/************************************************************************/
/* Main                                                                 */
/************************************************************************/

// Funcao principal chamada na inicalizacao do uC.
int main(void)
{
  init();

  // super loop
  // aplicacoes embarcadas não devem sair do while(1).
  while (1)
  {
	  if (!pio_get(BUT_PIO, PIO_INPUT, BUT_PIO_IDX_MASK)) {
		  for (int i = 0; i < 5; i++){
			  pio_set(LED_PIO, LED_PIO_IDX_MASK);
			  delay_ms(200);
			  pio_clear(LED_PIO, LED_PIO_IDX_MASK);
			  delay_ms(200);
		  }
		} else {
			pio_set(LED_PIO, LED_PIO_IDX_MASK);
			delay_ms(200);
		  }
		  // Controle do led 1
		 if (!pio_get(BUT_1_PIO, PIO_INPUT, BUT_1_IDX_MASK)) {
			pio_clear(LED_1_PIO, LED_1_IDX_MASK);
			delay_ms(200);
			pio_set(LED_1_PIO, LED_1_IDX_MASK);
			  
		 } else {
			pio_set(LED_1_PIO, LED_1_IDX_MASK);
		 }
		 // Controle do led 2
		 if (!pio_get(BUT_2_PIO, PIO_INPUT, BUT_2_IDX_MASK)) {
			 pio_clear(LED_2_PIO, LED_2_IDX_MASK);
			 delay_ms(200);
			 pio_set(LED_2_PIO, LED_2_IDX_MASK);
			 
			 } else {
			 pio_set(LED_2_PIO, LED_2_IDX_MASK);
		 }
		 
		 // Controle do led 3
		 if (!pio_get(BUT_3_PIO, PIO_INPUT, BUT_3_IDX_MASK)) {
			 pio_clear(LED_3_PIO, LED_3_IDX_MASK);
			 delay_ms(200);
			 pio_set(LED_3_PIO, LED_3_IDX_MASK);
			
			 } else {
			 pio_set(LED_3_PIO, LED_3_IDX_MASK);
		 }

	  
  }
  return 0;
}
