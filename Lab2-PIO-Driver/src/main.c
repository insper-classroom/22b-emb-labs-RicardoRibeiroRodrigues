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

/*  Default pin configuration (no attribute). */
#define _PIO_DEFAULT             (0u << 0)
/*  The internal pin pull-up is active. */
#define _PIO_PULLUP              (1u << 0)
/*  The internal glitch filter is active. */
#define _PIO_DEGLITCH            (1u << 1)
/*  The internal debouncing filter is active. */
#define _PIO_DEBOUNCE            (1u << 3)

// Config led da placa.
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
/**
 * \brief Set a high output level on all the PIOs defined in ul_mask.
 * This has no immediate effects on PIOs that are not output, but the PIO
 * controller will save the value if they are changed to outputs.
 *
 * \param p_pio Pointer to a PIO instance.
 * \param ul_mask Bitmask of one or more pin(s) to configure.
 */
void _pio_set(Pio *p_pio, const uint32_t ul_mask)
{
	p_pio->PIO_SODR = ul_mask;
}

/**
 * \brief Set a low output level on all the PIOs defined in ul_mask.
 * This has no immediate effects on PIOs that are not output, but the PIO
 * controller will save the value if they are changed to outputs.
 *
 * \param p_pio Pointer to a PIO instance.
 * \param ul_mask Bitmask of one or more pin(s) to configure.
 */
void _pio_clear(Pio *p_pio, const uint32_t ul_mask)
{
	p_pio->PIO_CODR = ul_mask;
}

/**
 * \brief Configure PIO internal pull-up.
 *
 * \param p_pio Pointer to a PIO instance.
 * \param ul_mask Bitmask of one or more pin(s) to configure.
 * \param ul_pull_up_enable Indicates if the pin(s) internal pull-up shall be
 * configured.
 */
void _pio_pull_up(Pio *p_pio, const uint32_t ul_mask, const uint32_t ul_pull_up_enable){
	if (ul_pull_up_enable) {
		p_pio->PIO_PUER = ul_mask;
	} else {
		p_pio->PIO_PUDR = ul_mask;
	}
 }
 
 /**
 * \brief Configure one or more pin(s) or a PIO controller as inputs.
 * Optionally, the corresponding internal pull-up(s) and glitch filter(s) can
 * be enabled.
 *
 * \param p_pio Pointer to a PIO instance.
 * \param ul_mask Bitmask indicating which pin(s) to configure as input(s).
 * \param ul_attribute PIO attribute(s).
 */
void _pio_set_input(Pio *p_pio, const uint32_t ul_mask, const uint32_t ul_attribute)
{
	if (ul_attribute & _PIO_DEGLITCH) {
		p_pio->PIO_IFSCDR = ul_mask;
	} else if (ul_attribute & _PIO_DEBOUNCE) {
		p_pio->PIO_IFSCER = ul_mask;
	}
	_pio_pull_up(p_pio, ul_mask, ul_attribute & _PIO_PULLUP);
}


void _pio_set_output(Pio *p_pio, const uint32_t ul_mask,
const uint32_t ul_default_level,
const uint32_t ul_multidrive_enable,
const uint32_t ul_pull_up_enable)
{
	p_pio->PIO_PER = ul_mask;
	p_pio->PIO_OER = ul_mask;
	if (ul_default_level) {
		_pio_set(p_pio, ul_mask);
	} else {
		_pio_clear(p_pio, ul_mask);
	}
	if (ul_multidrive_enable) {
		p_pio->PIO_MDER = ul_mask;
	} else {
		p_pio->PIO_MDDR = ul_mask;
	}
	_pio_pull_up(p_pio, ul_mask, ul_pull_up_enable);
}

/**
 * \brief Return 1 if one or more PIOs of the given Pin instance currently have
 * a high level; otherwise returns 0. This method returns the actual value that
 * is being read on the pin. To return the supposed output value of a pin, use
 * pio_get_output_data_status() instead.
 *
 * \param p_pio Pointer to a PIO instance.
 * \param ul_type PIO type.
 * \param ul_mask Bitmask of one or more pin(s) to configure.
 *
 * \retval 1 at least one PIO currently has a high level.
 * \retval 0 all PIOs have a low level.
 */
uint32_t _pio_get(Pio *p_pio, const pio_type_t ul_type, const uint32_t ul_mask){
	// Para ler saidas
	if (ul_type == PIO_OUTPUT_0) {
		return p_pio->PIO_ODSR & ul_mask;
	}
	// Para ler entradas.
	return p_pio->PIO_PDSR & ul_mask;
}

void _delay_ms(int time) {
	for (int i = 0; i < time * 150000; i++) {
		asm("NOP");
	}
}

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
	 
	 _pio_set_output(LED_PIO, LED_PIO_IDX_MASK, 0, 0, 0);
	 
	 // Ativa o PIO do led 1
	 pmc_enable_periph_clk(LED_1_PIO_ID);
	 
	 _pio_set_output(LED_1_PIO, LED_1_IDX_MASK, 0, 0, 0);
	 
	 // Ativa o PIO do led 2
	 pmc_enable_periph_clk(LED_2_PIO_ID);
	 
	 _pio_set_output(LED_2_PIO, LED_2_IDX_MASK, 0, 0, 0);
	 
	 // Ativa o PIO do led 3
	 pmc_enable_periph_clk(LED_3_PIO_ID);
	 _pio_set_output(LED_3_PIO, LED_3_IDX_MASK, 0, 0, 0);
	 
	 // Inicializa PIO do botao
	 pmc_enable_periph_clk(BUT_PIO_ID);
	
	 _pio_set_input(BUT_PIO, BUT_PIO_IDX_MASK, _PIO_PULLUP | _PIO_DEBOUNCE);

	 
	 // Inicializa PIO do botao 1
	 pmc_enable_periph_clk(BUT_1_PIO_ID);
	 
	 _pio_set_input(BUT_1_PIO, BUT_1_IDX_MASK, _PIO_PULLUP | _PIO_DEBOUNCE);
	 
	 // Inicializa PIO do botao 2
	 pmc_enable_periph_clk(BUT_2_PIO_ID);
	 _pio_set_input(BUT_2_PIO, BUT_2_IDX_MASK, _PIO_PULLUP | _PIO_DEBOUNCE);
	 
	 
	 // Inicializa PIO do botao 3
	 pmc_enable_periph_clk(BUT_3_PIO_ID);
	 _pio_set_input(BUT_3_PIO, BUT_3_IDX_MASK, _PIO_PULLUP | _PIO_DEBOUNCE);

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
	  if (!_pio_get(BUT_PIO, PIO_INPUT, BUT_PIO_IDX_MASK)) {
		  for (int i = 0; i < 5; i++){
			  _pio_set(LED_PIO, LED_PIO_IDX_MASK);
			  _delay_ms(200);
			  _pio_clear(LED_PIO, LED_PIO_IDX_MASK);
			  _delay_ms(200);
		  }
		} else {
			_pio_set(LED_PIO, LED_PIO_IDX_MASK);
			_delay_ms(200);
		  }
		  // Controle do led 1
		 if (!_pio_get(BUT_1_PIO, PIO_INPUT, BUT_1_IDX_MASK)) {
			_pio_clear(LED_1_PIO, LED_1_IDX_MASK);
			_delay_ms(200);
			_pio_set(LED_1_PIO, LED_1_IDX_MASK);
			  
		 } else {
			_pio_set(LED_1_PIO, LED_1_IDX_MASK);
		 }
		 // Controle do led 2
		 if (!_pio_get(BUT_2_PIO, PIO_INPUT, BUT_2_IDX_MASK)) {
			 _pio_clear(LED_2_PIO, LED_2_IDX_MASK);
			 _delay_ms(200);
			 _pio_set(LED_2_PIO, LED_2_IDX_MASK);
			 
			 } else {
			 _pio_set(LED_2_PIO, LED_2_IDX_MASK);
		 }
		 
		 // Controle do led 3
		 if (!_pio_get(BUT_3_PIO, PIO_INPUT, BUT_3_IDX_MASK)) {
			 _pio_clear(LED_3_PIO, LED_3_IDX_MASK);
			 _delay_ms(200);
			 _pio_set(LED_3_PIO, LED_3_IDX_MASK);
			
			 } else {
			 _pio_set(LED_3_PIO, LED_3_IDX_MASK);
		 }

	  
  }
  return 0;
}
