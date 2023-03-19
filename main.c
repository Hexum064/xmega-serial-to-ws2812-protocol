/*
 * xmega32e5-uart-to-ws2812.c
 *
 * Created: 2023-02-16 20:29:58
 * Author : Branden
 */ 

//
#define F_CPU 32000000U
#include <avr/io.h>
//#include <util/delay.h>
#include <avr/interrupt.h>


//# define cli()  __asm__ __volatile__ ("cli" ::: "memory")
//# define sei()  __asm__ __volatile__ ("sei" ::: "memory")

volatile uint8_t bytes[3000];
volatile uint16_t byte_count = 0;
volatile uint16_t rcv_index = 0;
//added the next three globally for debugging
volatile uint8_t hi_byte = 0;
volatile uint8_t lo_byte = 0;
volatile uint8_t setting = 0;
uint8_t baudctrla[16] = {12, 12, 12, 138, 12, 137, 12, 135, 12, 131, 123, 107, 75, 57, 11, 0};
int8_t baudctrlb[16] = {6, 5, 4, 0, 3, -1, 2, -2, 1, -3, -4, -5, -6, -7, -7, 0};

uint8_t selected_baudctrla;
int8_t selected_baudctrlb;

enum RcvState
{
	HiByte = 0,
	LoByte = 1,
	Data = 2,
	Trans = 3,
	Timeout = 4,
};

volatile enum RcvState rcvState = HiByte;

void read_baud_switches()
{
	PORTA.OUTCLR = PIN1_bm | PIN2_bm | PIN3_bm | PIN4_bm;
	PORTA_PIN1CTRL = PORT_OPC_PULLUP_gc | PORT_ISC_FORCE_ENABLE_gc;
	PORTA_PIN2CTRL = PORT_OPC_PULLUP_gc | PORT_ISC_FORCE_ENABLE_gc; 
	PORTA_PIN3CTRL = PORT_OPC_PULLUP_gc | PORT_ISC_FORCE_ENABLE_gc;
	PORTA_PIN4CTRL = PORT_OPC_PULLUP_gc | PORT_ISC_FORCE_ENABLE_gc;
	
	setting = (PORTA.IN & 0x1F) >> 1;
	selected_baudctrla = baudctrla[setting];
	selected_baudctrlb = baudctrlb[setting];
}

void init_clk()
{
	CCP = CCP_IOREG_gc;
	OSC_CTRL = OSC_RC32MEN_bm;
	
	while(!(OSC_STATUS & OSC_RC32MRDY_bm)){};
		
	CCP = CCP_IOREG_gc;
	CLK_CTRL = CLK_SCLKSEL_RC32M_gc;
	
	
}

void init_interrupts()
{
	PMIC_CTRL = PMIC_HILVLEN_bm | PMIC_MEDLVLEN_bm | PMIC_LOLVLEN_bm;
}

void init_usart()
{
	
	PORTD.DIRCLR = PIN2_bm;
	USARTD0_CTRLA = USART_DREIF_bm | USART_RXCINTLVL0_bm;
	USARTD0_CTRLB = USART_RXEN_bm | USART_TXEN_bm;
	USARTD0_CTRLC = USART_CMODE_ASYNCHRONOUS_gc | USART_PMODE_DISABLED_gc | USART_CHSIZE_8BIT_gc;

	//115.2 BAUD BSEL = 131, BSCALE = -3
//	USARTD0_BAUDCTRLA = 131;
//	USARTD0_BAUDCTRLB = -3 << 4;
	
	USARTD0_BAUDCTRLA = selected_baudctrla;// 131;
	USARTD0_BAUDCTRLB = selected_baudctrlb << 4; // -3 << 4;	

	//USARTD0_BAUDCTRLA = baudctrla[9];
	//USARTD0_BAUDCTRLB = baudctrlb[9] << 4;
	
}

//"Timeout" timer
void init_timer()
{
	
	TCC4.CTRLB = 0; //Normal mode, disable circ buffer, WFG: normal PER
	TCC4.CTRLC = 0; //no polarity change or comp output val
	TCC4.CTRLD = 0; //no event action
	TCC4.CTRLE = 0; //doesn't matter
	TCC4.CTRLF = 0; //doesn't matter
	TCC4.INTCTRLA = TC4_OVFINTLVL_gm;
	TCC4.CTRLA = TC4_CLKSEL0_bm; //Prescaler: Clk, start the timer
	//Stop the "timeout" timer until we start receiving
	TCC4.CTRLGSET = TC4_STOP_bm;
}


void ws2812drv_init(void)
{
	// Setup EDMA channel 0(+1)
	EDMA.CTRL = EDMA_ENABLE_bm | EDMA_CHMODE_STD02_gc | EDMA_DBUFMODE_DISABLE_gc | EDMA_PRIMODE_CH0123_gc;
	EDMA.CH0.CTRLB = EDMA_CH_TRNINTLVL_OFF_gc;
	EDMA.CH0.ADDRCTRL = EDMA_CH_RELOAD_NONE_gc | EDMA_CH_DIR_INC_gc;
	EDMA.CH0.DESTADDRCTRL = EDMA_CH_RELOAD_NONE_gc | EDMA_CH_DESTDIR_FIXED_gc;
	EDMA.CH0.DESTADDR = (uint16_t)&USARTC0.DATA;
	EDMA.CH0.TRIGSRC = EDMA_CH_TRIGSRC_USARTC0_DRE_gc;

	// Setup port pins for TxD, XCK and LUT0OUT
	PORTC.PIN0CTRL = PORT_OPC_TOTEM_gc;                         // LUT0OUT (data to WS2812)
	PORTC.PIN1CTRL = PORT_OPC_TOTEM_gc | PORT_ISC_RISING_gc;    // XCK
	PORTC.PIN3CTRL = PORT_OPC_TOTEM_gc | PORT_ISC_LEVEL_gc;     // TxD
	PORTC.DIRSET = PIN0_bm | PIN1_bm | PIN3_bm;

	// Setup Event channel 0 to TxD (async)
	EVSYS.CH0MUX = EVSYS_CHMUX_PORTC_PIN3_gc;
	EVSYS.CH0CTRL = EVSYS_DIGFILT_1SAMPLE_gc;
	// Setup Event channel 6 to XCK rising edge
	EVSYS.CH6MUX = EVSYS_CHMUX_PORTC_PIN1_gc;
	EVSYS.CH6CTRL = EVSYS_DIGFILT_1SAMPLE_gc;

	// Setup USART in master SPI mode 1, MSB first
	USARTC0.BAUDCTRLA = 19;                                     // 800.000 baud (1250 ns @ 32 MHz)
	USARTC0.BAUDCTRLB = 0;
	USARTC0.CTRLA = USART_RXCINTLVL_OFF_gc | USART_TXCINTLVL_OFF_gc; 
	USARTC0.CTRLC = USART_CMODE_MSPI_gc | (1 << 1);             // UDORD=0 UCPHA=1
	USARTC0.CTRLD = USART_DECTYPE_DATA_gc | USART_LUTACT_OFF_gc | USART_PECACT_OFF_gc;
	USARTC0.CTRLB = USART_TXEN_bm;

	// Setup XCL BTC0 timer to 1shot pwm generation
	XCL.CTRLE = XCL_CMDSEL_NONE_gc | XCL_TCSEL_BTC0_gc | XCL_CLKSEL_DIV1_gc;
	XCL.CTRLF = XCL_CMDEN_DISABLE_gc | 0x03;                    // 0x03 : One-shot PWM (missing in iox32e5.h)
	XCL.CTRLG = XCL_EVACTEN_bm | (0x03<<3) | XCL_EVSRC_EVCH6_gc; // 0x03<<3 : EVACT0=RESTART (missing in iox32e5.h)
	XCL.PERCAPTL = 22;                                          // Output high time if data is 1 (from RESTART to falling edge of one-shot)
	XCL.CMPL = 13;                                              // Output high time if data is 0 (from RESTART to rising edge of one-shot)

	// Setup XCL LUT
	XCL.CTRLA = XCL_LUT0OUTEN_PIN0_gc | XCL_PORTSEL_PC_gc | XCL_LUTCONF_MUX_gc;  // Setup glue logic for MUX
	XCL.CTRLB = 0x50;                                           // IN3SEL=XCL, IN2SEL=XCL, IN1SEL=EVSYS, IN0SEL=EVSYS (missing in iox32e5.h)
	XCL.CTRLC = XCL_EVASYSEL0_bm | XCL_DLY0CONF_DISABLE_gc;      // Async inputs, no delay
	XCL.CTRLD = 0xA0;                                           // LUT truthtables (only LUT1 is used)

}

void ws2812drv_start_transfer()
{
	EDMA.CH0.ADDR = (uint16_t)bytes;
	EDMA.CH0.TRFCNT = byte_count;
	EDMA.CH0.CTRLA = EDMA_CH_ENABLE_bm | EDMA_CH_SINGLE_bm; // Start DMA transfer to LEDs
}

void transmit_ws2812()
{
	//DEBUG START
	while(!(USARTD0_STATUS & USART_DREIF_bm)){};
	USARTD0_DATA = 0x55;
			
	while(!(USARTD0_STATUS & USART_DREIF_bm)){};
	USARTD0_DATA = hi_byte;
	while(!(USARTD0_STATUS & USART_DREIF_bm)){};
	USARTD0_DATA = lo_byte & 0xFF;

			while(!(USARTD0_STATUS & USART_DREIF_bm)){};
			USARTD0_DATA = 0x55;
			while(!(USARTD0_STATUS & USART_DREIF_bm)){};
			USARTD0_DATA = setting;
			while(!(USARTD0_STATUS & USART_DREIF_bm)){};
			USARTD0_DATA = 0x55;	
	//DEBUG END
	ws2812drv_start_transfer();
	rcvState = HiByte;

}

//swaps the data for r and g to match the g-r-b ws2812b data pattern
void swap_r_g()
{
	uint8_t temp;
	for(uint16_t i = 0; i < byte_count; i+=3)
	{
		temp = bytes[i];
		bytes[i] = bytes[i+1];
		bytes[i+1] = temp;
	}
}

int main(void)
{
	cli();
	
	
	
	PORTD.DIRSET = PIN3_bm;
	
	//DEBUG START
	PORTA.DIRSET = PIN0_bm;
	PORTA.OUTSET = PIN0_bm;
	//DEBUG END
	
	init_clk();
	init_interrupts();
	read_baud_switches();
	init_usart();
	ws2812drv_init();
	init_timer();
	sei();

    /* Replace with your application code */
    while (1) 
    {
		if (rcvState == Trans)
		{
			swap_r_g();
			
			transmit_ws2812();
		}
    }
}




ISR(USARTD0_RXC_vect)
{
	uint8_t data = USARTD0.DATA;
	
	switch (rcvState)
	{
		case HiByte:
		
			hi_byte = data;
			byte_count = (uint16_t)(hi_byte << 8);
			rcvState = LoByte;
			//Start the "timeout" timer because we are done receiving
			TCC4.CTRLGCLR = TC4_STOP_bm;
			
			break;
		case LoByte:
			lo_byte = data;
			byte_count += lo_byte;
			byte_count *= 3;
			rcv_index = 0;
			rcvState = Data;
			//reset "timeout" timer so it doesn't time out while receiving 
			TCC4.CTRLGSET = TC4_CMD1_bm;
			
		
			break;
		case Data:
			

		
				PORTA.OUTTGL = PIN0_bm;
			bytes[rcv_index] = data;
			rcv_index++;
			//reset "timeout" timer so it doesn't time out while receiving 
			TCC4.CTRLGSET = TC4_CMD1_bm;
			if (rcv_index == byte_count)
			{
				rcvState = Trans;
				//Stop the "timeout" timer because we are done receiving
				TCC4.CTRLGSET = TC4_STOP_bm;
			}
			
			
			
			break;
		case Timeout:
			/*
				If we are in this state, we can assume that we did not receive
				data for a short period and that it might be invalid or not
				be long enough. If this is the case, we will wait till no more
				data is received for a short period before going back to the
				startup state.
			*/
			//reset "timeout" timer to wait till we stop receiving
			TCC4.CTRLGSET = TC4_CMD1_bm;		
			break;
		case Trans:
		default:
			break;
		
	}
	

	USARTD0_STATUS = USART_RXCIF_bm;
}

ISR(TCC4_OVF_vect)
{
	if (rcvState == LoByte || rcvState == Data)
	{
		//We reached here while receiving so assume receive glitch
		rcvState = Timeout;		
	}
	else if (rcvState == Timeout)
	{
		//We reached here during timeout so we can go back to normal
		rcvState = HiByte;
	}
	
	TCC4.INTFLAGS = TC4_OVFIF_bm;
}