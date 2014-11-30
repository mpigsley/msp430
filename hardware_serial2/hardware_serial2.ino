
#include "msp430g2553.h"

#define RXD BIT1
#define TXD BIT2

void uart_rx_isr(unsigned char c);
void uart_set_rx_isr_ptr(void (*isr_ptr)(unsigned char c));
void (*uart_rx_isr_ptr)(unsigned char c);
unsigned char uart_getc();
void uart_putc(unsigned char c);
void uart_puts(const char *str);

int main(void) {
  WDTCTL  = WDTPW + WDTHOLD; 	// Stop WDT
  BCSCTL1 = CALBC1_1MHZ;      // Set DCO
  DCOCTL  = CALDCO_1MHZ;

  P1DIR  = BIT0 + BIT6; 		// P1.0 and P1.6 are the red+green LEDs	
  P1OUT  = BIT0 + BIT6; 		// All LEDs off

  uart_init();

  // register ISR called when data was received
  uart_set_rx_isr_ptr(uart_rx_isr);

  __bis_SR_register(GIE);

  uart_puts((char *)"\n\r***************\n\r");
  uart_puts((char *)"MSP430 harduart\n\r");
  uart_puts((char *)"***************\n\r\n\r");

  uart_puts((char *)"PRESS any key to start echo example ... ");
  
  P1OUT = ~BIT6;
  //unsigned char c = uart_getc();

  //uart_putc(c);
  uart_puts((char *)"\n\rOK\n\r");

  volatile unsigned long i;

  while(1) {
    P1OUT ^= BIT6; 			// Toggle P1.6 output (green LED) using exclusive-OR
    i = 50000;             	// Delay
    
    uart_puts("HO\r\n");
    do (i--);				// busy waiting (bad)
    while (i != 0);
  } 
}

void uart_init(void) {
  uart_set_rx_isr_ptr(0L);

  P1SEL  = RXD + TXD;                       
  P1SEL2 = RXD + TXD;                       
  UCA0CTL1 |= UCSSEL_2;                     // SMCLK
  UCA0BR0 = 104;                            // 1MHz 9600
  UCA0BR1 = 0;                              // 1MHz 9600
  UCA0MCTL = UCBRS0;                        // Modulation UCBRSx = 1
  UCA0CTL1 &= ~UCSWRST;                     // Initialize USCI state machine
  IE2 |= UCA0RXIE;                          // Enable USCI_A0 RX interrupt
}

void uart_rx_isr(unsigned char c) {
  uart_putc(c);
  P1OUT ^= BIT0;		// toggle P1.0 (red led)
}

void uart_set_rx_isr_ptr(void (*isr_ptr)(unsigned char c)) {
  uart_rx_isr_ptr = isr_ptr;	
}

unsigned char uart_getc() {
  while (!(IFG2&UCA0RXIFG));                // USCI_A0 RX buffer ready?
  return UCA0RXBUF;
}

void uart_putc(unsigned char c) {
  while (!(IFG2&UCA0TXIFG));              // USCI_A0 TX buffer ready?
  UCA0TXBUF = c;                    		// TX
}

void uart_puts(const char *str) {
  while(*str) uart_putc(*str++);
}
   
#pragma vector=USCIAB0RX_VECTOR 
__interrupt void USCI0RX_ISR(void) { 
  if(uart_rx_isr_ptr != 0L) {
    (uart_rx_isr_ptr)(UCA0RXBUF);
  }
}
