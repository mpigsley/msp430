/*
 *  Mitchel Pigsley
 *  Date: 4/23/14
 *  Description: Initializes the rx and tx pins for serial transmission. 
 *    If an 'a' is written to the serial port, then LED is turned on.
 *    This code also sends three numbers in a row after initializing.
 */

#include "msp430g2553.h"
#define TXLED BIT0
#define RXLED BIT6
#define TXD BIT2
#define RXD BIT1

void uart_print_str(const void *str);
void uart_init();

const char string[] = { "Hello World\r\n" };
unsigned int i;

int main(void) {
  WDTCTL = WDTPW + WDTHOLD; // Stop watchdog timer
  DCOCTL = 0; // Select lowest DCOx and MODx settings<
  BCSCTL1 = CALBC1_1MHZ; // Set DCO
  DCOCTL = CALDCO_1MHZ;
  uart_init();
  
  __bis_SR_register(CPUOFF + GIE); // Enter LPM0 w/ int until Byte RXed
  
  while (1) {
  }
}

void uart_init() {
  P2DIR = 0xFF; // All P2.x outputs<
  P2OUT = 0x00; // All P2.x reset
  P1SEL |= RXD + TXD ; // P1.1 = RXD, P1.2=TXD
  P1SEL2 |= RXD + TXD ; // P1.1 = RXD, P1.2=TXD
  P1DIR |= RXLED + TXLED;
  P1OUT = 0x00;
  
  UCA0CTL1 |= UCSSEL_2; // SMCLK
  UCA0BR0 = 0x68; // 1MHz 9600
  UCA0BR1 = 0x00; // 1MHz 9600
  UCA0MCTL = UCBRS2 + UCBRS0; // Modulation UCBRSx = 5
  UCA0CTL1 &= ~UCSWRST; // **Initialize USCI state machine**
  IE2 |= UCA0RXIE + UCA0TXIE; // Enable USCI_A0 RX interrupt
}

#pragma vector=USCIAB0TX_VECTOR
__interrupt void USCI0TX_ISR(void) {
  P1OUT |= TXLED; 
  UCA0TXBUF = string[i++]; // TX next character 
  if (i == sizeof string - 1) // TX over? 
    IE2 &= ~UCA0TXIE; // Disable USCI_A0 TX interrupt 
  P1OUT &= ~TXLED;
} 
   
#pragma vector=USCIAB0RX_VECTOR 
__interrupt void USCI0RX_ISR(void) { 
  P1OUT |= RXLED; 
  if (UCA0RXBUF == 'a') { 
    i = 0; 
    IE2 |= UCA0TXIE; // Enable USCI_A0 TX interrupt 
    UCA0TXBUF = string[i++]; 
  } 
  //P1OUT &= ~RXLED;
}
