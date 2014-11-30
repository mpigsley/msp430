#include "msp430g2553.h"

#define TXLED BIT0
#define RXLED BIT6
#define TXD BIT2
#define RXD BIT1

const char string[] = {"Hello World\r\n"};
unsigned int i;

int main (void) {
  WDTCTL = WDTPW + WDTHOLD; // Stop WDT
  DCOCTL = 0; // SELECT LOWEST DCOX AND MODX SETTINGS
  BCSCTL1 = CALBC1_1MHZ; // Set DCO
  DCOCTL = CALDCO_1MHZ;
  
  P1SEL |= RXD + TXD; // P1.1 = RXD, P1.2 = TXD
  P1SEL2 |= RXD + TXD; // P1.1 = RXD, P1.2 = TXD
  P1DIR |= RXLED + TXLED + TXD;
  P1OUT = TXD;
  
  UCA0CTL1 |= UCSSEL_2; // SMCLK
  UCA0BR0 = 0x08; // 1Mhz 115200
  UCA0BR1 = 0x08; // 1Mhz 115200
  UCA0MCTL = UCBRS2 + UCBRS0; // Modulation UCBR2x = 5
  UCA0CTL1 &= ~UCSWRST; // ** Initialize USCI state machine**
  UC0IE |= UCA0RXIE; // Enable USCI_A0 RX inturrupt
  
  __bis_SR_register(CPUOFF + GIE); // Enter LPM0 w/ int until Bte RXed
  
  while (1) {}
}

/*
 *  Receive Inturrupt
 */
#pragma vector=USCIAB0RX_VECTOR
__interrupt void USCI0RX_ISR(void) {
  P1OUT |= RXLED;
  if (UCA0RXBUF == 'a') { // 'a' received?
    i = 0;
    UC0IE |= UCA0TXIE; // Enable USCI_A0 TX inturrupt
    UCA0TXBUF = string[i++];
  }
}
/*
 *  Transmit Inturrupt
 */
#pragma vector=USCIAB0TX_VECTOR
__interrupt void USCI0TX_ISR(void) {
  P1OUT |= TXLED;
  UCA0TXBUF = string[i++]; // TX next character
  if (i == sizeof string - 1) // TX over?
    UC0IE &= ~UCA0TXIE; // Disable USCI_A0 inturrupt
  P1OUT &= ~TXLED;
}
