/*
 *  File: msp430_robot.c
 *  Author: Mitchel S Pigsley
 *  Created: 7/8/14
 *  Last Updated: 8/4/14
 *  Description: MSP430 script to communicate with sensors
 *          and raspberry pi
 *  Used MSP430 Pins:
 *      1.0 - On-Board LED 1
 *      1.1 - UART Receive
 *      1.2 - UART Transmit
 *      1.6 - I2C SCL
 *	1.7 - I2C SDA
 *      2.2 - Left Servo
 *      2.4 - Right Servo
 *
 */

/****************************************
    Includes
****************************************/
#include "msp430g2553.h"

/****************************************
    Defines
****************************************/
#define TRUE 1
#define FALSE 0

#define LEFT_SERVO BIT2
#define RIGHT_SERVO BIT4
#define SERVO_STOP 1500
#define SERVO_RIGHT 1000
#define SERVO_LEFT 2000

#define UART_RX BIT1
#define UART_TX BIT2

/****************************************
    Structs
****************************************/
typedef struct time_flags {
  unsigned time_1ms :1;
  unsigned time_100ms :1;
  unsigned time_1s :1;
} time_flags;

/****************************************
    Global Vars
****************************************/
volatile struct time_flags sys_time;
unsigned int counter_base;
unsigned int counter_100ms;
unsigned int counter_temp;

void uart_rx_isr(unsigned char c);
void uart_set_rx_isr_ptr(void (*isr_ptr)(unsigned char c));
void (*uart_rx_isr_ptr)(unsigned char c);

/****************************************
    Timer
****************************************/
void timer_init() {
  TA0CCTL0 = CCIE;                       // Enable Timer A0 interrupts
  TA0CCR0 = 1000;                        // Count limit
  TA0CTL = TASSEL_2 + MC_1;              // Timer A0 with SMCLK, count UP
  counter_temp = 0;
}

void main_100ms_routine() {
  P1OUT ^= BIT0;                         // Toggle red LED
}

void main_1s_routine() {
  P1OUT ^= BIT6;                         // Toggle green LED
  counter_temp = (counter_temp + 1) % 4;
  switch(counter_temp) {
    case 0:
      stop();
      break;
    case 1:
      left();
      break;
    case 2:
      right();
      break;
    case 3:
      forward();
      break;
  }
}

/****************************************
    Servo
****************************************/
void servo_init() {
  P2SEL |= LEFT_SERVO + RIGHT_SERVO;
  P2DIR |= LEFT_SERVO + RIGHT_SERVO;

  TA1CCTL1 = OUTMOD_7; 		         // reset/set (1)
  TA1CCTL2 = OUTMOD_7; 		         // reset/set (1)
  TA1CTL = TASSEL_2 + MC_1; 	         // SMCLK, up mode

  TA1CCR0 = 20000;  		         // Period
  stop();
}

void forward() {
  TA1CCR1 = SERVO_LEFT;                  // (2.2) Duty Cycle
  TA1CCR2 = SERVO_RIGHT;                 // (2.4) Duty Cycle
}

void stop() {
  TA1CCR1 = SERVO_STOP;                  // (2.2) Duty Cycle
  TA1CCR2 = SERVO_STOP;                  // (2.4) Duty Cycle
}

void left() {
  TA1CCR1 = SERVO_STOP;                  // (2.2) Duty Cycle
  TA1CCR2 = SERVO_RIGHT;                 // (2.4) Duty Cycle
}

void right() {
  TA1CCR1 = SERVO_LEFT;                  // (2.2) Duty Cycle
  TA1CCR2 = SERVO_STOP;                  // (2.4) Duty Cycle
}

/****************************************
    UART
****************************************/

void uart_init(void) {
  uart_set_rx_isr_ptr(0L);
  
  P1SEL  = UART_RX + UART_TX;                       
  P1SEL2 = UART_RX + UART_TX;                       
  UCA0CTL1 |= UCSSEL_2; 		 // SMCLK
  UCA0BR0 = 104; 			 // 1MHz 9600
  UCA0BR1 = 0; 			         // 1MHz 9600
  UCA0MCTL = UCBRS0; 		         // Modulation UCBRSx = 1
  UCA0CTL1 &= ~UCSWRST; 		 // Initialize USCI state machine
  IE2 |= UCA0RXIE; 		         // Enable USCI_A0 RX interrupt
  
  // register ISR called when data was received
  uart_set_rx_isr_ptr(uart_rx_isr);
}

void uart_rx_isr(unsigned char c) {
  uart_putc(c);
  P1OUT ^= BIT0;			 // Toggle Red
}

void uart_set_rx_isr_ptr(void (*isr_ptr)(unsigned char c)) {
  uart_rx_isr_ptr = isr_ptr;	
}

unsigned char uart_getc() {
  while (!(IFG2&UCA0RXIFG)); 	         // USCI_A0 RX buffer ready?
  return UCA0RXBUF;                      // RX
}

void uart_putc(unsigned char c) {
  while (!(IFG2&UCA0TXIFG)); 	         // USCI_A0 TX buffer ready?
  UCA0TXBUF = c;                         // TX
}

void uart_puts(const char *str) {
  while(*str) uart_putc(*str++);
}

/****************************************
    Main
****************************************/

int main(void) {
  WDTCTL  = WDTPW + WDTHOLD; 	         // Stop WDT
  BCSCTL1 = CALBC1_1MHZ; 		 // Set DCO
  DCOCTL  = CALDCO_1MHZ;

  P1DIR = BIT0 + BIT6;                   // P1.0 and P1.6 are the red+green LEDs	
  P1OUT &= ~(BIT0 + BIT6); 		 // Both LEDs off

  // Initialize Modules
  timer_init();
  servo_init();
  //uart_init();
  
  __bis_SR_register(GIE);                // Register Global Interrupts

  while(1) {
    if (sys_time.time_1ms == TRUE) {
      sys_time.time_1ms = FALSE;
    } else if (sys_time.time_100ms == TRUE) {
      main_100ms_routine();
      sys_time.time_100ms = FALSE;
    } else if (sys_time.time_1s == TRUE) {
      main_1s_routine();
      sys_time.time_1s = FALSE;
    }
  }
}

/****************************************
    Interrupts
****************************************/
#pragma vector=USCIAB0RX_VECTOR 
__interrupt void USCI0RX_ISR(void) { 
  if(uart_rx_isr_ptr != 0L) {
    (uart_rx_isr_ptr)(UCA0RXBUF);
  }
}

#pragma vector=TIMER0_A0_VECTOR
  __interrupt void Timer0_A0 (void) {
  sys_time.time_1ms = TRUE;
  counter_base++;
   
  if(counter_base >= 100) {
    counter_base = 0;
     
    sys_time.time_100ms = TRUE;
    counter_100ms++;
     
    if(counter_100ms >= 10) {
      counter_100ms = 0;
      sys_time.time_1s = TRUE;
    }
  }
}
