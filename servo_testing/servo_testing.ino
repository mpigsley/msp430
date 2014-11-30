#include <msp430g2553.h>

#define TRUE 1
#define FALSE 0
#define LEFT_SERVO BIT2
#define RIGHT_SERVO BIT4
#define SERVO_DIR_STOP 1500
#define SERVO_DIR_RIGHT 1000
#define SERVO_DIR_LEFT 2000

void main_100ms_routine();
void main_1s_routine();

typedef struct time_flags {
  unsigned time_1ms :1;
  unsigned time_100ms :1;
  unsigned time_1s :1;
} time_flags;

volatile struct time_flags sys_time;
unsigned int counter_base;
unsigned int counter_100ms;

void servo_init() {
  P2SEL |= LEFT_SERVO + RIGHT_SERVO;
  P2DIR |= LEFT_SERVO + RIGHT_SERVO;

  TA1CCTL1 = OUTMOD_7; 		             // reset/set (1)
  TA1CCTL2 = OUTMOD_7; 		             // reset/set (1)
  TA1CTL = TASSEL_2 + MC_1; 	             // SMCLK, up mode

  TA1CCR0 = 20000;  		             // Period
}

void bot_left() {
    TA1CCR1 = SERVO_DIR_STOP;                  // (2.2) Duty Cycle
    TA1CCR2 = SERVO_DIR_RIGHT;                 // (2.4) Duty Cycle
}

void bot_right() {
    TA1CCR1 = SERVO_DIR_LEFT;                  // (2.2) Duty Cycle
    TA1CCR2 = SERVO_DIR_STOP;                  // (2.4) Duty Cycle
}

void bot_forward() {
    TA1CCR1 = SERVO_DIR_LEFT;                  // (2.2) Duty Cycle
    TA1CCR2 = SERVO_DIR_RIGHT;                 // (2.4) Duty Cycle
}

void bot_stop() {
    TA1CCR1 = SERVO_DIR_STOP;                  // (2.2) Duty Cycle
    TA1CCR2 = SERVO_DIR_STOP;                  // (2.4) Duty Cycle
}

void main_100ms_routine() {
  P1OUT ^= BIT0;                       // Toggle red LED
  bot_forward();
}

void main_1s_routine() {
  bot_forward();
}

int main(void) {
  WDTCTL  = WDTPW + WDTHOLD; 	       // Stop WDT
  BCSCTL1 = CALBC1_1MHZ; 	       // Set DCO
  DCOCTL  = CALDCO_1MHZ;
  
  P1DIR |= BIT0 + BIT6; 	       // P1.0 and P1.6 are the red+green LEDs	
  P1OUT &= ~(BIT0 + BIT6); 	       // Both LEDs off
  
  TA0CCTL0 = CCIE;                     // Enable Timer A0 interrupts
  TA0CCR0 = 1000;                        // Count limit
  TA0CTL = TASSEL_2 + MC_1;            // Timer A0 with SMCLK, count UP
  
  servo_init();
  
  _BIS_SR(GIE);                        // Enable interrupts

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

#pragma vector=TIMER0_A0_VECTOR        // Timer0 A0 interrupt service routine
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
