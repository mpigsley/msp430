/*
 *  File: msp430_robot.ino
 *  Author: Mitchel S Pigsley
 *  Created: 7/8/14
 *  Last Updated: 11/24/14
 *  Description: MSP430 script to communicate with sensors
 *          and raspberry pi
 *  Used MSP430 Pins:
 *      1.0 - On-Board LED 1
 *      1.1 - UART Receive
 *      1.2 - UART Transmit
 *      1.4 - Multiplexer S0
 *      1.5 - Multiplexer S1
 *      1.6 - I2C SCL
 *      1.7 - I2C SDA
 *      2.2 - Left Servo
 *      2.4 - Right Servo
 *  Programming Pins (from square pin)
 *      VCC - GND - TEST - RESET
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

#define SERVO_STOP 's'
#define SERVO_LEFT 'l'
#define SERVO_RIGHT 'r'
#define SERVO_FORWARD 'f'
#define SERVO_OFF 'o'

#define LEFT_SERVO BIT2
#define RIGHT_SERVO BIT4
#define SERVO_DIR_STOP 1500
#define SERVO_DIR_RIGHT 1000
#define SERVO_DIR_LEFT 2000

#define I2C_SCL BIT6
#define I2C_SDA BIT7
#define I2C_ADDR 1
#define I2C_DATA 2
#define I2C_STOP 3
#define I2C_BUF_SIZE 130

#define UART_RX BIT1
#define UART_TX BIT2
#define UART_BUFFER_SIZE 64

#define S0 BIT4
#define S1 BIT5
#define SENS_LEFT 0
#define SENS_FRONT 1
#define SENS_RIGHT 2
#define SENS_LEFT_CUTOFF 4000
#define SENS_FRONT_CUTOFF 4000
#define SENS_RIGHT_CUTOFF 4000

#define VCNL4000_ADDRESS 0x13  // 0x26 write, 0x27 read
#define COMMAND_0 0x80
#define IR_CURRENT 0x83
#define PROXIMITY_RESULT_MSB 0x87
#define PROXIMITY_RESULT_LSB 0x88
#define PROXIMITY_FREQ 0x89
#define PROXIMITY_MOD 0x8A

/****************************************
    Global Vars
****************************************/
volatile unsigned time_1ms = 0;
volatile unsigned time_100ms = 0;
volatile unsigned time_1s = 0;
volatile unsigned int counter_base;
volatile unsigned int counter_100ms;

volatile unsigned left_sensor_is_initialized;
volatile unsigned left_sensor_last_val;
volatile unsigned front_sensor_is_initialized;
volatile unsigned front_sensor_last_val;
volatile unsigned right_sensor_is_initialized;
volatile unsigned right_sensor_last_val;
unsigned int current_sensor;
unsigned int sensor_hit;

volatile signed char byte_ctr;
volatile unsigned char *transmit_field;
volatile unsigned char receive_field;
volatile unsigned is_i2c_busy;
uint8_t buf[I2C_BUF_SIZE];
uint8_t buf_ptr;
volatile char servo_dir;

unsigned int incoming = FALSE;
char rx_buffer[UART_BUFFER_SIZE];
volatile int rx_head;
volatile int rx_tail;
char tx_buffer[UART_BUFFER_SIZE];
volatile int tx_head;
volatile int tx_tail;

/****************************************
    Ring Buffer
****************************************/
bool is_rx_ring_empty() {
  return rx_head == rx_tail;
}

bool is_tx_ring_empty() {
  return tx_head == tx_tail;
}

void tx_ring_push(char ch) {
  int i = (tx_head + 1) % UART_BUFFER_SIZE;
  if (i != tx_tail) {
    tx_buffer[tx_head] = ch;
    tx_head = i;
  }
}

bool rx_ring_pop(char *ch) {
  if (is_rx_ring_empty()) {
    return FALSE;
  }

  *ch = rx_buffer[rx_tail];
  rx_tail = (rx_tail + 1) % UART_BUFFER_SIZE;
  return TRUE;
}

bool tx_ring_pop(char *ch) {
  if (is_tx_ring_empty()) {
    return FALSE;
  }

  *ch = tx_buffer[tx_tail];
  tx_tail = (tx_tail + 1) % UART_BUFFER_SIZE;
  return TRUE;
}

/****************************************
    Servo
****************************************/
void servo_init() {
  P2SEL |= LEFT_SERVO + RIGHT_SERVO;
  P2DIR |= LEFT_SERVO + RIGHT_SERVO;

  TA1CCTL1 = OUTMOD_7; 		             // reset/set (1)
  TA1CCTL2 = OUTMOD_7; 		             // reset/set (1)
  TA1CTL = TASSEL_2 + MC_1; 	             // SMCLK, up mode

  TA1CCR0 = 20000;  		             // Period
  
  // Initialize Servo State
  servo_dir = SERVO_STOP;
}

void set_direction(char dir) {
  if (dir == SERVO_LEFT) {
    TA1CCR1 = SERVO_DIR_STOP;                  // (2.2) Duty Cycle
    TA1CCR2 = SERVO_DIR_RIGHT;                 // (2.4) Duty Cycle
  } else if (dir == SERVO_RIGHT) {
    TA1CCR1 = SERVO_DIR_LEFT;                  // (2.2) Duty Cycle
    TA1CCR2 = SERVO_DIR_STOP;                  // (2.4) Duty Cycle
  } else if (dir == SERVO_FORWARD) {
    TA1CCR1 = SERVO_DIR_LEFT;                  // (2.2) Duty Cycle
    TA1CCR2 = SERVO_DIR_RIGHT;                 // (2.4) Duty Cycle
  } else {
    TA1CCR1 = SERVO_DIR_STOP;                  // (2.2) Duty Cycle
    TA1CCR2 = SERVO_DIR_STOP;                  // (2.4) Duty Cycle
  }
}

/****************************************
    VCNL4000
****************************************/ 
void vcnl4000_init() {
  current_sensor = 0;
  sensor_hit = FALSE;
  
  i2c_write(IR_CURRENT, 20);                   // Set IR current to 200mA
  i2c_write(PROXIMITY_FREQ, 2);                // 781.25 kHz
  i2c_write(PROXIMITY_MOD, 0x81);              // 129, recommended by Vishay
  left_sensor_is_initialized = TRUE;
  next_sensor();
  
  i2c_write(IR_CURRENT, 20);                   // Set IR current to 200mA
  i2c_write(PROXIMITY_FREQ, 2);                // 781.25 kHz
  i2c_write(PROXIMITY_MOD, 0x81);              // 129, recommended by Vishay
  front_sensor_is_initialized = TRUE;
  next_sensor();
  
  i2c_write(IR_CURRENT, 20);                   // Set IR current to 200mA
  i2c_write(PROXIMITY_FREQ, 2);                // 781.25 kHz
  i2c_write(PROXIMITY_MOD, 0x81);              // 129, recommended by Vishay
  right_sensor_is_initialized = TRUE;
  next_sensor();
}

unsigned int read_proximity() {
  // Command sensor to perform a proximity measure
  unsigned temp = i2c_read(COMMAND_0);
  i2c_write(COMMAND_0, temp | 0x08);
  
  // Wait for the proximity data ready bit to be set
  while(!(i2c_read(COMMAND_0)&0x20));
  
  // Return the data
  unsigned int data = i2c_read(PROXIMITY_RESULT_MSB) << 8;
  data |= i2c_read(PROXIMITY_RESULT_LSB);
  return data;
}

void next_sensor() {
  current_sensor = (current_sensor + 1) % 3;
  switch (current_sensor) {
    case SENS_LEFT:
      P1OUT &= ~(S0 + S1);
      break;
    case SENS_FRONT:
      P1OUT |= S0;
      P1OUT &= ~S1;
      break;
    case SENS_RIGHT:
      P1OUT &= ~S0;
      P1OUT |= S1;
      break;
  }
}

void poll_sensors() {
  switch (current_sensor) {
    case SENS_LEFT:
      if (left_sensor_is_initialized) left_sensor_last_val = read_proximity();
      break;
    case SENS_FRONT:
      if (front_sensor_is_initialized) front_sensor_last_val = read_proximity();
      break;
    case SENS_RIGHT:
      if (right_sensor_is_initialized) right_sensor_last_val = read_proximity();
      break;
  }
  next_sensor();
  
  int left = left_sensor_last_val > SENS_LEFT_CUTOFF ? TRUE : FALSE;
  int front = front_sensor_last_val > SENS_FRONT_CUTOFF ? TRUE : FALSE;
  int right = right_sensor_last_val > SENS_RIGHT_CUTOFF ? TRUE : FALSE;
  
  if (left || front || right) {
    if (!sensor_hit) {
      P1OUT |= BIT0;
      char out[3];
      sprintf(out, "%c%c%c", left ? '1' : '0', front ? '1' : '0', right ? '1' : '0');
      uart_puts(out);
      sensor_hit = TRUE;
    }
  } else if (sensor_hit) {
    uart_puts("000");
    sensor_hit = FALSE;
  } else {
    P1OUT &= ~BIT0;
  }
}

/****************************************
    I2C
****************************************/
void i2c_init(void) {
  P1SEL  |= I2C_SCL + I2C_SDA;
  P1SEL2 |= I2C_SCL + I2C_SDA;
  
  UCB0CTL1 |= UCSWRST;                      // Enable SW reset
  UCB0CTL0 = UCMST + UCMODE_3 + UCSYNC;     // I2C Master, synchronous mode
  UCB0CTL1 = UCSSEL_2 + UCSWRST;            // Use SMCLK, keep SW reset
  UCB0BR0 = 12;                             // fSCL = SMCLK/12 = ~100kHz
  UCB0BR1 = 0;
  UCB0I2CSA = VCNL4000_ADDRESS;
  UCB0CTL1 &= ~UCSWRST;                     // Clear SW reset, resume operation
  IE2 |= UCB0TXIE + UCB0RXIE;               // Enable RX and TX interrupt
  
  is_i2c_busy = 0;
}

void i2c_write(unsigned address, unsigned data) {
  buf_ptr = 0;
  i2c_stuff_byte(address);
  i2c_stuff_byte(data);
  i2c_transmit();
}

unsigned i2c_read(unsigned address) {
  buf_ptr = 0;
  i2c_stuff_byte(address);
  i2c_transmit();
  i2c_receive();
  return receive_field;
}

void i2c_receive(void) {
  while (is_i2c_busy);
  UCB0CTL1 &= ~UCTR ;                       // Clear UCTR
  while (UCB0CTL1 & UCTXSTP);   
  UCB0CTL1 |= UCTXSTT;                      // I2C start condition
  while (UCB0CTL1 & UCTXSTT);               // Start condition sent?
  UCB0CTL1 |= UCTXSTP;                      // I2C stop condition
  while (UCB0CTL1 & UCTXSTP);   
}

void i2c_transmit(void) {
  while (is_i2c_busy);
  transmit_field = buf;
  byte_ctr = buf_ptr;
  while (UCB0CTL1 & UCTXSTP);               // Ensure stop condition got sent
  UCB0CTL1 |= UCTR + UCTXSTT;               // I2C TX, start condition
  is_i2c_busy = 1;
}

void i2c_stuff_byte(unsigned data) {
  if(buf_ptr < I2C_BUF_SIZE) {
    buf[buf_ptr++] = data;
  }
}

void i2c_read_start(void) {
  while (UCB0CTL1 & UCTXSTP);               // Ensure stop condition got sent
  UCB0CTL1 &= ~UCTR ;                       // Clear UCTR
  UCB0CTL1 |= UCTXSTT;                      // I2C start condition
}

/****************************************
    UART
****************************************/
void uart_init(void) {
  P1SEL |= UART_RX + UART_TX;                       
  P1SEL2 |= UART_RX + UART_TX;
  
  UCA0CTL1 = UCSWRST;
  UCA0CTL1 |= UCSSEL_2; 		    // SMCLK
  UCA0CTL0 = 0;
  UCA0ABCTL = 0;
  UCA0BR0 = 8; 			            // 1MHz 115200
  UCA0BR1 = 0; 			            // 1MHz 115200
  UCA0MCTL = UCBRS2 + UCBRS0; 	            // Modulation
  UCA0CTL1 &= ~UCSWRST; 		    // Initialize USCI state machine
  UC0IE |= UCA0TXIE + UCA0RXIE;
  IE2 |= UCA0TXIE + UCA0RXIE; 		    // Enable USCI_A0 RX interrupt
}

void uart_puts(const char *str) {
  while (*str)
    tx_ring_push(*str++);
  tx_ring_push('\r');
  UC0IE |= UCA0TXIE;
}

bool uart_getc(char *ch) {
  return rx_ring_pop(ch);
}

/****************************************
    Timer
****************************************/
void timer_init() {
  TA0CCTL0 = CCIE;                          // Enable Timer A0 interrupts
  TA0CCR0 = 1000;                           // Count limit
  TA0CTL = TASSEL_2 + MC_1;                 // Timer A0 with SMCLK, count UP
}

void main_100ms_routine() {
  poll_sensors();
}

void main_1s_routine() {
  //P1OUT ^= BIT0;
}

/****************************************
    Main
****************************************/
int main(void) {
  WDTCTL  = WDTPW + WDTHOLD; 	            // Stop WDT
  BCSCTL1 = CALBC1_1MHZ; 		    // Set DCO
  DCOCTL  = CALDCO_1MHZ;

  P1DIR = BIT0 + S0 + S1;                   // P1.0 is the red LED	
  P1OUT = ~(BIT0 + S0 + S1);                // Turn all off

  // Initialize Modules
  servo_init();
  i2c_init();
  timer_init();
  uart_init();
  
  _EINT();                                  // Register Global Interrupts
  
  vcnl4000_init();

  while(1) {
    if (time_1ms == TRUE) {
      time_1ms = FALSE;
    } else if (time_100ms == TRUE) {
      main_100ms_routine();
      time_100ms = FALSE;
    } else if (time_1s == TRUE) {
      main_1s_routine();
      time_1s = FALSE;
    }
  }
}

/****************************************
    Interrupts
****************************************/
#pragma vector=USCIAB0RX_VECTOR 
__interrupt void USCI0RX_ISR(void) {
  if (IFG2 & UCA0RXIFG) {
    set_direction(UCA0RXBUF);
  }
}

#pragma vector=USCIAB0TX_VECTOR 
__interrupt void USCI0TX_ISR(void) {
  if (IFG2 & UCB0RXIFG) {
    receive_field = UCB0RXBUF;
  } else if (IFG2 & UCB0TXIFG) {
    if (byte_ctr == 0){
      UCB0CTL1 |= UCTXSTP;                    // I2C stop condition
      IFG2 &= ~UCB0TXIFG;                     // Clear USCI_B0 TX int flag
      is_i2c_busy = 0;
    } else {
      UCB0TXBUF = *transmit_field;
      transmit_field++;
      byte_ctr--;
    }
  } else if (IFG2 & UCA0TXIFG) {
    if (is_tx_ring_empty()) {
      UC0IE &= ~UCA0TXIE;
    } else {
      char ch;
      if (tx_ring_pop(&ch)) {
        UCA0TXBUF = ch;
      }
    }
  }
}

#pragma vector=TIMER0_A0_VECTOR
__interrupt void Timer0_A0 (void) {
  time_1ms = TRUE;
  counter_base++;
  if(counter_base >= 100) {
    counter_base = 0;
    time_100ms = TRUE;
    counter_100ms++;
    if(counter_100ms >= 10) {
      counter_100ms = 0;
      time_1s = TRUE;
    }
  }
}
