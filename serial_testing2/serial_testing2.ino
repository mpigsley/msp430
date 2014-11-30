/*
 *  File: msp430_robot.ino
 *  Author: Mitchel S Pigsley
 *  Created: 7/8/14
 *  Last Updated: 10/31/14
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

#define SERVO_OFF -1
#define SERVO_STOP 0
#define SERVO_LEFT 1
#define SERVO_RIGHT 2
#define SERVO_FORWARD 3

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

#define VCNL4000_ADDRESS 0x13
#define COMMAND_0 0x80
#define PRODUCT_ID 0x81
#define IR_CURRENT 0x83
#define PROXIMITY_RESULT_MSB 0x87
#define PROXIMITY_RESULT_LSB 0x88
#define PROXIMITY_FREQ 0x89
#define PROXIMITY_MOD 0x8A

#define VCNL4000_ADDRESS 0x13  // 0x26 write, 0x27 read

// VCNL4000 Register Map
#define COMMAND_0 0x80  // starts measurments, relays data ready info
#define PRODUCT_ID 0x81  // product ID/revision ID, should read 0x11
#define IR_CURRENT 0x83  // sets IR current in steps of 10mA 0-200mA
#define AMBIENT_PARAMETER 0x84  // Configures ambient light measures
#define AMBIENT_RESULT_MSB 0x85  // high byte of ambient light measure
#define AMBIENT_RESULT_LSB 0x86  // low byte of ambient light measure
#define PROXIMITY_RESULT_MSB 0x87  // High byte of proximity measure
#define PROXIMITY_RESULT_LSB 0x88  // low byte of proximity measure
#define PROXIMITY_FREQ 0x89  // Proximity IR test signal freq, 0-3
#define PROXIMITY_MOD 0x8A  // proximity modulator timing

/****************************************
    Structs
****************************************/
typedef struct time_flags {
  unsigned time_1ms :1;
  unsigned time_100ms :1;
  unsigned time_1s :1;
} time_flags;

typedef struct servo_control {
  unsigned is_moving :1;
  unsigned is_continuous :1;
  int dir;
  unsigned countdown;
} servo_control;

typedef struct vcnl4000_control {
  unsigned is_initialized :1;
  unsigned int last_val;
} vcnl4000_control;

typedef struct ring_buffer {
  unsigned char buffer[UART_BUFFER_SIZE];
  volatile unsigned char head;
  volatile unsigned char tail;
} ring_buffer;

/****************************************
    Global Vars
****************************************/
volatile struct time_flags sys_time;
volatile struct servo_control servo_state;
unsigned int counter_base;
unsigned int counter_100ms;

volatile struct vcnl4000_control left_sensor;
volatile struct vcnl4000_control front_sensor;
volatile struct vcnl4000_control right_sensor;
unsigned int current_sensor;

uint8_t buf[I2C_BUF_SIZE];
uint8_t buf_ptr;
volatile signed char byte_ctr;
volatile unsigned char *transmit_field;
volatile unsigned char receive_field;
volatile unsigned is_i2c_busy;

static ring_buffer rx_buffer;
static ring_buffer tx_buffer;
unsigned int incoming = FALSE;

/****************************************
    Ring Buffer
****************************************/
__inline unsigned int ring_len(ring_buffer *buffer) {
  short first_len = buffer->head - buffer->tail;
  if (first_len >= 0) {
    return first_len;
  } else {
    return -first_len - 1;
  }
}

__inline bool ring_empty(ring_buffer *buffer) {
  return buffer->head == buffer->tail;
}

__inline void ring_push(ring_buffer *buffer, unsigned char ch) {
  unsigned char i = (unsigned char) (buffer->head + 1);
  if (i == UART_BUFFER_SIZE) {
    i = 0;
  }

  if (i != buffer->tail) {
    buffer->buffer[buffer->head] = ch;
    buffer->head = i;
  }
}

__inline bool ring_peek(ring_buffer *buffer, unsigned char *ch) {
  if (ring_empty(buffer)) {
    return false;
  }

  *ch = buffer->buffer[buffer->tail];
  return true;
}

__inline bool ring_pop(ring_buffer *buffer, unsigned char *ch) {
  if (ring_empty(buffer)) {
    return false;
  }

  *ch = buffer->buffer[buffer->tail];
  unsigned char new_tail = buffer->tail + 1;
  if (new_tail == UART_BUFFER_SIZE) {
    new_tail = 0;
  }

  buffer->tail = new_tail;
  return true;
}

/****************************************
    Timer
****************************************/
void timer_init() {
  TA0CCTL0 = CCIE;                       // Enable Timer A0 interrupts
  TA0CCR0 = 1000;                        // Count limit
  TA0CTL = TASSEL_2 + MC_1;              // Timer A0 with SMCLK, count UP
}

void main_100ms_routine() {
  change_servo_status();
  poll_sensors();
  
  unsigned char ch;
  if (uart_getc(&ch)) {
    if (ch == 'f' || ch == 'l' || ch == 'r' || ch == 's') {
      P1OUT ^= BIT0;
    }
  }
}

void main_1s_routine() {
  
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
  
  // Initialize Servo State
  servo_state.is_moving = FALSE;
  servo_state.is_continuous = FALSE;
  servo_state.dir = SERVO_OFF;
  servo_state.countdown = 0;
}

void forward() {
  TA1CCR1 = SERVO_DIR_LEFT;                  // (2.2) Duty Cycle
  TA1CCR2 = SERVO_DIR_RIGHT;                 // (2.4) Duty Cycle
}

void stop() {
  TA1CCR1 = SERVO_DIR_STOP;                  // (2.2) Duty Cycle
  TA1CCR2 = SERVO_DIR_STOP;                  // (2.4) Duty Cycle
}

void left() {
  TA1CCR1 = SERVO_DIR_STOP;                  // (2.2) Duty Cycle
  TA1CCR2 = SERVO_DIR_RIGHT;                 // (2.4) Duty Cycle
}

void right() {
  TA1CCR1 = SERVO_DIR_LEFT;                  // (2.2) Duty Cycle
  TA1CCR2 = SERVO_DIR_STOP;                  // (2.4) Duty Cycle
}

void set_direction(unsigned dir) {
  switch (dir) {
    case SERVO_STOP:
      stop();
      break;
    case SERVO_LEFT:
      left();
      break;
    case SERVO_RIGHT:
      right();
      break;
    case SERVO_FORWARD:
      forward();
      break;
  }
}

void parse_direction(char *input) {
  char *ptr;
  long dir = strtol(input, &ptr, 0);
  long length = strtol(&ptr[1], NULL, 0);
  
  servo_state.is_moving = FALSE;
  servo_state.is_continuous = FALSE;
  servo_state.dir = dir;
  servo_state.countdown = 0;
  
  if (length == 0) {
    servo_state.is_continuous = TRUE;
  } else if (dir == SERVO_STOP) {
    servo_state.countdown = 0;
  } else {
    servo_state.countdown = length;
  }
}

void change_servo_status() {
  // If continuous
  if (servo_state.is_continuous == TRUE) {
    if (servo_state.is_moving == FALSE) {
      set_direction(servo_state.dir);
      servo_state.is_moving = TRUE;
      servo_state.countdown = 0;
    }
  }
  // If it's not continuous and there is a countdown
  else if (servo_state.countdown > 0) {
    if (servo_state.is_moving == FALSE) {
      set_direction(servo_state.dir);
      servo_state.is_moving = TRUE;
    } else {
      // Countdown
      P1OUT ^= BIT0;                         // Toggle red LED
      servo_state.countdown--;
      
      if (servo_state.countdown == 0) {
        servo_state.dir = SERVO_STOP; 
      }
    }
  }
  // If not continuous and zero countdown
  else if (servo_state.dir == SERVO_STOP) {
    set_direction(SERVO_STOP);
    servo_state.is_moving = FALSE;
    servo_state.dir = SERVO_OFF;
  }
}

/****************************************
    VCNL4000
****************************************/ 
void vcnl4000_init() {
  current_sensor = 0;
  
  i2c_write(IR_CURRENT, 20);                // Set IR current to 200mA
  i2c_write(PROXIMITY_FREQ, 2);             // 781.25 kHz
  i2c_write(PROXIMITY_MOD, 0x81);           // 129, recommended by Vishay
  left_sensor.is_initialized = TRUE;
  next_sensor();
  
  i2c_write(IR_CURRENT, 20);                // Set IR current to 200mA
  i2c_write(PROXIMITY_FREQ, 2);             // 781.25 kHz
  i2c_write(PROXIMITY_MOD, 0x81);           // 129, recommended by Vishay
  front_sensor.is_initialized = TRUE;
  next_sensor();
  
  i2c_write(IR_CURRENT, 20);                // Set IR current to 200mA
  i2c_write(PROXIMITY_FREQ, 2);             // 781.25 kHz
  i2c_write(PROXIMITY_MOD, 0x81);           // 129, recommended by Vishay
  right_sensor.is_initialized = TRUE;
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
      if (left_sensor.is_initialized) left_sensor.last_val = read_proximity();
      break;
    case SENS_FRONT:
      if (front_sensor.is_initialized) front_sensor.last_val = read_proximity();
      break;
    case SENS_RIGHT:
      if (right_sensor.is_initialized) right_sensor.last_val = read_proximity();
      break;
  }
  next_sensor();
  
  if (left_sensor.last_val < 2300 ||
      front_sensor.last_val < 2300 ||
      right_sensor.last_val > 2350) {
    P1OUT |= BIT0;
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
  UCA0CTL1 |= UCSSEL_2; 		 // SMCLK
  UCA0CTL0 = 0;
  UCA0ABCTL = 0;
  UCA0BR0 = 8; 			         // 1MHz 115200
  UCA0BR1 = 0; 			         // 1MHz 115200
  UCA0MCTL = UCBRS2 + UCBRS0; 	         // Modulation
  UCA0CTL1 &= ~UCSWRST; 		 // Initialize USCI state machine
  UC0IE |= UCA0TXIE + UCA0RXIE;
  IE2 |= UCA0TXIE + UCA0RXIE; 		 // Enable USCI_A0 RX interrupt
}

void uart_putc(unsigned char ch) {
  ring_push(&tx_buffer, ch);
  ring_push(&tx_buffer, '\r');
  UC0IE |= UCA0TXIE;
}

void uart_puts(const char *str) {
  while (*str)
    ring_push(&tx_buffer, *str++);
  ring_push(&tx_buffer, '\r');
  UC0IE |= UCA0TXIE;
}

bool uart_getc(unsigned char *ch) {
  return ring_pop(&rx_buffer, ch);
}

bool uart_gets(unsigned short *str) {
  if (!incoming) return false;

  unsigned int len = ring_len(&rx_buffer)+1;
  unsigned char buf[len];
  unsigned int i = 0;
  for (i; i < len; i++) {
    if (!ring_pop(&rx_buffer, &buf[i])) break;
  }
  
  *str = *(unsigned short *) buf;
  incoming = FALSE;
  return true;
}

/****************************************
    Main
****************************************/
int main(void) {
  WDTCTL  = WDTPW + WDTHOLD; 	         // Stop WDT
  BCSCTL1 = CALBC1_1MHZ; 		 // Set DCO
  DCOCTL  = CALDCO_1MHZ;

  P1DIR = BIT0 + S0 + S1;                   // P1.0 is the red LED	
  P1OUT = ~(BIT0 + S0 + S1);                // Turn all off

  // Initialize Modules
  servo_init();
  i2c_init();
  timer_init();
  uart_init();
  
  _EINT();                               // Register Global Interrupts
  
  vcnl4000_init();
  
  //parse_direction("3-0");

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
  if (IFG2 & UCA0RXIFG) {
    unsigned char ch = UCA0RXBUF;
    if (ch == '~') {
      incoming = TRUE;
    } else {
      ring_push(&rx_buffer, ch);
    }
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
    if (ring_empty(&tx_buffer)) {
      UC0IE &= ~UCA0TXIE;
    } else {
      unsigned char ch;
      if (ring_pop(&tx_buffer, &ch)) {
        UCA0TXBUF = ch;
      }
    }
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
