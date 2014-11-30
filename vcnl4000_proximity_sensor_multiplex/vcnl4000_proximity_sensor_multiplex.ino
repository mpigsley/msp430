/****************************************
    Includes
****************************************/
#include "msp430g2553.h"

/****************************************
    Defines
****************************************/
#define TRUE 1
#define FALSE 0

#define I2C_SCL BIT6
#define I2C_SDA BIT7
#define I2C_ADDR 1
#define I2C_DATA 2
#define I2C_STOP 3
#define I2C_BUF_SIZE 130

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

/****************************************
    Structs
****************************************/
typedef struct time_flags {
  unsigned time_1ms :1;
  unsigned time_100ms :1;
  unsigned time_1s :1;
} time_flags;

typedef struct vcnl4000_control {
  unsigned is_initialized :1;
  unsigned int last_val;
} vcnl4000_control;

/****************************************
    Global Vars
****************************************/
volatile struct time_flags sys_time;
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

/****************************************
    Timer
****************************************/
void timer_init() {
  TA0CCTL0 = CCIE;                           // Enable Timer A0 interrupts
  TA0CCR0 = 1000;                            // Count limit
  TA0CTL = TASSEL_2 + MC_1;                  // Timer A0 with SMCLK, count UP
}

void main_100ms_routine() {
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
  
  if (left_sensor.last_val < 2400 ||
      front_sensor.last_val < 2350 ||
      right_sensor.last_val < 2350) {
    P1OUT |= BIT0;
  } else {
    P1OUT &= ~BIT0;
  }
}

void main_1s_routine() {
  _NOP();
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
  if(buf_ptr < I2C_BUF_SIZE)
    buf[buf_ptr++] = data;
}

void i2c_read_start(void) {
  while (UCB0CTL1 & UCTXSTP);               // Ensure stop condition got sent
  UCB0CTL1 &= ~UCTR ;                       // Clear UCTR
  UCB0CTL1 |= UCTXSTT;                      // I2C start condition
}

bool i2c_slave_not_present(void) {
  UCB0CTL1 |= UCTR + UCTXSTT + UCTXSTP;     // I2C TX, start condition + stop cond immmediately
  while (UCB0CTL1 & UCTXSTP);               // wait for STOP condition
  return UCB0STAT & UCNACKIFG;              // return 1 if not present (nack) 
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

/****************************************
    Main
****************************************/
int main(void) {
  WDTCTL  = WDTPW + WDTHOLD; 	       // Stop WDT
  BCSCTL1 = CALBC1_8MHZ; 	       // Set DCO
  DCOCTL  = CALDCO_8MHZ;
  BCSCTL2 |= DIVS_3;                   // SMCLK to 1 MHz
  
  P1DIR = BIT0 + S0 + S1;              // Initialize red LED & multiplexer sigs
  P1OUT = ~(BIT0 + S0 + S1);           // Turn all off
  
  i2c_init();
  timer_init();
  
  _EINT();
  
  vcnl4000_init();
  
  while (1) {
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
  
}

#pragma vector=USCIAB0TX_VECTOR 
__interrupt void USCI0TX_ISR(void) {
  if (IFG2 & UCB0RXIFG) {  
    receive_field = UCB0RXBUF;
  } else if (IFG2 & UCB0TXIFG) {
    if (byte_ctr == 0){
      UCB0CTL1 |= UCTXSTP;                  // I2C stop condition
      IFG2 &= ~UCB0TXIFG;                   // Clear USCI_B0 TX int flag
      is_i2c_busy = 0;
    } else {
      UCB0TXBUF = *transmit_field;
      transmit_field++;
      byte_ctr--;
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
