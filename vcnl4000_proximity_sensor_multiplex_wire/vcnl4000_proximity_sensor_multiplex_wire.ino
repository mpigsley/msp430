#include "msp430g2553.h"
#include <Wire.h>

#define VCNL4000_ADDRESS 0x13  // 0x26 write, 0x27 read

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

#define S0 BIT4
#define S1 BIT5

int proximityValue;

void setup() {
  P1SEL = BIT0 + BIT6;
  P1DIR = S0 + S1;
  P1OUT = 0;
  
  Serial.begin(9600);
  Wire.begin();
  
  byte temp = readByte(PRODUCT_ID);
  if (temp != 0x11) {
    Serial.print("Something's wrong. Not reading correct ID: 0x");
    Serial.println(temp, HEX);
  } else Serial.println("VNCL4000 Online...");
  
  P1OUT &= ~(S0 + S1);
  writeByte(AMBIENT_PARAMETER, 0x0F);  // Single conversion mode, 128 averages
  writeByte(IR_CURRENT, 20);  // Set IR current to 200mA
  writeByte(PROXIMITY_FREQ, 2);  // 781.25 kHz
  writeByte(PROXIMITY_MOD, 0x81);  // 129, recommended by Vishay
  
  P1OUT |= S0;
  writeByte(AMBIENT_PARAMETER, 0x0F);  // Single conversion mode, 128 averages
  writeByte(IR_CURRENT, 20);  // Set IR current to 200mA
  writeByte(PROXIMITY_FREQ, 2);  // 781.25 kHz
  writeByte(PROXIMITY_MOD, 0x81);  // 129, recommended by Vishay
  
  P1OUT &= ~S0;
  P1OUT |= S1;
  writeByte(AMBIENT_PARAMETER, 0x0F);  // Single conversion mode, 128 averages
  writeByte(IR_CURRENT, 20);  // Set IR current to 200mA
  writeByte(PROXIMITY_FREQ, 2);  // 781.25 kHz
  writeByte(PROXIMITY_MOD, 0x81);  // 129, recommended by Vishay
}

void loop() {
  // First Sensor
  P1OUT &= ~(S0 + S1);
  proximityValue = readProximity();
  Serial.print(proximityValue > 4000, DEC);
  Serial.print(" ");
  
  // Second Sensor
  P1OUT |= S0;
  proximityValue = readProximity();
  Serial.print(proximityValue > 4000, DEC);
  Serial.print(" ");
  
  // Third Sensor
  P1OUT &= ~S0;
  P1OUT |= S1;
  proximityValue = readProximity();
  Serial.println(proximityValue > 4000, DEC);
}

unsigned int readProximity() {
  unsigned int data;
  byte temp;
  
  temp = readByte(COMMAND_0);
  writeByte(COMMAND_0, temp | 0x08);  // command the sensor to perform a proximity measure
  
  while(!(readByte(COMMAND_0)&0x20)) 
    ;  // Wait for the proximity data ready bit to be set
  data = readByte(PROXIMITY_RESULT_MSB) << 8;
  data |= readByte(PROXIMITY_RESULT_LSB);
  
  return data;
}

void writeByte(byte address, byte data) {
  Wire.beginTransmission(VCNL4000_ADDRESS);
  Wire.write(address);
  Wire.write(data);
  Wire.endTransmission();
}

byte readByte(byte address) {
  byte data;
  
  Wire.beginTransmission(VCNL4000_ADDRESS);
  Wire.write(address);
  Wire.endTransmission();
  Wire.requestFrom(VCNL4000_ADDRESS, 1);
  while(!Wire.available())
    ;
  data = Wire.read();

  return data;
}
