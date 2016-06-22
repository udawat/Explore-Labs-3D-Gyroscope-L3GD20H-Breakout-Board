/*
 * Interface Code for Explore Labs 3D Gyroscope L3GD20H Breakout Board
 * and Arduino Uno. This code assumes selected communication method is
 * Serial Peripheral Interface (SPI) and hardware connections are made
 * as follows:
 * 
 * Sensor VIN - Arduino Uno 5V or 3.3V
 * Sensor GND - Arduino Uno GND
 * Sensor SDI/SDA - Arduino Uno Digital Pin 11 or MOSI (Master Out Slave In)
 * Sensor SCK/SCL - Arduino Uno Digital Pin 13 or SCK (Serial Clock)
 * Sensor SDO/ADR - Arduino Uno Digital Pin 12 or MISO (Master In Slave Out)
 * Sensor CS - Arduino Uno Digital Pin 10 or SS (Slave Select or Chip Select)
 */

#include <SPI.h>          // Include Arduino SPI library

#define CTRL1       0x20  // Register addresses from sensor datasheet.
#define OUT_X_L     0x28  // Only the registers that are used
#define OUT_X_H     0x29  // in this program are defined here.
#define OUT_Y_L     0x2A
#define OUT_Y_H     0x2B
#define OUT_Z_L     0x2C
#define OUT_Z_H     0x2D

int8_t readData   = 0x80;
int8_t writeData  = 0x00;

int16_t gx, gy, gz;       // 16-bit variables to hold raw data from sensor

const int CS = 10;        // Chip Select pin for SPI

void setup() {
  Serial.begin(9600);
  SPI.begin();
  pinMode(CS, OUTPUT);
  writeReg(CTRL1, 0x0F);  // Initialize the sensor by setting control register
  delay(100);
}

void loop() {
  gx = (int16_t) readReg(OUT_X_H) <<8 | readReg(OUT_X_L); // typecast as 16-bit
  gy = (int16_t) readReg(OUT_Y_H) <<8 | readReg(OUT_Y_L);
  gz = (int16_t) readReg(OUT_Z_H) <<8 | readReg(OUT_Z_L);
  
  Serial.print("Angular Velocity in degrees per second (dps):\t");
  
  //Sensitivity from Page 9 of datasheet - use 8.75 mdps/digit (LSb) for default 245dps.
  Serial.print(gx*0.00875F, DEC); Serial.print("\t");
  Serial.print(gy*0.00875F, DEC); Serial.print("\t");
  Serial.print(gz*0.00875F, DEC); Serial.print("\t");
  Serial.println();
  
  delay(100);
}

int8_t readReg(int8_t address) {
  int8_t buffer = 0;
  digitalWrite(CS, LOW); 
  SPI.transfer(readData | address);
  buffer = SPI.transfer(writeData);
  digitalWrite(CS, HIGH);
  return(buffer);  
}

void writeReg(int8_t address, int8_t val) {
  digitalWrite(CS, LOW);
  SPI.transfer(writeData | address);
  SPI.transfer(val);
  digitalWrite(CS, HIGH);
}
