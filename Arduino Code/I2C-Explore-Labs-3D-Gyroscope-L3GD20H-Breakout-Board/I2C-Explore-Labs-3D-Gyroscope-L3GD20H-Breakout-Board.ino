/*
 * Interface Code for Explore Labs 3D Gyroscope L3GD20H Breakout Board
 * and Arduino Uno. This code assumes selected communication method is
 * Inter Integrated Circuit (I2C) Interface and hardware connections are made
 * as follows:
 * 
 * Sensor VIN - Arduino Uno 5V or 3.3V
 * Sensor GND - Arduino Uno GND
 * Sensor SDI/SDA - Arduino Uno Analog Input Pin A4
 * Sensor SCK/SCL - Arduino Uno Analog Input Pin A5 
 */
 
#include <Wire.h>         // Include Arduino I2C library

#define CTRL1       0x20  // Register addresses from sensor datasheet.
#define OUT_X_L     0x28  // Only the registers that are used
#define OUT_X_H     0x29  // in this program are defined here.
#define OUT_Y_L     0x2A
#define OUT_Y_H     0x2B
#define OUT_Z_L     0x2C
#define OUT_Z_H     0x2D

int8_t readData   = 0x01;
int8_t writeData  = 0x00;
int8_t address    = 0xD6; // Device address of L3GD20H with SDO/ADR/SA0 pulled HIGH.
//int8_t address  = 0xD4; // Device address of L3GD20H with SDO/ADR/SA0 connected to GND.

int16_t gx, gy, gz;       // 16-bit variables to hold raw data from sensor

void setup() {
  Serial.begin(9600);
  Wire.begin();
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

int8_t readReg(int8_t reg) {
  int8_t buffer = 0;
  Wire.beginTransmission((address | writeData) >>1);
  Wire.write(reg);
  Wire.endTransmission(0);
  Wire.requestFrom((address | readData) >>1 , 1);
  while(Wire.available() == 0);
  buffer = Wire.read();  
  Wire.endTransmission();
  return(buffer);  
}

void writeReg(int8_t reg, int8_t val) {
  Wire.beginTransmission((address | writeData) >>1);
  Wire.write(reg);
  Wire.write(val);
  Wire.endTransmission();
}
