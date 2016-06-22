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
 
#include <Wire.h> // Include Arduino I2C library

#define CTRL1       0x20
#define OUT_X_L     0x28
#define OUT_X_H     0x29
#define OUT_Y_L     0x2A
#define OUT_Y_H     0x2B
#define OUT_Z_L     0x2C
#define OUT_Z_H     0x2D

int8_t readData   = 0x01;
int8_t writeData  = 0x00;
int8_t address    = 0xD6;  //address of L3GD20H with SDO/ADR/SA0 connected to 3Vo.
//int8_t address  = 0xD4;  //address of L3GD20H with SDO/ADR/SA0 connected to GND.

int16_t gx, gy, gz;

void setup() {
  Serial.begin(9600);
  Wire.begin();
  writeRegister(CTRL1, 0x0F);
  delay(100);
}

void loop() {
  gx = readRegister(OUT_X_H) <<8 | readRegister(OUT_X_L);
  gy = readRegister(OUT_Y_H) <<8 | readRegister(OUT_Y_L);
  gz = readRegister(OUT_Z_H) <<8 | readRegister(OUT_Z_L);
  Serial.print("Angular Velocity (dps):\t");
  //to convert the raw data to dps, use 8.75 mdps/digit (LSb) for default 250dps
  Serial.print((gx+0)*0.00875F, DEC); Serial.print("\t"); //replace +zero with x-axis offset value
  Serial.print((gy+0)*0.00875F, DEC); Serial.print("\t"); //replace +zero with y-axis offset value
  Serial.print((gz+0)*0.00875F, DEC); Serial.print("\t"); //replace +zero with z-axis offset value
  Serial.println();
  delay(100);
}

int8_t readRegister(int8_t reg) {
  int8_t buffer = 0;
  Wire.beginTransmission((address | writeData) >>1 ); //slave ID start talking
  Wire.write(reg);
  Wire.endTransmission(0);
  Wire.requestFrom((address | readData) >>1 , 1);
  while(Wire.available() == 0);
  buffer = Wire.read();  
  Wire.endTransmission();
  return(buffer);  
}

void writeRegister(int8_t reg, int8_t val) {
  Wire.beginTransmission((address | writeData) >>1 );
  Wire.write(reg);
  Wire.write(val);
  Wire.endTransmission();
}
