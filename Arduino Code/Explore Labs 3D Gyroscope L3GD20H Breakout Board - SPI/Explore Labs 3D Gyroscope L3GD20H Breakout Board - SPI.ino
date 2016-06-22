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

#include <SPI.h> // Include Arduino SPI library

#define CTRL1       0x20
#define OUT_X_L     0x28
#define OUT_X_H     0x29
#define OUT_Y_L     0x2A
#define OUT_Y_H     0x2B
#define OUT_Z_L     0x2C
#define OUT_Z_H     0x2D

int8_t readData   = 0x80;
int8_t writeData  = 0x00;
int16_t gx, gy, gz;

const int CS = 10;

void setup() {
  Serial.begin(9600);
  SPI.begin();
  pinMode(CS, OUTPUT);
  writeRegister(CTRL1, 0x0F);
  delay(100);
}

void loop() {
  gx = readRegister(OUT_X_L) <<8 | readRegister(OUT_X_H);
  gy = readRegister(OUT_Y_H) <<8 | readRegister(OUT_Y_L);
  gz = readRegister(OUT_Z_H) <<8 | readRegister(OUT_Z_L);
  Serial.print("Angular Velocity (dps):\t");
  //to convert the raw data to dps, use 8.75 mdps/digit (LSb) for default 250dps
  Serial.print((gx+0), DEC); Serial.print("\t"); //replace +zero with x-axis offset value
  Serial.print((gy+0), DEC); Serial.print("\t"); //replace +zero with y-axis offset value
  Serial.print((gz+0), DEC); Serial.print("\t"); //replace +zero with z-axis offset value
  Serial.println();
  delay(100);
}

int8_t readRegister(int8_t address) {
  
  int8_t buffer = 0;
  digitalWrite(CS, LOW); 
  SPI.transfer(readData | address);
  buffer = SPI.transfer(writeData);
  Serial.print(address, BIN);
  Serial.print(" : ");
  Serial.println(buffer, BIN);
  digitalWrite(CS, HIGH);
  return(buffer);  
}

void writeRegister(int8_t address, int8_t val) {
  
  digitalWrite(CS, LOW);
  SPI.transfer(writeData | address);
  SPI.transfer(val);
  digitalWrite(CS, HIGH);
}
