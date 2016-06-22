# Explore Labs 3D Gyroscope L3GD20H Breakout Board
## Overview
This breakout board is based on ST L3GD20H three-axis gyroscope, which measures the angular rates of rotation about the roll (X), pitch (Y), and yaw (Z) axes. Angular velocity measurements with a configurable range of ±245°/s, ±500°/s, or ±2000°/s can be read through a digital I2C or SPI interface.

The carrier board includes a low-dropout linear voltage regulator that provides the 3.3 V required by the L3GD20H, which allows the sensor to be powered from 2.5 V to 5.5 V. The regulator output is available on the 3Vo pin which can supply approx. 150 mA to external devices.

The breakout board also includes a circuit that shifts the two I2C lines or all four SPI lines (clock, data-in/out and chip select) to the same logic voltage level as the supplied VIN, making it simple to interface the board with 5V systems; and the board’s 0.1″ pin spacing makes it easy to use with standard solderless breadboards and 0.1″ perfboards. The board ships fully populated with its SMD components, including the L3GD20H, as shown in the product picture.

## Features of Explore Labs 3D Gyroscope L3GD20H Breakout Board:
* Wide supply voltage - 2.5V to 5V
* Level shifted IO lines for easy interfacing to high voltage boards like Arduino Uno, Mega, Leonardo, etc.
* All pins of the L3GD20H sensor are available like INT1, INT2 (DRDY) and DEN (Data Enable)
* Layout takes care of keeping the area below the sensor free from any active current carrying trace or a power or ground plane
* Capacitors with low ESR characteristics are used for decoupling on both VDD and VDDIO power inputs
* Open Source Hardware
* Designed, Assembled and Made in India

## Features of L3GD20H sensor IC:
* Extended operating temperature range (from -40 °C to 85 °C)
* Low power consumption
* Embedded power-down
* Sleep mode
* Fast turn-on and wake-up
* Three selectable full scales up to 2000 dps (degrees per second)
* 16 bit rate value data output
* 8 bit temperature data output
* I2C/SPI digital output interface
* 2 dedicated lines (1 interrupt, 1 data ready)
* User enable integrated high-pass filters
* Embedded temperature sensor
* Embedded 32 levels of 16 bit data output FIFO
* High shock survivability

## Specifications:
* Input Voltage: 2.5V to 5V
* 5V Ready IOs: Level shifted IO lines for easy interfacing to incompatible voltage boards
* Digital Output: Inter Integrated Circuit (I2C/IIC) or Serial Peripheral Interface (SPI)
* Regulator Pin - Provides 3.3V Out at 150mA (when 3.6V < VIN < 5V) else provides voltage ~equal to VIN
* Dimensions: 20.4 mm x 20 mm
* Weight: 2 grams

## Applications:
* Gaming and virtual reality input devices
* Motion control with MMI (man-machine interface)
* GPS navigation systems
* Appliances and robotics

## Output Format:
Provides individual readings for each axis in degrees per decond (dps). These readings may be used separately or together with an Accelerometer for 3D calculations like calculating euler angles (roll, pitch, yaw) and other complex orientation calculationsfor Inertial Measurement Units (IMU).

## Pinout Table for Explore Labs 3D Gyroscope L3GD20H Breakout Board
|Pin|5V Safe?   |Function|
|---|-----------|--------|
|VIN|Yes|**Input Voltage** Pin: 2.5V to 5.0V (depending on Master device being used as mentioned in later connection sections and tables below)|
|3Vo|Not Applicable|This is the output pin of the onboard 3.3V Voltage Regulator. It provides a clean and regulated 3.3V output to the sensor IC. It can also be used by the user to provide power to external devices or sensors. Maximum current that can be sourced easily would be approximately 125mA to 150mA after leaving some room for the on board sensor IC|
|GND|Not Applicable|Ground Connection. Connect this pin to the master device's ground pin or to any externally powered devices from onboard regulator|
|SDI/SDA|Yes. Bidirectional MOSFET with 4K7 pull-ups|This pin has dual functions - **Serial Data Input** (for SPI) or **Serial Data** (for I2C). In case of SPI, it can also be called as Master Out Slave In (MOSI)|
|SCK/SCL|Yes. Bidirectional MOSFET with 4K7 pull-ups|This pin is the **Serial Clock** input pin. It is common for both SPI and I2C interfaces|
|SDO/ADR|Yes. Only a 4K7 pull-up resistor needed as this is an output pin|This pin has dual functions - **Serial Data Output** (for SPI) or **Address Select** (for I2C). Also called as SA0. In case of SPI, it can also be called as Master In Slave Out (MOSI). In case of I2C, this pin may be pulled LOW or grounded for selecting an alternate address to avoid address conflicts with similar I2C devices. The change will have to be updated in code as well. Leave unconnected if no change in address required for I2C|
|CS|Yes. Bidirectional MOSFET with 4K7 pull-ups|**Chip Select** or Slave Select pin. By default, it is pulled HIGH using an onboard 4K7 pull-up resistor. This sets the default mode of communication for the sensor IC as I2C. For SPI, this pin acts as the chip select pin to communicate with an SPI master device. Leave unconnected when using I2C|
|DEN|No. Take proper care when interfacing with devices that have higher voltage levels. This pin is not 5V tolerant!|**Data Enable** input pin. The L3GD20H allows external trigger level recognition through enabling the EXTRen (bit7) and LVLen (bit6) bits in CTRL2 (21h) register and the IMPen (bit3) bit of CTRL4 register. Three different modes can be used: level-sensitive, pulse-sensitive and edge-sensitive trigger. More detailed explanation can be found in [Section 3.5 of ST Application Note AN4506](http://www.st.com/resource/en/application_note/dm00119037.pdf "ST Application Note - L3GD20H: 3-axis digital output gyroscope")|
|INT1|No|Interrupt output pin. The L3GD20H interrupt signal can be configured in a very flexible way allowing to recognize independent rotations of the X, Y and Z-axis. That signal can be driven to the INT1 pin. The device may be configured to generate interrupt signals by detecting an independent wake-up event. Thresholds and timing of the interrupt generator are programmable by the end user on the fly. More detailed explanation can be found in [Section 5 of ST Application Note AN4506](http://www.st.com/resource/en/application_note/dm00119037.pdf "ST Application Note - L3GD20H: 3-axis digital output gyroscope")|
|INT2|No|Second Interrupt output pin. This pin is dedicated to DRDY and FIFO interrupts. **DRDY Interrupt**: The device may be configured to have one HW signal to determine when a new set of measurement data is available for reading. This signal is represented by the XYZDA bit of the STATUS register. The signal can be driven to the DRDY/INT2 pin. **FIFO Interrupt**: The device may be configured to generate an interrupt on FIFO Threshold, FIFO Overrun or FIFO Empty event. For more details, refer to above application note from ST. More detailed explanation can be found in [Section 3.3 and Section 5.1 of ST Application Note AN4506](http://www.st.com/resource/en/application_note/dm00119037.pdf "ST Application Note - L3GD20H: 3-axis digital output gyroscope")|

## Connections with various development boards are explained below:

The carrier board supports both I2C as well as SPI method of communication with a host microcontroller. The communication mode is selected by pulling the CS pin HIGH or LOW. By default, the CS pin is pulled HIGH through a 4.7 kΩ resistor on the board. This makes I2C communication the default method to talk to this carrier board.

**I2C Communication**: By default, I2C communication is implemented. The Explore Labs 3D Gyroscope L3GD20H Breakout Board can be configured and its angular velocity readings can be queried through the I2C bus. Level shifters on the I2C clock (SCL) and data (SDA) lines enable I2C communication with microcontrollers operating at the same voltage as VIN (2.5V to 5V). A detailed explanation of the I2C interface on the L3GD20H can be found in its [datasheet][1] and more detailed information about I2C Protocol in general can be found in NXP’s I2C-bus specification manual.

**I2C Address**: In I2C mode, the gyro’s 7-bit slave address has its least significant bit (LSb) determined by the voltage on the SDO/ADR pin. The carrier board pulls SDO/ADR HIGH through a 4.7 kΩ resistor, making the LSb 1 and setting the slave address to 1101011b (0x6B). This is the default device address in I2C mode (same address as the previous generation L3GD20). If SDO/ADR pin is connected to ground, LSb value is ‘0’ which changes the device address to 1101010b (0x6A). This solution permits to connect and address two different gyroscopes to the same I2C bus.

**Arduino Uno with I2C Interface**
* Sensor VIN - Arduino Uno 5V
* Sensor GND - Arduino Uno GND
* Sensor SDI/SDA - Arduino Uno Analog Input Pin A4 or SDA
* Sensor SCK/SCL - Arduino Uno Analog Input Pin A5 or SCL

**Raspberry Pi (all models) with I2C Interface**
* Sensor VIN - Raspberry Pi 3.3V on Pin 1 or 17
* Sensor GND - Raspberry Pi GND
* Sensor SDI/SDA - Raspberry Pi SDA1 (I2C) on Pin 3 or GPIO2
* Sensor SCK/SCL - Raspberry Pi SCL1 (I2C) on Pin 5 or GPIO3

**BeagleBone Black with I2C Interface**
* Sensor VIN - BeagleBone Black 3.3V
* Sensor GND - BeagleBone Black GND
* Sensor SDI/SDA - BeagleBone Black I2C2_SDA (Pin 20) on Header P9
* Sensor SCK/SCL - BeagleBone Black I2C2_SCL (Pin 19) on Header P9

## Connecting Explore Labs 3D Gyroscope L3GD20H Breakout Board using I2C (Inter Integrated Circuit) Interface
|L3GD20H|Arduino Uno|Arduino Mega|Arduino Leonardo|Arduino Due|Genuino Zero|Genuino 101|Arduino MKR1000|Raspberry Pi|BeagleBone Black|
|---|---|---|---|---|---|---|---|---|---|
|VIN|5V|5V|5V|3.3V|3.3V|VCC (3.3V)|3.3V|3.3V|3.3V|
|GND|GND|GND|GND|GND|GND|GND|GND|GND|GND|
|SDI/SDA|A4 or SDA|Pin 20 (SDA)|Pin 2 (SDA)|Pin 20 (SDA)|SDA|SDA|Pin 12 (SDA)|Pin 3 (SDA1) or GPIO2|Pin 20 (I2C2_SDA)|
|SCK/SCL|A5 or SCL|Pin 21 (SCL)|Pin 3 (SCL)|Pin 21 (SCL)|SCL|SCL|Pin 11 (SCL)|Pin 5 (SCL1) or GPIO3|Pin 19 (I2C2_SCL)|

## SPI Communication:
To use the L3GD20H in SPI mode, CS pin must be driven low (connected to GND). A minimum of four logic connections are used viz., SDI, SCK, SDO and CS. These should be connected to an SPI bus operating at the same logic level as VIN. The SPI interface operates in 4-wire mode by default, with SDI and SDO on separate pins. In the default 4-wire mode, the sensor transmits data to the SPI master on a dedicated data out (SDO) line. If the SPI interface is configured to use 3-wire mode instead, the SDI line doubles as SDO and is driven by the L3GD20H when it transmits data to the master. A detailed explanation of the SPI interface on the L3GD20H can be found in its datasheet.

**Arduino Uno with Serial Peripheral Interface (SPI)**
* Sensor VIN - Arduino Uno 5V
* Sensor GND - Arduino Uno GND
* Sensor SDI/SDA - Arduino Uno Digital Pin 11 or MOSI (Master Out Slave In)
* Sensor SCK/SCL - Arduino Uno Digital Pin 13 or SCK (Serial Clock)
* Sensor SDO/ADR - Arduino Uno Digital Pin 12 or MISO (Master In Slave Out)
* Sensor CS - Arduino Uno Digital Pin 10 or SS (Slave Select or Chip Select)

**Raspberry Pi (all models) with Serial Peripheral Interface (SPI)**
* Sensor VIN - Raspberry Pi 3.3V on Pin 1 or 17
* Sensor GND - Raspberry Pi GND
* Sensor SDI/SDA - Raspberry Pi `MOSI` (SPI) on Pin 19 or GPIO10
* Sensor SCK/SCL - Raspberry Pi `SCLK` (SPI) on Pin 23 or GPIO11
* Sensor SDO/ADR - Raspberry Pi `MISO` (SPI) on Pin 21 or GPIO9
* Sensor CS - Raspberry Pi `CE0_N` (SPI) on Pin 24 or GPIO8

**BeagleBone Black with Serial Peripheral Interface (SPI)**
* Sensor VIN - BeagleBone Black 3.3V
* Sensor GND - BeagleBone Black GND
* Sensor SDI/SDA - BeagleBone Black `SPI0_D0` (Pin 21) on Header P9
* Sensor SCK/SCL - BeagleBone Black `SPI0_SCLK` (Pin 22) on Header P9
* Sensor SDO/ADR - BeagleBone Black `SPI0_D1` (Pin 18) on Header P9
* Sensor CS - BeagleBone Black `SPI0_CS0` (Pin 17) on Header P9

## Connecting Explore Labs 3D Gyroscope L3GD20H Breakout Board using Serial Peripheral Interface (SPI)
|L3GD20H|Arduino Uno|Arduino Mega|Arduino Leonardo|Arduino Due|Genuino Zero|Genuino 101|Arduino MKR1000|Raspberry Pi|BeagleBone Black|
|---|---|---|---|---|---|---|---|---|---|
|VIN|5V|5V|5V|3.3V|3.3V|VCC (3.3V)|3.3V|3.3V|3.3V|
|GND|GND|GND|GND|GND|GND|GND|GND|GND|GND|
|SDI/SDA|Pin 11 or MOSI|Pin 51|Pin 4 (ICSP)|Pin 4 (ICSP)|Pin 4 (ICSP)|Pin 11 (MOSI)|Pin 8 (MOSI)|Pin 19 (`MOSI`) or GPIO10|Pin 21 (`SPI0_D0`)|
|SCK/SCL|Pin 13 or SCK|Pin 52|Pin 3 (ICSP)|Pin 3 (ICSP)|Pin 3 (ICSP)|Pin 13|Pin 9|Pin 23 (`SCLK`) or GPIO11|Pin 22 (`SPI0_SCLK`)|
|SDO/ADR|Pin 12 or MISO|Pin 50|Pin 1 (ICSP)|Pin 1 (ICSP)|Pin 1 (ICSP)|Pin 12 (MISO)|Pin 10 (MISO)|Pin 21 (`MISO`) or GPIO9|Pin 18 (`SPI0_D1`)|
|CS|Pin 10 or SS|Pin 53|Any IO|Any IO|Any IO|Pin 10|Any IO|Pin 24 (`CE0_N`) or GPIO8|Pin 17 (`SPI0_CS0`)|

## Working Principle:
It includes a sensing element and an IC interface able to provide the measured angular rate to the external world through digital interface (I2C/SPI). The sensing element is manufactured using a dedicated micromachining process developed by ST to produce inertial sensors and actuators on silicon wafers. The IC interface is manufactured using a CMOS process that allows a high level of integration to design a dedicated circuit which is trimmed to better match the sensing element characteristics.

## Resources:
[1]: http://www.st.com/resource/en/datasheet/l3gd20h.pdf
[Datasheet ST L3GD20H: MEMS motion sensor: 3-axis digital output gyroscope](http://www.st.com/resource/en/datasheet/l3gd20h.pdf "Datasheet ST L3GD20H: MEMS motion sensor: 3-axis digital output gyroscope")

## Code:
Sample code is provided for interfacing the Explore Labs 3D Gyroscope L3GD20H Breakout Board to an Arduino Uno. After choosing the communication method (I2C or SPI), following code examples can be used to test the sensor.

**Using SPI**:
```c
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
 * 
 */

#include  // Include Arduino SPI library

#define CTRL1       0x20
#define OUT_X_L     0x28
#define OUT_X_H     0x29
#define OUT_Y_L     0x2A
#define OUT_Y_H     0x2B
#define OUT_Z_L     0x2C
#define OUT_Z_H     0x2D

int8_t readData      = 0x80;
int8_t writeData     = 0x00;
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
```

**Using I2C**:
```c
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
 * 
 */
 
#include  // Include Arduino I2C library

#define CTRL1       0x20
#define OUT_X_L     0x28
#define OUT_X_H     0x29
#define OUT_Y_L     0x2A
#define OUT_Y_H     0x2B
#define OUT_Z_L     0x2C
#define OUT_Z_H     0x2D

int8_t readData      = 0x01;
int8_t writeData     = 0x00;
int8_t address       = 0xD6;  //address of L3GD20H with SDO/ADR/SA0 connected to 3Vo.
//int8_t address     = 0xD4;  //address of L3GD20H with SDO/ADR/SA0 connected to GND.

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
```
