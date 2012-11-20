////////////////////////////////////////////////////////
//
// LCDI2Cw v0.1 
// Uses I2C Wires interface
// Uses Analog pin 4 - SDA
// Uses Analog pin 5 - SCL
////////////////////////////////////////////////////////
  
  #include <Wire.h>
  #include <stdio.h>
  #include <string.h>
  #include <inttypes.h>
  #include "Arduino.h"
  #include "LCDI2Cw_local.h"


//
// Note: resetting the Arduino doesn't reset the LCD, so we
// can't assume that its in that state when a sketch starts (and the
// LCDI2Cw constructor is called).

//constructor.  

uint8_t i2cAddr = 0x4C;   // Defalt address of the display

LCDI2Cw::LCDI2Cw (uint8_t cols, uint8_t lines, uint8_t i2c_address)
{
	i2cAddr = i2c_address;
}

void LCDI2Cw::begin() 
{
    Wire.begin(); 
    clear();
}

/********** high level commands, for the user! */
void LCDI2Cw::clear()
{
   Wire.beginTransmission(i2cAddr);
   Wire.write(LCDI2Cw_COMMAND);
   Wire.write(LCDI2Cw_CLEAR);
   Wire.endTransmission();
   delayMicroseconds(2000);
}

void LCDI2Cw::home()
{
   Wire.beginTransmission(i2cAddr);
   Wire.write(LCDI2Cw_COMMAND);
   Wire.write(LCDI2Cw_HOME);
   Wire.endTransmission();  // this command takes a long time!
   delayMicroseconds(2000);
}

///  Set cursor position: col - 0...19; row - 0...3
void LCDI2Cw::setCursor(uint8_t col, uint8_t row)
{
   Wire.beginTransmission(i2cAddr);
   Wire.write(LCDI2Cw_COMMAND);
   Wire.write(LCDI2Cw_SET_CURSOR);
   Wire.write(col);
   Wire.write(row);
   Wire.endTransmission();
}

// Turn the display on/off (quickly)
void LCDI2Cw::noDisplay() {
   Wire.beginTransmission(i2cAddr);
   Wire.write(LCDI2Cw_COMMAND);
   Wire.write(LCDI2Cw_NO_DISPLAY);
   Wire.endTransmission();
}
void LCDI2Cw::display() {
   Wire.beginTransmission(i2cAddr);
   Wire.write(LCDI2Cw_COMMAND);
   Wire.write(LCDI2Cw_DISPLAY);
   Wire.endTransmission();
}

// Turns the underline cursor on/off
void LCDI2Cw::noCursor() {
   Wire.beginTransmission(i2cAddr);
   Wire.write(LCDI2Cw_COMMAND);
   Wire.write(LCDI2Cw_NO_CURSOR);
   Wire.endTransmission();
}
void LCDI2Cw::cursor() {
   Wire.beginTransmission(i2cAddr);
   Wire.write(LCDI2Cw_COMMAND);
   Wire.write(LCDI2Cw_CURSOR);
   Wire.endTransmission();
}

// Turn on and off the blinking cursor
void LCDI2Cw::noBlink() {
   Wire.beginTransmission(i2cAddr);
   Wire.write(LCDI2Cw_COMMAND);
   Wire.write(LCDI2Cw_NO_BLINK);
   Wire.endTransmission();
}
void LCDI2Cw::blink() {
   Wire.beginTransmission(i2cAddr);
   Wire.write(LCDI2Cw_COMMAND);
   Wire.write(LCDI2Cw_BLINK);
   Wire.endTransmission();
}

// These commands move the cursor
void LCDI2Cw::cursorLeft(void) {
   Wire.beginTransmission(i2cAddr);
   Wire.write(LCDI2Cw_COMMAND);
   Wire.write(LCDI2Cw_CURSOR_LEFT);
   Wire.endTransmission();
}

void LCDI2Cw::cursorRight(void) {
   Wire.beginTransmission(i2cAddr);
   Wire.write(LCDI2Cw_COMMAND);
   Wire.write(LCDI2Cw_CURSOR_RIGHT);
   Wire.endTransmission();
}

// These commands scroll the display without changing the RAM
void LCDI2Cw::scrollDisplayLeft(void) {
   Wire.beginTransmission(i2cAddr);
   Wire.write(LCDI2Cw_COMMAND);
   Wire.write(LCDI2Cw_SCROLL_LEFT);
   Wire.endTransmission();
}
void LCDI2Cw::scrollDisplayRight(void) {
   Wire.beginTransmission(i2cAddr);
   Wire.write(LCDI2Cw_COMMAND);
   Wire.write(LCDI2Cw_SCROLL_RIGHT);
   Wire.endTransmission();
}


// Allows us to fill the first 8 CGRAM locations
// with custom characters
void LCDI2Cw::createChar(uint8_t location, uint8_t charmap[]) {
   location &= 0x7; // we only have 8 locations 0-7
   Wire.beginTransmission(i2cAddr);
   Wire.write(LCDI2Cw_COMMAND);
   Wire.write(LCDI2Cw_CREATE_CHAR);
   Wire.write(location);
   for (int i=0; i<8; i++) {
      Wire.write(charmap[i]);
   }
   Wire.endTransmission();
   delay(20);
}

/////////////////////////////////////////////////////////////////////////////////////
// Firmware version command
// Read the LCD firmware version 
//////////////////////////////////////////////////////////////////////////////////////
unsigned int LCDI2Cw::firmware(void){

  unsigned int data = 0;
  unsigned char number = 1;

  Wire.beginTransmission(i2cAddr);
  Wire.write(LCDI2Cw_COMMAND);
  Wire.write(LCDI2Cw_FIRM_VER);
  Wire.endTransmission();
  delayMicroseconds(100);
  
  //  Connect to device and request byte
  Wire.beginTransmission(i2cAddr);
  Wire.requestFrom(i2cAddr, number);

  if (Wire.available()) {
    data = Wire.read();
  }
  return data;
}

//////////////////////////////////////////////////////////////////////////////////////
// Keypad Mode command
// This function sets the Keypad Port mode
// mode: 0x00 - 4x4 matrix keypad mode 
//       0x01 - 8 button mode
//////////////////////////////////////////////////////////////////////////////////////
void LCDI2Cw::keypadMode(uint8_t mode){

      Wire.beginTransmission(i2cAddr);
      Wire.write(LCDI2Cw_COMMAND);
      Wire.write(LCDI2Cw_SET_KEYPAD_MODE);
      Wire.write(mode);  
      Wire.endTransmission();
}


//////////////////////////////////////////////////////////////////////////////////////
// Read keypad command
// Read a pressed keypad number from buffer
//////////////////////////////////////////////////////////////////////////////////////
unsigned int LCDI2Cw::keypad (void){

  unsigned int data = 0;
  unsigned char number = 1;

  //  Send Keypad read command
  Wire.beginTransmission(i2cAddr);
  Wire.write(LCDI2Cw_COMMAND);
  Wire.write(LCDI2Cw_READ_KEYPAD);
  Wire.endTransmission();
  delayMicroseconds(100);
  
  //  Connect to device and request byte
  Wire.beginTransmission(i2cAddr);
  Wire.requestFrom(i2cAddr, number);

  if (Wire.available()) {
    data = Wire.read();
  }

return data;
}

//////////////////////////////////////////////////////////////////////////////////////
// Read IR Remote control command
// Read a IR command from buffer
//////////////////////////////////////////////////////////////////////////////////////
unsigned int LCDI2Cw::remoteCtr (void){

  unsigned int data = 0;
  unsigned char number = 1;

  //  Send Keypad read command
  Wire.beginTransmission(i2cAddr);
  Wire.write(LCDI2Cw_COMMAND);
  Wire.write(LCDI2Cw_READ_REMOTE);
  Wire.endTransmission();
  delayMicroseconds(100);
  
  //  Connect to device and request byte
  Wire.beginTransmission(i2cAddr);
  Wire.requestFrom(i2cAddr, number);

  if (Wire.available()) {
    data = Wire.read();
  }

return data;
}

//////////////////////////////////////////////////////////////////////////////////////
// Backlight command
// This function sets the PWM value for the LCD backlight control
// light: 0x00 - backlight off 
//        0xFF - max backlight
//////////////////////////////////////////////////////////////////////////////////////
void LCDI2Cw::backlight(uint8_t light){

      Wire.beginTransmission(i2cAddr);
      Wire.write(LCDI2Cw_COMMAND);
      Wire.write(LCDI2Cw_BACKLIGHT);
      Wire.write(light);  
      Wire.endTransmission();
}

//////////////////////////////////////////////////////////////////////////////////////
// Contrast command
// This function sets the PWM value for the LCD contrast control
// contrast: from 0x00 to 0xFF 
//////////////////////////////////////////////////////////////////////////////////////
void LCDI2Cw::contrast(uint8_t contrast){

      Wire.beginTransmission(i2cAddr);
      Wire.write(LCDI2Cw_COMMAND);
      Wire.write(LCDI2Cw_CONTRAST);
      Wire.write(contrast);  
      Wire.endTransmission();
}



//////////////////////////////////////////////////////////////////////////////////////
// write command
// Send a data byte to LCD 
//////////////////////////////////////////////////////////////////////////////////////
inline size_t LCDI2Cw::write(uint8_t value) {
  Wire.beginTransmission(i2cAddr);
  Wire.write(value);
  Wire.endTransmission();
  return 1; // assume sucess
}



