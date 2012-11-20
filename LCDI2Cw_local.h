#ifndef LCDI2Cw_h
#define LCDI2Cw_h

#include <inttypes.h>
#include "Print.h"
#include <Wire.h>

//////////////////// Command List ///////////////////////////////////////
#define LCDI2Cw_CLEAR                   0x14
#define LCDI2Cw_HOME                    0x0D
#define LCDI2Cw_SET_CURSOR              0x0C
#define LCDI2Cw_NO_DISPLAY              0x0B
#define LCDI2Cw_DISPLAY                 0x0A
#define LCDI2Cw_NO_CURSOR               0x0F
#define LCDI2Cw_CURSOR                  0x0E
#define LCDI2Cw_NO_BLINK                0x13
#define LCDI2Cw_BLINK                   0x12
#define LCDI2Cw_CURSOR_RIGHT            0x11
#define LCDI2Cw_CURSOR_LEFT             0x10
#define LCDI2Cw_SCROLL_LEFT             0x1F
#define LCDI2Cw_SCROLL_RIGHT            0x20
#define LCDI2Cw_CREATE_CHAR             0x1A

#define LCDI2Cw_FIRM_VER                0x24
#define LCDI2Cw_SET_KEYPAD_MODE         0x31
#define LCDI2Cw_READ_KEYPAD             0x32
#define LCDI2Cw_READ_REMOTE             0x35
#define LCDI2Cw_BACKLIGHT               0x03
#define LCDI2Cw_CONTRAST                0x04

#define LCDI2Cw_COMMAND                 0xFE

class LCDI2Cw : public Print {
public:
  
  LCDI2Cw (uint8_t cols, uint8_t lines, uint8_t i2c_address);
    
  void begin();

  void clear();
  void home();

  void noDisplay();
  void display();
  void noBlink();
  void blink();
  void noCursor();
  void cursor();
  void cursorLeft(); 
  void cursorRight();   
  void scrollDisplayLeft();
  void scrollDisplayRight();
  void createChar(uint8_t, uint8_t[]);
  void setCursor(uint8_t, uint8_t); 
 
  unsigned int firmware();
  void keypadMode(uint8_t mode);
  unsigned int keypad ();
  unsigned int remoteCtr ();
  void backlight(uint8_t light);
  void contrast(uint8_t contrast);

  virtual size_t write(uint8_t);
  using Print::write; 

private:
  uint8_t i2cAddr;
};

#endif
