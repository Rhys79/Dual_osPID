#include <EEPROM.h>
#include <Wire.h>
#include "MAX31855_local.h"
#include "PID_v1_local.h"
#include "EEPROMAnything.h"
#include "PID_AutoTune_v0_local.h"
#include "LCDI2Cw_local.h"

// ***** PIN ASSIGNMENTS *****

const byte buzzerPin = 11;
const byte systemLEDPin = 13;
const byte thermocoupleCS = 8;
const byte thermocoupleCS2 = 6;
const byte thermocoupleSO = 12;
const byte thermocoupleCLK = 4;
const byte SSRPin = 9;
const byte SSRPin2 = 10;
const unsigned char i2cAddress = 0x4C;  // LCD module I2C address

const byte EEPROM_ID = 2; //used to automatically trigger and eeprom reset after firmware update (if necessary)

MAX31855 thermocouple(thermocoupleSO, thermocoupleCS, thermocoupleCLK);
MAX31855 thermocouple2(thermocoupleSO, thermocoupleCS2, thermocoupleCLK);
LCDI2Cw lcd(16, 2, i2cAddress);

unsigned long now, buttonTime, ioTime, serialTime;
boolean sendInfo=true, sendDash=true, sendTune=true, sendAtune=true;

bool editing=false;

bool tuning = false;
bool tuning2= false;

double error, error2;
int val, val2;

double setpoint=250, input=250, output=50, pidInput=250;
double setpoint2=250, input2=250, output2=50, pidInput2=250;

double kp = 2, ki = 0.5, kd = 2;
double kp2 = 2, ki2 = 0.5, kd2 = 2;
byte ctrlDirection = 0;
byte ctrlDirection2 = 0;
byte modeIndex = 1;
byte modeIndex2 = 1;
byte databyte;

PID myPID(&pidInput, &output, &setpoint,kp,ki,kd, DIRECT);
PID myPID2(&pidInput2, &output2, &setpoint2,kp2,ki2,kd2, DIRECT);

double aTuneStep = 20, aTuneNoise = 1;
double aTuneStep2 = 20, aTuneNoise2 = 1;
unsigned int aTuneLookBack = 10;
unsigned int aTuneLookBack2 = 10;
byte ATuneModeRemember = 0;
byte ATuneModeRemember2 = 0;
PID_ATune aTune(&pidInput, &output);
PID_ATune aTune2(&pidInput2, &output2);


byte curProfStep=0;
byte curProfStep2=0;
byte curType=0;
byte curType2=0;
float curVal=0;
float curVal2=0;
float helperVal=0;
float helperVal2=0;
unsigned long helperTime=0;
unsigned long helperTime2=0;
boolean helperflag=false;
boolean helperflag2=false;
unsigned long curTime=0;
unsigned long curTime2=0;

/*Profile declarations*/
const unsigned long profReceiveTimeout = 10000;
unsigned long profReceiveStart=0;
boolean receivingProfile=false;
const int nProfSteps = 15;
const int nProfSteps2 = 15;
byte proftypes[nProfSteps];
byte proftypes2[nProfSteps2];
unsigned long proftimes[nProfSteps];
unsigned long proftimes2[nProfSteps2];
float profvals[nProfSteps];
float profvals2[nProfSteps2];
boolean runningProfile = false;
boolean runningProfile2 = false;


void setup()
{
  TCCR1B = TCCR1B & 0b11111000 | 0x05;
  Serial.begin(9600);
  buttonTime=1;
  ioTime=5;
  serialTime=6;
  //windowStartTime=2;
  lcd.begin();
  lcd.keypadMode(1);
  lcd.backlight(250);
  lcd.contrast(20);

  lcd.setCursor(0,0);
  lcd.print(F(" Dual osPID   "));
  lcd.setCursor(0,1);
  lcd.print(F(" v1.00   "));
  delay(1000);

  initializeEEPROM();

  InitializeOutputCard();
  myPID.SetSampleTime(1000);
  myPID.SetOutputLimits(0, 255);
  myPID.SetTunings(kp, ki, kd);
  myPID.SetControllerDirection(ctrlDirection);
  myPID.SetMode(modeIndex);
  myPID2.SetSampleTime(1000);
  myPID2.SetOutputLimits(0, 255);
  myPID2.SetTunings(kp, ki, kd);
  myPID2.SetControllerDirection(ctrlDirection2);
  myPID2.SetMode(modeIndex2);

}

byte editDepth=0;
void loop()
{
  now = millis();

  bool doIO = now >= ioTime;
  //read in the input
  if(doIO)
  { 
    ioTime+=250;
    ReadInput();
    ReadInput2();
    if(!isnan(input))pidInput = input;
    if(!isnan(input2))pidInput2 = input2;

  }
  
  bool doButton = now >= buttonTime;
  
  if(doButton)  //read the button states
  {
    int dataByte = lcd.keypad();
    switch (databyte)
    {
      case 1:
        setpoint++;
        break;
      case 2:
        setpoint--;
        break;
      case 3:
        changeAutoTune();
        break;
      case 4:
        setpoint2++;
        break;
      case 5:
        setpoint2--;
        break;
      case 6:
        changeAutoTune2();
        break;
      case 7:
        if (!runningProfile)
        {
          StartProfile();
        }
        else
        {
          StopProfile();
        }
        break;
      case 8:
        if (!runningProfile2)
        {
          StartProfile2();
        }
        else
        {
          StopProfile2();
        }
        break;
      default:
        break;
    }
    buttonTime += 20;
  }

  if(tuning)
  {
    byte val = (aTune.Runtime());

    if(val != 0)
    {
      tuning = false;
    }

    if(!tuning)
    { 
      // We're done, set the tuning parameters
      kp = aTune.GetKp();
      ki = aTune.GetKi();
      kd = aTune.GetKd();
      myPID.SetTunings(kp, ki, kd);
      AutoTuneHelper(false);
      EEPROMBackupTunings();
    }
  }
  else
  {
    if(runningProfile) ProfileRunTime();
    //allow the pid to compute if necessary
    myPID.Compute();
  }

  if(tuning2)
  {
    byte val = (aTune2.Runtime());

    if(val != 0)
    {
      tuning2 = false;
    }

    if(!tuning2)
    { 
      // We're done, set the tuning parameters
      kp2 = aTune2.GetKp();
      ki2 = aTune2.GetKi();
      kd2 = aTune2.GetKd();
      myPID2.SetTunings(kp2, ki2, kd2);
      AutoTuneHelper2(false);
      EEPROMBackupTunings();
    }
  }
  else
  {
    if(runningProfile) ProfileRunTime();
    //allow the pid to compute if necessary
    myPID2.Compute();
  }

  drawLCD();

  if(doIO)
  {
    //send to output card
    WriteToOutput();
    WriteToOutput2();
  }

  if(millis() > serialTime)
  {
    //if(receivingProfile && (now-profReceiveStart)>profReceiveTimeout) receivingProfile = false;
    SerialReceive();
    SerialSend();
    serialTime += 500;
  }
}

void drawLCD()
{
  lcd.noCursor();
  lcd.home();
  lcd.print("A");
  lcd.print(input);
  lcd.print("  B");
  lcd.print(input2);
  lcd.setCursor(1,0);
  lcd.print(" ");
  lcd.print(setpoint);
  lcd.print("   ");
  lcd.print(setpoint2);
}

void ReadInput()
{
   double input = thermocouple.readThermocouple(CELSIUS);
   if (input==FAULT_OPEN || input==FAULT_SHORT_GND || input==FAULT_SHORT_VCC)
   {
     error = input;
     input = NAN;
   }
}

void ReadInput2()
{
   double input2 = thermocouple2.readThermocouple(CELSIUS);
   if (input2==FAULT_OPEN || input2==FAULT_SHORT_GND || input2==FAULT_SHORT_VCC)
   {
     error2 = input;
     input2 = NAN;
   }
}

void InitializeOutputCard()
{
  analogWrite(SSRPin, 0);
  analogWrite(SSRPin2, 0);
}

void WriteToOutput()
{
  analogWrite(SSRPin, output);
}

void WriteToOutput2()
{
  analogWrite(SSRPin2, output2);
}

void changeAutoTune()
{
  if(!tuning)
  {
    //initiate autotune
    AutoTuneHelper(true);
    aTune.SetNoiseBand(aTuneNoise);
    aTune.SetOutputStep(aTuneStep);
    aTune.SetLookbackSec((int)aTuneLookBack);
    tuning = true;
  }
  else
  { //cancel autotune
    aTune.Cancel();
    tuning = false;
    AutoTuneHelper(false);
  }
}

void changeAutoTune2()
{
  if(!tuning)
  {
    //initiate autotune
    AutoTuneHelper2(true);
    aTune2.SetNoiseBand(aTuneNoise);
    aTune2.SetOutputStep(aTuneStep);
    aTune2.SetLookbackSec((int)aTuneLookBack);
    tuning2 = true;
  }
  else
  { //cancel autotune
    aTune2.Cancel();
    tuning2 = false;
    AutoTuneHelper2(false);
  }
}

void AutoTuneHelper(boolean start)
{

  if(start)
  {
    ATuneModeRemember = myPID.GetMode();
    myPID.SetMode(MANUAL);
  }
  else
  {
    modeIndex = ATuneModeRemember;
    myPID.SetMode(modeIndex);
  } 
}

void AutoTuneHelper2(boolean start)
{

  if(start)
  {
    ATuneModeRemember2 = myPID2.GetMode();
    myPID2.SetMode(MANUAL);
  }
  else
  {
    modeIndex2 = ATuneModeRemember2;
    myPID.SetMode(modeIndex2);
  } 
}

void StartProfile()
{
  if(!runningProfile)
  {
    //initialize profle
    curProfStep=0;
    runningProfile = true;
    calcNextProf();
  }
}

void StartProfile2()
{
  if(!runningProfile2)
  {
    //initialize profle
    curProfStep2=0;
    runningProfile2 = true;
    calcNextProf2();
  }
}

void StopProfile()
{
  if(runningProfile)
  {
    curProfStep=nProfSteps;
    calcNextProf(); //runningProfile will be set to false in here
  } 
}

void StopProfile2()
{
  if(runningProfile2)
  {
    curProfStep2=nProfSteps2;
    calcNextProf2(); //runningProfile will be set to false in here
  } 
}

void ProfileRunTime()
{
  if(tuning || !runningProfile) return;
 
  boolean gotonext = false;

  //what are we doing?
  if(curType==1) //ramp
  {
    //determine the value of the setpoint
    if(now>helperTime)
    {
      setpoint = curVal;
      gotonext=true;
    }
    else
    {
      setpoint = (curVal-helperVal)*(1-(float)(helperTime-now)/(float)(curTime))+helperVal; 
    }
  }
  else if (curType==2) //wait
  {
    float err = input-setpoint;
    if(helperflag) //we're just looking for a cross
    {

      if(err==0 || (err>0 && helperVal<0) || (err<0 && helperVal>0)) gotonext=true;
      else helperVal = err;
    }
    else //value needs to be within the band for the perscribed time
    {
      if (abs(err)>curVal) helperTime=now; //reset the clock
      else if( (now-helperTime)>=curTime) gotonext=true; //we held for long enough
    }

  }
  else if(curType==3) //step
  {

    if((now-helperTime)>curTime)gotonext=true;
  }
  else if(curType==127) //buzz
  {
    if(now<helperTime)digitalWrite(buzzerPin,HIGH);
    else 
    {
       digitalWrite(buzzerPin,LOW);
       gotonext=true;
    }
  }
  else
  { //unrecognized type, kill the profile
    curProfStep=nProfSteps;
    gotonext=true;
  }
  if(gotonext)
  {
    curProfStep++;
    calcNextProf();
  }
}

void ProfileRunTime2()
{
  if(tuning2 || !runningProfile2) return;
 
  boolean gotonext2 = false;

  //what are we doing?
  if(curType2==1) //ramp
  {
    //determine the value of the setpoint
    if(now>helperTime2)
    {
      setpoint2 = curVal2;
      gotonext2=true;
    }
    else
    {
      setpoint2 = (curVal2-helperVal2)*(1-(float)(helperTime2-now)/(float)(curTime2))+helperVal2; 
    }
  }
  else if (curType2==2) //wait
  {
    float err2 = input2-setpoint2;
    if(helperflag2) //we're just looking for a cross
    {

      if(err2==0 || (err2>0 && helperVal2<0) || (err2<0 && helperVal2>0)) gotonext2=true;
      else helperVal2 = err2;
    }
    else //value needs to be within the band for the perscribed time
    {
      if (abs(err2)>curVal2) helperTime2=now; //reset the clock
      else if( (now-helperTime2)>=curTime2) gotonext2=true; //we held for long enough
    }

  }
  else if(curType2==3) //step
  {

    if((now-helperTime2)>curTime2)gotonext2=true;
  }
  else if(curType2==127) //buzz
  {
    if(now<helperTime2)digitalWrite(buzzerPin,HIGH);
    else 
    {
       digitalWrite(buzzerPin,LOW);
       gotonext2=true;
    }
  }
  else
  { //unrecognized type, kill the profile
    curProfStep2=nProfSteps2;
    gotonext2=true;
  }
  if(gotonext2)
  {
    curProfStep2++;
    calcNextProf2();
  }
}

void calcNextProf()
{
  if(curProfStep>=nProfSteps) 
  {
    curType=0;
    helperTime=0;
  }
  else
  { 
    curType = proftypes[curProfStep];
    curVal = profvals[curProfStep];
    curTime = proftimes[curProfStep];
  }
  if(curType==1) //ramp
  {
    helperTime = curTime + now; //at what time the ramp will end
    helperVal = setpoint;
  }   
  else if(curType==2) //wait
  {
    helperflag = (curVal==0);
    if(helperflag) helperVal= input-setpoint;
    else helperTime=now; 
  }
  else if(curType==3) //step
  {
    setpoint = curVal;
    helperTime = now;
  }
  else if(curType==127) //buzzer
  {
    helperTime = now + curTime;    
  }
  else
  {
    curType=0;
  }

  if(curType==0) //end
  { //we're done 
    runningProfile=false;
    curProfStep=0;
    Serial.println("P_DN");
    digitalWrite(buzzerPin,LOW);
  } 
  else
  {
    Serial.print("P_STP ");
    Serial.print(int(curProfStep));
    Serial.print(" ");
    Serial.print(int(curType));
    Serial.print(" ");
    Serial.print((curVal));
    Serial.print(" ");
    Serial.println((curTime));
  }
}

void calcNextProf2()
{
  if(curProfStep2>=nProfSteps2) 
  {
    curType2=0;
    helperTime2 =0;
  }
  else
  { 
    curType2 = proftypes2[curProfStep2];
    curVal2 = profvals2[curProfStep2];
    curTime2 = proftimes2[curProfStep2];
  }
  if(curType2==1) //ramp
  {
    helperTime2 = curTime2 + now; //at what time the ramp will end
    helperVal2 = setpoint2;
  }   
  else if(curType2==2) //wait
  {
    helperflag2 = (curVal2==0);
    if(helperflag2) helperVal2= input2-setpoint2;
    else helperTime2=now; 
  }
  else if(curType2==3) //step
  {
    setpoint2 = curVal2;
    helperTime2 = now;
  }
  else if(curType2==127) //buzzer
  {
    helperTime2 = now + curTime2;    
  }
  else
  {
    curType2=0;
  }

  if(curType2==0) //end
  { //we're done 
    runningProfile2=false;
    curProfStep2=0;
    Serial.println("P2_DN");
    digitalWrite(buzzerPin,LOW);
  } 
  else
  {
    Serial.print("P2_STP ");
    Serial.print(int(curProfStep2));
    Serial.print(" ");
    Serial.print(int(curType2));
    Serial.print(" ");
    Serial.print((curVal2));
    Serial.print(" ");
    Serial.println((curTime2));
  }
}



const int eepromTuningOffset = 1; //13 bytes
const int eepromATuneOffset = 14; //12 bytes
const int eepromTuningOffset2 = 26; //13 bytes
const int eepromATuneOffset2 = 39; //12 bytes
const int eepromProfileOffset = 51; //128 bytes
const int eepromProfileOffset2 = 179; //128 bytes


void initializeEEPROM()
{
  //read in eeprom values
  byte firstTime = EEPROM.read(0);
  if(firstTime!=EEPROM_ID)
  {//the only time this won't be 1 is the first time the program is run after a reset or firmware update
    //clear the EEPROM and initialize with default values
    for(int i=1;i<1024;i++) EEPROM.write(i,0);
    EEPROMBackupTunings();
    EEPROMBackupATune();
    EEPROMBackupProfile();
    EEPROM.write(0,EEPROM_ID); //so that first time will never be true again (future firmware updates notwithstanding)
  }
  else
  {
    EEPROMRestoreTunings();
    EEPROMRestoreATune();
    EEPROMRestoreProfile();    
  }
}  

void EEPROMreset()
{
  EEPROM.write(0,0);
}

void EEPROMBackupTunings()
{
  EEPROM.write(eepromTuningOffset,ctrlDirection);
  EEPROM_writeAnything(eepromTuningOffset+1,kp);
  EEPROM_writeAnything(eepromTuningOffset+5,ki);
  EEPROM_writeAnything(eepromTuningOffset+9,kd);
  EEPROM.write(eepromTuningOffset2,ctrlDirection2);
  EEPROM_writeAnything(eepromTuningOffset2+1,kp2);
  EEPROM_writeAnything(eepromTuningOffset2+5,ki2);
  EEPROM_writeAnything(eepromTuningOffset2+9,kd2);
}

void EEPROMRestoreTunings()
{
  ctrlDirection = EEPROM.read(eepromTuningOffset);
  EEPROM_readAnything(eepromTuningOffset+1,kp);
  EEPROM_readAnything(eepromTuningOffset+5,ki);
  EEPROM_readAnything(eepromTuningOffset+9,kd);
  ctrlDirection2 = EEPROM.read(eepromTuningOffset2);
  EEPROM_readAnything(eepromTuningOffset2+1,kp2);
  EEPROM_readAnything(eepromTuningOffset2+5,ki2);
  EEPROM_readAnything(eepromTuningOffset2+9,kd2);
}

void EEPROMBackupATune()
{
  EEPROM_writeAnything(eepromATuneOffset,aTuneStep);
  EEPROM_writeAnything(eepromATuneOffset+4,aTuneNoise);
  EEPROM_writeAnything(eepromATuneOffset+8,aTuneLookBack);
  EEPROM_writeAnything(eepromATuneOffset2,aTuneStep2);
  EEPROM_writeAnything(eepromATuneOffset2+4,aTuneNoise2);
  EEPROM_writeAnything(eepromATuneOffset2+8,aTuneLookBack2);
}

void EEPROMRestoreATune()
{
  EEPROM_readAnything(eepromATuneOffset,aTuneStep);
  EEPROM_readAnything(eepromATuneOffset+4,aTuneNoise);
  EEPROM_readAnything(eepromATuneOffset+8,aTuneLookBack);
  EEPROM_readAnything(eepromATuneOffset2,aTuneStep2);
  EEPROM_readAnything(eepromATuneOffset2+4,aTuneNoise2);
  EEPROM_readAnything(eepromATuneOffset2+8,aTuneLookBack2);
}

void EEPROMBackupProfile()
{
  EEPROM_writeAnything(eepromProfileOffset, proftypes);
  EEPROM_writeAnything(eepromProfileOffset + 16, profvals);
  EEPROM_writeAnything(eepromProfileOffset + 77, proftimes); //there might be a slight issue here (/1000?)
  EEPROM_writeAnything(eepromProfileOffset2, proftypes);
  EEPROM_writeAnything(eepromProfileOffset2 + 16, profvals);
  EEPROM_writeAnything(eepromProfileOffset2 + 77, proftimes); //there might be a slight issue here (/1000?)
}

void EEPROMRestoreProfile()
{
  EEPROM_readAnything(eepromProfileOffset, proftypes);
  EEPROM_readAnything(eepromProfileOffset + 16, profvals);
  EEPROM_readAnything(eepromProfileOffset + 77, proftimes); //there might be a slight issue here (/1000?)
  EEPROM_readAnything(eepromProfileOffset2, proftypes);
  EEPROM_readAnything(eepromProfileOffset2 + 16, profvals);
  EEPROM_readAnything(eepromProfileOffset2 + 77, proftimes); //there might be a slight issue here (/1000?)
}

/********************************************
 * Serial Communication functions / helpers
 ********************************************/

boolean ackDash = false, ackTune = false;
union {                // This Data structure lets
  byte asBytes[32];    // us take the byte array
  float asFloat[8];    // sent from processing and
}                      // easily convert it to a
foo;                   // float array

// getting float values from processing into the arduino
// was no small task.  the way this program does it is
// as follows:
//  * a float takes up 4 bytes.  in processing, convert
//    the array of floats we want to send, into an array
//    of bytes.
//  * send the bytes to the arduino
//  * use a data structure known as a union to convert
//    the array of bytes back into an array of floats
void SerialReceive()
{

  // read the bytes sent from Processing
  
  byte identifier=0;
  byte index=0;
  byte val=0,val2=0,val3=0;

  if(Serial.available())
  {
    byte identifier = Serial.read();
    byte val = Serial.read();
    byte val2 = Serial.read();
    while (Serial.available())
    {
      byte val3 = Serial.read();
      foo.asBytes[index] = val3;
      index++;
    }
    Serial.println("ready");
    switch(identifier)
    {
      case 0: //Settings Recieved 
        ReceiveSettings();
        break;
      case 1: //Tunings Recieved 
        ReceiveTunings();
        break;
      case 2: //autotune Recieved
        ReceiveAtune();
        break;
      case 3: //EEPROM reset
        if((val==9) && (val2==8))
        {
          EEPROMreset(); 
        }
        break;
      case 4:  //receiving profile
        ReceiveProfile();
        break;
      case 5:  //receiving profile2
        ReceiveProfile2();
        break;
      case 6: //profile command
        ProfileCommand();
        break;
      default:
        break;
    }
  }
}

void ReceiveSettings()
{
  setpoint=double(foo.asFloat[0]);
  setpoint2=double(foo.asFloat[1]);
  if(val==0)                              // * change PID1 mode to manual and set output 
  {
    myPID.SetMode(MANUAL);
    modeIndex=0;
    output=double(foo.asFloat[2]);
  }
  else 
  {
    myPID.SetMode(AUTOMATIC);
    modeIndex=1;
  }

  if(val2==0)                              // * change PID2 mode to manual and set output
  {
    myPID2.SetMode(MANUAL);
    modeIndex2=0;
    output2=double(foo.asFloat[3]);
  }
  else 
  {
    myPID2.SetMode(AUTOMATIC);
    modeIndex2=1;
  }
  sendDash = true;
}
  
void ReceiveTunings()
{
  kp = double(foo.asFloat[0]);
  kp2 = double(foo.asFloat[1]);
  ki = double(foo.asFloat[2]);
  ki2 = double(foo.asFloat[3]);
  kd = double(foo.asFloat[4]);
  kd2 = double(foo.asFloat[5]);
  ctrlDirection = val;
  ctrlDirection2 = val2;
  myPID.SetTunings(kp, ki, kd);
  myPID2.SetTunings(kp2, ki2, kd2);
  if(val==0) myPID.SetControllerDirection(DIRECT);
  else myPID.SetControllerDirection(REVERSE);
  if(val2==0) myPID.SetControllerDirection(DIRECT);
  else myPID.SetControllerDirection(REVERSE);
  EEPROMBackupTunings();
  sendTune = true;
}

void ReceiveAtune()
{
  aTuneStep = foo.asFloat[0];
  aTuneStep2 = foo.asFloat[1];
  aTuneNoise = foo.asFloat[2];
  aTuneNoise2 = foo.asFloat[3];  
  aTuneLookBack = (unsigned int)foo.asFloat[4];
  aTuneLookBack2 = (unsigned int)foo.asFloat[5];
  if((val==0 && tuning) || (val==1 && !tuning))
  { //toggle autotune state
    changeAutoTune();
  }
  if((val2==0 && tuning2) || (val2==1 && !tuning2))
  { //toggle autotune state
    changeAutoTune2();
  }
  EEPROMBackupATune();
  sendAtune = true;   
}

void ReceiveProfile()
{
  val=nProfSteps;
  receivingProfile=true;
  if(runningProfile)                          //stop the current profile execution
  {
    StopProfile();
  }
  if(!val2)                               // store profile receive start time
  {
    profReceiveStart = millis();
  }
  while(receivingProfile)
  {
    if((millis() - profReceiveStart) >= 1000) //there was a timeout issue.  reset this transfer
    {
      receivingProfile=false;
      Serial.println("ProfError");
      EEPROMRestoreProfile();
    }
    if(val2>=nProfSteps)                      // profile receive complete
    {
      receivingProfile=false;
      Serial.print("ProfDone ");              // acknowledge profile recieve completed
      EEPROMBackupProfile();
      Serial.println("Archived");             // acknowledge profile stored
    }
    else                                      // read in profile step values
    {
      profvals[val2] = foo.asFloat[0];
      proftimes[val2] = (unsigned long)(foo.asFloat[1] * 1000);
      Serial.print("ProfAck ");              // request next profile step values
    }
    
    byte index=0;                                 // read in next profile step bytes
    byte val2 = Serial.read();
    while (Serial.available())
    {
      byte val3 = Serial.read();
      foo.asBytes[index] = val3;
      index++;
    }
  }
}

void ReceiveProfile2()
{
  val=nProfSteps2;
  receivingProfile=true;
  if(runningProfile2)                          //stop the current profile execution
  {
    StopProfile2();
  }
  while(receivingProfile)
  {
    if((millis() - profReceiveStart) >= 1000) //there was a timeout issue.  reset this transfer
    {
      receivingProfile=false;
      Serial.println("ProfError");
      EEPROMRestoreProfile();
    }
    if(val2==0)                               // store profile receive start time
    {
      profReceiveStart = millis();
    }
    if(val2>=nProfSteps)                      // profile receive complete
    {
      receivingProfile=false;
      Serial.print("ProfDone ");              // acknowledge profile recieve completed
      EEPROMBackupProfile();
      Serial.println("Archived");             // acknowledge profile stored
    }
    else                                      // read in profile step values
    {
      profvals2[val2] = foo.asFloat[0];
      proftimes2[val2] = (unsigned long)(foo.asFloat[1] * 1000);
      Serial.print("ProfAck ");              // request next profile step values
    }
    
    byte index=0;                                 // read in next profile step bytes
    byte val2 = Serial.read();
    while (Serial.available())
    {
      byte val3 = Serial.read();
      foo.asBytes[index] = val3;
      index++;
    }
  }
}

void ProfileCommand()
{
  if(!val && !runningProfile) StartProfile();
  if(val && runningProfile) StopProfile();
  if(!val2 && !runningProfile2) StartProfile2();
  if(val2 && runningProfile2) StopProfile2();
}


// unlike our tiny microprocessor, the processing ap
// has no problem converting strings into floats, so
// we can just send strings.  much easier than getting
// floats from processing to here no?
void SerialSend()
{
  if(sendInfo)
  {//just send out the stock identifier
    Serial.print("\nDual osPID v1.0");
    Serial.println("");
    sendInfo = false; //only need to send this info once per request
  }
  if(sendDash)
  {
    Serial.print("DASH ");
    Serial.print(setpoint); 
    Serial.print(" ");
    Serial.print(setpoint2); 
    Serial.print(" ");
    if(isnan(input)) Serial.print(error);
    else Serial.print(input); 
    Serial.print(" ");
    if(isnan(input2)) Serial.print(error2);
    else Serial.print(input2); 
    Serial.print(" ");
    Serial.print(output); 
    Serial.print(" ");
    Serial.print(output2); 
    Serial.print(" ");
    Serial.print(myPID.GetMode());
    Serial.print(" ");
    Serial.print(myPID2.GetMode());
    Serial.print(" ");
    Serial.println(ackDash?1:0);
    if(sendDash)sendDash=false;
  }
  if(sendTune)
  {
    Serial.print("TUNE ");
    Serial.print(myPID.GetKp()); 
    Serial.print(" ");
    Serial.print(myPID2.GetKp()); 
    Serial.print(" ");
    Serial.print(myPID.GetKi()); 
    Serial.print(" ");
    Serial.print(myPID2.GetKi()); 
    Serial.print(" ");
    Serial.print(myPID.GetKd()); 
    Serial.print(" ");
    Serial.print(myPID2.GetKd()); 
    Serial.print(" ");
    Serial.print(myPID.GetDirection()); 
    Serial.print(" ");
    Serial.print(myPID2.GetDirection()); 
    Serial.print(" ");
    Serial.print(tuning?1:0);
    Serial.print(" ");
    if(sendTune)sendTune=false;
  }
  if(sendAtune)
  {
    Serial.print(aTuneStep); 
    Serial.print(" ");
    Serial.print(aTuneStep2); 
    Serial.print(" ");
    Serial.print(aTuneNoise); 
    Serial.print(" ");
    Serial.print(aTuneNoise2); 
    Serial.print(" ");
    Serial.print(aTuneLookBack); 
    Serial.print(" ");
    Serial.print(aTuneLookBack2); 
    Serial.print(" ");
    Serial.println(ackTune?1:0);
    if(sendAtune)sendAtune=false;
  }
  if(runningProfile)
  {
    Serial.print("PROF ");
    Serial.print(int(curProfStep));
    Serial.print(" ");
    Serial.print(int(curType));
    Serial.print(" ");
  }
  switch(curType)
  {
    case 1: //ramp
      Serial.println((helperTime-now)); //time remaining
      break;
    case 2: //wait
      Serial.print(abs(input-setpoint));
      Serial.print(" ");
      Serial.println(curVal==0? -1 : float(now-helperTime));
      break;  
    case 3: //step
      Serial.println(curTime-(now-helperTime));
      break;
    default: 
      break;
  }
}









