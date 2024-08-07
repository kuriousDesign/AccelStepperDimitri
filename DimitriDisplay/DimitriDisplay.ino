/////////////////////////////////////////////////////////////////////
// DIMITRI DISPLAY
// this is used to control the epaper shifter display

// TODO: cannot use Serial with certain part of ePaper library, can use disrete i/o for comms
/////////////////////////////////////////////////////////////////////

const bool DEMO_MODE = true; //this is the normal case

// SERIAL BUFFER
long tempLong = 0;
char cmdChr = '0';
String readBuffer = "";
#define MAX_LENGTH 50
char readBufferArray[MAX_LENGTH + 1];

#include "Dimitri.h"
// EPAPER INCLUDES
#include <SPI.h>
#include "epd1in54_V2.h"
#include "imagedata.h"
#include "epdpaint.h"
#include <stdio.h>

/*
// Pin definition
#define RST_PIN         8   // WHITE
#define DC_PIN          9   // GREEN
#define CS_PIN          10  // ORANGE
#define BUSY_PIN        7   // PURPLE
#define PWR_PIN         6   // GREY - JUST PLUG INTO 5V
*/
#define PIN_BIT0 22 // NOTE THAT PINS 22, 24, 26 & 28 ARE USED AS INPUTS FOR GEAR NUMBER
#define NUM_BITS 4


// E-PAPER DISPLAY
Epd epd;
unsigned char image[1024];
Paint paint(image, 0, 0);

unsigned long time_start_ms;
unsigned long time_now_s;
#define COLORED 0
#define UNCOLORED 1

// INPUTS
int prevGear = 0;
int displayGear = 0;
bool isHomed = true;
bool isIdle = false;
bool atPos = false;
int homingState = 0;
int mode = 0;
int step = 0;
int error = 0;

void setup()
{
  setupDisplay();
  mode = Dimitri::Modes::ABORTING;

  if(!DEMO_MODE){
    Serial.begin(115200); // the serial interferes with the epaper display's ability to display images
    showInfo(mode, step);
    Serial.println("Dimitri Display Setup Complete");
  } else {
    showGearImage(0);
  }
  

  // INPUTS
  for (int i = 0; i < NUM_BITS; i++)
  {
    pinMode(2*i+PIN_BIT0, INPUT);
  }
}

String stringInput = "";

bool infoUpdated = false;

void loop()
{
  if (DEMO_MODE)
  {
    // 1. READ THE DIGITAL INPUTS AND CALCULATE THE GEAR NUMBER
    displayGear = getGearNumber();
    //displayGear++;
    //displayGear = getGearNumberTest(displayGear%13);

    // 2. CHECK IF GEAR NUMBER CHANGE, IF SO THEN UPDATE THE DISPLAY
    if(displayGear != prevGear){
      showGearImage(displayGear);
      prevGear = displayGear;
    }
    delay(5);
  }
  else{
    
    taskProcessUserInput();

    if (infoUpdated){
      showInfo(mode, step);
    }

    switch (mode)
    {
    case Dimitri::Modes::ABORTING:
    case Dimitri::Modes::RESETTING:
    case Dimitri::Modes::KILLED:
    case Dimitri::Modes::INACTIVE:
    case Dimitri::Modes::HOMING:
      if (infoUpdated)
      {
        //showInfo(mode, step);
      }
      break;
    case Dimitri::Modes::IDLE:
    case Dimitri::Modes::RUNNING:
      if (infoUpdated)
      {
        //showGearImage(displayGear);
      }
      break;
    default:
      break;
    }
    infoUpdated = false;
  }
}

// handles serial cmds that aren't already handled by ISerial (connect, mode, debug, maybe more?)
void handleSerialCmds()
{
  switch (cmdChr)
  {
  case 'M':
    processModeInfo();
    break;

  case 'S':
    processStepInfo();
    break;

  case 'G':
    processGearInfo();
    break;

  case 'E':
    processErrorInfo();
    break;

  default:
    processUnrecognizedCmd();
    break;
  }
}

void setupDisplay()
{
  Serial.println("e-Paper init and clear");
  epd.HDirInit();
  epd.Clear();
  paint.SetRotate(ROTATE_0);
  paint.SetWidth(95);
  paint.SetHeight(80);
  paint.Clear(UNCOLORED);
}

void showGearImage(int gear)
{
  if (gear < 0 || gear > 12)
  {
    gear = 0;
  }
  epd.DisplayPartBaseImage(IMAGE_DATA[gear]);
}

void showError(int mode, int step)
{
  // char str[13][10] = {"0", "1", "2", "3", "4", "5", "6", "7", "8", "9", "10", "11", "12"};
  Serial.println("showInfo");
  paint.Clear(UNCOLORED);

  paint.SetWidth(95);
  paint.SetHeight(80);
  String modeStr = String(mode);
  String stepStr = String(step);

  paint.DrawStringAt(0, 0, "M", &Font24, COLORED);
  paint.DrawStringAt(30, 0, modeStr.c_str(), &Font24, COLORED);

  paint.DrawStringAt(30, 30, stepStr.c_str(), &Font24, COLORED);
  paint.DrawStringAt(0, 30, "E", &Font24, COLORED);

  paint.SetRotate(ROTATE_0);
  epd.SetFrameMemoryPartial(paint.GetImage(), 0, 140, paint.GetWidth(), paint.GetHeight());
  epd.DisplayPartFrame();
}

void showInfo(int mode, int step)
{
  // char str[13][10] = {"0", "1", "2", "3", "4", "5", "6", "7", "8", "9", "10", "11", "12"};
  Serial.println("showInfo");
  paint.Clear(UNCOLORED);

  paint.SetWidth(95);
  paint.SetHeight(80);
  String modeStr = String(mode);
  String stepStr = String(step);

  paint.DrawStringAt(0, 0, "M", &Font24, COLORED);
  paint.DrawStringAt(30, 0, modeStr.c_str(), &Font24, COLORED);

  paint.DrawStringAt(30, 30, stepStr.c_str(), &Font24, COLORED);
  paint.DrawStringAt(0, 30, "S", &Font24, COLORED);

  paint.SetRotate(ROTATE_0);
  epd.SetFrameMemoryPartial(paint.GetImage(), 0, 140, paint.GetWidth(), paint.GetHeight());
  epd.DisplayPartFrame();
}

void showInfoOffset(const char *chr1, int val1, const char *chr2, int val2)
{
  Serial.println("showInfo");
  paint.Clear(UNCOLORED);

  paint.SetWidth(95);
  paint.SetHeight(80);
  String val1Str = String(val1);
  String val2Str = String(val2);

  paint.DrawStringAt(0, 0, chr1, &Font24, COLORED);
  paint.DrawStringAt(30, 0, val1Str.c_str(), &Font24, COLORED);

  paint.DrawStringAt(30, 30, val2Str.c_str(), &Font24, COLORED);
  paint.DrawStringAt(0, 30, chr2, &Font24, COLORED);
  paint.SetRotate(ROTATE_0);
  epd.SetFrameMemoryPartial(paint.GetImage(), 0, 140, paint.GetWidth(), paint.GetHeight());
  epd.DisplayPartFrame();
}

void processGearInfo()
{

  if (!parseLong(readBuffer, tempLong))
  {
    displayGear = tempLong;
    infoUpdated = true;
  }
  else
  {
    Serial.println("could not parse position data");
  }
}

void processModeInfo()
{
  if (!parseLong(readBuffer, tempLong))
  {
    mode = tempLong;
    Serial.print("new mode data: ");
    Serial.println(mode);
    infoUpdated = true;
  }
  else
  {
    Serial.println("could not parse position data");
  }
}

void processStepInfo()
{
  if (!parseLong(readBuffer, tempLong))
  {
    step = tempLong;
    infoUpdated = true;
  }
  else
  {
    Serial.println("could not parse position data");
  }
}

void processErrorInfo()
{
  if (!parseLong(readBuffer, tempLong))
  {
    error = tempLong;
    showInfoOffset("E", error, "E", error);
  }
  else
  {
    Serial.println("could not parse position data");
  }
}

void processUnrecognizedCmd()
{
  String msg1 = "didn't recognize cmdChr: ";
  Serial.println(msg1 + String(cmdChr));
  // msg1.concat(char(iSerial.cmdChr));
  // iSerial.writeCmdWarning(msg1);
}

void taskProcessUserInput()
{
  if (readIncomingData()) // reads input stream if data available, returns true if delimiter is found
  {
    Serial.println("handleSerialCmds()");
    handleSerialCmds();
  }
}

bool readIncomingData()
{
  readBuffer = "";
  readBufferArray[0] = '\0';
  static int idx = 0;
  if (Serial.available())
  {
    delay(1); // let the buffer fill
    idx = 0;
    while (Serial.available())
    {
      char c = char(Serial.read());

      if (c == '\n')
      {
        if (idx >= 2)
        {
          cmdChr = readBufferArray[0];
          readBuffer.remove(0, 1);
          Serial.println(readBuffer);
          return true;
        }
        else
        {
          return false;
        }
      }
      else
      {
        readBufferArray[idx] = c;
        readBuffer += c;
        idx++;
      }
    }
  }
  return false;
}

bool parseLong(String myString, long &myNumber)
{

  // Convert the String to a char array
  char myCharArray[myString.length() + 1];
  myString.toCharArray(myCharArray, myString.length() + 1);

  // Parse the char array to a long
  char *endptr;
  myNumber = strtol(myCharArray, &endptr, 10); // Base 10

  // Check if the entire string was parsed
  if (*endptr != '\0')
  {
    // Serial.println("The input string was not a valid number.");
    return true;
  }
  else
  {
    return false;
  }
}

int convertBoolArrayToInt(bool boolArray[4])
{
  int result = 0;
  for (int i = 0; i < 4; i++)
  {
    if (boolArray[i])
    {
      result |= (1 << i);
    }
  }
  return result;
}

int getGearNumber()
{
  int result = 0;
  for (int i = 0; i < NUM_BITS; i++)
  {
    if (digitalRead(2*i + PIN_BIT0))
    {
      result |= (1 << i);
    }
  }
  return result;
}

////////////////////////////////////////////////////////////////////////////////////////////////////////////
// JUST USED FOR TESTING

// sets the digital outputs for the gear number to be received by the display device
bool fakeIo[4] = {false,false,false,false};
void updateFakeGearNumberDigitalOutputs(int num){
    for (int i = 0; i < NUM_BITS; i++) {
        bool val = (num >> i) & 1;
        fakeIo[i] = val;
    }
}

int getGearNumberTest(int num)
{
  updateFakeGearNumberDigitalOutputs(num);
  int result = 0;
  for (int i = 0; i < NUM_BITS; i++)
  {
    if (fakeIo[i])
    {
      result |= (1 << i);
    }
  }
  return result;
}