/////////////////////////////////////////////////////////////////////
// DIMITRI DISPLAY
// this is used to control the epaper shifter display
/////////////////////////////////////////////////////////////////////

// SERIAL BUFFER
long tempLong = 0;
char cmdChr = '0';
String readBuffer = "";
#define MAX_LENGTH 50
char readBufferArray[MAX_LENGTH + 1];

class Dimitri
{
public:
  // Game::Modes, these should match Game::Modes enum in the outerspace
  enum Modes : int32_t
  {
    ABORTING = -3,
    KILLED = -2,
    INACTIVE = 0,
    RESETTING = 50,
    IDLE = 100,
    HOMING = 200,
    RUNNING = 500,
    MANUAL = 1100,

  };

  enum Events : int
  {
    NONE = 0,
    ASTEROID_1_PASSED = 1,
  };
};
// EPAPER INCLUDES
#include <SPI.h>
#include "epd1in54_V2.h"
#include "imagedata.h"
#include "epdpaint.h"
#include <stdio.h>

// E-PAPER DISPLAY
Epd epd;
unsigned char image[1024];
Paint paint(image, 0, 0);

unsigned long time_start_ms;
unsigned long time_now_s;
#define COLORED 0
#define UNCOLORED 1

// INPUTS
int currentGear = 0;
int targetGear = 0;
bool isHomed = true;
bool isIdle = false;
bool atPos = false;
int homingState = 0;
int mode = 0;
int step = 0;
int error = 0;

void setup()
{
  //Serial.begin(115200);
  setupDisplay();
  infoDisplay(mode, step);
  gearDisplay(13);
  mode = Dimitri::Modes::ABORTING;
  Serial.println("Dimitri Display Setup Complete");
}

String stringInput = "";

bool infoUpdated = false;

void loop()
{

  taskProcessUserInput();

  switch (mode)
  {
  case Dimitri::Modes::ABORTING:
  case Dimitri::Modes::RESETTING:
  case Dimitri::Modes::KILLED:
  case Dimitri::Modes::INACTIVE:
  case Dimitri::Modes::HOMING:
    if (infoUpdated)
    {
      infoDisplay(mode, step);
    }
    break;
  case Dimitri::Modes::IDLE:
  case Dimitri::Modes::RUNNING:
    if (infoUpdated)
    {
      gearDisplay(targetGear);
    }
    break;
  default:
    break;
  }
  infoUpdated = false;
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

void gearDisplay(int gear)
{
  epd.DisplayPartBaseImage(IMAGE_DATA[gear % 13]);
}

void infoDisplay(int mode, int step)
{
  // char str[13][10] = {"0", "1", "2", "3", "4", "5", "6", "7", "8", "9", "10", "11", "12"};
  Serial.println("infoDisplay");
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

void infoDisplayOffset(const char *chr1, int val1, const char *chr2, int val2)
{
  Serial.println("infoDisplay");
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
    targetGear = tempLong;
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
    infoDisplayOffset("E", error, "E", error);
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