/////////////////////////////////////////////////////////////////////
// DIMITRI DISPLAY
// this is used to control the epaper shifter display
/////////////////////////////////////////////////////////////////////

// ISERIAL
#include "ISerial.h"
ISerial iSerial;
long tempLong;
float tempFloat;

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
int step = 0;

void setup()
{
    iSerial.init();
    iSerial.THIS_DEVICE_ID = 2; 
    setupDisplay();
    infoDisplay(targetGear, step);
    // gearDisplay(targetGear);
    //Serial.println("Setup complete");
    iSerial.setNewMode(Dimitri::Modes::ABORTING);
}

String stringInput = "";

bool infoUpdated = false;

void loop()
{

  if (iSerial.taskProcessUserInput())
  {
      handleSerialCmds();
  }

  switch (iSerial.status.mode)
  {
      case Dimitri::Modes::ABORTING:
          iSerial.setNewMode(Dimitri::Modes::IDLE);
          break;
      case Dimitri::Modes::RESETTING:
        if(infoUpdated)
        {
            infoDisplay(targetGear, step);
            infoUpdated = false;
        }
        break;
      case Dimitri::Modes::IDLE:
          if(infoUpdated)
          {
              //gearDisplay(targetGear);
              infoDisplay(targetGear, step);
              infoUpdated = false;
          }
          break;

      default:
          break;
  }


  // reset values at end of each loop, like OTEs and Event
  iSerial.event = 0;
}

// handles serial cmds that aren't already handled by ISerial (connect, mode, debug, maybe more?)
void handleSerialCmds()
{
  int idx = iSerial.idChr - '0';

  switch (iSerial.cmdChr)
  {
    case Cmds::ABSPOS_CMD: //P0
        break;

    case Cmds::ACC_SET: //A0
        processStepInfo();
        break;

    case Cmds::SERVOPOSINFO_CMD: //N0
        processGearInfo();
        break;

    case Cmds::SERIAL_OUTPUT: // prints serial information for use with a serial monitor, not to be used with high frequency (use INFO_CMD for that)
        iSerial.writeCmdChrIdChr();
        iSerial.writeNewline();
        Serial.print("iSerial.status.mode: ");
        Serial.println(iSerial.status.mode);
        break;

    case Cmds::PARAMS_SET: // set params
        // processParamsCmd();
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

void infoDisplay(int gear, int step)
{
    // char str[13][10] = {"0", "1", "2", "3", "4", "5", "6", "7", "8", "9", "10", "11", "12"};
    paint.Clear(UNCOLORED);
    
    paint.SetWidth(95);
    paint.SetHeight(80);
    String gearStr = String(gear);
    String stepStr = String(step);

    paint.DrawStringAt(0, 0, "G", &Font24, COLORED);
    paint.DrawStringAt(30, 0, gearStr.c_str(), &Font24, COLORED);
    
    paint.DrawStringAt(30, 30, stepStr.c_str(), &Font24, COLORED);
    paint.DrawStringAt(0, 30, "S", &Font24, COLORED);
    paint.SetRotate(ROTATE_0);
    epd.SetFrameMemoryPartial(paint.GetImage(), 0, 140, paint.GetWidth(), paint.GetHeight());
    epd.DisplayPartFrame();
}

void processGearInfo()
{
  int motor = iSerial.idChr - '0';
  if (!iSerial.parseLong(tempLong))
  {
    targetGear = tempLong;
    /*
    iSerial.writeCmdChrIdChr();
    iSerial.writeLong(tempLong);
    iSerial.writeNewline();
    */
    iSerial.debugPrintln("TargetGear: " + String(tempLong));
    //infoDisplay(targetGear, step);
    infoUpdated = true;
  }
  else
  {
    iSerial.writeCmdWarning("could not parse position data");
  }
}

void processStepInfo()
{
  int motor = iSerial.idChr - '0';
  if (!iSerial.parseLong(tempLong))
  {
    step = tempLong;
    /*
    iSerial.writeCmdChrIdChr();
    iSerial.writeLong(tempLong);
    iSerial.writeNewline();
    */
    iSerial.debugPrintln("Step: " + String(tempLong));
    infoUpdated = true;
    //infoDisplay(targetGear, step);
  }
  else
  {
    iSerial.writeCmdWarning("could not parse position data");
  }
}

void processUnrecognizedCmd()
{
  String msg1 = "didn't recognize cmdChr: ";
  msg1.concat(char(iSerial.cmdChr));
  iSerial.writeCmdWarning(msg1);
}