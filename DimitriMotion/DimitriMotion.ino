/////////////////////////////////////////////////////////////////////////
// DIMITRI MOTION
/////////////////////////////////////////////////////////////////////////


// CONFIGURATION
const bool CALIBRATE_HOME = true; // set to true when training home offset position
const int HOME_OFFSET = 0; //

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

// STEPPER INCLUDES
#include <AccelStepper.h>

// PIN DEFINITIONS
#define PIN_SHIFT_UP 30
#define PIN_SHIFT_DOWN 32
#define PIN_ENABLE 26
#define PIN_DIR 2
#define PIN_STEP 3
#define PIN_POS_LIM 4
#define PIN_CAM 5

#define MICRO_STEP 2 // 1 = Full, 2 = Half, 4, 8, 16
#define STEPS_PER_REV 200 * MICRO_STEP

#define MAX_RPM 1000 // adjust this value

//const int MAX_VELOCITY = round(double(MAX_RPM)/60.0)*STEPS_PER_REV;
const int MAX_VELOCITY = 6000;
#define ACCEL_TIME_MS 10000 //ADJUST THIS VALUE
const int MAX_ACCELERATION = 6000*5;//  (MAX_VELOCITY/(ACCEL_TIME_MS))*1000;
#define REVS_PER_INDEX 10 //ADJUST THIS TO MATCH WORM GEAR RATIO
#define STEPS_PER_INDEX STEPS_PER_REV*REVS_PER_INDEX
#define NUM_GEARS 12

// Define stepper motor object
AccelStepper stepper(AccelStepper::DRIVER, PIN_STEP, PIN_DIR);

unsigned long time_start_ms;
unsigned long time_now_s;
bool infoUpdated = false;

void setup()
{
    //Serial.begin(115200);
    iSerial.init();
    iSerial.THIS_DEVICE_ID = 1; 

    // Set the maximum speed and acceleration
    stepper.setMaxSpeed(MAX_VELOCITY);
    stepper.setAcceleration(MAX_ACCELERATION);
    pinMode(PIN_ENABLE, OUTPUT);
    pinMode(PIN_SHIFT_UP, INPUT_PULLUP);   //  use a 10K resistor at ground to switch
    pinMode(PIN_SHIFT_DOWN, INPUT_PULLUP); //  use a 10K resistor at ground to switch
    pinMode(PIN_POS_LIM, INPUT);       //  use a 10K resistor at ground to switch
    pinMode(PIN_CAM, INPUT);       //  use a 10K resistor at ground to switch

    enableMotor();
    //Serial.println("Setup complete");
    iSerial.debug = true;
    iSerial.setNewMode(Dimitri::Modes::ABORTING);
}

// INPUTS
bool iShiftUpSw = false;
bool iShiftDownSw = false;
bool iPosLimSw = false;
bool iCamSw = false;

// SHIFTER VARIABLES

bool shiftUpReq = false;
bool shiftDownReq = false;

int currentGear = 0;
int targetGear = 0;
bool isHomed = true;
bool isIdle = false;




void updateIo()
{
    iShiftUpSw = !digitalRead(PIN_SHIFT_UP);
    iShiftDownSw = !digitalRead(PIN_SHIFT_DOWN);
    //iShiftDownSw = false; // TODO: remove this line
    if (iShiftUpSw)
    {
        //Serial.println("1");
    }
    else
    {
        //Serial.println("0");
    }

    iPosLimSw = !digitalRead(PIN_POS_LIM);
    iCamSw = !digitalRead(PIN_CAM);
}

bool atPos = false;

bool updateGearNumber()
{
    if (stepper.currentPosition() % STEPS_PER_INDEX == 0)
    {
        currentGear = stepper.currentPosition() / STEPS_PER_INDEX + 1;
        if (currentGear == targetGear)
        {
            if (!atPos)
            {
                atPos = true;
                // do some more display here
            }
        }
        return true;
    }
    atPos = false;
    return false;
}

void loop()
{
    updateIo();
    updateGearNumber();

    if (iSerial.taskProcessUserInput())
    {
        handleSerialCmds();
    }

    switch (iSerial.status.mode)
    {
        case Dimitri::Modes::ABORTING:
            iSerial.setNewMode(Dimitri::Modes::HOMING);
            break;
        case Dimitri::Modes::IDLE:
            if (!shiftUpReq && !shiftDownReq && iShiftUpSw && isHomed && targetGear < NUM_GEARS)
            {
                shiftUpReq = true;
                enableMotor();
                stepper.move(STEPS_PER_INDEX);
                targetGear += 1;
            }
            else if (!iShiftUpSw)
            {
                shiftUpReq = false;
            }

            if (!shiftUpReq && !shiftDownReq && iShiftDownSw && !iPosLimSw && isHomed && targetGear > - NUM_GEARS)
            {
                shiftDownReq = true;
                enableMotor();
                stepper.move(-STEPS_PER_INDEX);
                targetGear -= 1;
            }
            else if (!iShiftDownSw)
            {
                shiftDownReq = false;
            }

            // Wait until the motor reaches the target position
            if (stepper.distanceToGo() == 0)
            {
                if (!isIdle)
                {
                    disableMotor();
                    isIdle = true;
                }
                else
                {
                    isIdle = false;
                }
            }
            break;
        case Dimitri::Modes::HOMING:
            homeMotor();
            break;
        case Dimitri::Modes::MANUAL:
            runVelocityUsingShifter();
            break;  
        default:
            break;
    }

    iSerial.event = 0;
    stepper.run();
}


int homingState = 0;
int prevHomingState = -1;
bool homeMotor()
{
    if (iShiftDownSw || (homingState >= 90 && homingState < 911))
    {
        enableMotor();
    }
    else
    {
        disableMotor();
        //stepper.setMaxSpeed(MAX_VELOCITY/10);
        if (homingState != 0)
        {
            iSerial.debugPrintln("Instant Kill Condition: user stop holding down shift switch during homing");
        }
        homingState = 0;
    }
    
    switch (homingState)
    {
    case 0:
        
        if (!iShiftDownSw){
            iSerial.debugPrintln("HOMING STARTED: hold down shift switch to proceed");
            homingState = 0;
        }
        else if (iPosLimSw)
        {
            iSerial.debugPrintln("Pos Lim Switch detected at homing start: moving backwards until sw is OFF");
            homingState = 5;
        }
        else
        {
            iSerial.debugPrintln("Pos Lim Switch not detected at homing start: moving forward until sw is ON");
            homingState = 10;
        }
        break;

    case 5: //MOVING NEGATIVE UNTIL POS LIM SW IS OFF
        stepper.move(-STEPS_PER_INDEX);
        homingState = 6;
        break;
    case 6:
        if (!iPosLimSw)
        {
            iSerial.debugPrintln("Pos Lim Switch is Now off: proceeding to normal procedure");
            stepper.stop();
            homingState = 10;
        }
        else if (stepper.distanceToGo() == 0)
        {
            iSerial.debugPrintln("ERROR! Expected the Pos Lim switch to turn off before move finished");
            homingState = 911;
        }
        break;
    case 10: // MOVING FORWARD UNTIL POS LIM SWITCH IS ON
        if (stepper.distanceToGo() == 0){
            stepper.setMaxSpeed(MAX_VELOCITY/5);
            stepper.move(STEPS_PER_INDEX * NUM_GEARS);
            homingState = 11;
        }
        break;
    case 11:
        if (iPosLimSw)
        {
            iSerial.debugPrintln("Pos Lim switch detected: slowly moving forward until cam switch turns on");
            stepper.stop();
            homingState = 20;
        }
        else if (stepper.distanceToGo() == 0)
        {
            iSerial.debugPrintln("ERROR! Expected the Pos Lim switch to turn on before move finished");
            homingState = 911;
        }
        break;
    case 20: //MOVING FORWARD UNTIL CAM SWITCH TURNS ON
        //Serial.println("Homing: step 5");
        if (stepper.distanceToGo() == 0){
            stepper.setMaxSpeed(MAX_VELOCITY/50);
            stepper.move(STEPS_PER_INDEX/4);
            homingState = 21;
        }
        break;
    case 21:
        if (iCamSw)
        {
            iSerial.debugPrintln("Cam switch detected: crawling backward until cam switch turns off");
            stepper.stop();
            homingState = 30;
        }
        else if (stepper.distanceToGo() == 0)
        {
            iSerial.debugPrintln("ERROR! Expected the Cam switch to turn on before move finished");
            homingState = 911;
        }
        break;
    case 30: //CRAWLING BACKWARD UNTIL CAM SWITCH TURNS ON
        
        if (stepper.distanceToGo() == 0){
            stepper.setMaxSpeed(MAX_VELOCITY/50);
            stepper.move(-STEPS_PER_INDEX/4);
            homingState = 31;
        }
        break;
    case 31:
        if (!iCamSw)
        {
            iSerial.debugPrint("Cam switch turned off: ");
            stepper.setCurrentPosition(0);
            stepper.stop();
            if (CALIBRATE_HOME)
            {
                iSerial.debugPrintln("calibrating home is active: use up and down shifter to move motor");
                homingState = 110;
            }
            else
            {
                iSerial.debugPrintln("moving to home offset position");
                homingState = 70;
            }
   
        }
        else if (stepper.distanceToGo() == 0)
        {
            iSerial.debugPrintln("ERROR! Expected the Cam switch to turn off before move finished");
            homingState = 911;
        }
        break;
    case 70: //FINISHING HOMING
        if (stepper.distanceToGo() == 0){
            stepper.moveTo(HOME_OFFSET);
            homingState = 80;
        }
        break;
    case 71:
        if (stepper.distanceToGo() == 0)
        {
            
            homingState = 80;
        }
        break;
    case 80: //INITIALIZE CURRENT AND TARGET GEAR NUMBER
        stepper.setCurrentPosition(STEPS_PER_INDEX * NUM_GEARS);
        targetGear = NUM_GEARS;
        updateGearNumber();
        iSerial.debugPrintln("HOMING IS FINISHED! Release the down shift switch to continue");
        homingState = 90;
        break;

    case 90:
        if (!iShiftDownSw)
        {
            homingState = 100;
        }
        break;

    case 100: // DONE
        stepper.setMaxSpeed(MAX_VELOCITY);
        isHomed = true;
        break;

    case 110: // CALIBRATE HOME
        int dist = 2;
        if (iShiftDownSw)
        {
            stepper.move(-dist);
            homingState = 111;
        }
        else if (iShiftUpSw)
        {
            stepper.move(dist);
            homingState = 111;
        }
        break;
    case 111:
        if (stepper.distanceToGo() == 0)
        {
            homingState = 110;
            iSerial.debugPrint("Current Position: ");
            iSerial.debugPrintln(String(stepper.currentPosition()));
            //textDisplay(String(stepper.currentPosition()).c_str());
        }
        break;


    case 911: // ERROR
        disableMotor();
        isHomed = false;
        break;
    default:
        break;
    }

    if (prevHomingState != homingState)
    {
        if(!iSerial.debug){
            sendStepInfo(homingState);
        } 
        iSerial.debugPrint("Homing State: ");
        iSerial.debugPrintln(String(homingState));

    }
    prevHomingState = homingState;
    return isHomed;
}

void enableMotor()
{
    digitalWrite(PIN_ENABLE, LOW);
}

void disableMotor()
{
    digitalWrite(PIN_ENABLE, HIGH);
}

void runVelocityUsingShifter(){
    if (iShiftDownSw)
    {
        stepper.setSpeed(-MAX_VELOCITY/100);
    }
    else if (iShiftUpSw)
    {
        stepper.setSpeed(MAX_VELOCITY/100);

    }
    else {
        stepper.stop();
    }
}


/////////////////////////////////////////////////////////////////////////////////////////
// ISERIAL HELPERS
/////////////////////////////////////////////////////////////////////////////////////////

// handles serial cmds that aren't already handled by ISerial (connect, mode, debug, maybe more?)
void handleSerialCmds()
{
  int idx = iSerial.idChr - '0';

  switch (iSerial.cmdChr)
  {
    case Cmds::ABSPOS_CMD: //P0
        processGearChange();
        break;

    case Cmds::ACC_SET: //A0
        //processStepInfo();
        break;

    case Cmds::SERVOPOSINFO_CMD: //N0
        
        break;

    case Cmds::SERIAL_OUTPUT: // X0: prints serial information for use with a serial monitor, not to be used with high frequency (use INFO_CMD for that)
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

void processUnrecognizedCmd()
{
  String msg1 = "didn't recognize cmdChr: ";
  msg1.concat(char(iSerial.cmdChr));
  iSerial.writeCmdWarning(msg1);
}

int serialGearNumReq = 0;
void processGearChange()
{
  //int motor = iSerial.idChr - '0';
  if (!iSerial.parseLong(tempLong))
  {
    serialGearNumReq = tempLong;
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

void sendGearInfo(long gear)
{
    iSerial.writeString("N0");
    //iSerial.writeCmdChrIdChr();
    iSerial.writeLong(gear);
    iSerial.writeNewline();
}

void sendStepInfo(long step)
{
    iSerial.writeString("A0");
    //iSerial.writeCmdChrIdChr();
    iSerial.writeLong(step);
    iSerial.writeNewline();
}