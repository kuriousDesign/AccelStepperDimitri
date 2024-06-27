/////////////////////////////////////////////////////////////////////////
// DIMITRI MOTION
/////////////////////////////////////////////////////////////////////////

// CONFIGURATION
const bool CALIBRATE_HOME = true; // set to true when training home offset position
const int HOME_OFFSET = 0;        //

// ISERIAL
#include "ISerial.h"
#include <Arduino.h> // Add this line to include the Arduino library

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

    enum Errors : int32_t
    {
        NONE = 0,
        POS_LIM_SW_ON_WHEN_NOT_AT_GEAR_12 = 1,
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

// const int MAX_VELOCITY = round(double(MAX_RPM)/60.0)*STEPS_PER_REV;
const int MAX_VELOCITY = 6000;
#define ACCEL_TIME_MS 10000            // ADJUST THIS VALUE
const int MAX_ACCELERATION = 6000 * 5; //  (MAX_VELOCITY/(ACCEL_TIME_MS))*1000;
#define REVS_PER_INDEX 10              // ADJUST THIS TO MATCH WORM GEAR RATIO
#define STEPS_PER_INDEX STEPS_PER_REV *REVS_PER_INDEX
#define NUM_GEARS 12

// Define stepper motor object
AccelStepper stepper(AccelStepper::DRIVER, PIN_STEP, PIN_DIR);

unsigned long time_start_ms;
unsigned long time_now_s;


void setup()
{

    iSerial.init();
    iSerial.THIS_DEVICE_ID = 1;

    Serial1.begin(115200);

    // Set the maximum speed and acceleration
    stepper.setMaxSpeed(MAX_VELOCITY);
    stepper.setAcceleration(MAX_ACCELERATION);
    pinMode(PIN_ENABLE, OUTPUT);
    pinMode(PIN_SHIFT_UP, INPUT_PULLUP);   //  use a 10K resistor at ground to switch
    pinMode(PIN_SHIFT_DOWN, INPUT_PULLUP); //  use a 10K resistor at ground to switch
    pinMode(PIN_POS_LIM, INPUT);           //  use a 10K resistor at ground to switch
    pinMode(PIN_CAM, INPUT);               //  use a 10K resistor at ground to switch

    enableMotor();

    iSerial.debug = true;
    iSerial.setNewMode(Dimitri::Modes::ABORTING);
    iSerial.debugPrintln("Setup complete");
}

// INPUTS
bool iShiftUpSw = false;
bool iShiftDownSw = false;
bool iPosLimSw = false;
bool iCamSw = false;

// SHIFTER VARIABLES

int currentGear = 0;
int targetGear = 0;
bool isHomed = false;
bool isIdle = false;

void updateIo()
{
    iShiftUpSw = !digitalRead(PIN_SHIFT_UP);
    iShiftDownSw = !digitalRead(PIN_SHIFT_DOWN);
    // iShiftDownSw = false; // TODO: remove this line
    if (iShiftUpSw)
    {
        // Serial.println("1");
    }
    else
    {
        // Serial.println("0");
    }

    iPosLimSw = !digitalRead(PIN_POS_LIM);
    iCamSw = !digitalRead(PIN_CAM);
}

bool atPos = false;

// returns true if at position and no steps left
bool updateGearNumber()
{
    // only updates the current gear if it is at the exact position
    if (stepper.currentPosition() % STEPS_PER_INDEX == 0)
    {
        currentGear = stepper.currentPosition() / STEPS_PER_INDEX;
        if (currentGear == targetGear && stepper.distanceToGo() == 0)
        {
            if (!atPos)
            {
                atPos = true;
                iSerial.debugPrintln("In Position at Gear: " + String(currentGear));
            }
            return true;
        }
    }
    atPos = false;
    return false;
}

int homeMotor(bool reset = false)
{
    static int homingState = 0;
    static int prevHomingState = -1;

    if (reset)
    {
        homingState = 0;
        prevHomingState = -1;
        disableMotor();
        // isHomed = false;
        // return homingState;
    }
    else if (!iShiftDownSw && homingState < 90 || homingState == 911)
    {
        disableMotor();
        // stepper.setMaxSpeed(MAX_VELOCITY/10);
        if (homingState != 1 && homingState != 0)
        {
            iSerial.debugPrintln("WARNING! user stopped holding down shift switch during homing, disabling motor and restarting sequence");
            homingState = 0;
        }
    }
    else
    {
        enableMotor();
    }

    switch (homingState)
    {
    case 0: // START HOMING
        iSerial.debugPrintln("HOMING STARTED: hold down shift switch to proceed");
        homingState = 1;
        break;

    case 1: // WAIT FOR DOWN SHIFT SW
        if (iShiftDownSw)
        {
            if (iPosLimSw)
            {
                iSerial.debugPrintln("Pos Lim Switch detected at homing start: moving backwards until sw is OFF");
                homingState = 5;
            }
            else
            {
                iSerial.debugPrintln("Pos Lim Switch not detected at homing start: moving forward until sw is ON");
                homingState = 10;
            }
        }
        break;
    case 5: // MOVING NEGATIVE UNTIL POS LIM SW IS OFF
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
        if (stepper.distanceToGo() == 0)
        {
            stepper.setMaxSpeed(MAX_VELOCITY / 5);
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
    case 20: // MOVING FORWARD UNTIL CAM SWITCH TURNS ON
        // Serial.println("Homing: step 5");
        if (stepper.distanceToGo() == 0)
        {
            stepper.setMaxSpeed(MAX_VELOCITY / 50);
            stepper.move(STEPS_PER_INDEX / 4);
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
    case 30: // CRAWLING BACKWARD UNTIL CAM SWITCH TURNS ON

        if (stepper.distanceToGo() == 0)
        {
            stepper.setMaxSpeed(MAX_VELOCITY / 50);
            stepper.move(-STEPS_PER_INDEX / 4);
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
    case 70: // FINISHING HOMING
        if (stepper.distanceToGo() == 0)
        {
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
    case 80: // INITIALIZE CURRENT AND TARGET GEAR NUMBER
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
        // isHomed = true;
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
            // textDisplay(String(stepper.currentPosition()).c_str());
        }
        break;

    case 911: // ERROR
        disableMotor();
        // isHomed = false;
        break;
    default:
        break;
    }

    if (prevHomingState != homingState)
    {
        sendStepInfo(homingState);
        iSerial.debugPrint("Homing State: ");
        iSerial.debugPrintln(String(homingState));
    }
    prevHomingState = homingState;
    return homingState;
}

void loop()
{
    static bool autoHomeAtPowerup = true;
    static int prevMode = -1;
    static int prevStep = -1;
    static int prevTargetGear = -1;

    updateIo();
    updateGearNumber();

    if (iSerial.taskProcessUserInput())
    {
        handleSerialCmds();
    }

    switch (iSerial.status.mode)
    {
    case Dimitri::Modes::ABORTING:
        disableMotor();
        iSerial.setNewMode(Dimitri::Modes::KILLED);
        break;
    case Dimitri::Modes::KILLED:
        disableMotor();
        iSerial.setNewMode(Dimitri::Modes::INACTIVE);
        break;
    case Dimitri::Modes::INACTIVE:
        if (autoHomeAtPowerup)
        {
            iSerial.debugPrintln("Auto homing at powerup");
            autoHomeAtPowerup = false;
            iSerial.setNewMode(Dimitri::Modes::HOMING);
        }
        break;
    case Dimitri::Modes::IDLE:
        disableMotor();
        if (checkGearChange())
        {
            enableMotor();
            stepper.moveTo(targetGear * STEPS_PER_INDEX);
            iSerial.setNewMode(Dimitri::Modes::RUNNING);
        }
        break;
    case Dimitri::Modes::RUNNING:
        enableMotor();
        if (checkGearChange())
        {
            stepper.moveTo(targetGear * STEPS_PER_INDEX);
        }
        else if (atPos)
        {
            iSerial.setNewMode(Dimitri::Modes::IDLE);
        }
        break;
    case Dimitri::Modes::HOMING:
        if (iSerial.status.step == 0)
        {
            isHomed = false;
            iSerial.status.step = homeMotor(true); // this should return a value of 1
            // iSerial.status.step = 1;
        }
        else
        {
            iSerial.status.step = homeMotor();
            if (iSerial.status.step == 100)
            {
                iSerial.setNewMode(Dimitri::Modes::IDLE);
            }
            else if (iSerial.status.step == 911)
            {
                iSerial.debugPrintln("ERROR! Homing failed, aborting");
                iSerial.setNewMode(Dimitri::Modes::ABORTING);
            }
        }

        break;
    case Dimitri::Modes::MANUAL:
        enableMotor();
        runVelocityUsingShifter();
        break;
    default:
        break;
    }

    iSerial.event = 0;

    stepper.run();

    // send status updates to the display device
    if (prevMode != iSerial.status.mode)
    {
        iSerial.debugPrintln("Mode: " + getModeString(iSerial.status.mode));
        sendModeInfo();
        prevMode = iSerial.status.mode;
    }
    if (prevStep != iSerial.status.step)
    {
        iSerial.debugPrintln("Step: " + String(iSerial.status.step));
        sendStepInfo(iSerial.status.step);
        prevStep = iSerial.status.step;
    }
    if (prevTargetGear != targetGear)
    {
        iSerial.debugPrintln("Target Gear: " + String(targetGear));
        sendGearInfo(targetGear);
        prevTargetGear = targetGear;
    }
}

// checks for gear change inputs and updates the target gear accordingly
bool checkGearChange()
{
    static bool shiftUpReq = false;
    static bool shiftDownReq = false;

    if (!shiftUpReq && !shiftDownReq && iShiftUpSw && targetGear < NUM_GEARS) // UP SHIFT - ONE SHOT
    {
        if(iPosLimSw)
        {
            iSerial.debugPrintln("ERROR! Pos Lim Switch is active when not at gear 12, cannot shift up");
            sendErrorInfo(Dimitri::Errors::POS_LIM_SW_ON_WHEN_NOT_AT_GEAR_12);
            iSerial.debugPrintln("auto aborting...");
            iSerial.setNewMode(Dimitri::Modes::ABORTING);
            return false;
        } 
        else {
            targetGear += 1;
            return true;
        }
        shiftUpReq = true;
    }
    else if (!iShiftUpSw)
    {
        shiftUpReq = false;
    }

    if (!shiftUpReq && !shiftDownReq && iShiftDownSw  && targetGear > 1) // DOWN SHIFT - ONE SHOT
    {
        shiftDownReq = true;
        targetGear -= 1;
        return true;
    }
    else if (!iShiftDownSw)
    {
        shiftDownReq = false;
    }

    // Wait until the motor reaches the target position
}

void enableMotor()
{
    digitalWrite(PIN_ENABLE, LOW);
}

void disableMotor()
{
    digitalWrite(PIN_ENABLE, HIGH);
}

void runVelocityUsingShifter()
{
    
    if (iShiftDownSw)
    {
        stepper.setSpeed(-MAX_VELOCITY / 100);
    }
    else if (iShiftUpSw)
    {
        stepper.setSpeed(MAX_VELOCITY / 100);
    }
    else
    {
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
    case Cmds::ABSPOS_CMD: // P0
        processGearChange();
        break;

    case Cmds::ACC_SET: // A0
        // processStepInfo();
        break;

    case Cmds::SERVOPOSINFO_CMD: // N0

        break;

    case Cmds::SERIAL_OUTPUT: // X0: prints serial information for use with a serial monitor, not to be used with high frequency (use INFO_CMD for that)
        // iSerial.writeCmdChrIdChr();
        // iSerial.writeNewline();
        Serial.print("Mode: ");
        Serial.println(getModeString(iSerial.status.mode));
        Serial.print("Step: ");
        Serial.println(iSerial.status.step);
        Serial.println("Current Position: " + String(stepper.currentPosition()));
        Serial.println("Current Gear: " + String(currentGear));
        Serial.println("Target Gear: " + String(targetGear));
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
    // int motor = iSerial.idChr - '0';
    if (!iSerial.parseLong(tempLong))
    {
        serialGearNumReq = tempLong;
    }
    else
    {
        iSerial.writeCmdWarning("could not parse position data");
    }
}

// send gear info to the display device
void sendGearInfo(long gear)
{
    Serial1.println("G" + String(gear));
}

// send step info to the display device
void sendStepInfo(long step)
{
    Serial1.println("S" + String(step));
}

// send step info to the display device
void sendModeInfo()
{
    Serial1.println("M" + String(iSerial.status.mode));
}

void sendErrorInfo(int errorCode)
{
    Serial1.println("E" + String(errorCode));
}

String getModeString(int mode)
{
    String modeString;
    switch (mode)
    {
    case Dimitri::Modes::ABORTING:
        modeString = "ABORTING";
        break;
    case Dimitri::Modes::KILLED:
        modeString = "KILLED";
        break;
    case Dimitri::Modes::INACTIVE:
        modeString = "INACTIVE";
        break;
    case Dimitri::Modes::RESETTING:
        modeString = "RESETTING";
        break;
    case Dimitri::Modes::IDLE:
        modeString = "IDLE";
        break;
    case Dimitri::Modes::HOMING:
        modeString = "HOMING";
        break;
    case Dimitri::Modes::RUNNING:
        modeString = "RUNNING";
        break;
    case Dimitri::Modes::MANUAL:
        modeString = "MANUAL";
        break;
    default:
        modeString = "UNKNOWN";
        break;
    }
    return modeString;
}