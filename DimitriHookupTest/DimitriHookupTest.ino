/////////////////////////////////////////////////////////////////////////
// DIMITRI MOTION
/////////////////////////////////////////////////////////////////////////

// CONFIGURATION
const bool CALIBRATE_HOME = true; // set to true when training home offset position
const int HOME_OFFSET = 0;        //

// ISERIAL

#include <Arduino.h> // Add this line to include the Arduino library

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

// CNC SHIELD INFO
/*
X+_LIMIT_PIN = 9;
Y+_LIMIT_PIN = 10;
Z+_LIMIT_PIN = 11;
SPIN_EN = 12;
SPIN_DIR = 13;
*/

// PIN DEFINITIONS
#define PIN_STEP 4 // labelled as Z_STEP_PIN on Shield
#define PIN_DIR 7  // labelled as Z_DIR_PIN on Shield
#define PIN_ENABLE 8
#define PIN_SHIFT_UP 9    // labelled as X_LIMIT_PIN on Shield, ORANGE WIRE on Shifter
#define PIN_SHIFT_DOWN 10 // labelled as Y_LIMIT_PIN on Shield, BROWN WIRE on Shifter
#define PIN_POS_LIM 11    // labelled as Z_LIMIT_PIN on Shield
#define PIN_CAM 12        // labelled as SPIN_EN on Shield

const int MICRO_STEP = 1; // 1 = Full, 2 = Half, 4, 8, 16
const int STEPS_PER_REV = 200 * MICRO_STEP;

//const int MAX_RPM = 1000; // adjust this value

// const int MAX_VELOCITY = round(double(MAX_RPM)/60.0*STEPS_PER_REV);
const int MAX_VELOCITY = 1800 * MICRO_STEP;
const int MAX_ACCELERATION = MAX_VELOCITY*10 ; //  (MAX_VELOCITY/(ACCEL_TIME_MS))*1000;
const int REVS_PER_INDEX = 10;                 // ADJUST THIS TO MATCH WORM GEAR RATIO
const int STEPS_PER_INDEX = STEPS_PER_REV * REVS_PER_INDEX;
const int NUM_GEARS = 12;

// Define stepper motor object
AccelStepper stepper(AccelStepper::DRIVER, PIN_STEP, PIN_DIR);

unsigned long time_start_ms;
unsigned long time_now_s;

void setup()
{

    // Set the maximum speed and acceleration
    Serial.begin(115200);
    stepper.setMaxSpeed(MAX_VELOCITY);
    stepper.setAcceleration(MAX_ACCELERATION);
    stepper.setPinsInverted(true,false,false);
    pinMode(PIN_ENABLE, OUTPUT);
    pinMode(PIN_SHIFT_UP, INPUT_PULLUP); //  use a 10K resistor at ground to switch
    pinMode(PIN_SHIFT_DOWN, INPUT_PULLUP); //  use a 10K resistor at ground to switch
    pinMode(PIN_POS_LIM, INPUT_PULLUP);    //  use a 10K resistor at ground to switch
    pinMode(PIN_CAM, INPUT_PULLUP); //  use a 10K resistor at ground to switch

    disableMotor();
    enableMotor();
    //delay(200);
    stepper.move(-STEPS_PER_INDEX);

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

int step = 0;

void loop()
{
    if (digitalRead(PIN_CAM) == LOW)
    {
        Serial.println("ON");
    }
    else
    {
        //iShiftUpSw = false;
    }
    
    stepper.run();
    // delayMicroseconds(1000);
    if(stepper.distanceToGo() == 0){
        //Serial.println(millis()-time_start_ms);
        //delay(1000);
        //stepper.move(STEPS_PER_INDEX);
        time_start_ms = millis();
        
    }
}

void enableMotor()
{
    digitalWrite(PIN_ENABLE, LOW);
    // delay(500);
}

void disableMotor()
{
    digitalWrite(PIN_ENABLE, HIGH);
    // delay(50);
}