// DIMITRI SHIFTER


const bool CALIBRATE_HOME = false; // set to true when training home offset position

const int HOME_OFFSET = 0; // 


// EPAPER INCLUDES
#include <SPI.h>
#include "epd1in54_V2.h"
#include "imagedata.h"
#include "epdpaint.h"
#include <stdio.h>

// STEPPER INCLUDES
#include <AccelStepper.h>


// PIN DEFINITIONS
#define PIN_SHIFT_UP 8
#define PIN_SHIFT_DOWN 9
#define PIN_ENABLE 10
#define PIN_DIR 2
#define PIN_STEP 3
#define PIN_HOME 4

#define MICRO_STEP 2 // 1 = Full, 2 = Half, 4, 8, 16
#define STEPS_PER_REV 200*MICRO_STEP

#define MAX_RPM 2000 //adjust this value

#define MAX_VELOCITY round(double(MAX_RPM)/60.0)*STEPS_PER_REV
#define ACCEL_TIME_MS 50 //ADJUST THIS VALUE
#define MAX_ACCELERATION (MAX_VELOCITY/(ACCEL_TIME_MS))*1000
#define REVS_PER_INDEX 4 //ADJUST THIS TO MATCH WORM GEAR RATIO
#define STEPS_PER_INDEX STEPS_PER_REV*REVS_PER_INDEX
#define NUM_GEARS 12

// Define stepper motor object
AccelStepper stepper(AccelStepper::DRIVER, PIN_STEP, PIN_DIR);

// E-PAPER DISPLAY
Epd epd;
unsigned char image[1024];
Paint paint(image, 0, 0);

unsigned long time_start_ms;
unsigned long time_now_s;
#define COLORED 0
#define UNCOLORED 1

void setup() {
    Serial.begin(115200);

    // Set the maximum speed and acceleration
    stepper.setMaxSpeed(MAX_VELOCITY);
    stepper.setAcceleration(MAX_ACCELERATION);
    pinMode(PIN_ENABLE, OUTPUT);
    pinMode(PIN_SHIFT_UP, INPUT_PULLUP);         //  use a 10K resistor at ground to switch
    pinMode(PIN_SHIFT_DOWN, INPUT_PULLUP);         //  use a 10K resistor at ground to switch
    pinMode(PIN_HOME, INPUT);         //  use a 10K resistor at ground to switch
    disableMotor();
    setupDisplay();
}

// INPUTS
bool iShiftUpSw = false;
bool iShiftDownSw = false;
bool iHomeSw = false;

// SHIFTER VARIABLES

bool shiftUpReq = false;
bool shiftDownReq = false;

int currentGear = 0;
int targetGear = 0;
bool isHomed = false;
bool isIdle = false;


void updateIo(){
    iShiftUpSw = !digitalRead(PIN_SHIFT_UP);
    iShiftDownSw = !digitalRead(PIN_SHIFT_DOWN);
    iHomeSw = digitalRead(PIN_HOME);
}

bool updateGearNumber(){
    if (stepper.currentPosition()%STEPS_PER_INDEX == 0)
    {
        currentGear = stepper.currentPosition()/STEPS_PER_INDEX + 1;
        return true;
    }
    return false;
}

void loop() {

    updateIo();
    updateGearNumber();
    if(isHomed){
        if(!shiftUpReq && !shiftDownReq && iShiftUpSw && isHomed && targetGear < NUM_GEARS)
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

        if(!shiftUpReq && !shiftDownReq && iShiftDownSw && !iHomeSw && isHomed && targetGear > 1)
        {
            shiftDownReq = true;
            enableMotor();
            stepper.move(-STEPS_PER_INDEX);
            targetGear -= 1;
        }
        else if(!iShiftDownSw)
        {
            shiftDownReq = false;
        }

        // Wait until the motor reaches the target position
        if (stepper.distanceToGo() == 0) {
            if (!isIdle){
                disableMotor();
                isIdle = true;
            } else {
                isIdle = false;
            }
        }
    } else {
        homeMotor();
    }

    stepper.run();
}


int homingState = 0;
bool homeMotor(){
    if(iShiftDownSw || (homingState >= 10 && homingState < 100)){
        enableMotor();

    }
    else {
        disableMotor();
    }
    int prevHomingState = homingState;
    switch (homingState)
    {
        case 0:
            if (iHomeSw)
            {
                homingState = 5;
            } else {
                homingState = 1;
            }
        case 1:
            stepper.move(round(-STEPS_PER_INDEX/5));
            homingState = 2;
            break;
        case 2:
            if (iHomeSw)
            {
                homingState = 5;
            } else if (stepper.distanceToGo() == 0)
            {
                homingState = 1;
            }
            break;
        case 5:
            stepper.move(2);
            homingState = 6;
            break;
        case 6:
            if (!iHomeSw)
            {
                stepper.setCurrentPosition(0);
                stepper.stop();
                homingState = 7;
            } else if (stepper.distanceToGo() == 0)
            {
                homingState = 5;
            }
            break;
        case 7:
            stepper.moveTo(HOME_OFFSET);
            homingState = 8;
            break;
        case 8:
            if (stepper.distanceToGo() == 0)
            {
                targetGear = 1;
                updateGearNumber();
                homingState = 9;
                updateDisplay(1);
            }
            break;

        case 9:
            if(!iShiftDownSw){
                if(CALIBRATE_HOME){
                    homingState = 10;
                } else {
                    homingState = 100;
                }
            }
            break;

        case 10: //CALIBRATE HOME
            int dist = 2;
            if (iShiftDownSw){
                 stepper.move(-dist);
                 homingState = 11;
            } else if(iShiftUpSw){
                stepper.move(dist);
                homingState = 11;
            } 
            break;
        case 11:
            if (stepper.distanceToGo() == 0)
            {
                homingState = 10;
                Serial.println(stepper.currentPosition());
                textDisplay(String(stepper.currentPosition()).c_str());
            }
            break;
        
        case 100: // DONE
            isHomed = true;
            break;
        default:
            break;
    }

    if (prevHomingState != homingState)
    {
        Serial.print("Homing State: ");
        Serial.println(homingState);
    }
    return isHomed;

}

void enableMotor(){
    digitalWrite(PIN_ENABLE, LOW);
}

void disableMotor(){
    digitalWrite(PIN_ENABLE, HIGH);
}

void setupDisplay()
{
  Serial.println("e-Paper init and clear");
  epd.HDirInit();
  epd.Clear();
  paint.SetRotate(ROTATE_0);
  paint.SetWidth(80);
  paint.SetHeight(80);
  paint.Clear(UNCOLORED);
}


void updateDisplay(int imgIndex)
{
  epd.DisplayPartBaseImage(IMAGE_DATA[imgIndex % 13]);
}

void textDisplay(const char * str)
{
  //char str[13][10] = {"0", "1", "2", "3", "4", "5", "6", "7", "8", "9", "10", "11", "12"};
  paint.Clear(UNCOLORED);
  paint.DrawStringAt(0, 0, str, &Font24, COLORED);
  epd.SetFrameMemoryPartial(paint.GetImage(), 80, 80, paint.GetWidth(), paint.GetHeight());
  epd.DisplayPartFrame();
}