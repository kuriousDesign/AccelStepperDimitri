/* 

DIMITRI SHIFTER

Example sketch to control a stepper motor with 
   A4988/DRV8825 stepper motor driver and 
   Arduino without a library. 
   More info: https://www.makerguides.com
   
    ADDED IN SHIFTING ACTUATION
    */

// Define stepper motor connections and steps per revolution:
#include <AccelStepper.h>
#define dirPin 2
#define stepPin 3
#define microStep 2 // 1 = Full, 2 = Half, 4, 8, 16
#define stepsPerRevolution 200*microStep

#define MAX_RPM 2000 //adjust this value

#define MAX_VELOCITY round(double(MAX_RPM)/60.0)*stepsPerRevolution
#define ACCEL_TIME_MS 50 //ADJUST THIS VALUE
#define MAX_ACCELERATION (MAX_VELOCITY/(ACCEL_TIME_MS))*1000
#define REVS_PER_INDEX 4 //ADJUST THIS TO MATCH WORM GEAR RATIO
#define INDEX_DISTANCE stepsPerRevolution*REVS_PER_INDEX

// Define stepper motor object
AccelStepper stepper(AccelStepper::DRIVER, stepPin, dirPin);


#define PIN_SHIFT_UP 8
#define PIN_SHIFT_DOWN 9
#define PIN_ENABLE 10

void setup() {
   

    // Set the maximum speed and acceleration
    stepper.setMaxSpeed(MAX_VELOCITY);
    stepper.setAcceleration(MAX_ACCELERATION);

    digitalWrite(Enable, HIGH);

    Serial.begin(9600);
    pinMode(PIN_ENABLE, OUTPUT);
    pinMode(PIN_SHIFT_UP, INPUT);         //  use a 10K resistor at ground to switch
    pinMode(PIN_SHIFT_DOWN, INPUT);         //  use a 10K resistor at ground to switch
}

//******************

bool shiftUpReq = false;
bool shiftDownReq = false;

void loop() {

    if(!shiftUpReq && !shiftDownReq && digitalRead(PIN_SHIFT_UP))
    {
        shiftUpReq = true;
        digitalWrite(Enable, LOW);
        stepper.move(INDEX_DISTANCE);
    }
    else if (!digitalRead(PIN_SHIFT_UP))
    {
        shiftUpReq = false;
    }

    if(!shiftUpReq && !shiftDownReq && digitalRead(PIN_SHIFT_DOWN))
    {
        shiftDownReq = true;
        digitalWrite(Enable, LOW);
        stepper.move(-INDEX_DISTANCE);
    }
    else if(!digitalRead(PIN_SHIFT_DOWN))
    {
        shiftDownReq = false;
    }

    stepper.run();

    // Wait until the motor reaches the target position
    if (stepper.distanceToGo() == 0) {
        digitalWrite(Enable, HIGH);
    }
}