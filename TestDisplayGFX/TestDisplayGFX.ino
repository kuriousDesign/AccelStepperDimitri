#include <GxEPD2_BW.h>
#include <GxEPD2_3C.h>
#include <Adafruit_GFX.h> // Library for graphics support
#include "imagedata.h"

// Select the display type (GxEPD2_154) and assign the correct pin numbers
GxEPD2_BW<GxEPD2_154, GxEPD2_154::HEIGHT> display(GxEPD2_154(/*CS=*/ 5, /*DC=*/ 6, /*RST=*/ 7, /*BUSY=*/ 8));


void setup() {
  Serial.begin(115200);
  display.init();
  display.setRotation(0); // Set the rotation if necessary
  display.setFullWindow();

  display.firstPage();
  do {
    display.fillScreen(GxEPD_WHITE); // Clear the display
    display.drawBitmap(0, 0, IMAGE_DATA[0], 200, 200, GxEPD_BLACK);
  } while (display.nextPage());

  Serial.println("Image displayed!");
}

void loop() {
    static int i = 0;
    displayImage(i%13);
    delay(1000);
    i++;
}


void displayImage(int imgIndex) {
  display.firstPage();
  do {
    display.fillScreen(GxEPD_WHITE); // Clear the display
    //display.drawBitmap(0, 0, IMAGE_DATA[imgIndex], 200, 200, GxEPD_BLACK);
  } while (display.nextPage());
}