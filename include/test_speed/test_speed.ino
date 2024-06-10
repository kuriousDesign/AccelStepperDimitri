
// Display Library example for SPI e-paper panels from Dalian Good Display and boards from Waveshare.
// Requires HW SPI and Adafruit_GFX. Caution: these e-papers require 3.3V supply AND data lines!
//
// based on Demo Example from Good Display: http://www.good-display.com/download_list/downloadcategoryid=34&isMode=false.html
//
// Author: Jean-Marc Zingg
//
// Version: see library.properties
//
// Library: https://github.com/ZinggJM/GxEPD2_AVR

// Supporting Arduino Forum Topics:
// Waveshare e-paper displays with SPI: http://forum.arduino.cc/index.php?topic=487007.0
// Good Dispay ePaper for Arduino : https://forum.arduino.cc/index.php?topic=436411.0

// mapping suggestion from Waveshare SPI e-Paper to Wemos D1 mini
// BUSY -> D2, RST -> D4, DC -> D3, CS -> D8, CLK -> D5, DIN -> D7, GND -> GND, 3.3V -> 3.3V

// mapping suggestion from Waveshare SPI e-Paper to generic ESP8266
// BUSY -> GPIO4, RST -> GPIO2, DC -> GPIO0, CS -> GPIO15, CLK -> GPIO14, DIN -> GPIO13, GND -> GND, 3.3V -> 3.3V

// mapping suggestion for ESP32, e.g. LOLIN32, see .../variants/.../pins_arduino.h for your board
// NOTE: there are variants with different pins for SPI ! CHECK SPI PINS OF YOUR BOARD
// BUSY -> 4, RST -> 16, DC -> 17, CS -> SS(5), CLK -> SCK(18), DIN -> MOSI(23), GND -> GND, 3.3V -> 3.3V

// new mapping suggestion for STM32F1, e.g. STM32F103C8T6 "BluePill"
// BUSY -> A1, RST -> A2, DC -> A3, CS-> A4, CLK -> A5, DIN -> A7

// mapping suggestion for AVR, UNO, NANO etc.
// BUSY -> 7, RST -> 9, DC -> 8, CS-> 10, CLK -> 13, DIN -> 11

#include <GxEPD2_AVR_BW.h>
//#include <GxEPD2_AVR_3C.h>
#include <Fonts/FreeMonoBold9pt7b.h>

// select only the bitmaps for your display to reduce code space used, or de-select all
#include "bitmaps/Bitmaps200x200.h" // 1.54" b/w
//#include "bitmaps/Bitmaps128x250.h" // 2.13" b/w
//#include "bitmaps/Bitmaps128x296.h" // 2.9"  b/w
//#include "bitmaps/Bitmaps176x264.h" // 2.7"  b/w
//#include "bitmaps/Bitmaps400x300.h" // 4.2"  b/w // does not work (code size)
//#include "bitmaps/Bitmaps640x384.h" // 7.5"  b/w // does not work (code size)
// 3-color
//#include "bitmaps/Bitmaps3c200x200.h" // 1.54" b/w/r
//#include "bitmaps/Bitmaps3c104x212.h" // 2.13" b/w/r
//#include "bitmaps/Bitmaps3c128x296.h" // 2.9"  b/w/r
//#include "bitmaps/Bitmaps3c176x264.h" // 2.7"  b/w/r
//#include "bitmaps/Bitmaps3c400x300.h" // 4.2"  b/w/r


// select one and adapt to your mapping
GxEPD2_AVR_BW display(GxEPD2::GDEP015OC1, /*CS=*/ 10, /*DC=*/ 9, /*RST=*/ 8, /*BUSY=*/ 7);
//GxEPD2_AVR_BW display(GxEPD2::GDE0213B1, /*CS=*/ SS, /*DC=*/ 9, /*RST=*/ 8, /*BUSY=*/ 7);
//GxEPD2_AVR_BW display(GxEPD2::GDE0213B1, /*CS=*/ SS, /*DC=*/ 8, /*RST=*/ 9, /*BUSY=*/ 7);
//GxEPD2_AVR_BW display(GxEPD2::GDEH029A1, /*CS=*/ SS, /*DC=*/ 8, /*RST=*/ 9, /*BUSY=*/ 7);
//GxEPD2_AVR_BW display(GxEPD2::GDEW027W3, /*CS=*/ SS, /*DC=*/ 8, /*RST=*/ 9, /*BUSY=*/ 7);
//GxEPD2_AVR_BW display(GxEPD2::GDEW042T2, /*CS=*/ SS, /*DC=*/ 8, /*RST=*/ 9, /*BUSY=*/ 7);
//GxEPD2_AVR_BW display(GxEPD2::GDEW075T8, /*CS=*/ SS, /*DC=*/ 8, /*RST=*/ 9, /*BUSY=*/ 7);
// 3-color e-papers
//GxEPD2_AVR_3C display(GxEPD2::GDEW0154Z04, /*CS=*/ SS, /*DC=*/ 8, /*RST=*/ 9, /*BUSY=*/ 7);
//GxEPD2_AVR_3C display(GxEPD2::GDEW0213Z16, /*CS=*/ SS, /*DC=*/ 8, /*RST=*/ 9, /*BUSY=*/ 7);
//GxEPD2_AVR_3C display(GxEPD2::GDEW029Z10, /*CS=*/ SS, /*DC=*/ 8, /*RST=*/ 9, /*BUSY=*/ 7);
//GxEPD2_AVR_3C display(GxEPD2::GDEW027C44, /*CS=*/ SS, /*DC=*/ 8, /*RST=*/ 9, /*BUSY=*/ 7);
//GxEPD2_AVR_3C display(GxEPD2::GDEW042Z15, /*CS=*/ SS, /*DC=*/ 8, /*RST=*/ 9, /*BUSY=*/ 7);
void setup()
{
  display.init();
  helloWorld();
}

void loop()
{

  helloWorld1();
  delay(50);
  helloWorld2();
  helloWorld3();
}

void helloWorld()
{
  display.setFullWindow();
  display.setRotation(2);
  display.setFont(&FreeMonoBold9pt7b);
  display.setTextColor(GxEPD_BLACK);
  display.firstPage();
  do
  {
    display.fillScreen(GxEPD_WHITE);
    display.setCursor(15, 20);
    display.println(F("AAAAAAAAAA"));
    display.setCursor(15, 40);
    display.println(F("BBBBBBBBBB"));
    display.setCursor(15, 60);
    display.println(F("CCCCCCCCCC"));
    display.setCursor(15, 80);
    display.println(F("DDDDDDDDDD"));
    display.setCursor(15, 100);
    display.println(F("EEEEEEEEEE"));
  }
  while (display.nextPage());
}

void helloWorld1()
{
  //display.setFullWindow();
  display.setPartialWindow(0, 0, display.width(), display.height());
  display.setRotation(2);
  display.setFont(&FreeMonoBold9pt7b);
  display.setTextColor(GxEPD_BLACK);
  display.firstPage();
  do
  {
    display.fillScreen(GxEPD_WHITE);
    display.setCursor(15, 20);
    display.println(F("AAAAAAAAAA"));
    display.setCursor(15, 40);
    display.println(F("BBBBBBBBBB"));
    display.setCursor(15, 60);
    display.println(F("CCCCCCCCCC"));
    display.setCursor(15, 80);
    display.println(F("DDDDDDDDDD"));
    display.setCursor(15, 100);
    display.println(F("EEEEEEEEEE"));
  }
  while (display.nextPage());
}

void helloWorld2()
{
  display.setRotation(2);
  display.setFont(&FreeMonoBold9pt7b);
  display.setTextColor(GxEPD_BLACK);
  display.setPartialWindow(0, 20, 128, 25); // x,y,width,height
  display.firstPage();
  do
  {
    display.fillScreen(GxEPD_WHITE);
    //display.hasFastPartialUpdate(); // is a read method
    display.setCursor(15, 40);
    display.println(F("0000000000"));
    delay(50);
  }
  while (display.nextPage());
}


void helloWorld3()
{
  display.setRotation(2);
  display.setFont(&FreeMonoBold9pt7b);
  display.setTextColor(GxEPD_BLACK);
  display.setPartialWindow(0, 90, 128, 25); // x,y,width,height
  display.firstPage();
  do
  {
    display.fillScreen(GxEPD_WHITE);
    //display.hasFastPartialUpdate(); // is a read method
    display.setCursor(15, 100);
    display.println(F("1111111111"));
    delay(50);
  }
  while (display.nextPage());
}