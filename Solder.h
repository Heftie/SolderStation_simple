#ifndef Solder_h
#define Solder_h

#define CS_PIN 10

#include "Arduino.h"
#include <Wire.h>
#include <LiquidCrystal_I2C.h>
#include <SPI.h>

class Solder
{
  public:
    Solder(LiquidCrystal_I2C *lcd);
    void showDisplay(int tipTempSet, int tipTempIs);
    void updateTemperature(int tipTempSet, int tipTempIs);
    void showInStand();
    void showSleep();
  private:
    LiquidCrystal_I2C *_lcd;
    int tipTempSetDisplay;
    int tipTempIsDisplay;
};

#endif
