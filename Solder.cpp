#include "Arduino.h"
#include <Wire.h>
#include <LiquidCrystal_I2C.h>
#include <SPI.h>
#include "Solder.h"

//Constructor
Solder::Solder(LiquidCrystal_I2C *lcd) {
  _lcd = lcd;
  tipTempSetDisplay = 0;
  tipTempIsDisplay = 0;
}

void Solder::showDisplay(int tipTempSet, int tipTempIs) {
  _lcd->clear();
  _lcd->setCursor(0, 0);
  _lcd->print("Temp ");
  _lcd->setCursor(8, 0);
  _lcd->print("\337C/");
  _lcd->setCursor(14, 0);
  _lcd->print("\337C");
  char buffer[4];
  if (tipTempIs > 999) tipTempIs = 999;
  sprintf(buffer, "%3i", tipTempIs);
  _lcd->setCursor(5, 0);
  _lcd->print(buffer);
  tipTempIsDisplay = tipTempIs;
  sprintf(buffer, "%3i", tipTempSet);
  _lcd->setCursor(11, 0);
  _lcd->print(buffer);
  tipTempSetDisplay = tipTempSet;
}

//updates only the Temperature
void Solder::updateTemperature(int tipTempSet, int tipTempIs) {
  char buffer[4];
  if (tipTempIs != tipTempIsDisplay) {
    if (tipTempIs > 999) tipTempIs = 999;
    sprintf(buffer, "%3i", tipTempIs);
    _lcd->setCursor(5, 0);
    _lcd->print(buffer);
    tipTempIsDisplay = tipTempIs;
  }
  if (tipTempSet != tipTempSetDisplay) {
    sprintf(buffer, "%3i", tipTempSet);
    _lcd->setCursor(11, 0);
    _lcd->print(buffer);
    tipTempSetDisplay = tipTempSet;
  }
}

// show that Iron is in Stand
void Solder::showInStand(){
  _lcd->setCursor(0, 1);
  _lcd->print("Iron hibernation");
}

// show that Station enters Sleep
void Solder::showSleep(){
  _lcd->setCursor(0, 1);
  _lcd->print("Sleep Mode      ");
}

