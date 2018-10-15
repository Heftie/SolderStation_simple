#define PHASE_A     (PINC & 1<<PC1)
#define PHASE_B     (PINC & 1<<PC2)
#define HEAT_ON     PORTD |= (1<<PD5)
#define HEAT_OFF    PORTD &= ~(1<<PD5)
#define IRON_DEC    !(PIND & (1<< PD6))
#define HEAT_PIN 5
#define IRON_DEC_PIN 6
#define ZERO_DEC_PIN 2
#define ENC_PUSH_PIN 3
#define ENC_A_PIN A1
#define ENC_B_PIN A2
#define MOSI_PIN 11
#define MISO_PIN 12
#define SCLK_PIN 13
#define CS_PIN 10

#define DEBUG
// Sleept time is created with the 100hz interruptsignal from mains so 100 ticks are 1 sec
// 10 min = 10 x 60 x 100 
#define SLEEP_TIME ((unsigned long) 60000)
#define MAX_TEMP 400
#define T_SETTLE 23
#define T_MAX_HEAT 25
#define ESUM_MAX 750

#define KI  7// shift div

#include <Wire.h>
#include <LiquidCrystal_I2C.h>
#include <SPI.h>
#include "Solder.h"

// Set the LCD address to 0x3F for a 16 chars and 2 line display
LiquidCrystal_I2C lcd(0x3F, 16, 2);

Solder solder( &lcd );

//Var for ISR
volatile uint8_t enc_last = PHASE_A;
volatile int8_t enc_delta = 0;
volatile uint8_t encPush = 0;
volatile int8_t last = 0;
volatile int mainsCycles = 0;

//global var
int sel = 0;
int tipTempIs = 0;
int tipTempSet = 0;
int tipTempSetStore = 0;
int heatTime = 0;
int esum = 0;
uint8_t tempReached = 0;
uint8_t ironIsInStand = 0;
unsigned long mainsCyclesCnt = 0;

//func prototyps
void encPushInterrupt();
void zeroCrossingInterrupt();
float readTempSens(void);

void setup()
{
  //encoder Pins
  pinMode(ENC_PUSH_PIN, INPUT_PULLUP);
  pinMode(ENC_A_PIN, INPUT_PULLUP);
  pinMode(ENC_B_PIN, INPUT_PULLUP);
  //Iron Pins
  pinMode(HEAT_PIN, OUTPUT);
  pinMode(IRON_DEC_PIN, INPUT);
  pinMode(ZERO_DEC_PIN, INPUT);
  digitalWrite(HEAT_PIN, LOW);
  //SPI
  pinMode(CS_PIN, OUTPUT);
  digitalWrite(CS_PIN, HIGH);
  SPI.begin();
  tipTempIs = (int) readTempSens();
#ifdef DEBUG
  //Serial for Debug
  Serial.begin(38400);
#endif
  // initialize the LCD
  lcd.begin();
  lcd.backlight();
  delay(100);
  solder.showDisplay(tipTempSet, tipTempIs);
  // Pin change Interrupt Encoder
  cli();
  PCICR = (1 << PCIE1);
  PCMSK1 = (1 << PCINT9) | (1 << PCINT10);
  sei();
  //ISR INT0/INT1
  attachInterrupt(digitalPinToInterrupt(ZERO_DEC_PIN), zeroCrossingInterrupt, RISING);
  attachInterrupt(digitalPinToInterrupt(ENC_PUSH_PIN), encPushInterrupt, FALLING);
#ifdef DEBUG
  Serial.println("Start up");
#endif
}

void loop()
{
  // At 0 turn off heater
  if (mainsCycles >= 0) HEAT_OFF;
  // Wait for T_SETTLE more mains cycles to get an undisturbed reading
  if (mainsCycles > T_SETTLE) {
    noInterrupts();
    //averageing Readings
    tipTempIs += (int) readTempSens();
    tipTempIs = tipTempIs >> 1;
    mainsCyclesCnt += mainsCycles;
    mainsCycles = 0;
    //Overheat Protection
    if ( tipTempIs > MAX_TEMP + 20) {
      HEAT_OFF;
      tipTempSet = 0;
      lcd.setCursor(0, 1);
      lcd.print("Overheated!");
    }
    if ( tipTempSet != 0) {
      if (tempReached) {
        esum += tipTempSet - tipTempIs;
        if (esum > ESUM_MAX) esum = ESUM_MAX;
        if (esum < (- ESUM_MAX)) esum = - ESUM_MAX;
        if (tipTempIs > tipTempSet + 5) esum -= 50;
        if (tipTempIs > tipTempSet + 10) esum -= 25;
      } else {
        esum = 0;
        if (tipTempIs > tipTempSet - 5) tempReached = 1;
      }
      // Schedule next measurement
      heatTime = sqrt(tipTempSet - tipTempIs) + 1;
      if (tempReached) {
        // extra heat via integration
        heatTime += (esum >> KI);
      } else {
        // tbd 1 or 2 or 3
        heatTime += 1;
      }
      //Overshoot Protection
      if (heatTime > T_MAX_HEAT) heatTime = T_MAX_HEAT;
      // set Heating
      if (heatTime > 0) {
        mainsCycles = - heatTime;
        mainsCyclesCnt += - mainsCycles;
        HEAT_ON;
      } else {
        mainsCycles = heatTime;
        mainsCyclesCnt += - mainsCycles;
        HEAT_OFF;
      }
#ifdef DEBUG
      Serial.print(heatTime * 10);
      Serial.print(" ");
      Serial.print(esum);
      Serial.print(" ");
      Serial.println(tipTempIs);
#endif
    }
    else {
      HEAT_OFF;
    }
    interrupts();
  }

  //encoder rotate
  if (enc_delta != 0) {
    if (ironIsInStand) {
      if (enc_delta > 0) {
        if ((tipTempSetStore + 5) < MAX_TEMP + 1) {
          tipTempSetStore += 5;
        }
      }
      else {
        if ((tipTempSetStore - 5) >= 0) {
          tipTempSetStore -= 5;
          if (tipTempSetStore == 0) {
            tipTempSet = 0;
          }
        }
      }
    } else {
      if (enc_delta > 0) {
        if ((tipTempSet + 5) < MAX_TEMP + 1) {
          tipTempSet += 5;
          tempReached = 0;
        }
      }
      else {
        if ((tipTempSet - 5) >= 0) {
          tipTempSet -= 5;
          tempReached = 0;
        }
      }
    }
    mainsCyclesCnt = 0;
    enc_delta = 0;
  }

  //encoder push
  if (encPush >= 1) {
    if (ironIsInStand) {
      // Disable heat even in station
      if (tipTempSetStore > 0) {
        tipTempSetStore = 0;
        tipTempSet = 0;
      }
      else {
        tipTempSetStore = 350;
      }
      solder.showDisplay(tipTempSetStore, tipTempIs);
      solder.showInStand();
    } else {
      if (tipTempSet > 0) {
        tipTempSet = 0;
        tempReached = 0;
      }
      else {
        tipTempSet = 350;
        tempReached = 0;
      }
      solder.showDisplay(tipTempSet, tipTempIs);
    }
    mainsCyclesCnt = 0;
    encPush = 0;
  }

  //Stand detection
  if (IRON_DEC && (tipTempSet != 0) && (ironIsInStand == 0)) {
    tipTempSetStore = tipTempSet;
    tipTempSet = 180;
    mainsCyclesCnt = 0;
    ironIsInStand = 1;
  } else {
    //restore previous tempSet
    if ((IRON_DEC == 0) && (ironIsInStand == 1 )) {
      tipTempSet = tipTempSetStore;
      tipTempSetStore = 0;
      ironIsInStand = 0;
      tempReached = 0;
      mainsCyclesCnt = 0;
      solder.showDisplay(tipTempSet, tipTempIs);
    }
  }

  //SLEEP if not used
  if ( (mainsCyclesCnt > SLEEP_TIME) && (tipTempSet != 0)) {
    tipTempSet = 0;
    tipTempSetStore = 0;
    tempReached = 0;
    ironIsInStand = 0;
    solder.showSleep();
  }

  //Display Update for in Stand and not
  if (ironIsInStand) {
    //update Temperatures but Set Temp is tipTempSetStore
    solder.updateTemperature(tipTempSetStore, tipTempIs);
    solder.showInStand();
  } else {
    //update Temperatures
    solder.updateTemperature(tipTempSet, tipTempIs);
  }
}


ISR(PCINT1_vect) {
  // check if Pin A is HIGH and if Pin A was HIGH last interrupt
  if ( PHASE_A && enc_last) {
    // check Pin B is HIGH or LOW which gives the direction
    if ( PHASE_B ) {
      //left
      enc_delta--;
    } else {
      //right
      enc_delta++;
    }
  }
  enc_last = PHASE_A;
}

void zeroCrossingInterrupt() {
  mainsCycles++;
}
void encPushInterrupt() {
  encPush += 1;
}

float readTempSens(void) {
  uint16_t data = 0;
  digitalWrite(CS_PIN, LOW);
  cli();
  SPI.beginTransaction(SPISettings(4000000, MSBFIRST, SPI_MODE0));  // Defaults
  data = (SPI.transfer(0x00) << 8);
  data |= SPI.transfer(0x00);
  SPI.endTransaction();
  digitalWrite(CS_PIN, HIGH);
  sei();
  // thermocoupler not detected
  if (data & 0x4) {
#ifdef DEBUG
    Serial.println("no thermocoupler");
#endif
    return (float) 0.0;
  }
  data = data >> 3;
  // Code dosent work for neg temp duen to no checking the MSBit(sign bit)
  //LSBit == 0.25 Celsius
  return (float) data * 0.37; // measured offset
}

