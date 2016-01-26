/**
 * (C) Copyright 2016 Smart Homebrew by Antonio Marin Neto
* Licensed under the Apache License, Version 2.0 (the "License");
* you may not use this file except in compliance with the License.
* You may obtain a copy of the License at
*
* http://www.apache.org/licenses/LICENSE-2.0
*
* Unless required by applicable law or agreed to in writing, software
* distributed under the License is distributed on an "AS IS" BASIS,
* WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
* See the License for the specific language governing permissions and
* limitations under the License.
*
* Author: Neto Marin
* E-mail: netomarin@gmail.com
*/
#include <EEPROM.h>
#include <avr/eeprom.h>
#include <OneWire.h>
#include <Wire.h>
#include <LiquidCrystal_I2C.h>

// ID of the settings block
#define CONFIG_VERSION "ls1"
// Tell it where to store your config data in EEPROM
#define CONFIG_START 32

//Setup configuration
struct StoreStruct {
  float fermentationTemp;
  float tempDelta;
  char tempUnit;
  long startedWhen;
  char version_of_program[4];
} settings = {
  // The default values
  19.0,
  0.5,
  'C',
  0,
  CONFIG_VERSION
};

/**
 * Variable to control stage/flow
 * 0 - Reading temperature
 * 1 - Default value loaded message
 * 2 - Configuration screen
 */
int stage = 0;
bool waitingCommand = false;
long pressedSince;
int stateButton;
long waiting;

//Aux variables
float tempAux;

//LCD message state
bool invalidateLCD;

//indicator leds configuration
int blueLed = 8;
int redLed = 9;

//Push buttons
int setButton = 2;
int upButton = 3;
int downButton = 4;

//Temperature sensor configuration
int SensorPin = 12;
static char outstr[4];
OneWire ds(SensorPin);

//relay 1 connected to port A0
int relay1 = 14;
//relay 2 connected to port A1
int relay2 = 15;

//LCD setup
#define I2C_ADDR    0x27
#define BACKLIGHT_PIN 3
#define En_pin  2
#define Rw_pin  1
#define Rs_pin  0
#define D4_pin  4
#define D5_pin  5
#define D6_pin  6
#define D7_pin  7

LiquidCrystal_I2C lcd(I2C_ADDR, En_pin, Rw_pin, Rs_pin,
                      D4_pin, D5_pin, D6_pin, D7_pin);

//Operational variables
float fermentationTemp;
float tempVariation;

void setup(void) {
  Serial.begin(9600);

  //Indicator leds outputs
  pinMode(blueLed, OUTPUT);
  pinMode(redLed, OUTPUT);

  //push buttons
  pinMode(setButton, INPUT);
  pinMode(upButton, INPUT);
  pinMode(downButton, INPUT);

  //Defining relay outputs
  pinMode(relay1, OUTPUT);
  pinMode(relay2, OUTPUT);

  //configuring temperature parameters
  loadConfig();

  //Starting LCD
  lcd.begin (20, 4);
  lcd.setBacklightPin(BACKLIGHT_PIN, POSITIVE);
  lcd.setBacklight(HIGH);
  invalidateLCD = true; //first time, invalidating to be drawn
}

void loop(void) {

  if (!waitingCommand) {
    /*
     * Application's flow control
     */
    switch (stage) {
      case 0:
        readingTemperature();
        break;
      case 1:
        confirmDefaultValuesLoaded();
        break;
      case 2:
        startConfigurationWizard();
        break;
      case 3:
        configureMaxDelta();
        break;
    }
  } else {
    /**
     * Command's control
     */

    /**
     * SET button
     */
    if (digitalRead(setButton) == HIGH) {
      waitingCommand = false;
      if (stage == 1) {
        stage = 0;
      } else if (stage == 2) {
        settings.fermentationTemp = tempAux;
        saveConfig();
        stage = 3;
      } else if (stage == 3) {
        settings.tempDelta = tempAux;
        saveConfig();
        stage = 0;
        invalidateLCD = true;
      }
    }

    /**
     * UP button
     */
    if (digitalRead(upButton) == HIGH) {
      if (stage == 2  || stage == 3) {
        tempAux = (tempAux + 0.1);

        //checking if new value and reseting if it's too high
        if (stage == 2 && tempAux >= 40.1) {
          tempAux = -5.0;
        } else if (stage == 3 && tempAux >= 5.1) {
          tempAux = 5.0;
        }
        //updating LCD
        updateTempConfiguration();
      }
    }

    /**
     * DOWN button
     */
    if (digitalRead(downButton) == HIGH) {
      if (stage == 2 || stage == 3) {
        tempAux = (tempAux - 0.1);

        //checking if new value and reseting if it's too low
        if (stage == 2 && tempAux <= -5.1) {
          tempAux = 40.0;
        } else if (stage == 3 && tempAux <= 0.1) {
          tempAux = 0.2;
        }
        //updating LCD
        updateTempConfiguration();
      }
    }

    /*
     * return to stage 0 (read and show temperature) after 5 seconds of idle
     */
    if (stage > 1 && (millis() - waiting) >= 5000) {
      stage = 0;
      invalidateLCD = true;
      waitingCommand = false;
    }
  }

  //Detecting hold on set button
  stateButton = digitalRead(setButton);
  if ( stateButton == HIGH) {
    if (pressedSince == 0) {
      //started holding
      pressedSince = millis();
    } else {
      //set button pressed during 3 seconds
      if ((millis() - pressedSince) >= 3000) {
        stage = 2;
      }
    }
  } else if (pressedSince > 0) {
    //reseting after release the button
    pressedSince = 0;
  }

  //Hidden command to reset configuration
  if (digitalRead(setButton) == HIGH && digitalRead(upButton) == HIGH && digitalRead(downButton) == HIGH) {
    resetEEPROM();
  }
}

/*******************************************************************************************************
 * Stage 0: Reading and showing information about temperatures
 *******************************************************************************************************/
void readingTemperature() {
  if (invalidateLCD) {
    invalidateLCD = false;
    // Starting and configuring LCD
    lcd.clear();
    lcd.home();
    lcd.print("Temperatura:");
    lcd.setCursor(0, 1);
    lcd.print("Temp. ideal: ");
    dtostrf(settings.fermentationTemp, 4, 1, outstr);
    lcd.setCursor(14, 1);
    lcd.print(outstr);
    lcd.print((char)223);
    lcd.print("C");
    lcd.setCursor(0, 2);
    lcd.print("Delta max.: ");
    dtostrf(settings.tempDelta, 4, 1, outstr);
    lcd.setCursor(14, 2);
    lcd.print(outstr);
    lcd.print((char)223);
    lcd.print("C");
    lcd.setCursor(0, 3);
    lcd.print("SmartHomebrew@GitHub");
  }

  float temp = getTemp();
  lcd.setCursor(14, 0);
  dtostrf(temp, 4, 1, outstr);
  lcd.print(outstr);
  lcd.print((char)223);
  lcd.print("C");

  if (temp >= (settings.fermentationTemp + settings.tempDelta)) {
    //Temperature is higher than desired, let's cool down!
    //turn on relay 1 - turn off relay 2 - turn on blue led
    digitalWrite(relay1, LOW);
    digitalWrite(relay2, HIGH);
    digitalWrite(blueLed, HIGH);
    digitalWrite(redLed, LOW);
  } else if (temp <= (settings.fermentationTemp - settings.tempDelta)) {
    //Temperature is lower than desired, let's warm up!
    //turn on relay 2 - turn off relay 1 - turn on red led
    digitalWrite(relay1, HIGH);
    digitalWrite(relay2, LOW);
    digitalWrite(blueLed, LOW);
    digitalWrite(redLed, HIGH);
  }
}

/*******************************************************************************************************
 * Stage 1: Warning user about default values
 *          It could be dangerous to the fermentation proccess.
 *******************************************************************************************************/
void confirmDefaultValuesLoaded() {
  lcd.clear();
  lcd.print("***** ATENCAO! *****");
  lcd.setCursor(0, 1);
  lcd.print("  Inicializado com");
  lcd.setCursor(0, 2);
  lcd.print("  valores padrao !");
  lcd.setCursor(0, 3);
  lcd.print(" Press. set p/ sair");
  waitingCommand = true;
}

/*******************************************************************************************************
 * Stage 2: Configuring temperature to control.
 *******************************************************************************************************/
void startConfigurationWizard() {
  lcd.clear();
  lcd.print("*** Configuracao ***");
  lcd.setCursor(0, 1);
  lcd.print("Temperatura: ");
  dtostrf(settings.fermentationTemp, 4, 1, outstr);
  lcd.setCursor(14, 1);
  lcd.print(outstr);
  lcd.print((char)223);
  lcd.print("C");
  lcd.setCursor(0, 2);
  lcd.print(" + e - para ajustar");
  lcd.setCursor(0, 3);
  lcd.print(" Set para continuar");
  tempAux = settings.fermentationTemp;
  pressedSince = 0;
  waitingCommand = true;
  delay(500);
  waiting = millis();
}

/*******************************************************************************************************
 * Stage 3: Configuring temperature delta
 *******************************************************************************************************/
void configureMaxDelta() {
  lcd.clear();
  lcd.print("*** Configuracao ***");
  lcd.setCursor(0, 1);
  lcd.print("Delta Max.: ");
  dtostrf(settings.tempDelta, 4, 1, outstr);
  lcd.setCursor(14, 1);
  lcd.print(outstr);
  lcd.print((char)223);
  lcd.print("C");
  lcd.setCursor(0, 2);
  lcd.print(" + e - para ajustar");
  lcd.setCursor(0, 3);
  lcd.print(" Set para finalizar");
  tempAux = settings.tempDelta;
  pressedSince = 0;
  waitingCommand = true;
  delay(200);
  waiting = millis();
}

/*******************************************************************************************************
 * Updating LCD with new value after pressing
 * up or down buttons
 *******************************************************************************************************/
void updateTempConfiguration() {
  dtostrf(tempAux, 4, 1, outstr);
  lcd.setCursor(14, 1);
  lcd.print(outstr);
  lcd.print((char)223);
  lcd.print("C");
  delay(200);
  waiting = millis();
}

/*******************************************************************************************************
 * Loading configuration data from EEPROM
 *******************************************************************************************************/
void loadConfig() {
  // To make sure there are settings, and they are YOURS!
  // If nothing is found it will use the default settings.
  if (//EEPROM.read(CONFIG_START + sizeof(settings) - 1) == settings.version_of_program[3] // this is '\0'
    EEPROM.read(CONFIG_START + sizeof(settings) - 2) == settings.version_of_program[2] &&
    EEPROM.read(CONFIG_START + sizeof(settings) - 3) == settings.version_of_program[1] &&
    EEPROM.read(CONFIG_START + sizeof(settings) - 4) == settings.version_of_program[0])
  { // reads settings from EEPROM
    Serial.println("Lendo da EEPROM");
    for (unsigned int t = 0; t < sizeof(settings); t++)
      *((char*)&settings + t) = EEPROM.read(CONFIG_START + t);
  } else {
    Serial.println("Carregando default");
    // settings aren't valid! will overwrite with default settings
    saveConfig();
    stage = 1;
  }
}

/*******************************************************************************************************
 * Saving configuration data to EEPROM
 *******************************************************************************************************/
void saveConfig() {
  Serial.println("Salvando dados");
  for (unsigned int t = 0; t < sizeof(settings); t++)
  { // writes to EEPROM
    EEPROM.write(CONFIG_START + t, *((char*)&settings + t));
    // and verifies the data
    if (EEPROM.read(CONFIG_START + t) != *((char*)&settings + t))
    {
      // error writing to EEPROM
      Serial.println("Erro ao salvar EEPROM");
    }
  }
}

/*******************************************************************************************************
 * Clear EEPROM, writing 0 into all bytes
 *******************************************************************************************************/
void resetEEPROM() {
  for (int i = 0 ; i < EEPROM.length() ; i++) {
    EEPROM.write(i, 0);
  }
  digitalWrite(blueLed, HIGH);
  digitalWrite(redLed, HIGH);
  delay(500);
  digitalWrite(blueLed, LOW);
  digitalWrite(redLed, LOW);
}

/*******************************************************************************************************
 * Reading temperature from the sensor
 *******************************************************************************************************/
float getTemp() {

  byte data[12];
  byte addr[8];
  if ( !ds.search(addr)) {
    //no more sensors on chain, reset search
    ds.reset_search();
    return -1000;
  }
  if ( OneWire::crc8( addr, 7) != addr[7]) {
    Serial.println("CRC is not valid!");
    return -1000;
  }
  if ( addr[0] != 0x10 && addr[0] != 0x28) {
    Serial.print("Device is not recognized");
    return -1000;
  }
  ds.reset();
  ds.select(addr);
  ds.write(0x44, 1);
  byte present = ds.reset();
  ds.select(addr);
  ds.write(0xBE);

  for (int i = 0; i < 9; i++) {
    data[i] = ds.read();
  }

  ds.reset_search();

  byte MSB = data[1];
  byte LSB = data[0];
  float TRead = ((MSB << 8) | LSB);
  float Temperature = TRead / 16;

  return Temperature;

}
