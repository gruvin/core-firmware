/**
 ******************************************************************************
 * @file    application.cpp
 * @authors  Satish Nair, Zachary Crockett and Mohit Bhoite
 * @version V1.0.0
 * @date    05-November-2013
 * @brief   Tinker application
 ******************************************************************************
  Copyright (c) 2013 Spark Labs, Inc.  All rights reserved.

  This program is free software; you can redistribute it and/or
  modify it under the terms of the GNU Lesser General Public
  License as published by the Free Software Foundation, either
  version 3 of the License, or (at your option) any later version.

  This program is distributed in the hope that it will be useful,
  but WITHOUT ANY WARRANTY; without even the implied warranty of
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
  Lesser General Public License for more details.

  You should have received a copy of the GNU Lesser General Public
  License along with this program; if not, see <http://www.gnu.org/licenses/>.
 ******************************************************************************
 */

/*************************/
/*** DRYER TIMER Mk.II ***/
/*************************/


/* Includes ------------------------------------------------------------------*/  
#include "application.h"
#include "onewire.h"
#include "hardware.h"

#include <math.h>
#include <stdlib.h>

/* Spark Cloud Variables ----------------------------------------------------------*/
int cloudTimerSeconds = 0;
char cloudRunTemp[5];
int flagPublishTime = false;      // globals needed because we cannot call publish()
int flagPublishRunTemp = false;   // ... from inside a Spark function

/* Spark Cloud Function prototypes ------------------------------------------------*/
int cloudSetTimer(String args);
int cloudSetRunTemp(String args);

void publishTime() // in seconds
{
  char sTime[6];
  sprintf(sTime, "%u", (int)(mainTimer200 / 200));
  Spark.publish("timer", sTime, 60, PRIVATE);
}
  
void publishRunTemp()
{
  Spark.publish("runtemp", asRunTemp[dt_RunTemp], 60, PRIVATE);
}

// 1-wire Temperature Sensor Data Aquisition
void iterateTemperatureStateMachine() {

  uint8_t sensorData[12];
  uint8_t sensorPresent;
  uint8_t tempCRC;
  static uint8_t tempState = 0;
  static unsigned long tempMillis = 0;
  static uint16_t oldTemperature;

  // Temp sensor state machine (to control timing without a blocking call to delay())
  switch (tempState) {
    case 0: 
      ds.reset();
      ds.select(gSensor1Addr);
      ds.write(0x44, 1);  // start conversion, with parasite power on at the end
      tempMillis = millis() + 1000; // wait 1 second (750ms should be enough, but hey)

      tempState = 1;
      break;

    case 1: // has 1 second elapsed?
      if (
          (millis() >= tempMillis) 
                    ||
          (millis() < 1100) // allow for potential 49th day millis() wrapping back around to zero
        )
          tempState = 2;
      break;

    case 2: // get data rom sensor

      sensorPresent = ds.reset();
      if (sensorPresent) {
        ds.select(gSensor1Addr);    
        ds.write(0xBE);         // Read Scratchpad

        for (uint8_t i = 0; i < 9; i++)  // we need 9 bytes
          sensorData[i] = ds.read();
  /*
        // XXX: DEBUG display raw data
        Serial.print("  Data = ");
        Serial.print(present, HEX);
        Serial.print(" ");
        for ( i = 0; i < 9; i++) { 
          Serial.print(i, DEC);
          Serial.print(":");
          Serial.print(sensorData[i], HEX);
          Serial.print(" ");
        }
        Serial.print(" CRC=");
        Serial.print(tempCRC, HEX);
        Serial.println();
  */
     tempCRC = OneWire::crc8(sensorData, 8);

        if(sensorData[8] == tempCRC) // if data is valid
        {
          // Convert the data to actual temperature
          // because the result is a 16 bit signed integer, it should
          // be stored to an "int16_t" type, which is always 16 bits
          // even when compiled on a 32 bit processor.
          int16_t raw = (sensorData[1] << 8) | sensorData[0];
          if (giSensorType > 0) { // depending on type of chip found during search in setup() 
            raw = raw << 3; // 9 bit resolution default
            if (sensorData[7] == 0x10) {
              // "count remain" gives full 12 bit resolution
              raw = (raw & 0xFFF0) + 12 - sensorData[6];
            }
          } else {
            byte cfg = (sensorData[4] & 0x60);
            // at lower res, the low bits are undefined, so let's zero them
            if (cfg == 0x00) raw = raw & ~7;  // 9 bit resolution, 93.75 ms
            else if (cfg == 0x20) raw = raw & ~3; // 10 bit res, 187.5 ms
            else if (cfg == 0x40) raw = raw & ~1; // 11 bit res, 375 ms
            //// default is 12 bit resolution, 750 ms conversion time
          }
          float celsius = (float)raw / 16.0;
          Serial.print(celsius);
          Serial.println("C");
          currentTemperature = round(celsius);

          // Publish any change in temperature. Maximum rate will be once a second, due to sensor read delay (above)
          if (oldTemperature != currentTemperature)
          {
            oldTemperature = currentTemperature;
            Spark.publish("temperature_1", String(currentTemperature, (unsigned char)DEC).c_str(), 60, PRIVATE);
          }

        }
      }

      tempState = 0; // reset for another temp sensor sample
      break;
  
    default:
      tempState = 0;
  }

  /// END 1-wire Temperature Sensor Data Aquisition
  /////////////////////////////////////////////////////////////
}

/* This function is called once at start up ----------------------------------*/
void setup()
{

  // Register cloud functions and variables
  Spark.function("settimer", cloudSetTimer);
  Spark.function("setruntemp", cloudSetRunTemp);
  Spark.variable("timerseconds", &cloudTimerSeconds, INT);
  Spark.variable("runtemp", cloudRunTemp, STRING);
        
  pinMode(dt_StatusLED, OUTPUT);
  pinMode(dt_LE, OUTPUT);

  pinMode(dt_UpButton, INPUT_PULLUP);
  pinMode(dt_DownButton, INPUT_PULLUP);
  pinMode(dt_TempButton, INPUT_PULLUP);
  pinMode(dt_RedLED, OUTPUT);
  pinMode(dt_OrangeLED, OUTPUT);
  pinMode(dt_BlueLED, OUTPUT);

  pinMode(dt_LEDA, OUTPUT);
  pinMode(dt_LEDB, OUTPUT);
  pinMode(dt_LEDC, OUTPUT);
  pinMode(dt_LEDD, OUTPUT);
  pinMode(dt_LEDE, OUTPUT);
  pinMode(dt_LEDF, OUTPUT);
  pinMode(dt_LEDG, OUTPUT);
  pinMode(dt_LEDDP, OUTPUT);

  // We'll use Timer 2 for LED output digit scanning
  RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2, ENABLE);
  TIM_TimeBaseInitTypeDef timerInitStructure; 
  timerInitStructure.TIM_Prescaler = 45000; // 1,600 counts per second (for finer grain calibration)
  timerInitStructure.TIM_Period = 7; // reload 200 times a second. Measured accuracy well within 5ppm
  timerInitStructure.TIM_CounterMode = TIM_CounterMode_Up;
  timerInitStructure.TIM_ClockDivision = TIM_CKD_DIV1;
  timerInitStructure.TIM_RepetitionCounter = 0;
  TIM_TimeBaseInit(TIM2, &timerInitStructure);
  TIM_ITConfig(TIM2, TIM_IT_Update, ENABLE);  // enable the timer update event, for our interrupt
  TIM_Cmd(TIM2, ENABLE);

  // Set up an interrupt handler for the timer 2 reload event, enabled above
  NVIC_InitTypeDef nvicStructure;
  nvicStructure.NVIC_IRQChannel = TIM2_IRQn;
  nvicStructure.NVIC_IRQChannelPreemptionPriority = 0;
  nvicStructure.NVIC_IRQChannelSubPriority = 1;
  nvicStructure.NVIC_IRQChannelCmd = ENABLE;
  NVIC_Init(&nvicStructure);

  dt_RunTemp = OFF; // paranoia
  dt_relayOutputs = ALL_OFF; //  paranoia too

  Serial.begin(115200); // just for debugging

  searchForSensors();
  if (giSensorType < 0) // hmmm. try one more time ...
    searchForSensors();

  Spark.publish("sensor_type", gsSensorType, 60, PRIVATE);

}

/* This function loops forever --------------------------------------------*/
void loop()
{
  iterateTemperatureStateMachine(); // process 1-wire temperatures

  /////////////////////////////////////////////////////////////
  /// Debounced User Button Procesing State Machine

  #define DEBOUNCE_COUNT 5
  static uint8_t buttons[] = {
    dt_UpButton,
    dt_DownButton,
    dt_TempButton
  };
  static uint8_t buttonCounters[3] = {0, 0, 0};
  static uint8_t activeButton = 0;

  static enum buttonState {
    bsRESET,
    bsSENSE,
    bsPROCESS,
    bsDEBOUNCE
  } buttonState = bsRESET;
  switch (buttonState) 
  {
    case RESET:
      for (uint8_t i=0; i < sizeof(buttons); i++) // clear button debounce counters
        buttonCounters[i] = 0;

      buttonState = bsSENSE;
      break;

    case bsSENSE:
      for (uint8_t i=0; i < sizeof(buttons); i++)
      {
        if (!digitalRead(buttons[i]))
        {
          if (++buttonCounters[i] >= DEBOUNCE_COUNT)
          {
            activeButton = i;
            buttonState = bsPROCESS;
          }
        }
        else
          buttonCounters[i] = 0;
      }   
      break;

    case bsPROCESS:
      switch (activeButton) {
        case 0: // up button
          if (getTimerSeconds() == 0)
            mainTimer200 = 90 * 60 * 200 + 199; // 90:00 + 199/200th second, to let the display read "90:00" for a second
          else if (getTimerSeconds() < (90*60))
            mainTimer200 = (((getTimerSeconds()/600)+1) * 600 * 200) + 199; // next whole 10 minutes (XX:00) + 199/200th second

          publishTime();
          break;

        case 1: // down butAton
          if (getTimerSeconds() >= 600)
            mainTimer200 = ( ((getTimerSeconds() - 1) / 600) * 600) * 200 + 199; // previous whole 10 minutes (XX:00)
          else
            mainTimer200 = 0;

          publishTime();
          break;

        case 2: // temp select button
          if (dt_RunTemp != COOL) // disallow if during cooling time or not running
            dt_RunTemp = (dt_RunTemp == WARM) ? HOT : WARM;
          break;
      } // switch (activeButton)

      buttonState = bsDEBOUNCE;
      break;

    case bsDEBOUNCE:
      if (digitalRead(buttons[activeButton]) && --buttonCounters[activeButton] == 0)
        buttonState = bsRESET;
      break;
  }
  /// END Process UP/DOWN/SELECT buttons
  /////////////////////////////////////////////////////////////

  /////////////////////////////////////////////////////////////
  ///  Output Relay State Machine

  static dt_RunTempType dt_PreCoolTemp;
  
  switch (dt_relayOutputs) {
    case ALL_OFF:
      // monitor timer
      if (getTimerSeconds() >= COOLSECONDS) {
        
        // set initial run temperature, based on last run value, if available
        dt_RunTemp = dt_PreCoolTemp;
        if (dt_RunTemp < WARM) dt_RunTemp = HOT;

        dt_relayOutputs = (dt_RunTemp == HOT) ? RUN_HOT : RUN_WARM;

        Spark.publish("runtemp", asRunTemp[dt_RunTemp], 60, PRIVATE);
      }
      break;

    case RUN_HOT:
    case RUN_WARM:
      // look for changes to RunTemp
      if (dt_RunTemp != dt_PreCoolTemp)
      {
        dt_relayOutputs = (dt_RunTemp == HOT) ? RUN_HOT : RUN_WARM;
        dt_PreCoolTemp = dt_RunTemp;

        publishRunTemp();
        break;
      }

      // monitor timer
      if (getTimerSeconds() == 0) // can happen is user hits DOWN button
      {
        dt_PreCoolTemp = dt_RunTemp;
        dt_RunTemp = OFF;
        dt_relayOutputs = ALL_OFF;

        publishTime();
        publishRunTemp();
      }
      else if (getTimerSeconds() < COOLSECONDS) {
        dt_PreCoolTemp = dt_RunTemp;
        dt_RunTemp = COOL;
        dt_relayOutputs = RUN_COLD;

        publishRunTemp();
      }
      break;

    case RUN_COLD:
      // minitor timer
      if (getTimerSeconds() >= COOLSECONDS)
      {
        dt_RunTemp = dt_PreCoolTemp;
        dt_relayOutputs = (dt_RunTemp == HOT) ? RUN_HOT : RUN_WARM;

        publishRunTemp();
      }
      else if (getTimerSeconds() == 0)
      {
        dt_RunTemp = OFF;
        dt_relayOutputs = ALL_OFF;

        publishTime();
        publishRunTemp();
      }
      break;
  }
  /// END Output Relay State Machine
  /////////////////////////////////////////////////////////////

  /////////////////////////////////////////////////////////////
  /// LED Status Update (stateless)
  switch (dt_RunTemp) {
    
    case OFF:
      digitalWrite(dt_RedLED, 0);
      digitalWrite(dt_OrangeLED, 0);
      digitalWrite(dt_BlueLED, 0);
      break;

    case COOL:
      digitalWrite(dt_RedLED, 0);
      digitalWrite(dt_OrangeLED, 0);
      digitalWrite(dt_BlueLED, 1);
      break;

    case WARM:
      digitalWrite(dt_RedLED, 0);
      digitalWrite(dt_OrangeLED, 1);
      digitalWrite(dt_BlueLED, 0);
      break;

    case HOT:
      digitalWrite(dt_RedLED, 1);
      digitalWrite(dt_OrangeLED, 0);
      digitalWrite(dt_BlueLED, 0);
      break;
  }
  ////////////////////////////////////////////////////


  // Publish timer value every 10 seconds (if running) or when prompted by flagPublishTime
  static uint32_t lastEventTime = 0;
  uint32_t thisEventTime = ((mainTimer200 / 200) % 60) % 10; // need one-shot only
  if (flagPublishTime || (mainTimer200 > 0 && lastEventTime != thisEventTime))
  {
    lastEventTime = thisEventTime; // needs to be a one-shot for each 10 second interval
    if (thisEventTime == 0)
      publishTime();
    flagPublishTime = false;
  }

  if (flagPublishRunTemp)
  {
    publishRunTemp();
    flagPublishRunTemp = false;
  }

  // Update spark cloud variables
  cloudTimerSeconds = mainTimer200 / 200;
  strcpy(cloudRunTemp, asRunTemp[dt_RunTemp]);
  // TODO: add temperature sensor variables


}

////////////////////////////////////////////////////
/// Cloud API functions
int cloudSetTimer(String args)
{
  uint16_t seconds;
  seconds =  args.toInt();
  mainTimer200 = seconds * 200;
  flagPublishTime = true;
  cloudTimerSeconds = seconds;
  return seconds;
}


int cloudSetRunTemp(String args)
{
  if (mainTimer200 == 0)
    return dt_RunTemp;
  else if (args == "hot")
    dt_RunTemp = HOT;
  else if (args == "warm")
    dt_RunTemp = WARM;
  flagPublishRunTemp = true;
  return dt_RunTemp;
}
