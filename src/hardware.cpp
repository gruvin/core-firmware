#include "application.h"
#include "hardware.h"
#include "onewire.h"

// 7-SEG LED 'font' definition
const unsigned char dt_ledDigitsFont[] = {
  0b00111111, // 0
  0b00000110, // 1
  0b01011011, // 2
  0b01001111, // 3
  0b01100110, // 4
  0b01101101, // 5
  0b01111101, // 6
  0b00000111, // 7
  0b01111111, // 8
  0b01101111, // 9
  0b00000000, // ' ' (blank)
  0b01100001, // super-script 'c'
  0b01000000, // '-'
};

int dt_ledIOBits[] = {
  dt_LEDA,
  dt_LEDB,
  dt_LEDC,
  dt_LEDD,
  dt_LEDE,
  dt_LEDF,
  dt_LEDG,
  dt_LEDDP
};
uint8_t dt_ledDigits[] = { 0, 0, 0, 0 }; // the digits output by the scanning interrupt handler

char asRunTemp[][5] = { "off", "cool", "warm", "hot" };

volatile uint32_t mainTimer200 = 0;
int16_t currentTemperature = -32767; // mark invalid temperature reading

volatile dt_relaysType  dt_relayOutputs = ALL_OFF;
volatile dt_RunTempType dt_RunTemp = OFF;

static __INLINE void dt_setDisplayTimeTo(uint16_t seconds)
{
  uint8_t mins, secs;

  mins = seconds / 60;
  secs = seconds % 60;

  dt_ledDigits[0] = mins / 10;
  dt_ledDigits[1] = mins % 10;
  dt_ledDigits[2] = secs / 10;
  dt_ledDigits[3] = secs % 10;

}

static __INLINE void dt_setDisplayTempTo(int16_t celsius)
{
  if (celsius > -32767)
  {
    int hundreds = celsius / 100; // assume temp. will never go below -99C
    dt_ledDigits[0] = (hundreds) ? hundreds : 10; // 10=blank
    if (celsius < 0) dt_ledDigits[0] = 12; // '-'

    dt_ledDigits[1] = (abs(celsius) % 100) / 10;
    dt_ledDigits[2] = abs(celsius) % 10;
  }
  else
  {
    dt_ledDigits[0] = 10; // blank
    dt_ledDigits[1] = 12; // '-'
    dt_ledDigits[2] = 12; // '-'
  }
  dt_ledDigits[3] = 11; // 12th digit is a superscript 'c'
}

static __INLINE void dt_outputDigit(uint8_t fontByte)
{
  for (uint8_t i=0; i <= 7; i++)
  {
    digitalWrite(dt_ledIOBits[i], (dt_ledDigitsFont[fontByte] & (1<<i)) ? 1: 0);
  }
}

static __INLINE void dt_latchDigitCommon(uint8_t digitNumber)
{
  uint8_t latchData = (dt_relayOutputs << 4) | (1 << digitNumber);

  for (uint8_t i=0; i <= 7; i++) // prepare latch data
    digitalWrite(dt_ledIOBits[i], (latchData & (1<<i)) ? 1: 0);

  digitalWrite(dt_LE, 1); // send latch pulse
  digitalWrite(dt_LE, 0);
}

uint16_t getTimerSeconds()
{
  return (mainTimer200 / 200);
}

/// END FUNCTIONS
////////////////////////////////////////////////////

////////////////////////////////////////////////////
/// IRQ: LED Digit Output Scanning
/// Called 200 times a second, for effective 50Hz on each digit.
extern "C" void TIM2_IRQHandler()
{
  static uint8_t currentDigitNumber = 0;
  static uint8_t debugFreqD7 = 0;

  digitalWrite(D7, (debugFreqD7 = !debugFreqD7)); // toggle D7 for freq. of 100Hz.

  if (TIM_GetITStatus(TIM2, TIM_IT_Update) != RESET){
    TIM_ClearITPendingBit(TIM2, TIM_IT_Update); // clear the interrupt flag

    if (mainTimer200 > 0)
      dt_setDisplayTimeTo(getTimerSeconds());
    else
      dt_setDisplayTempTo(currentTemperature); // XXX: temp hack

    /////////// Update LED display ////////////
    dt_latchDigitCommon(currentDigitNumber);
    dt_outputDigit(dt_ledDigits[currentDigitNumber]);
    if (mainTimer200 > 0)
      digitalWrite(dt_ledIOBits[7], 1); // decimal place always on
    if (++currentDigitNumber > 3) currentDigitNumber = 0;
    ///////////////////////////////////////////

    // Decrement the seconds x 200 counter
    if (mainTimer200 > 0) mainTimer200--;

  }
}
// END IRQ
////////////////////////////////////////////////////

void searchForSensors()
{

  // Search for temp sensor 1-wire device
  if ( !ds.search(gSensor1Addr)) {
    giSensorType = -1;
    Serial.println("No temperature sensors found.");
    ds.reset_search();
  } else {
    if (OneWire::crc8(gSensor1Addr, 7) != gSensor1Addr[7])
    {
      giSensorType = -1;
      Serial.println("Temp sensor CRC error :'(");
    }
    else 
    {
      Serial.print("Temp sensor chip = ");

      // the first ROM byte indicates which chip
      // TODO: Typedef enum the sensor types and use the raw value -- 0x10, 0x22 or 0x28 therein
      switch (gSensor1Addr[0]) {
        case 0x10:
          strcpy(gsSensorType, "DS18S20");  // or old DS1820
          giSensorType = 1;
          break;
        case 0x28:
          strcpy(gsSensorType, "DS18B20");
          giSensorType = 0;
          break;
        case 0x22:
          strcpy(gsSensorType, "DS1822");
          giSensorType = 0;
          break;
        default:
          strcpy(gsSensorType, "NONE");
          giSensorType = -1;
      } 
      Serial.println(gsSensorType);
    }
  }
  if (giSensorType >= 0) {
    Serial.print("ROM =");
    for(uint8_t i = 0; i < 8; i++) {
      Serial.write(' ');
      Serial.print(gSensor1Addr[i], HEX);
    }
  }
}


