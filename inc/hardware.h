#define dt_LEDA A0
#define dt_LEDB A1
#define dt_LEDC A2
#define dt_LEDD A3
#define dt_LEDE A4
#define dt_LEDF A5
#define dt_LEDG A6
#define dt_LEDDP A7

#define dt_LE         D0      // LE on 74HC573 octal latch chip (outputs always enabled)
#define dt_TempButton D1      // temperature selection button
#define dt_DownButton D2      // Time Down button
#define dt_UpButton   D3      // Time Up / Start button
#define dt_RedLED     D4      // Temp HOT
#define dt_OrangeLED  D5      // Temp WARM
#define dt_BlueLED    D6      // Cool down indicator (motoro running with both heaters off)
#define dt_StatusLED  D7      // Spark Core on-board LED (reserved)

#define COOLSECONDS 300 // 5 minutes

enum dt_relaysType {
  ALL_OFF   = 0b0000,
  RUN_COLD  = 0b0011,
  RUN_WARM  = 0b0111,
  RUN_HOT   = 0b1111
};

enum dt_RunTempType {
  OFF=0, COOL, WARM, HOT
};

extern volatile uint32_t mainTimer200;
extern int16_t currentTemperature;

extern char asRunTemp[][5];

extern volatile dt_relaysType  dt_relayOutputs;
extern volatile dt_RunTempType dt_RunTemp;

uint16_t getTimerSeconds();
void searchForSensors();

