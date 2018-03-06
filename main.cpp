#include <Arduino.h>

/**
 *  @file    planttestNG.ino
 *  @author  peter c
 *  @date    4/14/2017
 *  @version 0.1
 *
 *
 *  @section DESCRIPTION
 *  Control an ebb and flow hydroponic setup. Data published via as a modbus
 *slave and setpoints,
 *  alarm limits saved in EEPRPOM
 *
 */


#include <Time.h>
#include <HardwareSerial.h>
#include <TimeLib.h>
#include <Timezone.h>   // https://github.com/JChristensen/Timezone
#include <OneWire.h>
#include <DallasTemperature.h>
#include <Wire.h>       // for RTC
#include <DS3232RTC.h>  // http://github.com/JChristensen/DS3232RTC
#include <avr/eeprom.h>
#include <TimeAlarms.h> // https://github.com/PaulStoffregen/TimeAlarms
#include <Streaming.h>
#include <NewPing.h>
#include <Adafruit_Sensor.h>
#include <DHT.h>           // DHT-22 humidity sensor
#include <LiquidCrystal_I2C.h> //https://bitbucket.org/fmalpartida/new-liquidcrystal/wiki/Home
#include <RunningMedian.h> // https://github.com/RobTillaart/Arduino/tree/master/libraries/RunningMedian

#include <DA_Flowmeter.h>
#include <DA_Analoginput.h>
#include <DA_Discreteinput.h>
#include <DA_DiscreteOutput.h>
#include <DA_DiscreteOutputTmr.h>
#include <DA_HOASwitch.h>
#include <DA_AtlasPH.h>
#include <DA_AtlasEC.h>

#include "PlantModbus.h"


#define DEFAULT_LIGHTS_ON_ALARM_TIME 1519992000       // AlarmHMS (5, 0, 0)
#define DEFAULT_LIGHTS_OFF_ALARM_TIME 1520056800      // AlarmHMS (23, 0, 0)
#define DEFAULT_RESET_TIME  1519974000                // AlarmHMS(0, 0, 0) //
                                                      // midnight
#define DEFAULT_CIRCULATION_PUMP_ON_DURATION 15 * 60  // 15 min * 60 s
#define DEFAULT_CIRCULATION_PUMP_OFF_DURATION 45 * 60 // 45 min * 60 s
#define DEFAULT_FAN_ON_DURATION 30 * 60               // 30 min * 60 s
#define DEFAULT_FAN_OFF_DURATION 30 * 60              // 30 min * 60 s

#define STRLEN(s) (sizeof(s) / sizeof(s[0]))


// #define HOST_COMMAND_CHECK_INTERVAL  1000
#define ALARM_REFRESH_INTERVAL 10

const unsigned long DEFAULT_TIME = 1000188000;

#define EEPROM_CONFIGURED 2       // this value stored at address
                                  // CONFIG_FLAG_ADDR
#define EEPROM_CONFIG_FLAG_ADDR 0 // is 0 if nothing was written
#define EEPROM_GROWING_CHAMBER_ON_TIME_ADDR EEPROM_CONFIG_FLAG_ADDR + \
  sizeof(unsigned short)
#define EEPROM_GROWING_CHAMBER_OFF_TIME_ADDR EEPROM_GROWING_CHAMBER_ON_TIME_ADDR + \
  sizeof(time_t)
#define EEPROM_SEEDING_AREA_ON_TIME_ADDR EEPROM_GROWING_CHAMBER_OFF_TIME_ADDR + \
  sizeof(time_t)
#define EEPROM_SEEDING_AREA_OFF_TIME_ADDR EEPROM_SEEDING_AREA_ON_TIME_ADDR + \
  sizeof(time_t)
#define EEPROM_FAN_ON_DURATION_ADDR EEPROM_SEEDING_AREA_OFF_TIME_ADDR + \
  sizeof(time_t)
#define EEPROM_FAN_OFF_DURATION_ADDR EEPROM_FAN_ON_DURATION_ADDR + \
  sizeof(unsigned int)
#define EEPROM_CIRCULATION_PUMP_ON_DURATION_ADDR EEPROM_FAN_OFF_DURATION_ADDR + \
  sizeof(unsigned int)
#define EEPROM_CIRCULATION_PUMP_OFF_DURATION_ADDR \
  EEPROM_CIRCULATION_PUMP_ON_DURATION_ADDR + sizeof(unsigned int)

#define VERSION 110 // imlied two decimal


// comment out to not include terminal processing
// #define PROCESS_TERMINAL
// #define PROCESS_TERMINAL_VERBOSE
HardwareSerial *tracePort = &Serial;

// comment out to not implement modbus
#define PROCESS_MODBUS
// refresh intervals
#define POLL_CYCLE_SECONDS 5            // sonar and 1-wire refresh rate

#define TEMPERATURE_COMPENSATE_CYCLE 60 // temperature compensate every 60s
// flow meter
#define FLOW_CALC_PERIOD_SECONDS 1      // flow rate calc period
#define FT002_SENSOR_INTERUPT_PIN 2

#define ENABLE_FT002_SENSOR_INTERRUPTS attachInterrupt(digitalPinToInterrupt(        \
                                                         FT002_SENSOR_INTERUPT_PIN), \
                                                       onFT_002_PulseIn,             \
                                                       RISING)
#define DISABLE_FT002_SENSOR_INTERRUPTS detachInterrupt(digitalPinToInterrupt( \
                                                          FT002_SENSOR_INTERUPT_PIN))
FlowMeter FT_002(FT002_SENSOR_INTERUPT_PIN, FLOW_CALC_PERIOD_SECONDS); // interrupt
                                                                       // pin,
                                                                       // calculation
                                                                       // period
                                                                       // in
                                                                       // seconds

#define FT003_SENSOR_INTERUPT_PIN 3
#define ENABLE_FT003_SENSOR_INTERRUPTS attachInterrupt(digitalPinToInterrupt(        \
                                                         FT003_SENSOR_INTERUPT_PIN), \
                                                       onFT_003_PulseIn,             \
                                                       RISING)
#define DISABLE_FT003_SENSOR_INTERRUPTS detachInterrupt(digitalPinToInterrupt( \
                                                          FT003_SENSOR_INTERUPT_PIN))
FlowMeter FT_003(FT003_SENSOR_INTERUPT_PIN, FLOW_CALC_PERIOD_SECONDS); // interrupt
                                                                       // pin,
                                                                       // calculation
                                                                       // period
                                                                       // in
                                                                       // seconds

#define CO2_INTERRUPT_PIN 18
#define ENABLE_CO2_SENSOR_RISING_INTERRUPTS attachInterrupt(digitalPinToInterrupt( \
                                                              CO2_INTERRUPT_PIN),  \
                                                            on_AT_102_Rising,      \
                                                            RISING)
#define ENABLE_CO2_SENSOR_FALLING_INTERRUPTS attachInterrupt(digitalPinToInterrupt( \
                                                               CO2_INTERRUPT_PIN),  \
                                                             on_AT_102_Falling,     \
                                                             FALLING)
#define DISABLE_CO2_SENSOR_INTERRUPTS detachInterrupt(digitalPinToInterrupt( \
                                                        CO2_INTERRUPT_PIN))

// CO2 timing vars
#define  CO2_PERIOD   1004 // 1 PWM cycle in millisecs
#define  CO2_MAX_RATE_OF_CHANGE 200
volatile unsigned long timeOn_AT_102Start = 0;

// volatile unsigned int AT_102Raw = 0;

volatile unsigned int AT_102Raw  = 0;
RunningMedian AT_102MedianFilter = RunningMedian(5);

// Ph and EC probes
//
#define PH_I2C_ADDRESS 100
#define EC_I2C_ADDRESS 101
DA_AtlasPH AT_001 = DA_AtlasPH(PH_DEFAULT_I2C_ADDRESS);
DA_AtlasEC AT_002 = DA_AtlasEC(EC_DEFAULT_I2C_ADDRESS);

/*
   Blue Serial IIC/I2C/TWI 2004 204 20X4 Character LCD Module Display For
      Arduino
   PCF8574 BIT     HD44780 Function
   0               RS
   1               RW
   2               EN
   3               backlight (POSITIVE)
   4               D4
   5               D5
   6               D6
   7               D7
   back light pin, polarity
 */
LiquidCrystal_I2C lcd(0x27, 2, 1, 0, 4, 5, 6, 7, 3, POSITIVE);

// screens
// enum ScreenType  { HomeScreen=0, HOAStatus, MiscStatus  };
// ScreenType currentScreen = HomeScreen;
typedef struct LCDScreen LCDScreen;
struct LCDScreen
{
  void              (*displayFunc) (bool clearScreen);
  struct LCDScreen *previousScreen;
  struct LCDScreen *nextScreen;
};

LCDScreen *currentScreen;


LCDScreen firstScreen;
LCDScreen secondScreen;
LCDScreen thirdScreen;
LCDScreen fourthScreen;
LCDScreen fifthScreen;

// DHT-22 - one wire type humidity sensor (won't work with one wire lib)
#define DHT_BUS_PIN 5
DHT   AT_101   = DHT(DHT_BUS_PIN, DHT22);
float AT_101T  = NAN;
float AT_101H  = NAN;
float AT_101HI = NAN;

// atlas sensor
IO_TYPE currentIOType = i2c_ph;

// One Wire
//
//
DeviceAddress ambientTemperatureAddress =
{
  0x28, 0x6F, 0xE3, 0xA0, 0x04, 0x00, 0x00, 0x5A
};

float TT_001T                           = NAN;
DeviceAddress mixtureTemperatureAddress =
{
  0x28, 0xFF, 0xF4, 0xF6, 0x84, 0x16, 0x05, 0x0C
};

#define WIRE_BUS_PIN 4 // pin
#define ONE_TEMPERATURE_PRECISION 9
OneWire oneWire(WIRE_BUS_PIN);
DallasTemperature sensors(&oneWire);

// Analog Inputs
//
// NewPing sonar(TRIGGER_PIN, ECHO_PIN, MAX_DISTANCE);
#define NUTRIENT_TANK_HEIGHT 45    // hight of nutrient tank in cm
#define NUTRIENT_TANK_AIR_GAP 16.5 // space between sensor and max water level
                                   // in cm
#define NUTRIENT_DEAD_BAND 5.0     // % change allowed between samples as sensor
                                   // bounces because of noise issue
// volume 28.5 * 76.2 * 45.75 => 100 L volume peroxyde ration 3 ml per 3.8 L
const float NUTRIENT_TANK_MIXTURE_MAX = NUTRIENT_TANK_HEIGHT -
                                        NUTRIENT_TANK_AIR_GAP;
NewPing LT_002(15, 16, NUTRIENT_TANK_HEIGHT); // Water Level
float   LT_002Raw = 0.0;                      // Water Level present value in cm
                                              // from top,  0-> undefined

RunningMedian LT_002MedianFilter = RunningMedian(5);

// Discete Outputs
// DO 35,36 AC spares
// DO DC 39,41,42,43 spares
DA_DiscreteOutput DY_102    = DA_DiscreteOutput(31, LOW); // Seeding LED 120 VAC
DA_DiscreteOutput DY_103    = DA_DiscreteOutput(32, LOW); // Growing Chamber LED
                                                          // 120 VAC
DA_DiscreteOutputTmr PY_001 = DA_DiscreteOutputTmr(33,
                                                   LOW,
                                                   DEFAULT_CIRCULATION_PUMP_ON_DURATION,
                                                   DEFAULT_CIRCULATION_PUMP_OFF_DURATION); //
                                                                                           // Circulation
                                                                                           // Pump
                                                                                           // 15
                                                                                           // mins
                                                                                           // on/45
                                                                                           // off
                                                                                           // 120VAC
DA_DiscreteOutputTmr MY_101 = DA_DiscreteOutputTmr(34,
                                                   LOW,
                                                   DEFAULT_FAN_ON_DURATION,
                                                   DEFAULT_FAN_OFF_DURATION); //
                                                                              // Fan,
                                                                              // 60
                                                                              // on/60
                                                                              // off
                                                                              // 120VAC
DA_DiscreteOutput VY_001A = DA_DiscreteOutput(37, LOW);                       //
                                                                              // inlet
                                                                              // H20
                                                                              // valve,
                                                                              // active
                                                                              // low
                                                                              // 12VDC
DA_DiscreteOutput PY_002  = DA_DiscreteOutput(38, LOW);                       //
                                                                              // Drain
                                                                              // Pump
                                                                              // 12VDC


// Discrete Inputs
DA_DiscreteInput LSHH_002 = DA_DiscreteInput(51,
                                             DA_DiscreteInput::ToggleDetect,
                                             true); // nutrient mixture hi-hi
                                                    // level switch
DA_DiscreteInput HS_001 = DA_DiscreteInput(50,
                                           DA_DiscreteInput::ToggleDetect,
                                           true);   // Drain Pump Hand status :
                                                    // Start/Stop
DA_DiscreteInput SSH_101 = DA_DiscreteInput(9);     // Smoke Detector
DA_DiscreteInput HS_002  = DA_DiscreteInput(49,
                                            DA_DiscreteInput::ToggleDetect,
                                            true);  // Inlet H20 Open/Close
DA_DiscreteInput HS_003A = DA_DiscreteInput(25,
                                            DA_DiscreteInput::RisingEdgeDetect,
                                            true);  // LCD display next
DA_DiscreteInput HS_003B = DA_DiscreteInput(26,
                                            DA_DiscreteInput::RisingEdgeDetect,
                                            true);  // LCD display previous
DA_DiscreteInput HS_003C = DA_DiscreteInput(27,
                                            DA_DiscreteInput::RisingEdgeDetect,
                                            true);  // LCD display Enter
DA_HOASwitch HS_001AB = DA_HOASwitch(7, 0, 8);      // Circulation Pump Hand
                                                    // Status : HOA : Hand/Auto
// DA_HOASwitch HS_101AB = DA_HOASwitch(28, 0, 29); // Fan : HOA : Hand/Auto
DA_HOASwitch HS_102AB = DA_HOASwitch(52, 0, 53);    // Seeding Area LED : HOA :
                                                    // Hand/Auto
DA_HOASwitch HS_103AB = DA_HOASwitch(10, 0, 11);    // Growing Chamber LED : HOA
                                                    // : Hand/Auto


TimeChangeRule usMDT =
{
  "MDT", Second, dowSunday, Mar, 12, -360
};

TimeChangeRule usMST =
{
  "MST", First, dowSunday, Nov, 5, -420
};

Timezone usMT(usMDT, usMST);
struct _AlarmEntry
{
  time_t  epoch;
  AlarmId id        = dtINVALID_ALARM_ID;
  bool    firstTime = true;
};

struct _lightControlEntry
{
  time_t onEpoch;  // utc epoch
  time_t offEpoch; // utc epoch

  void (*onLightsOn)();
  void (*onLightsOff)();
};


typedef _lightControlEntry LightControlEntry;

LightControlEntry growingChamberLights;
LightControlEntry seedingAreaLights;


typedef _AlarmEntry AlarmEntry;
AlarmEntry onMidnight;
AlarmEntry onRefreshAnalogs;        // sonar and 1-wire read refresh
AlarmEntry onFlowCalc;              // flow calculations
AlarmEntry onTemperatureCompensate; // atlas sensor temperature compensation


/**
 * .ino file converted to plantformIO main.c
 *  forward declare functions
 */
time_t       alarmTimeToUTC(time_t localAlarmTime);
time_t       alarmTimeToLocal(time_t utcAlarmTime);
void         dateTimeToBuffer(time_t aTime,
                              char  *buffer);
void         displayDateTime();
void         displayHOAStatuses(bool clearScreen);
void         displayHomeScreen(bool clearScreen);
void         displayMiscStatuses(bool clearScreen);
void         displayTimerStatuses(bool clearScreen);
void         do_ONP_SPoll();
void         doLCDSplashScreen();
void         doOnCalcFlowRate();
void         doOnMidnight();
void         doOnTemperatureCompensate();
void         EEPROMLoadConfig();
time_t       EEPROMReadAlarmEntry(unsigned int atAddress);
void         EEPROMWriteAlarmEntry(time_t       epoch,
                                   unsigned int atAddress);
void         EEPROMWriteDefaultConfig();
void         EEPROMWriteDuration(unsigned int duration,
                                 unsigned int atAddress);
bool         getModbusCoilValue(unsigned short startAddress,
                                unsigned short bitPos);
void         initOneWire();
void         initOneWireDevice(DeviceAddress aDevice,
                               uint8_t       aIndex);
unsigned int isEEPROMConfigured();
unsigned int EEPROMReadDuration(unsigned int atAddress);
void         on_AT_102_Falling();
void         on_AT_102_Falling();
void         on_AT_102_Rising();
void         on_Circulation_Pump_Process(DA_HOASwitch::HOADetectType state);
void         on_DrainPump_Process(bool state,
                                  int  aPin);
void         on_GrowingChamberLED_Process(DA_HOASwitch::HOADetectType state);
void         on_InletValve_Process(bool state,
                                   int  aPin);
void         on_LCD_Enter(bool state,
                          int  aPin);
void         on_LCD_Next_Screen(bool state,
                                int  aPin);
void         on_LCD_Previous_Screen(bool state,
                                    int  aPin);
void         on_SeedingAreaLED_Process(DA_HOASwitch::HOADetectType state);
void         onAtlasECSample(IO_TYPE type,
                             float   value);
void         onAtlasPhSample(IO_TYPE type,
                             float   value);
void         onFT_002_PulseIn();
void         onFT_003_PulseIn();
void         printOneWireAddress(HardwareSerial *tracePort,
                                 DeviceAddress   aDeviceAddress,
                                 bool            aCR);
void         processModbusCommands();
void         processModbusECCommands();
void         processModbusPHCommands();
void         refreshDiscreteInputs();
void         refreshDiscreteOutputs();
void         refreshModbusRegisters();
void         setConfigToDefaults();
void         setModbusCirculationPumpOffDuration();
void         setModbusCirculationPumpOnDuration();
void         setModbusFanOffDuration();
void         setModbusFanOnDuration();
void         setModbusForceGrowingChamberLightsOff();
void         setModbusForceGrowingChamberLightsOn();
void         setModbusForceSeedingAreaLightsOff();
void         setModbusForceSeedingAreaLightsOn();
void         setModbusGrowingChamberLightsOffTime();
void         setModbusGrowingChamberLightsOnTime();
void         setModbusSeedingAreaLightsOffTime();
void         setModbusSeedingAreaLightsOffTime();
void         setModbusSeedingAreaLightsOnTime();
void         setModbusTime();
void         setupLCD();
void         setupLCDScreens();
void         setupRTC();
void         timeToBuffer(time_t aTime,
                          char  *buffer);
void         writeModbusCoil(unsigned short startAddress,
                             unsigned short bitPos,
                             bool           value);


#ifdef PROCESS_TERMINAL
void processTerminalCommands();
void processCalibrateMessage(IO_TYPE aIO_Type);
void processTemperatureCompensation();
void processSerializeMessage();
void processTimeSetMessage();
void showCommands();
void processFanDurations();
void processCirculationPumpDurations();
void processSAAlarmMessage();
void processGCAlarmMessage();
void processLightEntryMessage(time_t      *anEpoch,
                              unsigned int eepromAddr);
void processAlarmEntryMessage(struct _AlarmEntry *aAlarmEntry,
                              unsigned int        eepromAddr,
                              OnTick_t            alarmHandler);
void processLightsMessage();
void processDisplayMessage();
void processDisplayIOMessage();
void displayLightControlEntry(char  *who,
                              time_t anEpoch);
void displayAlarm(char              *who,
                  struct _AlarmEntry aAlarmEntry);
#endif // ifdef PROCESS_TERMINAL


void setup()
{
#ifdef PROCESS_TERMINAL
  tracePort->begin(9600);
#endif // ifdef PROCESS_TERMINAL

#ifdef PROCESS_MODBUS
  slave.begin(MB_SERIAL_BAUD);
#endif // ifdef PROCESS_MODBUS

  randomSeed(analogRead(0));

  // LCD display
  setupLCD();
  setupLCDScreens();
  setupRTC();

  // InletValve
  //
  //
  onRefreshAnalogs.id =
    Alarm.timerRepeat(POLL_CYCLE_SECONDS, do_ONP_SPoll);
  onFlowCalc.id = Alarm.timerRepeat(FLOW_CALC_PERIOD_SECONDS,
                                    doOnCalcFlowRate);
  onTemperatureCompensate.id = Alarm.timerRepeat(TEMPERATURE_COMPENSATE_CYCLE,
                                                 doOnTemperatureCompensate);

  onMidnight.epoch = alarmTimeToUTC(DEFAULT_RESET_TIME);
  onMidnight.id    = Alarm.alarmRepeat(onMidnight.epoch, doOnMidnight);

  if (isEEPROMConfigured() == EEPROM_CONFIGURED)
  {
    EEPROMLoadConfig();
  }
  else
  {
    EEPROMWriteDefaultConfig();
    EEPROMLoadConfig();
  }
  HS_002.setPollingInterval(500);   // ms
  LSHH_002.setDebounceTime(1000);   // float switch bouncing around
  LSHH_002.setPollingInterval(500); // ms
  HS_002.setOnEdgeEvent(&on_InletValve_Process);
  LSHH_002.setOnEdgeEvent(&on_InletValve_Process);
  HS_001.setPollingInterval(200);   // ms
  // HS_001.setDebounceTime( 110);
  HS_001.setOnEdgeEvent(&on_DrainPump_Process);
  HS_003A.setPollingInterval(200);  // ms
  HS_003B.setPollingInterval(200);  // ms
  HS_003C.setPollingInterval(200);  // ms
  HS_003A.setDebounceTime(150);     // ms
  HS_003B.setDebounceTime(150);     // ms
  HS_003C.setDebounceTime(150);     // ms
  HS_003A.setOnEdgeEvent(&on_LCD_Next_Screen);
  HS_003B.setOnEdgeEvent(&on_LCD_Previous_Screen);
  HS_003C.setOnEdgeEvent(&on_LCD_Enter);

  HS_001AB.setOnStateChangeDetect(&on_Circulation_Pump_Process);

  // HS_101AB.setOnStateChangeDetect(& on_Fan_Process);
  HS_102AB.setOnStateChangeDetect(&on_SeedingAreaLED_Process);
  HS_103AB.setOnStateChangeDetect(&on_GrowingChamberLED_Process);

  AT_001.setOnPollCallBack(onAtlasPhSample);
  AT_002.setOnPollCallBack(onAtlasECSample);
  AT_002.setPollingInterval(2000);

  AT_001.setPollingInterval(3000);
  AT_001.retrieveCompensatedTemperature();
  AT_002.retrieveCompensatedTemperature();


  // 1-wire
  sensors.begin();
  initOneWire();

  // humidity sensor
  AT_101.begin();
  ENABLE_FT002_SENSOR_INTERRUPTS;
  ENABLE_CO2_SENSOR_RISING_INTERRUPTS;
  lcd.clear();
}

void loop()
{
#ifdef PROCESS_MODBUS
  refreshModbusRegisters();
  slave.poll(modbusRegisters, MODBUS_REG_COUNT);
  processModbusCommands();
#endif // ifdef PROCESS_MODBUS

#ifdef PROCESS_TERMINAL
  processTerminalCommands();
#endif // ifdef PROCESS_TERMINAL

  refreshDiscreteInputs();
  refreshDiscreteOutputs();
  Alarm.delay(ALARM_REFRESH_INTERVAL);


  if (currentIOType == i2c_ph) AT_001.refresh();
  else AT_002.refresh();

  // . AT_001.serialize(tracePort,true);
  // doReadInputs();
  // doUpdateOutputs();
  // LSL_100.refresh();
}

void onFT_002_PulseIn()
{
  FT_002.handleFlowDetection();
}

void onFT_003_PulseIn()
{
  FT_003.handleFlowDetection();
}

void on_AT_102_Rising()
{
  DISABLE_CO2_SENSOR_INTERRUPTS;

  // unsigned long timestamp = micros();
  // timeCycle = micros();
  timeOn_AT_102Start = millis();

  // timeOff_AT_102Raw= timeOn_AT_102Start - timeOff_AT_102Start;
  ENABLE_CO2_SENSOR_FALLING_INTERRUPTS;
}

void on_AT_102_Falling()
{
  DISABLE_CO2_SENSOR_INTERRUPTS;
  unsigned int  tempAT_102Raw;
  unsigned long timeOn  = (unsigned long)abs(millis() - timeOn_AT_102Start);
  unsigned long timeOff = CO2_PERIOD - timeOn;

  tempAT_102Raw = (unsigned int)(2000 * (timeOn - 2) / (timeOn + timeOff - 4));


  if (tempAT_102Raw  <= 2000)
  {
    AT_102Raw = tempAT_102Raw;
  }


  // Serial << "AT-102="<< AT_102Raw<< " timeOn=" << timeOn << " timeOff=" <<
  // timeOff << endl;
  ENABLE_CO2_SENSOR_RISING_INTERRUPTS;
}

/*
   Only open inlet H20 Valve iff no Hi-Hi water level
   note LSHH is high on high level (fail safe )
 */
void on_InletValve_Process(bool state, int aPin)
{
#ifdef PROCESS_TERMINAL_VERBOSE
  *tracePort << "on_InletValve_Process HS_002, LSHH_002" << endl;
  HS_002.serialize(tracePort, true);
  LSHH_002.serialize(tracePort, true);
#endif // ifdef PROCESS_TERMINAL_VERBOSE

  if ((HS_002.getSample() == LOW) &&
      (LSHH_002.getSample() == HIGH)) VY_001A.activate();
  else VY_001A.reset();
}

void on_LCD_Next_Screen(bool state, int aPin)
{
  currentScreen = currentScreen->nextScreen;
  currentScreen->displayFunc(true);
}

void on_LCD_Previous_Screen(bool state, int aPin)
{
  currentScreen = currentScreen->previousScreen;
  currentScreen->displayFunc(true);
}

void on_LCD_Enter(bool state, int aPin)
{
#ifdef PROCESS_TERMINAL_VERBOSE
  *tracePort << "TO DO: LCD Enter HS_003C" << endl;
  HS_003C.serialize(tracePort, true);
#endif // ifdef PROCESS_TERMINAL_VERBOSE
}

void on_DrainPump_Process(bool state, int aPin)
{
#ifdef PROCESS_TERMINAL_VERBOSE
  *tracePort << "on_DrainPump_Process HS_001" << endl;
  HS_001.serialize(tracePort, true);
#endif // ifdef PROCESS_TERMINAL_VERBOSE

  if (HS_001.getSample() == LOW)
  {
    PY_002.activate();
  }
  else PY_002.reset();
}

void on_Circulation_Pump_Process(DA_HOASwitch::HOADetectType state)
{
#ifdef PROCESS_TERMINAL_VERBOSE
  *tracePort << "on_Circulation_Pump_Process HS_001AB" << endl;
  HS_001AB.serialize(tracePort, true);
#endif // ifdef PROCESS_TERMINAL_VERBOSE

  switch (state)
  {
  case DA_HOASwitch::Hand:

    PY_001.pauseTimer();
    PY_001.disable();
    PY_001.forceActive(); // force the pump on


    break;

  case DA_HOASwitch::Off:
    PY_001.pauseTimer();
    PY_001.disable();

    break;

  case DA_HOASwitch::Auto:
    PY_001.enable();
    PY_001.resumeTimer();

    // PY_001.restart();
    // PY_001.resume();
    break;

  default:
    break;
  }
}

void on_GrowingChamberLED_Process(DA_HOASwitch::HOADetectType state)
{
#ifdef PROCESS_TERMINAL_VERBOSE
  *tracePort << "on_FlowingLED_Process HS_103AB" << endl;
  HS_103AB.serialize(tracePort, true);
#endif // ifdef PROCESS_TERMINAL_VERBOSE

  switch (state)
  {
  case DA_HOASwitch::Hand:
    DY_103.disable();
    DY_103.forceActive(); // force the Flowing on
    break;

  case DA_HOASwitch::Off:
    DY_103.disable();
    break;

  case DA_HOASwitch::Auto:
    DY_103.enable();
    break;

  default:
    break;
  }
}

void on_SeedingAreaLED_Process(DA_HOASwitch::HOADetectType state)
{
#ifdef PROCESS_TERMINAL_VERBOSE
  *tracePort << "on_NonFlowingLED_Process HS_102AB" << endl;
  HS_102AB.serialize(tracePort, true);
#endif // ifdef PROCESS_TERMINAL_VERBOSE

  switch (state)
  {
  case DA_HOASwitch::Hand:
    DY_102.disable();
    DY_102.forceActive(); // force the Flowing on
    break;

  case DA_HOASwitch::Off:
    DY_102.disable();
    break;

  case DA_HOASwitch::Auto:
    DY_102.enable();
    break;

  default:
    break;
  }
}

void setupRTC()
{
  setSyncProvider(RTC.get);

  // setSyncInterval(30);
  if (timeStatus() != timeSet)
  {
#ifdef PROCESS_TERMINAL
    *tracePort << F("Unable to sync with the RTC-setting to default") << endl;
#endif // ifdef PROCESS_TERMINAL
    RTC.set(DEFAULT_TIME); // set the RTC and the system time to the received
                           // value
    setTime(DEFAULT_TIME); // Sync Arduino clock to the time received on the
                           // Serial2 port
  }
  else
  {
#ifdef PROCESS_TERMINAL
    *tracePort << F("RTC has set the system time") << endl;
    *tracePort << F("Enter Command:") << endl;
#endif // ifdef PROCESS_TERMINAL
  }
}

#ifdef PROCESS_TERMINAL
void printOneWireAddress(HardwareSerial *tracePort,
                         DeviceAddress   aDeviceAddress,
                         bool            aCR)
{
  for (uint8_t i = 0; i < 8; i++)
  {
    // zero pad the address if necessary
    if (aDeviceAddress[i] < 16) *tracePort << '0';
    tracePort->print(aDeviceAddress[i], HEX);
  }

  if (aCR) *tracePort << endl;
}

#endif // ifdef PROCESS_TERMINAL

void initOneWireDevice(DeviceAddress aDevice, uint8_t aIndex)
{
  if (!sensors.getAddress(aDevice, aIndex))
  {
    // TODO Alarm

#ifdef PROCESS_TERMINAL
    *tracePort << "Unable to find address for Device at index " << aIndex <<
      "address:";
    printOneWireAddress(tracePort, aDevice, true);
#endif // ifdef PROCESS_TERMINAL

    sensors.setResolution(aDevice, ONE_TEMPERATURE_PRECISION);
  }
}

void initOneWire()
{
  initOneWireDevice(ambientTemperatureAddress, 0);
  initOneWireDevice(mixtureTemperatureAddress, 1);
}

void doLCDSplashScreen()
{
  lcd.setCursor(2, 0);
  lcd << F("Home Hydroponics");
  lcd.setCursor(6, 2);
  lcd << F("Peter C");
  lcd.setCursor(8, 3);
  lcd << _FLOAT(VERSION / 100.0, 2);
  delay(2000);
  lcd.clear();
}

void timeToBuffer(time_t aTime, char *buffer)
{
  sprintf(buffer, "%02d:%02d:%02d", hour(aTime), minute(aTime), second(aTime));
}

void dateTimeToBuffer(time_t aTime, char *buffer)
{
  sprintf(buffer,
          "%02d/%02d/%04d %02d:%02d",
          month(aTime),
          day(aTime),
          year(aTime),
          hour(aTime),
          minute(aTime));
}

void displayDateTime()
{
  if (timeStatus() != timeNotSet)
  {
    TimeChangeRule *tcr; // pointer to the time change rule, use to get the TZ
                         // abbrev
    time_t atime;
    atime = now();
    atime = usMT.toLocal(atime, &tcr);
    char buffer[20];
    dateTimeToBuffer(atime, buffer);
    lcd << buffer;
  }
}

void displayHomeScreen(bool clearScreen)
{
  if (clearScreen) lcd.clear();
  lcd.home();
  displayDateTime();
  lcd.setCursor(14, 1);
  lcd << "      ";
  lcd.setCursor( 0, 1);
  lcd << F("Mixture Level:") << _FLOAT(LT_002MedianFilter.getMedian(), 1) << "%";
  lcd.setCursor( 0, 2);
  lcd << F("MT:") << _FLOAT(TT_001T, 1) << "C ";
  lcd << F("AT:") << _FLOAT(AT_101T, 1) << "C";
  lcd.setCursor( 0, 3);
  lcd << F("Rel Hum:") << _FLOAT(AT_101H, 1) << "%";

  /*
   * tracePort << "Sonar:cm:" << LT_002Raw << endl;
   * tracePort << "Ambient Temperature:" <<
   *sensors.getTempC(ambientTemperatureAddress) << "C" << endl;
   * tracePort << "Mixture Temperature:" <<
   *sensors.getTempC(mixtureTemperatureAddress) << "C" << endl;
   * tracePort << "HT-101: " << "Rel Humidity:" << AT_101H << " % Temperature:"
   *<< AT_101T;
   * tracePort << " C Heat Index " << AT_101HI << endl;
   * */
}

void displayMiscStatuses(bool clearScreen)
{
  char sprintfBuf[20];

  if (clearScreen) lcd.clear();
  lcd.setCursor( 0, 0);
  lcd << F("FT-002:");
  lcd.setCursor( 7, 0);
  lcd << FT_002.getCurrentFlowRate();
  lcd.setCursor(12, 0);
  lcd << F("L/min");
  lcd.setCursor( 0, 1);
  lcd << F("FT-003:");
  lcd.setCursor( 7, 1);
  lcd << FT_003.getCurrentFlowRate();
  lcd.setCursor(12, 1);
  lcd << F("L/min");
  lcd.setCursor( 0, 2);
  lcd << F("LSHH-002:");
  lcd.setCursor( 9, 2);
  lcd << LSHH_002.getSample();

  lcd.setCursor( 0, 4);
  sprintf(sprintfBuf,
          "AT-102:%04u ppm",
          (unsigned int)AT_102MedianFilter.getMedian());
  lcd << sprintfBuf;
}

void displayHOAStatuses(bool clearScreen)
{
  if (clearScreen) lcd.clear();

  // lcd.home();
  lcd.setCursor( 0, 0);
  lcd << F("   CPMP   GC   SEED");
  lcd.setCursor( 0, 1);
  lcd << "H";
  lcd.setCursor( 4, 1);
  lcd << ((HS_001AB.getCurrentState() == DA_HOASwitch::Hand) ? "X" : " ");
  lcd.setCursor(10, 1);
  lcd << ((HS_103AB.getCurrentState() == DA_HOASwitch::Hand) ? "X" : " ");
  lcd.setCursor(16, 1);
  lcd << ((HS_102AB.getCurrentState() == DA_HOASwitch::Hand) ? "X" : " ");
  lcd.setCursor( 0, 2);
  lcd << "O";
  lcd.setCursor( 4, 2);
  lcd << ((HS_001AB.getCurrentState() == DA_HOASwitch::Off) ? "X" : " ");
  lcd.setCursor(10, 2);
  lcd << ((HS_103AB.getCurrentState() == DA_HOASwitch::Off) ? "X" : " ");
  lcd.setCursor(16, 2);
  lcd << ((HS_102AB.getCurrentState() == DA_HOASwitch::Off) ? "X" : " ");
  lcd.setCursor( 0, 3);
  lcd << "A";
  lcd.setCursor( 4, 3);
  lcd << ((HS_001AB.getCurrentState() == DA_HOASwitch::Auto) ? "X" : " ");
  lcd.setCursor(10, 3);
  lcd << ((HS_103AB.getCurrentState() == DA_HOASwitch::Auto) ? "X" : " ");
  lcd.setCursor(16, 3);
  lcd << ((HS_102AB.getCurrentState() == DA_HOASwitch::Auto) ? "X" : " ");
}

void displayTimerStatuses(bool clearScreen)
{
  char sprintfBuf[7];


  if (clearScreen) lcd.clear();

  // lcd.home();
  lcd.setCursor(0, 0);
  lcd << F("Timers  On    Off");
  lcd.setCursor(0, 1);

  lcd << "MY-101:";
  lcd.setCursor(7, 1);
  sprintf(sprintfBuf, "%05u",
          (unsigned int)(MY_101.getCurrentOnDuration() / 1000));
  lcd << sprintfBuf;

  lcd.setCursor(13, 1);
  sprintf(sprintfBuf,
          "%05u",
          (unsigned int)(MY_101.getCurrentOffDuration() / 1000));
  lcd << sprintfBuf;

  lcd.setCursor(0, 2);
  lcd << "PY-101:";

  lcd.setCursor(7, 2);
  sprintf(sprintfBuf, "%05u",
          (unsigned int)(PY_001.getCurrentOnDuration() / 1000));
  lcd << sprintfBuf;

  lcd.setCursor(13, 2);
  sprintf(sprintfBuf,
          "%05u",
          (unsigned int)(PY_001.getCurrentOffDuration() / 1000));
  lcd << sprintfBuf;
}

#ifdef PROCESS_TERMINAL
void traceAlarmTime(char *label, time_t utcTime)
{
  char sprintfBuf[10];

  time_t localAlarmTime = alarmTimeToLocal(utcTime);

  sprintf(sprintfBuf, "%02d:%02d", hour(localAlarmTime), minute(localAlarmTime));
  *tracePort << label << "=" << sprintfBuf <<
  (isAM(localAlarmTime) == true ? "AM" : "PM") << endl;
}

#endif // ifdef PROCESS_TERMINAL

void displayLightStatuses(bool clearScreen)
{
  char sprintfBuf[10];


  if (clearScreen) lcd.clear();

  // lcd.home();
  lcd.setCursor(0, 0);
  lcd << F("Timers  On     Off");
  lcd.setCursor(0, 1);
  lcd << "D102:";
  time_t localAlarmTime = alarmTimeToLocal(seedingAreaLights.onEpoch);
  lcd.setCursor(5, 1);
  sprintf(sprintfBuf, "%02d:%02d", hour(localAlarmTime), minute(localAlarmTime));
  lcd << sprintfBuf;
  lcd.setCursor(10, 1);
  lcd << (isAM(localAlarmTime) == true ? "AM" : "PM");


  localAlarmTime = alarmTimeToLocal(seedingAreaLights.offEpoch);
  lcd.setCursor(13, 1);
  sprintf(sprintfBuf, "%02d:%02d", hour(localAlarmTime), minute(localAlarmTime));
  lcd << sprintfBuf;
  lcd.setCursor(18, 1);
  lcd << (isAM(localAlarmTime) == true ? "AM" : "PM");


  lcd.setCursor(0, 2);
  lcd << "D103:";
  localAlarmTime = alarmTimeToLocal(growingChamberLights.onEpoch);
  lcd.setCursor(5, 2);
  sprintf(sprintfBuf, "%02d:%02d", hour(localAlarmTime), minute(localAlarmTime));
  lcd << sprintfBuf;
  lcd.setCursor(10, 2);
  lcd << (isAM(localAlarmTime) == true ? "AM" : "PM");

  localAlarmTime = alarmTimeToLocal(growingChamberLights.offEpoch);
  lcd.setCursor(13, 2);
  sprintf(sprintfBuf, "%02d:%02d", hour(localAlarmTime), minute(localAlarmTime));
  lcd << sprintfBuf;
  lcd.setCursor(18, 2);
  lcd << (isAM(localAlarmTime) == true ? "AM" : "PM");
}

void displayPhEC(bool clearScreen)
{}

void setupLCD()
{
  lcd.begin(20, 4);
  lcd.backlight();
  doLCDSplashScreen();
}

void setupLCDScreens()
{
  firstScreen.displayFunc     = displayHomeScreen;
  firstScreen.previousScreen  = &fifthScreen;
  firstScreen.nextScreen      = &secondScreen;
  secondScreen.displayFunc    = displayHOAStatuses;
  secondScreen.previousScreen = &firstScreen;
  secondScreen.nextScreen     = &thirdScreen;
  thirdScreen.displayFunc     = displayMiscStatuses;
  thirdScreen.previousScreen  = &secondScreen;
  thirdScreen.nextScreen      = &fourthScreen;
  fourthScreen.displayFunc    = displayTimerStatuses;
  fourthScreen.previousScreen = &thirdScreen;
  fourthScreen.nextScreen     = &fifthScreen;
  fifthScreen.displayFunc     = displayLightStatuses;
  fifthScreen.previousScreen  = &fourthScreen;
  fifthScreen.nextScreen      = &firstScreen;

  currentScreen = &firstScreen;
}

void onAtlasPhSample(IO_TYPE type, float value)
{
  // AT_001.serialize( tracePort, true);
  currentIOType = i2c_ec;
}

void onAtlasECSample(IO_TYPE type, float value)
{
  // AT_002.serialize( tracePort, true);
  currentIOType = i2c_ph;
}

void refreshDiscreteInputs()
{
  LSHH_002.refresh();
  HS_002.refresh();
  HS_003A.refresh();
  HS_003B.refresh();
  HS_003C.refresh();
  HS_001.refresh();
  HS_001AB.refresh();

  // HS_101AB.refresh();
  HS_102AB.refresh();
  HS_103AB.refresh();
}

void refreshDiscreteOutputs()
{
  PY_001.refresh(); // on/off timer
  MY_101.refresh(); // on/off timer
}

void doOnMidnight()
{
  // FT_002.dayRollOver();
  // *tracePort << "day Rollover:";
  // FT_002.serialize(tracePort, true);
}

void refreshLCD()
{
  currentScreen->displayFunc(false);
}

void doOnCalcFlowRate()
{
  DISABLE_FT002_SENSOR_INTERRUPTS;
  FT_002.end();
  FT_002.begin();
  DISABLE_FT003_SENSOR_INTERRUPTS;
  FT_003.end();
  FT_003.begin();
  refreshLCD();
  AT_102MedianFilter.add(AT_102Raw);
  ENABLE_FT002_SENSOR_INTERRUPTS;
  ENABLE_FT003_SENSOR_INTERRUPTS;
}

bool isTimeForLightsOn(time_t currentEpoch, time_t offEpoch, time_t onEpoch)
{
  int  curTimeInMins = hour(currentEpoch) * 60 + minute(currentEpoch);
  int  offTimeInMins = hour(offEpoch) * 60 + minute(offEpoch);
  int  onTimeInMins  = hour(onEpoch) * 60 + minute(onEpoch);
  bool isOnEvent     = false;

  if (offTimeInMins < onTimeInMins)
  {
    if (((curTimeInMins >= onTimeInMins) && (curTimeInMins >= offTimeInMins)) ||
        ((curTimeInMins <= onTimeInMins) &&
         (curTimeInMins <= offTimeInMins))) isOnEvent = true;
  }
  else
  {
    if ((curTimeInMins >= onTimeInMins) &&
        (curTimeInMins <= offTimeInMins)) isOnEvent = true;
  }

  return isOnEvent;
}

void doLightControl(struct _lightControlEntry *aLightControlEntry)
{
  if (isTimeForLightsOn(alarmTimeToUTC(now()),
                        alarmTimeToUTC(aLightControlEntry->offEpoch),
                        alarmTimeToUTC(aLightControlEntry->onEpoch)))
    aLightControlEntry->onLightsOn();
  else aLightControlEntry->onLightsOff();
}

// alternate which sensor to compensate
//
bool altenateProbe = false;
void doOnTemperatureCompensate()
{
  if (altenateProbe)
  {
    currentIOType = i2c_ec;
    AT_002.setCompensatedTemperature(TT_001T);
  }
  else
  {
    currentIOType = i2c_ph;
    AT_001.setCompensatedTemperature(TT_001T);
  }
  altenateProbe = !altenateProbe;
}

// update sonar and 1-wire DHT-22 readings
void do_ONP_SPoll()
{
  float tLevel;

  doLightControl(&growingChamberLights);
  doLightControl(&seedingAreaLights);


  unsigned int distanceCM = LT_002.ping() / US_ROUNDTRIP_CM -
                            NUTRIENT_TANK_AIR_GAP;

  // compute distanace from high level mark
  tLevel = (NUTRIENT_TANK_MIXTURE_MAX - distanceCM) /
           NUTRIENT_TANK_MIXTURE_MAX; // NUTRIENT_TANK_MIXTURE_MAX;
  tLevel   *= 100.0;
  LT_002Raw = tLevel;
  LT_002MedianFilter.add(LT_002Raw);
  sensors.requestTemperatures();
  AT_101H = AT_101.readHumidity(); // allow 1/4 sec to read
  AT_101T = AT_101.readTemperature();
  TT_001T = sensors.getTempC(mixtureTemperatureAddress);

  if (isnan(AT_101H) || isnan(AT_101T))
  {
#ifdef PROCESS_TERMINAL

    // *tracePort << "Error reading from DHT-22 sensor." << endl;
#endif // ifdef PROCESS_TERMINAL
  }
  else
  {
    AT_101HI = AT_101.computeHeatIndex(AT_101T, AT_101H, false);
  }

  // ENABLE_CO2_SENSOR_RISING_INTERRUPTS;
}

void doReadInputs()
{
  // TE_001.refresh();
  // QE_001.refresh();
}

void doUpdateOutputs()
{}

void doGrowingChamberLightsOn()
{
  DY_103.activate(); // if disabled, it won't activate
  // DY_103.activate(); // if disabled, it won't activate

#ifdef PROCESS_TERMINAL

  // *tracePort << "...Growing Chamber Lights on" << endl;
#endif // ifdef PROCESS_TERMINAL
}

void doGrowingChamberLightsOff()
{
  DY_103.reset();

  // DY_103.reset();

#ifdef PROCESS_TERMINAL

  // *tracePort << "...Growing Chamber Lights off" << endl;
#endif // ifdef PROCESS_TERMINAL
}

void doSeedingAreaLightsOn()
{
  DY_102.activate(); // if disabled, it won't activate
  // DY_103.activate(); // if disabled, it won't activate

#ifdef PROCESS_TERMINAL

  // *tracePort << "...Seeding Lights on" << endl;
#endif // ifdef PROCESS_TERMINAL
}

void doSeedingAreaLightsOff()
{
  DY_102.reset();

  // DY_103.reset();

#ifdef PROCESS_TERMINAL

  // *tracePort << "...Seeding Lights off" << endl;
#endif // ifdef PROCESS_TERMINAL
}

// No-op
time_t alarmTimeToUTC(time_t localAlarmTime)
{
  // time_t utcAlarmTime = usMT.toUTC(localAlarmTime);

  /*
     if (usMT.utcIsDST(now()))
     {
     utcAlarmTime -= 60 * 60;
     }
     if (utcAlarmTime > SECS_PER_DAY)
     utcAlarmTime -= SECS_PER_DAY;
   */

  // *tracePort << "alarmTimeToUTC:localAlarmTime="  <<  localAlarmTime << "
  // UTC=" << utcAlarmTime << endl;
  return localAlarmTime;
}

time_t alarmTimeToLocal(time_t utcAlarmTime)
{
  TimeChangeRule *tcr; // pointer to the time change rule, use to get the TZ
                       // abbrev
  time_t localAlarmTime = usMT.toLocal(utcAlarmTime, &tcr);

  /*
     int offset = tcr -> offset; // in minutes
     //if (usMT.utcIsDST(now()))
     // {
     offset += 60;
     localAlarmTime += 60 * 60;
     //}


     if ((hour(utcAlarmTime) + (int) offset / 60) < 0)
     {
     int hr = 12 + (hour(utcAlarmTime) - (int) offset / 60);

     localAlarmTime = AlarmHMS(hr, minute(utcAlarmTime), second(utcAlarmTime));
     }
   */
  return localAlarmTime;
}

/*
** Modbus related functions
*/
void refreshModbusRegisters()
{
  writeModbusCoil(COIL_STATUS_READ_WRITE_OFFSET, CS_HS_002, HS_002.getSample());
  writeModbusCoil(COIL_STATUS_READ_WRITE_OFFSET, CS_LSHH_002,
                  LSHH_002.getSample());
  writeModbusCoil(COIL_STATUS_READ_WRITE_OFFSET, CS_HS_001, HS_001.getSample());
  writeModbusCoil(COIL_STATUS_READ_WRITE_OFFSET, CW_DY_102, DY_102.isActive());
  writeModbusCoil(COIL_STATUS_READ_WRITE_OFFSET, CW_DY_103, DY_103.isActive());
  writeModbusCoil(COIL_STATUS_READ_WRITE_OFFSET, CW_PY_001, PY_001.isActive());
  writeModbusCoil(COIL_STATUS_READ_WRITE_OFFSET, CW_MY_101, MY_101.isActive());


  bfconvert.val                  = AT_101H;
  modbusRegisters[HR_AT_101]     = bfconvert.regsf[0];
  modbusRegisters[HR_AT_101 + 1] = bfconvert.regsf[1];

  bfconvert.val                  = AT_101T;
  modbusRegisters[HR_TT_101]     = bfconvert.regsf[0];
  modbusRegisters[HR_TT_101 + 1] = bfconvert.regsf[1];

  bfconvert.val                  = LT_002MedianFilter.getMedian();
  modbusRegisters[HR_LT_002]     = bfconvert.regsf[0];
  modbusRegisters[HR_LT_002 + 1] = bfconvert.regsf[1];

  bfconvert.val                  = FT_002.getCurrentFlowRate();
  modbusRegisters[HR_FT_002]     = bfconvert.regsf[0];
  modbusRegisters[HR_FT_002 + 1] = bfconvert.regsf[1];

  bfconvert.val                  = FT_003.getCurrentFlowRate();
  modbusRegisters[HR_FT_003]     = bfconvert.regsf[0];
  modbusRegisters[HR_FT_003 + 1] = bfconvert.regsf[1];


  bfconvert.val                  = TT_001T; // sensors.getTempC(mixtureTemperatureAddress);
  modbusRegisters[HR_TT_001]     = bfconvert.regsf[0];
  modbusRegisters[HR_TT_001 + 1] = bfconvert.regsf[1];


  modbusRegisters[HR_HS_001HOA] = HS_001AB.getCurrentState();
  modbusRegisters[HR_HS_102HOA] = HS_102AB.getCurrentState();
  modbusRegisters[HR_HS_103HOA] = HS_103AB.getCurrentState();
  modbusRegisters[HR_KY_002]    = VERSION;


  blconvert.val                         = seedingAreaLights.onEpoch;
  modbusRegisters[HR_DY_102_ONT_CV]     = blconvert.regsl[0];
  modbusRegisters[HR_DY_102_ONT_CV + 1] = blconvert.regsl[1];

  blconvert.val                         = seedingAreaLights.offEpoch;
  modbusRegisters[HR_DY_102_OFT_CV]     = blconvert.regsl[0];
  modbusRegisters[HR_DY_102_OFT_CV + 1] = blconvert.regsl[1];

  blconvert.val                         = growingChamberLights.onEpoch;
  modbusRegisters[HR_DY_103_ONT_CV]     = blconvert.regsl[0];
  modbusRegisters[HR_DY_103_ONT_CV + 1] = blconvert.regsl[1];

  blconvert.val                         = growingChamberLights.offEpoch;
  modbusRegisters[HR_DY_103_OFT_CV]     = blconvert.regsl[0];
  modbusRegisters[HR_DY_103_OFT_CV + 1] = blconvert.regsl[1];

  modbusRegisters[HR_MY_101_ONP_CV] = MY_101.getCurrentOnDuration() / 1000;
  modbusRegisters[HR_MY_101_OFP_CV] = MY_101.getCurrentOffDuration() / 1000;
  modbusRegisters[HR_PY_001_ONP_CV] = PY_001.getCurrentOnDuration() / 1000;
  modbusRegisters[HR_PY_001_OFP_CV] = PY_001.getCurrentOffDuration() / 1000;

  // CO2
  modbusRegisters[HR_AT_102] = (unsigned int)AT_102MedianFilter.getMedian();

  // pH
  bfconvert.val                  = AT_001.getSample();
  modbusRegisters[HR_AT_001]     = bfconvert.regsf[0];
  modbusRegisters[HR_AT_001 + 1] = bfconvert.regsf[1];

  bfconvert.val                     = AT_001.getCompensatedTemperature();
  modbusRegisters[HR_AT_001_TV]     = bfconvert.regsf[0];
  modbusRegisters[HR_AT_001_TV + 1] = bfconvert.regsf[1];


  // EC
  bfconvert.val                  = AT_002.getSample();
  modbusRegisters[HR_AT_002]     = bfconvert.regsf[0];
  modbusRegisters[HR_AT_002 + 1] = bfconvert.regsf[1];

  bfconvert.val                     = AT_002.getTDS();
  modbusRegisters[HR_AT_002TDS]     = bfconvert.regsf[0];
  modbusRegisters[HR_AT_002TDS + 1] = bfconvert.regsf[1];

  bfconvert.val                     = AT_002.getSalinity();
  modbusRegisters[HR_AT_002SAL]     = bfconvert.regsf[0];
  modbusRegisters[HR_AT_002SAL + 1] = bfconvert.regsf[1];

  bfconvert.val                    = AT_002.getSpecificGravity();
  modbusRegisters[HR_AT_002SG]     = bfconvert.regsf[0];
  modbusRegisters[HR_AT_002SG + 1] = bfconvert.regsf[1];

  bfconvert.val                     = AT_002.getCompensatedTemperature();
  modbusRegisters[HR_AT_002_TV]     = bfconvert.regsf[0];
  modbusRegisters[HR_AT_002_TV + 1] = bfconvert.regsf[1];
}

void setModbusTime()
{
  if (modbusRegisters[HW_QT_001] != 0)
  {
    unsigned long pctime;
    blconvert.regsl[0] = modbusRegisters[HW_QT_001];
    blconvert.regsl[1] = modbusRegisters[HW_QT_001 + 1];
    pctime             = blconvert.val;
    RTC.set(pctime); // set the RTC and the system time to the received value
    setTime(pctime); // Sync Arduino clock to the time received on the Serial2
                     // port
    modbusRegisters[HW_QT_001]     = 0;
    modbusRegisters[HW_QT_001 + 1] = 0;
  }
}

void setModbusGrowingChamberLightsOnTime()
{
  if (modbusRegisters[HW_DY_103_ONT_SP] != 0)
  {
    blconvert.regsl[0]           = modbusRegisters[HW_DY_103_ONT_SP];
    blconvert.regsl[1]           = modbusRegisters[HW_DY_103_ONT_SP + 1];
    growingChamberLights.onEpoch = alarmTimeToUTC(blconvert.val);
    EEPROMWriteAlarmEntry(growingChamberLights.onEpoch,
                          EEPROM_GROWING_CHAMBER_ON_TIME_ADDR);
    modbusRegisters[HW_DY_103_ONT_SP]     = 0;
    modbusRegisters[HW_DY_103_ONT_SP + 1] = 0;
  }
}

void setModbusGrowingChamberLightsOffTime()
{
  if (modbusRegisters[HW_DY_103_OFT_SP] != 0)
  {
    blconvert.regsl[0]            = modbusRegisters[HW_DY_103_OFT_SP];
    blconvert.regsl[1]            = modbusRegisters[HW_DY_103_OFT_SP + 1];
    growingChamberLights.offEpoch = alarmTimeToUTC(blconvert.val);
    EEPROMWriteAlarmEntry(growingChamberLights.offEpoch,
                          EEPROM_GROWING_CHAMBER_OFF_TIME_ADDR);
    modbusRegisters[HW_DY_103_OFT_SP]     = 0;
    modbusRegisters[HW_DY_103_OFT_SP + 1] = 0;
  }
}

void setModbusSeedingAreaLightsOnTime()
{
  if (modbusRegisters[HW_DY_102_ONT_SP] != 0)
  {
    blconvert.regsl[0]        = modbusRegisters[HW_DY_102_ONT_SP];
    blconvert.regsl[1]        = modbusRegisters[HW_DY_102_ONT_SP + 1];
    seedingAreaLights.onEpoch = alarmTimeToUTC(blconvert.val);
    EEPROMWriteAlarmEntry(seedingAreaLights.onEpoch,
                          EEPROM_SEEDING_AREA_ON_TIME_ADDR);
    modbusRegisters[HW_DY_102_ONT_SP]     = 0;
    modbusRegisters[HW_DY_102_ONT_SP + 1] = 0;
  }
}

void setModbusSeedingAreaLightsOffTime()
{
  if (modbusRegisters[HW_DY_102_OFT_SP] != 0)
  {
    blconvert.regsl[0] = modbusRegisters[HW_DY_102_OFT_SP];
    blconvert.regsl[1] = modbusRegisters[HW_DY_102_OFT_SP + 1];

    seedingAreaLights.offEpoch = alarmTimeToUTC(blconvert.val);
    EEPROMWriteAlarmEntry(seedingAreaLights.offEpoch,
                          EEPROM_SEEDING_AREA_OFF_TIME_ADDR);
    modbusRegisters[HW_DY_102_OFT_SP]     = 0;
    modbusRegisters[HW_DY_102_OFT_SP + 1] = 0;
  }
}

void setModbusFanOnDuration()
{
  if (modbusRegisters[HW_MY_101_ONP_SP] != 0)
  {
    MY_101.setOnDuration(modbusRegisters[HW_MY_101_ONP_SP]);

    EEPROMWriteDuration(modbusRegisters[HW_MY_101_ONP_SP],
                        EEPROM_FAN_ON_DURATION_ADDR);
    modbusRegisters[HW_MY_101_ONP_SP] = 0;
  }
}

void setModbusFanOffDuration()
{
  if (modbusRegisters[HW_MY_101_OFP_SP] != 0)
  {
    MY_101.setOffDuration(modbusRegisters[HW_MY_101_OFP_SP]);
    EEPROMWriteDuration(modbusRegisters[HW_MY_101_OFP_SP],
                        EEPROM_FAN_OFF_DURATION_ADDR);
    modbusRegisters[HW_MY_101_ONP_SP] = 0;
  }
}

void setModbusCirculationPumpOnDuration()
{
  if (modbusRegisters[HW_PY_001_ONP_SP] != 0)
  {
    PY_001.setOnDuration(modbusRegisters[HW_PY_001_ONP_SP]);

    EEPROMWriteDuration(modbusRegisters[HW_PY_001_ONP_SP],
                        EEPROM_CIRCULATION_PUMP_ON_DURATION_ADDR);
    modbusRegisters[HW_PY_001_ONP_SP] = 0;
  }
}

void setModbusCirculationPumpOffDuration()
{
  if (modbusRegisters[HW_PY_001_OFP_SP] != 0)
  {
    PY_001.setOffDuration(modbusRegisters[HW_PY_001_OFP_SP]);

    EEPROMWriteDuration(modbusRegisters[HW_PY_001_OFP_SP],
                        EEPROM_CIRCULATION_PUMP_OFF_DURATION_ADDR);
    modbusRegisters[HW_PY_001_OFP_SP] = 0;
  }
}

// force on or off if only in Auto
void setModbusForceSeedingAreaLightsOn()
{
  if (getModbusCoilValue(COIL_STATUS_READ_WRITE_OFFSET, CW_DY_102_ON))
  {
    DY_102.activate();
    writeModbusCoil(COIL_STATUS_READ_WRITE_OFFSET, CW_DY_102_ON, false);
  }
}

// force on or off if only in Auto
void setModbusForceSeedingAreaLightsOff()
{
  if (getModbusCoilValue(COIL_STATUS_READ_WRITE_OFFSET, CW_DY_102_OFF))
  {
    DY_102.reset();
    writeModbusCoil(COIL_STATUS_READ_WRITE_OFFSET, CW_DY_102_OFF, false);
  }
}

// force on or off if only in Auto
void setModbusForceGrowingChamberLightsOn()
{
  if (getModbusCoilValue(COIL_STATUS_READ_WRITE_OFFSET, CW_DY_103_ON))
  {
    DY_103.activate();
    writeModbusCoil(COIL_STATUS_READ_WRITE_OFFSET, CW_DY_103_ON, false);
  }
}

// force on or off if only in Auto
void setModbusForceGrowingChamberLightsOff()
{
  if (getModbusCoilValue(COIL_STATUS_READ_WRITE_OFFSET, CW_DY_103_OFF))
  {
    DY_103.reset();
    writeModbusCoil(COIL_STATUS_READ_WRITE_OFFSET, CW_DY_103_OFF, false);
  }
}

void setConfigToDefaults()
{
  if (getModbusCoilValue(COIL_STATUS_READ_WRITE_OFFSET, CW_KY_001))
  {
    EEPROMWriteDefaultConfig();
    EEPROMLoadConfig();
    writeModbusCoil(COIL_STATUS_READ_WRITE_OFFSET, CW_KY_001, false);
  }
}

bool getModbusCoilValue(unsigned short startAddress, unsigned short bitPos)
{
  return bitRead(modbusRegisters[startAddress + (int)(bitPos / 16)], bitPos % 16);
}

void writeModbusCoil(unsigned short startAddress,
                     unsigned short bitPos,
                     bool           value)
{
  bitWrite(modbusRegisters[startAddress + (int)(bitPos / 16)], bitPos % 16,
           value);
}

void processModbusECCommands()
{
  bool status;

  // calibrate low command
  if (getModbusCoilValue(COIL_STATUS_READ_WRITE_OFFSET, CW_AT_002_CL))
  {
    status                        = AT_002.calibrateLow();
    modbusRegisters[HR_AT_002_CS] = AT_002.getProbeCommandStatus();
    writeModbusCoil(COIL_STATUS_READ_WRITE_OFFSET, CW_AT_002_CL, false);
  }
  else
  if (getModbusCoilValue(COIL_STATUS_READ_WRITE_OFFSET, CW_AT_002_CH))
  {
    status                        = AT_002.calibrateHigh();
    modbusRegisters[HR_AT_002_CS] = AT_002.getProbeCommandStatus();
    writeModbusCoil(COIL_STATUS_READ_WRITE_OFFSET, CW_AT_002_CH, false);
  }
  else
  if (getModbusCoilValue(COIL_STATUS_READ_WRITE_OFFSET, CW_AT_002_CD))
  {
    status                        = AT_002.calibrateDry();
    modbusRegisters[HR_AT_002_CS] = AT_002.getProbeCommandStatus();
    writeModbusCoil(COIL_STATUS_READ_WRITE_OFFSET, CW_AT_002_CD, false);
  }
  else
  if (getModbusCoilValue(COIL_STATUS_READ_WRITE_OFFSET, CW_AT_002_CR))
  {
    status                        = AT_002.calibrateClear();
    modbusRegisters[HR_AT_002_CS] = AT_002.getProbeCommandStatus();
    writeModbusCoil(COIL_STATUS_READ_WRITE_OFFSET, CW_AT_002_CR, false);
  }
  else
  if (getModbusCoilValue(COIL_STATUS_READ_WRITE_OFFSET, CW_AT_002_CQ))
  {
    int calCount = AT_002.calibrateQuery();
    modbusRegisters[HR_AT_002_CS] = AT_002.getProbeCommandStatus();
    modbusRegisters[HR_AT_002_CC] = calCount;
    writeModbusCoil(COIL_STATUS_READ_WRITE_OFFSET, CW_AT_002_CQ, false);
  }
  else
  if (getModbusCoilValue(COIL_STATUS_READ_WRITE_OFFSET, CW_AT_002_TC))
  {
    currentIOType = i2c_ec;
    AT_002.setCompensatedTemperature(TT_001T);
    writeModbusCoil(COIL_STATUS_READ_WRITE_OFFSET, CW_AT_002_TC, false);
  }
}

void processModbusPHCommands()
{
  bool status;

  // calibrate low command
  if (getModbusCoilValue(COIL_STATUS_READ_WRITE_OFFSET, CW_AT_001_CL))
  {
    status                        = AT_001.calibrateLow();
    modbusRegisters[HR_AT_001_CS] = AT_001.getProbeCommandStatus();
    writeModbusCoil(COIL_STATUS_READ_WRITE_OFFSET, CW_AT_001_CL, false);
  }
  else
  if (getModbusCoilValue(COIL_STATUS_READ_WRITE_OFFSET, CW_AT_001_CH))
  {
    status                        = AT_001.calibrateHigh();
    modbusRegisters[HR_AT_001_CS] = AT_001.getProbeCommandStatus();
    writeModbusCoil(COIL_STATUS_READ_WRITE_OFFSET, CW_AT_001_CH, false);
  }
  else
  if (getModbusCoilValue(COIL_STATUS_READ_WRITE_OFFSET, CW_AT_001_CM))
  {
    status                        = AT_001.calibrateMid();
    modbusRegisters[HR_AT_001_CS] = AT_001.getProbeCommandStatus();
    writeModbusCoil(COIL_STATUS_READ_WRITE_OFFSET, CW_AT_001_CM, false);
  }
  else
  if (getModbusCoilValue(COIL_STATUS_READ_WRITE_OFFSET, CW_AT_001_CR))
  {
    status                        = AT_001.calibrateClear();
    modbusRegisters[HR_AT_001_CS] = AT_001.getProbeCommandStatus();
    writeModbusCoil(COIL_STATUS_READ_WRITE_OFFSET, CW_AT_001_CR, false);
  }
  else
  if (getModbusCoilValue(COIL_STATUS_READ_WRITE_OFFSET, CW_AT_001_CQ))
  {
    int calCount = AT_001.calibrateQuery();
    modbusRegisters[HR_AT_001_CS] = AT_001.getProbeCommandStatus();
    modbusRegisters[HR_AT_001_CC] = calCount;
    writeModbusCoil(COIL_STATUS_READ_WRITE_OFFSET, CW_AT_001_CQ, false);
  }
  else
  if (getModbusCoilValue(COIL_STATUS_READ_WRITE_OFFSET, CW_AT_001_TC))
  {
    currentIOType = i2c_ec;
    AT_001.setCompensatedTemperature(TT_001T);
    writeModbusCoil(COIL_STATUS_READ_WRITE_OFFSET, CW_AT_001_TC, false);
  }
}

void processModbusCommands()
{
  setModbusTime();

  setModbusGrowingChamberLightsOnTime();
  setModbusGrowingChamberLightsOffTime();
  setModbusSeedingAreaLightsOnTime();
  setModbusSeedingAreaLightsOffTime();

  setModbusFanOnDuration();
  setModbusFanOffDuration();

  setModbusCirculationPumpOnDuration();
  setModbusCirculationPumpOffDuration();

  // setModbusLightsOff();
  setConfigToDefaults();
  setModbusForceGrowingChamberLightsOn();
  setModbusForceGrowingChamberLightsOff();
  setModbusForceSeedingAreaLightsOn();
  setModbusForceSeedingAreaLightsOff();

  // atlas sensors
  processModbusPHCommands();
  processModbusECCommands();
}

/*
** EEPROM functions
*/
void EEPROMWriteDefaultConfig()
{
  unsigned short configFlag = EEPROM_CONFIGURED;

  eeprom_write_block((const void *)&configFlag,
                     (void *)EEPROM_CONFIG_FLAG_ADDR,
                     sizeof(configFlag));

  time_t epoch = alarmTimeToUTC(DEFAULT_LIGHTS_ON_ALARM_TIME);
  EEPROMWriteAlarmEntry(epoch, EEPROM_GROWING_CHAMBER_ON_TIME_ADDR);
  EEPROMWriteAlarmEntry(epoch, EEPROM_SEEDING_AREA_ON_TIME_ADDR);

  epoch = alarmTimeToUTC(DEFAULT_LIGHTS_OFF_ALARM_TIME);
  EEPROMWriteAlarmEntry(epoch, EEPROM_GROWING_CHAMBER_OFF_TIME_ADDR);
  EEPROMWriteAlarmEntry(epoch, EEPROM_SEEDING_AREA_OFF_TIME_ADDR);

  EEPROMWriteDuration(              DEFAULT_FAN_ON_DURATION,
                                    EEPROM_FAN_ON_DURATION_ADDR);
  EEPROMWriteDuration(             DEFAULT_FAN_OFF_DURATION,
                                    EEPROM_FAN_OFF_DURATION_ADDR);

  EEPROMWriteDuration( DEFAULT_CIRCULATION_PUMP_ON_DURATION,
                                    EEPROM_CIRCULATION_PUMP_ON_DURATION_ADDR);
  EEPROMWriteDuration(DEFAULT_CIRCULATION_PUMP_OFF_DURATION,
                                    EEPROM_CIRCULATION_PUMP_OFF_DURATION_ADDR);
}

void EEPROMWriteDuration(unsigned int duration, unsigned int atAddress)
{
  eeprom_write_block((const void *)&duration, (void *)atAddress,
                     sizeof(duration));
}

unsigned int EEPROMReadDuration(unsigned int atAddress)
{
  unsigned int duration = 0;

  eeprom_read_block((void *)&duration, (void *)atAddress, sizeof(duration));
  return duration;
}

void EEPROMWriteAlarmEntry(time_t epoch, unsigned int atAddress)
{
  eeprom_write_block((const void *)&epoch, (void *)atAddress, sizeof(epoch));
}

time_t EEPROMReadAlarmEntry(unsigned int atAddress)
{
  time_t epoch = 0;

  eeprom_read_block((void *)&epoch, (void *)atAddress, sizeof(epoch));
  return epoch;
}

unsigned int isEEPROMConfigured()
{
  unsigned short configFlag;

  eeprom_read_block((void *)&configFlag, (void *)EEPROM_CONFIG_FLAG_ADDR,
                    sizeof(configFlag));
  return configFlag;
}

void EEPROMLoadConfig()
{
  growingChamberLights.offEpoch = EEPROMReadAlarmEntry(
    EEPROM_GROWING_CHAMBER_OFF_TIME_ADDR);
  growingChamberLights.onLightsOff =  doGrowingChamberLightsOff;

  growingChamberLights.onEpoch = EEPROMReadAlarmEntry(
    EEPROM_GROWING_CHAMBER_ON_TIME_ADDR);
  growingChamberLights.onLightsOn =  doGrowingChamberLightsOn;

  seedingAreaLights.offEpoch = EEPROMReadAlarmEntry(
    EEPROM_SEEDING_AREA_OFF_TIME_ADDR);
  seedingAreaLights.onLightsOff = doSeedingAreaLightsOff;

  seedingAreaLights.onEpoch = EEPROMReadAlarmEntry(
    EEPROM_SEEDING_AREA_ON_TIME_ADDR);
  seedingAreaLights.onLightsOn = doSeedingAreaLightsOn;

  PY_001.setOnDuration(EEPROMReadDuration(
                         EEPROM_CIRCULATION_PUMP_ON_DURATION_ADDR));
  PY_001.setOffDuration(EEPROMReadDuration(
                          EEPROM_CIRCULATION_PUMP_OFF_DURATION_ADDR));

  MY_101.setOnDuration(EEPROMReadDuration(EEPROM_FAN_ON_DURATION_ADDR));
  MY_101.setOffDuration(EEPROMReadDuration(EEPROM_FAN_OFF_DURATION_ADDR));

  // Alarm.free( dutyCycleChangeAlarm.id );
  // dutyCycleChangeAlarm.id = Alarm.alarmRepeat(dutyCycleChangeAlarm.epoch,
  // alterLEDPattern);
}

#ifdef PROCESS_TERMINAL

// single character message tags from terminal host
# define TIME_HEADER 'T'    // Header tag for Serial2 time sync message
# define DISPLAY_HEADER 'D' // Display header tag
# define DISPLAY_TIME 't'
# define DISPLAY_ALARMS 'a'
# define DISPLAY_DUTY_CYCLE 'd'
# define TIMER_GROWING_CHAMBER_HEADER 'G'  // Growing Chamber Alarm Header tag
# define TIMER_SEEDING_AREA_HEADER 'A'     // Seeding Area Alarm Header tag
# define TIMER_ALARM_ON '1'                // Timer Alarm On A14:00:00->turn on
                                           // lights at 4AM
# define TIMER_ALARM_OFF '0'               // Timer Alarm On A023:00:00->turn
                                           // off lighst at 11 PM
# define HELP_HEADER '?'
# define LIGHT_HEADER 'L'                  // Light Header tag
# define LIGHTS_ON '1'                     // Turn lights on L1
# define LIGHTS_OFF '0'                    // Turn lights off L0
# define CIRCULATION_PUMP_HEADER 'C'       // Light Header tag
# define FAN_HEADER 'F'                    // Light Header tag
# define TEMPERATURE_COMPENSATE_HEADER 'E' // Atlas Sensor Tempearture
                                           // Compensate
# define CALIBRATE_EC_HEADER 'Y'
# define CALIBRATE_PH_HEADER 'Z'


# define LIGHTS_ON '1'                // Turn lights on L1
# define LIGHTS_OFF '0'               // Turn lights off L0

// efine LIGHTS_TOOGLE 't'         // toggle
# define LIGHTS_DUTY_CYCLE 'd'        // Ld60->60% dominant color default mostly
                                      // red 60-90 percent allowed
# define LIGHTS_DUTY_CYCLE_PERIOD 'r' // Lr3600->change the duty cycle between
                                      // 60-90 % every hour
# define LIGHTS_RESET_TO_DEFAULTS 'c'
# define IO_HEADER 'I'                // read IO points
# define IO_AMBIENT_TEMP 't'
# define IO_SOIL_MOISTURE 'm'
# define SERIALIZE_HEADER 'S'
# define SERIALIZE_CIRCULATION_PUMP 'c'
# define SERIALIZE_CIRCULATION_FAN 'f'
# define SERIALIZE_PH 'p'
# define SERIALIZE_EC 'e'

# define CALIBRATE_LOW 'l'    // EC, pH
# define CALIBRATE_MID 'm'    // pH
# define CALIBRATE_HIGH 'h'   // EC, pH
# define CALIBRATE_DRY  'd'   // EC
# define CALIBRATE_CLEAR  'c' // EC,pH
# define CALIBRATE_QUERY  'q' // EC,pH


void displayAlarm(char *who, struct _AlarmEntry aAlarmEntry)
{
  char buffer[STRLEN("HH:MM:SS")];

  timeToBuffer(aAlarmEntry.epoch, buffer);

  *tracePort << who << " id:" << aAlarmEntry.id << " UTC:" << buffer <<
    " epoch:" << aAlarmEntry.epoch;


  time_t localAlarmTime = alarmTimeToLocal(aAlarmEntry.epoch);
  timeToBuffer(localAlarmTime, buffer);
  *tracePort << " Local:" << buffer << " epoch:" << localAlarmTime << endl;
}

void displayLightControlEntry(char *who, time_t anEpoch)
{
  char buffer[STRLEN("HH:MM:SS")];

  timeToBuffer(anEpoch, buffer);

  *tracePort << who <<  " UTC:" << buffer << " epoch:" << anEpoch;


  time_t localAlarmTime = alarmTimeToLocal(anEpoch);
  timeToBuffer(localAlarmTime, buffer);
  *tracePort << " Local:" << buffer << " epoch:" << localAlarmTime << endl;
}

void processDisplayIOMessage()
{
  char c = tracePort->read();

  if (c == IO_AMBIENT_TEMP)
  {
    // float h = AT_101.readHumidity(); // allow 1/4 sec to read
    // float t = AT_101.readTemperature();
    *tracePort << "Sonar:cm:" << LT_002Raw << endl;
    *tracePort << "Ambient Temperature:" << sensors.getTempC(
      ambientTemperatureAddress) << "C" << endl;
    *tracePort << "Mixture Temperature:" << sensors.getTempC(
      mixtureTemperatureAddress) << "C" << endl;
    *tracePort << "HT-101: " << "Rel Humidity:" << AT_101H << " % Temperature:" <<
      AT_101T;
    *tracePort << " C Heat Index " << AT_101HI << endl;
    *tracePort << " LT_002Raw= " << LT_002Raw << endl;

    // AT_101.serialize( tracePort, true);
    // *tracePort << "Abient Temp = " << TE_001.getScaledSample() << " C" <<
    // endl;
    // float hic = AT_101.computeHeatIndex(t, h, false);
    // *tracePort << "DHT-22 Relative Humidity:" << h << " Temperature:" << t <<
    // " C Humidex:" << hic << endl;
  }
  else
  if (c == IO_SOIL_MOISTURE)
  {
    // *tracePort << "Soil Moisture = " << QE_001.getScaledSample() << "%" <<
    // endl;
  }
}

void processDisplayMessage()
{
  char c = tracePort->read();

  if (c == DISPLAY_TIME)
  {
    char buffer[STRLEN("MM/DD/YYYY HH:MM")];
    TimeChangeRule *tcr; // pointer to the time change rule, use to get the TZ
                         // abbrev
    time_t atime = now();
    atime = usMT.toLocal(atime, &tcr);
    dateTimeToBuffer(atime, buffer);
    *tracePort << buffer << endl;
  }
  else
  if (c == DISPLAY_ALARMS)
  {
    *tracePort << endl;
    displayLightControlEntry("...GC Off", growingChamberLights.offEpoch);
    displayLightControlEntry( "...GC On", growingChamberLights.onEpoch);
    displayLightControlEntry("...SA Off", seedingAreaLights.offEpoch);
    displayLightControlEntry( "...SA On", seedingAreaLights.onEpoch);
    displayAlarm("...Reset Midnight", onMidnight);

    // *tracePort << "Duty Cycle Period = " << dutyCycleChangeAlarm.epoch << " s
    // id=" << dutyCycleChangeAlarm.id << endl;
  }
  else
  if (c == DISPLAY_DUTY_CYCLE)
  {
    // *tracePort << "Duty Cycle = " << plantStrip.getDutyCycle() << endl;
  }
}

void processLightsMessage()
{
  char c = tracePort->read();

  switch (c)
  {
  case LIGHTS_ON:
    doGrowingChamberLightsOn();
    break;

  case LIGHTS_OFF:
    doGrowingChamberLightsOff();
    break;

  case LIGHTS_DUTY_CYCLE:

    // plantStrip.setDutyCycle( tracePort->parseInt());
    break;

  case LIGHTS_RESET_TO_DEFAULTS:
    EEPROMWriteDefaultConfig();
    EEPROMLoadConfig();
    *tracePort << F("Settings set to Defaults") << endl;
    break;

  default:
    break;
  }
}

void processAlarmEntryMessage(struct _AlarmEntry *aAlarmEntry,
                              unsigned int        eepromAddr,
                              OnTick_t            alarmHandler)
{
  Alarm.free(aAlarmEntry->id);
  unsigned int shour   = tracePort->parseInt();
  unsigned int sminute = tracePort->parseInt();
  unsigned int ssecond = tracePort->parseInt();

  aAlarmEntry->epoch = alarmTimeToUTC(AlarmHMS(shour, sminute, ssecond));
  aAlarmEntry->id    = Alarm.alarmRepeat(aAlarmEntry->epoch, alarmHandler);

  // Serial << "alarm epoch " << aAlarmEntry->epoch << " alarm ID " <<
  // aAlarmEntry->id <<endl;
  EEPROMWriteAlarmEntry(aAlarmEntry->epoch, eepromAddr);
}

void processLightEntryMessage(time_t *anEpoch, unsigned int eepromAddr)
{
  // Alarm.free(aAlarmEntry->id);
  // unsigned int shour = tracePort -> parseInt();
  // unsigned int sminute = tracePort -> parseInt();
  // unsigned int ssecond = tracePort -> parseInt();
  time_t localAlarmTime = tracePort->parseInt();

  // *anEpoch = alarmTimeToUTC(AlarmHMS(shour, sminute, ssecond));
  *anEpoch = alarmTimeToUTC(localAlarmTime);

  // Serial << "alarm epoch " << aAlarmEntry->epoch << " alarm ID " <<
  // aAlarmEntry->id <<endl;
  EEPROMWriteAlarmEntry(*anEpoch, eepromAddr);
}

void processGCAlarmMessage()
{
  char c = tracePort->read();

  if (c == TIMER_ALARM_OFF)
  {
    processLightEntryMessage(&growingChamberLights.offEpoch,
                             EEPROM_GROWING_CHAMBER_OFF_TIME_ADDR);
    displayLightControlEntry("GC OFF", growingChamberLights.offEpoch);
  }
  else
  if (c == TIMER_ALARM_ON)
  {
    processLightEntryMessage(&growingChamberLights.onEpoch,
                             EEPROM_GROWING_CHAMBER_ON_TIME_ADDR);
    displayLightControlEntry("GC On", growingChamberLights.onEpoch);
  }
}

void processSAAlarmMessage()
{
  char c = tracePort->read();

  if (c == TIMER_ALARM_OFF)
  {
    processLightEntryMessage(&seedingAreaLights.offEpoch,
                             EEPROM_SEEDING_AREA_OFF_TIME_ADDR);
    displayLightControlEntry("SA OFF", seedingAreaLights.offEpoch);
  }
  else
  if (c == TIMER_ALARM_ON)
  {
    processLightEntryMessage(&seedingAreaLights.onEpoch,
                             EEPROM_SEEDING_AREA_ON_TIME_ADDR);
    displayLightControlEntry("SA ON", seedingAreaLights.onEpoch);
  }
}

void processCirculationPumpDurations()
{
  char c = tracePort->read();
  unsigned int duration;

  duration = (unsigned int)tracePort->parseInt();


  if (c == TIMER_ALARM_ON)
  {
    PY_001.setOnDuration(duration);
    EEPROMWriteDuration(duration, EEPROM_CIRCULATION_PUMP_ON_DURATION_ADDR);
  }
  else
  {
    PY_001.setOffDuration(duration);
    EEPROMWriteDuration(duration, EEPROM_CIRCULATION_PUMP_OFF_DURATION_ADDR);
  }
}

void processFanDurations()
{
  char c = tracePort->read();
  unsigned int duration;

  duration = (unsigned int)tracePort->parseInt();


  if (c == TIMER_ALARM_ON)
  {
    MY_101.setOnDuration(duration);
    EEPROMWriteDuration(duration, EEPROM_FAN_ON_DURATION_ADDR);
  }
  else
  {
    MY_101.setOffDuration(duration);
    EEPROMWriteDuration(duration, EEPROM_FAN_OFF_DURATION_ADDR);
  }
}

void showCommands()
{
  *tracePort <<
    "-------------------------------------------------------------------" << endl;
  *tracePort << F("Dt - Display Date/Time") << endl;
  *tracePort << F("Da - Display Alarms") << endl;
  *tracePort << F("T9999999999 - Set time using UNIX Epoch numner") << endl;
  *tracePort << F("G1HH:MM:SS - GC on") << endl;
  *tracePort << F("G0HH:MM:SS  - GC Off ") << endl;
  *tracePort << F("A1HH:MM:SS - SA on") << endl;
  *tracePort << F("A0HH:MM:SS  - SA Off ") << endl;
  *tracePort << F("Lc  - reset/clear settings to defaults ") << endl;
  *tracePort << F("It  - display 1 wire temps and humidity, sonar") << endl;
  *tracePort << F("Im  - display Soil Moisture") << endl;
  *tracePort << F("?? - Display commands") << endl;
  *tracePort << F("Sc - Serialize Circulation Pump") << endl;
  *tracePort << F("Sf - Serialize Fan") << endl;
  *tracePort << F("Sp - Serialize pH") << endl;
  *tracePort << F("Se - Serialize EC") << endl;
  *tracePort << F("Ep99.99 - Temperature Compensate pH") << endl;
  *tracePort << F("Zl  - Calibrate pH Low") << endl;
  *tracePort << F("Zm - Calibrate pH mid") << endl;
  *tracePort << F("Zh - Calibrate pH High") << endl;
  *tracePort << F("Zc - Clear pH Calibration") << endl;
  *tracePort << F(
    "Zq - Return calibration state (0=none, 1=1point, 2=2 point, 3=3point") <<
    endl;
  *tracePort << F("Ee99.99 - Temperature Compensate EC") << endl;
  *tracePort << F("Yl - Calibrate EC Low") << endl;
  *tracePort << F("Yh - Calibrate EC High") << endl;
  *tracePort << F("Yd - Calibrate EC Dry") << endl;
  *tracePort << F("Yc - Clear EC Calibration") << endl;
  *tracePort << F("Yq - Return calibration state (0=none, 1=1point, 2=2 point") <<
    endl;
  *tracePort << F("C19999999 - Circulation Pump On Duration in s") << endl;
  *tracePort << F("CO9999999 - Circulation Pump Off Duration in s") << endl;
  *tracePort << F("F19999999 - Fan On Duration in s") << endl;
  *tracePort << F("FO9999999 - Fan Off Duration in s") << endl;
  *tracePort <<
    "------------------------------------------------------------------" << endl;
}

void processTimeSetMessage()
{
  unsigned long pctime;

  pctime = tracePort->parseInt();

  // *tracePort << pctime << endl;
  if (pctime >= DEFAULT_TIME)

  // check the integer is a valid time (greater than Jan 1 2013)
  {
    RTC.set(pctime);     // set the RTC and the system time to the received
                         // value
    setTime(pctime);     // Sync Arduino clock to the time received on the
                         // Serial2 port
    char buffer[STRLEN("MM/DD/YYYY HH:MM")];
    TimeChangeRule *tcr; // pointer to the time change rule, use to get the TZ
                         // abbrev
    time_t atime = now();
    atime = usMT.toLocal(atime, &tcr);
    dateTimeToBuffer(atime, buffer);
    *tracePort << buffer << endl;
  }
}

void processSerializeMessage()
{
  char c = tracePort->read();

  switch (c)
  {
  case SERIALIZE_CIRCULATION_PUMP:
    PY_001.serialize(tracePort, true);
    break;

  case SERIALIZE_CIRCULATION_FAN:
    MY_101.serialize(tracePort, true);
    break;

  case SERIALIZE_PH:
    AT_001.serialize(tracePort, true);
    break;

  case SERIALIZE_EC:
    AT_002.serialize(tracePort, true);
    break;

  case LIGHTS_DUTY_CYCLE:

    // plantStrip.setDutyCycle( tracePort->parseInt());
    break;

  default:
    break;
  }
}

void processTemperatureCompensation()
{
  char  c = tracePort->read();
  float temperature;

  temperature = tracePort->parseFloat();

  if (c == SERIALIZE_PH)
  {
    // AT_002.pause();
    currentIOType = i2c_ph;
    AT_001.setCompensatedTemperature(temperature);
  }
  else
  {
    // AT_001.pause();
    currentIOType = i2c_ec;
    AT_002.setCompensatedTemperature(temperature);
  }
}

void processCalibrateMessage(IO_TYPE aIO_Type)
{
  char calCmd = tracePort->read();

  switch (calCmd)
  {
  case CALIBRATE_LOW:

    if (aIO_Type ==
        i2c_ec) *tracePort << "EC Cal Low:" << AT_002.calibrateLow() << endl;
    else *tracePort << "pH Cal Low:" << AT_001.calibrateLow() << endl;
    break;

  case CALIBRATE_MID:

    if (aIO_Type == i2c_ec) *tracePort << "EC Cal Mid does not exist:" << endl;
    else *tracePort << "pH Cal Mid:" << AT_001.calibrateMid() <<  endl;
    break;

  case CALIBRATE_HIGH:

    if (aIO_Type ==
        i2c_ec) *tracePort << "EC Cal High:" << AT_002.calibrateHigh() << endl;
    else *tracePort << "pH Cal High:" << AT_001.calibrateHigh() << endl;

    break;

  case CALIBRATE_DRY:

    if (aIO_Type ==
        i2c_ec) *tracePort << "EC Cal Dry:" << AT_002.calibrateDry() << endl;
    else *tracePort << "pH Cal Dry does not exist." <<  endl;
    break;

  case CALIBRATE_CLEAR:

    if (aIO_Type ==
        i2c_ec) *tracePort << "EC Cal Clear:" << AT_002.calibrateClear() << endl;
    else *tracePort << "pH Cal Clear:" << AT_001.calibrateClear() << endl;
    break;
    break;

  case CALIBRATE_QUERY:

    if (aIO_Type ==
        i2c_ec) *tracePort << "EC Points Calibrated:" <<
      AT_002.calibrateQuery() <<
        endl;
    else *tracePort << "pH Points Calibrated:" << AT_001.calibrateQuery() << endl;
    break;

  default:
    *tracePort << "Invalid command" << endl;
    break;
  }
}

void processTerminalCommands()
{
  if (tracePort->available() > 1)
  {
    // wait for at least two characters
    char c = tracePort->read();

    // *tracePort << c << endl;
    if (c == TIME_HEADER)
    {
      processTimeSetMessage();
    }
    else
    if (c == DISPLAY_HEADER)
    {
      processDisplayMessage();
    }
    else
    if (c == LIGHT_HEADER)
    {
      processLightsMessage();
    }
    else
    if (c == HELP_HEADER)
    {
      tracePort->read();
      showCommands();
    }
    else
    if (c == TIMER_GROWING_CHAMBER_HEADER)
    {
      processGCAlarmMessage();
    }
    else
    if (c == TIMER_SEEDING_AREA_HEADER)
    {
      processSAAlarmMessage();
    }
    else
    if (c == IO_HEADER)
    {
      processDisplayIOMessage();
    }
    else
    if (c == SERIALIZE_HEADER)
    {
      processSerializeMessage();
    }
    else
    if (c == CIRCULATION_PUMP_HEADER)
    {
      processCirculationPumpDurations();
    }
    else
    if (c == FAN_HEADER)
    {
      processFanDurations();
    }
    else
    if (c == CALIBRATE_EC_HEADER) processCalibrateMessage(i2c_ec);
    else
    if (c == CALIBRATE_PH_HEADER) processCalibrateMessage(i2c_ph);
    else
    if (c == TEMPERATURE_COMPENSATE_HEADER) processTemperatureCompensation();
  }
}

#endif // ifdef PROCESS_TERMINAL
