/**
*  @file    planttestNG.ino
*  @author  peter c
*  @date    4/14/2017
*  @version 0.1
*
*
*  @section DESCRIPTION
*  Control an ebb and flow hydroponic setup. Data published via as a modbus slave and setpoints,
*  alarm limits saved in EEPRPOM
*
*  2017Sep19 - Removed - Control LED strip lighting used for plants. LED strip is mostly red with some blue light. Ratio
*              set via dutycycle between 60 and 90%. Lights turn off and on at a predetermined interval.
*  2017Sep19 - Added On/Off control of full spectrum LED strip
*  2017Sep19 - Added ambient temperature reading via SPI
*  2017Sep19 - Added ambient humidity reading via SPI
*  2017Sep19 - Added solution temperature via SPI
*  2017Sep19 - Added Fan On/Off control
*  2017Sep19 - Added On/Off control for nutrient flow
*  // stino sublime text - libs to import go here C:\Users\home\Documents\Arduino\libraries
*  TODO- xbee integration next to feed into data collection system.
*  Hydroponic integration - temperature, humidity, pH control, water cycles
*  History - added temperature, humidity, and
*
*/
#include <Time.h>
#include <HardwareSerial.h>
#include <TimeLib.h>
#include <Timezone.h> //https://github.com/JChristensen/Timezone
#include <OneWire.h>
#include <DallasTemperature.h>
#include <Wire.h> // for RTC
#include <DS3232RTC.h> //http://github.com/JChristensen/DS3232RTC
#include <avr/eeprom.h>
#include <TimeAlarms.h>
#include <Streaming.h>
#include <NewPing.h>
#include <Adafruit_Sensor.h>
#include <DHT.h> // DHT-22 humidity sensor
#include <LiquidCrystal_I2C.h>
#include <flowmeter.h>
#include <DA_Analoginput.h>
#include <DA_Discreteinput.h>
#include <DA_DiscreteOutput.h>
#include <DA_DiscreteOutputTmr.h>

#include <DA_HOASwitch.h>
#include "PlantModbus.h"
#define DEFAULT_LIGHTS_ON_ALARM_TIME AlarmHMS (4, 0, 0)
#define DEFAULT_LIGHTS_OFF_ALARM_TIME AlarmHMS (23, 0, 0)
#define DEFAULT_RESET_TIME AlarmHMS(0, 0, 0) // midnight
// #define HOST_COMMAND_CHECK_INTERVAL  1000
#define LIGHT_REFRESH_INTERVAL 100
const unsigned long DEFAULT_TIME = 976492800;
#define EEPROM_CONFIGURED 2 // this value stored at address CONFIG_FLAG_ADDR
#define EEPROM_CONFIG_FLAG_ADDR 0 // is 0 if nothing was written
#define EEPROM_LED_LIGHTS_ON_TIME_ADDR EEPROM_CONFIG_FLAG_ADDR + sizeof (unsigned short)
#define EEPROM_LED_LIGHTS_OFF_TIME_ADDR EEPROM_LED_LIGHTS_ON_TIME_ADDR + sizeof (time_t)
#define EEPROM_LIGHTS_DUTY_CYCLE_PERIOD EEPROM_LED_LIGHTS_OFF_TIME_ADDR + sizeof (time_t)
#define VERSION F ("0.9")
// comment out to not include terminal processing
#define PROCESS_TERMINAL
// comment out to not implement modbus
//#define PROCESS_MODBUS
// refresh intervals
#define POLL_CYCLE_SECONDS 5 // sonar and 1-wire refresh rate
// flow meter
#define FT002_SENSOR_INTERUPT_PIN 2
#define FT003_SENSOR_INTERUPT_PIN 3
#define FLOW_CALC_PERIOD_SECONDS 1 // flow rate calc period
#define ENABLE_FT002_SENSOR_INTERRUPTS attachInterrupt(digitalPinToInterrupt(FT002_SENSOR_INTERUPT_PIN), onFT_002_PulseIn, RISING)
#define DISABLE_FT002_SENSOR_INTERRUPTS detachInterrupt(digitalPinToInterrupt(FT002_SENSOR_INTERUPT_PIN))
FlowMeter FT_002(FT002_SENSOR_INTERUPT_PIN, FLOW_CALC_PERIOD_SECONDS); // interrupt pin, calculation period in seconds
#define ENABLE_FT003_SENSOR_INTERRUPTS attachInterrupt(digitalPinToInterrupt(FT003_SENSOR_INTERUPT_PIN), onFT_003_PulseIn, RISING)
#define DISABLE_FT003_SENSOR_INTERRUPTS detachInterrupt(digitalPinToInterrupt(FT003_SENSOR_INTERUPT_PIN))
FlowMeter FT_003(FT003_SENSOR_INTERUPT_PIN, FLOW_CALC_PERIOD_SECONDS); // interrupt pin, calculation period in seconds



/*
Blue Serial IIC/I2C/TWI 2004 204 20X4 Character LCD Module Display For Arduino
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
//enum ScreenType  { HomeScreen=0, HOAStatus, MiscStatus  };
//ScreenType currentScreen = HomeScreen;

typedef struct LCDScreen LCDScreen;
 struct LCDScreen
{
  void (*displayFunc)( bool clearScreen ) ;
  struct LCDScreen *previousScreen ;
  struct LCDScreen *nextScreen ;

} ;



LCDScreen *currentScreen;
LCDScreen firstScreen ;
LCDScreen secondScreen;
LCDScreen thirdScreen;


// DHT-22 - one wire type humidity sensor (won't work with one wire lib)
#define DHT_BUS_PIN 5
DHT AT_101 = DHT(DHT_BUS_PIN, DHT22);
float AT_101T = NAN;
float AT_101H = NAN;
float AT_101HI = NAN;
// One Wire
// 
// 

DeviceAddress ambientTemperatureAddress =
{
  0x28, 0x6F, 0xE3, 0xA0, 0x04, 0x00, 0x00, 0x5A
};

DeviceAddress mixtureTemperatureAddress =
{
  0x28, 0xFF, 0xF4, 0xF6, 0x84, 0x16, 0x05, 0x0C
};

#define WIRE_BUS_PIN 4 // pin
#define ONE_TEMPERATURE_PRECISION 9
OneWire oneWire(WIRE_BUS_PIN);
DallasTemperature sensors(& oneWire);
// Analog Inputs
// 
// NewPing sonar(TRIGGER_PIN, ECHO_PIN, MAX_DISTANCE);

#define NUTRIENT_TANK_HEIGHT 45        // hight of nutrient tank in cm
#define NUTRIENT_TANK_AIR_GAP 16.5 // space between sensor and max water level in cm
#define NUTRIENT_TANK_MIXTURE_MAX NUTRIENT_TANK_HEIGHT - NUTRIENT_TANK_AIR_GAP
NewPing LT_002(15, 16, NUTRIENT_TANK_HEIGHT); // Water Level
float LT_002_PV = 0.0; // Water Level present value in cm from top,  0-> undefined
// Discrete Inputs
DA_DiscreteInput LSHH_002 = DA_DiscreteInput(51, DA_DiscreteInput::ToggleDetect, true); // nutrient mixture hi-hi level switch
DA_DiscreteInput HS_001 = DA_DiscreteInput(50, DA_DiscreteInput::ToggleDetect, true); // Drain Pump Hand status : Start/Stop


DA_DiscreteInput SSH_101 = DA_DiscreteInput(9); // Smoke Detector
DA_DiscreteInput HS_002 = DA_DiscreteInput(49, DA_DiscreteInput::ToggleDetect, true); // Inlet H20 Open/Close
DA_DiscreteInput HS_003A = DA_DiscreteInput(25, DA_DiscreteInput::RisingEdgeDetect, true); // LCD display next
DA_DiscreteInput HS_003B = DA_DiscreteInput(26, DA_DiscreteInput::RisingEdgeDetect, true); // LCD display previous
DA_DiscreteInput HS_003C = DA_DiscreteInput(27, DA_DiscreteInput::RisingEdgeDetect, true); // LCD display Enter
DA_HOASwitch  HS_001AB = DA_HOASwitch(7,0,8); // Circulation Pump Hand Status : HOA : Hand/Auto
//DA_HOASwitch HS_101AB = DA_HOASwitch(28, 0, 29); // Fan : HOA : Hand/Auto
DA_HOASwitch HS_102AB = DA_HOASwitch(52, 0, 53); // Non flowing LED : HOA : Hand/Auto
DA_HOASwitch HS_103AB = DA_HOASwitch(10, 0, 11); // flowing LED : HOA : Hand/Auto
// Discete Outputs
// DO 35,36 AC spares
// DO DC 39,41,42,43 spares
DA_DiscreteOutput DY_102 = DA_DiscreteOutput(31, LOW); // non-flowering LED 120 VAC
DA_DiscreteOutput DY_103 = DA_DiscreteOutput(32, LOW); // flowering LED 120 VAC
DA_DiscreteOutputTmr PY_001 = DA_DiscreteOutputTmr(33, LOW, 15*60, 45*60); // Circulation Pump 15 mins on/45 off 120VAC
DA_DiscreteOutputTmr MY_101 = DA_DiscreteOutputTmr(34, LOW, 60*60, 60*60); // Fan, 60 on/60 off 120VAC
DA_DiscreteOutput VY_001A = DA_DiscreteOutput(37, LOW); // inlet H20 valve, active low 12VDC
DA_DiscreteOutput PY_002 = DA_DiscreteOutput(38, LOW); // Drain Pump 12VDC
HardwareSerial *tracePort = & Serial;
TimeChangeRule usMDT =
{
  "MDT", Second, dowSunday, Mar, 2, -360
};

TimeChangeRule usMST =
{
  "MST", First, dowSunday, Nov, 2, -420
};

Timezone usMT(usMDT, usMST);
struct _AlarmEntry
{
  time_t epoch;
  AlarmId id = dtINVALID_ALARM_ID;
  bool firstTime = true;
};

typedef _AlarmEntry AlarmEntry;
AlarmEntry lightsOn; // = { AlarmHMS(4, 0, 0), dtINVALID_ALARM_ID };
AlarmEntry lightsOff; // = { AlarmHMS(11, 0, 0),  dtINVALID_ALARM_ID};
AlarmEntry onMidnight;
AlarmEntry onRefreshAnalogs; // sonar and 1-wire read refresh
AlarmEntry onFlowCalc; // flow calculations

void onFT_002_PulseIn()
{
  FT_002.handleFlowDetection();
}


void onFT_003_PulseIn()
{
  FT_003.handleFlowDetection();
}

/*
Only open inlet H20 Valve iff no Hi-Hi water level
note LSHH is high on high level (fail safe )
*/
void on_InletValve_Process(bool state)
{

#ifdef PROCESS_TERMINAL
  *tracePort << "on_InletValve_Process HS_002, LSHH_002" << endl;
  HS_002.serialize(tracePort, true);
  LSHH_002.serialize(tracePort, true);
#endif

  if (HS_002.getSample() == LOW && LSHH_002.getSample() == HIGH)
    VY_001A.activate();
  else
    VY_001A.reset();
}

void on_LCD_Next_Screen(bool state)
{


    currentScreen = currentScreen->nextScreen;
    currentScreen->displayFunc( true );

}

void on_LCD_Previous_Screen(bool state)
{



    currentScreen = currentScreen->previousScreen;
    currentScreen->displayFunc( true );


}

void on_LCD_Enter(bool state)
{

#ifdef PROCESS_TERMINAL
  *tracePort << "TO DO: LCD Enter HS_003C" << endl;
  HS_003C.serialize(tracePort, true);
#endif

}

void on_DrainPump_Process(bool state)
{

#ifdef PROCESS_TERMINAL
  *tracePort << "on_DrainPump_Process HS_001" << endl;
  HS_001.serialize(tracePort, true);
#endif

  if (HS_001.getSample() == LOW)
  {
    PY_002.activate();
  }
  else
    PY_002.reset();
}




void on_Circulation_Pump_Process(DA_HOASwitch::HOADetectType state)
{

#ifdef PROCESS_TERMINAL
  *tracePort << "on_Circulation_Pump_Process HS_001AB" << endl;
  HS_001AB.serialize(tracePort, true);
#endif

  switch (state)
  {
    case DA_HOASwitch::Hand:
      PY_001.disable();
      PY_001.forceActive(); // force the fan on
      break;
    case DA_HOASwitch::Off:
      PY_001.disable();
      break;
    case DA_HOASwitch::Auto:
      PY_001.enable();
      break;
    default:
      break;
  }
}

/*
void on_Fan_Process(DA_HOASwitch::HOADetectType state)
{

#ifdef PROCESS_TERMINAL
  *tracePort << "on_Fan_Process HS_101AB" << endl;
  HS_101AB.serialize(tracePort, true);
#endif

  switch (state)
  {
    case DA_HOASwitch::Hand:
      MY_101.disable();
      MY_101.forceActive(); // force the fan on
      break;
    case DA_HOASwitch::Off:
     // MY_101.disable();
      break;
    case DA_HOASwitch::Auto:
      MY_101.enable();
      break;
    default:
      break;
  }
}
*/
void on_FloweringLED_Process(DA_HOASwitch::HOADetectType state)
{

#ifdef PROCESS_TERMINAL
  *tracePort << "on_FlowingLED_Process HS_103AB" << endl;
  HS_103AB.serialize(tracePort, true);
#endif

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

void on_NonFloweringLED_Process(DA_HOASwitch::HOADetectType state)
{

#ifdef PROCESS_TERMINAL
  *tracePort << "on_NonFlowingLED_Process HS_102AB" << endl;
  HS_102AB.serialize(tracePort, true);
#endif

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
    *tracePort << F("Unable to sync with the RTC") << endl;
#endif

  }
  else
  {

#ifdef PROCESS_TERMINAL
    *tracePort << F("RTC has set the system time") << endl;
    *tracePort << F("Enter Command:") << endl;
#endif

  }
}

void printOneWireAddress(HardwareSerial *tracePort, DeviceAddress aDeviceAddress, bool aCR)
{
  for (uint8_t i = 0; i < 8; i++)
  {
    // zero pad the address if necessary
    if (aDeviceAddress[i] < 16)
      *tracePort << '0';
    tracePort->print(aDeviceAddress[i], HEX);
  }
  if (aCR)
    *tracePort << endl;
}

void initOneWireDevice(DeviceAddress aDevice, uint8_t aIndex)
{
  if (!sensors.getAddress(aDevice, aIndex))
  {
    // TODO Alarm

#ifdef PROCESS_TERMINAL
    *tracePort << "Unable to find address for Device at index " << aIndex << "address:";
    printOneWireAddress(tracePort, aDevice, true);
#endif

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
  lcd << VERSION;
  delay(2000);
  lcd.clear();
}

void printDigits(int digits)
{

  if (digits < 10)
  {

#ifdef PROCESS_TERMINAL
    *tracePort << '0';
    *tracePort << digits;
    *tracePort << ":";
#endif

    lcd << F("0");
  }

  lcd << digits;
}

void displayTime(time_t aTime, char * timeZone)
{

  printDigits(hour(aTime)) ;
  lcd << ":";
  printDigits(minute(aTime));
 // lcd << ":";
 // printDigits(second(aTime));


}


void displayDate(time_t aTime)
{


  //lcd << " " << timeZone << " ";
 // lcd << dayStr(weekday(aTime)) << " ";
 // lcd << monthShortStr(month(aTime)) << " ";
 printDigits(month(aTime));
 lcd << "/";
  printDigits(day(aTime));
 lcd << "/" <<  year(aTime) << " ";

#ifdef PROCESS_TERMINAL

  *tracePort << dayStr(weekday(aTime)) << " ";
  *tracePort << monthShortStr(month(aTime)) << " ";
  *tracePort << day(aTime) << " " << year(aTime) << endl;
#endif

}



void displayDateTime()
{
  if (timeStatus() != timeNotSet)
  {
    TimeChangeRule * tcr; // pointer to the time change rule, use to get the TZ abbrev
  time_t atime;
  atime = now();
  // printDateTime(atime, "UTC");
  atime = usMT.toLocal(atime, & tcr);
  displayDate(atime);
    //lcd.setCursor(0,1);
  displayTime(atime, tcr->abbrev);

  }
}


void displayHomeScreen( bool clearScreen )
{
  // RTC.set(1511196565); // set the RTC and the system time to the received value
  // setTime(1511196565);
  if( clearScreen )
    lcd.clear();

  lcd.home();
  displayDateTime();
  lcd.setCursor(0,1);
  lcd << F("Mixture Level:") << _FLOAT(LT_002_PV,1) << "%" ;
  lcd.setCursor(0,2);
  lcd << F("MT:") << _FLOAT(sensors.getTempC(mixtureTemperatureAddress),1) << "C " ;

  lcd << F("AT:") << _FLOAT(AT_101T,1) << "C" ; 
  lcd.setCursor(0,3);
  lcd << F("Rel Hum:") << _FLOAT(AT_101H,1) << "%";
/*
      *tracePort << "Sonar:cm:" << LT_002_PV << endl;
    *tracePort << "Ambient Temperature:" << sensors.getTempC(ambientTemperatureAddress) << "C" << endl;
    *tracePort << "Mixture Temperature:" << sensors.getTempC(mixtureTemperatureAddress) << "C" << endl;
    *tracePort << "HT-101: " << "Rel Humidity:" << AT_101H << " % Temperature:" << AT_101T;
    *tracePort << " C Heat Index " << AT_101HI << endl;
    * */

}

void displayMiscStatuses( bool clearScreen ) 
{
  if( clearScreen )
    lcd.clear();
  lcd.setCursor(0,0);
  lcd << F("FT-002:");
  lcd.setCursor(7,0);
  lcd << FT_002.getPreviousFlowRate();
  lcd.setCursor(12,0);
  lcd << F("L/min");

  lcd.setCursor(0,1);
  lcd << F("FT-003:");
  lcd.setCursor(7,1);
  lcd << FT_003 .getPreviousFlowRate();
  lcd.setCursor(12,1);
  lcd << F("L/min");
  
  lcd.setCursor(0,2);
  lcd << F("LSHH-002:");
  lcd.setCursor(9,2);
  lcd << LSHH_002.getSample();


}

void displayHOAStatuses( bool clearScreen ) 
{
  if( clearScreen )
    lcd.clear();

  //lcd.home();
  lcd.setCursor(0,0);
  lcd << F("   CPMP  FLED  NLED");

  lcd.setCursor(0,1);
  lcd << "H";
  lcd.setCursor(4,1);
  lcd << ((HS_001AB.getCurrentState()==DA_HOASwitch::Hand)?"X":" ");   
  lcd.setCursor(10,1);
  lcd << ((HS_103AB.getCurrentState()==DA_HOASwitch::Hand)?"X":" ");
  lcd.setCursor(16,1);
  lcd << ((HS_102AB.getCurrentState()==DA_HOASwitch::Hand)?"X":" ");      

   
 

  lcd.setCursor(0,2);  
  lcd << "O";  
  lcd.setCursor(4,2);
  lcd << ((HS_001AB.getCurrentState()==DA_HOASwitch::Off)?"X":" ");    
  lcd.setCursor(10,2);
  lcd << ((HS_103AB.getCurrentState()==DA_HOASwitch::Off)?"X":" ");
  lcd.setCursor(16,2);
  lcd << ((HS_102AB.getCurrentState()==DA_HOASwitch::Off)?"X":" ");      

   

  lcd.setCursor(0,3);
  lcd << "A";  
  lcd.setCursor(4,3);
  lcd << ((HS_001AB.getCurrentState()==DA_HOASwitch::Auto)?"X":" ");  
  lcd.setCursor(10,3);
  lcd << ((HS_103AB.getCurrentState()==DA_HOASwitch::Auto)?"X":" ");
  lcd.setCursor(16,3);
  lcd << ((HS_102AB.getCurrentState()==DA_HOASwitch::Auto)?"X":" ");      

       

}


void setupLCD()
{
  lcd.begin(20, 4);
  lcd.backlight();
  doLCDSplashScreen();
}

void setupLCDScreens()
{
firstScreen.displayFunc = displayHomeScreen;
firstScreen.previousScreen = &thirdScreen;
firstScreen.nextScreen = &secondScreen;

secondScreen.displayFunc = displayHOAStatuses;
secondScreen.previousScreen = &firstScreen;
secondScreen.nextScreen = &thirdScreen;

thirdScreen.displayFunc = displayMiscStatuses;
thirdScreen.previousScreen = &secondScreen;
thirdScreen.nextScreen = &firstScreen;
currentScreen = &firstScreen;

}
void setup()
{

#ifdef PROCESS_TERMINAL
  tracePort->begin(9600);
#endif

#ifdef PROCESS_MODBUS
  slave.begin(19200);
#endif

  randomSeed(analogRead(0));
  // LCD display
  setupLCD();
  setupLCDScreens();
  setupRTC();
  // InletValve
  // 
  // 
  onRefreshAnalogs.id = Alarm.timerRepeat(POLL_CYCLE_SECONDS, doOnPoll);
  onFlowCalc.id = Alarm.timerRepeat(FLOW_CALC_PERIOD_SECONDS, doOnCalcFlowRate);
  onMidnight.epoch = alarmTimeToUTC(DEFAULT_RESET_TIME);
  onMidnight.id = Alarm.alarmRepeat(onMidnight.epoch, doOnMidnight);
  if (isEEPROMConfigured() == EEPROM_CONFIGURED)
  {
    EEPROMLoadConfig();
  }
  else
  {
    EEPROMWriteDefaultConfig();
    EEPROMLoadConfig();
  }
  HS_002.setPollingInterval(500); // ms
  LSHH_002.setDebounceTime(1000); // float switch bouncing around
  LSHH_002.setPollingInterval(500); // ms
  HS_002.setOnEdgeEvent(& on_InletValve_Process);
  LSHH_002.setOnEdgeEvent(& on_InletValve_Process);
  HS_001.setPollingInterval(200); // ms
  // HS_001.setDebounceTime( 110);
  HS_001.setOnEdgeEvent(& on_DrainPump_Process);
  HS_003A.setPollingInterval(200); // ms
  HS_003B.setPollingInterval(200); // ms
  HS_003C.setPollingInterval(200); // ms  
  HS_003A.setOnEdgeEvent(& on_LCD_Next_Screen);
  HS_003B.setOnEdgeEvent(& on_LCD_Previous_Screen);
  HS_003C.setOnEdgeEvent(& on_LCD_Enter);;
  HS_001AB.setOnStateChangeDetect(& on_Circulation_Pump_Process);
  //HS_101AB.setOnStateChangeDetect(& on_Fan_Process);
  HS_102AB.setOnStateChangeDetect(& on_NonFloweringLED_Process);
  HS_103AB.setOnStateChangeDetect(& on_FloweringLED_Process);
  // 1-wire
  sensors.begin();
  initOneWire();
  // humidity sensor
  AT_101.begin();
  ENABLE_FT002_SENSOR_INTERRUPTS;
  lcd.clear();
}

void loop()
{

#ifdef PROCESS_MODBUS
  refreshModbusRegisters();
  slave.poll(modbusRegisters, MODBUS_REG_COUNT);
  processModbusCommands();
#endif

#ifdef PROCESS_TERMINAL
  processTerminalCommands();
#endif

  refreshDiscreteInputs();
  refreshDiscreteOutputs();
  Alarm.delay(LIGHT_REFRESH_INTERVAL);
  // doReadInputs();
  // doUpdateOutputs();
  // LSL_100.refresh();
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
  //HS_101AB.refresh();
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
  FT_002.dayRollOver();
  *tracePort << "day Rollover:";
  FT_002.serialize(tracePort, true);
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

  ENABLE_FT002_SENSOR_INTERRUPTS;
  ENABLE_FT003_SENSOR_INTERRUPTS;
}

// update sonar and 1-wire DHT-22 readings
void doOnPoll()
{
  float tLevel = 0.0;

   unsigned int imperial = LT_002.ping();
   // compute distanace from high level mark
   tLevel = NUTRIENT_TANK_MIXTURE_MAX - imperial / US_ROUNDTRIP_CM;

  // convert to % full
   LT_002_PV = (1 - tLevel / NUTRIENT_TANK_MIXTURE_MAX) * 100 ;
*tracePort << "imperial=" << imperial << " tLevel = " << tLevel << " LT_002_PV= " << LT_002_PV << endl;

  sensors.requestTemperatures();
   AT_101H = AT_101.readHumidity(); // allow 1/4 sec to read
   AT_101T = AT_101.readTemperature();
  if (isnan(AT_101H) || isnan(AT_101T))
  {

#ifdef PROCESS_TERMINAL
    // *tracePort << "Error reading from DHT-22 sensor." << endl;
#endif

  }
  else
  {
    AT_101HI = AT_101.computeHeatIndex(AT_101T, AT_101H, false);
  }
}

void doReadInputs()
{
  // TE_001.refresh();
  // QE_001.refresh();
}

void doUpdateOutputs()
{
}

void doLightsOn()
{
  DY_102.activate(); // if disabled, it won't activate
  DY_103.activate(); // if disabled, it won't activate

#ifdef PROCESS_TERMINAL
  *tracePort << "...Lights on" << endl;
#endif

}

void doLightsOff()
{
  DY_102.reset();
  DY_103.reset();

#ifdef PROCESS_TERMINAL
  *tracePort << "...Lights off" << endl;
#endif

}

// timezone lib does not handle short 24 hr duration epochs
time_t alarmTimeToUTC(time_t localAlarmTime)
{
  time_t utcAlarmTime = usMT.toUTC(localAlarmTime);
  if (usMT.utcIsDST(now()))
  {
    utcAlarmTime -= 60 * 60;
  }
  if (utcAlarmTime > SECS_PER_DAY)
    utcAlarmTime -= SECS_PER_DAY;
  return(utcAlarmTime);
}

time_t alarmTimeToLocal(time_t utcAlarmTime)
{
  TimeChangeRule * tcr; // pointer to the time change rule, use to get the TZ abbrev
  time_t localAlarmTime = usMT.toLocal(utcAlarmTime, & tcr);
  int offset = tcr->offset; // in minutes
  if (usMT.utcIsDST(now()))
  {
    offset += 60;
    localAlarmTime += 60 * 60;
  }
  if ((hour(utcAlarmTime) + (int) offset / 60) < 0)
  {
    int hr = 12 + (hour(utcAlarmTime) - (int) offset / 60);
    localAlarmTime = AlarmHMS(hr, minute(utcAlarmTime), second(utcAlarmTime));
  }
  return(localAlarmTime);
}

/*
** Modbus related functions
*/
void refreshModbusRegisters()
{
  // modbusRegisters[HR_LED_DUTY_CYCLE] = plantStrip.getDutyCycle();
  // modbusRegisters[HR_LED_DUTY_CYCLE_PERIOD] = dutyCycleChangeAlarm.epoch;
  blconvert.val = lightsOn.epoch;
  modbusRegisters[HR_LED_ON_TIME] = blconvert.regsl[0];
  modbusRegisters[HR_LED_ON_TIME + 1] = blconvert.regsl[1];
  blconvert.val = lightsOff.epoch;
  modbusRegisters[HR_LED_OFF_TIME] = blconvert.regsl[0];
  modbusRegisters[HR_LED_OFF_TIME + 1] = blconvert.regsl[1];
  blconvert.val = now();
  modbusRegisters[HR_CURRENT_TIME] = blconvert.regsl[0];
  modbusRegisters[HR_CURRENT_TIME + 1] = blconvert.regsl[1];
  // modbusRegisters[ CS_LED_STATUS ] = plantStrip.isLightsOn();
  // modbusRegisters[ CS_LED_STATUS ] = plantStrip.isLightsOn();
  // writeModbusCoil( COIL_STATUS_READ_WRITE_OFFSET, CS_LED_STATUS, plantStrip.isLightsOn());
  // modbusRegisters[HR_AMBIENT_TEMPERATURE] = (int) TE_001.getScaledSample() * 10;
  // modbusRegisters[HR_SOIL_MOISTURE] = (int) QE_001.getScaledSample() * 10;
  // bitWrite(modbusRegisters[ COIL_STATUS_READ_OFFSET], CS_SET_LED_ON, plantStrip.isLightsOn());
}

void setModbusTime()
{
  if (modbusRegisters[HR_SET_TIME] != 0)
  {
    unsigned long pctime;
    blconvert.regsl[0] = modbusRegisters[HR_SET_TIME];
    blconvert.regsl[1] = modbusRegisters[HR_SET_TIME + 1];
    pctime = blconvert.val;
    RTC.set(pctime); // set the RTC and the system time to the received value
    setTime(pctime); // Sync Arduino clock to the time received on the Serial2 port
    modbusRegisters[HR_SET_TIME] = 0;
    modbusRegisters[HR_SET_TIME + 1] = 0;
  }
}

void setModbusLightsOnTime()
{
  if (modbusRegisters[HR_SET_LED_ON_TIME] != 0)
  {
    blconvert.regsl[0] = modbusRegisters[HR_SET_LED_ON_TIME];
    blconvert.regsl[1] = modbusRegisters[HR_SET_LED_ON_TIME + 1];
    lightsOn.epoch = alarmTimeToUTC(blconvert.val);
    Alarm.free(lightsOn.id);
    lightsOn.id = Alarm.alarmRepeat(lightsOn.epoch, doLightsOn);
    EEPROMWriteAlarmEntry(lightsOn.epoch, EEPROM_LED_LIGHTS_ON_TIME_ADDR);
    modbusRegisters[HR_SET_LED_ON_TIME] = 0;
    modbusRegisters[HR_SET_LED_ON_TIME + 1] = 0;
  }
}

void setModbusLightsOffTime()
{
  if (modbusRegisters[HR_SET_LED_OFF_TIME] != 0)
  {
    blconvert.regsl[0] = modbusRegisters[HR_SET_LED_OFF_TIME];
    blconvert.regsl[1] = modbusRegisters[HR_SET_LED_OFF_TIME + 1];
    lightsOff.epoch = alarmTimeToUTC(blconvert.val);
    Alarm.free(lightsOff.id);
    lightsOff.id = Alarm.alarmRepeat(lightsOff.epoch, doLightsOff);
    EEPROMWriteAlarmEntry(lightsOff.epoch, EEPROM_LED_LIGHTS_OFF_TIME_ADDR);
    modbusRegisters[HR_SET_LED_OFF_TIME] = 0;
    modbusRegisters[HR_SET_LED_OFF_TIME + 1] = 0;
  }
}

void setModbusDutyCycle()
{
  if (modbusRegisters[HR_SET_DUTY_CYCLE] != 0)
  {
    // plantStrip.setDutyCycle( modbusRegisters[ HR_SET_DUTY_CYCLE ] );
    modbusRegisters[HR_SET_DUTY_CYCLE] = 0;
  }
}

void setModbusDutyCyclePeriod()
{
  if (modbusRegisters[HR_SET_DUTY_CYCLE_PERIOD] != 0)
  {
    // Alarm.free( dutyCycleChangeAlarm.id );
    // dutyCycleChangeAlarm.epoch = modbusRegisters[ HR_SET_DUTY_CYCLE_PERIOD ] ;
    // dutyCycleChangeAlarm.id = Alarm.timerRepeat(dutyCycleChangeAlarm.epoch, alterLEDPattern);
    // EEPROMWriteAlarmEntry( dutyCycleChangeAlarm.epoch, EEPROM_LIGHTS_DUTY_CYCLE_PERIOD );
    modbusRegisters[HR_SET_DUTY_CYCLE_PERIOD] = 0;
  }
}

void setModbusLightsOn()
{
  // *tracePort << "...modbus pre Lights on" <<  modbusRegisters[COIL_STATUS_WRITE_OFFSET+1]  << endl;
  if (getModbusCoilValue(COIL_STATUS_READ_WRITE_OFFSET, CS_SET_LED_ON))
  {
    // *tracePort << "...got a bit" << endl;
    doLightsOn();
    writeModbusCoil(COIL_STATUS_READ_WRITE_OFFSET, CS_SET_LED_ON, false);
  }
}

void setModbusLightsOff()
{
  // *tracePort << "...modbus pre Lights on" <<  modbusRegisters[COIL_STATUS_WRITE_OFFSET+1]  << endl;
  if (getModbusCoilValue(COIL_STATUS_READ_WRITE_OFFSET, CS_SET_LED_OFF))
  {
    doLightsOff();
    writeModbusCoil(COIL_STATUS_READ_WRITE_OFFSET, CS_SET_LED_OFF, false);
  }
}

void setConfigToDefaults()
{
  if (getModbusCoilValue(COIL_STATUS_READ_WRITE_OFFSET, CS_RESET_TO_DEFAULTS))
  {
    EEPROMWriteDefaultConfig();
    EEPROMLoadConfig();
    writeModbusCoil(COIL_STATUS_READ_WRITE_OFFSET, CS_RESET_TO_DEFAULTS, false);
  }
}

bool getModbusCoilValue(unsigned short startAddress, unsigned short bitPos)
{
  return(bitRead(modbusRegisters[startAddress + (int) (bitPos / 16)], bitPos % 16));
}

void writeModbusCoil(unsigned short startAddress, unsigned short bitPos, bool value)
{
  bitWrite(modbusRegisters[startAddress + (int) (bitPos / 16)], bitPos % 16, value);
}

void processModbusCommands()
{
  setModbusTime();
  setModbusDutyCycle();
  setModbusDutyCyclePeriod();
  setModbusLightsOnTime();
  setModbusLightsOffTime();
  setModbusLightsOn();
  setModbusLightsOff();
  setConfigToDefaults();
}

/*
** EEPROM functions
*/
void EEPROMWriteDefaultConfig()
{
  unsigned short configFlag = EEPROM_CONFIGURED;
  eeprom_write_block((const void *) & configFlag,(void *) EEPROM_CONFIG_FLAG_ADDR, sizeof(configFlag));
  time_t epoch = alarmTimeToUTC(DEFAULT_LIGHTS_ON_ALARM_TIME);
  EEPROMWriteAlarmEntry(epoch, EEPROM_LED_LIGHTS_ON_TIME_ADDR);
  epoch = alarmTimeToUTC(DEFAULT_LIGHTS_OFF_ALARM_TIME);
  EEPROMWriteAlarmEntry(epoch, EEPROM_LED_LIGHTS_OFF_TIME_ADDR);
  // epoch = DEFAULT_LIGHTS_DUTY_CYCLE_PERIOD;
  EEPROMWriteAlarmEntry(epoch, EEPROM_LIGHTS_DUTY_CYCLE_PERIOD);
}

void EEPROMWriteAlarmEntry(time_t epoch, unsigned int atAddress)
{
  eeprom_write_block((const void *) & epoch,(void *) atAddress, sizeof(epoch));
}

time_t EEPROMReadAlarmEntry(unsigned int atAddress)
{
  time_t epoch = 0;
  eeprom_read_block((void *) & epoch,(void *) atAddress, sizeof(epoch));
  return(epoch);
}

unsigned int isEEPROMConfigured()
{
  unsigned short configFlag;
  eeprom_read_block((void *) & configFlag,(void *) EEPROM_CONFIG_FLAG_ADDR, sizeof(configFlag));
  return(configFlag);
}

void EEPROMLoadConfig()
{
  lightsOn.epoch = EEPROMReadAlarmEntry(EEPROM_LED_LIGHTS_ON_TIME_ADDR);
  Alarm.free(lightsOn.id);
  lightsOn.id = Alarm.alarmRepeat(lightsOn.epoch, doLightsOn);
  lightsOff.epoch = EEPROMReadAlarmEntry(EEPROM_LED_LIGHTS_OFF_TIME_ADDR);
  Alarm.free(lightsOff.id);
  lightsOff.id = Alarm.alarmRepeat(lightsOff.epoch, doLightsOff);
  // Alarm.free( dutyCycleChangeAlarm.id );
  // dutyCycleChangeAlarm.id = Alarm.alarmRepeat(dutyCycleChangeAlarm.epoch, alterLEDPattern);
}

#ifdef PROCESS_TERMINAL
// single character message tags from terminal host
#define TIME_HEADER 'T' // Header tag for Serial2 time sync message
#define DISPLAY_HEADER 'D' // Display header tag
#define DISPLAY_TIME 't'
#define DISPLAY_ALARMS 'a'
#define DISPLAY_DUTY_CYCLE 'd'
#define TIMER_ALARM_HEADER 'A' // Timer Alarm Header tag
#define TIMER_ALARM_ON '1' // Timer Alarm On A14:00:00->turn on lights at 4AM
#define TIMER_ALARM_OFF '0' // Timer Alarm On A023:00:00->turn off lighst at 11 PM
#define HELP_HEADER '?'
#define LIGHT_HEADER 'L' // Light Header tag
#define LIGHTS_ON '1' // Turn lights on L1
#define LIGHTS_OFF '0' // Turn lights off L0
// efine LIGHTS_TOOGLE 't'         // toggle
#define LIGHTS_DUTY_CYCLE 'd' // Ld60->60% dominant color default mostly red 60-90 percent allowed
#define LIGHTS_DUTY_CYCLE_PERIOD 'r' // Lr3600->change the duty cycle between 60-90 % every hour
#define LIGHTS_RESET_TO_DEFAULTS 'c'
#define IO_HEADER 'I' // read IO points
#define IO_AMBIENT_TEMP 't'
#define IO_SOIL_MOISTURE 'm'
#define SERIALIZE_HEADER 'S'
#define SERIALIZE_CIRCULATION_PUMP 'c'
#define SERIALIZE_CIRCULATION_FAN 'f'
void displayAlarm(char * who, struct _AlarmEntry aAlarmEntry)
{
  *tracePort << who << "id = " << aAlarmEntry.id << " UTC set to " << hour(aAlarmEntry.epoch);
  printDigits(minute(aAlarmEntry.epoch));
  printDigits(second(aAlarmEntry.epoch));
  time_t localAlarmTime = alarmTimeToLocal(aAlarmEntry.epoch);
  *tracePort << " UTC epoch is " << aAlarmEntry.epoch;
  *tracePort << " Local is " << hour(localAlarmTime);
  printDigits(minute(localAlarmTime));
  printDigits(second(localAlarmTime));
  *tracePort << " epoch is " << localAlarmTime << endl;
}

void processDisplayIOMessage()
{
  char c = tracePort->read();
  if (c == IO_AMBIENT_TEMP)
  {
    // float h = AT_101.readHumidity(); // allow 1/4 sec to read
    // float t = AT_101.readTemperature();
    *tracePort << "Sonar:cm:" << LT_002_PV << endl;
    *tracePort << "Ambient Temperature:" << sensors.getTempC(ambientTemperatureAddress) << "C" << endl;
    *tracePort << "Mixture Temperature:" << sensors.getTempC(mixtureTemperatureAddress) << "C" << endl;
    *tracePort << "HT-101: " << "Rel Humidity:" << AT_101H << " % Temperature:" << AT_101T;
    *tracePort << " C Heat Index " << AT_101HI << endl;
    // AT_101.serialize( tracePort, true);
    // *tracePort << "Abient Temp = " << TE_001.getScaledSample() << " C" << endl;
    // float hic = AT_101.computeHeatIndex(t, h, false);
    // *tracePort << "DHT-22 Relative Humidity:" << h << " Temperature:" << t << " C Humidex:" << hic << endl;
  }
  else
    if (c == IO_SOIL_MOISTURE)
    {
      // *tracePort << "Soil Moisture = " << QE_001.getScaledSample() << "%" << endl;
    }
}

void processDisplayMessage()
{
  char c = tracePort->read();
  if (c == DISPLAY_TIME)
  {
    displayDateTime();
  }
  else
    if (c == DISPLAY_ALARMS)
    {
      displayAlarm("...Lights On Alarm", lightsOn);
      displayAlarm("...Lights Off Alarm", lightsOff);
      displayAlarm("...Reset Midnight", onMidnight);
      // *tracePort << "Duty Cycle Period = " << dutyCycleChangeAlarm.epoch << " s id=" << dutyCycleChangeAlarm.id << endl;
    }
  else
    if (c == DISPLAY_DUTY_CYCLE)
    {
      // *tracePort << "Duty Cycle = " << plantStrip.getDutyCycle() << endl;
    }
}

void processShowTime()
{
  char c = tracePort->read();
  displayDateTime();
}

void processLightsMessage()
{
  char c = tracePort->read();
  switch (c)
  {
    case LIGHTS_ON:
      doLightsOn();
      break;
    case LIGHTS_OFF:
      doLightsOff();
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

void processAlarmMessage()
{
  char c = tracePort->read();
  if (c == TIMER_ALARM_ON)
  {
    Alarm.free(lightsOn.id);
    unsigned int shour = tracePort->parseInt(); // constrain(tracePort->parseInt(), 0, 23);
    unsigned int sminute = tracePort->parseInt(); // constrain(tracePort->parseInt(), 0, 59);
    unsigned int ssecond = tracePort->parseInt(); // constrain(tracePort->parseInt(), 0, 59);
    lightsOn.epoch = alarmTimeToUTC(AlarmHMS(shour, sminute, ssecond));
    lightsOn.id = Alarm.alarmRepeat(lightsOn.epoch, doLightsOn);
    EEPROMWriteAlarmEntry(lightsOn.epoch, EEPROM_LED_LIGHTS_ON_TIME_ADDR);
    displayAlarm("Lights On Alarm", lightsOn);
    // *tracePort << tracePort->parseInt() << "-" << tracePort->parseInt() << "-" << tracePort->parseInt() << endl;
  }
  else
    if (c == TIMER_ALARM_OFF)
    {
      Alarm.free(lightsOff.id);
      unsigned int shour = tracePort->parseInt(); // constrain(tracePort->parseInt(), 0, 23);
      unsigned int sminute = tracePort->parseInt(); // constrain(tracePort->parseInt(), 0, 59);
      unsigned int ssecond = tracePort->parseInt(); // constrain(tracePort->parseInt(), 0, 59);
      lightsOff.epoch = alarmTimeToUTC(AlarmHMS(shour, sminute, ssecond));
      lightsOff.id = Alarm.alarmRepeat(lightsOff.epoch, doLightsOff);
      EEPROMWriteAlarmEntry(lightsOff.epoch, EEPROM_LED_LIGHTS_OFF_TIME_ADDR);
      displayAlarm("Lights Off Alarm", lightsOff);
    }
}

void showCommands()
{
  *tracePort << "-------------------------------------------------------------------" << endl;
  *tracePort << F("Dt - Display Date/Time") << endl;
  *tracePort << F("Da - Display Alarms") << endl;
  *tracePort << F("T9999999999 - Set time using UNIX Epoch numner") << endl;
  *tracePort << F("A1HH:MM:SS - Set lights on time ") << endl;
  *tracePort << F("A0HH:MM:SS  - Set lights off time ") << endl;
  *tracePort << F("L1  - Lights On ") << endl;
  *tracePort << F("L0  - Lights Off ") << endl;
  *tracePort << F("Ld99 - Lighting duty cycle 99 From 60 to 90") << endl;
  *tracePort << F("Lr99999 - Lighting time to randomly change duty cycle in seconds") << endl;
  *tracePort << F("Lc  - reset/clear settings to defaults ") << endl;
  *tracePort << F("It  - display 1 wire temps and humidity, sonar") << endl;
  *tracePort << F("Im  - display Soil Moisture") << endl;
  *tracePort << F("?? - Display commands") << endl;
  *tracePort << F("Sc - Serialize Circulation Pump") << endl;
  *tracePort << "------------------------------------------------------------------" << endl;
}

void processTimeSetMessage()
{
  unsigned long pctime;
  pctime = tracePort->parseInt();
  // *tracePort << pctime << endl;
  if (pctime >= DEFAULT_TIME)
  // check the integer is a valid time (greater than Jan 1 2013)
  {
    RTC.set(pctime); // set the RTC and the system time to the received value
    setTime(pctime); // Sync Arduino clock to the time received on the Serial2 port
    displayDateTime();
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
    case LIGHTS_DUTY_CYCLE:
    // plantStrip.setDutyCycle( tracePort->parseInt());
    break;
    default:
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
      if (c == TIMER_ALARM_HEADER)
      {
        processAlarmMessage();
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
  }
}

#endif
