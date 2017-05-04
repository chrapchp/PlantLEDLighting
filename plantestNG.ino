#include <SerialModbusSlave.h>

/**
 *  @file    planttestNG.ino
 *  @author  peter c
 *  @date    4/14/2017
 *  @version 0.1
 *
 *
 *  @section DESCRIPTION
 *  Control LED strip lighting used for plants. LED strip is mostly red with some blue light. Ratio
 *  set via dutycycle between 60 and 90%. Lights turn off and on at a predetermined interval.
 *
 *  TODO- xbee integration next to feed into data collection system.
 *  Hydroponic integration - temperature, humidity, pH control, water cycles
 *
 *
 */


#include <HardwareSerial.h>
#include <TimeLib.h>
#include <Timezone.h>    //https://github.com/JChristensen/Timezone
#include <Wire.h>
#include <DS3232RTC.h>    //http://github.com/JChristensen/DS3232RTC



#include <TimeAlarms.h>
#include <Streaming.h>

#include "PlantModbus.h"
#include "PlantLED.h"
#include "DA_Analoginput.h"
#include "DA_Discreteinput.h"

#define STRIP_1_PIN 6

#define MIN_DUTY_CYCLE 20
#define MAX_DUTY_CYCLE 90

// single character message tags
#define TIME_HEADER   'T'   // Header tag for Serial2 time sync message

#define DISPLAY_HEADER   'D'   // Display header tag 
#define DISPLAY_TIME   't'
#define DISPLAY_ALARMS   'a'
#define DISPLAY_DUTY_CYCLE 'd'


#define TIMER_ALARM_HEADER  'A'    // Timer Alarm Header tag
#define TIMER_ALARM_ON  '1'    // Timer Alarm On A14:00:00 -> turn on lights at 4AM
#define TIMER_ALARM_OFF  '0'    // Timer Alarm On A023:00:00 -> turn off lighst at 11 PM

#define HELP_HEADER '?'


#define LIGHT_HEADER 'L'          // Light Header tag
#define LIGHTS_ON    '1'          // Turn lights on L1
#define LIGHTS_OFF   '0'          // Turn lights off L0
//efine LIGHTS_TOOGLE 't'         // toggle
#define LIGHTS_DUTY_CYCLE 'd'     // Ld60 -> 60% dominant color default mostly red 60-90 percent allowed
#define LIGHTS_RANDOM_DUTY_CYCLE_TIME 'r' // Lr3600 -> change the duty cycle between 60-90 % every hour


//#define HOST_COMMAND_CHECK_INTERVAL  1000
#define LIGHT_REFRESH_INTERVAL  100

const unsigned long DEFAULT_TIME = 976492800;


//DA_AnalogInput TT_100 = DA_AnalogInput(  A4, -40.0, 40.0 );
//DA_DiscreteInput LSL_100 = DA_DiscreteInput(  12 );



PlantLEDStrip strip1 = PlantLEDStrip(3 * 60 + 2 * 144  , STRIP_1_PIN, NEO_GRB + NEO_KHZ800);
//PlantLEDStrip strip2 = PlantLEDStrip(60  , STRIP_2_PIN, NEO_GRB + NEO_KHZ800);

HardwareSerial *tracePort = &Serial2;

TimeChangeRule usMDT = {"MDT", Second, dowSunday, Mar, 2, -360};
TimeChangeRule usMST = {"MST", First, dowSunday, Nov, 2, -420};
Timezone usMT(usMDT, usMST);

struct _AlarmEntry
{
  short hours;
  short minutes;
  short seconds;
  AlarmId id;
} ;

struct _AlarmIntervalEntry
{
  unsigned long interval;
  AlarmId id;
} ;

typedef _AlarmEntry AlarmEntry;
typedef _AlarmIntervalEntry AlarmIntervalEntry;

AlarmEntry lightsOnAlarm = { 11, 0, 0, dtINVALID_ALARM_ID };
AlarmEntry lightsOffAlarm = { 4, 0, 0,  dtINVALID_ALARM_ID};
AlarmIntervalEntry dutyCycleChangeAlarm = { 60 * 60, dtINVALID_ALARM_ID};


void displayAlarm( char *who, struct _AlarmEntry aAlarmEntry)
{
  *tracePort << who << "id = " << aAlarmEntry.id << " set to "  << aAlarmEntry.hours ;
  printDigits(aAlarmEntry.minutes );
  printDigits(aAlarmEntry.seconds );
  *tracePort << endl;
}

void onTT_100Sample( float aValue )
{
  *tracePort << "TT-100 = " << aValue << endl;
}

void onTT_100SampleDeadband( float aValue )
{
  *tracePort << "Deadband TT-100 = " << aValue << endl;
}

void onLSL_100Sample( bool aValue )
{
  *tracePort << "LSL_100 = " << aValue << endl;
}

void onLSL_100EdgeDetect( bool state )
{
  *tracePort << "ToggleDetected=" << state << endl;
  alterLEDPattern();
}

void alterLEDPattern()
{
  strip1.setDutyCycle( random(60, 90 ));
  //strip2.setDutyCycle( random(80, 90) );
// strip1.flipColors();
  //strip2.flipColors();
}

void setup()
{
  tracePort->begin(9600);
  slave.begin( 19200 );
  randomSeed(analogRead(0));
  //setSyncProvider( requestSync);  //set function to call when sync required
  setSyncProvider(RTC.get);
  //setSyncInterval(30);
  if (timeStatus() != timeSet)
  {
    *tracePort << F("Unable to sync with the RTC") << endl;
    // setTime(DEFAULT_TIME);
  }
  else
    *tracePort << F("RTC has set the system time") << endl;
  showCommands();
  *tracePort << F("Enter Command:") << endl;
  lightsOnAlarm.id = Alarm.alarmRepeat(lightsOnAlarm.hours, lightsOnAlarm.minutes, lightsOnAlarm.seconds, doLightsOn);
  displayAlarm("...Lights On Alarm", lightsOnAlarm );
  lightsOffAlarm.id = Alarm.alarmRepeat(lightsOffAlarm.hours, lightsOffAlarm.minutes, lightsOffAlarm.seconds, doLightsOff);
  displayAlarm("...Lights Off Alarm", lightsOffAlarm );
  dutyCycleChangeAlarm.id = Alarm.timerRepeat(dutyCycleChangeAlarm.interval, alterLEDPattern);
  strip1.initialize();
  //strip2.initialize();
  /*
  TT_100.setPollingInterval( 1000 );
  TT_100.setOnPollCallBack(&onTT_100Sample);
  TT_100.setOutsideDeadbandDetectedEvent(&onTT_100SampleDeadband);
  TT_100.setDeadband(.01);

  LSL_100.setPollingInterval( 15 );
  // LSL_100.setOnPollCallBack(&onLSL_100Sample);
  LSL_100.setOnEdgeEvent(&onLSL_100EdgeDetect);
  LSL_100.enableInternalPullup();
  LSL_100.setEdgeDetectType( DA_DiscreteInput::FallingEdgeDetect );
  LSL_100.setDebouceTime(10);
  */
}

void loop()
{
  //*tracePort << analogRead(POT_PIN);
  // inputDutyCycle = map(analogRead(POT_PIN), 0, 1023, MIN_DUTY_CYCLE, MAX_DUTY_CYCLE);
  //*tracePort << inputDutyCycle;
  strip1.refresh();
  refreshModbusRegisters();
  slave.poll( modbusRegisters, MODBUS_REG_COUNT );
  processModbusCommands();
  //strip2.refresh();
  doCommandFromHostCheck();
  Alarm.delay(LIGHT_REFRESH_INTERVAL);
  // TT_100.refresh();
  //LSL_100.refresh();
}



void doCommandFromHostCheck()
{
  if (tracePort->available() > 1)
  {
    // wait for at least two characters
    char c = tracePort->read();
    // *tracePort << c << endl;
    if ( c == TIME_HEADER)
    {
      processSyncMessage();
    }
    else if ( c == DISPLAY_HEADER)
    {
      processDisplayMessage();
    }
    else if ( c == LIGHT_HEADER)
    {
      processLightsMessage();
    }
    else if ( c == HELP_HEADER)
    {
      tracePort->read();
      showCommands();
    }
    else if ( c == TIMER_ALARM_HEADER)
    {
      processAlarmMessage();
    }
  }
}


void printDateTime( time_t aTime , char *timeZone)
{
  *tracePort << hour(aTime) ;
  printDigits(minute(aTime));
  printDigits(second(aTime)) ;
  *tracePort << " " << timeZone << " ";
  *tracePort << dayStr(weekday(aTime)) << " " ;
  *tracePort << monthShortStr(month(aTime)) << " ";
  *tracePort << day(aTime) << " "  << year(aTime) << endl;
}

void digitalClockDisplay()
{
  *tracePort << hour() ;
  printDigits(minute());
  printDigits(second()) ;
  *tracePort << " ";
  *tracePort << dayStr(weekday()) << " " ;
  *tracePort << monthShortStr(month()) << " ";
  *tracePort << day() << " "  << year() << endl;
  TimeChangeRule *tcr;        //pointer to the time change rule, use to get the TZ abbrev
  time_t atime;
  atime = now();
  printDateTime(atime, "UTC");
  atime = usMT.toLocal(atime, &tcr);
  printDateTime(atime, tcr -> abbrev);
}

void printDigits(int digits)
{
  *tracePort << ":";
  if (digits < 10)
    *tracePort << '0';
  *tracePort << digits;
}

void displayTime()
{
  if (timeStatus() != timeNotSet)
  {
    digitalClockDisplay();
  }
}

void processDisplayMessage()
{
  char c = tracePort->read();
  if ( c == DISPLAY_TIME)
  {
    displayTime();
  }
  else if ( c == DISPLAY_ALARMS)
  {
    displayAlarm("...Lights On Alarm", lightsOnAlarm );
    displayAlarm("...Lights Off Alarm", lightsOffAlarm );
    *tracePort << "Duty Cycle Period = " << dutyCycleChangeAlarm.interval << " s" << endl;
  }
  else if ( c == DISPLAY_DUTY_CYCLE)
  {
    *tracePort << "Duty Cycle = " << strip1.getDutyCycle() << endl;
  }
}

void processShowTime()
{
  char c = tracePort->read();
  displayTime();
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
    strip1.setDutyCycle( tracePort->parseInt());
    break;
  case LIGHTS_RANDOM_DUTY_CYCLE_TIME:
    Alarm.free( dutyCycleChangeAlarm.id );
    dutyCycleChangeAlarm.interval = tracePort->parseInt();
    dutyCycleChangeAlarm.id = Alarm.timerRepeat(dutyCycleChangeAlarm.interval, alterLEDPattern);
    *tracePort << F("Duty cycle time set to ") << dutyCycleChangeAlarm.interval << " s" << endl;
    break;
  default:
    break;
  }
}

void processAlarmMessage()
{
  char c = tracePort->read();
  if ( c == TIMER_ALARM_ON)
  {
    Alarm.free( lightsOnAlarm.id );
    lightsOnAlarm.hours = tracePort->parseInt(); //constrain(tracePort->parseInt(), 0, 23);
    lightsOnAlarm.minutes = tracePort->parseInt(); //constrain(tracePort->parseInt(), 0, 59);
    lightsOnAlarm.seconds = tracePort->parseInt(); // constrain(tracePort->parseInt(), 0, 59);
    lightsOnAlarm.id = Alarm.alarmRepeat(lightsOnAlarm.hours, lightsOnAlarm.minutes, lightsOnAlarm.seconds, doLightsOn);
    displayAlarm("Lights On Alarm", lightsOnAlarm );
    // *tracePort << tracePort->parseInt() << "-" << tracePort->parseInt() << "-" << tracePort->parseInt() << endl;
  }
  else if ( c == TIMER_ALARM_OFF)
  {
    Alarm.free( lightsOffAlarm.id );
    lightsOffAlarm.hours = tracePort->parseInt(); //constrain(tracePort->parseInt(), 0, 23);
    lightsOffAlarm.minutes = tracePort->parseInt(); //constrain(tracePort->parseInt(), 0, 59);
    lightsOffAlarm.seconds = tracePort->parseInt(); // constrain(tracePort->parseInt(), 0, 59);
    lightsOffAlarm.id = Alarm.alarmRepeat(lightsOffAlarm.hours, lightsOffAlarm.minutes, lightsOffAlarm.seconds, doLightsOff);
    displayAlarm("Lights Off Alarm", lightsOffAlarm );
  }
}

void showCommands()
{
  *tracePort << F("Dt - Display Date/Time") << endl;
  *tracePort << F("Da - Display Alarms") << endl;
  *tracePort << F("T9999999999 - Set time using UNIX Epoch numner") << endl;
  *tracePort << F("A1HH:MM:SS - Set lights on time ") << endl;
  *tracePort << F("A0HH:MM:SS  - Set lights off time ") << endl;
  *tracePort << F("L1  - Lights On ") << endl;
  *tracePort << F("L0  - Lights Off ") << endl;
  *tracePort << F("Ld99 - Lighting duty cycle 99 From 60 to 90") << endl;
  *tracePort << F("Lr99999 - Lighting time to randomly change duty cycle in seconds") << endl;
  *tracePort << F("?? - Display commands") << endl;
}
void processSyncMessage()
{
  unsigned long pctime;
  pctime = tracePort->parseInt();
  //*tracePort << pctime << endl;
  if ( pctime >= DEFAULT_TIME)   // check the integer is a valid time (greater than Jan 1 2013)
  {
    RTC.set(pctime);   // set the RTC and the system time to the received value
    setTime(pctime); // Sync Arduino clock to the time received on the Serial2 port
    displayTime();
  }
}



void doLightsOn()
{
  strip1.turnOn();
  //strip2.turnOn();
  *tracePort << "...Lights on" << endl;
}


void doLightsOff()
{
  strip1.turnOff();
  //strip2.turnOff();
  *tracePort << "...Lights off" << endl;
}


void refreshModbusRegisters()
{
  modbusRegisters[HR_LED_DUTY_CYCLE] = strip1.getDutyCycle();
  modbusRegisters[HR_LED_DUTY_CYCLE_PERIOD] = dutyCycleChangeAlarm.interval;
  blconvert.val = AlarmHMS( lightsOnAlarm.hours, lightsOnAlarm.minutes, lightsOnAlarm.seconds);
  modbusRegisters[ HR_LED_ON_TIME ] = blconvert.regsl[0];
  modbusRegisters[ HR_LED_ON_TIME + 1 ] = blconvert.regsl[1];
  blconvert.val = AlarmHMS( lightsOffAlarm.hours, lightsOffAlarm.minutes, lightsOffAlarm.seconds);
  modbusRegisters[ HR_LED_OFF_TIME ] = blconvert.regsl[0];
  modbusRegisters[ HR_LED_OFF_TIME + 1 ] = blconvert.regsl[1];
  blconvert.val = now();
  modbusRegisters[ HR_CURRENT_TIME ] = blconvert.regsl[0];
  modbusRegisters[ HR_CURRENT_TIME + 1 ] = blconvert.regsl[1];
}

void processModbusCommands()
{
  if (  modbusRegisters[HR_SET_TIME] != 0 )
  {
    unsigned long pctime;
    blconvert.regsl[0] = modbusRegisters[ HR_SET_TIME ];
    blconvert.regsl[1] = modbusRegisters[ HR_SET_TIME + 1 ];
    pctime = blconvert.val;
    RTC.set(pctime);   // set the RTC and the system time to the received value
    setTime(pctime); // Sync Arduino clock to the time received on the Serial2 port
    modbusRegisters[ HR_SET_TIME ] = 0;
    modbusRegisters[ HR_SET_TIME + 1 ] = 0;
  }
}