

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
#include <avr/eeprom.h>


#include <TimeAlarms.h>
#include <Streaming.h>

#include "PlantModbus.h"
#include "PlantLED.h"
#include "DA_Analoginput.h"
#include "DA_Discreteinput.h"

#define STRIP_1_PIN 6

#define DEFAULT_LIGHTS_ON_ALARM_TIME AlarmHMS(4, 0, 0)
#define DEFAULT_LIGHTS_OFF_ALARM_TIME AlarmHMS(23, 0, 0)
#define DEFAULT_LIGHTS_DUTY_CYCLE_PERIOD 60 * 60


// single character message tags from terminal host
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
#define LIGHTS_DUTY_CYCLE_PERIOD 'r' // Lr3600 -> change the duty cycle between 60-90 % every hour
#define LIGHTS_RESET_TO_DEFAULTS 'c'


//#define HOST_COMMAND_CHECK_INTERVAL  1000
#define LIGHT_REFRESH_INTERVAL  100

const unsigned long DEFAULT_TIME = 976492800;

// comment out to not include terminal processing
#define PROCESS_TERMINAL


#define EEPROM_CONFIGURED       2   // this value stored at address CONFIG_FLAG_ADDR 
#define EEPROM_CONFIG_FLAG_ADDR 0   // is 0 if nothing was written
#define EEPROM_LED_LIGHTS_ON_TIME_ADDR EEPROM_CONFIG_FLAG_ADDR + sizeof(unsigned short)
#define EEPROM_LED_LIGHTS_OFF_TIME_ADDR EEPROM_LED_LIGHTS_ON_TIME_ADDR + sizeof(time_t)
#define EEPROM_LIGHTS_DUTY_CYCLE_PERIOD  EEPROM_LED_LIGHTS_OFF_TIME_ADDR + sizeof(time_t)


//DA_AnalogInput TT_100 = DA_AnalogInput(  A4, -40.0, 40.0 );
//DA_DiscreteInput LSL_100 = DA_DiscreteInput(  12 );



PlantLEDStrip plantStrip = PlantLEDStrip(3 * 60 + 2 * 144  , STRIP_1_PIN, NEO_GRB + NEO_KHZ800);
//PlantLEDStrip strip2 = PlantLEDStrip(60  , STRIP_2_PIN, NEO_GRB + NEO_KHZ800);

HardwareSerial *tracePort = &Serial2;

TimeChangeRule usMDT = {"MDT", Second, dowSunday, Mar, 2, -360};
TimeChangeRule usMST = {"MST", First, dowSunday, Nov, 2, -420};
Timezone usMT(usMDT, usMST);

struct _AlarmEntry
{
  time_t epoch;
  AlarmId id = dtINVALID_ALARM_ID;
} ;


typedef _AlarmEntry AlarmEntry;



AlarmEntry lightsOnAlarm ; //= { AlarmHMS(4, 0, 0), dtINVALID_ALARM_ID };
AlarmEntry lightsOffAlarm ; //= { AlarmHMS(11, 0, 0),  dtINVALID_ALARM_ID};
AlarmEntry dutyCycleChangeAlarm ; //  = { 60 * 60, dtINVALID_ALARM_ID};


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
  plantStrip.randomizeDutyCycle();
  //strip2.setDutyCycle( random(80, 90) );
// plantStrip.flipColors();
  //strip2.flipColors();
}

void setup()
{
#ifdef PROCESS_TERMINAL
  tracePort->begin(9600);
#endif
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
  //showCommands();
  *tracePort << F("Enter Command:") << endl;
  /*
   lightsOnAlarm.id = Alarm.alarmRepeat(lightsOnAlarm.epoch, doLightsOn);
   displayAlarm("...Lights On Alarm", lightsOnAlarm );
   lightsOffAlarm.id = Alarm.alarmRepeat(lightsOffAlarm.epoch, doLightsOff);
   displayAlarm("...Lights Off Alarm", lightsOffAlarm );
   dutyCycleChangeAlarm.id = Alarm.timerRepeat(dutyCycleChangeAlarm.epoch, alterLEDPattern);
   */
  plantStrip.initialize();
  if (isEEPROMConfigured() == EEPROM_CONFIGURED )
  {
    EEPROMLoadConfig();
  }
  else
  {
    EEPROMWriteDefaultConfig();
    EEPROMLoadConfig();
  }
//*tracePort << "offset=" + timeToLocal << endl;
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
  plantStrip.refresh();
  refreshModbusRegisters();
  slave.poll( modbusRegisters, MODBUS_REG_COUNT );
  processModbusCommands();
#ifdef PROCESS_TERMINAL
  processTerminalCommands();
#endif
  Alarm.delay(LIGHT_REFRESH_INTERVAL);
  // TT_100.refresh();
  //LSL_100.refresh();
}


/*
 ** handle terminal commands from host
 */
#ifdef PROCESS_TERMINAL

void displayAlarm( char *who, struct _AlarmEntry aAlarmEntry)
{
  *tracePort << who << "id = " << aAlarmEntry.id << " UTC set to "  << hour(aAlarmEntry.epoch) ;
  printDigits(minute(aAlarmEntry.epoch) );
  printDigits(second(aAlarmEntry.epoch ) );
  time_t localAlarmTime = alarmTimeToLocal(aAlarmEntry.epoch);
  *tracePort << " UTC epoch is " << aAlarmEntry.epoch ;
  *tracePort << " Local is " << hour(localAlarmTime) ;
  printDigits(minute(localAlarmTime) );
  printDigits(second(localAlarmTime ) );
  *tracePort << " epoch is " << localAlarmTime << endl;
}


void processTerminalCommands()
{
  if (tracePort->available() > 1)
  {
    // wait for at least two characters
    char c = tracePort->read();
    // *tracePort << c << endl;
    if ( c == TIME_HEADER)
    {
      processTimeSetMessage();
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
    *tracePort << "Duty Cycle Period = " << dutyCycleChangeAlarm.epoch << " s id=" << dutyCycleChangeAlarm.id << endl;
  }
  else if ( c == DISPLAY_DUTY_CYCLE)
  {
    *tracePort << "Duty Cycle = " << plantStrip.getDutyCycle() << endl;
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
    plantStrip.setDutyCycle( tracePort->parseInt());
    break;
  case LIGHTS_DUTY_CYCLE_PERIOD:
    Alarm.free( dutyCycleChangeAlarm.id );
    dutyCycleChangeAlarm.epoch = tracePort->parseInt();
    dutyCycleChangeAlarm.id = Alarm.timerRepeat(dutyCycleChangeAlarm.epoch, alterLEDPattern);
    EEPROMWriteAlarmEntry( dutyCycleChangeAlarm.epoch, EEPROM_LIGHTS_DUTY_CYCLE_PERIOD );
    *tracePort << F("Duty cycle time set to ") << dutyCycleChangeAlarm.epoch << " s" << endl;
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
  if ( c == TIMER_ALARM_ON)
  {
    Alarm.free( lightsOnAlarm.id );
    unsigned int shour = tracePort->parseInt(); //constrain(tracePort->parseInt(), 0, 23);
    unsigned int sminute = tracePort->parseInt(); //constrain(tracePort->parseInt(), 0, 59);
    unsigned int ssecond = tracePort->parseInt(); // constrain(tracePort->parseInt(), 0, 59);
    lightsOnAlarm.epoch = alarmTmeToUTC(AlarmHMS(shour, sminute, ssecond));
    lightsOnAlarm.id = Alarm.alarmRepeat(lightsOnAlarm.epoch, doLightsOn);
    EEPROMWriteAlarmEntry( lightsOnAlarm.epoch, EEPROM_LED_LIGHTS_ON_TIME_ADDR );
    displayAlarm("Lights On Alarm", lightsOnAlarm );
    // *tracePort << tracePort->parseInt() << "-" << tracePort->parseInt() << "-" << tracePort->parseInt() << endl;
  }
  else if ( c == TIMER_ALARM_OFF)
  {
    Alarm.free( lightsOffAlarm.id );
    unsigned int shour = tracePort->parseInt(); //constrain(tracePort->parseInt(), 0, 23);
    unsigned int sminute = tracePort->parseInt(); //constrain(tracePort->parseInt(), 0, 59);
    unsigned int ssecond = tracePort->parseInt(); // constrain(tracePort->parseInt(), 0, 59);
    lightsOffAlarm.epoch = alarmTmeToUTC(AlarmHMS(shour, sminute, ssecond));
    lightsOffAlarm.id = Alarm.alarmRepeat(lightsOffAlarm.epoch, doLightsOff);
    EEPROMWriteAlarmEntry( lightsOffAlarm.epoch, EEPROM_LED_LIGHTS_OFF_TIME_ADDR );
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
  *tracePort << F("Lc  - reset/clear settings to defaults ") << endl;
  *tracePort << F("?? - Display commands") << endl;
}


void processTimeSetMessage()
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
#endif


void doLightsOn()
{
  plantStrip.turnOn();
  //strip2.turnOn();
#ifdef PROCESS_TERMINAL
  *tracePort << "...Lights on" << endl;
#endif
}


void doLightsOff()
{
  plantStrip.turnOff();
  //strip2.turnOff();
#ifdef PROCESS_TERMINAL
  *tracePort << "...Lights off" << endl;
#endif
}

// timezone lib does not handle short 24 hr duration epochs
time_t alarmTmeToUTC( time_t localAlarmTime )
{
  time_t utcAlarmTime =  usMT.toUTC( localAlarmTime);
  if ( usMT.utcIsDST(now()))
  {
    utcAlarmTime -= 60 * 60;
  }
  if ( utcAlarmTime > SECS_PER_DAY)
    utcAlarmTime -= SECS_PER_DAY;
  return ( utcAlarmTime );
}

time_t alarmTimeToLocal( time_t utcAlarmTime )
{
  TimeChangeRule *tcr;        //pointer to the time change rule, use to get the TZ abbrev
  time_t localAlarmTime = usMT.toLocal(utcAlarmTime, &tcr);
  int offset = tcr->offset ; // in minutes
  if ( usMT.utcIsDST(now()))
  {
    offset += 60 ;
    localAlarmTime += 60 * 60;
  }
  if ( ( hour(utcAlarmTime) + (int) offset / 60)  < 0 )
  {
    int hr = 12 + (hour(utcAlarmTime) - (int)offset / 60);
    localAlarmTime = AlarmHMS( hr , minute(utcAlarmTime), second(utcAlarmTime));
  }
  return ( localAlarmTime);
}

/*
** Modbus related functions
*/

void refreshModbusRegisters()
{
  modbusRegisters[HR_LED_DUTY_CYCLE] = plantStrip.getDutyCycle();
  modbusRegisters[HR_LED_DUTY_CYCLE_PERIOD] = dutyCycleChangeAlarm.epoch;
  blconvert.val = lightsOnAlarm.epoch ;
  modbusRegisters[ HR_LED_ON_TIME ] = blconvert.regsl[0];
  modbusRegisters[ HR_LED_ON_TIME + 1 ] = blconvert.regsl[1];
  blconvert.val =  lightsOffAlarm.epoch;
  modbusRegisters[ HR_LED_OFF_TIME ] = blconvert.regsl[0];
  modbusRegisters[ HR_LED_OFF_TIME + 1 ] = blconvert.regsl[1];
  blconvert.val = now();
  modbusRegisters[ HR_CURRENT_TIME ] = blconvert.regsl[0];
  modbusRegisters[ HR_CURRENT_TIME + 1 ] = blconvert.regsl[1];
  modbusRegisters[ CS_LED_STATUS ] = plantStrip.isLightsOn();
  modbusRegisters[ CS_LED_STATUS ] = plantStrip.isLightsOn();
  writeModbusCoil( COIL_STATUS_READ_WRITE_OFFSET, CS_LED_STATUS, plantStrip.isLightsOn());
  //bitWrite(modbusRegisters[ COIL_STATUS_READ_OFFSET], CS_SET_LED_ON, plantStrip.isLightsOn());
}


void setModbusTime()
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

void setModbusLightsOnTime()
{
  if (  modbusRegisters[HR_SET_LED_ON_TIME] != 0 )
  {
    blconvert.regsl[0] = modbusRegisters[ HR_SET_LED_ON_TIME ];
    blconvert.regsl[1] = modbusRegisters[ HR_SET_LED_ON_TIME + 1 ];
    lightsOnAlarm.epoch  = alarmTmeToUTC(blconvert.val);
    Alarm.free( lightsOnAlarm.id );
    lightsOnAlarm.id = Alarm.alarmRepeat(lightsOnAlarm.epoch , doLightsOn);
    EEPROMWriteAlarmEntry( lightsOnAlarm.epoch, EEPROM_LED_LIGHTS_ON_TIME_ADDR );
    modbusRegisters[ HR_SET_LED_ON_TIME ] = 0;
    modbusRegisters[ HR_SET_LED_ON_TIME + 1 ] = 0;
  }
}

void setModbusLightsOffTime()
{
  if (  modbusRegisters[HR_SET_LED_OFF_TIME] != 0 )
  {
    blconvert.regsl[0] = modbusRegisters[ HR_SET_LED_OFF_TIME ];
    blconvert.regsl[1] = modbusRegisters[ HR_SET_LED_OFF_TIME + 1 ];
    lightsOffAlarm.epoch  = alarmTmeToUTC(blconvert.val);
    Alarm.free( lightsOffAlarm.id );
    lightsOffAlarm.id = Alarm.alarmRepeat(lightsOffAlarm.epoch , doLightsOff);
    EEPROMWriteAlarmEntry( lightsOffAlarm.epoch, EEPROM_LED_LIGHTS_OFF_TIME_ADDR );
    modbusRegisters[ HR_SET_LED_OFF_TIME ] = 0;
    modbusRegisters[ HR_SET_LED_OFF_TIME + 1 ] = 0;
  }
}

void setModbusDutyCycle()
{
  if (  modbusRegisters[HR_SET_DUTY_CYCLE] != 0 )
  {
    plantStrip.setDutyCycle( modbusRegisters[ HR_SET_DUTY_CYCLE ] );
    modbusRegisters[ HR_SET_DUTY_CYCLE ] = 0;
  }
}


void setModbusDutyCyclePeriod()
{
  if (  modbusRegisters[HR_SET_DUTY_CYCLE_PERIOD] != 0 )
  {
    Alarm.free( dutyCycleChangeAlarm.id );
    dutyCycleChangeAlarm.epoch = modbusRegisters[ HR_SET_DUTY_CYCLE_PERIOD ] ;
    dutyCycleChangeAlarm.id = Alarm.timerRepeat(dutyCycleChangeAlarm.epoch, alterLEDPattern);
    EEPROMWriteAlarmEntry( dutyCycleChangeAlarm.epoch, EEPROM_LIGHTS_DUTY_CYCLE_PERIOD );
    modbusRegisters[ HR_SET_DUTY_CYCLE_PERIOD ] = 0;
  }
}

void setModbusLightsOn()
{
//   *tracePort << "...modbus pre Lights on" <<  modbusRegisters[COIL_STATUS_WRITE_OFFSET+1]  << endl;
  if (  getModbusCoilValue( COIL_STATUS_READ_WRITE_OFFSET, CS_SET_LED_ON )  )
  {
    //*tracePort << "...got a bit" << endl;
    doLightsOn();
    writeModbusCoil( COIL_STATUS_READ_WRITE_OFFSET, CS_SET_LED_ON, false);
  }
}


void setModbusLightsOff()
{
//   *tracePort << "...modbus pre Lights on" <<  modbusRegisters[COIL_STATUS_WRITE_OFFSET+1]  << endl;
  if (  getModbusCoilValue( COIL_STATUS_READ_WRITE_OFFSET, CS_SET_LED_OFF )  )
  {
    doLightsOff();
    writeModbusCoil( COIL_STATUS_READ_WRITE_OFFSET, CS_SET_LED_OFF, false);
  }
}

void setConfigToDefaults()
{
  if (  getModbusCoilValue( COIL_STATUS_READ_WRITE_OFFSET, CS_RESET_TO_DEFAULTS )  )
  {
    EEPROMWriteDefaultConfig();
    EEPROMLoadConfig();
    writeModbusCoil( COIL_STATUS_READ_WRITE_OFFSET, CS_RESET_TO_DEFAULTS, false);
  }
}
bool getModbusCoilValue( unsigned short startAddress, unsigned short bitPos)
{
  return ( bitRead(modbusRegisters[startAddress + (int)(bitPos / 16)], bitPos % 16 ) );
}

void writeModbusCoil( unsigned short startAddress, unsigned short bitPos, bool value )
{
  bitWrite(modbusRegisters[startAddress + (int)(bitPos / 16)], bitPos % 16 , value) ;
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
  eeprom_write_block((const void*)&configFlag, (void*)EEPROM_CONFIG_FLAG_ADDR, sizeof(configFlag));
  time_t epoch = alarmTmeToUTC(DEFAULT_LIGHTS_ON_ALARM_TIME);
  EEPROMWriteAlarmEntry( epoch, EEPROM_LED_LIGHTS_ON_TIME_ADDR );
  epoch = alarmTmeToUTC(DEFAULT_LIGHTS_OFF_ALARM_TIME);
  EEPROMWriteAlarmEntry( epoch, EEPROM_LED_LIGHTS_OFF_TIME_ADDR );
  epoch = DEFAULT_LIGHTS_DUTY_CYCLE_PERIOD;
  EEPROMWriteAlarmEntry( epoch, EEPROM_LIGHTS_DUTY_CYCLE_PERIOD );
}

void EEPROMWriteAlarmEntry( time_t epoch, unsigned int atAddress )
{
  eeprom_write_block((const void*)&epoch, (void*)atAddress, sizeof(epoch));
}

time_t EEPROMReadAlarmEntry( unsigned int atAddress )
{
  time_t epoch = 0;
  eeprom_read_block((void*)&epoch, (void*)atAddress, sizeof(epoch));
  return ( epoch );
}

unsigned int isEEPROMConfigured()
{
  unsigned short configFlag;
  eeprom_read_block((void*)&configFlag, (void*)EEPROM_CONFIG_FLAG_ADDR, sizeof(configFlag));
  return ( configFlag);
}


void EEPROMLoadConfig()
{
  lightsOnAlarm.epoch = EEPROMReadAlarmEntry( EEPROM_LED_LIGHTS_ON_TIME_ADDR);
  Alarm.free( lightsOnAlarm.id );
  lightsOnAlarm.id = Alarm.alarmRepeat(lightsOnAlarm.epoch , doLightsOn);
  lightsOffAlarm.epoch = EEPROMReadAlarmEntry( EEPROM_LED_LIGHTS_OFF_TIME_ADDR);
  Alarm.free( lightsOffAlarm.id );
  lightsOffAlarm.id = Alarm.alarmRepeat( lightsOffAlarm.epoch, doLightsOff);
  dutyCycleChangeAlarm.epoch = EEPROMReadAlarmEntry( EEPROM_LIGHTS_DUTY_CYCLE_PERIOD);
  Alarm.free( dutyCycleChangeAlarm.id );
  dutyCycleChangeAlarm.id = Alarm.alarmRepeat(dutyCycleChangeAlarm.epoch, alterLEDPattern);
}