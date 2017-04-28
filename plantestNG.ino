
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

#include <Time.h>
#include <HardwareSerial.h>
#include <TimeLib.h>
#include <Wire.h>
#include <DS3232RTC.h>    //http://github.com/JChristensen/DS3232RTC
#include <TimeAlarms.h>
#include <Streaming.h>

#include "PlantLED.h"
#include "DA_Analoginput.h"
#include "DA_Discreteinput.h"

#define STRIP_1_PIN 6
#define STRIP_2_PIN 5


#define MIN_DUTY_CYCLE 20
#define MAX_DUTY_CYCLE 90

#define POT_PIN  A3



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

#define TIME_REQUEST  7     // ASCII bell character requests a time sync message 
//#define HOST_COMMAND_CHECK_INTERVAL  1000
#define LIGHT_REFRESH_INTERVAL  100




//DA_AnalogInput TT_100 = DA_AnalogInput(  A4, -40.0, 40.0 );
//DA_DiscreteInput LSL_100 = DA_DiscreteInput(  12 );



// Parameter 1 = number of pixels in strip
// Parameter 2 = pin number (most are valid)
// Parameter 3 = pixel type flags, add together as needed:
//   NEO_KHZ800  800 KHz bitstream (most NeoPixel products w/WS2812 LEDs)
//   NEO_KHZ400  400 KHz (classic 'v1' (not v2) FLORA pixels, WS2811 drivers)
//   NEO_GRB     Pixels are wired for GRB bitstream (most NeoPixel products)
//   NEO_RGB     Pixels are wired for RGB bitstream (v1 FLORA pixels, not v2)
PlantLEDStrip strip1 = PlantLEDStrip(3 * 60 + 2 * 144  , STRIP_1_PIN, NEO_GRB + NEO_KHZ800);
//PlantLEDStrip strip2 = PlantLEDStrip(60  , STRIP_2_PIN, NEO_GRB + NEO_KHZ800);
//
HardwareSerial *comPort = &Serial;
int inputDutyCycle = 0;  // variable to store the value coming from the sensor


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

AlarmEntry lightsOnAlarm = { 4, 0, 0, dtINVALID_ALARM_ID };
AlarmEntry lightsOffAlarm = { 23, 0, 0,  dtINVALID_ALARM_ID};
AlarmIntervalEntry dutyCycleChangeAlarm = { 60 * 60, dtINVALID_ALARM_ID};


void displayAlarm( char *who, struct _AlarmEntry aAlarmEntry)
{
  *comPort << who << "id = " << aAlarmEntry.id << " set to "  << aAlarmEntry.hours ;
  printDigits(aAlarmEntry.minutes );
  printDigits(aAlarmEntry.seconds );
  *comPort << endl;
}

void onTT_100Sample( float aValue )
{
  *comPort << "TT-100 = " << aValue << endl;
}

void onTT_100SampleDeadband( float aValue )
{
  *comPort << "Deadband TT-100 = " << aValue << endl;
}

void onLSL_100Sample( bool aValue )
{
  *comPort << "LSL_100 = " << aValue << endl;
}

void onLSL_100EdgeDetect( bool state )
{
  *comPort << "ToggleDetected=" << state << endl;
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
  comPort->begin(9600);
  randomSeed(analogRead(0));
  //setSyncProvider( requestSync);  //set function to call when sync required
  setSyncProvider(RTC.get);  
  //setSyncInterval(30);
  if(timeStatus()!= timeSet) 
     *comPort << "Unable to sync with the RTC" << endl;
  else
     *comPort << "RTC has set the system time" << endl;     
  showCommands();
  *comPort << "Enter Command:" << endl;
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
  //*comPort << analogRead(POT_PIN);
  // inputDutyCycle = map(analogRead(POT_PIN), 0, 1023, MIN_DUTY_CYCLE, MAX_DUTY_CYCLE);
  //*comPort << inputDutyCycle;
  strip1.refresh();
  //strip2.refresh();
  doCommandFromHostCheck();
  Alarm.delay(LIGHT_REFRESH_INTERVAL);
  // TT_100.refresh();
  //LSL_100.refresh();
}


void doCommandFromHostCheck()
{
  if (comPort->available() > 1)
  {
    // wait for at least two characters
    char c = comPort->read();
    // *comPort << c << endl;
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
      comPort->read();
      showCommands();
    }
    else if ( c == TIMER_ALARM_HEADER)
    {
      processAlarmMessage();
    }
  }
}




void digitalClockDisplay()
{
  *comPort << hour() ;
  printDigits(minute());
  printDigits(second()) ;
  *comPort << " ";
  *comPort << dayStr(weekday()) << " " ;
  *comPort << monthShortStr(month()) << " ";
  *comPort << day() << " "  << year() << endl;
}

void printDigits(int digits)
{
  *comPort << ":";
  if (digits < 10)
    *comPort << '0';
  *comPort << digits;
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
  char c = comPort->read();
  if ( c == DISPLAY_TIME)
  {
    displayTime();
  }
  else if ( c == DISPLAY_ALARMS)
  {
    displayAlarm("...Lights On Alarm", lightsOnAlarm );
    displayAlarm("...Lights Off Alarm", lightsOffAlarm );
    *comPort << "Duty Cycle Period = " << dutyCycleChangeAlarm.interval << " s" << endl;
  }
  else if ( c == DISPLAY_DUTY_CYCLE)
  {
    *comPort << "Duty Cycle = " << strip1.getDutyCycle() << endl;
  }

}

void processShowTime()
{
  char c = comPort->read();
  displayTime();
}


void processLightsMessage()
{
  char c = comPort->read();
  switch (c)
  {
  case LIGHTS_ON:
    doLightsOn();
    break;
  case LIGHTS_OFF:
    doLightsOff();
    break;
  case LIGHTS_DUTY_CYCLE:
    strip1.setDutyCycle( comPort->parseInt());
    break;
  case LIGHTS_RANDOM_DUTY_CYCLE_TIME:
    Alarm.free( dutyCycleChangeAlarm.id );
      dutyCycleChangeAlarm.interval = comPort->parseInt();
      dutyCycleChangeAlarm.id = Alarm.timerRepeat(dutyCycleChangeAlarm.interval, alterLEDPattern);
      *comPort << "Duty cycle time set to " << dutyCycleChangeAlarm.interval << " s" << endl;
    break;
  default:
    break;
  }
}

void processAlarmMessage()
{
  char c = comPort->read();
  if ( c == TIMER_ALARM_ON)
  {
    Alarm.free( lightsOnAlarm.id );
    lightsOnAlarm.hours = comPort->parseInt(); //constrain(comPort->parseInt(), 0, 23);
    lightsOnAlarm.minutes = comPort->parseInt(); //constrain(comPort->parseInt(), 0, 59);
    lightsOnAlarm.seconds = comPort->parseInt(); // constrain(comPort->parseInt(), 0, 59);
    lightsOnAlarm.id = Alarm.alarmRepeat(lightsOnAlarm.hours, lightsOnAlarm.minutes, lightsOnAlarm.seconds, doLightsOn);
    displayAlarm("Lights On Alarm", lightsOnAlarm );
    // *comPort << comPort->parseInt() << "-" << comPort->parseInt() << "-" << comPort->parseInt() << endl;
  }
  else if ( c == TIMER_ALARM_OFF)
  {
    Alarm.free( lightsOffAlarm.id );
    lightsOffAlarm.hours = comPort->parseInt(); //constrain(comPort->parseInt(), 0, 23);
    lightsOffAlarm.minutes = comPort->parseInt(); //constrain(comPort->parseInt(), 0, 59);
    lightsOffAlarm.seconds = comPort->parseInt(); // constrain(comPort->parseInt(), 0, 59);
    lightsOffAlarm.id = Alarm.alarmRepeat(lightsOffAlarm.hours, lightsOffAlarm.minutes, lightsOffAlarm.seconds, doLightsOff);
    displayAlarm("Lights Off Alarm", lightsOffAlarm );
  }
}

void showCommands()
{
  *comPort << "Fs - Format time display short format" << endl;
  *comPort << "Fl - Format time display long format" << endl;
  *comPort << "Dt - Display Date/Time" << endl;
  *comPort << "Da - Display Alarms" << endl;
  *comPort << "T9999999999 - Set time using UNIX Epoch numner" << endl;
  *comPort << "A1HH:MM:SS - Set lights on time " << endl;
  *comPort << "A0HH:MM:SS  - Set lights off time " << endl;
  *comPort << "L1  - Lights On " << endl;
  *comPort << "L0  - Lights Off " << endl;
  *comPort << "Ld99 - Lighting duty cycle 99 From 60 to 90" << endl;
  *comPort << "Lr99999 - Lighting time to randomly change duty cycle in seconds" << endl;
  *comPort << "?? - Display commands" << endl;
}
void processSyncMessage()
{
  unsigned long pctime;
  const unsigned long DEFAULT_TIME = 976492800; 
  pctime = comPort->parseInt();
  //*comPort << pctime << endl;
  if ( pctime >= DEFAULT_TIME)   // check the integer is a valid time (greater than Jan 1 2013)
  {
    RTC.set(pctime);   // set the RTC and the system time to the received value
    setTime(pctime); // Sync Arduino clock to the time received on the Serial2 port
    displayTime();
  }
}

time_t requestSync()
{
  comPort->write(TIME_REQUEST);
  return 0; // the time will be sent later in response to Serial2 mesg
}


void doLightsOn()
{
  strip1.turnOn();
  //strip2.turnOn();
  *comPort << "...Lights on" << endl;
}


void doLightsOff()
{
  strip1.turnOff();
  //strip2.turnOff();
  *comPort << "...Lights off" << endl;
}



