
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

#include <TimeLib.h>
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
#define TIME_HEADER   'T'   // Header tag for serial time sync message

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
int inputDutyCycle = 0;  // variable to store the value coming from the sensor


struct _AlarmEntry
{
  short hours;
  short minutes;
  short seconds;
  AlarmId id;
} ;

typedef _AlarmEntry AlarmEntry;
AlarmEntry lightsOnAlarm = { 4, 0, 0, dtINVALID_ALARM_ID };
AlarmEntry lightsOffAlarm = { 23, 0, 0,  dtINVALID_ALARM_ID};



void displayAlarm( char *who, struct _AlarmEntry aAlarmEntry)
{
  Serial << who << "id = " << aAlarmEntry.id << " set to "  << aAlarmEntry.hours ;
  printDigits(aAlarmEntry.minutes );
  printDigits(aAlarmEntry.seconds );
  Serial << endl;
}

void onTT_100Sample( float aValue )
{
  Serial << "TT-100 = " << aValue << endl;
}

void onTT_100SampleDeadband( float aValue )
{
  Serial << "Deadband TT-100 = " << aValue << endl;
}

void onLSL_100Sample( bool aValue )
{
  Serial << "LSL_100 = " << aValue << endl;
}

void onLSL_100EdgeDetect( bool state )
{
  Serial << "ToggleDetected=" << state << endl;
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
  Serial.begin(9600);
  randomSeed(analogRead(0));
  setSyncProvider( requestSync);  //set function to call when sync required
  showCommands();
  Serial << "Enter Command:" << endl;
  setTime(7, 13, 00, 25, 4, 17);
  lightsOnAlarm.id = Alarm.alarmRepeat(lightsOnAlarm.hours, lightsOnAlarm.minutes, lightsOnAlarm.seconds, doLightsOn);
  displayAlarm("...Lights On Alarm", lightsOnAlarm );
  lightsOffAlarm.id = Alarm.alarmRepeat(lightsOffAlarm.hours, lightsOffAlarm.minutes, lightsOffAlarm.seconds, doLightsOff);
  displayAlarm("...Lights Off Alarm", lightsOffAlarm );
  Alarm.timerRepeat(60 * 60, alterLEDPattern);
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
  //Serial << analogRead(POT_PIN);
  // inputDutyCycle = map(analogRead(POT_PIN), 0, 1023, MIN_DUTY_CYCLE, MAX_DUTY_CYCLE);
  //Serial << inputDutyCycle;
  strip1.refresh();
  //strip2.refresh();
  doCommandFromHostCheck();
  Alarm.delay(LIGHT_REFRESH_INTERVAL);
  // TT_100.refresh();
  //LSL_100.refresh();
}


void doCommandFromHostCheck()
{
  if (Serial.available() > 1)
  {
    // wait for at least two characters
    char c = Serial.read();
    // Serial << c << endl;
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
      Serial.read();
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
  Serial << hour() ;
  printDigits(minute());
  printDigits(second()) ;
  Serial << " ";
  Serial << dayStr(weekday()) << " " ;
  Serial << monthShortStr(month()) << " ";
  Serial << day() << " "  << year() << endl;
}

void printDigits(int digits)
{
  Serial << ":";
  if (digits < 10)
    Serial << '0';
  Serial << digits;
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
  char c = Serial.read();
  if ( c == DISPLAY_TIME)
  {
    displayTime();
  }
  else if ( c == DISPLAY_ALARMS)
  {
    displayAlarm("...Lights On Alarm", lightsOnAlarm );
    displayAlarm("...Lights Off Alarm", lightsOffAlarm );
  }
  else if ( c == DISPLAY_DUTY_CYCLE)
  {
    Serial << "Duty Cycle = " << strip1.getDutyCycle() << endl;
  }
}

void processShowTime()
{
  char c = Serial.read();
  displayTime();
}


void processLightsMessage()
{
  char c = Serial.read();
  switch (c)
  {
  case LIGHTS_ON:
    doLightsOn();
    break;
  case LIGHTS_OFF:
    doLightsOff();
    break;
  case LIGHTS_DUTY_CYCLE:
    strip1.setDutyCycle( Serial.parseInt());
    break;
  case LIGHTS_RANDOM_DUTY_CYCLE_TIME:
    break;
  default:
    break;
  }
}

void processAlarmMessage()
{
  char c = Serial.read();
  if ( c == TIMER_ALARM_ON)
  {
    Alarm.free( lightsOnAlarm.id );
    lightsOnAlarm.hours = Serial.parseInt(); //constrain(Serial.parseInt(), 0, 23);
    lightsOnAlarm.minutes = Serial.parseInt(); //constrain(Serial.parseInt(), 0, 59);
    lightsOnAlarm.seconds = Serial.parseInt(); // constrain(Serial.parseInt(), 0, 59);
    lightsOnAlarm.id = Alarm.alarmRepeat(lightsOnAlarm.hours, lightsOnAlarm.minutes, lightsOnAlarm.seconds, doLightsOn);
    displayAlarm("Lights On Alarm", lightsOnAlarm );
    // Serial << Serial.parseInt() << "-" << Serial.parseInt() << "-" << Serial.parseInt() << endl;
  }
  else if ( c == TIMER_ALARM_OFF)
  {
    Alarm.free( lightsOffAlarm.id );
    lightsOffAlarm.hours = Serial.parseInt(); //constrain(Serial.parseInt(), 0, 23);
    lightsOffAlarm.minutes = Serial.parseInt(); //constrain(Serial.parseInt(), 0, 59);
    lightsOffAlarm.seconds = Serial.parseInt(); // constrain(Serial.parseInt(), 0, 59);
    lightsOffAlarm.id = Alarm.alarmRepeat(lightsOffAlarm.hours, lightsOffAlarm.minutes, lightsOffAlarm.seconds, doLightsOff);
    displayAlarm("Lights Off Alarm", lightsOffAlarm );
  }
}

void showCommands()
{
  Serial << "Fs - Format time display short format" << endl;
  Serial << "Fl - Format time display long format" << endl;
  Serial << "Dt - Display Date/Time" << endl;
  Serial << "Da - Display Alarms" << endl;
  Serial << "T9999999999 - Set time using UNIX Epoch numner" << endl;
  Serial << "A1HH:MM:SS - Set lights on time " << endl;
  Serial << "A0HH:MM:SS  - Set lights off time " << endl;
  Serial << "L1  - Lights On " << endl;
  Serial << "L0  - Lights Off " << endl;
  Serial << "Ld99 - Lighting duty cycle 99 From 60 to 90" << endl;
  Serial << "Lr99999 - Lighting time to randomly change duty cycle in seconds" << endl;
  Serial << "?? - Display commands" << endl;
}
void processSyncMessage()
{
  unsigned long pctime;
  const unsigned long DEFAULT_TIME = 1357041600; // Jan 1 2013 - paul, perhaps we define in time.h?
  pctime = Serial.parseInt();
  //Serial << pctime << endl;
  if ( pctime >= DEFAULT_TIME)   // check the integer is a valid time (greater than Jan 1 2013)
  {
    setTime(pctime); // Sync Arduino clock to the time received on the serial port
    displayTime();
  }
}

time_t requestSync()
{
  Serial.write(TIME_REQUEST);
  return 0; // the time will be sent later in response to serial mesg
}


void doLightsOn()
{
  strip1.turnOn();
  //strip2.turnOn();
  Serial << "...Lights on" << endl;
}


void doLightsOff()
{
  strip1.turnOff();
  //strip2.turnOff();
  Serial << "...Lights off" << endl;
}



