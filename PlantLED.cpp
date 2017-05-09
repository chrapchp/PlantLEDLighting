#include <Streaming.h>
#include "PlantLED.h"

PlantLEDStrip::PlantLEDStrip(int length,  uint8_t pin, uint8_t pixel_type ) : Adafruit_NeoPixel( length, pin, pixel_type ),
  stripLength(length)
{
  //segmentLength = length;
  currentPattern  = MOSTLY_RED;
  currentDutyCycle = 90;
  majorityColor = RED;
  minorityColor = BLUE;
  //  Serial << " Color red = " << RED << " neo red =" << Color(255,0,0);
}


void PlantLEDStrip::refresh()
{
  if ( isLightsOn() )
  {
    doRegularPlantLighting();
    show();
  }
}

void PlantLEDStrip::initialize()
{
  begin();
  show();
}


void PlantLEDStrip::setPattern( stripPattern aPattern )
{
  currentPattern = aPattern;
  switch ( aPattern )
  {
  case MOSTLY_BLUE:
    majorityColor = BLUE;
    minorityColor = RED;
    break;
  case MOSTLY_RED:
    majorityColor = RED;
    minorityColor = BLUE;
    break;
  //    case BLEND_BLUE_RED:
  //      majorityColor = 0xFF00FF;
  //      minorityColor = 0xFF00FF;
  //      break;
  default:
    majorityColor = RED;
    minorityColor = BLUE;
    currentPattern = MOSTLY_RED;
  }
}


void PlantLEDStrip::fillSegment( uint32_t color )
{
  for ( int i = 0; i < numPixels(); i++ )
  {
    setPixelColor( i, color );
  }
}

unsigned short PlantLEDStrip::getDutyCycle()
{
  return ( currentDutyCycle);
}

void PlantLEDStrip::setDutyCycle( unsigned short aDutyCycle )
{

  currentDutyCycle = constrain(aDutyCycle, MIN_DUTY_CYCLE, MAX_DUTY_CYCLE);
}

void PlantLEDStrip::randomizeDutyCycle()
{
  setDutyCycle( random(MIN_DUTY_CYCLE, MAX_DUTY_CYCLE ));
}

void PlantLEDStrip::turnOff()
{
  _isLightsOn = false;
  fillSegment( BLACK );
  show();
}

void PlantLEDStrip::turnOn()
{
  _isLightsOn = true;
}

bool PlantLEDStrip::isLightsOn()
{
  return( _isLightsOn );
}
void PlantLEDStrip::flipColors()
{
  if ( isLightsOn() )
  {
    uint32_t tColor = majorityColor;
    majorityColor = minorityColor;
    minorityColor = tColor;
  }
}

void PlantLEDStrip::doRegularPlantLighting( )
{
  uint16_t i, j;
  uint8_t lDuty = currentDutyCycle > MIN_DUTY_CYCLE - 1 ? currentDutyCycle : MIN_DUTY_CYCLE;
  uint16_t segmentSize = (int) numPixels() / NUM_OF_SEGMENTS;
  uint8_t dutyHighCount = (int) segmentSize * currentDutyCycle / 100;
  uint8_t dutyLowCount =  (int) segmentSize - dutyHighCount;
  //  Serial << "dutyHighCount = " <<  dutyHighCount << " dutyLowCount = " << dutyLowCount << endl;
  // uint32_t blue = strip.Color(0, 0, 255);
  // uint32_t red = strip.Color(255, 0, 0);
  lDuty =  currentDutyCycle < MIN_DUTY_CYCLE + 1 ? currentDutyCycle : MAX_DUTY_CYCLE;
  fillSegment( majorityColor );
  for (  i = 0; i < NUM_OF_SEGMENTS; i++ )
  {
    for (  j = dutyHighCount; j < dutyLowCount + dutyHighCount; j++ )
    {
      setPixelColor( i * segmentSize + j, minorityColor );
    }
  }
  show();
}


