#include <Streaming.h>
#include "DA_AnalogInput.h"

DA_AnalogInput::DA_AnalogInput( uint8_t aPin, float aEUMin, float aEUMax ): DA_Input(analog, aPin )
{

  euMin = aEUMin;
  euMax = aEUMax;
  euSpan = euMax - euMin;
}


void DA_AnalogInput::setEUMin( float aEUMin )
{
  euMin = aEUMin;
  euSpan = euMax - euMin;
}

/**
 * [DA_AnalogInput description]
 * @type {[type]}
 */
void DA_AnalogInput::setEUMax( float aEUMax )
{
  euMax = aEUMax;
  euSpan = euMax - euMin;
}

/*
  unsigned int DA_AnalogInput::getRawSample() {
  return ( currentRawSample );
  }

  float DA_AnalogInput::getScaledSample() {
  return ( currentScaledSample );

  }

*/

float DA_AnalogInput::fmap( unsigned int raw, unsigned int raw_min, unsigned int raw_max, float EU_min, float EU_max)
{
  float retVal;

  retVal = (raw - raw_min) * (EU_max - EU_min) / (raw_max - raw_min) + EU_min;
  // Serial << retVal << endl;
  return ( retVal );
}

bool DA_AnalogInput::isOutsideDeadband( int aNewValue,  int aCurValue )
{
// Serial << "delta=" << abs(aNewValue - aCurValue) << " deadband=" << deadBand * 1023 << endl;
  return ( abs(aNewValue - aCurValue) >=  deadBand * 1023 );
}
/*
  bool DA_AnalogInput::isOutsideDeadband(float aNewValue, float aCurValue ) {
  float result = constrain( aNewValue, aCurValue * (1 - deadBand), aCurValue * (1 + deadBand));
  Serial << "current value=" << aCurValue << " result=" << result << "neval=" << aNewValue << " LDB=" <<  aCurValue * (1 - deadBand) << " hDB=" << aCurValue * (1 + deadBand) << endl;
  return ( !(result == aNewValue));
  }
*/
void DA_AnalogInput::onRefresh()
{

  unsigned int pendingRawValue = analogRead(pin);
  float pendingScaledValue = fmap(pendingRawValue, 0, 1023, euMin, euMax);

  if ( isFirstSample || isOutsideDeadband( pendingRawValue, previousRawSample ))
  {
    currentScaledSample = pendingScaledValue;
    previousRawSample = pendingRawValue;
    if (onOutsideDeadbandDetected != NULL)
    {
      //Serial << getScaledSample() << endl;
      onOutsideDeadbandDetected(getScaledSample());
    }
  }
  currentRawSample =  pendingRawValue;
  if (onPoll != NULL)
  {

    onPoll(pendingScaledValue);
  }
  isFirstSample = false;
}

void DA_AnalogInput::setOnPollCallBack( void (*callBack)( float scaledValue ))
{
  onPoll = callBack;
}

void DA_AnalogInput::setOutsideDeadbandDetectedEvent( void (*callBack)( float scaledValue ))
{
  onOutsideDeadbandDetected = callBack;
}

float DA_AnalogInput::setDeadband( float aPercentage )   // +/- .11 -> 11% deadband
{
  deadBand = aPercentage;
}




