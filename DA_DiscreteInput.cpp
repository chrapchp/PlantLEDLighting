#include <Streaming.h>
#include "DA_DiscreteInput.h"

DA_DiscreteInput::DA_DiscreteInput( uint8_t aPin ): DA_Input(discrete, aPin )
{
  //pinMode(aPin, INPUT);
}

DA_DiscreteInput::DA_DiscreteInput(  uint8_t aPin, DA_DiscreteInput::edgeDetectType aEdgeDectType, bool aEnableInternalPullup ):DA_Input(discrete, aPin ) 
{
  setEdgeDetectType( aEdgeDectType );
  if( aEnableInternalPullup )
    enableInternalPullup(); 
}

/*
DA_DiscreteInput::DA_DiscreteInput(  uint8_t aPin, DA_DiscreteInput::edgeDetectType aEdgeDectType )
{
  DA_DiscreteInput( aPin);
}
*/
bool DA_DiscreteInput::getRawSample()
{
  return ( currentRawSample );
}


bool DA_DiscreteInput::isInputToggled()
{
  if ( currentRawSample != previousState )
  {
    previousState = currentRawSample;
    return ( true );
  }
  else
    return ( false );

}

void DA_DiscreteInput::doEdgeDetection()
{
  DA_DiscreteInput::edgeDetectType edgeDirection;
  bool invokeCallBack = false;

  if ( currentRawSample != previousState )
  {
    if ( previousState == HIGH )
    {
      edgeDirection = FallingEdgeDetect;
    }
    else
    {
      edgeDirection = RisingEdgeDetect;
    }

    previousState = currentRawSample;
    switch ( edgeDectionType )
    {
    case FallingEdgeDetect:
      if ( edgeDirection == FallingEdgeDetect )
        invokeCallBack = true;
      break;
    case RisingEdgeDetect:
      if ( edgeDirection == RisingEdgeDetect )
        invokeCallBack = true;
      break;
    case ToggleDetect:

      invokeCallBack = true;
      break;
    default:
      // nothing
      invokeCallBack = false;
    }

    //if (     !isFirstSample && invokeCallBack && onEdgeDetect != NULL )
    if (      invokeCallBack && onEdgeDetect != NULL )    
    {
      onEdgeDetect( currentRawSample );
    }
    //isFirstSample = false;
  }



}

bool DA_DiscreteInput::debouncedRead()
{
  currentRawSample = digitalRead(pin);
  bool retVal = false;
  if (currentRawSample != previousDebounceRead)
  {
    // reset the debouncing timer
    debounceTimestamp = millis();

  }
  if ((millis() - debounceTimestamp) > debounceTime)
  {
    retVal = true;
  }
  previousDebounceRead = currentRawSample;
  return ( retVal );
}

void DA_DiscreteInput::onRefresh()
{

  if ( debouncedRead() )
  {
    if (onPoll != NULL)
    {
      onPoll(getRawSample());
    }

    if ( edgeDectionType != None )
      doEdgeDetection();
  }
}


void DA_DiscreteInput::setDebounceTime( unsigned int aDebounceTime)
{
  debounceTime = aDebounceTime;
}

void DA_DiscreteInput::setOnPollCallBack( void (*callBack)( bool aValue ))
{
  onPoll = callBack;
}

void DA_DiscreteInput::setOnEdgeEvent( void (*callBack)( bool aValue ))
{
  onEdgeDetect = callBack;
}

void DA_DiscreteInput::setEdgeDetectType( DA_DiscreteInput::edgeDetectType aEdgeDectType )
{
  edgeDectionType = aEdgeDectType;
}


void DA_DiscreteInput::enableInternalPullup()    // internal pull up resitor enabled
{
  pinMode(pin, INPUT_PULLUP);

}

void DA_DiscreteInput::disableInternalPullup()   // default-input pin floats
{
  pinMode(pin, INPUT);
}


void DA_DiscreteInput::serialize( HardwareSerial *tracePort, bool includeCR )
{
  DA_Input::serialize( tracePort, false);

  *tracePort << "{currentState:" << getRawSample()  << " debounceTime:" << debounceTime << " ms edgeDetectType:" << edgeDectionType << "}";
  if( includeCR )
    *tracePort << endl;
}
