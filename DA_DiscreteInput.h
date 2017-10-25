/** 
 *  @file    DA_DiscreteInput.h
 *  @author  peter c
 *  @date    4/14/2017  
 *  @version 0.1
 *  
 *
 *  @section DESCRIPTION
 *  Simple base class to represent an arduino digital input

 */
#ifndef DA_DISCRETE_INPUT_H
#define DA_DISCRETE_INPUT_H
#include "DA_Input.h"


#define DEFAULT_DEBOUNCE_TIME 100 // 100 ms

class DA_DiscreteInput: public DA_Input
{
public:
  /**
   * [RisingEdgeDetect low to high detection
   * [FalliongEdgeDetect high to low detection]
   * [ToggleDetect detect a change in state]
   */
  enum edgeDetectType  { None, RisingEdgeDetect, FallingEdgeDetect, ToggleDetect };
  // default toggle detect, pull-up resistor off
  DA_DiscreteInput(  uint8_t aPin );
  DA_DiscreteInput(  uint8_t aPin, DA_DiscreteInput::edgeDetectType aEdgeDectType, bool aEnableInternalPullup ) ;
  bool getSample();  // return debounced current state 

  void setOnPollCallBack( void (*callBack)( bool aValue ));
  void setDebounceTime( unsigned int aDebounceTime); //ms
  void setEdgeDetectType( DA_DiscreteInput::edgeDetectType aEdgeDectType );
  void setOnEdgeEvent( void (*callBack)( bool aValue ));
  void enableInternalPullup();  // internal pull up resitor enabled
  void disableInternalPullup(); // default-input pin floats

  void serialize( HardwareSerial *tracePort, bool includeCR); 
private:

  bool isInputToggled();
 // bool isFirstSample = false;  // to avoid false edge dectection
  bool currentRawSample;
  bool previousState = false; // for edge dectection
  bool previousDebounceRead = false;

  bool sampleDebounced = false;  // true sample ready
  unsigned int debounceTime = DEFAULT_DEBOUNCE_TIME;
  unsigned int debounceTimestamp = 0;
  void (*onPoll)( int value ) = NULL;
  void (*onEdgeDetect)( bool aValue ) = NULL;
  void onRefresh();
  void doEdgeDetection();
  bool debouncedRead();
  DA_DiscreteInput::edgeDetectType edgeDectionType = DA_DiscreteInput::ToggleDetect;

};

#endif
