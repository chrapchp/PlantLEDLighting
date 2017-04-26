/** 
 *  @file    DA_AnalogInput.h
 *  @author  peter c
 *  @date    4/14/2017  
 *  @version 0.1
 *  
 *
 *  @section DESCRIPTION
 *  Simple class to represent an arduinio analog input
 *  
 *
 */

#ifndef DA_ANALOG_INPUT_H
#define DA_ANALOG_INPUT_H
#include "DA_Input.h"

#define DEAD_BAND_DEFAULT  0.0

class DA_AnalogInput: public DA_Input
{
public:
  DA_AnalogInput(  uint8_t aPin, float aEUMin, float aEUMax );
  void setEUMin( float aEUMin);
  void setEUMax( float aEUMax);
  //unsigned int getRawSample();
  inline unsigned int getRawSample() __attribute__((always_inline))
  {
    return currentRawSample;
  }

  inline float getScaledSample() __attribute__((always_inline))
  {
    return currentScaledSample;
  }

/**
 * [setOnPollCallBack sets callback on each poll regardless of deadband. ]
*/
  void setOnPollCallBack( void (*callBack)( float scaledValue ));
/**
 * [setOutsideDeadbandDetectedEvent sets callback when poll value exceed deadband]
*/
  void setOutsideDeadbandDetectedEvent( void (*callBack)( float scaledValue ));
/**
 * [setDeadband sets deadband for analog point // +/- .11 -> +/- 11% deadband]
*/  
  float setDeadband( float aPercentage ); 

private:
  float euMin = 0.0;
  float euMax = 1023.0;
  float euSpan = euMax - euMin;

  float fmap( unsigned int raw, unsigned int raw_min, unsigned int raw_max, float EU_min, float EU_max);

  bool isOutsideDeadband( int aNewValue,  int aCurValue );  //
  // bool isOutsideDeadband(float aCurValue ); // deadband from full EU range
  unsigned int currentRawSample = 0;
  unsigned int previousRawSample = 0;
  float currentScaledSample = 0;

  float deadBand = DEAD_BAND_DEFAULT;
  void (*onPoll)( float value ) = NULL;
  void (*onOutsideDeadbandDetected)( float aValue ) = NULL;
  void onRefresh();
  bool isFirstSample = true;  // to avoid false edge dectection

};

#endif
