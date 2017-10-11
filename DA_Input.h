/** 
 *  @file    DA_Input.h
 *  @author  peter c
 *  @date    4/14/2017  
 *  @version 0.1
 *  
 *
 *  @section DESCRIPTION
 *  Simple base class to Discrete and Analog inputs

 */


#ifndef DA_INPUT_H
#define DA_INPUT_H


#include "DA_IO.h"
#include <HardwareSerial.h>
#define DEFAULT_POLLING_INTERVAL 50 // ms 


class DA_Input
{

public:
  DA_Input( IO_TYPE aInputType, uint8_t aPin );
  void refresh(); 
  //void setOnPollCallBack( void (*callBack)( int scaledValue ));
  /**
   * [setPollingInterval how often to read inputs]
   * @param aPollingInterval [in ms]
   */
  void setPollingInterval( unsigned int aPollingInterval );
  virtual void serialize( HardwareSerial *tracePort, bool includeCR );
protected:
  virtual void onRefresh() = 0;
  //  virtual void doAlarmCheck() = 0;
  int  pin;
  unsigned int rawValue;
  //T scaledValue;


  IO_TYPE inputType;
private:
  unsigned long lastUpdateTime = 0;
  unsigned int pollingInterval = DEFAULT_POLLING_INTERVAL;

};


#endif
