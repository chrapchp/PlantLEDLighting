/** 
 *  @file    HOASwitch.h
 *  @author  peter c
 *  @date    10/6/2017  
 *  @version 0.1
 *  
 *
 *  @section DESCRIPTION
 *  Hand-Off-Auto Switch

 */


#ifndef DA_HOASwitch_h
#define DA_HOASwitch_h
#include <HardwareSerial.h>
#include "DA_DiscreteInput.h"

class DA_HOASwitch
{
public:
enum HOADetectType  { Unknown, Hand, Off, Auto  };
DA_HOASwitch(  uint8_t aHandPin, uint8_t aOffPin, uint8_t aAutoPin  );




	void setOnStateChangeDetect( void (*callBack)( HOADetectType aState ));  

  // HOA 
  // Auto switch  Off Switch   STATE       ACTION
  // LOW       		High       Auto      Enable Timer
  // HIGH      		LOW        Hand      Disable Timer, Force Fan on
  // HIGH      		HIGH       Off       Disable Timer
  // otherwise do nothing		
	void refresh();
    HOADetectType getCurrentState();
	 void serialize( HardwareSerial *tracePort, bool includeCR); 
protected:

private:
	DA_DiscreteInput autoSwitch = NULL;
	DA_DiscreteInput handSwitch = NULL;
	DA_DiscreteInput offSwitch = NULL;
	HOADetectType state = Unknown;
  	void (*onStateChangeDetect)( HOADetectType aState ) = NULL;
  	bool invokeCallBack = false;

  	inline bool stateChanged( HOADetectType aPendingState )
    {
       return (state != aPendingState);
    };
    inline bool changeState( HOADetectType aToState )
    {
       state = aToState;
       invokeCallBack = true;
    };
};

#endif