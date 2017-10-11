/**
*  @file    DA_HOASwitch.h
*  @author  peter c
*  @date    10/6/2017
*  @version 0.1
*
*
*  @section DESCRIPTION
*  Hand-Off-Auto Switch
*/
#include <Streaming.h>
#include "DA_HOASwitch.h"
DA_HOASwitch::DA_HOASwitch(uint8_t aHandPin, uint8_t aOffPin, uint8_t aAutoPin)
{

	autoSwitch = DA_DiscreteInput(aAutoPin, DA_DiscreteInput::ToggleDetect, true);
	autoSwitch.setPollingInterval( 250 );

	handSwitch = DA_DiscreteInput(aHandPin, DA_DiscreteInput::ToggleDetect, true);
	handSwitch.setPollingInterval( 250 );
	// autoPin = aAutoPin ;
}


void DA_HOASwitch::setOnStateChangeDetect( void (*callBack)( HOADetectType aState ))
{
	onStateChangeDetect = callBack;
} 

  // HOA 
  // Auto switch  Off Switch   STATE      
  // LOW       		High       Auto      
  // HIGH      		LOW        Hand      
  // HIGH      		HIGH       Off       
  // otherwise do nothing
void DA_HOASwitch::refresh()
{
	handSwitch.refresh();
	autoSwitch.refresh();
	bool handSwitchSample = handSwitch.getRawSample();
	bool autoSwitchSample = autoSwitch.getRawSample();

	invokeCallBack = false;

	if( !autoSwitchSample && handSwitchSample  ) 
  	{
  		if( stateChanged( Auto  ))
  		{
  			changeState( Auto );
  		}

  	}
  	else if( autoSwitchSample && !handSwitchSample  ) {
  		if( stateChanged( Hand  ))
  		{
  			changeState( Hand );
  		}
  	}
  	else if( autoSwitchSample && handSwitchSample  ) {
  		if( stateChanged( Off  ))
  		{
  			changeState( Off );
  		}
  	}

  	if( invokeCallBack && onStateChangeDetect != NULL)
		onStateChangeDetect( state );

}

DA_HOASwitch::HOADetectType DA_HOASwitch::getCurrentState()
{
	return( state );
}

 void DA_HOASwitch::serialize( HardwareSerial *tracePort, bool includeCR)
 {
 	*tracePort << "HOA:hand, auto status" << endl;
 	handSwitch.serialize( tracePort, true);
	autoSwitch.serialize( tracePort, true); 
	*tracePort << "{state:" << state << "}" << endl;	
 }
  