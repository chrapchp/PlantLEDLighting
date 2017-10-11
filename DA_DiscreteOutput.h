/**
*  @file    DA_DiscreteOutput.h
*  @author  peter c
*  @date    25/09/2017
*  @version 0.1
*
*
*  @section DESCRIPTION
*  Simple class to represent an arduinio discrete output
*
*
*/

#ifndef DA_DISCRETE_OUTPUT_H
#define DA_DISCRETE_OUTPUT_H
#include "DA_Output.h"
class DA_DiscreteOutput:public DA_Output
{
	public:
		DA_DiscreteOutput(uint8_t aPin);
		DA_DiscreteOutput(uint8_t aPin, bool aActiveState);
		bool isActiveLow();
		bool isActive();
		// state for active mode e.g. on is low for fail safe scenarios.
		// Default: active low (LOW)
		bool setActiveState(bool aState);
		// set to default active state
		void reset();
		// if disabled value written will be the non active state. e.g is active low, high will be written
		void write(bool aValue); 
		void disable();
		void forceActive();  // force write regardless of disabled status
		void activate();
		void toggle(); 		
		void serialize( HardwareSerial *tracePort, bool includeCR);		
	protected:
		bool activeState = LOW;
	private:

		bool currentState = HIGH;
};

#endif
