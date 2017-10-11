/**
*  @file    DA_DiscreteOutput.cpp
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
#include <Streaming.h>
#include "DA_DiscreteOutput.h"
DA_DiscreteOutput::DA_DiscreteOutput(uint8_t aPin):DA_Output(discrete, aPin)
{
	reset();
}

DA_DiscreteOutput::DA_DiscreteOutput(uint8_t aPin, bool aActiveState):DA_Output(discrete, aPin)
{
	setActiveState(aActiveState);
	reset();
}

void DA_DiscreteOutput::write(bool aValue)
{
	bool value = aValue;

	if( isDisabled() )
	{
		value = !activeState;
	}

	digitalWrite(pin, value);
	currentState = value;
}


void DA_DiscreteOutput::forceActive()
{

	digitalWrite(pin, activeState);
	currentState = activeState;
}

void DA_DiscreteOutput::disable()
{
	DA_Output:_disable();
	reset();
}

bool DA_DiscreteOutput::isActiveLow()
{
	return(!activeState);
}

bool DA_DiscreteOutput::isActive()
{
	return(currentState == activeState);
}

bool DA_DiscreteOutput::setActiveState(bool aState)
{
	activeState = aState;
}

void DA_DiscreteOutput::reset()
{

	write(!activeState);
}

void DA_DiscreteOutput::activate()
{
	write(activeState);
}

void DA_DiscreteOutput::toggle() {
	write( !currentState);
}

void DA_DiscreteOutput::serialize(HardwareSerial * tracePort, bool includeCR)
{
	* tracePort << "{pin:" << pin << " isActiveLow:" << isActiveLow() << " currentState:" << currentState << " }";
	if (includeCR)
		* tracePort << endl;
}
