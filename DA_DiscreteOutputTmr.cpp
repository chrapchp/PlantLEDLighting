/**
*  @file    DA_DiscreteOutputTmr.h
*  @author  peter c
*  @date    4/10/2017
*  @version 0.1
*
*
*  @section DESCRIPTION
*  Simple class to represent an arduinio discrete output with on/off duration
*
*
*/
#include <Streaming.h>
#include "DA_DiscreteOutputTmr.h"
DA_DiscreteOutputTmr::DA_DiscreteOutputTmr(uint8_t aPin, bool aActiveState, unsigned int aOnDurationInSec, unsigned int aOffDurationInSec):DA_DiscreteOutput(aPin, aActiveState)
{
	onDurationInMilliSec = (long) aOnDurationInSec * 1000;
	offDurationInMilliSec = (long) aOffDurationInSec * 1000;
}

bool DA_DiscreteOutputTmr::setOnDuration(unsigned int aOnDurationInSec)
{
	bool retVal = false;
	if (aOnDurationInSec > 0)
		onDurationInMilliSec = aOnDurationInSec * 1000;
	return retVal;
}

bool DA_DiscreteOutputTmr::setOffDuration(unsigned int aOffDurationInSec)
{
	bool retVal = false;
	if (aOffDurationInSec > 0)
		offDurationInMilliSec = aOffDurationInSec * 1000;
	return retVal;
}

void DA_DiscreteOutputTmr::restart()
{
	DA_DiscreteOutput::reset();
	firstRun = true;
}

void DA_DiscreteOutputTmr::serialize(HardwareSerial *tracePort, bool includeCR)
{
	DA_DiscreteOutput::serialize(tracePort, false);
	*tracePort << "{onDurationInMilliSec:" << onDurationInMilliSec << " offDurationInMilliSec:" << offDurationInMilliSec << " startMode:" << startMode << " }";
	if (includeCR)
		*tracePort << endl;
}

void DA_DiscreteOutputTmr::setStartDefault(bool aStartMode)
{
	startMode = aStartMode;
}

bool DA_DiscreteOutputTmr::refresh()
{
	if( isDisabled() )
		return true;

	if (onDurationInMilliSec < 1 || offDurationInMilliSec < 1)
		return false;

	unsigned long currentEpoch = millis();
	if (firstRun)
	{
		// force a start with active state
		if (startMode)
		{
			stateInterval = onDurationInMilliSec;
			DA_DiscreteOutput::activate();
		}
		else
		{
			stateInterval = offDurationInMilliSec;
			DA_DiscreteOutput::reset();
		}
		firstRun = false;
	}


	if ((unsigned long) (currentEpoch - previousEpoch) >= stateInterval)
	{
		if (DA_DiscreteOutput::isActive())
		{
			stateInterval = offDurationInMilliSec;
		}
		else
		{
			stateInterval = onDurationInMilliSec;
		}
		DA_DiscreteOutput::toggle();
		previousEpoch = currentEpoch;
	}
	return(true);
}

