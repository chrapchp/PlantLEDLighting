#include <Streaming.h>
#include "DA_Input.h"

DA_Input::DA_Input( INPUT_TYPE aInputType, uint8_t aPin )
{
	inputType = aInputType;
	pin = aPin;
}



void DA_Input::setPollingInterval(unsigned int aPollingInterval  )
{
	pollingInterval = aPollingInterval ;
}

void DA_Input::refresh()
{
	unsigned long currentTime = millis();
	if ( currentTime - lastUpdateTime > pollingInterval )
	{
		lastUpdateTime = currentTime;
		onRefresh();
	}
}

