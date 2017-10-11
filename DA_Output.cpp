#include <Streaming.h>
#include "DA_Output.h"


DA_Output::DA_Output( IO_TYPE aOutputType, uint8_t aPin )
{
	outputType = aOutputType;
	pin = aPin;
	pinMode(pin, OUTPUT);
}


void DA_Output::_disable() 
{
	disabled = true;
}
void DA_Output::enable() 
{
	disabled = false;
}

bool DA_Output::isDisabled()
{
	return( disabled );
}