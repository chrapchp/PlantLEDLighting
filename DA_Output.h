/** 
 *  @file    DA_Output.h
 *  @author  peter c
 *  @date    09/25/2017  
 *  @version 0.1
 *  
 *
 *  @section DESCRIPTION
 *  Simple base class to Discrete and Analog Outputs

 */


#ifndef DA_OUTPUT_H
#define DA_OUTPUT_H
#include <HardwareSerial.h>



#include "DA_IO.h"

class DA_Output
{

public:
  	DA_Output( IO_TYPE aOutputType, uint8_t aPin );
  	void _disable();
 	void enable();
 	bool isDisabled(); 	
   	virtual void serialize( HardwareSerial *tracePort, bool includeCR ) {};
protected:
 	int  pin;	
 	virtual void write( bool aValue ) = 0;


 	IO_TYPE outputType;




  //  virtual void doAlarmCheck() = 0;

  unsigned int rawValue;
  //T scaledValue;

private:
	bool disabled = false;




};




#endif
