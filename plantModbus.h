/**
 * @file 	plantModbus.h
 * @version     0.1
 * @date        2017May3
 * @author 	pjc

 *
 * @description
 *  Helpers for plant lighting and control using Modbus
 *
 * Using arduino modbus implementation @
 * https://github.com/smarmengol/Modbus-Master-Slave-for-Arduino
*/


#include <SoftwareSerial.h>
#include <ModbusRtu.h>

 
#define COIL_STATUS_READ_WRITE_OFFSET 0
#define COIL_STATUS_WRITE_START_BIT	(16*5)  // 80 coils/bits for reads then followed by write memory for coils

#define HOLDING_REGISTER_READ_OFFSET 10		// start read holding regisers
#define HOLDING_REGISTER_WRITE_OFFSET 30




#define HR_AMBIENT_TEMPERATURE HOLDING_REGISTER_READ_OFFSET
#define HR_SOIL_MOISTURE	   HR_AMBIENT_TEMPERATURE + 1
#define HR_LED_DUTY_CYCLE	   HR_SOIL_MOISTURE + 1
#define HR_LED_DUTY_CYCLE_PERIOD HR_LED_DUTY_CYCLE + 1
#define HR_CURRENT_TIME			HR_LED_DUTY_CYCLE_PERIOD + 1   // UTC
#define HR_LED_ON_TIME			HR_CURRENT_TIME + 2
#define HR_LED_OFF_TIME			HR_LED_ON_TIME + 2

#define HR_SET_TIME				HOLDING_REGISTER_WRITE_OFFSET  // UTC
#define HR_SET_LED_ON_TIME		HR_SET_TIME + 2
#define HR_SET_LED_OFF_TIME		HR_SET_LED_ON_TIME + 2
#define HR_SET_DUTY_CYCLE		HR_SET_LED_OFF_TIME + 2
#define HR_SET_DUTY_CYCLE_PERIOD HR_SET_DUTY_CYCLE + 1

#define MODBUS_REG_COUNT HOLDING_REGISTER_WRITE_OFFSET + HR_SET_DUTY_CYCLE_PERIOD + 1

// coil write commands
#define CS_SET_LED_ON			 COIL_STATUS_WRITE_START_BIT //bit position
#define CS_SET_LED_OFF			 CS_SET_LED_ON + 1
#define CS_RESET_TO_DEFAULTS	 CS_SET_LED_OFF + 1

// coil statuses
#define CS_LED_STATUS			COIL_STATUS_READ_WRITE_OFFSET


#define MB_SLAVE_ID				1
#define MB_SERIAL_PORT			0



union
{
  int regsf[2];
  float val;
} 
bfconvert;


union
{
  int regsl[2];
  long val;
} 
blconvert;

uint16_t modbusRegisters[MODBUS_REG_COUNT];

Modbus slave(MB_SLAVE_ID, MB_SERIAL_PORT); 


