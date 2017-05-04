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


#define COIL_STATUS_READ_OFFSET 0
#define COIL_STATUS_WRITE_OFFSET 5
#define HOLDING_REGISTER_READ_OFFSET 10
#define HOLDING_REGISTER_WRITE_OFFSET 30
#define MODBUS_REG_COUNT HOLDING_REGISTER_WRITE_OFFSET + 5


#define HR_AMBIENT_TEMPERATURE HOLDING_REGISTER_READ_OFFSET
#define HR_SOIL_MOISTURE	   HR_AMBIENT_TEMPERATURE + 1
#define HR_LED_DUTY_CYCLE	   HR_SOIL_MOISTURE + 1
#define HR_LED_DUTY_CYCLE_PERIOD HR_LED_DUTY_CYCLE + 1
#define HR_CURRENT_TIME			HR_LED_DUTY_CYCLE_PERIOD + 1   // UTC
#define HR_LED_OFF_TIME			HR_CURRENT_TIME + 2
#define HR_LED_ON_TIME			HR_LED_OFF_TIME + 2

#define HR_SET_TIME				HOLDING_REGISTER_WRITE_OFFSET  // UTC
#define HR_SET_LED_ON_TIME		HR_SET_TIME + 2
#define HR_SET_LED_OFF_TIME		HR_SET_LED_ON_TIME + 2
#define HR_SET_DUTY_CYCLE		HR_SET_LED_OFF_TIME + 2
#define HR_SET_DUTY_CYCLE_PERIOD HR_SET_DUTY_CYCLE + 1

#define CS_SET_LED_ON			0
#define CS_SET_LED_OFF			1


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


