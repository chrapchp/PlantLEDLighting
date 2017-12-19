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
#define MODBUS_REG_COUNT 50

#define CS_HS_001   0        // Drain Pump Hand status : Start/Stop
#define CS_SSH_101   1        // Smoke Detector
#define CS_HS_002   2        // Inlet H20 Open/Close
#define CS_LSHH_002   3        // nutrient mixture hi-hi level switch

#define HR_AT_101   10        // Ambient relative humidity
#define HR_TT_101   12        // Abient Temperature
#define HR_LT_002   14        // Water Level present value in cm from top,  0-> undefined
#define HR_FT_002   16        // Inlet Flow
#define HR_FT_003   18        // Nutrient Flow
#define HR_HS_001HOA   20        // Circulation Pump  HOA  (H=1, O=2, A=3)
#define HR_HS_102HOA   21        // Seeding Area  HOA  (H=1, O=2, A=3)
#define HR_HS_103HOA   22        // Growing Chamber  HOA  (H=1, O=2, A=3)
#define HR_HS_101HOA   23        // Fan HOA  (H=1, O=2, A=3)
#define HR_TT_001   24        // mixture temperature
#define HR_KY_002   25        // Version


#define HW_DY_102_ONT   30        // Growing Chamber LED ON Time (UNIX EPOCH)
#define HW_DY_102_OFT   32        // Growing Chamber LE30D OFF Time  (UNIX EPOCH)
#define HW_DY_103_ONT   34        // Seeding Area LED ON Time  (UNIX EPOCH)
#define HW_DY_103_OFT   36        // Seeding Area LED OFF Time  (UNIX EPOCH)
#define HW_QT_001   38        // Realt-time clock time (UNIX EPOCH)
#define HW_MY_101_ONP   40        // Fan  120VAC ON Period in sec
#define HW_MY_101_OFP   41        // Fan  120VAC OFF Period in sec
#define HW_PY_001_ONP   42        // Circulation Pump 120VAC ON Period in sec
#define HW_PY_001_OFP   43        // Circulation Pump 120VAC OFF Period in sec


#define CW_VY_001A   81        // inlet H20 valve, active low 12VDC
#define CW_PY_002   82        // Drain Pump 12VDC
#define CW_KY_001   83        // Reset config to defaults









#define MB_SLAVE_ID				1
#define MB_SERIAL_PORT			1
#define MB_SERIAL_BAUD			19200


union
{
  unsigned int regsf[2];
  float val;
} 
bfconvert;


union
{
  unsigned int regsl[2];
  long val;
} 
blconvert;

uint16_t modbusRegisters[MODBUS_REG_COUNT];

Modbus slave(MB_SLAVE_ID, MB_SERIAL_PORT); 


