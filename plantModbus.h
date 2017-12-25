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
#define MODBUS_REG_COUNT 60

#define CS_HS_001   0        // Drain Pump Hand status : Start/Stop
#define CS_SSH_101   1        // Smoke Detector
#define CS_HS_002   2        // Inlet H20 Open/Close
#define CS_LSHH_002   3        // nutrient mixture hi-hi level switch

#define CW_VY_001A   81        // inlet H20 valve, active low 12VDC
#define CW_PY_002   82        // Drain Pump 12VDC
#define CW_KY_001   83        // Reset config to defaults

#define HR_AT_101   10        // Ambient relative humidity
#define HR_TT_101   12        // Abient Temperature
#define HR_LT_002   14        // Water Level present value in cm from top,  0-> undefined
#define HR_FT_002   16        // Inlet Flow
#define HR_FT_003   18        // Nutrient Flow
#define HR_TT_001   20        // mixture temperature

#define HR_HS_001HOA   22        // Circulation Pump  HOA  (H=1, O=2, A=3)
#define HR_HS_102HOA   23        // Seeding Area  HOA  (H=1, O=2, A=3)
#define HR_HS_103HOA   24        // Growing Chamber  HOA  (H=1, O=2, A=3)
#define HR_HS_101HOA   25        // Fan HOA  (H=1, O=2, A=3)
#define HR_KY_002   26        // Version

#define HW_DY_102_ONT_SP   30        // Seeding Area Chamber LED ON Time (UNIX EPOCH) Setpoint
#define HW_DY_102_OFT_SP   32        // Seeding Area LED OFF Time  (UNIX EPOCH)  Setpoint
#define HW_DY_103_ONT_SP   34        // Growing Chamber LED ON Time (UNIX EPOCH)  Setpoint
#define HW_DY_103_OFT_SP   36        // Growing Chamber LED OFF Time  (UNIX EPOCH)  Setpoint
#define HW_QT_001   	   38        // Realt-time clock time (UNIX EPOCH)  Setpoint
#define HW_MY_101_ONP_SP   40        // Fan  120VAC ON Period in sec  Setpoint
#define HW_MY_101_OFP_SP   41        // Fan  120VAC OFF Period in sec  Setpoint
#define HW_PY_001_ONP_SP   42        // Circulation Pump 120VAC ON Period in sec  Setpoint
#define HW_PY_001_OFP_SP   43        // Circulation Pump 120VAC OFF Period in sec  Setpoint


#define HW_DY_102_ONP_CV   44        // Seeding Area Chamber LED ON Time (UNIX EPOCH) Current Value
#define HW_DY_102_OFP_CV   46        // Seeding Area LED OFF Time  (UNIX EPOCH)  Current Value
#define HW_DY_103_ONP_CV   48        // Growing Chamber LED ON Time (UNIX EPOCH)  Current Value
#define HW_DY_103_OFP_CV   50        // Growing Chamber LED OFF Time  (UNIX EPOCH)  Current Value
#define HW_MY_101_ONP_CV   52        // Fan  120VAC ON Period in sec  Current Value
#define HW_MY_101_OFP_CV   53        // Fan  120VAC OFF Period in sec  Current Value
#define HW_PY_001_ONP_CV   54        // Circulation Pump 120VAC ON Period in sec  Current Value
#define HW_PY_001_OFP_CV   55        // Circulation Pump 120VAC OFF Period in sec  Current Value



#define MB_SLAVE_ID				1
#define MB_SERIAL_PORT			0
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


