#ifndef CONTROLLERSERIALPROTOCOL_HPP 
#define CONTROLLERSERIALPROTOCOL_HPP

#ifdef BIG_ENDIAN
#define EXTRACT_BYTE_FROM_4BYTE_VALUE(x,y) (*((uint8_t*)(&x)+y))
#endif

#ifdef SMALL_ENDIAN
#define EXTRACT_BYTE_FROM_4BYTE_VALUE(x,y) (*((uint8_t*)(&x)+(3-y)))
#endif

namespace ByteUtil
{
  inline float reconFloat(uint8_t *packet, uint8_t pos, bool isSmallEndian = true)
  {
    if (isSmallEndian)
    {
      uint8_t ctn[4] = {packet[pos+3], packet[pos+2], packet[pos+1], packet[pos]};
      return *(float*)&ctn;
    }
    else
    {
      uint8_t ctn[4] = {packet[pos], packet[pos+1], packet[pos+2], packet[pos+3]};
      return *(float*)&ctn;
    }              
  }
}

#include <stdint.h>



/*
  Packet from Controller:
  
  00  |   StartBit
  01  |   VirtualEStop
  02  |   ControlMode

  03  |   ControllerMode

  04  |   LeftWheelVoltage   (1st Byte)
  05  |   LeftWheelVoltage   (2nd Byte)
  06  |   LeftWheelVoltage   (3rd Byte)
  07  |   LeftWheelVoltage   (4th Byte)

  08  |   RightWheelVoltage   (1st Byte)
  09  |   RightWheelVoltage   (2nd Byte)
  10  |   RightWheelVoltage   (3rd Byte)
  11  |   RightWheelVoltage   (4th Byte)
  
  12  |   CheckSum

  13  |   EndBit
*/

/*
  Packet from Master:
  
  00  |   StartBit
  01  |   DebugCode

  02  |   LeftFOCAngle
  03  |   RightFOCAngle
  
  04  |   CheckSum

  05  |   EndBit
*/


#define C2M_PACKET_SIZE   14
#define M2C_PACKET_SIZE   6

// #define M2S_PACKET_SIZE   33
// #define S2M_PACKET_SIZE   5


/*        Byte Position Macros        */
#define BYTE_POS_C2M_STARTBIT        0
#define BYTE_POS_C2M_VESTOP          1
#define BYTE_POS_C2M_CONTROLMODE     2
#define BYTE_POS_C2M_CTRMODE         3
#define BYTE_POS_C2M_LWHEELVOLTAGE   4
#define BYTE_POS_C2M_RWHEELVOLTAGE   8
#define BYTE_POS_C2M_CHECKSUM       12
#define BYTE_POS_C2M_ENDBIT         13

#define BYTE_POS_M2C_STARTBIT        0
#define BYTE_POS_M2C_DEBUGCODE       1
#define BYTE_POS_M2C_LEFTFOCANGLE    2
#define BYTE_POS_M2C_RIGHTFOCANGLE   3
#define BYTE_POS_M2C_CHECKSUM        4
#define BYTE_POS_M2C_ENDBIT          5


#define START_BIT         0x3E
#define END_BIT           0x3F

#define V_ESTOP_EN_CODE   0xA1
#define V_ESTOP_DIS_CODE  0xA2

typedef uint8_t control_mode_t;
#define NULL_CONTROL      0x00
#define SPEED_CONTROL     0x01
#define ANGLE_CONTROL     0x02
#define MANUAL_CONTROL    0x03

#define CTR_EN_CODE       0x01
#define CTR_DIS_CODE      0x00

#define FOC_EN_CODE       0xB1
#define FOC_DIS_CODE      0xB2

typedef uint8_t debug_code_t;
#define DEBUG_NORMAL      0x00

#endif


