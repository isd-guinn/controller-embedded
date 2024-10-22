#ifndef MASTERSERIALPROTOCOL_HPP 
#define MASTERSERIALPROTOCOL_HPP

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
  Packet from Master:
  
  00  |   StartBit
  01  |   VirtualEStop
  02  |   ControlMode

  03  |   TargetSpeed   (1st Byte)
  04  |   TargetSpeed   (2nd Byte)
  05  |   TargetSpeed   (3rd Byte)
  06  |   TargetSpeed   (4th Byte)

  07  |   CurrentSpeed  (1st Byte)
  08  |   CurrentSpeed  (2nd Byte)
  09  |   CurrentSpeed  (3rd Byte)
  10  |   CurrentSpeed  (4th Byte)

  11  |   TargetAngle   (1st Byte)
  12  |   TargetAngle   (2nd Byte)
  13  |   TargetAngle   (3rd Byte)
  14  |   TargetAngle   (4th Byte)

  15  |   CurrentAngle  (1st Byte)
  16  |   CurrentAngle  (2nd Byte)
  17  |   CurrentAngle  (3rd Byte)
  18  |   CurrentAngle  (4th Byte)

  19  |   TarAngSpeed   (1st Byte)
  20  |   TarAngSpeed   (2nd Byte)
  21  |   TarAngSpeed   (3rd Byte)
  22  |   TarAngSpeed   (4th Byte)

  23  |   CurAngSpeed  (1st Byte)
  24  |   CurAngSpeed  (2nd Byte)
  25  |   CurAngSpeed  (3rd Byte)
  26  |   CurAngSpeed  (4th Byte)

  27  |   VacuumVoltage (1st Byte)
  28  |   VacuumVoltage (2nd Byte)
  29  |   VacuumVoltage (3rd Byte)
  30  |   VacuumVoltage (4th Byte)

  31  |   FOCMode

  32  |   CheckSum

  33  |   EndBit
*/

/*
  Packet from Slave:
  
  00  |   StartBit
  01  |   DebugCode

  02  |   LeftFOCAngle
  03  |   RightFOCAngle
  
  04  |   CheckSum

  05  |   EndBit
*/


#define M2S_PACKET_SIZE   34
#define S2M_PACKET_SIZE   6

// #define M2S_PACKET_SIZE   33
// #define S2M_PACKET_SIZE   5


/*        Byte Position Macros        */
#define BYTE_POS_M2S_STARTBIT        0
#define BYTE_POS_M2S_VESTOP          1
#define BYTE_POS_M2S_CONTROLMODE     2
#define BYTE_POS_M2S_TARGETSPEED     3
#define BYTE_POS_M2S_CURRENTSPEED    7
#define BYTE_POS_M2S_TARGETANGLE    11
#define BYTE_POS_M2S_CURRENTANGLE   15
#define BYTE_POS_M2S_TARANGSPEED    19
#define BYTE_POS_M2S_CURANGSPEED    23
#define BYTE_POS_M2S_VACUUMVOLTAGE  27
#define BYTE_POS_M2S_FOCMODE        31
#define BYTE_POS_M2S_CHECKSUM       32
#define BYTE_POS_M2S_ENDBIT         33

#define BYTE_POS_S2M_STARTBIT        0
#define BYTE_POS_S2M_DEBUGCODE       1
#define BYTE_POS_S2M_LEFTFOCANGLE    2
#define BYTE_POS_S2M_RIGHTFOCANGLE   3
#define BYTE_POS_S2M_CHECKSUM        4
#define BYTE_POS_S2M_ENDBIT          5


#define START_BIT         0x3E
#define END_BIT           0x3F

#define V_ESTOP_EN_CODE   0xA1
#define V_ESTOP_DIS_CODE  0xA2

typedef uint8_t control_mode_t;
#define NULL_CONTROL      0x00
#define SPEED_CONTROL     0x01
#define ANGLE_CONTROL     0x02
#define MANUAL_CONTROL    0x03

#define FOC_EN_CODE       0xB1
#define FOC_DIS_CODE      0xB2

typedef uint8_t debug_code_t;
#define DEBUG_NORMAL      0x00

#endif


