#define SMALL_ENDIAN

// #include "MasterSerialProtocol.hpp"
#include "ControllerSerialProtocol.hpp"
#include "SerialCommunication.hpp"
#include <math.h>

#define LED_BUILTIN 2 //Remote
#define JOYADCY_PIN 15 //Remote
#define JOYADCX_PIN 25 //Remote

#define TX_PIN 21
#define RX_PIN 22

#define CMAP(v,o)  ((v < 0)? map(v, -o, 0, -2047, 0) : map(v, 0, 4095-o, 0, 2047)) 

int joy_adc_x_origin = 1840;
int joy_adc_y_origin = 1850;

// Adafruit_ST7789 tft = Adafruit_ST7789(TFT_CS, TFT_DC, TFT_MOSI, TFT_SCLK);
HardwareSerial MasterSerial(2);
SerialInterface master(&MasterSerial, M2C_PACKET_SIZE, C2M_PACKET_SIZE, START_BIT, END_BIT);

//  Controller
typedef struct
{
  int joy_x;
  int joy_y;

} Controller;

Controller ctl;

struct RobotState
{ 
  bool v_estop;
  control_mode_t control_mode;
  float speed_target;
  float speed_current;
  float angle_target;
  float angle_current;
  float angle_speed_target;
  float angular_speed_current;
  float vacuum_voltage;
  bool foc_engaged;
};

struct RobotState rs{ false, NULL_CONTROL, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, false };

void vomitTxBuffer()
{
      for(int i=0;i<master.getTxPacketSize();i++)
    {
      Serial.print( master.getTxBufferPtr()[i] , HEX);
      Serial.print(" ");
      vTaskDelay(10);
    }
    Serial.print("| Size: ");
    Serial.println( master.getTxCounter() );
    vTaskDelay(10);
}

void calibJoystick(int &x, int &y, int v=50)
{
  for(int i=0;i<v;i++) 
  {
    x += analogRead(JOYADCX_PIN);
    Serial.printf( "Calibrating x: %d\n",i );
    vTaskDelay(50 / portTICK_PERIOD_MS);
  }
  for(int i=0;i<v;i++) 
  {
    y += analogRead(JOYADCY_PIN);
    Serial.printf( "Calibrating y: %d\n",i );
    vTaskDelay(50 / portTICK_PERIOD_MS);
  }
  x = x/v;
  y = y/v;
}
/*////////////////////////////////////////////////////////////////

                              TASK INIT

////////////////////////////////////////////////////////////////*/
uint32_t xBlinking_stack = 1024;
TaskHandle_t xBlinking_handle = NULL;
void xBlinking( void* pv );

uint32_t xGetJoystickReading_stack = 2048;
TaskHandle_t xGetJoystickReading_handle = NULL;
void xGetJoystickReading( void* pv );

uint32_t xSendCommand_stack = 10000;
TaskHandle_t xSendCommand_handle = NULL;
void xSendCommand( void* pv );

/*////////////////////////////////////////////////////////////////

                              TASK DEFINE

////////////////////////////////////////////////////////////////*/

void xBlinking( void* pv )
{

  // Serial.println("INIT\t||\tBlinking");

  pinMode(LED_BUILTIN,OUTPUT);
  digitalWrite(LED_BUILTIN,LOW);

  // Serial.println("DONE\t||\tBlinking");
  for( ; ; )
  {
    // Serial.println("DEBUG\t||\tBlinking");
    digitalWrite(LED_BUILTIN,LOW);
    vTaskDelay(500 / portTICK_PERIOD_MS);
    digitalWrite(LED_BUILTIN,HIGH);
    vTaskDelay(500 / portTICK_PERIOD_MS);
  }

}

void xGetJoystickReading( void* pv )
{

  for( ; ; )
  { 

    //map to -2047 to 2047
    ctl.joy_x = CMAP(analogRead(JOYADCX_PIN) - joy_adc_x_origin, joy_adc_x_origin);
    ctl.joy_y = CMAP(analogRead(JOYADCY_PIN) - joy_adc_y_origin, joy_adc_y_origin);

    rs.control_mode = MANUAL_CONTROL;
    rs.speed_target = constrain(ctl.joy_x+ctl.joy_y,-2047,2047)/2047.0f*12; //virtual override (L wheel)
    rs.angle_target = constrain(ctl.joy_y-ctl.joy_x,-2047,2047)/2047.0f*12; //virtual override (R wheel)

    // Serial.printf("%d\t%d\t%f\t%f\n",ctl.joy_x,ctl.joy_y,rs.speed_target,rs.angle_target);
    vTaskDelay(100 / portTICK_PERIOD_MS);
  }
}

void xSendCommand( void* pv )
{
  uint8_t* packet = master.getTxBufferPtr();

  for( ; ; )
  {

    master.txPush(START_BIT);          // 00  |   StartBit
    master.txPush(V_ESTOP_DIS_CODE);   // 01  |   VirtualEStop
    master.txPush(MANUAL_CONTROL);     // 02  |   ControlMode

    master.txPush(CTR_EN_CODE);        // 03  |   ControllerMode


    // Serial.printf("%x ",  EXTRACT_BYTE_FROM_4BYTE_VALUE(rs.speed_target,0));
    // Serial.printf("%x ",  EXTRACT_BYTE_FROM_4BYTE_VALUE(rs.speed_target,1));
    // Serial.printf("%x ",  EXTRACT_BYTE_FROM_4BYTE_VALUE(rs.speed_target,2));
    // Serial.printf("%x \n",EXTRACT_BYTE_FROM_4BYTE_VALUE(rs.speed_target,3));

    master.txPush( EXTRACT_BYTE_FROM_4BYTE_VALUE(rs.speed_target,0) ); // 04  |   LeftWheelVoltage   (1st Byte)
    master.txPush( EXTRACT_BYTE_FROM_4BYTE_VALUE(rs.speed_target,1) );
    master.txPush( EXTRACT_BYTE_FROM_4BYTE_VALUE(rs.speed_target,2) );
    master.txPush( EXTRACT_BYTE_FROM_4BYTE_VALUE(rs.speed_target,3) );

    master.txPush( EXTRACT_BYTE_FROM_4BYTE_VALUE(rs.angle_target,0) );   // 08  |   RightWheelVoltage   (1st Byte)
    master.txPush( EXTRACT_BYTE_FROM_4BYTE_VALUE(rs.angle_target,1) );
    master.txPush( EXTRACT_BYTE_FROM_4BYTE_VALUE(rs.angle_target,2) );
    master.txPush( EXTRACT_BYTE_FROM_4BYTE_VALUE(rs.angle_target,3) );
    
    master.txPush(master.getCheckSum(master.getTxBufferPtr(),master.getTxPacketSize())); // 13  |   EndBit
    master.txPush(END_BIT); // 12  |   CheckSum

    // Debug
    // vomitTxBuffer();

    master.txSend();
    master.txClear(false);

    vTaskDelay(100 / portTICK_PERIOD_MS);
  }
}



/*////////////////////////////////////////////////////////////////

                              MAIN PROGRAME

////////////////////////////////////////////////////////////////*/

void setup() 
{
  // MasterSerial.begin(115200, SERIAL_8N1, 3, 1); //ESP32S Serial0
  MasterSerial.begin(115200, SERIAL_8N1, RX_PIN, TX_PIN); //ESP32S Custom
  Serial.begin(115200);


  calibJoystick(joy_adc_x_origin, joy_adc_y_origin);

  
  xTaskCreatePinnedToCore(  xBlinking,
                            "Blinking",
                            xBlinking_stack,
                            NULL,
                            1,
                            &xBlinking_handle,
                            1 );

  xTaskCreatePinnedToCore(  xGetJoystickReading,
                            "Get Joystick Reading",
                            xGetJoystickReading_stack,
                            NULL,
                            1,
                            &xGetJoystickReading_handle,
                            1 );

  xTaskCreatePinnedToCore(  xSendCommand,
                            "Send Command",
                            xSendCommand_stack,
                            NULL,
                            1,
                            &xSendCommand_handle,
                            1 );
                            

  vTaskDelay(50 / portTICK_PERIOD_MS);
}

void loop() {}
