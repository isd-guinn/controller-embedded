#ifndef SERIALCOMMUNICATION_HPP
#define SERIALCOMMUNICATION_HPP

#include <Arduino.h>
#include <HardwareSerial.h>


class SerialInterface
{
  private:
    HardwareSerial* _serial_ptr;
    uint8_t _start_bit;
    uint8_t _end_bit;
    bool _is_packet;

    uint8_t _rx_packet_size;
    uint8_t* _rx_buffer_ptr;
    uint8_t _rx_counter;

    uint8_t _tx_packet_size;
    uint8_t* _tx_buffer_ptr;
    uint8_t _tx_counter;

  public:
    SerialInterface( HardwareSerial* serial_ptr, uint8_t rx_packet_size, uint8_t tx_packet_size, uint8_t startbit, uint8_t endbit);
    ~SerialInterface();

    uint8_t getStartBit() const;
    uint8_t getEndBit() const;
    uint8_t getRxPacketSize() const;
    uint8_t getTxPacketSize() const;
    uint8_t* getRxBufferPtr() const;
    uint8_t getRxCounter() const;
    uint8_t getTxCounter() const;
    uint8_t* getTxBufferPtr() const;

    uint8_t rxPush();
    void rxClear( bool keepStartBit );

    bool txPush( uint8_t inByte );
    void txSend();
    void txClear( bool keepStartBit );

    uint8_t getCheckSum( uint8_t* data, uint8_t size );
    bool checkCheckSum( uint8_t* data, uint8_t size );

    bool onRecievedCommand();


    void set( HardwareSerial* serial_ptr,  uint8_t rx_packet_size, uint8_t tx_packet_size, uint8_t startbit, uint8_t endbit, uint8_t* rx_buffer_ptr, uint8_t* tx_buffer_ptr );

};


#endif