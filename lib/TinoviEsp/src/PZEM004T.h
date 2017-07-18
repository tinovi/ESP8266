#ifndef PZEM004T_H
#define PZEM004T_H

#include "Arduino.h"

//#include <SoftwareSerial.h>
#include <IPAddress.h>

struct PZEMCommand {
    uint8_t command;
    uint8_t addr[4];
    uint8_t data;
    uint8_t crc;
};

class PZEM004T
{
public:
    PZEM004T();
    //PZEM004T(uint8_t receivePin, uint8_t transmitPin);

    void setReadTimeout(unsigned long msec);
    unsigned long readTimeout() {return _readTimeOut;}

    float voltage(const IPAddress &addr);
    float current(const IPAddress &addr);
    float power(const IPAddress &addr);
    float energy(const IPAddress &addr);

    bool setAddress(const IPAddress &newAddr);
    bool setPowerAlarm(const IPAddress &addr, uint8_t threshold);
    void close();

private:
//    SoftwareSerial serial;
    unsigned long _readTimeOut;

    void send(const IPAddress &addr, uint8_t cmd, uint8_t data = 0);
    bool recieve(uint8_t resp, uint8_t *data = 0);
    uint8_t crc(uint8_t *data, uint8_t sz);
};

#endif // PZEM004T_H
