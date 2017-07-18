#include "../../TinoviEsp/src/PZEM004T.h"

#define PZEM_VOLTAGE (uint8_t)0xB0
#define RESP_VOLTAGE (uint8_t)0xA0

#define PZEM_CURRENT (uint8_t)0xB1
#define RESP_CURRENT (uint8_t)0xA1

#define PZEM_POWER   (uint8_t)0xB2
#define RESP_POWER   (uint8_t)0xA2

#define PZEM_ENERGY  (uint8_t)0xB3
#define RESP_ENERGY  (uint8_t)0xA3

#define PZEM_SET_ADDRESS (uint8_t)0xB4
#define RESP_SET_ADDRESS (uint8_t)0xA4

#define PZEM_POWER_ALARM (uint8_t)0xB5
#define RESP_POWER_ALARM (uint8_t)0xA5

#define RESPONSE_SIZE sizeof(PZEMCommand)
#define RESPONSE_DATA_SIZE RESPONSE_SIZE - 2

#define DEFAULT_READ_TIMEOUT 1000

//
//PZEM004T::PZEM004T(uint8_t receivePin, uint8_t transmitPin)
//    : Serial(receivePin, transmitPin,false, 256)
//    , _readTimeOut(DEFAULT_READ_TIMEOUT)
//{
//    Serial.begin(9600);
//}

PZEM004T::PZEM004T()
    :_readTimeOut(DEFAULT_READ_TIMEOUT)
{
    Serial.begin(9600);
}

void PZEM004T::close(){
	Serial.end();
}

void PZEM004T::setReadTimeout(unsigned long msec)
{
    _readTimeOut = msec;
}

float PZEM004T::voltage(const IPAddress &addr)
{
    uint8_t data[RESPONSE_DATA_SIZE];

    send(addr, PZEM_VOLTAGE);
    if(!recieve(RESP_VOLTAGE, data))
        return -1.0;

    return (data[0] << 8) + data[1] + (data[2] / 10.0);
}

float PZEM004T::current(const IPAddress &addr)
{
    uint8_t data[RESPONSE_DATA_SIZE];

    send(addr, PZEM_CURRENT);
    if(!recieve(RESP_CURRENT, data))
        return -1.0;

    return (data[0] << 8) + data[1] + (data[2] / 100.0);
}

float PZEM004T::power(const IPAddress &addr)
{
    uint8_t data[RESPONSE_DATA_SIZE];

    send(addr, PZEM_POWER);
    if(!recieve(RESP_POWER, data))
        return -1.0;

    return (data[0] << 8) + data[1];
}

float PZEM004T::energy(const IPAddress &addr)
{
    uint8_t data[RESPONSE_DATA_SIZE];

    send(addr, PZEM_ENERGY);
    if(!recieve(RESP_ENERGY, data))
        return -1.0;

    return (data[0] << 16) + (data[1] << 8) + data[2];
}

bool PZEM004T::setAddress(const IPAddress &newAddr)
{
    send(newAddr, PZEM_SET_ADDRESS);
    return recieve(RESP_SET_ADDRESS);
}

bool PZEM004T::setPowerAlarm(const IPAddress &addr, uint8_t threshold)
{
    send(addr, PZEM_POWER_ALARM, threshold);
    return recieve(RESP_POWER_ALARM);
}

void PZEM004T::send(const IPAddress &addr, uint8_t cmd, uint8_t data)
{
    PZEMCommand pzem;

    pzem.command = cmd;
    for(int i=0; i<sizeof(pzem.addr); i++)
        pzem.addr[i] = addr[i];
    pzem.data = data;

    uint8_t *bytes = (uint8_t*)&pzem;
    pzem.crc = crc(bytes, sizeof(pzem) - 1);

    while(Serial.available())
        Serial.read();

    Serial.write(bytes, sizeof(pzem));
}

bool PZEM004T::recieve(uint8_t resp, uint8_t *data)
{
    uint8_t buffer[RESPONSE_SIZE];

    unsigned long startTime = millis();
    uint8_t len = 0;
    while((len < RESPONSE_SIZE) && (millis() - startTime < _readTimeOut))
    {
        if(Serial.available() > 0)
        {
            uint8_t c = (uint8_t)Serial.read();
            if(!c && !len)
                continue; // skip 0 at startup
            buffer[len++] = c;
        }
    }

    if(len != RESPONSE_SIZE)
        return false;

    if(buffer[6] != crc(buffer, len - 1))
        return false;

    if(buffer[0] != resp)
        return false;

    if(data)
    {
        for(int i=0; i<RESPONSE_DATA_SIZE; i++)
            data[i] = buffer[1 + i];
    }

    return true;
}

uint8_t PZEM004T::crc(uint8_t *data, uint8_t sz)
{
    uint16_t crc = 0;
    for(uint8_t i=0; i<sz; i++)
        crc += *data++;
    return (uint8_t)(crc & 0xFF);
}
