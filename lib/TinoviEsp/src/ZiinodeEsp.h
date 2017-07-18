/*
This file is part of the Tinovi.io esp8266 WiFi communications library for Arduino

 * ZiinodeEsp is a library for the ESP8266/Arduino platform
 * (https://github.com/esp8266/Arduino) to enable easy
 * configuration and reconfiguration of WiFi credentials
 * inspired by:
 * https://github.com/tzapu/WiFiManager
 * http://www.esp8266.com/viewtopic.php?f=29&t=2520
 * https://github.com/chriscook8/esp-arduino-apboot
 * https://github.com/esp8266/Arduino/tree/esp8266/hardware/esp8266com/esp8266/libraries/DNSServer/examples/CaptivePortalAdvanced

2016

This library is free software; you can redistribute it and/or
modify it under the terms of the GNU Lesser General Public
License as published by the Free Software Foundation; either
version 2.1 of the License, or (at your option) any later version.

This library is distributed in the hope that it will be useful,
but WITHOUT ANY WARRANTY; without even the implied warranty of
MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
Lesser General Public License for more details.

You should have received a copy of the GNU Lesser General Public
License along with this library; if not, write to the Free Software
Foundation, Inc., 51 Franklin St, Fifth Floor, Boston, MA  02110-1301  USA

The latest version of this library can always be found at
https://github.com/BlueVia/Official-Arduino
*/

#ifndef ZiinodeEsp_h
#define ZiinodeEsp_h

#include <ESP8266WiFi.h>
#include <WiFiUdp.h>
 //#include <WiFiClientSecure.h>
#include <EEPROM.h>
#include <ESP8266WebServer.h>
#include <ESP8266HTTPClient.h>
#include <DNSServer.h>
#include <stdarg.h>
#include "ByteBuffer.h"

int ets_vsprintf(char *str, const char *format, va_list argptr);
int ets_vsnprintf(char *buffer, size_t sizeOfBuffer,  const char *format, va_list argptr);

const char CacheControl[] PROGMEM = "Cache-Control";
const char NCNS[] PROGMEM = "no-cache, no-store, must-revalidate";
const char HTTP_404[] PROGMEM = "HTTP/1.1 404 Not Found\r\n\r\n";
const char HTTP_200[] PROGMEM = "HTTP/1.1 200 OK\r\nContent-Type: text/html\r\n\r\n";
const char HTTP_HEAD[] PROGMEM = "<!DOCTYPE html><html lang=\"en\"><head><meta name=\"viewport\" content=\"width=device-width, initial-scale=1\"/><title>{v}</title>";
const char HTTP_STYLE[] PROGMEM = "<style>div,input {margin-bottom: 5px;} body{width:200px;display:block;margin-left:auto;margin-right:auto;} button{padding:0.75rem 1rem;border:0;border-radius:0.317rem;background-color:#1fa3ec;color:#fff;line-height:1.5;cursor:pointer;}</style>";
const char HTTP_SCRIPT[] PROGMEM = "<script>function c(l){document.getElementById('s').value=l.innerText||l.textContent;document.getElementById('p').focus();}</script>";
const char HTTP_HEAD_END[] PROGMEM = "</head><body>";
const char HTTP_ITEM[] PROGMEM = "<div><a href='#' onclick='c(this)'>{v}</a></div>";
const char HTTP_FORM[] PROGMEM = "<form method='get' action='wifisave'><input id='s' name='s' length=32 placeholder='SSID'><input id='p' name='p' length=64 placeholder='password'><br/><input id='z' name='z' length=4 placeholder='PIN' value='1111'><br/><button type='submit'>save</button></form>";
const char HTTP_SAVED[] PROGMEM = "<div>Credentials Saved<br />Trying to connect ESP to network.<br />If it fails reconnect to AP to try again</div>";
const char HTTP_END[] PROGMEM = "</body></html>";

const char HTTP_PORTAL_OPTIONS[] PROGMEM = "<form action=\"/wifi\" method=\"get\"><button>Configure WiFi</button></form><br/><form action=\"/0wifi\" method=\"get\"><button>Configure WiFi (No Scan)</button></form>";

//prefix
#define DISCONN 3
#define ACK 4
#define ENQ 5
#define CONN_ACK 6
#define TRAP 7  //
#define ANNOTATION 8  //
#define LOG 9  //

//command
#define CMD_DISCOVER 11   //character response  +
#define CMD_MAC 12  //6 byte mac
#define CMD_NET 13  //net settings IP+GW+SM+DNS = 4x4=16bytes
#define CMD_INT_TYPE 14  // 8 byte
#define CMD_TRIG 15  //  6 byte
#define CMD_TRIG_ALL  16  // TRIG_COUNT * 6 byte
#define CMD_TRAP_ADDR 17  //28 byte adderess
#define CMD_TRIG_OUT 18  // 2byte
#define CMD_POSTING_INTERVAL 19  // 2byte

#define DEBUG 1
#define MAX_SIZE 156


#define APITYPE  "WIFISE0"
//#define APIKEY  "00000A1"
#define PIN  "1111"
#define VERSION  0

class ZiinodeEsp
{
public:
	ZiinodeEsp();
	typedef void (*TOnDataHandler)(byte, ByteBuffer*);
	typedef void (*TOnConnAckHandler)();
	typedef void (*TOnAckHandler)(int);
    void    stop();
    boolean    startConfigPortal(char const *apName);
    void    setupConfigPortal();
    void    setupConfigPortal(char const *apName);
    void debugL(const char *fmt, ...);
    void flush();
    size_t    write(uint8_t b);
    size_t    write(const uint8_t *buf, size_t size);
    void    writeInt(int in);
    void    writeIntE(int in);
    void    writeUint32(uint32_t in);
    void    writeInt64(int64_t in);
    void    sendEvent(int64_t time, int code, const char *fmt, ...);
    void    writeLog(int64_t time, int code, const char *fmt, ...);
    void 	sendNet();
    void    sendInterval();
    void    sendEnq();
    void    ether_loop();
    boolean checkConn();
    uint8_t connected();
    int16_t getInt();
    boolean autoConnect(TOnDataHandler handler, TOnConnAckHandler cack, TOnAckHandler ack);
    boolean autoConnect(char const *apName, TOnDataHandler handler, TOnConnAckHandler cack, TOnAckHandler ack);
    int readThermistor(int adcpin);
    boolean hasConnected();
    void setDevId(String s);
    String getDevId();
    String getApiTypeId();
    void setApiTypeId(String s);
    
    String  beginConfigMode(void);
    void    startWebConfig();
    
    String  getSSID();
    String  getPassword();
    void    setSSID(String s);
    void    setPassword(String p);
    void    resetSettings();
    //for conveniennce
    String  urldecode(const char*);

    //sets timeout before webserver loop ends and exits even if there has been no setup. 
    //usefully for devices that failed to connect at some point and got stuck in a webserver loop
    //in seconds
    void    setTimeout(unsigned long seconds);
    void    setDebugOutput(boolean debug);

    //sets a custom ip /gateway /subnet configuration
    void    setAPConfig(IPAddress ip, IPAddress gw, IPAddress sn);
    
    int     serverLoop();

    uint32_t postingInterval = 1000;

    int    	ncnt();
    void    msg(uint8_t type, int size);
    void    writeBody(uint8_t cmd, const uint8_t *buf, int size);

private:
    int cnt;
    //std::unique_ptr<DNSServer> dnsServer;
    std::unique_ptr<ESP8266WebServer> server;
    HTTPClient http;
    bool hasAddr = false;
    bool acked = false;
    char host[27];
    WiFiClient _client;
    WiFiUDP Udp;
    char packetBuffer[UDP_TX_PACKET_MAX_SIZE]; //buffer to hold incoming packet,
    unsigned long timeoutBegin;
    ByteBuffer* _clientBuffer;
    TOnDataHandler _onDataHandler;
    TOnConnAckHandler _cack;
    TOnAckHandler _ack;

    const int WM_DONE = 0;
    const int WM_WAIT = 10;

    String _apid = APITYPE;
    String _devid = APITYPE;
    const char* _apName = "ZiinodeAP";
    String _ssid = "";
    String _pass = "";
    String _pin = "1111";
    unsigned long timeout = 180 * 1000;
    unsigned long _connectTimeout = 20 * 1000;
    unsigned long start = 0;
    IPAddress _ip;
    IPAddress _gw;
    IPAddress _sn;
    
    String getEEPROMString(int start, int len, String def);
    void setEEPROMString(int start, int len, String string);

    bool keepLooping = true;
    int status = WL_IDLE_STATUS;
    int connectWifi(String ssid, String pass);

    void handleRoot();
    void handleWifi(bool scan);
    void handleWifiSave();
    void handleNotFound();
    void handle204();
    boolean captivePortal();
    uint8_t waitForConnectResult();
    void setAPTimeout(unsigned long seconds);
    
    // DNS server
    const byte DNS_PORT = 53;
    
    boolean isIp(String str);
    String toStringIp(IPAddress ip);

    boolean connect;
#if DEBUG
    boolean _debug = true;
#else
    boolean _debug = false;
#endif

    boolean _hasDevId = false;
    template <typename Generic>
    void DEBUG_WM(Generic text);
};



#endif
