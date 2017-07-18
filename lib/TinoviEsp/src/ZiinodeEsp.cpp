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

/**************************************************************
 * ZiinodeEsp is a library for the ESP8266/Arduino platform
 * (https://github.com/esp8266/Arduino) to enable easy
 * configuration and reconfiguration of WiFi credentials
 * inspired by: 
 * https://github.com/tzapu/WiFiManager
 * http://www.esp8266.com/viewtopic.php?f=29&t=2520
 * https://github.com/chriscook8/esp-arduino-apboot
 * https://github.com/esp8266/Arduino/tree/esp8266/hardware/esp8266com/esp8266/libraries/DNSServer/examples/CaptivePortalAdvanced
 * Licensed under MIT license
 **************************************************************/

#include "ZiinodeEsp.h"

#define MaxBuff 512



#if DEBUG

	void putarr(char * str, unsigned int len)
	{
	  while (len--) // send len bytes
	  {
		  Serial.write(*str++);
	  }
	}
	void ZiinodeEsp::debugL(const char *fmt, ...){
		va_list va;
		va_start (va, fmt);
		char buffer[MaxBuff];
		uint16 len = ets_vsnprintf (buffer, MaxBuff-1, fmt, va);
		va_end (va);
		putarr(buffer,len);
	}
#endif

typedef struct{
	uint8_t ip[4];
	uint8_t gw[4];
	uint8_t sm[4];
	uint8_t dns[4];
} net_t; //16


void ZiinodeEsp::sendNet(){
//	_client.write(CMD_NET);
//	IPAddress ip =WiFi.localIP();
//	IPAddress gw =WiFi.gatewayIP();
//	IPAddress sm =WiFi.subnetMask();
//	IPAddress dns =WiFi.dnsIP();
//	net_t net = {{ip[0],ip[1],ip[2],ip[3]},{gw[0],gw[1],gw[2],gw[3]},{sm[0],sm[1],sm[2],sm[3]},{dns[0],dns[1],dns[2],dns[3]}};
//	writeBody((const byte*)&net,16);
}

void ZiinodeEsp::flush()
{
    return _client.flush();
}

size_t ZiinodeEsp::write(uint8_t b)
{
    return _client.write(b);
}

size_t ZiinodeEsp::write(const uint8_t *buf, size_t size){
	return _client.write(buf,size);
}

void  ZiinodeEsp::writeInt(int in){
    byte *pointer = (byte *)&in;
    _client.write(pointer[1]);
    _client.write(pointer[0]);
}

void  ZiinodeEsp::writeIntE(int in){
    byte *pointer = (byte *)&in;
    _client.write(pointer[0]);
    _client.write(pointer[1]);
}

void ZiinodeEsp::writeUint32(uint32_t in){
    byte *pointer = (byte *)&in;
    _client.write(pointer[3]);
    _client.write(pointer[2]);
    _client.write(pointer[1]);
    _client.write(pointer[0]);
}

void ZiinodeEsp::writeInt64(int64_t in){
    byte *pointer = (byte *)&in;
    _client.write(pointer[7]);
    _client.write(pointer[6]);
    _client.write(pointer[5]);
    _client.write(pointer[4]);
    _client.write(pointer[3]);
    _client.write(pointer[2]);
    _client.write(pointer[1]);
    _client.write(pointer[0]);
}
//ANNOTATION
void  ZiinodeEsp::sendEvent(int64_t time, int code, const char *fmt, ...){
	if(!acked){
		return;
	}
	va_list va;
	va_start (va, fmt);
	char buffer[MaxBuff];
	uint16 size = ets_vsnprintf (buffer, MaxBuff-1, fmt, va);
	va_end (va);
	if(_client && _client.connected()){
		msg(ANNOTATION,size+2+8);
		writeInt64(0);
		writeInt(code);
		//writeInt(len);
		_client.write((const uint8_t *)buffer,size);
		//flush();
	}
}

void  ZiinodeEsp::writeLog(int64_t time, int code, const char *fmt, ...){
	if(!acked){
		return;
	}
	va_list va;
	va_start (va, fmt);
	char buffer[MaxBuff];
	uint16 size = ets_vsnprintf (buffer, MaxBuff-1, fmt, va);
	va_end (va);
	if(_client && _client.connected()){
		msg(LOG,size+2+8);
		writeInt64(0);
		writeInt(code);
		//writeInt(len);
		_client.write((const uint8_t *)buffer,size);
		//flush();
	}
}


void ZiinodeEsp::msg(uint8_t cmd, int size){
//	if(!acked){
//		return;
//	}
	_client.write(cmd);
	writeInt(ncnt());
	writeInt(size);
}

int ZiinodeEsp::ncnt(){
	cnt++;
	return cnt;
}


void  ZiinodeEsp::writeBody(uint8_t cmd, const uint8_t *buf, int size) {
	msg(cmd,size);
	_client.write(buf,size);
}

unsigned long ul_LastComm = 0UL;
boolean ZiinodeEsp::checkConn(){
	if(!hasAddr){
		return false;
	}
	if(!_client.connected() || (ul_LastComm!=0 && (millis() - ul_LastComm) > ((postingInterval*3)+5000))){
		if(_client.connected()){
			_client.write(DISCONN);
		}
		stop();
	}
	return hasAddr;

}
void lastComm(){
	ul_LastComm = millis();
}


void  ZiinodeEsp::sendInterval(){
	acked = true;
	msg(CMD_POSTING_INTERVAL,4);
	writeUint32(postingInterval);
}

uint8_t ZiinodeEsp::connected(){
	if(!hasAddr){
		return 0;
	}
	return _client && (_client.connected() || _client.available());
}


const uint16_t udpLocalTimeout = 5*1000;
// Number of milliseconds to wait without receiving any data before we give up
const uint16_t kNetworkTimeout = 30*1000;
// Number of milliseconds to wait if no data is available before trying again
const uint16_t kNetworkDelay = 100;


unsigned long timeoutStart;
byte cmd=0;
int zsize=-1;

void ZiinodeEsp::stop(){
#if DEBUG
	debugL("reset\n\r");
#endif
	ESP.reset();
	//System.restart();
	hasAddr = false;
	acked = false;
	_client.stop();
	_clientBuffer->clear();
	cmd=0;
	zsize=-1;
#if DEBUG
	debugL("stop\n\r");
#endif
}


int16_t ZiinodeEsp::getInt(){
	int ret;
    byte *pointer = (byte *)&ret;
	pointer[0] =  _client.read();
	pointer[1] =  _client.read();
	return ret;
}

String ZiinodeEsp::getDevId(){
	return _devid;
}
void ZiinodeEsp::setDevId(String s) {
  _hasDevId=true;
  hasAddr = false;
  _devid = s;
  setEEPROMString(0, 7, _devid);
}

String ZiinodeEsp::getApiTypeId(){
	return _apid;
}
void ZiinodeEsp::setApiTypeId(String s) {
	_apid = s;
}



void ZiinodeEsp::resetSettings() {
	  EEPROM.begin(512);
	  delay(10);
	  EEPROM.write(0, 0);
	  EEPROM.write(8, 0);
	  EEPROM.end();

  DEBUG_WM(F("settings invalidated"));
  //WiFi.disconnect();
  delay(200);
}
IPAddress rem(0,0,0,0);

void ZiinodeEsp::ether_loop() {
	int packetSize = Udp.parsePacket();
	if(packetSize){
		IPAddress remote = Udp.remoteIP();
		Udp.read(packetBuffer,UDP_TX_PACKET_MAX_SIZE);
//#if DEBUG
//			Serial.print("Udp ");
//			Serial.print(remote);
//			Serial.print(":");
//			Serial.print(Udp.remotePort());
//			Serial.print(" d:");
//			Serial.println(packetBuffer);
//#endif

		if(packetBuffer[0]=='S' && packetBuffer[1]=='E' && packetBuffer[2]=='E' && packetBuffer[3]=='K'){
			//reconnect to new agent
			if(rem[0]!=remote[0] || rem[1]!=remote[1] || rem[2]!=remote[2] || rem[3]!=remote[3]){
				rem = remote;
				memset(host, 0, sizeof(host));
				sprintf(host, "%d.%d.%d.%d", remote[0], remote[1], remote[2], remote[3]);
#if DEBUG
debugL(" conn to:%s:%i \n\r",host,8787);
#endif
				hasAddr = true;
				if(_client.connected()){
					_client.stop();
				}
				timeoutStart = millis();
				while(((millis() - timeoutStart) < kNetworkTimeout) ){
#if DEBUG
Serial.println("connecting ..");
#endif
					if(_client.connect(host,8787)){
						write(ENQ);
						write((const byte*)_apid.c_str(),7);
						write((const byte*)_devid.c_str(),7);
						write((const byte*)PIN,4);
						writeInt(VERSION);
#if DEBUG
Serial.println("enq sent");
#endif
						return;
					}
				}
#if DEBUG
Serial.println("conn timeout");
#endif
			    hasAddr = false;
			}
		}

	}else if(((millis() - timeoutBegin) < udpLocalTimeout) && !hasAddr){ //check local server first - wait 5sec
		return;
	}
	while(!hasAddr)
	{
		//Serial.print("[HTTP] begin...\n");
		// configure traged server and url
		//http.begin("192.168.1.12", 443, "/test.html", true, "7a 9c f4 db 40 d3 62 5a 6e 21 bc 5c cc 66 c8 3e a1 45 59 38"); //HTTPS
		char path[50];
	    sprintf(path,"/api/v1/node/host/%s%s%s",_apid.c_str(),_devid.c_str(),PIN);

#if DEBUG
	debugL("HTTP GET...:%s hasAdr:%i hasD:%i %s\n\r",_devid.c_str(),hasAddr,_hasDevId, path);
#endif
		http.begin("www.tinovi.io", 80, path); //HTTP
		int httpCode = http.GET();
		// file found at server
		if(httpCode == 200)
		{
				//String payload = http.getString();
				//Serial.println(payload);

		        int bodyLen = http.getSize();
				#if DEBUG
					debugL("Content length is:%i \n\r",bodyLen);
				#endif

				WiFiClient* htc = http.getStreamPtr();
		        // Now we've got to the body, so we can print it out
		        timeoutStart = millis();
		        // Whilst we haven't timed out & haven't reached the end of the body
		        int rec=0;

				memset(host, 0, sizeof(host));
		        while (!hasAddr && (htc->connected() || htc->available()) &&
		               ((millis() - timeoutStart) < kNetworkTimeout) )
		        {
		        	if (htc->available() && rec<bodyLen) {
		        		while(htc->available() && rec<bodyLen)
		        		{
			        		host[rec]=htc->read();
			        		rec++;
		        		}
		        		if(rec == 4 && bodyLen == 4 && host[0]==host[1] && host[1]==host[2] && host[2]==host[3] && host[3]==0)
		        		{
							http.end();
							//client.stop();
							hasAddr = false;
#if DEBUG
	debugL("wait for reg, got 0.0.0.0 \n\r");
#endif
							delay(5000);
							return;
		        		}else if(rec == bodyLen)
		        		{
							http.end();
							if(!_hasDevId){
								setDevId(host);
								return;
							}
							hasAddr = true;
							http.end();
							//_client.stop();
#if DEBUG
debugL("API conn: %s:%i \n\r",host,8787);
#endif
							timeoutStart = millis();
							while(((millis() - timeoutStart) < kNetworkTimeout) ){
								if(_client.connect(host,8787)){
									write(ENQ);
									write((const byte*)_apid.c_str(),7);
									write((const byte*)_devid.c_str(),7);
									write((const byte*)PIN,4);
									writeInt(VERSION);
#if DEBUG
debugL("enq sent  \n\r");
#endif
								    return;
								}
		        		   }
		        	}
		            else
		            {
		                delay(kNetworkDelay);
		            }
		        }

			}
		} else {
#if DEBUG
	debugL("[HTTP] GET... failed not 200\n");
#endif
		}
		http.end();
	}

	if (connected())
	{

		if(_client.available() > 0){
			if(cmd==0){
				lastComm();
				cmd = _client.read();
				zsize = -1;
				if(cmd==CONN_ACK){
					if( _cack != 0 ){
						_cack();
					}
					cmd = 0;
					return;
				}
				if(cmd==DISCONN){
#if DEBUG
			Serial.println("DISCONN:");
#endif

					if(_client.connected()){
						_client.stop();
					}
					hasAddr = false;
					return;
				}

			}
			if(cmd==ACK && _client.available() > 1){
				int aa = getInt();
				if( _ack != 0 ){
					_ack(aa);
				}
				cmd = 0;
				return;
			}else if(cmd!=0 && _client.available() > 1 && zsize==-1 && _clientBuffer->getSize()==0){
				zsize = getInt();
				timeoutStart = millis();
				if(zsize>MAX_SIZE){
#if DEBUG
			Serial.print("cmd:");
			Serial.print(cmd);
			Serial.print(" av:");
			Serial.print(_client.available());
			Serial.print(" SKIP:");
			Serial.println(zsize);
			while(_client.available()){
				Serial.print(_client.read());
			}
			Serial.println();
#endif
					cmd=0;
					zsize=-1;
					_client.flush();
					return;
				}
#if DEBUG
			Serial.print("cmd:");
			Serial.print(cmd);
			Serial.print(" av:");
			Serial.print(_client.available());
			Serial.print(" s:");
			Serial.println(zsize);
#endif
			}
			while(zsize>0 && _client.available() > 0  && _clientBuffer->getCapacity()>0 && ((millis() - timeoutStart) < 2000)) {
				_clientBuffer->put(_client.read());
				zsize--;
			}

			if(zsize>0 && ((millis() - timeoutStart) > 2000)){
#if DEBUG
			Serial.print("tiemout:");
			Serial.println(cmd);
#endif
				cmd=0;
				zsize=-1;
				_clientBuffer->clear();
			}
			if(cmd!=0 && zsize==0){
				//packet ready
#if DEBUG
			Serial.print("packet:");
			Serial.println(_clientBuffer->getSize());
#endif
				if( _onDataHandler != 0 ){
					_onDataHandler(cmd,_clientBuffer);
				}else{
					//
				}
				_clientBuffer->clear();
				cmd=0;
				zsize=-1;
			}
		}

	}
}


// resistance at 25 degrees C
#define THERMISTORNOMINAL 10000
// temp. for nominal resistance (almost always 25 C)
#define TEMPERATURENOMINAL 25
// how many samples to take and average, more takes longer
// but is more 'smooth'
#define NUMSAMPLES 5
// The beta coefficient of the thermistor (usually 3000-4000)
#define BCOEFFICIENT 3470
// the value of the 'other' resistor
#define SERIESRESISTOR 10000
//========================================

int samples[NUMSAMPLES];

float readAnalog(int adcpin){
	uint8_t i;
	float average;
// take N samples in a row, with a slight delay
	for (i = 0; i < NUMSAMPLES; i++) {
		samples[i] = analogRead(adcpin);
		delay(10);
	}
// average all the samples out
	average = 0;
	for (i = 0; i < NUMSAMPLES; i++) {
		average += samples[i];
	}
	return average /= NUMSAMPLES;
}

int ZiinodeEsp::readThermistor(int adcpin) {
//	if (RawADC == 0) {
//		return 0;
//	}
	float average = readAnalog(adcpin);
//#if DEBUG
//	Serial.print("Average analog reading ");
//	Serial.println(average);
//#endif
	if (average == 0)
		return 0;
// convert the value to resistance
	average = 1023 / average - 1;
	average = SERIESRESISTOR / average;
//#if DEBUG
//	Serial.print("Thermistor resistance ");
//	Serial.println(average);
//#endif
	float steinhart;
	steinhart = average / THERMISTORNOMINAL; // (R/Ro)
	steinhart = log(steinhart); // ln(R/Ro)
	steinhart /= BCOEFFICIENT; // 1/B * ln(R/Ro)
	steinhart += 1.0 / (TEMPERATURENOMINAL + 273.15); // + (1/To)
	steinhart = 1.0 / steinhart; // Invert
	steinhart -= 273.15; // convert to C
//#if DEBUG
//	//Serial.print("Temperature ");
//	Serial.print(steinhart);
//	//Serial.println(" *C");
//#endif
//delay(1000);
	return steinhart * 10;
}

void ZiinodeEsp::setupConfigPortal() {
	setupConfigPortal("ZiinodeESP");
}

void ZiinodeEsp::setupConfigPortal(char const *apName) {
  //dnsServer.reset(new DNSServer());
  server.reset(new ESP8266WebServer(80));

  DEBUG_WM(F(""));
  _apName = apName;
  //_apPassword = apPassword;
  start = millis();

  DEBUG_WM(F("Configuring access point... "));
  DEBUG_WM(_apName);
//  if (_apPassword != NULL) {
//    if (strlen(_apPassword) < 8 || strlen(_apPassword) > 63) {
//      // fail passphrase to short or long!
//      DEBUG_WM(F("Invalid AccessPoint password. Ignoring"));
//      _apPassword = NULL;
//    }
//    DEBUG_WM(_apPassword);
//  }

  //optional soft ip config
  if (_ip) {
    DEBUG_WM(F("Custom IP/GW/Subnet"));
    WiFi.softAPConfig(_ip, _gw, _sn);
  }

  WiFi.softAP(_apName);

  delay(500); // Without delay I've seen the IP address blank
  DEBUG_WM(F("AP IP address: "));
  DEBUG_WM(WiFi.softAPIP());

  /* Setup the DNS server redirecting all the domains to the apIP */
  //dnsServer->setErrorReplyCode(DNSReplyCode::NoError);
  //dnsServer->start(DNS_PORT, "*", WiFi.softAPIP());

  /* Setup web pages: root, wifi config pages, SO captive portal detectors and not found. */
  server->on("/", std::bind(&ZiinodeEsp::handleRoot, this));
  server->on("/wifi", std::bind(&ZiinodeEsp::handleWifi, this, true));
  server->on("/0wifi", std::bind(&ZiinodeEsp::handleWifi, this, false));
  server->on("/wifisave", std::bind(&ZiinodeEsp::handleWifiSave, this));
  server->on("/generate_204", std::bind(&ZiinodeEsp::handle204, this));  //Android/Chrome OS captive portal check.
  server->on("/fwlink", std::bind(&ZiinodeEsp::handleRoot, this));  //Microsoft captive portal. Maybe not needed. Might be handled by notFound handler.
  server->onNotFound (std::bind(&ZiinodeEsp::handleNotFound, this));
  server->begin(); // Web server start
  DEBUG_WM(F("HTTP server started"));
}



//IPAddress ipMulti(239, 193, 110, 110);

boolean  ZiinodeEsp::startConfigPortal(char const *apName) {
  //setup AP
  WiFi.mode(WIFI_AP);


  connect = false;
  setupConfigPortal(apName);
  DEBUG_WM("start loop");
  start = millis();
  bool con = false;
  while (timeout == 0 || millis() < (start + timeout)) {
    //DNS
    //dnsServer->processNextRequest();
    //HTTP
	 // DEBUG_WM("handleClient");
    server->handleClient();


    if (connect) {
      delay(2000);
      DEBUG_WM(F("Connecting to "));
      DEBUG_WM(_ssid);
      DEBUG_WM(_pass);
      connect = false;
      // using user-provided  _ssid, _pass in place of system-stored ssid amd pass
      if (connectWifi(_ssid, _pass) != WL_CONNECTED) {
        DEBUG_WM(F("Failed to connect."));
      } else {
        DEBUG_WM(F("COnneced."));
        //connected
        WiFi.mode(WIFI_STA);
        con = true;
        break;
      }

    }
    //digitalWrite(2,HIGH);
	//  DEBUG_WM("yield");
    yield();
  }

  server.reset();
//  dnsServer.reset();
  DEBUG_WM("end loop");
  return  con;
}

ZiinodeEsp::ZiinodeEsp() {
	_clientBuffer = (ByteBuffer*)malloc(sizeof(ByteBuffer));
	_clientBuffer->init(512);
	timeoutBegin = millis();
	//"239.193.110.110"
}

boolean ZiinodeEsp::autoConnect(TOnDataHandler handler, TOnConnAckHandler cack, TOnAckHandler ack){
  String ssid = "ESP" + String(ESP.getChipId());
  return autoConnect(ssid.c_str(),handler, cack, ack);
}

boolean ZiinodeEsp::autoConnect(char const *apName, TOnDataHandler handler, TOnConnAckHandler cack, TOnAckHandler ack){
  _onDataHandler = handler;
  _cack=cack;
  _ack=ack;
  DEBUG_WM(F(""));
  DEBUG_WM(F("AutoConnect"));
  if(digitalRead(0)){
	  getSSID();
  }
  //getSSID();
  // read eeprom for ssid and pass
  _devid=getEEPROMString(0,7, _devid);
  if (_ssid != ""){
	  getPassword();
	  _pin = getEEPROMString(8,4,_pin);
	  //use SDK functions to get SSID and pass
	  WiFi.mode(WIFI_STA);
	  int s = connectWifi(_ssid, _pass);
	  if (s == WL_CONNECTED) {
		DEBUG_WM(F("IP Address:"));
		DEBUG_WM(WiFi.localIP());
		//connected
		return true;
	  }
  }

  return startConfigPortal(apName);
}

uint8_t ZiinodeEsp::waitForConnectResult() {
  if (_connectTimeout == 0) {
    return WiFi.waitForConnectResult();
  } else {
    unsigned long strt = millis() + _connectTimeout;
    boolean keepConnecting = true;
    uint8_t status;
    while (keepConnecting) {
      status = WiFi.status();
      if (millis() > strt ) {
        keepConnecting = false;
        DEBUG_WM (F("Connection timed out"));
      }
      if (status == WL_CONNECTED || status == WL_CONNECT_FAILED) {
        keepConnecting = false;
      }
      delay(100);
    }
    return status;
  }
}


int ZiinodeEsp::connectWifi(String ssid, String pass) {
  WiFi.disconnect();
  WiFi.begin(ssid.c_str(), pass.c_str());
  int connRes = waitForConnectResult();
  DEBUG_WM ("connectWifi: ");
  DEBUG_WM ( connRes );
  if(connRes==WL_CONNECTED){
	  uint8_t i = Udp.beginMulticast(WiFi.localIP(),  IPAddress(239, 193, 110, 110), 9500);
	  DEBUG_WM(F("beginMulticast:"));
	  DEBUG_WM(i);
	  timeoutBegin = millis();
  }
  return connRes;
}


String ZiinodeEsp::getSSID() {
  if (_ssid == "") {
    DEBUG_WM(F("Reading WiFi SSID"));
    _ssid = WiFi.SSID();//getEEPROMString(0, 32);
    DEBUG_WM(F("SSID: "));
    DEBUG_WM(_ssid);
  }
  return _ssid;
}

String ZiinodeEsp::getPassword() {
  if (_pass == "") {
    _pass = WiFi.psk();//getEEPROMString(32, 64);
  }
  return _pass;
}

String ZiinodeEsp::getEEPROMString(int sss, int len, String def) {
  EEPROM.begin(512);
  delay(10);
  String string = "";
  if(EEPROM.read(sss)==55){
	  if(start==0){
		_hasDevId = true;
	  }
	  for (int i = 1 + sss; i < (1 + sss + len); i++) {
		////DEBUG_WM(F(i));
		string += char(EEPROM.read(i));
	  }
  }else{
	  string = def;
  }
  EEPROM.end();
  //DEBUG_WM(F("Read " + string));
  return string;
}
void ZiinodeEsp::setEEPROMString(int sss, int len, String string) {
  EEPROM.begin(512);
  delay(10);
  EEPROM.write(sss, 55);
  int si = 0;
  for (int i = 1 + sss; i < (1 + sss + len); i++) {
    char c;
    if (si < string.length()) {
      c = string[si];
      ////DEBUG_WM(F("Wrote: "));
      ////DEBUG_WM(F(c));
    } else {
      c = 0;
    }
    EEPROM.write(i, c);
    si++;
  }
  EEPROM.end();
  //DEBUG_WM(F("Wrote " + string));
}

String ZiinodeEsp::urldecode(const char *src)
{
  String decoded = "";
  char a, b;
  while (*src) {
    if ((*src == '%') &&
        ((a = src[1]) && (b = src[2])) &&
        (isxdigit(a) && isxdigit(b))) {
      if (a >= 'a')
        a -= 'a' - 'A';
      if (a >= 'A')
        a -= ('A' - 10);
      else
        a -= '0';
      if (b >= 'a')
        b -= 'a' - 'A';
      if (b >= 'A')
        b -= ('A' - 10);
      else
        b -= '0';

      decoded += char(16 * a + b);
      src += 3;
    } else if (*src == '+') {
      decoded += ' ';
      *src++;
    } else {
      decoded += *src;
      *src++;
    }
  }
  decoded += '\0';

  return decoded;
}

void ZiinodeEsp::setTimeout(unsigned long seconds) {
  timeout = seconds * 1000;
}

void ZiinodeEsp::setAPTimeout(unsigned long seconds) {
	timeout = seconds * 1000;
}
void ZiinodeEsp::setDebugOutput(boolean debug) {
  _debug = debug;
}

void ZiinodeEsp::setAPConfig(IPAddress ip, IPAddress gw, IPAddress sn) {
  _ip = ip;
  _gw = gw;
  _sn = sn;
}


/** Handle root or redirect to captive portal */
void ZiinodeEsp::handleRoot() {
  DEBUG_WM(F("Handle root"));
//  if (captivePortal()) { // If caprive portal redirect instead of displaying the page.
//    return;
//  }

  server->sendHeader(FPSTR(CacheControl),FPSTR(NCNS));
  server->sendHeader("Pragma", "no-cache");
  server->sendHeader("Expires", "-1");
  server->setContentLength(CONTENT_LENGTH_UNKNOWN);
  server->send(200, "text/html", ""); // Empty content inhibits Content-length header so we have to close the socket ourselves.

  String head = FPSTR(HTTP_HEAD);
  head.replace("{v}", "Options");
  server->sendContent(head);
  server->sendContent_P(HTTP_SCRIPT);
  server->sendContent_P(HTTP_STYLE);
  server->sendContent_P(HTTP_HEAD_END);

  String title = "<h1>";
  title += _apName;
  title += "</h1>";
  server->sendContent(title);

  server->sendContent_P(HTTP_PORTAL_OPTIONS);
  server->sendContent_P(HTTP_END);

  server->client().stop(); // Stop is needed because we sent no content length
}

/** Wifi config page handler */
void ZiinodeEsp::handleWifi(bool scan) {
	  DEBUG_WM(F("handleWifi"));
 server->sendHeader(FPSTR(CacheControl),FPSTR(NCNS));
  server->sendHeader("Pragma", "no-cache");
  server->sendHeader("Expires", "-1");
  server->setContentLength(CONTENT_LENGTH_UNKNOWN);
  server->send(200, "text/html", ""); // Empty content inhibits Content-length header so we have to close the socket ourselves.
  server->setContentLength(CONTENT_LENGTH_UNKNOWN);

  String head = FPSTR(HTTP_HEAD);
  head.replace("{v}", "Config ESP");
  server->sendContent(head);
  server->sendContent_P(HTTP_SCRIPT);
  server->sendContent_P(HTTP_STYLE);
  server->sendContent_P(HTTP_HEAD_END);

  if (scan) {
    int n = WiFi.scanNetworks();
    DEBUG_WM(F("Scan done"));
    if (n == 0) {
      DEBUG_WM(F("No networks found"));
      server->sendContent("<div>No networks found. Refresh to scan again.</div>");
    }
    else {
      for (int i = 0; i < n; ++i)
      {
        DEBUG_WM(WiFi.SSID(i));
        DEBUG_WM(WiFi.RSSI(i));
        String item = FPSTR(HTTP_ITEM);
        item.replace("{v}", WiFi.SSID(i));
        server->sendContent(item);
        yield();
      }
    }
  }
  
  server->sendContent_P(HTTP_FORM);
  server->sendContent_P(HTTP_END);
  server->client().stop();
  
  //DEBUG_WM(F("Sent config page"));
}

/** Handle the WLAN save form and redirect to WLAN config page again */
void ZiinodeEsp::handleWifiSave() {
  DEBUG_WM(F("WiFi save"));
  _ssid = urldecode(server->arg("s").c_str());
  _pass = urldecode(server->arg("p").c_str());
  _pin = urldecode(server->arg("z").c_str());
  setEEPROMString(8,4,_pin);
  server->sendHeader(FPSTR(CacheControl),FPSTR(NCNS));
  server->sendHeader("Pragma", "no-cache");
  server->sendHeader("Expires", "-1");
  server->send(200, "text/html", ""); // Empty content inhibits Content-length header so we have to close the socket ourselves.

  String head = FPSTR(HTTP_HEAD);
  head.replace("{v}", "Credentials Saved");
  server->sendContent(head);
  server->sendContent_P(HTTP_SCRIPT);
  server->sendContent_P(HTTP_STYLE);
  server->sendContent_P(HTTP_HEAD_END);
  
  server->sendContent_P(HTTP_SAVED);

  server->sendContent_P(HTTP_END);
  server->client().stop();
  
  DEBUG_WM(F("Sent wifi save page"));
  
  //saveCredentials();
  connect = true; //signal ready to connect/reset
}

void ZiinodeEsp::handle204() {
  DEBUG_WM(F("204 No Response"));
  server->sendHeader(FPSTR(CacheControl),FPSTR(NCNS));
  server->sendHeader("Pragma", "no-cache");
  server->sendHeader("Expires", "-1");
  server->send ( 204, "text/plain", "");
}


void ZiinodeEsp::handleNotFound() {
//  if (captivePortal()) { // If captive portal redirect instead of displaying the error page.
//    return;
//  }
  String message = "File Not Found\n\n";
  message += "URI: ";
  message += server->uri();
  message += "\nMethod: ";
  message += ( server->method() == HTTP_GET ) ? "GET" : "POST";
  message += "\nArguments: ";
  message += server->args();
  message += "\n";

  for ( uint8_t i = 0; i < server->args(); i++ ) {
    message += " " + server->argName ( i ) + ": " + server->arg ( i ) + "\n";
  }
  server->sendHeader(FPSTR(CacheControl),FPSTR(NCNS));
  server->sendHeader("Pragma", "no-cache");
  server->sendHeader("Expires", "-1");
  server->send ( 404, "text/plain", message );
}


/** Redirect to captive portal if we got a request for another domain. Return true in that case so the page handler do not try to handle the request again. */
//boolean ZiinodeEsp::captivePortal() {
//  if (!isIp(server->hostHeader()) ) {
//    DEBUG_WM(F("Request redirected to captive portal"));
//    server->sendHeader("Location", String("http://") + toStringIp(server->client().localIP()), true);
//    server->send ( 302, "text/plain", ""); // Empty content inhibits Content-length header so we have to close the socket ourselves.
//    server->client().stop(); // Stop is needed because we sent no content length
//    return true;
//  }
//  return false;
//}


template <typename Generic>
void ZiinodeEsp::DEBUG_WM(Generic text) {
  if(_debug) {
    Serial.print("*WM: ");
    Serial.println(text);    
  }
}




/** Is this an IP? */
boolean ZiinodeEsp::isIp(String str) {
  for (int i = 0; i < str.length(); i++) {
    int c = str.charAt(i);
    if (c != '.' && (c < '0' || c > '9')) {
      return false;
    }
  }
  return true;
}

/** IP to String? */
String ZiinodeEsp::toStringIp(IPAddress ip) {
  String res = "";
  for (int i = 0; i < 3; i++) {
    res += String((ip >> (8 * i)) & 0xFF) + ".";
  }
  res += String(((ip >> 8 * 3)) & 0xFF);
  return res;
}


