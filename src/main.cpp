
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

#include <ESP8266WiFi.h>          //https://github.com/esp8266/Arduino

//needed for library
#include <DNSServer.h>
#include <ESP8266WebServer.h>

#include "DallasTemperature.h"
#include "DHT.h"
#include "PZEM004T.h"
#include "ZiinodeEsp.h"
#include "SHT1x.h"

ZiinodeEsp zn;

//analog 10k thermocouple
#define NTC10K 1
//dallas sensor
#define INPUT_DS 2
//digital themp & hummudity
#define INPUT_TH_11 3
#define INPUT_TH_22 4
//analog read
#define VOLTAGE 5
#define ENERGYPZ 7
#define SHT10 8

//total 4+1
#define IN_start 15
//total 20*6+1
#define TRIG_start 20

#define READ_COUNT 16
#define TRIG_COUNT 10

IPAddress ip(192,168,1,1);

typedef struct{
	uint8_t input;
	uint8_t oper;
	uint8_t out;
	uint8_t out_dir;
	int16_t val;
} trigger_t;//size 6


#define IN_COUNT 8
#define OUT_COUNT 4
//uint8_t in_pin[IN_COUNT] = { A0, 0, 4, 12, 5, 13, 2, 16};
//uint8_t out_pins[OUT_COUNT] = {14};
uint8_t in_pin[IN_COUNT] = { A0, 30, 12, 30, 4, 5, 2, 0};
uint8_t out_pins[OUT_COUNT] = {15,16,14,13};

//reading digital may have hummuduty as well

#define OPER_EQUAL 1
#define OPER_LESS 2
#define OPER_GREATER 3

int readings[READ_COUNT];

byte in_type[IN_COUNT];
trigger_t trigger[TRIG_COUNT];

byte out_s;

boolean hasAck=false;

void savePostingInt(){
  EEPROM.begin(512);
  delay(10);
  EEPROM.write(14, 55);
  EEPROM.put(15,zn.postingInterval);
  EEPROM.end();
}

void saveInType(){
  EEPROM.begin(512);
  delay(10);
  EEPROM.write(19, 55);
  EEPROM.put(20,in_type);
  EEPROM.end();
}

void  saveTrig(){
  EEPROM.begin(512);
  delay(10);
  EEPROM.write(24, 55);
  EEPROM.put(25,trigger);
  EEPROM.end();
}

void readConf(){
  EEPROM.begin(512);
  delay(10);
  //total 4+1
  if(EEPROM.read(14)==55){
	  EEPROM.get(15,zn.postingInterval);
#if DEBUG
	zn.debugL("conf pinterval %i,",zn.postingInterval);
#endif
	if(zn.postingInterval>600000){
		zn.postingInterval = 1000;
		savePostingInt();
	}
  }

  if(EEPROM.read(19)==55){
	  EEPROM.get(20,in_type);
//		if(in_type[4]==ENERGYPZ){
//			pzem = new PZEM004T(4,5);
//		}
  }
  //total 20*6+1
  if(EEPROM.read(24)==55){
	  EEPROM.get(25,trigger);
  }
  EEPROM.end();
}



void sendInputType(){
	zn.writeBody(CMD_INT_TYPE,(const byte *)in_type,IN_COUNT);
}

void sendTrigAll(){
	zn.writeBody(CMD_TRIG_ALL,(const byte *)trigger,TRIG_COUNT * 6);
}


void sendBinTrap(){
	zn.msg(TRAP,READ_COUNT * 2 + 1);
    for(int i=0;i<READ_COUNT;i++){
    	zn.writeIntE(readings[i]);
#if DEBUG
	zn.debugL("%i,",readings[i]);
#endif
    }
	Serial.println();
	//zn.write((const byte*)&readings,8);
	zn.write(out_s);
}

void ack(int mid){
#if DEBUG
	zn.debugL("->ack:%i. \n\r",mid);
#endif
}


void cack(){
	hasAck = true;
	ESP.wdtEnable(WDTO_8S);
	sendInputType();
	sendTrigAll();
	//zn.sendNet();
	zn.sendInterval();
#if DEBUG
zn.debugL("conn ack.... \n\r");
#endif
}

void dataReceived(byte cmd, ByteBuffer *buf){
	if(cmd==CMD_POSTING_INTERVAL){
		zn.postingInterval = buf->getRUInt32();
		savePostingInt();
	}else if(cmd==CONN_ACK){
		cack();
	}else if(cmd==CMD_TRIG_OUT){
		byte idx = buf->get();
		uint8_t val = buf->get();
		if(idx<OUT_COUNT){
			uint8_t pin = out_pins[idx];
			uint8_t cur = digitalRead(pin);
			zn.writeLog(0,1,"setPin #:%i val:%i  idx:%i cur:%i",pin,val, idx, cur);
			if(cur!=val){
				digitalWrite(pin,val);
				if(val){
					bitSet(out_s, idx);
				}else{
					bitClear(out_s, idx);
				}
			}
		}else{
			zn.writeLog(0,1,"ERR out idx:%i max:%i",idx,OUT_COUNT);
		}
	}else if(cmd==CMD_INT_TYPE){
		byte idx = buf->get();
		if(idx<IN_COUNT){
			byte t = buf->get();
			in_type[idx]=t;
			saveInType();
			zn.writeLog(0,1,"in type idx:%i val:%i",idx,t);
		}else{
			zn.writeLog(0,1,"ERR in type idx:%i max:%i",idx,IN_COUNT);
		}
	}else if(cmd==CMD_TRIG){
		//typedef struct{
		//	uint8_t input;
		//	uint8_t oper;
		//	uint8_t out;
		//	uint8_t out_dir;
		//	int val;
		//} trigger_t;//size 6
		byte idx = buf->get();
		trigger[idx].input = buf->get();
		trigger[idx].oper = buf->get();
		trigger[idx].out = buf->get();
		trigger[idx].out_dir = buf->get();
		trigger[idx].val = buf->getInt();
		if(idx<TRIG_COUNT && trigger[idx].input < READ_COUNT){
			saveTrig();
			zn.writeLog(0,1,"trig idx:%i input:%i oper:%i out:%i out_dir:%i val:%i",idx,trigger[idx].input,trigger[idx].oper ,trigger[idx].out,trigger[idx].out_dir,trigger[idx].val);
		}else{
			zn.writeLog(0,1,"ERR ttrig idx:%i max:%i",idx,TRIG_COUNT);
		}
	}else if(cmd==CMD_TRIG_ALL){
		for(int idx=0;idx<TRIG_COUNT;idx++){
			trigger[idx].input = buf->get();
			trigger[idx].oper = buf->get();
			trigger[idx].out = buf->get();
			trigger[idx].out_dir = buf->get();
			trigger[idx].val = buf->getInt();
		}
		//EEPROM_writeAnything(e_trigger,trig);
	}else if(cmd==ACK){
		//ul_PreviousMillis = millis();
	}
	zn.msg(ACK,1);
	zn.write(cmd);
	//zn.flush();
}

char oper[3] = {'=','<','>'};

void setPin(uint8_t i){
	if(trigger[i].out<OUT_COUNT){
		uint8_t pin = out_pins[trigger[i].out];
		uint8_t val = trigger[i].out_dir;
		if(digitalRead(pin)!=val){
			zn.writeLog(0,1,"setPin:%i val:%i",pin,val);
			zn.sendEvent(0,i,"Output changed #:%i state:%i on value:%i",trigger[i].out,val,trigger[i].val);
			digitalWrite(pin,val);
		}
		if(val){
			bitSet(out_s, trigger[i].out);
		}else{
			bitClear(out_s, trigger[i].out);
		}
	}else{
		zn.sendEvent(0,i,"Trigger #:%i on %i %c %i",i,trigger[i].val,oper[trigger[i].oper], readings[trigger[i].input]);
	}

}

void trigger_out() {
	for (uint8_t i = 0; i < TRIG_COUNT; i++) {
		if(trigger[i].oper>0 && trigger[i].oper<255 && trigger[i].input < READ_COUNT){
			if(trigger[i].oper==OPER_EQUAL && trigger[i].val==readings[trigger[i].input] ){
				setPin(i);
			}if(trigger[i].oper==OPER_GREATER && trigger[i].val<readings[trigger[i].input] ){
				setPin(i);
			}if(trigger[i].oper==OPER_LESS && trigger[i].val>readings[trigger[i].input] ){
				setPin(i);
			}
		}
	}
}


void read_inputs() {

	for (int i = 0; i < IN_COUNT; i++) {
		if(in_pin[i]<30){
		///=====NTC10K=======================================================
		if(in_type[i]==NTC10K){
			readings[(i * 2)] = zn.readThermistor(in_pin[i]);
//#if DEBUG
//	zn.writeLog(1,"NTC10k :%i  temp:%i",i,readings[i * 2]);
//#endif
		}else if(in_type[i]==INPUT_DS){
			OneWire oneWire(in_pin[i]);
			DallasTemperature sensor_inhouse(&oneWire);
			sensor_inhouse.begin();
			if(sensor_inhouse.getDeviceCount()){
				sensor_inhouse.requestTemperatures();
				int rez = sensor_inhouse.getTempCByIndex(0)*10;
				if(rez !=-1270){
					readings[i * 2] = rez;
				}
			}

//			DS1820 dsb(in_pin[i]);
//			if(dsb.deviceFamily()){
//				readings[i * 2] =(dsb.getTemperatureHiRes()*10);
//			}

			//			DS18B20 dsb(in_pin[i]);
//			float ds=0;
//			int rez = dsb.getTemperature(ds);
//			if(rez==0){
//				readings[i * 2] =ds *10;//(dsb.getTemperatureHiRes()*10);
//			}else{
//#if DEBUG
//	NeoSerial.print("DS18err:");
//	NeoSerial.println(rez);
//#endif
//			}
			//			#if DEBUG
			//				NeoSerial.print("DS18:");
			//				NeoSerial.println(dsb.deviceFamily());
			//			#endif
//#if DEBUG
//	zn.debugL(" r:%i t:%i DS:%i val:%i adr:%i\n\r",i, in_type[i], in_pin[i],readings[(i * 2)],(i * 2));
//#endif
//#if DEBUG
//	zn.writeLog(1,"DS:%i val:%i",in_pin[i],readings[i * 2]);
//#endif
		}else if(in_type[i]==INPUT_TH_11){
			DHT dht(in_pin[i], DHT11);
			dht.begin();
			int t = dht.readTemperature() * 10;
			int h = dht.readHumidity() * 10;
			if(t>0){
				readings[i * 2] = t;
				readings[i * 2+1] = h;
			}
//#if DEBUG
//	zn.writeLog(1,"DS:%i humy:%i temp:%i",in_pin[i],readings[i * 2], readings[i * 2+1]);
//#endif
		}else if(in_type[i]==INPUT_TH_22){
			DHT dht(in_pin[i], DHT22);
			dht.begin();
			int t = dht.readTemperature() * 10;
			int h = dht.readHumidity() * 10;
			if(t>0){
				readings[i * 2] = t;
				readings[i * 2+1] = h;
			}
//#if DEBUG
//	zn.debugL("DS:%i humy:%i temp:%i ",in_pin[i],readings[i * 2], readings[i * 2+1]);
//#endif
		}else if(in_type[i]==ENERGYPZ && i==1){
			PZEM004T pzem;
			pzem.setAddress(ip);
			float v = pzem.voltage(ip);
			float i = pzem.current(ip);
			float p = pzem.power(ip);
			float e = pzem.energy(ip);
			readings[2] = v*10; //voltage /1A
			readings[3] = i*10; //current /1B
			readings[6] = p*10; //power /3a
			readings[7] = e*10; //energy /3b
			pzem.close();
		}else if(in_type[i]==SHT10 && i%2 == 0){
			SHT1x  sht1x(in_pin[i],in_pin[i+1]);
			//pzem.setAddress(ip);
			float h = sht1x.readHumidity();
			float t = sht1x._temperature;
			readings[i] = t*10;
			readings[i+1] = h*10;
		}else if(in_type[i]==VOLTAGE){
			readings[(i * 2)]  = analogRead(in_pin[i]);
//#if DEBUG
//	zn.writeLog(0,1,"voltage:%i voltage:%i ",in_pin[i], readings[i * 2]);
//#endif
		}else if(in_type[i]==SHT10 && (i == 4 || i == 6)){
			SHT1x  sht1x(in_pin[i],in_pin[i+1]);
			//pzem.setAddress(ip);
			float h = sht1x.readHumidity();
			float t = sht1x._temperature;
			readings[i] = t*10;
			readings[i+1] = h*10;
		}
	}
}



uint32_t lastConnectionTime = 0;

void loop() {
	delay(15);
//	if(!reset){
//		wdt_reset();
//	}
	zn.ether_loop();
	if(hasAck){
		ESP.wdtFeed();
	}
	if((millis() - lastConnectionTime > zn.postingInterval)) {
#if DEBUG
	zn.debugL("... %i %i \n\r",zn.connected(), hasAck);
#endif
//		read_inputs();
//		trigger_out();
		boolean cc = zn.checkConn();
		if(cc){
			if(hasAck){
				read_inputs();
				trigger_out();
				sendBinTrap();
			}
		}else{
			hasAck = false;
			ESP.wdtDisable();

#if DEBUG
	zn.debugL("->hasAck=false \n\r");
#endif
		}
		lastConnectionTime = millis();
	}

}


void setup() {
  // put your setup code here, to run once:
  Serial.begin(115200);

  //in seconds
  //zn.setTimeout(180);

  if(digitalRead(0)){
	  readConf();
  }

  if(!zn.autoConnect("TinoviAP",dataReceived,cack,ack)) {
    Serial.println("failed to connect and hit timeout");
    delay(2000);
    //reset and try again, or maybe put it to deep sleep
    ESP.reset();
    delay(5000);
  }
  //out_pins[OUT_COUNT]
  for(int i=0;i<OUT_COUNT;i++){
	  pinMode(out_pins[i],OUTPUT);
  }
  read_inputs();
  trigger_out();

  //if you get here you have connected to the WiFi
  Serial.println("connected...yeey :)");

}

int main(void) {

	init();
	setup();

	while (true) {
		loop();
	}
}
