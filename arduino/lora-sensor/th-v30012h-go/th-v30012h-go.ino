#include <avr/wdt.h>
#include <avr/power.h>
#include "LowPower.h"
#include <AltSoftSerial.h>
#include <EEPROM.h>
#include <CayenneLPP.h>
#include <Wire.h>
#include "ClosedCube_SHT31D.h"

const uint8_t SHT_ALR_PIN   = 0;  // PD2/RXD1/INT2
const uint8_t SHT_RES_PIN   = 1;  // PD3/TXD1/INT3
const uint8_t RAK_RES_PIN   = 4;   // PD4/ADC8
const uint8_t LED_PIN       = 10;  // PB6/ADC13/PCINT6
const uint8_t BAT_PIN       = A0;  // PF7/ADC7
const uint8_t BAT_EN_PIN    = A1;  // PF6/ADC6
const uint8_t VREF_EN_PIN   = A2;  // PF5/ADC5

float BatVolt, BatVoltPrev;
volatile bool isAlarm = false;
bool isPowerUp;
const uint8_t batEnDly = 1, batSampDly = 1, batSampNum = 3;
const uint8_t atWake = 1, atSleep = 2, atJoin = 3, atSend = 4, atDr2 = 5;
uint16_t minuteRead, minuteSend;
const long tmrSec120 = 120000, tmrSec10 = 10000, tmrMsec100 = 100;

struct Conf {
  uint16_t read_t;
  uint16_t send_t;
  float bat_lo_v;       
  float alr_max[2];
  float alr_min[2];
  float alr_hys[2];    
};

Conf conf;
ClosedCube_SHT31D sht;
AltSoftSerial rakSerial;
CayenneLPP lpp(51);

void setup() {  
  setPins();
  rakSerial.begin(9600);
  //resSht();
  analogReference(INTERNAL);
  loadConf();  
  flashLed();
  //setSht();    
  //if (USBSTA >> VBUS & 1) {
  //  setUsb();
  //}

  Serial.begin(115200);
  while (!Serial);
    
  readAll();
  if (rakJoin()) {
    Serial.println("join true");
    if (!rakDr2()) {
      resetMe();     
    }    
    if (!rakSleep()) {      
      resetMe();
    }    
    uplink();         
  } else {
    Serial.println("join false");
    if (!rakDr2()) {
      resetMe();     
    }       
    if (!rakSleep()) {          
      resetMe();
    }    
  }   
}
void loop() {  
  for (uint8_t slpCnt = 0; slpCnt < 8 ; slpCnt++) {   
    sleepAndWake();
    if (isAlarm) {
      readAll();       
      uplink();
      return;      
    }              
  }    
  //minuteRead++;
  minuteSend++;  
  //if (minuteRead >= conf.read_t) {
  //  minuteRead = 0;    
  //  readAll();
  //  if (isAlarm) {      
  //    uplink();
  //    return;      
   // }    
  //}    
  if (minuteSend >= conf.send_t) {
    //
    readAll();
    //
    uplink();
    return;
  }    
}
void sleepAndWake() { 
  EIFR = 255;  
  attachInterrupt(digitalPinToInterrupt(SHT_ALR_PIN), wakeUp, CHANGE);  
  isAlarm = false; 
  LowPower.powerDown(SLEEP_8S, ADC_OFF, BOD_OFF); ///??????????????????????????????? BOD_OFF
  detachInterrupt(digitalPinToInterrupt(SHT_ALR_PIN));
}
void uplink() {
  isAlarm = false;   
  minuteRead = 0;
  minuteSend = 0;  
  //SHT31D result = sht.periodicFetchData();  
  lpp.reset();
  //lpp.addAnalogInput(0, BatVolt); 
  //lpp.addTemperature(1, result.t);
  //lpp.addRelativeHumidity(2, result.rh);
  lpp.addTemperature(1, 66);
  lpp.addRelativeHumidity(2, 77);
  //if (!isPowerUp) {
  //  isPowerUp = true;
  //  lpp.addAnalogOutput(30, 0);    
  //}
  if (!rakWake()) {
    resetMe();
  }
  if (!rakSend(lppGetBuffer())) {
    resetMe();
  }
  if (!rakSleep()) {
    resetMe();    
  }
}
void readAll() {  
  readBatVolt();
  //calcBatAlarm();  
}
void readBatVolt() {
  power_adc_enable();  
  digitalWrite(BAT_EN_PIN, LOW);
  delay(batEnDly);
  uint16_t samples[batSampNum];
  for (uint8_t ii = 0; ii < batSampNum; ii++) {
    samples[ii] = analogRead(BAT_PIN);
    delay(batSampDly);
  } 
  digitalWrite(BAT_EN_PIN, HIGH); 
  power_adc_disable(); 
  BatVolt = 0;
  for (uint8_t jj = 0; jj < batSampNum; jj++) {
    BatVolt += samples[jj];
  }
  BatVolt /= batSampNum;   
  BatVolt = ( BatVolt / 1023 ) * 2.56 * 2;  
}
void calcBatAlarm() {
  if (BatVolt <= conf.bat_lo_v) {
    if (BatVoltPrev > conf.bat_lo_v) {
      isAlarm = true;
    }
  } else {
    if (BatVoltPrev <= conf.bat_lo_v) {
      isAlarm = true;
    }
  }
  BatVoltPrev = BatVolt;
}
void setSht() {
  Wire.begin();
  sht.begin(0x44);  
  sht.periodicStart(SHT3XD_REPEATABILITY_LOW, SHT3XD_FREQUENCY_1HZ);
  sht.writeAlertHigh(conf.alr_max[0] + conf.alr_max[0] * conf.alr_hys[0], conf.alr_max[0] - conf.alr_max[0] * conf.alr_hys[0], conf.alr_max[1] + conf.alr_max[1] * conf.alr_hys[1], conf.alr_max[1] - conf.alr_max[1] * conf.alr_hys[1]);
  sht.writeAlertLow(conf.alr_min[0] + conf.alr_min[0] * conf.alr_hys[0], conf.alr_min[0] - conf.alr_min[0] * conf.alr_hys[0],  conf.alr_min[1] + conf.alr_min[1] * conf.alr_hys[1], conf.alr_min[1] - conf.alr_min[1] * conf.alr_hys[1]);
  sht.clearAll();     
}
void loadConf() {
  EEPROM.get(0, conf);
  if (conf.read_t < 1) {
    conf.read_t = 1; 
  }
  //if (conf.send_t < 5) {
  //  conf.send_t = 5;
  //}  
}
void rakClear() {  
  delay(1000);  
  while (rakSerial.available()) {
    const char inChar = (char)rakSerial.read();           
  }    
}
bool rakWake() {
  rakClear();
  rakSerial.println(F("at+set_config=device:sleep:0"));
  return rakResponse(atWake, tmrSec10); 
}
bool rakSleep() {
  rakClear();
  rakSerial.println(F("at+set_config=device:sleep:1"));
  return rakResponse(atSleep, tmrSec10); 
}
bool rakJoin() {
  delay(100);
  digitalWrite(RAK_RES_PIN, HIGH);
  return rakResponse(atJoin, tmrSec120);  
}
bool rakSend(String str) {  
  str = "at+send=lora:1:" + str;
  rakClear();   
  rakSerial.println(str);  
  return rakResponse(atSend, tmrSec10);    
}
bool rakDr2() {
  rakClear();   
  rakSerial.println(F("at+set_config=lora:dr:2"));  
  return rakResponse(atDr2, tmrSec10);
}
bool rakResponse(const uint8_t atCommand, const long atTmr) {
  digitalWrite(LED_PIN, LOW);
  String str;
  unsigned long startMs = millis();  
  while (millis() - startMs < atTmr) {    
    while (rakSerial.available()) {
      const char inChar = (char)rakSerial.read();
      str += inChar;
      if (inChar == '\n') {
        str.trim();
        if (atCommand == atWake) {
          if (str.equalsIgnoreCase(F("Wake up"))) {
            Serial.println("Wake up");
            digitalWrite(LED_PIN, HIGH);
            return true;
          }          
        } else if (atCommand == atSleep) {          
          if (str.equalsIgnoreCase(F("Go to Sleep"))) { 
            Serial.println("Go to Sleep");           
            digitalWrite(LED_PIN, HIGH);
            return true;
          }          
        } else if (atCommand == atJoin) {          
          if (str.equalsIgnoreCase(F("[LoRa]:Join Success"))) {
            Serial.println("Join success");
            digitalWrite(LED_PIN, HIGH);
            return true;
          } else if (str.equalsIgnoreCase(F("ERROR: RUI_AT_LORA_INFO_STATUS_JOIN_FAIL 99"))) {            
            Serial.println("Join fail");
            digitalWrite(LED_PIN, HIGH);
            return false;
          }                   
        } else if (atCommand == atSend) {
          if (str.equalsIgnoreCase(F("[LoRa]: RUI_MCPS_UNCONFIRMED send success"))) {
            Serial.println("Send success");
            digitalWrite(LED_PIN, HIGH);
            return true;
          } else if (str.equalsIgnoreCase(F("ERROR: RUI_LORA_STATUS_NO_NETWORK_JOINED 86"))) {
            Serial.println("Send fail");
            digitalWrite(LED_PIN, HIGH);
            return false;       
          }
        } else if (atCommand == atDr2) {          
          if (str.equalsIgnoreCase(F("LoRa configure DR2 success"))) {
            Serial.println("DR2 success");            
            digitalWrite(LED_PIN, HIGH);
            return true;
          }            
        }
        Serial.println("------");
        Serial.println(str);
        str = "";        
      }
    }        
  }  
  digitalWrite(LED_PIN, HIGH);  
  return false;  
}
String lppGetBuffer() {
  String str;
  for(uint8_t ii = 0; ii < lpp.getSize(); ii++){    
    if (lpp.getBuffer()[ii] < 16) {
      str += '0';       
    }
    str += String(lpp.getBuffer()[ii], HEX);
    str.toUpperCase();        
  }
  return str;
}
/* 
  str = RakReadLine(tmrMsec100);
  if (str.endsWith(F("ff"))) {
    str.replace("ff", "");
    const uint8_t i = str.lastIndexOf(',');
    str = str.substring(i + 1);    
    lppDownlinkDec(str);
    EEPROM.put(0, conf);
  }
*/    
void lppDownlinkDec(String str) {
  char buf[6];
  str.substring(0, 2).toCharArray(buf, sizeof(buf));
  const uint8_t downCh = strtol(buf, NULL, 0);
  str.substring(2).toCharArray(buf, sizeof(buf));
  str = strtol(buf, NULL, 0);
  const uint8_t confKey = str.substring(str.length() - 2).toInt();
  const uint16_t confValue = str.substring(0, str.length() - 2).toInt();
  const uint8_t digOutValue = strtol(buf, NULL, 0);
  if (downCh == 30) {    
    if (confKey == 1) {
      conf.read_t = confValue;      
    } else if (confKey == 2) {
      conf.send_t = confValue;       
    } else if (confKey == 3) {
      conf.bat_lo_v = confValue;          
    } else if (confKey == 4) {
      conf.alr_max[0] = confValue;      
    } else if (confKey == 5) {
      conf.alr_max[1] = confValue;      
    } else if (confKey == 6) {
      conf.alr_min[0] = confValue;      
    } else if (confKey == 7) {
      conf.alr_min[1] = confValue;
    } else if (confKey == 8) {
      conf.alr_hys[0] = confValue;
    } else if (confKey == 9) {
      conf.alr_hys[1] = confValue;     
    }
    EEPROM.put(0, conf);    
    resetMe();  
  }  
}
void setPins() {
  pinMode(SHT_ALR_PIN, INPUT);
  pinMode(SHT_RES_PIN, OUTPUT); 
  pinMode(RAK_RES_PIN, OUTPUT);
  pinMode(LED_PIN, OUTPUT);  
  pinMode(BAT_PIN, INPUT);
  pinMode(BAT_EN_PIN, OUTPUT);
  pinMode(VREF_EN_PIN, OUTPUT);    
  digitalWrite(SHT_RES_PIN, HIGH);
  digitalWrite(RAK_RES_PIN, LOW);
  digitalWrite(LED_PIN, HIGH);
  digitalWrite(BAT_EN_PIN, HIGH);
  digitalWrite(VREF_EN_PIN, LOW);  
}
void resSht() {
  delay(100);
  digitalWrite(SHT_RES_PIN, HIGH);
}
void setUsb() {
  Serial.begin(115200);
  while (!Serial); 
  delay(100);
  digitalWrite(RAK_RES_PIN, HIGH); 
  String str;
  while (true) {   
    if (Serial.available()) {
      const char chrUsb = (char)Serial.read();
      str += chrUsb;
      if (chrUsb == '\n') {
        str.trim();       
        if (str.startsWith(F("at"))) {
          rakClear();///////////////////////////////////////       
          rakSerial.println(str);
        } else if (str.startsWith(F("read_t"))) {
          if (str.indexOf(F("=")) >= 0) {
            str.replace(F("read_t="), "");
            conf.read_t = str.toInt();
            EEPROM.put(0, conf);
            Serial.println(F("OK"));
          } else {
            Serial.print(F("OK"));
            Serial.println(conf.read_t);
          } 
        } else if (str.startsWith(F("send_t"))) {
          if (str.indexOf(F("=")) >= 0) {
            str.replace(F("send_t="), "");
            conf.send_t = str.toInt();
            EEPROM.put(0, conf);
            Serial.println(F("OK"));
          } else {
            Serial.print(F("OK"));
            Serial.println(conf.send_t);
          }             
        } else if (str.startsWith(F("bat_lo_v"))) {
          if (str.indexOf(F("=")) >= 0) {
            str.replace(F("bat_lo_v="), "");
            conf.bat_lo_v = str.toFloat();
            EEPROM.put(0, conf);
            Serial.println(F("OK"));
          } else {
            Serial.print(F("OK"));
            Serial.println(conf.bat_lo_v);
          }          
        } else if (str.startsWith(F("alr_max_1"))) {
          if (str.indexOf(F("=")) >= 0) {
            str.replace(F("alr_max_1="), "");
            conf.alr_max[0] = str.toFloat();
            EEPROM.put(0, conf);
            Serial.println(F("OK"));
          } else {
            Serial.print(F("OK"));
            Serial.println(conf.alr_max[0]);
          }
        } else if (str.startsWith(F("alr_max_2"))) {
          if (str.indexOf(F("=")) >= 0) {
            str.replace(F("alr_max_2="), "");
            conf.alr_max[1] = str.toFloat();
            EEPROM.put(0, conf);
            Serial.println(F("OK"));
          } else {
            Serial.print(F("OK"));
            Serial.println(conf.alr_max[1]);
          } 
        } else if (str.startsWith(F("alr_min_1"))) {
          if (str.indexOf(F("=")) >= 0) {
            str.replace(F("alr_min_1="), "");
            conf.alr_min[0] = str.toFloat();
            EEPROM.put(0, conf);
            Serial.println(F("OK"));
          } else {
            Serial.print(F("OK"));
            Serial.println(conf.alr_min[0]);
          }
        } else if (str.startsWith(F("alr_min_2"))) {
          if (str.indexOf(F("=")) >= 0) {
            str.replace(F("alr_min_2="), "");
            conf.alr_min[1] = str.toFloat();
            EEPROM.put(0, conf);
            Serial.println(F("OK"));
          } else {
            Serial.print(F("OK"));
            Serial.println(conf.alr_min[1]);
          }
        } else if (str.startsWith(F("alr_hys_1"))) {
          if (str.indexOf(F("=")) >= 0) {
            str.replace(F("alr_hys_1="), "");
            conf.alr_hys[0] = str.toFloat();
            EEPROM.put(0, conf);
            Serial.println(F("OK"));
          } else {
            Serial.print(F("OK"));
            Serial.println(conf.alr_hys[0]);
          } 
        } else if (str.startsWith(F("alr_hys_2"))) {
          if (str.indexOf(F("=")) >= 0) {
            str.replace(F("alr_hys_2="), "");
            conf.alr_hys[1] = str.toFloat();
            EEPROM.put(0, conf);
            Serial.println(F("OK"));
          } else {
            Serial.print(F("OK"));
            Serial.println(conf.alr_hys[1]);
          }                            
        }
        str = "";        
      }      
    }
    if (rakSerial.available()) {
      const char chrSerial = (char)rakSerial.read();   
      Serial.print(chrSerial);   
    }      
  }    
}
void flashLed() {
  for (uint8_t ii = 0; ii < 5; ii++) {  
    digitalWrite(LED_PIN, LOW);
    delay(100);
    digitalWrite(LED_PIN, HIGH);
    delay(100);
  }
}
void wakeUp() {
  isAlarm = true;   
}
void resetMe() {
  wdt_enable(WDTO_15MS);
  while(true); 
}
