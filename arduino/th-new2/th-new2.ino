#include <avr/wdt.h>
#include <avr/power.h>
#include "LowPower.h"
#include <AltSoftSerial.h>
#include <EEPROM.h>
#include <Wire.h>
#include "ClosedCube_SHT31D.h"
#include <CayenneLPP.h>
//#include <stdlib.h>

float batteryVolt;
const uint16_t voutOnTime = 1;
const uint8_t anSampNum = 3, anSampDly = 1;
bool isBatLow, isBatLowFlag, isExtInt, isPowerUp = true;
const unsigned long wdtMs30000 = 30000;
const unsigned long wdtMs100 = 100; 

const uint8_t VOUT_CNT_PIN  = 7;   // PD7/AIN1/PCINT23
const uint8_t BAT_PIN       = 14;  // PC0/ADC0/PCINT8
const uint8_t BAT_CNT_PIN   = 5;   // PD5/PCINT21
const uint8_t USB_RX_PIN    = 8;   // PB0/PCINT0           AltSoftSerial RX
const uint8_t USB_TX_PIN    = 9;   // PB1/PCINT1           AltSoftSerial TX
const uint8_t USB_PIN       = 6;   // PD6/AIN0/PCINT22
const uint8_t RAK_RX_PIN    = 0;   // PD0/RXD/PCINT16      Hardware Serial RX
const uint8_t RAK_TX_PIN    = 1;   // PD1/TXD/PCINT17      Hardware Serial TX
const uint8_t RAK_RES_PIN   = 15;  // PC1/ADC1/PCINT9
const uint8_t SHT_SDA_PIN   = 18;  // PC4/ADC4/SDA/PCINT12 Hardware I2C SDA
const uint8_t SHT_SCL_PIN   = 19;  // PC5/ADC5/SCL/PCINT13 Hardware I2C SCL
const uint8_t SHT_RES_PIN   = 3;   // PD3/INT1/PCINT19
const uint8_t SHT_ALR_PIN   = 2;   // PD2/INT0/PCINT18

struct Conf {
  uint16_t period;
  float bat_lo_v;  
  float tmp_alr_hi_set;
  float tmp_alr_hi_clr;
  float tmp_alr_lo_set;
  float tmp_alr_lo_clr;
  float hum_alr_hi_set;
  float hum_alr_hi_clr;
  float hum_alr_lo_set;
  float hum_alr_lo_clr;
  uint8_t temp_en;
  uint8_t hum_en;  
};

Conf conf;
AltSoftSerial usbSerial;
ClosedCube_SHT31D sht;
CayenneLPP lpp(51);

void setup() {
  setPins();
  EEPROM.get(0, conf);
  setPeripheral();  
  Serial.begin(115200);  
  usbSerial.begin(9600); 
  delay(3000);
  if (digitalRead(USB_PIN) == HIGH) {
    setUsb();
  } 
  setupSht();  
  wdt_enable(WDTO_8S);
  atRakJoinOtaa();
  readAll();  
  uplink(sht.periodicFetchData()); 
}
void loop() {  
  for (uint8_t ii = 0; ii < (conf.period * 7.5); ii++) {
    attachInterrupt(digitalPinToInterrupt(SHT_ALR_PIN), wakeUp, CHANGE);
    LowPower.powerDown(SLEEP_8S, ADC_OFF, BOD_OFF);
    wdt_enable(WDTO_8S);
    readAll();     
    if (isExtInt || isBatLow) {
      uplink(sht.periodicFetchData());      
    }
  }   
  uplink(sht.periodicFetchData());  
}
void readAll() {
  checkExtInt();
  checkBatLow();
}
void checkExtInt() {
  //if (PCIF2 || INTF0) {
  isExtInt = PCIF2;  
  detachInterrupt(digitalPinToInterrupt(SHT_ALR_PIN));
}
void checkBatLow() {
  power_adc_enable();
  analogReference(INTERNAL);
  digitalWrite(BAT_CNT_PIN, LOW);
  delay(voutOnTime);
  uint16_t samples[anSampNum];
  uint8_t i;
  for (i=0; i< anSampNum; i++) {
    samples[i] = analogRead(BAT_PIN);
    delay(anSampDly);
  } 
  digitalWrite(BAT_CNT_PIN, HIGH); 
  power_adc_disable(); 
  batteryVolt = 0;
  for (i=0; i< anSampNum; i++) {
    batteryVolt += samples[i];
  }
  batteryVolt /= anSampNum;   
  batteryVolt = ( batteryVolt / 1023 ) * 3.6;
  const bool isBatLowLog = isBatLowFlag; 
  if (batteryVolt <= conf.bat_lo_v) {
    isBatLowFlag = true;   
  } else {
    isBatLowFlag = false;
  }
  if (isBatLowFlag != isBatLowLog) {
    isBatLow = true;  
  } else {
    isBatLow = false;
  }
}
void uplink(SHT31D result) {
  lpp.reset();
  if (conf.temp_en) {
    lpp.addTemperature(1, result.t);
  }
  if (conf.hum_en) {
    lpp.addRelativeHumidity(2, result.rh);
  }  
  lpp.addAnalogInput(20, batteryVolt); 
  if (isPowerUp) {
    isPowerUp = false;
    lpp.addAnalogOutput(30, 0);    
  }
  atRakWake();
  atRakSend(lpp.getBuffer());  
  atRakSleep();  
}
void setPins() {
  pinMode(VOUT_CNT_PIN, OUTPUT);
  pinMode(BAT_PIN, INPUT);
  pinMode(BAT_CNT_PIN, OUTPUT);
  pinMode(USB_PIN, INPUT);
  pinMode(RAK_RES_PIN, OUTPUT);
  pinMode(SHT_RES_PIN, OUTPUT);
  pinMode(SHT_ALR_PIN, INPUT);  
  digitalWrite(VOUT_CNT_PIN, LOW);
  digitalWrite(BAT_CNT_PIN, HIGH);
  digitalWrite(RAK_RES_PIN, HIGH);
  digitalWrite(SHT_RES_PIN, HIGH); 
}
void setPeripheral() {
  digitalWrite(RAK_RES_PIN, LOW);
  digitalWrite(SHT_RES_PIN, LOW); 
  delay(10);
  digitalWrite(RAK_RES_PIN, HIGH);
  digitalWrite(SHT_RES_PIN, HIGH); 
}
void setupSht() {
  Wire.begin();
  sht.begin(0x44);
  sht.clearAll();
  sht.periodicStart(SHT3XD_REPEATABILITY_LOW, SHT3XD_FREQUENCY_1HZ);
  sht.writeAlertHigh(conf.tmp_alr_hi_set, conf.tmp_alr_hi_clr, conf.hum_alr_hi_set, conf.hum_alr_hi_clr);
  sht.writeAlertLow(conf.tmp_alr_lo_clr, conf.tmp_alr_lo_set, conf.hum_alr_lo_clr, conf.hum_alr_lo_set); 
}
void setUsb() {
  String str;
  while (true) {   
    if (usbSerial.available()) {
      const char chrUsb = (char)usbSerial.read();
      str += chrUsb;
      if (chrUsb == '\n') {
        str.trim();       
        if (str.startsWith(F("at"))) {       
          Serial.println(str);
        } else if (str.startsWith(F("period"))) {
          if (str.indexOf(F("=")) >= 0) {
            str.replace(F("period="), "");
            conf.period = str.toInt();
            EEPROM.put(0, conf);
            usbSerial.println(F("OK"));
          } else {
            usbSerial.print(F("OK"));
            usbSerial.println(conf.period);
          }       
        } else if (str.startsWith(F("bat_lo_v"))) {
          if (str.indexOf(F("=")) >= 0) {
            str.replace(F("bat_lo_v="), "");
            conf.bat_lo_v = str.toFloat();
            EEPROM.put(0, conf);
            usbSerial.println(F("OK"));
          } else {
            usbSerial.print(F("OK"));
            usbSerial.println(conf.bat_lo_v);
          }  
        } else if (str.startsWith(F("tmp_alr_hi_set"))) {
          if (str.indexOf(F("=")) >= 0) {
            str.replace(F("tmp_alr_hi_set="), "");
            conf.tmp_alr_hi_set = str.toFloat();
            EEPROM.put(0, conf);
            usbSerial.println(F("OK"));
          } else {
            usbSerial.print(F("OK"));
            usbSerial.println(conf.tmp_alr_hi_set);
          }   
        } else if (str.startsWith(F("tmp_alr_hi_clr"))) {
          if (str.indexOf(F("=")) >= 0) {
            str.replace(F("tmp_alr_hi_clr="), "");
            conf.tmp_alr_hi_clr = str.toFloat();
            EEPROM.put(0, conf);
            usbSerial.println(F("OK"));
          } else {
            usbSerial.print(F("OK"));
            usbSerial.println(conf.tmp_alr_hi_clr);
          } 
        } else if (str.startsWith(F("tmp_alr_lo_set"))) {
          if (str.indexOf(F("=")) >= 0) {
            str.replace(F("tmp_alr_lo_set="), "");
            conf.tmp_alr_lo_set = str.toFloat();
            EEPROM.put(0, conf);
            usbSerial.println(F("OK"));
          } else {
            usbSerial.print(F("OK"));
            usbSerial.println(conf.tmp_alr_lo_set);
          } 
        } else if (str.startsWith(F("tmp_alr_lo_clr"))) {
          if (str.indexOf(F("=")) >= 0) {
            str.replace(F("tmp_alr_lo_clr="), "");
            conf.tmp_alr_lo_clr = str.toFloat();
            EEPROM.put(0, conf);
            usbSerial.println(F("OK"));
          } else {
            usbSerial.print(F("OK"));
            usbSerial.println(conf.tmp_alr_lo_clr);
          }         
        } else if (str.startsWith(F("hum_alr_hi_set"))) {
          if (str.indexOf(F("=")) >= 0) {
            str.replace(F("hum_alr_hi_set="), "");
            conf.hum_alr_hi_set = str.toFloat();
            EEPROM.put(0, conf);
            usbSerial.println(F("OK"));
          } else {
            usbSerial.print(F("OK"));
            usbSerial.println(conf.hum_alr_hi_set);
          } 
        } else if (str.startsWith(F("hum_alr_hi_clr"))) {
          if (str.indexOf(F("=")) >= 0) {
            str.replace(F("hum_alr_hi_clr="), "");
            conf.hum_alr_hi_clr = str.toFloat();
            EEPROM.put(0, conf);
            usbSerial.println(F("OK"));
          } else {
            usbSerial.print(F("OK"));
            usbSerial.println(conf.hum_alr_hi_clr);
          }  
        } else if (str.startsWith(F("hum_alr_lo_set"))) {
          if (str.indexOf(F("=")) >= 0) {
            str.replace(F("hum_alr_lo_set="), "");
            conf.hum_alr_lo_set = str.toFloat();
            EEPROM.put(0, conf);
            usbSerial.println(F("OK"));
          } else {
            usbSerial.print(F("OK"));
            usbSerial.println(conf.hum_alr_lo_set);
          }   
        } else if (str.startsWith(F("hum_alr_lo_clr"))) {
          if (str.indexOf(F("=")) >= 0) {
            str.replace(F("hum_alr_lo_clr="), "");
            conf.hum_alr_lo_clr = str.toFloat();
            EEPROM.put(0, conf);
            usbSerial.println(F("OK"));
          } else {
            usbSerial.print(F("OK"));
            usbSerial.println(conf.hum_alr_lo_clr);
          }  
        } else if (str.startsWith(F("temp_en"))) {
          if (str.indexOf(F("=")) >= 0) {
            str.replace(F("temp_en="), "");
            conf.temp_en = str.toInt();
            EEPROM.put(0, conf);
            usbSerial.println(F("OK"));
          } else {
            usbSerial.print(F("OK"));
            usbSerial.println(conf.temp_en);
          }  
        } else if (str.startsWith(F("hum_en"))) {
          if (str.indexOf(F("=")) >= 0) {
            str.replace(F("hum_en="), "");
            conf.hum_en = str.toInt();
            EEPROM.put(0, conf);
            usbSerial.println(F("OK"));
          } else {
            usbSerial.print(F("OK"));
            usbSerial.println(conf.hum_en);
          }                    
        }
        str = "";        
      }      
    }
    if (Serial.available()) {
      const char chrSerial = (char)Serial.read();   
      usbSerial.print(chrSerial);   
    }      
  }    
}
void atRakJoinOtaa() {  
  Serial.println(F("at+join=otaa"));
  String str;
  str = RakReadLine(wdtMs30000);
  if (!str.endsWith(F("OK"))) {
    resetMe();
  }
  str = RakReadLine(wdtMs30000);
  if (!str.endsWith(F("3,0,0"))) {
    resetMe();
  } 
}
void atRakSend(String str) { 
  str = "at+send=0,2," + str; 
  Serial.println(str);
  str = RakReadLine(wdtMs30000);
  if (!str.endsWith(F("OK"))) {
    resetMe();
  }
  str = RakReadLine(wdtMs30000);
  if (!str.endsWith(F("2,0,0"))) {
    resetMe();
  } 
  str = RakReadLine(wdtMs100);
  if (str.endsWith(F("ff"))) {
    str.replace("ff", "");
    const uint8_t i = str.lastIndexOf(',');
    str = str.substring(i + 1);    
    lppDownlinkDec(str);
    EEPROM.put(0, conf);
  }     
}
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
      conf.period = confValue;
      EEPROM.put(0, conf);  
    } else if (confKey == 2) {
      conf.bat_lo_v = confValue;
      EEPROM.put(0, conf);
    } else if (confKey == 3) {
      conf.tmp_alr_hi_set = confValue;
      EEPROM.put(0, conf);
    } else if (confKey == 4) {
      conf.tmp_alr_hi_clr = confValue;
      EEPROM.put(0, conf);
    } else if (confKey == 5) {
      conf.tmp_alr_lo_set = confValue;
      EEPROM.put(0, conf);
    } else if (confKey == 6) {
      conf.tmp_alr_lo_clr = confValue;
      EEPROM.put(0, conf);
    } else if (confKey == 7) {
      conf.hum_alr_hi_set = confValue;
      EEPROM.put(0, conf);
    } else if (confKey == 8) {
      conf.hum_alr_hi_clr = confValue;
      EEPROM.put(0, conf);  
    } else if (confKey == 9) {
      conf.hum_alr_lo_set = confValue;
      EEPROM.put(0, conf);
    } else if (confKey == 10) {
      conf.hum_alr_lo_clr = confValue;
      EEPROM.put(0, conf); 
    } else if (confKey == 11) {
      conf.temp_en = confValue;
      EEPROM.put(0, conf);
    } else if (confKey == 12) {
      conf.hum_en = confValue;
      EEPROM.put(0, conf);   
    } else if (confKey == 99) {
      resetMe();  
    }     
  }  
}
void atRakWake() {  
  Serial.println(F("w"));
  String str;  
  str = RakReadLine(wdtMs30000);
  if (!str.endsWith(F("8,0,0"))) {
    resetMe();
  }   
}
void atRakSleep() {  
  Serial.println(F("at+sleep"));
  String str;
  str = RakReadLine(wdtMs30000);
  if (!str.endsWith(F("OK"))) {
    resetMe();
  }
}
String RakReadLine(const unsigned long wdtMs) {  
  String str;
  unsigned long startMs = millis();
  while ((unsigned long)(millis() - startMs) < wdtMs) {    
    while (Serial.available()) {
      const char inChar = (char)Serial.read();
      str += inChar;
      if (inChar == '\n') {
        str.trim(); 
        return str;
      }
    }
    wdt_reset();
  }
  str = "";
  if (wdtMs == wdtMs30000) {
    resetMe();
  }     
}
void resetMe() {
  wdt_enable(WDTO_15MS);
  while(true); 
}
void wakeUp() {   
}
