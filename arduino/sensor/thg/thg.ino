#include <avr/wdt.h>
#include <avr/power.h>
#include "LowPower.h"
#include <AltSoftSerial.h>
#include <EEPROM.h>
#include <Wire.h>
#include "ClosedCube_SHT31D.h"
#include <CayenneLPP.h>
//#include <stdlib.h>

const uint8_t CCS_ALR_PIN   = 0;   // PD2/RXD1/INT2
const uint8_t SHT_ALR_PIN   = 1;   // PD3/TXD1/INT3
const uint8_t RAK_RES_PIN   = 4;   // PD4/ADC8
const uint8_t DOUT_PIN      = 6;   // PD7/ADC10
const uint8_t CCS_WAKE_PIN  = 8;   // PB4/ADC11/PCINT4
const uint8_t PER_RES_PIN   = 9;   // PB5/ADC12/PCINT5
const uint8_t LED_PIN       = 10;  // PB6/ADC13/PCINT6
const uint8_t ALT_TX_PIN    = 5;   // PC6
const uint8_t ALT_RX_PIN    = 13;  // PC7
const uint8_t BAT_PIN       = A0;  // PF7/ADC7
const uint8_t BAT_ON_PIN    = A1;  // PF6/ADC6
const uint8_t CCS_SUP_PIN   = A2;  // PF5/ADC5
const uint8_t SHT_SUP_PIN   = A3;  // PF4/ADC4
const uint8_t VEXT_PIN      = A4;  // PF1/ADC1
const uint8_t USB_PIN       = A5;  // PF0/ADC0

const uint8_t supOnDly = 1, batSampDly = 1, batSampNum = 3;
uint16_t minuteRead, minuteSend;
volatile bool isExtInt;
bool isBatAlarm, isBatLowPrev, isPowerUp;
const unsigned long wdtMs30000 = 30000, wdtMs100 = 100;

struct Conf {
  uint16_t read_period;
  uint16_t send_period;
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
AltSoftSerial rakSerial;
ClosedCube_SHT31D sht;
CayenneLPP lpp(51);

void setup() {
  setPins();
  setPeripheral();
  analogReference(INTERNAL);
  EEPROM.get(0, conf);    
  Serial.begin(115200);  
  rakSerial.begin(9600);
  flashLed3(); 
  delay(5000);
  if (digitalRead(USB_PIN) == HIGH) {
    setUsb();
  } 
  setSht();  
  readAll();
  atRakClrSerial();
  atRakJoinOtaa();  
  uplink(); 
}
void loop() {   
  for (uint16_t ii = 0; ii < 8 ; ii++) {   
    sleepAndWake();
    if (isExtInt) {
      isExtInt = false;
      readAll();   
      uplink();      
    }            
  }    
  minuteRead++;
  minuteSend++;  
  if (minuteRead >= conf.read_period) {
    minuteRead = 0;    
    readAll();
    if (isBatAlarm) {
      uplink();      
    }    
  }    
  if (minuteSend >= conf.send_period) {
    uplink();
  }    
}
void sleepAndWake() {
  attachInterrupt(digitalPinToInterrupt(SHT_ALR_PIN), wakeUpSht, CHANGE);
  attachInterrupt(digitalPinToInterrupt(CCS_ALR_PIN), wakeUpCcs, CHANGE);
  LowPower.powerDown(SLEEP_8S, ADC_OFF, BOD_OFF); ///??????????????????????????????? BOD_OFF
  //isExtInt = INTF2 || INTF3;  
  detachInterrupt(digitalPinToInterrupt(SHT_ALR_PIN));
  detachInterrupt(digitalPinToInterrupt(CCS_ALR_PIN));  
}
void uplink() {
  minuteRead = 0;
  minuteSend = 0;  
  SHT31D result = sht.periodicFetchData();
  lpp.reset();
  lpp.addDigitalInput(0, isBatLowPrev); 
  if (conf.temp_en) {
    lpp.addTemperature(1, result.t);
  }
  if (conf.hum_en) {
    lpp.addRelativeHumidity(2, result.rh);
  } 
  lpp.addDigitalOutput(20, digitalRead(DOUT_PIN));   
  if (!isPowerUp) {
    isPowerUp = true;
    lpp.addAnalogOutput(30, 0);    
  }
  atRakWake();
  atRakSend(lpp.getBuffer());  
  atRakSleep();  
}
void readAll() {
  wdt_enable(WDTO_8S);
  wdt_reset();
  checkBat();  
}
void checkBat() {
  power_adc_enable();  
  digitalWrite(BAT_ON_PIN, LOW);
  delay(supOnDly);
  uint16_t samples[batSampNum];
  uint8_t i;
  for (i = 0; i < batSampNum; i++) {
    samples[i] = analogRead(BAT_PIN);
    delay(batSampDly);
  } 
  digitalWrite(BAT_ON_PIN, HIGH); 
  power_adc_disable(); 
  float batteryVolt = 0;
  for (i = 0; i < batSampNum; i++) {
    batteryVolt += samples[i];
  }
  batteryVolt /= batSampNum;   
  batteryVolt = ( batteryVolt / 1023 ) * 3.6;
  bool isBatLow; 
  if (batteryVolt <= conf.bat_lo_v) {
    isBatLow = true;   
  } else {
    isBatLow = false;
  }
  if (isBatLow != isBatLowPrev) {
    isBatAlarm = true;  
  } else {
    isBatAlarm = false;
  }
  isBatLowPrev = isBatLow;
}
void setPins() {
  pinMode(CCS_ALR_PIN, INPUT);
  pinMode(SHT_ALR_PIN, INPUT);
  pinMode(RAK_RES_PIN, OUTPUT);
  pinMode(DOUT_PIN, OUTPUT);
  pinMode(CCS_WAKE_PIN, OUTPUT);
  pinMode(PER_RES_PIN, OUTPUT);
  pinMode(LED_PIN, OUTPUT);
  pinMode(BAT_PIN, INPUT);
  pinMode(BAT_ON_PIN, OUTPUT);
  pinMode(CCS_SUP_PIN, OUTPUT);
  pinMode(SHT_SUP_PIN, OUTPUT);
  pinMode(VEXT_PIN, INPUT);
  pinMode(USB_PIN, INPUT);  
  digitalWrite(RAK_RES_PIN, HIGH);
  digitalWrite(DOUT_PIN, LOW);
  digitalWrite(CCS_WAKE_PIN, HIGH);
  digitalWrite(PER_RES_PIN, HIGH);
  digitalWrite(LED_PIN, HIGH);
  digitalWrite(BAT_ON_PIN, HIGH);
  digitalWrite(CCS_SUP_PIN, LOW);
  digitalWrite(SHT_SUP_PIN, LOW);  
}
void setPeripheral() {
  digitalWrite(RAK_RES_PIN, LOW);
  digitalWrite(PER_RES_PIN, LOW); 
  delay(10);
  digitalWrite(RAK_RES_PIN, HIGH);
  digitalWrite(PER_RES_PIN, HIGH); 
}
void setSht() {
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
    if (Serial.available()) {
      const char chrUsb = (char)Serial.read();
      str += chrUsb;
      if (chrUsb == '\n') {
        str.trim();       
        if (str.startsWith(F("at"))) {       
          rakSerial.println(str);
        } else if (str.startsWith(F("read_period"))) {
          if (str.indexOf(F("=")) >= 0) {
            str.replace(F("read_period="), "");
            conf.read_period = str.toInt();
            EEPROM.put(0, conf);
            Serial.println(F("OK"));
          } else {
            Serial.print(F("OK"));
            Serial.println(conf.read_period);
          } 
        } else if (str.startsWith(F("send_period"))) {
          if (str.indexOf(F("=")) >= 0) {
            str.replace(F("send_period="), "");
            conf.send_period = str.toInt();
            EEPROM.put(0, conf);
            Serial.println(F("OK"));
          } else {
            Serial.print(F("OK"));
            Serial.println(conf.send_period);
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
        } else if (str.startsWith(F("tmp_alr_hi_set"))) {
          if (str.indexOf(F("=")) >= 0) {
            str.replace(F("tmp_alr_hi_set="), "");
            conf.tmp_alr_hi_set = str.toFloat();
            EEPROM.put(0, conf);
            Serial.println(F("OK"));
          } else {
            Serial.print(F("OK"));
            Serial.println(conf.tmp_alr_hi_set);
          }   
        } else if (str.startsWith(F("tmp_alr_hi_clr"))) {
          if (str.indexOf(F("=")) >= 0) {
            str.replace(F("tmp_alr_hi_clr="), "");
            conf.tmp_alr_hi_clr = str.toFloat();
            EEPROM.put(0, conf);
            Serial.println(F("OK"));
          } else {
            Serial.print(F("OK"));
            Serial.println(conf.tmp_alr_hi_clr);
          } 
        } else if (str.startsWith(F("tmp_alr_lo_set"))) {
          if (str.indexOf(F("=")) >= 0) {
            str.replace(F("tmp_alr_lo_set="), "");
            conf.tmp_alr_lo_set = str.toFloat();
            EEPROM.put(0, conf);
            Serial.println(F("OK"));
          } else {
            Serial.print(F("OK"));
            Serial.println(conf.tmp_alr_lo_set);
          } 
        } else if (str.startsWith(F("tmp_alr_lo_clr"))) {
          if (str.indexOf(F("=")) >= 0) {
            str.replace(F("tmp_alr_lo_clr="), "");
            conf.tmp_alr_lo_clr = str.toFloat();
            EEPROM.put(0, conf);
            Serial.println(F("OK"));
          } else {
            Serial.print(F("OK"));
            Serial.println(conf.tmp_alr_lo_clr);
          }         
        } else if (str.startsWith(F("hum_alr_hi_set"))) {
          if (str.indexOf(F("=")) >= 0) {
            str.replace(F("hum_alr_hi_set="), "");
            conf.hum_alr_hi_set = str.toFloat();
            EEPROM.put(0, conf);
            Serial.println(F("OK"));
          } else {
            Serial.print(F("OK"));
            Serial.println(conf.hum_alr_hi_set);
          } 
        } else if (str.startsWith(F("hum_alr_hi_clr"))) {
          if (str.indexOf(F("=")) >= 0) {
            str.replace(F("hum_alr_hi_clr="), "");
            conf.hum_alr_hi_clr = str.toFloat();
            EEPROM.put(0, conf);
            Serial.println(F("OK"));
          } else {
            Serial.print(F("OK"));
            Serial.println(conf.hum_alr_hi_clr);
          }  
        } else if (str.startsWith(F("hum_alr_lo_set"))) {
          if (str.indexOf(F("=")) >= 0) {
            str.replace(F("hum_alr_lo_set="), "");
            conf.hum_alr_lo_set = str.toFloat();
            EEPROM.put(0, conf);
            Serial.println(F("OK"));
          } else {
            Serial.print(F("OK"));
            Serial.println(conf.hum_alr_lo_set);
          }   
        } else if (str.startsWith(F("hum_alr_lo_clr"))) {
          if (str.indexOf(F("=")) >= 0) {
            str.replace(F("hum_alr_lo_clr="), "");
            conf.hum_alr_lo_clr = str.toFloat();
            EEPROM.put(0, conf);
            Serial.println(F("OK"));
          } else {
            Serial.print(F("OK"));
            Serial.println(conf.hum_alr_lo_clr);
          }  
        } else if (str.startsWith(F("temp_en"))) {
          if (str.indexOf(F("=")) >= 0) {
            str.replace(F("temp_en="), "");
            conf.temp_en = str.toInt();
            EEPROM.put(0, conf);
            Serial.println(F("OK"));
          } else {
            Serial.print(F("OK"));
            Serial.println(conf.temp_en);
          }  
        } else if (str.startsWith(F("hum_en"))) {
          if (str.indexOf(F("=")) >= 0) {
            str.replace(F("hum_en="), "");
            conf.hum_en = str.toInt();
            EEPROM.put(0, conf);
            Serial.println(F("OK"));
          } else {
            Serial.print(F("OK"));
            Serial.println(conf.hum_en);
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
void atRakClrSerial() {
  String str;
  while (rakSerial.available()) {
    const char inChar = (char)rakSerial.read();
  }
  rakSerial.println(F("at"));  
  str = RakReadLine(wdtMs30000);  
}
void atRakJoinOtaa() {  
  rakSerial.println(F("at+join=otaa"));
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
  flashLed();
  str = "at+send=0,1," + str; 
  rakSerial.println(str);
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
      conf.read_period = confValue;
      EEPROM.put(0, conf);
    } else if (confKey == 2) {
      conf.send_period = confValue;
      EEPROM.put(0, conf);  
    } else if (confKey == 3) {
      conf.bat_lo_v = confValue;
      EEPROM.put(0, conf);
    } else if (confKey == 4) {
      conf.tmp_alr_hi_set = confValue;
      EEPROM.put(0, conf);
    } else if (confKey == 5) {
      conf.tmp_alr_hi_clr = confValue;
      EEPROM.put(0, conf);
    } else if (confKey == 6) {
      conf.tmp_alr_lo_set = confValue;
      EEPROM.put(0, conf);
    } else if (confKey == 7) {
      conf.tmp_alr_lo_clr = confValue;
      EEPROM.put(0, conf);
    } else if (confKey == 8) {
      conf.hum_alr_hi_set = confValue;
      EEPROM.put(0, conf);
    } else if (confKey == 9) {
      conf.hum_alr_hi_clr = confValue;
      EEPROM.put(0, conf);  
    } else if (confKey == 10) {
      conf.hum_alr_lo_set = confValue;
      EEPROM.put(0, conf);
    } else if (confKey == 11) {
      conf.hum_alr_lo_clr = confValue;
      EEPROM.put(0, conf); 
    } else if (confKey == 12) {
      conf.temp_en = confValue;
      EEPROM.put(0, conf);
    } else if (confKey == 13) {
      conf.hum_en = confValue;
      EEPROM.put(0, conf);   
    } else if (confKey == 99) {
      resetMe();  
    }     
  }  
}
void atRakWake() {  
  rakSerial.println(F("w"));
  String str;  
  str = RakReadLine(wdtMs30000);
  if (!str.endsWith(F("8,0,0"))) {
    resetMe();
  }   
}
void atRakSleep() {  
  rakSerial.println(F("at+sleep"));
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
    while (rakSerial.available()) {
      const char inChar = (char)rakSerial.read();
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
void wakeUpSht() {
  isExtInt = true;   
}
void wakeUpCcs() { 
  isExtInt = true;   
}
void flashLed() {
  digitalWrite(LED_PIN, LOW);
  delay(50);
  digitalWrite(LED_PIN, HIGH);
}
void flashLed3() {
  for (uint8_t i = 0; i < 3; i++) {
    digitalWrite(LED_PIN, LOW);
    delay(50);
    digitalWrite(LED_PIN, HIGH);
    delay(200);
  }  
}
