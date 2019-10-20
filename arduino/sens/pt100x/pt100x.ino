#include <avr/wdt.h>
#include <avr/power.h>
#include "LowPower.h"
#include <AltSoftSerial.h>
#include <EEPROM.h>
#include <CayenneLPP.h>
#include "ADS1118.h"
#include <SPI.h>
//#include <stdlib.h>

const uint8_t DIG_PIN[2]    = {3, 2}; // PD0/SCL/INT0, PD1/SDA/INT1
const uint8_t RAK_RES_PIN   = 4;   // PD4/ADC8
const uint8_t LED_PIN       = 10;  // PB6/ADC13/PCINT6
const uint8_t BAT_PIN       = A0;  // PF7/ADC7
const uint8_t BAT_EN_PIN    = A1;  // PF6/ADC6
const uint8_t VREF_EN_PIN   = A2;  // PF5/ADC5
const uint8_t VOUT_EN_PIN   = A3;  // PF4/ADC4
const uint8_t ADS_CS_PIN    = A4;  // PF1/ADC1

float In, R, Val[2], ValPrev[2], BatVolt, BatVoltPrev;
const float rtd_coeff = 0.3851, rtd_r0 = 100, extRef = 2.5, r_ext = 2400;
volatile bool isAlarm;
bool isPowerUp;
const uint8_t vrefEnDly = 1, digDly = 10, batEnDly = 1, batSampDly = 1, batSampNum = 3;
uint16_t minuteRead, minuteSend;
const long wdtMs30000 = 30000, wdtMs100 = 100;

struct Conf {
  uint16_t read_t = 1;
  uint16_t send_t = 2;
  float bat_lo_v = 1.1;       
  float alr_max[2] = {50, 50};
  float alr_min[2] = {10, 10};
  float alr_hys[2] = {0.01, 0.01};  
  uint8_t an_type[2] = {1, 1};
  uint8_t dig_type[2] = {0, 0};  
};

Conf conf;
ADS1118 ads1118(ADS_CS_PIN);
AltSoftSerial rakSerial;
CayenneLPP lpp(51);

void setup() {
  setPins();
  //setPeripheral();
  analogReference(INTERNAL);
  //loadConf();  
  rakSerial.begin(9600); 
  //Serial.begin(115200);
  //while (!Serial);
  flashLed3();  
  //if (USBSTA >> VBUS & 1) {
  //  setUsb();
  //}   
  setAds();   
  //readAll();
  atRakClrSerial();
  atRakJoinOtaa();
  atRakSleep(); 
  //delay(1000);   
  //uplink();      
}
void loop() { 
 
  for (uint8_t ii = 0; ii < 8 ; ii++) {   
    sleepAndWake();
    if (isAlarm) {
      delay(digDly);       
      uplink();      
    }              
  }    
  minuteRead++;
  minuteSend++;  
  if (minuteRead >= conf.read_t) {
    minuteRead = 0;    
    readAll();
    if (isAlarm) {      
      uplink();      
    }    
  }    
  if (minuteSend >= conf.send_t) {
    uplink();
  }
    
}
void sleepAndWake() { 
/* 
  for (uint8_t ch = 0; ch < 2 ; ch++) {
    if (conf.dig_type[ch]) {
      attachInterrupt(digitalPinToInterrupt(DIG_PIN[ch]), wakeUp, conf.dig_type[ch]);    
    }    
  }    
  LowPower.powerDown(SLEEP_8S, ADC_OFF, BOD_OFF); ///??????????????????????????????? BOD_OFF     
  for (uint8_t ch = 0; ch < 2 ; ch++) {
    if (conf.dig_type[ch]) {
      detachInterrupt(digitalPinToInterrupt(DIG_PIN[ch])); 
    }    
  }
*/
  LowPower.powerDown(SLEEP_8S, ADC_OFF, BOD_OFF); ///??????????????????????????????? BOD_OFF        
}
void uplink() {
  isAlarm = false;   
  minuteRead = 0;
  minuteSend = 0;  
  lpp.reset();
  lpp.addAnalogInput(0, BatVolt); 
  for (uint8_t ch = 0; ch < 2 ; ch++) {
    if (conf.an_type[ch]) {
      lpp.addTemperature(ch + 1, Val[ch]);
      Serial.println(Val[ch]);
    } 
  } 
  for (uint8_t ch = 0; ch < 2 ; ch++) {
    if (conf.dig_type[ch]) {
      lpp.addDigitalInput(ch + 11, digitalRead(DIG_PIN[ch]));
    } 
  } 
  if (!isPowerUp) {
    isPowerUp = true;
    lpp.addAnalogOutput(30, 0);    
  }  
  //Serial.println(lppGetBuffer());    
  atRakWake();
  atRakSend(lppGetBuffer());  
  atRakSleep();  
}
void readAll() {
  //wdt_enable(WDTO_8S);
  //wdt_reset();
  readBatVolt();
  //calcBatAlarm();
  for (uint8_t ch = 0; ch < 2 ; ch++) {
    if (conf.an_type[ch]) {
      adjAds(ch);
      readIn();
      calcR();
      calcVal(ch);
      //calcValAlarm(ch); 
    }
  }  
}
void readIn() {
  digitalWrite(VREF_EN_PIN, LOW);
  delay(vrefEnDly);  
  In = ads1118.getMilliVolts();  
  digitalWrite(VREF_EN_PIN, HIGH); 
  In /= 1000; // mV / 1000 = volt 
}
void calcR() {
  R = (In * r_ext) / (extRef - In);  
}
void calcVal(const uint8_t ch) {     
  Val[ch] = (R - rtd_r0) / rtd_coeff;    
}
void calcValAlarm(const uint8_t ch) {     
  if (Val[ch] <= conf.alr_min[ch] - conf.alr_min[ch] * conf.alr_hys[ch]) {
    if (ValPrev[ch] >= conf.alr_min[ch] + conf.alr_min[ch] * conf.alr_hys[ch]) {
      isAlarm = true; 
    }
  }
  if (Val[ch] >= conf.alr_min[ch] + conf.alr_min[ch] * conf.alr_hys[ch]) {
    if (ValPrev[ch] <= conf.alr_min[ch] - conf.alr_min[ch] * conf.alr_hys[ch]) {
      isAlarm = true; 
    }
  }
  if (Val[ch] >= conf.alr_max[ch] + conf.alr_max[ch] * conf.alr_hys[ch]) {
    if (ValPrev[ch] <= conf.alr_max[ch] - conf.alr_max[ch] * conf.alr_hys[ch]) {
      isAlarm = true; 
    }
  }
  if (Val[ch] <= conf.alr_max[ch] - conf.alr_max[ch] * conf.alr_hys[ch]) {
    if (ValPrev[ch] >= conf.alr_max[ch] + conf.alr_max[ch] * conf.alr_hys[ch]) {
      isAlarm = true; 
    }
  }
  ValPrev[ch] = Val[ch];  
} 
void readBatVolt() {
  power_adc_enable();  
  digitalWrite(BAT_EN_PIN, LOW);
  delay(batEnDly);
  uint16_t samples[batSampNum];
  uint8_t ii;
  for (ii = 0; ii < batSampNum; ii++) {
    samples[ii] = analogRead(BAT_PIN);
    delay(batSampDly);
  } 
  digitalWrite(BAT_EN_PIN, HIGH); 
  power_adc_disable(); 
  BatVolt = 0;
  for (ii = 0; ii < batSampNum; ii++) {
    BatVolt += samples[ii];
  }
  BatVolt /= batSampNum;   
  BatVolt = ( BatVolt / 1023 ) * 3.6;  
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
void setPins() {
  for (uint8_t ch = 0; ch < 2 ; ch++) {
    pinMode(DIG_PIN[ch], INPUT);
  }  
  pinMode(RAK_RES_PIN, OUTPUT);
  pinMode(LED_PIN, OUTPUT);  
  pinMode(BAT_PIN, INPUT);
  pinMode(BAT_EN_PIN, OUTPUT);
  pinMode(VREF_EN_PIN, OUTPUT);
  pinMode(VOUT_EN_PIN, OUTPUT);
  pinMode(ADS_CS_PIN, OUTPUT);
  
  digitalWrite(RAK_RES_PIN, HIGH);
  digitalWrite(LED_PIN, HIGH);
  digitalWrite(BAT_EN_PIN, HIGH);
  digitalWrite(VREF_EN_PIN, HIGH);
  digitalWrite(VOUT_EN_PIN, LOW);
  digitalWrite(ADS_CS_PIN, HIGH);
}
void setPeripheral() {
  digitalWrite(RAK_RES_PIN, LOW);
  delay(10);
  digitalWrite(RAK_RES_PIN, HIGH);
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
void setAds() {
  ads1118.begin();
  //ads1118.setSampligRate(ads1118.RATE_64SPS);  
  ads1118.disablePullup(); 
  ads1118.setFullScaleRange(ads1118.FSR_0256); 
}
void adjAds(const uint8_t ch) {
  if (ch == 0) {
    ads1118.setInputSelected(ads1118.DIFF_0_1);
  } else if (ch == 1) {
    ads1118.setInputSelected(ads1118.DIFF_2_3);
  }     
}
void setUsb() {
  Serial.begin(115200);
  while (!Serial);
  String str;
  while (true) {   
    if (Serial.available()) {
      const char chrUsb = (char)Serial.read();
      str += chrUsb;
      if (chrUsb == '\n') {
        str.trim();       
        if (str.startsWith(F("at"))) {       
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
        } else if (str.startsWith(F("an_type_1"))) {
          if (str.indexOf(F("=")) >= 0) {
            str.replace(F("an_type_1="), "");
            conf.an_type[0] = str.toInt();
            EEPROM.put(0, conf);
            Serial.println(F("OK"));
          } else {
            Serial.print(F("OK"));
            Serial.println(conf.an_type[0]);
          }   
        } else if (str.startsWith(F("an_type_2"))) {
          if (str.indexOf(F("=")) >= 0) {
            str.replace(F("an_type_2="), "");
            conf.an_type[1] = str.toInt();
            EEPROM.put(0, conf);
            Serial.println(F("OK"));
          } else {
            Serial.print(F("OK"));
            Serial.println(conf.an_type[1]);
          }        
        } else if (str.startsWith(F("dig_type_1"))) {
          if (str.indexOf(F("=")) >= 0) {
            str.replace(F("dig_type_1="), "");
            conf.dig_type[0] = str.toInt();
            EEPROM.put(0, conf);
            Serial.println(F("OK"));
          } else {
            Serial.print(F("OK"));
            Serial.println(conf.dig_type[0]);
          }                  
        } else if (str.startsWith(F("dig_type_2"))) {
          if (str.indexOf(F("=")) >= 0) {
            str.replace(F("dig_type_2="), "");
            conf.dig_type[1] = str.toInt();
            EEPROM.put(0, conf);
            Serial.println(F("OK"));
          } else {
            Serial.print(F("OK"));
            Serial.println(conf.dig_type[1]);
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
  while (rakSerial.available()) {
    const char inChar = (char)rakSerial.read();
  }  
}
void atRakJoinOtaa() {
  digitalWrite(LED_PIN, LOW);  
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
  digitalWrite(LED_PIN, HIGH);
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
    } else if (confKey == 10) {
      conf.an_type[0] = confValue;
    } else if (confKey == 11) {
      conf.an_type[1] = confValue;         
    } else if (confKey == 12) {
      conf.dig_type[0] = confValue;      
    } else if (confKey == 13) {
      conf.dig_type[1] = confValue;  
    }
    EEPROM.put(0, conf);    
    resetMe();  
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
String RakReadLine(const long wdtMs) { 
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
    //wdt_reset();
  }
  str = "";
  if (wdtMs == wdtMs30000) {    
    resetMe();
  }
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
void resetMe() {
  //wdt_enable(WDTO_15MS);
  //while(true); 
}
void wakeUp() {
  isAlarm = true;   
}
void flashLed() {
  digitalWrite(LED_PIN, LOW);
  delay(100);
  digitalWrite(LED_PIN, HIGH);
}
void flashLed3() {
  for (uint8_t ii = 0; ii < 3; ii++) {
    digitalWrite(LED_PIN, LOW);
    delay(100);
    digitalWrite(LED_PIN, HIGH);
    delay(200);
  }  
}
