#include <avr/wdt.h>
#include <avr/power.h>
#include "LowPower.h"
#include <AltSoftSerial.h>
#include <EEPROM.h>
#include <CayenneLPP.h>
#include "ADS1118.h"
#include <SPI.h>
//#include <stdlib.h>

const uint8_t RAK_RES_PIN   = 4;   // PD4/ADC8
const uint8_t LED_PIN       = 10;  // PB6/ADC13/PCINT6
const uint8_t BAT_PIN       = A0;  // PF7/ADC7
const uint8_t BAT_EN_PIN    = A1;  // PF6/ADC6
const uint8_t VREF_EN_PIN   = A2;  // PF5/ADC5
const uint8_t VOUT_EN_PIN   = A3;  // PF4/ADC4
const uint8_t ADS_CS_PIN    = A4;  // PF1/ADC1

float mV[2], Val[2];
const float rangeMv1 = 0, rangeMv2 = 20.644;
const float k1[] PROGMEM = {0.0000000E+0, 2.5173462E+1, -1.1662878E+0, -1.0833638E+0, -8.9773540E-1, -3.7342377E-1, -8.6632643E-2, -1.0450598E-2, -5.1920577E-4}; 
const float k2[] PROGMEM = {0.0000000E+0, 2.5083550E+1, 7.8601060E-2, -2.5031310E-1, 8.3152700E-2, -1.2280340E-2, 9.8040360E-4, -4.4130300E-5, 1.0577340E-6, -1.0527550E-8};
const float k3[] PROGMEM = {-1.3180580E+2, 4.8302220E+1, -1.6460310E+0, 5.4647310E-2, -9.6507150E-4, 8.8021930E-6, -3.1108100E-8};  
bool isAlarm, isValOverPrev[2], isBatLowPrev, isPowerUp;
const uint8_t vrefEnDly = 1, batEnDly = 1, batSampDly = 1, batSampNum = 3;
uint16_t minuteRead, minuteSend;
const unsigned long wdtMs30000 = 30000, wdtMs100 = 100;

struct Conf {
  uint16_t read_t;
  uint16_t send_t;
  float bat_lo_v;  
  uint8_t inp_en[2];
  float val_alr_hi[2];
  float val_alr_lo[2];
  float val_alr_hys[2];   
};

Conf conf;
ADS1118 ads1118(ADS_CS_PIN);
AltSoftSerial rakSerial;
CayenneLPP lpp(51);

void setup() {
  setPins();
  setPeripheral();
  analogReference(INTERNAL);
  loadConf();
  Serial.begin(115200);
  while (!Serial);
  rakSerial.begin(9600); 
  flashLed3();
  delay(5000);
  if (USBSTA >> VBUS & 1) {
    setUsb();
  } 
  setAds();   
  readAll();
  atRakClrSerial();
  atRakJoinOtaa();    
  uplink();     
}
void loop() {  
  for (uint8_t ii = 0; ii < 8 ; ii++) {   
    sleepAndWake();             
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
  LowPower.powerDown(SLEEP_8S, ADC_OFF, BOD_OFF); ///??????????????????????????????? BOD_OFF     
}
void uplink() { 
  isAlarm = false; 
  minuteRead = 0;
  minuteSend = 0;  
  lpp.reset();
  lpp.addDigitalInput(0, isBatLowPrev); 
  for (uint8_t ch = 0; ch < 2 ; ch++) {
    if (conf.inp_en[ch]) {
      lpp.addTemperature(ch + 1, Val[ch]);
    } 
  } 
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
  calcBatAlarm();
  for (uint8_t ch = 0; ch < 2 ; ch++) {
    if (conf.inp_en[ch]) {
      readMv(ch);
      calcT(ch);
      calcValAlarm(ch); 
    }
  }  
}
void readMv(uint8_t ch) {
  digitalWrite(VREF_EN_PIN, LOW);
  delay(vrefEnDly);
  if (ch == 0) {
    mV[0] = ads1118.getMilliVolts(ads1118.DIFF_0_1);
  } else if (ch == 1) {
    mV[1] = ads1118.getMilliVolts(ads1118.DIFF_2_3);
  } 
  digitalWrite(VREF_EN_PIN, HIGH);  
}
void calcT(uint8_t ch) {     
  if (mV[ch] < rangeMv1) {
    Val[ch] = pgm_read_float(&k1[0]) + mV[ch]*(pgm_read_float(&k1[1]) + mV[ch]*(pgm_read_float(&k1[2]) + mV[ch]*(pgm_read_float(&k1[3])
      + mV[ch]*(pgm_read_float(&k1[4]) + mV[ch]*(pgm_read_float(&k1[5]) + mV[ch]*(pgm_read_float(&k1[6]) + mV[ch]*(pgm_read_float(&k1[7])
      + mV[ch]*(pgm_read_float(&k1[8])))))))));      
  } else if (mV[ch] > rangeMv2) {
    Val[ch] = pgm_read_float(&k3[0]) + mV[ch]*(pgm_read_float(&k3[1]) + mV[ch]*(pgm_read_float(&k3[2]) + mV[ch]*(pgm_read_float(&k3[3])
      + mV[ch]*(pgm_read_float(&k3[4]) + mV[ch]*(pgm_read_float(&k3[5]) + mV[ch]*(pgm_read_float(&k3[6])))))));
  } else {
    Val[ch] = pgm_read_float(&k2[0]) + mV[ch]*(pgm_read_float(&k2[1]) + mV[ch]*(pgm_read_float(&k2[2]) + mV[ch]*(pgm_read_float(&k2[3])
      + mV[ch]*(pgm_read_float(&k2[4]) + mV[ch]*(pgm_read_float(&k2[5]) + mV[ch]*(pgm_read_float(&k2[6]) + mV[ch]*(pgm_read_float(&k2[7])
      + mV[ch]*(pgm_read_float(&k2[8]) + mV[ch]*(pgm_read_float(&k2[9]))))))))));
  } 
  Val[ch] = Val[ch] + ads1118.getTemperature();    
}
void calcValAlarm(uint8_t ch) {  
  bool isValOver; 
  if ((Val[ch] >= conf.val_alr_hi[ch] + conf.val_alr_hys[ch]) || (Val[ch] <= conf.val_alr_lo[ch] - conf.val_alr_hys[ch])) {
    isValOver = true;    
  } else if ((Val[ch] <= conf.val_alr_hi[ch] - conf.val_alr_hys[ch]) || (Val[ch] >= conf.val_alr_lo[ch] + conf.val_alr_hys[ch])) {
    isValOver = false;
  }
  if (isValOver != isValOverPrev[ch]) {
    isValOverPrev[ch] = isValOver;
    isAlarm = true;          
  }    
} 
void calcBatAlarm() {
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
  float batteryVolt = 0;
  for (ii = 0; ii < batSampNum; ii++) {
    batteryVolt += samples[ii];
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
    isBatLowPrev = isBatLow;
    isAlarm = true;  
  }  
}
void setPins() {
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
  if (conf.send_t < 5) {
    conf.send_t = 5;
  }  
}  
void setAds() {
  ads1118.begin();
  //ads1118.setSampligRate(ads1118.RATE_64SPS);
  ads1118.setFullScaleRange(ads1118.FSR_0256);
  ads1118.disablePullup(); 
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
        } else if (str.startsWith(F("inp_en_1"))) {
          if (str.indexOf(F("=")) >= 0) {
            str.replace(F("inp_en_1="), "");
            conf.inp_en[0] = str.toInt();
            EEPROM.put(0, conf);
            Serial.println(F("OK"));
          } else {
            Serial.print(F("OK"));
            Serial.println(conf.inp_en[0]);
          } 
        } else if (str.startsWith(F("inp_en_2"))) {
          if (str.indexOf(F("=")) >= 0) {
            str.replace(F("inp_en_2="), "");
            conf.inp_en[1] = str.toInt();
            EEPROM.put(0, conf);
            Serial.println(F("OK"));
          } else {
            Serial.print(F("OK"));
            Serial.println(conf.inp_en[1]);
          }
        } else if (str.startsWith(F("val_alr_hi_1"))) {
          if (str.indexOf(F("=")) >= 0) {
            str.replace(F("val_alr_hi_1="), "");
            conf.val_alr_hi[0] = str.toFloat();
            EEPROM.put(0, conf);
            Serial.println(F("OK"));
          } else {
            Serial.print(F("OK"));
            Serial.println(conf.val_alr_hi[0]);
          }
        } else if (str.startsWith(F("val_alr_hi_2"))) {
          if (str.indexOf(F("=")) >= 0) {
            str.replace(F("val_alr_hi_2="), "");
            conf.val_alr_hi[1] = str.toFloat();
            EEPROM.put(0, conf);
            Serial.println(F("OK"));
          } else {
            Serial.print(F("OK"));
            Serial.println(conf.val_alr_hi[1]);
          } 
        } else if (str.startsWith(F("val_alr_lo_1"))) {
          if (str.indexOf(F("=")) >= 0) {
            str.replace(F("val_alr_lo_1="), "");
            conf.val_alr_lo[0] = str.toFloat();
            EEPROM.put(0, conf);
            Serial.println(F("OK"));
          } else {
            Serial.print(F("OK"));
            Serial.println(conf.val_alr_lo[0]);
          }
        } else if (str.startsWith(F("val_alr_lo_2"))) {
          if (str.indexOf(F("=")) >= 0) {
            str.replace(F("val_alr_lo_2="), "");
            conf.val_alr_lo[1] = str.toFloat();
            EEPROM.put(0, conf);
            Serial.println(F("OK"));
          } else {
            Serial.print(F("OK"));
            Serial.println(conf.val_alr_lo[1]);
          }
        } else if (str.startsWith(F("val_alr_hys_1"))) {
          if (str.indexOf(F("=")) >= 0) {
            str.replace(F("val_alr_hys_1="), "");
            conf.val_alr_hys[0] = str.toFloat();
            EEPROM.put(0, conf);
            Serial.println(F("OK"));
          } else {
            Serial.print(F("OK"));
            Serial.println(conf.val_alr_hys[0]);
          } 
        } else if (str.startsWith(F("val_alr_hys_2"))) {
          if (str.indexOf(F("=")) >= 0) {
            str.replace(F("val_alr_hys_2="), "");
            conf.val_alr_hys[1] = str.toFloat();
            EEPROM.put(0, conf);
            Serial.println(F("OK"));
          } else {
            Serial.print(F("OK"));
            Serial.println(conf.val_alr_hys[1]);
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
      conf.read_t = confValue;      
    } else if (confKey == 2) {
      conf.send_t = confValue;       
    } else if (confKey == 3) {
      conf.bat_lo_v = confValue;      
    } else if (confKey == 4) {
      conf.inp_en[0] = confValue;      
    } else if (confKey == 5) {
      conf.inp_en[1] = confValue;      
    } else if (confKey == 6) {
      conf.val_alr_hi[0] = confValue;      
    } else if (confKey == 7) {
      conf.val_alr_hi[1] = confValue;      
    } else if (confKey == 8) {
      conf.val_alr_lo[0] = confValue;      
    } else if (confKey == 9) {
      conf.val_alr_lo[1] = confValue;
    } else if (confKey == 10) {
      conf.val_alr_hys[0] = confValue;
    } else if (confKey == 11) {
      conf.val_alr_hys[1] = confValue;                  
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
void flashLed() {
  digitalWrite(LED_PIN, LOW);
  delay(50);
  digitalWrite(LED_PIN, HIGH);
}
void flashLed3() {
  for (uint8_t ii = 0; ii < 3; ii++) {
    digitalWrite(LED_PIN, LOW);
    delay(50);
    digitalWrite(LED_PIN, HIGH);
    delay(200);
  }  
}
