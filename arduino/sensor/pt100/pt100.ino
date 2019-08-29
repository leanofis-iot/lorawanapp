#include <avr/wdt.h>
#include <avr/power.h>
#include "LowPower.h"
#include <AltSoftSerial.h>
#include <EEPROM.h>
#include <CayenneLPP.h>
#include "ADS1118.h"
#include <SPI.h>
//#include <stdlib.h>

const uint8_t ADS_CS_PIN    = 7;   // PE6/AIN0/INT6
const uint8_t DIN_PIN       = 3;   // PD0/SCL/INT0
const uint8_t RAK_RES_PIN   = 4;   // PD4/ADC8
const uint8_t DOUT_PIN      = 6;   // PD7/ADC10
const uint8_t LED_PIN       = 10;  // PB6/ADC13/PCINT6
const uint8_t ALT_TX_PIN    = 5;   // PC6
const uint8_t ALT_RX_PIN    = 13;  // PC7
const uint8_t BAT_PIN       = A0;  // PF7/ADC7
const uint8_t BAT_ON_PIN    = A1;  // PF6/ADC6
const uint8_t DSUP_PIN      = A2;  // PF5/ADC5
const uint8_t ASUP_PIN      = A3;  // PF4/ADC4
const uint8_t VEXT_PIN      = A4;  // PF1/ADC1
const uint8_t USB_PIN       = A5;  // PF0/ADC0

const uint8_t change = 1, rising = 2, falling = 3;
const uint8_t digDebounce = 10;
float anVolt, temperature;
const float extRef = 2048, ptCo = 0.3851, ptR0 = 100, ptRext = 2000;
bool isTempAlarm, isTempAlarmPrev, isBat, isBatStatePrev, isPowerUp;
volatile bool isExtInt;
const uint8_t supOnDly = 1, batSampDly = 1, batSampNum = 3;
uint16_t minuteRead, minuteSend;
const unsigned long wdtMs30000 = 30000, wdtMs100 = 100;

struct Conf {
  uint16_t read_period;
  uint16_t send_period;
  float bat_lo_v;
  float tmp_alr_hi_set;
  float tmp_alr_hi_clr;
  float tmp_alr_lo_set;
  float tmp_alr_lo_clr;
  uint8_t dig;  
};

Conf conf;
ADS1118 ads1118(ADS_CS_PIN);
AltSoftSerial rakSerial;
CayenneLPP lpp(51);

void setup() {
  setPins();
  setPeripheral();
  EEPROM.get(0, conf);
  Serial.begin(115200);  
  rakSerial.begin(9600); 
  flashLed3();
  delay(5000);
  if (digitalRead(USB_PIN) == HIGH) {
    setUsb();
  }
  setAds();   
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
    if (isTempAlarm || isBat) {
      uplink();      
    }    
  }    
  if (minuteSend >= conf.send_period) {
    uplink();
  }    
}
void sleepAndWake() {
  if (conf.dig == 1) {
    attachInterrupt(digitalPinToInterrupt(DIN_PIN), wakeUp, CHANGE);
  } else if (conf.dig == 2) {
    attachInterrupt(digitalPinToInterrupt(DIN_PIN), wakeUp, RISING);
  } else if (conf.dig == 3) {
    attachInterrupt(digitalPinToInterrupt(DIN_PIN), wakeUp, FALLING);
  }  
  LowPower.powerDown(SLEEP_8S, ADC_OFF, BOD_OFF); ///??????????????????????????????? BOD_OFF
  //isExtInt = INTF2 || INTF3;  
  detachInterrupt(digitalPinToInterrupt(DIN_PIN));   
}
void uplink() {  
  minuteRead = 0;
  minuteSend = 0;  
  lpp.reset();
  lpp.addDigitalInput(0, isBatStatePrev); 
  lpp.addTemperature(1, temperature);
  delay(digDebounce);
  lpp.addDigitalInput(10, digitalRead(DIN_PIN));
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
  readAds();
  calcTemperature();
  //checkTempAlarm();  
}
void readAds() {
  digitalWrite(ASUP_PIN, LOW);
  delay(supOnDly);
  anVolt = ads1118.getMilliVolts();
  digitalWrite(ASUP_PIN, HIGH);  
}
void calcTemperature() {
  const float ptR = ( anVolt * ptRext ) / ( extRef - anVolt );
  temperature = ( ptR - ptR0 ) / ptCo;  
}
/*
void checkTempAlarm() {  
  const bool isAnAlarmLog = isAnAlarmFlag; 
  if (analogVolt >= conf.an_alr_hi_set || analogVolt <= conf.an_alr_lo_set) {
    isAnAlarmFlag = true;    
  } else if (analogVolt <= conf.an_alr_hi_clr || analogVolt >= conf.an_alr_lo_clr) {
    isAnAlarmFlag = false;
  }
  if (isAnAlarmFlag != isAnAlarmLog) {
    isAnAlarm = true;  
  } else {
    isAnAlarm = false;
  }
} 
 */

void checkBat() {
  power_adc_enable();
  analogReference(INTERNAL);
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
  bool isBatState; 
  if (batteryVolt <= conf.bat_lo_v) {
    isBatState = true;   
  } else {
    isBatState = false;
  }
  if (isBatState != isBatStatePrev) {
    isBat = true;  
  } else {
    isBat = false;
  }
  isBatStatePrev = isBatState;
}
void setPins() {
  pinMode(ADS_CS_PIN, OUTPUT);
  pinMode(DIN_PIN, INPUT);
  pinMode(RAK_RES_PIN, OUTPUT);
  pinMode(DOUT_PIN, OUTPUT);
  pinMode(LED_PIN, OUTPUT);
  pinMode(BAT_PIN, INPUT);
  pinMode(BAT_ON_PIN, OUTPUT);
  pinMode(DSUP_PIN, OUTPUT);
  pinMode(ASUP_PIN, OUTPUT);
  pinMode(VEXT_PIN, INPUT);
  pinMode(USB_PIN, INPUT);  
  digitalWrite(ADS_CS_PIN, HIGH);
  digitalWrite(RAK_RES_PIN, HIGH);
  digitalWrite(DOUT_PIN, LOW);
  digitalWrite(LED_PIN, HIGH);
  digitalWrite(BAT_ON_PIN, HIGH);
  digitalWrite(DSUP_PIN, HIGH);
  digitalWrite(ASUP_PIN, HIGH);
}
void setPeripheral() {
  digitalWrite(RAK_RES_PIN, LOW);
  delay(10);
  digitalWrite(RAK_RES_PIN, HIGH);
}
void setAds() {
  ads1118.begin();
  //ads1118.setSampligRate(ads1118.RATE_64SPS);
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
        } else if (str.startsWith(F("dig"))) {
          if (str.indexOf(F("=")) >= 0) {
            str.replace(F("dig="), "");
            conf.dig = str.toInt();
            EEPROM.put(0, conf);
            Serial.println(F("OK"));
          } else {
            Serial.print(F("OK"));
            Serial.println(conf.dig);
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
      conf.dig = confValue;
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
void wakeUp() {
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
