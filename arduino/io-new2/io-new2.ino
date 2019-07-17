#include <AltSoftSerial.h>
#include <EEPROM.h>
#include <avr/wdt.h>
#include <avr/power.h>
#include "LowPower.h"
#include <CayenneLPP.h>
//#include <stdlib.h>

const uint8_t VOUT_CNT_PIN  = 7;   // PD7/AIN1/PCINT23
const uint8_t BAT_PIN       = 14;  // PC0/ADC0/PCINT8
const uint8_t BAT_CNT_PIN   = 5;   // PD5/PCINT21
const uint8_t USB_RX_PIN    = 8;   // PB0/PCINT0           AltSoftSerial RX
const uint8_t USB_TX_PIN    = 9;   // PB1/PCINT1           AltSoftSerial TX
const uint8_t USB_PIN       = 6;   // PD6/AIN0/PCINT22
const uint8_t RAK_RX_PIN    = 0;   // PD0/RXD/PCINT16      Hardware Serial RX
const uint8_t RAK_TX_PIN    = 1;   // PD1/TXD/PCINT17      Hardware Serial TX
const uint8_t RAK_RES_PIN   = 15;  // PC1/ADC1/PCINT9
const uint8_t IN1A_PIN      = 18;  // PC4/ADC4/SDA/PCINT12 Hardware I2C SDA
const uint8_t IN2A_PIN      = 19;  // PC5/ADC5/SCL/PCINT13 Hardware I2C SCL
const uint8_t IN1D_PIN      = 2;   //PD2/INT0/PCINT18
const uint8_t IN2D_PIN      = 3;   // PD3/INT1/PCINT19
const uint8_t OUT1_PIN      = 16; // PC2/ADC2/PCINT10
const uint8_t OUT2_PIN      = 17; // PC3/ADC3/PCINT11

const uint8_t _an = 0;
const uint8_t _dig = 1;
const uint8_t _3v = 1;
const uint8_t _5v = 2;
const uint8_t _10v = 3;
const uint8_t _ntc = 4;

float analogVolt, batteryVolt;
const uint16_t voutOnTime = 1;
const uint8_t anSampNum = 3, anSampDly = 1, digInDebounce = 10;
bool isAnAlarm, isAnAlarmFlag, isBatLow, isBatLowFlag, isExtInt, isPowerUp = true;
const unsigned long wdtMs30000 = 30000;
const unsigned long wdtMs100 = 100; 

struct Conf {
  uint16_t period;
  float bat_lo_v;
  uint8_t interface;
  uint8_t sensor;
  float ntc_tmp;
  float ntc_res; 
  float ntc_beta; 
  float ntc_ser_res;  
  uint8_t an_alr_en;
  float an_alr_hi_set;
  float an_alr_hi_clr;
  float an_alr_lo_set;
  float an_alr_lo_clr;
  uint8_t dig_alr_hi_en;
  uint8_t dig_alr_lo_en;  
};

Conf conf;
AltSoftSerial usbSerial;
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
  wdt_enable(WDTO_8S);
  atRakJoinOtaa();
  readAll();  
  uplink();     
}
void loop() {  
  for (uint8_t ii = 0; ii < (conf.period * 7.5); ii++) {
    setupAttachInt();    
    LowPower.powerDown(SLEEP_8S, ADC_OFF, BOD_OFF); 
    wdt_enable(WDTO_8S);
    readAll();    
    if (isAnAlarm || isBatLow || isExtInt) {
      uplink(); 
    }    
  }   
  uplink();  
}
void uplink() {  
  lpp.reset();
  if (conf.interface == _an) {  
    if (conf.sensor == _ntc) {      
      lpp.addTemperature(1, analogVolt);
    } else if (conf.sensor == _3v || conf.sensor == _5v || conf.sensor == _10v ) {
      lpp.addAnalogInput(1, analogVolt);
    }
  } else if (conf.interface == _dig) {
    delay(digInDebounce);
    lpp.addDigitalInput(1, digitalRead(IN1D_PIN));
  }
  lpp.addAnalogInput(20, batteryVolt);  
  if (isPowerUp) {
    isPowerUp = false;
    lpp.addAnalogOutput(30, 0);
    lpp.addDigitalOutput(40, digitalRead(OUT1_PIN));
  }
  atRakWake();
  atRakSend(lpp.getBuffer());  
  atRakSleep();  
}
void readAll() {
  if (conf.interface == _dig) {
    checkExtInt();        
  } else if (conf.interface == _an) {  
    readAnalog();
    if (conf.sensor == _ntc) {
      calcNtc();
    } else if (conf.sensor == _3v) {
      calc3v();
    } else if (conf.sensor == _5v) {
      calc5v();
    } else if (conf.sensor == _10v) {
      calc10v();
    }
    if (conf.an_alr_en) {
      checkAnalogAlarm();      
    }
  }   
  checkBatteryLow();  
}
void readAnalog() {
  power_adc_enable();  
  if (conf.sensor == _ntc) {
    analogReference(DEFAULT);
  } else {
    analogReference(INTERNAL);
  }   
  digitalWrite(VOUT_CNT_PIN, LOW);
  delay(voutOnTime);
  uint16_t samples[anSampNum];
  uint8_t i;
  for (i=0; i< anSampNum; i++) {
    samples[i] = analogRead(IN1A_PIN);
    delay(anSampDly);
  } 
  digitalWrite(VOUT_CNT_PIN, HIGH); 
  power_adc_disable(); 
  analogVolt = 0;
  for (i=0; i< anSampNum; i++) {
    analogVolt += samples[i];
  }
  analogVolt /= anSampNum; 
}
void calcNtc() { 
  analogVolt = 1023 / analogVolt - 1;
  analogVolt = conf.ntc_ser_res / analogVolt;  
  analogVolt = analogVolt / conf.ntc_res;     
  analogVolt = log(analogVolt);                  
  analogVolt /= conf.ntc_beta;                   
  analogVolt += 1.0 / (conf.ntc_tmp + 273.15); 
  analogVolt = 1.0 / analogVolt;                 
  analogVolt -= 273.15;   
}
void calc3v() {
  analogVolt = ( analogVolt / 1023 ) * 3;
}
void calc5v() {
  analogVolt = ( analogVolt / 1023 ) * 5;
}
void calc10v() {
  analogVolt = ( analogVolt / 1023 ) * 10;
}
void checkAnalogAlarm() {  
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
void checkBatteryLow() {
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
void setPins() {
  pinMode(VOUT_CNT_PIN, OUTPUT);
  pinMode(BAT_PIN, INPUT);
  pinMode(BAT_CNT_PIN, OUTPUT);
  pinMode(USB_PIN, INPUT);
  pinMode(RAK_RES_PIN, OUTPUT);
  pinMode(IN1A_PIN, INPUT);
  pinMode(IN2A_PIN, INPUT);
  pinMode(IN1D_PIN, INPUT);
  pinMode(IN2D_PIN, INPUT);
  pinMode(OUT1_PIN, OUTPUT);
  pinMode(OUT2_PIN, OUTPUT);  
  digitalWrite(VOUT_CNT_PIN, HIGH);
  digitalWrite(BAT_CNT_PIN, HIGH);
  digitalWrite(RAK_RES_PIN, HIGH);
  digitalWrite(OUT1_PIN, LOW);
  digitalWrite(OUT2_PIN, LOW);
}
void setPeripheral() {
  if (conf.interface == _dig) {
    digitalWrite(VOUT_CNT_PIN, LOW);       
  }
  digitalWrite(RAK_RES_PIN, LOW);
  delay(10);
  digitalWrite(RAK_RES_PIN, HIGH);
}
void setupAttachInt() {
  if (conf.dig_alr_hi_en && conf.dig_alr_lo_en) {
    attachInterrupt(digitalPinToInterrupt(IN1D_PIN), wakeUp, CHANGE);  
  } else if (conf.dig_alr_hi_en) {
    attachInterrupt(digitalPinToInterrupt(IN1D_PIN), wakeUp, RISING);
  } else if (conf.dig_alr_lo_en) {
    attachInterrupt(digitalPinToInterrupt(IN1D_PIN), wakeUp, FALLING);  
  }
}
void checkExtInt() {
  if (PCIF2 || INTF0 || INTF1) {
    isExtInt = true;
  } else {
    isExtInt = false;
  }
  detachInterrupt(digitalPinToInterrupt(IN1D_PIN));
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
        } else if (str.startsWith(F("interface"))) {
          if (str.indexOf(F("=")) >= 0) {
            str.replace(F("interface="), "");
            conf.interface = str.toInt();
            EEPROM.put(0, conf);
            usbSerial.println(F("OK"));
          } else {
            usbSerial.print(F("OK"));
            usbSerial.println(conf.interface);
          }  
        } else if (str.startsWith(F("sensor"))) {
          if (str.indexOf(F("=")) >= 0) {
            str.replace(F("sensor="), "");
            conf.sensor = str.toInt();
            EEPROM.put(0, conf);
            usbSerial.println(F("OK"));
          } else {
            usbSerial.print(F("OK"));
            usbSerial.println(conf.sensor);
          }  
        } else if (str.startsWith(F("ntc_tmp"))) {
          if (str.indexOf(F("=")) >= 0) {
            str.replace(F("ntc_tmp="), "");
            conf.ntc_tmp = str.toFloat();
            EEPROM.put(0, conf);
            usbSerial.println(F("OK"));
          } else {
            usbSerial.print(F("OK"));
            usbSerial.println(conf.ntc_tmp);
          }   
        } else if (str.startsWith(F("ntc_res"))) {
          if (str.indexOf(F("=")) >= 0) {
            str.replace(F("ntc_res="), "");
            conf.ntc_res = str.toFloat();
            EEPROM.put(0, conf);
            usbSerial.println(F("OK"));
          } else {
            usbSerial.print(F("OK"));
            usbSerial.println(conf.ntc_res);
          } 
        } else if (str.startsWith(F("ntc_beta"))) {
          if (str.indexOf(F("=")) >= 0) {
            str.replace(F("ntc_beta="), "");
            conf.ntc_beta = str.toFloat();
            EEPROM.put(0, conf);
            usbSerial.println(F("OK"));
          } else {
            usbSerial.print(F("OK"));
            usbSerial.println(conf.ntc_beta);
          } 
        } else if (str.startsWith(F("ntc_ser_res"))) {
          if (str.indexOf(F("=")) >= 0) {
            str.replace(F("ntc_ser_res="), "");
            conf.ntc_ser_res = str.toFloat();
            EEPROM.put(0, conf);
            usbSerial.println(F("OK"));
          } else {
            usbSerial.print(F("OK"));
            usbSerial.println(conf.ntc_ser_res);
          }      
        } else if (str.startsWith(F("an_alr_en"))) {
          if (str.indexOf(F("=")) >= 0) {
            str.replace(F("an_alr_en="), "");
            conf.an_alr_en = str.toInt();
            EEPROM.put(0, conf);
            usbSerial.println(F("OK"));
          } else {
            usbSerial.print(F("OK"));
            usbSerial.println(conf.an_alr_en);
          }  
        } else if (str.startsWith(F("an_alr_hi_set"))) {
          if (str.indexOf(F("=")) >= 0) {
            str.replace(F("an_alr_hi_set="), "");
            conf.an_alr_hi_set = str.toFloat();
            EEPROM.put(0, conf);
            usbSerial.println(F("OK"));
          } else {
            usbSerial.print(F("OK"));
            usbSerial.println(conf.an_alr_hi_set);
          } 
        } else if (str.startsWith(F("an_alr_hi_clr"))) {
          if (str.indexOf(F("=")) >= 0) {
            str.replace(F("an_alr_hi_clr="), "");
            conf.an_alr_hi_clr = str.toFloat();
            EEPROM.put(0, conf);
            usbSerial.println(F("OK"));
          } else {
            usbSerial.print(F("OK"));
            usbSerial.println(conf.an_alr_hi_clr);
          }  
        } else if (str.startsWith(F("an_alr_lo_set"))) {
          if (str.indexOf(F("=")) >= 0) {
            str.replace(F("an_alr_lo_set="), "");
            conf.an_alr_lo_set = str.toFloat();
            EEPROM.put(0, conf);
            usbSerial.println(F("OK"));
          } else {
            usbSerial.print(F("OK"));
            usbSerial.println(conf.an_alr_lo_set);
          }   
        } else if (str.startsWith(F("an_alr_lo_clr"))) {
          if (str.indexOf(F("=")) >= 0) {
            str.replace(F("an_alr_lo_clr="), "");
            conf.an_alr_lo_clr = str.toFloat();
            EEPROM.put(0, conf);
            usbSerial.println(F("OK"));
          } else {
            usbSerial.print(F("OK"));
            usbSerial.println(conf.an_alr_lo_clr);
          }
        } else if (str.startsWith(F("dig_alr_hi_en"))) {
          if (str.indexOf(F("=")) >= 0) {
            str.replace(F("dig_alr_hi_en="), "");
            conf.dig_alr_hi_en = str.toInt();
            EEPROM.put(0, conf);
            usbSerial.println(F("OK"));
          } else {
            usbSerial.print(F("OK"));
            usbSerial.println(conf.dig_alr_hi_en);
          }   
        } else if (str.startsWith(F("dig_alr_lo_en"))) {
          if (str.indexOf(F("=")) >= 0) {
            str.replace(F("dig_alr_lo_en="), "");
            conf.dig_alr_lo_en = str.toInt();
            EEPROM.put(0, conf);
            usbSerial.println(F("OK"));
          } else {
            usbSerial.print(F("OK"));
            usbSerial.println(conf.dig_alr_lo_en);
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
      conf.interface = confValue;
      EEPROM.put(0, conf);
    } else if (confKey == 4) {
      conf.sensor = confValue;
      EEPROM.put(0, conf);
    } else if (confKey == 5) {
      conf.ntc_tmp = confValue;
      EEPROM.put(0, conf);
    } else if (confKey == 6) {
      conf.ntc_res = confValue;
      EEPROM.put(0, conf);
    } else if (confKey == 7) {
      conf.ntc_beta = confValue;
      EEPROM.put(0, conf);
    } else if (confKey == 8) {
      conf.ntc_ser_res = confValue;
      EEPROM.put(0, conf);  
    } else if (confKey == 9) {
      conf.an_alr_en = confValue;
      EEPROM.put(0, conf);
    } else if (confKey == 10) {
      conf.an_alr_hi_set = confValue;
      EEPROM.put(0, conf);
    } else if (confKey == 11) {
      conf.an_alr_hi_clr = confValue;
      EEPROM.put(0, conf);
    } else if (confKey == 12) {
      conf.an_alr_lo_set = confValue;
      EEPROM.put(0, conf);
    } else if (confKey == 13) {
      conf.an_alr_lo_clr = confValue;
      EEPROM.put(0, conf);
    } else if (confKey == 14) {
      conf.dig_alr_hi_en = confValue;
      EEPROM.put(0, conf);
    } else if (confKey == 15) {
      conf.dig_alr_lo_en = confValue;
      EEPROM.put(0, conf);
    } else if (confKey == 99) {
      resetMe();  
    }       
  } else if (downCh == 40) {
    digitalWrite(OUT1_PIN, digOutValue);
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
