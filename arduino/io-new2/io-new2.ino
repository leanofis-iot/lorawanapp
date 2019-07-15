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
const uint8_t anSampNum = 3, anSampDly = 1;
bool isAnAlarm, isAnAlarmFlag, isBatLow, isBatLowFlag, isExtInt, isPowerUp = true;
const unsigned long wdtMs60000 = 60000;
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
  setupPins();
  setupPeripheral();  
  setupConf();
  Serial.begin(115200);  
  usbSerial.begin(9600); 
  delay(3000);
  if (digitalRead(USB_PIN) == HIGH) {
    setupUsb();
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
    lpp.addDigitalInput(1, digitalRead(IN1D_PIN));
  }
  lpp.addAnalogInput(10, batteryVolt);
  if (isPowerUp) {
    isPowerUp = false;
    lpp.addAnalogOutput(11, 0);
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
  if (batteryVolt < conf.bat_lo_v) {
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
void setupPins() {
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
void setupPeripheral() {
  digitalWrite(RAK_RES_PIN, LOW);
  delay(10);
  digitalWrite(RAK_RES_PIN, HIGH);
}
void setupConf() {
  EEPROM.get(0, conf);
  if (conf.interface == _dig) {
    digitalWrite(VOUT_CNT_PIN, LOW);       
  }
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
void setupUsb() {   
  String strSerial, strUsbSerial; 
  while (true) { 
    if (usbSerial.available()) {
      strUsbSerial = usbSerialReadLine();
      if (strUsbSerial.startsWith(F("at"))) {       
        Serial.println(strUsbSerial);
      } else if (strUsbSerial.startsWith(F("period"))) {
        if (strUsbSerial.indexOf(F("=")) >= 0) {
          strUsbSerial.replace(F("period="), "");
          conf.period = strUsbSerial.toInt();
          EEPROM.put(0, conf);
          usbSerial.println(F("OK"));
        } else {
          usbSerial.print(F("OK"));
          usbSerial.println(conf.period);
        }       
      } else if (strUsbSerial.startsWith(F("bat_lo_v"))) {
        if (strUsbSerial.indexOf(F("=")) >= 0) {
          strUsbSerial.replace(F("bat_lo_v="), "");
          conf.bat_lo_v = strUsbSerial.toFloat();
          EEPROM.put(0, conf);
          usbSerial.println(F("OK"));
        } else {
          usbSerial.print(F("OK"));
          usbSerial.println(conf.bat_lo_v);
        }  
      } else if (strUsbSerial.startsWith(F("interface"))) {
        if (strUsbSerial.indexOf(F("=")) >= 0) {
          strUsbSerial.replace(F("interface="), "");
          conf.interface = strUsbSerial.toInt();
          EEPROM.put(0, conf);
          usbSerial.println(F("OK"));
        } else {
          usbSerial.print(F("OK"));
          usbSerial.println(conf.interface);
        }  
      } else if (strUsbSerial.startsWith(F("sensor"))) {
        if (strUsbSerial.indexOf(F("=")) >= 0) {
          strUsbSerial.replace(F("sensor="), "");
          conf.sensor = strUsbSerial.toInt();
          EEPROM.put(0, conf);
          usbSerial.println(F("OK"));
        } else {
          usbSerial.print(F("OK"));
          usbSerial.println(conf.sensor);
        }  
      } else if (strUsbSerial.startsWith(F("ntc_tmp"))) {
        if (strUsbSerial.indexOf(F("=")) >= 0) {
          strUsbSerial.replace(F("ntc_tmp="), "");
          conf.ntc_tmp = strUsbSerial.toFloat();
          EEPROM.put(0, conf);
          usbSerial.println(F("OK"));
        } else {
          usbSerial.print(F("OK"));
          usbSerial.println(conf.ntc_tmp);
        }   
      } else if (strUsbSerial.startsWith(F("ntc_res"))) {
        if (strUsbSerial.indexOf(F("=")) >= 0) {
          strUsbSerial.replace(F("ntc_res="), "");
          conf.ntc_res = strUsbSerial.toFloat();
          EEPROM.put(0, conf);
          usbSerial.println(F("OK"));
        } else {
          usbSerial.print(F("OK"));
          usbSerial.println(conf.ntc_res);
        } 
      } else if (strUsbSerial.startsWith(F("ntc_beta"))) {
        if (strUsbSerial.indexOf(F("=")) >= 0) {
          strUsbSerial.replace(F("ntc_beta="), "");
          conf.ntc_beta = strUsbSerial.toFloat();
          EEPROM.put(0, conf);
          usbSerial.println(F("OK"));
        } else {
          usbSerial.print(F("OK"));
          usbSerial.println(conf.ntc_beta);
        } 
      } else if (strUsbSerial.startsWith(F("ntc_ser_res"))) {
        if (strUsbSerial.indexOf(F("=")) >= 0) {
          strUsbSerial.replace(F("ntc_ser_res="), "");
          conf.ntc_ser_res = strUsbSerial.toFloat();
          EEPROM.put(0, conf);
          usbSerial.println(F("OK"));
        } else {
          usbSerial.print(F("OK"));
          usbSerial.println(conf.ntc_ser_res);
        }      
      } else if (strUsbSerial.startsWith(F("an_alr_en"))) {
        if (strUsbSerial.indexOf(F("=")) >= 0) {
          strUsbSerial.replace(F("an_alr_en="), "");
          conf.an_alr_en = strUsbSerial.toInt();
          EEPROM.put(0, conf);
          usbSerial.println(F("OK"));
        } else {
          usbSerial.print(F("OK"));
          usbSerial.println(conf.an_alr_en);
        }  
      } else if (strUsbSerial.startsWith(F("an_alr_hi_set"))) {
        if (strUsbSerial.indexOf(F("=")) >= 0) {
          strUsbSerial.replace(F("an_alr_hi_set="), "");
          conf.an_alr_hi_set = strUsbSerial.toFloat();
          EEPROM.put(0, conf);
          usbSerial.println(F("OK"));
        } else {
          usbSerial.print(F("OK"));
          usbSerial.println(conf.an_alr_hi_set);
        } 
      } else if (strUsbSerial.startsWith(F("an_alr_hi_clr"))) {
        if (strUsbSerial.indexOf(F("=")) >= 0) {
          strUsbSerial.replace(F("an_alr_hi_clr="), "");
          conf.an_alr_hi_clr = strUsbSerial.toFloat();
          EEPROM.put(0, conf);
          usbSerial.println(F("OK"));
        } else {
          usbSerial.print(F("OK"));
          usbSerial.println(conf.an_alr_hi_clr);
        }  
      } else if (strUsbSerial.startsWith(F("an_alr_lo_set"))) {
        if (strUsbSerial.indexOf(F("=")) >= 0) {
          strUsbSerial.replace(F("an_alr_lo_set="), "");
          conf.an_alr_lo_set = strUsbSerial.toFloat();
          EEPROM.put(0, conf);
          usbSerial.println(F("OK"));
        } else {
          usbSerial.print(F("OK"));
          usbSerial.println(conf.an_alr_lo_set);
        }   
      } else if (strUsbSerial.startsWith(F("an_alr_lo_clr"))) {
        if (strUsbSerial.indexOf(F("=")) >= 0) {
          strUsbSerial.replace(F("an_alr_lo_clr="), "");
          conf.an_alr_lo_clr = strUsbSerial.toFloat();
          EEPROM.put(0, conf);
          usbSerial.println(F("OK"));
        } else {
          usbSerial.print(F("OK"));
          usbSerial.println(conf.an_alr_lo_clr);
        }
      } else if (strUsbSerial.startsWith(F("dig_alr_hi_en"))) {
        if (strUsbSerial.indexOf(F("=")) >= 0) {
          strUsbSerial.replace(F("dig_alr_hi_en="), "");
          conf.dig_alr_hi_en = strUsbSerial.toInt();
          EEPROM.put(0, conf);
          usbSerial.println(F("OK"));
        } else {
          usbSerial.print(F("OK"));
          usbSerial.println(conf.dig_alr_hi_en);
        }   
      } else if (strUsbSerial.startsWith(F("dig_alr_lo_en"))) {
        if (strUsbSerial.indexOf(F("=")) >= 0) {
          strUsbSerial.replace(F("dig_alr_lo_en="), "");
          conf.dig_alr_lo_en = strUsbSerial.toInt();
          EEPROM.put(0, conf);
          usbSerial.println(F("OK"));
        } else {
          usbSerial.print(F("OK"));
          usbSerial.println(conf.dig_alr_lo_en);
        }               
      }       
    }
    if (Serial.available()) {
      strSerial = serialReadLine();   
      usbSerial.println(strSerial);   
    }      
  }    
}
void atRakSleep() {  
  Serial.println(F("at+sleep"));
  String str;
  str = RakReadLine(wdtMs60000);
  if (!str.endsWith(F("OK"))) {
    while(true);
  }
}
void atRakJoinOtaa() {  
  Serial.println(F("at+join=otaa"));
  String str;
  str = RakReadLine(wdtMs60000);
  if (!str.endsWith(F("OK"))) {
    while(true);
  }
  str = RakReadLine(wdtMs60000);
  if (!str.endsWith(F("3,0,0"))) {
    while(true);
  } 
}
void atRakSend(String str) { 
  str = "at+send=0,2," + str; 
  Serial.println(str);
  str = RakReadLine(wdtMs60000);
  if (!str.endsWith(F("OK"))) {
    while(true);
  }
  str = RakReadLine(wdtMs60000);
  if (!str.endsWith(F("2,0,0"))) {
    while(true);
  } 
  str = RakReadLine(wdtMs100);
  if (str) {
    const uint8_t i = str.lastIndexOf(',');
    str = str.substring(i + 3);
    str.replace("ff", "");
    lppDownDec(str);
    EEPROM.put(0, conf);
  }     
}
void lppDownDec(String str) {
  const char buf[4];
  str.toCharArray(buf, sizeof(buf));
  str = strtol(buf, NULL, 0);
  const uint8_t conf_key = str.substring(str.length() - 2).toInt();
  const uint16_t conf_value = str.substring(0, str.length() - 2).toInt();
  if (conf_key == 1) {
    conf.period = conf_value;  
  } else if (conf_key == 2) {
    conf.bat_lo_v = conf_value;
  } else if (conf_key == 3) {
    conf.interface = conf_value;
  } else if (conf_key == 4) {
    conf.sensor = conf_value;
  } else if (conf_key == 5) {
    conf.ntc_tmp = conf_value;
  } else if (conf_key == 6) {
    conf.ntc_res = conf_value;
  } else if (conf_key == 7) {
    conf.ntc_beta = conf_value;
  } else if (conf_key == 8) {
    conf.ntc_ser_res = conf_value;  
  } else if (conf_key == 9) {
    conf.an_alr_en = conf_value;
  } else if (conf_key == 10) {
    conf.an_alr_hi_set = conf_value;
  } else if (conf_key == 11) {
    conf.an_alr_hi_clr = conf_value;
  } else if (conf_key == 12) {
    conf.an_alr_lo_set = conf_value;
  } else if (conf_key == 13) {
    conf.an_alr_lo_clr = conf_value;
  } else if (conf_key == 14) {
    conf.dig_alr_hi_en = conf_value;
  } else if (conf_key == 15) {
    conf.dig_alr_lo_en = conf_value;    
  }  
}
void atRakWake() {  
  Serial.println(F("w"));
  String str;
  str = RakReadLine(wdtMs60000);
  if (!str.endsWith(F("OK"))) {
    while(true);
  }
  str = RakReadLine(wdtMs60000);
  if (!str.endsWith(F("8,0,0"))) {
    while(true);
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
  if (wdtMs == wdtMs60000) {
    while (true);
  }     
}
String serialReadLine() {  
  String str;  
  while (Serial.available()) {
    const char inChar = (char)Serial.read();
    str += inChar;
    if (inChar == '\n') {
      str.trim(); 
      return str;
    }
    // delay(2);
  }  
}
String usbSerialReadLine() {  
  String str;  
  while (usbSerial.available()) {
    const char inChar = (char)usbSerial.read();
    str += inChar;
    if (inChar == '\n') {
      str.trim(); 
      return str;
    }
    // delay(2);
  }  
}
void clearSerial() {
  Serial.flush();
  // delay(2);
  while (Serial.available()) {
    Serial.read();
    // delay(2);
  }
}
void clearUsbSerial() {
  usbSerial.flush();
  // delay(2);
  while (usbSerial.available()) {
    usbSerial.read();
    // delay(2);
  }
}
void wakeUp() {   
}
