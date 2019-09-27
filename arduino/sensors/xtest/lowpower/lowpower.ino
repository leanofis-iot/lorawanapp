#include <avr/wdt.h>
#include <avr/power.h>
#include "LowPower.h"
#include <AltSoftSerial.h>
#include <EEPROM.h>
#include <CayenneLPP.h>
//#include <stdlib.h>

const uint8_t DIG_PIN[4]    = {3, 2, 0, 1};   // PD0/SCL/INT0, PD1/SDA/INT1, PD2/RXD1/INT2, PD3/TXD1/INT3 
const uint8_t RAK_RES_PIN   = 4;   // PD4/ADC8
const uint8_t LED_PIN       = 10;  // PB6/ADC13/PCINT6
const uint8_t BAT_PIN       = A0;  // PF7/ADC7
const uint8_t BAT_EN_PIN    = A1;  // PF6/ADC6
const uint8_t VREF_EN_PIN   = A2;  // PF5/ADC5
const uint8_t VOUT_EN_PIN   = A3;  // PF4/ADC4

const uint16_t read_t = 2, send_t = 5;
uint16_t minuteRead, minuteSend;

void setup() {
  setPins();  
  Serial.begin(115200);
  while (!Serial);
  Serial.println("powerup");
  digitalWrite(LED_PIN, LOW);
  delay(500);
  digitalWrite(LED_PIN, HIGH);
  delay(2000);
}
void loop() {  
  for (uint8_t ii = 0; ii < 8 ; ii++) {   
    sleepAndWake(); 
    Serial.println("wake: " + ii);            
  }    
  minuteRead++;
  minuteSend++;
  Serial.println("-------");
  Serial.println("minuteRead: " + minuteRead);
  Serial.println("minuteSend: " + minuteSend);   
  if (minuteRead >= read_t) {
    minuteRead = 0;    
    Serial.println("read"); 
  }    
  if (minuteSend >= send_t) {
    minuteRead = 0;
    minuteSend = 0;
    Serial.println("send");
  }    
}
void sleepAndWake() {  
  LowPower.powerDown(SLEEP_8S, ADC_OFF, BOD_OFF); ///??????????????????????????????? BOD_OFF     
}
void setPins() {  
  for (uint8_t ch = 0; ch < 4 ; ch++) {
    pinMode(DIG_PIN[ch], INPUT);
  }  
  pinMode(RAK_RES_PIN, OUTPUT);  
  pinMode(LED_PIN, OUTPUT);  
  pinMode(BAT_PIN, INPUT);
  pinMode(BAT_EN_PIN, OUTPUT);
  pinMode(VREF_EN_PIN, OUTPUT);
  pinMode(VOUT_EN_PIN, OUTPUT);  
  
  digitalWrite(RAK_RES_PIN, HIGH);
  digitalWrite(LED_PIN, HIGH);
  digitalWrite(BAT_EN_PIN, HIGH);
  digitalWrite(VREF_EN_PIN, HIGH);
  digitalWrite(VOUT_EN_PIN, LOW);  
}
/*
float BatVolt, BatVoltPrev;
volatile bool isAlarm;
bool isPowerUp;
const uint8_t batEnDly = 1, batSampDly = 1, batSampNum = 3, digDebounce = 10;
uint16_t minuteRead;
const unsigned long wdtMs30000 = 30000, wdtMs100 = 100;

struct Conf {
  uint16_t read_t;
  float bat_lo_v;  
  uint8_t dig_int[4];  
};

Conf conf;
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
  readAll();
  atRakClrSerial();
  atRakJoinOtaa();    
  uplink();   
}
void loop() {  
  for (uint8_t ii = 0; ii < 8 ; ii++) {   
    sleepAndWake();
    if (isAlarm) {
      delay(digDebounce);       
      uplink();      
    }               
  }    
  minuteRead++;
  if (minuteRead >= conf.read_t) {
    minuteRead = 0;    
    readAll();
    if (isAlarm) {      
      uplink();      
    }    
  }  
}
void sleepAndWake() {  
  for (uint8_t ch = 0; ch < 4 ; ch++) {
    if (conf.dig_int[ch]) {
      attachInterrupt(digitalPinToInterrupt(DIG_PIN[ch]), wakeUp, conf.dig_int[ch]);    
    }    
  }    
  LowPower.powerDown(SLEEP_8S, ADC_OFF, BOD_OFF); ///??????????????????????????????? BOD_OFF     
  for (uint8_t ch = 0; ch < 4 ; ch++) {
    if (conf.dig_int[ch]) {
      detachInterrupt(digitalPinToInterrupt(DIG_PIN[ch])); 
    }    
  }  
}
void uplink() { 
  isAlarm = false; 
  minuteRead = 0;
  lpp.reset();
  lpp.addAnalogInput(0, BatVolt);  
  for (uint8_t ch = 0; ch < 4 ; ch++) {
    if (conf.dig_int[ch]) {
      lpp.addDigitalInput(ch + 1, digitalRead(DIG_PIN[ch]));
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
  readBatVolt();
  calcBatAlarm();  
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
  for (uint8_t ch = 0; ch < 4 ; ch++) {
    pinMode(DIG_PIN[ch], INPUT);
  }  
  pinMode(RAK_RES_PIN, OUTPUT);  
  pinMode(LED_PIN, OUTPUT);  
  pinMode(BAT_PIN, INPUT);
  pinMode(BAT_EN_PIN, OUTPUT);
  pinMode(VREF_EN_PIN, OUTPUT);
  pinMode(VOUT_EN_PIN, OUTPUT);  
  
  digitalWrite(RAK_RES_PIN, HIGH);
  digitalWrite(LED_PIN, HIGH);
  digitalWrite(BAT_EN_PIN, HIGH);
  digitalWrite(VREF_EN_PIN, HIGH);
  digitalWrite(VOUT_EN_PIN, LOW);  
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
        } else if (str.startsWith(F("dig_int_1"))) {
          if (str.indexOf(F("=")) >= 0) {
            str.replace(F("dig_int_1="), "");
            conf.dig_int[0] = str.toInt();
            EEPROM.put(0, conf);
            Serial.println(F("OK"));
          } else {
            Serial.print(F("OK"));
            Serial.println(conf.dig_int[0]);
          } 
        } else if (str.startsWith(F("dig_int_2"))) {
          if (str.indexOf(F("=")) >= 0) {
            str.replace(F("dig_int_2="), "");
            conf.dig_int[1] = str.toInt();
            EEPROM.put(0, conf);
            Serial.println(F("OK"));
          } else {
            Serial.print(F("OK"));
            Serial.println(conf.dig_int[1]);
          }
        } else if (str.startsWith(F("dig_int_3"))) {
          if (str.indexOf(F("=")) >= 0) {
            str.replace(F("dig_int_3="), "");
            conf.dig_int[2] = str.toInt();
            EEPROM.put(0, conf);
            Serial.println(F("OK"));
          } else {
            Serial.print(F("OK"));
            Serial.println(conf.dig_int[2]);
          } 
        } else if (str.startsWith(F("dig_int_4"))) {
          if (str.indexOf(F("=")) >= 0) {
            str.replace(F("dig_int_4="), "");
            conf.dig_int[3] = str.toInt();
            EEPROM.put(0, conf);
            Serial.println(F("OK"));
          } else {
            Serial.print(F("OK"));
            Serial.println(conf.dig_int[3]);
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
      conf.bat_lo_v = confValue;      
    } else if (confKey == 3) {
      conf.dig_int[0] = confValue;      
    } else if (confKey == 4) {
      conf.dig_int[1] = confValue;      
    } else if (confKey == 5) {
      conf.dig_int[2] = confValue;      
    } else if (confKey == 6) {
      conf.dig_int[3] = confValue;     
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
void wakeUp() {
  isAlarm = true;   
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
*/
