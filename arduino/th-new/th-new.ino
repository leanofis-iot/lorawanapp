#include <AltSoftSerial.h>
#include <EEPROM.h>
#include <Wire.h>
#include "ClosedCube_SHT31D.h"
#include <avr/wdt.h>
#include "LowPower.h"
#include <CayenneLPP.h>

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
  uint16_t pow_tm;
  float bat_lo_v;
  float tmp_alr_hi_set;
  float tmp_alr_hi_clr;
  float tmp_alr_lo_set;
  float tmp_alr_lo_clr;
  float hum_alr_hi_set;
  float hum_alr_hi_clr;
  float hum_alr_lo_set;
  float hum_alr_lo_clr;  
};

bool isBatteryLowChanged, isBatteryLowFlag;
bool isExtInt;

Conf conf;
AltSoftSerial usbSerial;
ClosedCube_SHT31D sht;
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
  Serial.setTimeout(20000);
  setupSht();
  wdt_enable(WDTO_8S);
  atRakJoinOtaa();  
  uplink(sht.periodicFetchData());   
}
void loop() {  
  for (uint8_t i = 0; i < (conf.period * 7.5); i++) {
    attachInterrupt(digitalPinToInterrupt(SHT_ALR_PIN), wakeUp, CHANGE);
    LowPower.powerDown(SLEEP_8S, ADC_OFF, BOD_OFF);
    wdt_enable(WDTO_8S);
    readAll();     
    if (isExtInt || isBatteryLowChanged) {
      uplink(sht.periodicFetchData());      
    }
  }   
  uplink(sht.periodicFetchData());  
}
void uplink(SHT31D result) {
  lpp.reset();
  lpp.addTemperature(1, result.t);
  lpp.addRelativeHumidity(2, result.rh);
  lpp.addAnalogInput(3, isBatteryLowFlag);
  atRakWake();
  atRakSend("at+send=0,2," + *lpp.getBuffer());  
  atRakSleep();
}
void readAll() {
  checkExtInt();
  checkBatteryLow();
}
void checkBatteryLow() {
  analogReference(INTERNAL);
  digitalWrite(BAT_CNT_PIN, LOW);
  delay(conf.pow_tm);
  uint16_t bat_value = analogRead(BAT_PIN);
  digitalWrite(BAT_CNT_PIN, HIGH);
  float battery = ( bat_value / 1023 ) * 3.6;
  const bool isBatteryLowLog = isBatteryLowFlag; 
  if (battery < conf.bat_lo_v) {
    isBatteryLowFlag = true;   
  } else {
    isBatteryLowFlag = false;
  }
  if (isBatteryLowFlag != isBatteryLowLog) {
    isBatteryLowChanged = true;  
  } else {
    isBatteryLowChanged = false;
  }
}
void setupPins() {
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
void setupPeripheral() {
  digitalWrite(RAK_RES_PIN, LOW);
  digitalWrite(SHT_RES_PIN, LOW); 
  delay(10);
  digitalWrite(RAK_RES_PIN, HIGH);
  digitalWrite(SHT_RES_PIN, HIGH); 
}
void setupConf() {
  EEPROM.get(0, conf);   
}
void setupSht() {
  Wire.begin();
  sht.begin(0x44);
  sht.clearAll();
  sht.periodicStart(SHT3XD_REPEATABILITY_LOW, SHT3XD_FREQUENCY_1HZ);
  sht.writeAlertHigh(conf.tmp_alr_hi_set, conf.tmp_alr_hi_clr, conf.hum_alr_hi_set, conf.hum_alr_hi_clr);
  sht.writeAlertLow(conf.tmp_alr_lo_clr, conf.tmp_alr_lo_set, conf.hum_alr_lo_clr, conf.hum_alr_lo_set); 
}
void checkExtInt() {
  //if (PCIF2 || INTF0) {
  isExtInt = PCIF2;  
  detachInterrupt(digitalPinToInterrupt(SHT_ALR_PIN));
}
void setupUsb() {     
  clearSerial();
  clearUsbSerial(); 
  String str; 
  while (true) { 
    if (usbSerial.available()) {
      str = "";
      str = usbSerial.readStringUntil('\n');
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
      } else if (str.startsWith(F("pow_tm"))) {
        if (str.indexOf(F("=")) >= 0) {
          str.replace(F("pow_tm="), "");
          conf.pow_tm = str.toInt();
          EEPROM.put(0, conf);
          usbSerial.println(F("OK"));
        } else {
          usbSerial.print(F("OK"));
          usbSerial.println(conf.pow_tm);
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
      }       
    }
    if (Serial.available()) {
      str = "";
      str = Serial.readStringUntil('\n');
      str.trim();    
      usbSerial.println(str);   
    }      
  }    
}
void atRakSleep() {  
  clearSerial();  
  Serial.println(F("at+sleep"));
  Serial.find(F("OK\r\n"));  
}
void atRakJoinOtaa() {  
  clearSerial();  
  Serial.println(F("at+join=otaa"));
  Serial.find(F("3,0,0\r\n"));
}
void atRakSend(const String message) {  
  clearSerial();  
  Serial.println(message);
  Serial.find(F("2,0,0\r\n"));
  Serial.setTimeout(100);
  const String recv_data;
  recv_data = Serial.readStringUntil('\n');
  Serial.setTimeout(20000);
  recv_data.trim();
  const uint8_t i = recv_data.lastIndexOf(',');
  recv_data = recv_data.substring(i + 3);
  recv_data.replace("ff", ""); 
  const char buf[4];
  recv_data.toCharArray(buf, sizeof(buf));
  conf.period = strtoul(buf, NULL, 0) / 100;   
}
void atRakWake() {  
  clearSerial();  
  Serial.println(F("w"));
  Serial.find(F("8,0,0\r\n"));
}
void clearSerial() {
  wdt_reset();
  Serial.flush();
  while (Serial.available()) {
    Serial.read();
  }
}
void clearUsbSerial() {
  usbSerial.flush();
  while (usbSerial.available()) {
    usbSerial.read();
  }
}
void wakeUp() {   
}
