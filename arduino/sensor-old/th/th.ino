#include <AltSoftSerial.h>
#include <EEPROM.h>
#include <EEWrap.h>
#include <Wire.h>
#include "ClosedCube_SHT31D.h"
#include "LowPower.h"
#include <CayenneLPP.h>

const byte VOUT_CNT_PIN  = 7;   // PD7/AIN1/PCINT23
const byte BAT_PIN       = 14;  // PC0/ADC0/PCINT8
const byte BAT_CNT_PIN   = 5;   // PD5/PCINT21
const byte USB_RX_PIN    = 8;   // PB0/PCINT0           AltSoftSerial RX
const byte USB_TX_PIN    = 9;   // PB1/PCINT1           AltSoftSerial TX
const byte USB_PIN       = 6;   // PD6/AIN0/PCINT22
const byte RAK_RX_PIN    = 0;   // PD0/RXD/PCINT16      Hardware Serial RX
const byte RAK_TX_PIN    = 1;   // PD1/TXD/PCINT17      Hardware Serial TX
const byte RAK_RES_PIN   = 15;  // PC1/ADC1/PCINT9
const byte SHT_SDA_PIN   = 18;  // PC4/ADC4/SDA/PCINT12 Hardware I2C SDA
const byte SHT_SCL_PIN   = 19;  // PC5/ADC5/SCL/PCINT13 Hardware I2C SCL
const byte SHT_RES_PIN   = 3;   // PD3/INT1/PCINT19
const byte SHT_ALR_PIN   = 2;   // PD2/INT0/PCINT18

struct Conf {
  uint16_e period;
  uint16_e pow_tm;
  float_e bat_lo_v;
  bool_e alr_en;
  float_e tmp_alr_hi_set;
  float_e tmp_alr_hi_clr;
  float_e tmp_alr_lo_set;
  float_e tmp_alr_lo_clr;
  float_e hum_alr_hi_set;
  float_e hum_alr_hi_clr;
  float_e hum_alr_lo_set;
  float_e hum_alr_lo_clr;  
};

bool isBatteryLowChanged, isBatteryLowFlag;
bool isExtInt;

Conf conf EEMEM;
AltSoftSerial usbSerial;
ClosedCube_SHT31D sht;
CayenneLPP lpp(51);

void setup() {
  setupPins(); 
  resetPeripheral(); 
  setupConf();
  Serial.begin(115200);
  Serial.setTimeout(3000);
  usbSerial.begin(9600); 
  delay(3000);
  checkUsb();
  setupSht();
  setupRak();    
}
void loop() {  
  for (byte i = 0; i < (conf.period * 7.5); i++) {
    attachInterrupt(digitalPinToInterrupt(SHT_ALR_PIN), wakeUp, RISING);
    LowPower.powerDown(SLEEP_8S, ADC_OFF, BOD_OFF);
    checkExtInt();
    checkBatteryLow(); 
    if (isExtInt || isBatteryLowChanged) {
      uplink(sht.periodicFetchData());      
    }
  }   
  uplink(sht.periodicFetchData());  
}
void uplink(SHT31D result) {
  isExtInt = false;
  clearSerialRx();
  Serial.println("w");   
  lpp.reset();
  lpp.addTemperature(1, result.t);
  lpp.addRelativeHumidity(2, result.rh);
  lpp.addAnalogInput(3, isBatteryLowFlag);
  Serial.readStringUntil('\n'); 
  Serial.println("at+send=0,2," + *lpp.getBuffer());
  Serial.readStringUntil('\n');
  Serial.readStringUntil('\n');
  clearSerialRx();
  Serial.println(F("at+sleep"));
  Serial.readStringUntil('\n'); 
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
void resetPeripheral() {
  digitalWrite(RAK_RES_PIN, LOW);
  digitalWrite(SHT_RES_PIN, LOW); 
  delay(10);
  digitalWrite(RAK_RES_PIN, HIGH);
  digitalWrite(SHT_RES_PIN, HIGH); 
}
void setupConf() {
  conf.period = 10;
  conf.pow_tm = 1;
  conf.bat_lo_v = 3.2;
  conf.alr_en = true;
  conf.tmp_alr_hi_set = 60;
  conf.tmp_alr_hi_clr = 58;
  conf.tmp_alr_lo_set = -10;
  conf.tmp_alr_lo_clr = -9;
  conf.hum_alr_hi_set = 80;
  conf.hum_alr_hi_clr = 79;
  conf.hum_alr_lo_set = 20;
  conf.hum_alr_lo_clr = 22;    
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
  if (INTF0 && conf.alr_en) {
    isExtInt = true;
  } else {
    isExtInt = false;
  }
  detachInterrupt(digitalPinToInterrupt(SHT_ALR_PIN));
}
void setupRak() {
  const String str = F("at+set_config=");
  clearSerialRx();
  Serial.println(F("at+mode=0"));
  Serial.readStringUntil('\n');
  Serial.println(F("at+band=EU868"));
  Serial.readStringUntil('\n');
  Serial.println(str + F("class:0&public_net:on&adr:off&dr:5&ch_mask:0,00FF&pwr_level:0&rx_delay1:1000"));
  Serial.readStringUntil('\n');
  Serial.println(str + F("ch_list:0,on,868100000,0,5"));
  Serial.readStringUntil('\n');
  Serial.println(str + F("ch_list:1,on,868300000,0,5"));
  Serial.readStringUntil('\n');
  Serial.println(str + F("ch_list:2,on,868500000,0,5"));
  Serial.readStringUntil('\n');
  Serial.println(str + F("ch_list:3,on,867100000,0,5"));
  Serial.readStringUntil('\n');
  Serial.println(str + F("ch_list:4,on,867300000,0,5"));
  Serial.readStringUntil('\n');
  Serial.println(str + F("ch_list:5,on,867500000,0,5"));
  Serial.readStringUntil('\n');
  Serial.println(str + F("ch_list:6,on,867700000,0,5"));
  Serial.readStringUntil('\n');
  Serial.println(str + F("ch_list:7,on,867900000,0,5"));
  Serial.readStringUntil('\n');
  Serial.println(str + F("ch_list:8,off"));
  Serial.readStringUntil('\n');
  Serial.println(str + F("ch_list:9,off"));
  Serial.readStringUntil('\n');
  Serial.println(str + F("ch_list:10,off"));
  Serial.readStringUntil('\n');
  Serial.println(str + F("ch_list:11,off"));
  Serial.readStringUntil('\n');
  Serial.println(str + F("ch_list:12,off"));
  Serial.readStringUntil('\n');
  Serial.println(str + F("ch_list:13,off"));
  Serial.readStringUntil('\n');
  Serial.println(str + F("ch_list:14,off"));
  Serial.readStringUntil('\n');
  Serial.println(str + F("ch_list:15,off"));
  Serial.readStringUntil('\n');
  Serial.println(F("at+join=otaa"));
  Serial.readStringUntil('\n');
  Serial.readStringUntil('\n');      
}
void checkUsb() { 
  clearSerialRx();
  clearUsbSerialRx(); 
  String str;   
  while (digitalRead(USB_PIN) == HIGH) {    
    str = "";
    str = usbSerial.readStringUntil('\n');
    if (str.startsWith(F("lorawan"))) {  
      str.replace(F("lorawan"),F("at"));    
      Serial.print(str + '\n');   
    } else if (str.startsWith(F("config"))) {
      str.replace(F("config+"), "");
      str.trim();

      if (str.startsWith(F("period"))) {
        if (str.indexOf(F("=")) >= 0) {
          str.replace(F("period="), "");
          conf.period = str.toInt();
          usbSerial.println(F("OK"));
        } else {
          usbSerial.println(conf.period);
        }      
      } else if (str.startsWith(F("pow_tm"))) {
        if (str.indexOf(F("=")) >= 0) {
          str.replace(F("pow_tm="), "");
          conf.pow_tm = str.toInt();
          usbSerial.println(F("OK"));
        } else {
          usbSerial.println(conf.pow_tm);
        }  
      } else if (str.startsWith(F("bat_lo_v"))) {
        if (str.indexOf(F("=")) >= 0) {
          str.replace(F("bat_lo_v="), "");
          conf.bat_lo_v = str.toFloat();
          usbSerial.println(F("OK"));
        } else {
          usbSerial.println(conf.bat_lo_v);
        }   
      } else if (str.startsWith(F("alr_en"))) {
        if (str.indexOf(F("=")) >= 0) {
          str.replace(F("alr_en="), "");
          conf.alr_en = str;
          usbSerial.println(F("OK")); 
        } else {
          usbSerial.println(conf.alr_en);
        }  
      } else if (str.startsWith(F("tmp_alr_hi_set"))) {
        if (str.indexOf(F("=")) >= 0) {
          str.replace(F("tmp_alr_hi_set="), "");
          conf.tmp_alr_hi_set = str.toFloat();
          usbSerial.println(F("OK"));
        } else {
          usbSerial.println(conf.tmp_alr_hi_set);
        }  
      } else if (str.startsWith(F("tmp_alr_hi_clr"))) {
        if (str.indexOf(F("=")) >= 0) {
          str.replace(F("tmp_alr_hi_clr="), "");
          conf.tmp_alr_hi_clr = str.toFloat();
          usbSerial.println(F("OK"));
        } else {
          usbSerial.println(conf.tmp_alr_hi_clr);
        } 
      } else if (str.startsWith(F("tmp_alr_lo_set"))) {
        if (str.indexOf(F("=")) >= 0) {
          str.replace(F("tmp_alr_lo_set="), "");
          conf.tmp_alr_lo_set = str.toFloat();
          usbSerial.println(F("OK"));
        } else {
          usbSerial.println(conf.tmp_alr_lo_set);
        }   
      } else if (str.startsWith(F("tmp_alr_lo_clr"))) {
        if (str.indexOf(F("=")) >= 0) {
          str.replace(F("tmp_alr_lo_clr="), "");
          conf.tmp_alr_lo_clr = str.toFloat();
          usbSerial.println(F("OK"));
        } else {
          usbSerial.println(conf.tmp_alr_lo_clr);
        } 
      } else if (str.startsWith(F("hum_alr_hi_set"))) { 
        if (str.indexOf(F("=")) >= 0) {       
          str.replace(F("hum_alr_hi_set="), "");
          conf.hum_alr_hi_set = str.toFloat();
          usbSerial.println(F("OK"));
        } else {
          usbSerial.println(conf.hum_alr_hi_set);
        } 
      } else if (str.startsWith(F("hum_alr_hi_clr"))) {
        if (str.indexOf(F("=")) >= 0) { 
          str.replace(F("hum_alr_hi_clr="), "");
          conf.hum_alr_hi_clr = str.toFloat();
          usbSerial.println(F("OK"));
        } else {
          usbSerial.println(conf.hum_alr_hi_clr);
        } 
      } else if (str.startsWith(F("hum_alr_lo_set"))) {
        if (str.indexOf(F("=")) >= 0) {
          str.replace(F("hum_alr_lo_set="), "");
          conf.hum_alr_lo_set = str.toFloat();
          usbSerial.println(F("OK"));
        } else {
          usbSerial.println(conf.hum_alr_lo_set);
        } 
      } else if (str.startsWith(F("hum_alr_lo_clr"))) {
        if (str.indexOf(F("=")) >= 0) {
          str.replace(F("hum_alr_lo_clr="), "");
          conf.hum_alr_lo_clr = str.toFloat(); 
          usbSerial.println(F("OK"));
        } else {
          usbSerial.println(conf.hum_alr_lo_clr);
        }             
      }           
    }       
    str = "";
    str = Serial.readStringUntil('\n');
    if (str != "") {
      usbSerial.print(str + '\n');
    }    
  }  
}
void clearSerialRx() {
  while (Serial.available()) {
    Serial.read();
  }
}
void clearUsbSerialRx() {
  while (usbSerial.available()) {
    usbSerial.read();
  }
}
void wakeUp() {   
}
