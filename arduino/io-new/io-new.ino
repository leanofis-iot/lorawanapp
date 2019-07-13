#include <AltSoftSerial.h>
#include <EEPROM.h>
#include <avr/wdt.h>
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
const byte IN1A_PIN      = 18;  // PC4/ADC4/SDA/PCINT12 Hardware I2C SDA
const byte IN2A_PIN      = 19;  // PC5/ADC5/SCL/PCINT13 Hardware I2C SCL
const byte IN1D_PIN      = 2;   //PD2/INT0/PCINT18
const byte IN2D_PIN      = 3;   // PD3/INT1/PCINT19
const byte OUT1_PIN      = 16; // PC2/ADC2/PCINT10
const byte OUT2_PIN      = 17; // PC3/ADC3/PCINT11

struct Conf {
  uint16_t period;
  uint16_t pow_tm;
  float bat_lo_v;
  uint8_t interface;
  uint8_t sensor;
  float ntc_tmp;
  float ntc_res; 
  float ntc_beta; 
  float ntc_ser_res;
  uint8_t an_num_samp;
  uint8_t an_dly_samp;
  uint8_t an_alr_en;
  float an_alr_hi_set;
  float an_alr_hi_clr;
  float an_alr_lo_set;
  float an_alr_lo_clr;
  uint8_t dig_alr_hi_en;
  uint8_t dig_alr_lo_en;  
};

const uint8_t _an = 0;
const uint8_t _dig = 1;
const uint8_t _3v = 1;
const uint8_t _5v = 2;
const uint8_t _10v = 3;
const uint8_t _ntc = 4;

float analog;
bool isAnalogAlarmChanged, isAnalogAlarmFlag, isBatteryLowChanged, isBatteryLowFlag;
bool isExtInt;

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
  Serial.setTimeout(20000); 
  wdt_enable(WDTO_8S);
  atRakJoinOtaa();
  readAll();  
  uplink();     
}
void loop() {  
  for (byte ii = 0; ii < (conf.period * 7.5); ii++) {
    setupAttachInt();    
    LowPower.powerDown(SLEEP_8S, ADC_OFF, BOD_OFF); 
    wdt_enable(WDTO_8S);
    readAll();    
    if (isAnalogAlarmChanged || isBatteryLowChanged || isExtInt) {
      uplink(); 
    }    
  }   
  uplink();  
}
void uplink() {  
  lpp.reset();
  if (conf.interface == _an) {  
    if (conf.sensor == _ntc) {      
      lpp.addTemperature(1, analog);
    } else if (conf.sensor == _3v || conf.sensor == _5v || conf.sensor == _10v ) {
      lpp.addAnalogInput(1, analog);
    }
  } else if (conf.interface == _dig) {
    lpp.addDigitalInput(1, digitalRead(IN1D_PIN));
  }
  lpp.addAnalogInput(3, isBatteryLowFlag);
  atRakWake();
  atRakSend("at+send=0,2," + *lpp.getBuffer());  
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
  uint16_t samples[conf.an_num_samp];
  uint8_t i;
  if (conf.sensor == _ntc) {
    analogReference(DEFAULT);
  } else {
    analogReference(INTERNAL);
  }   
  digitalWrite(VOUT_CNT_PIN, LOW);
  delay(conf.pow_tm);
  for (i=0; i< conf.an_num_samp; i++) {
    samples[i] = analogRead(IN1A_PIN);
    delay(conf.an_dly_samp);
  } 
  digitalWrite(VOUT_CNT_PIN, HIGH);  
  analog = 0;
  for (i=0; i< conf.an_num_samp; i++) {
    analog += samples[i];
  }
  analog /= conf.an_num_samp; 
}
void calcNtc() { 
  analog = 1023 / analog - 1;
  analog = conf.ntc_ser_res / analog;  
  analog = analog / conf.ntc_res;     
  analog = log(analog);                  
  analog /= conf.ntc_beta;                   
  analog += 1.0 / (conf.ntc_tmp + 273.15); 
  analog = 1.0 / analog;                 
  analog -= 273.15;   
}
void calc3v() {
  analog = ( analog / 1023 ) * 3;
}
void calc5v() {
  analog = ( analog / 1023 ) * 5;
}
void calc10v() {
  analog = ( analog / 1023 ) * 10;
}
void checkAnalogAlarm() {  
  const bool isAnalogAlarmLog = isAnalogAlarmFlag; 
  if (analog >= conf.an_alr_hi_set || analog <= conf.an_alr_lo_set) {
    isAnalogAlarmFlag = true;    
  } else if (analog <= conf.an_alr_hi_clr || analog >= conf.an_alr_lo_clr) {
    isAnalogAlarmFlag = false;
  }
  if (isAnalogAlarmFlag != isAnalogAlarmLog) {
    isAnalogAlarmChanged = true;  
  } else {
    isAnalogAlarmChanged = false;
  }
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
  pinMode(IN1A_PIN, INPUT);
  pinMode(IN2A_PIN, INPUT);
  pinMode(IN1D_PIN, INPUT);
  pinMode(IN2D_PIN, INPUT);
  pinMode(OUT1_PIN, OUTPUT);
  pinMode(OUT2_PIN, OUTPUT);  
  digitalWrite(VOUT_CNT_PIN, HIGH);
  digitalWrite(BAT_CNT_PIN, HIGH);
  digitalWrite(RAK_RES_PIN, HIGH);
  digitalWrite(OUT1_PIN, HIGH);
  digitalWrite(OUT2_PIN, HIGH);
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
      } else if (str.startsWith(F("an_num_samp"))) {
        if (str.indexOf(F("=")) >= 0) {
          str.replace(F("an_num_samp="), "");
          conf.an_num_samp = str.toInt();
          EEPROM.put(0, conf);
          usbSerial.println(F("OK"));
        } else {
          usbSerial.print(F("OK"));
          usbSerial.println(conf.an_num_samp);
        }    
      } else if (str.startsWith(F("an_dly_samp"))) {
        if (str.indexOf(F("=")) >= 0) {
          str.replace(F("an_dly_samp="), "");
          conf.an_dly_samp = str.toInt();
          EEPROM.put(0, conf);
          usbSerial.println(F("OK"));
        } else {
          usbSerial.print(F("OK"));
          usbSerial.println(conf.an_dly_samp);
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
  recv_data = Serial.readStringUntil("\n");
  Serial.setTimeout(20000);
  recv_data.trim();
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
