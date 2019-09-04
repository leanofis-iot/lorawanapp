#include <AltSoftSerial.h>
#include <EEPROM.h>
#include <EEWrap.h>
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
  uint16_e period;
  uint16_e pow_tm;
  float_e bat_lo_v;
  int8_e* interface;
  int8_e* sensor;
  float_e ntc_tmp;
  float_e ntc_res; 
  float_e ntc_beta; 
  float_e ntc_ser_res;
  uint8_e an_num_samp;
  uint8_e an_dly_samp;
  bool_e an_alr_en;
  float_e an_alr_hi_set;
  float_e an_alr_hi_clr;
  float_e an_alr_lo_set;
  float_e an_alr_lo_clr;
  bool_e dig_alr_hi_en;
  bool_e dig_alr_lo_en;  
};

float analog;
bool isAnalogAlarmChanged, isAnalogAlarmFlag, isBatteryLowChanged, isBatteryLowFlag;
bool isExtInt;

Conf conf EEMEM;
//Conf def_conf EEMEM;
AltSoftSerial usbSerial;
CayenneLPP lpp(51);

void setup() {
  setupPins();
  resetPeripheral();  
  //setupConf();
  Serial.begin(115200);
  //Serial.setTimeout(3000);
  usbSerial.begin(9600); 
  delay(3000);
  checkUsb();
  setupRak();      
}
void loop() {  
  for (byte i = 0; i < (conf.period * 7.5); i++) {
    setupAttachInt();    
    LowPower.powerDown(SLEEP_8S, ADC_OFF, BOD_OFF); 
    readAll();    
    if (isAnalogAlarmChanged || isBatteryLowChanged || isExtInt) {
      uplink(); 
    }    
  }   
  uplink();  
}
void readAll() {
  if (*conf.interface == "dig") {
    checkExtInt();        
  } else if (*conf.interface == "an") {  
    readAnalog();
    if (*conf.sensor == "ntc") {
      calcNtc();
    } else if (*conf.sensor == "5v") {
      calc5v();
    } else if (*conf.sensor == "10v") {
      calc10v();
    }
    if (conf.an_alr_en) {
      checkAnalogAlarm();      
    }
  }   
  checkBatteryLow();  
}
void uplink() {
  isExtInt = false;
  clearSerialRx();
  Serial.println("w");   
  lpp.reset();
  if (*conf.interface == "an") {  
    if (*conf.sensor == "ntc") {      
      lpp.addTemperature(1, analog);
    } else if (*conf.sensor == "5v" || *conf.sensor == "10v" ) {
      lpp.addAnalogInput(1, analog);
    }
  } else if (*conf.interface == "dig") {
    lpp.addDigitalInput(1, digitalRead(IN1D_PIN));
  }
  lpp.addAnalogInput(3, isBatteryLowFlag);   
  Serial.readStringUntil('\n'); 
  Serial.println("at+send=0,2," + *lpp.getBuffer());
  Serial.readStringUntil('\n');
  Serial.readStringUntil('\n');
  clearSerialRx();
  Serial.println(F("at+sleep"));
  Serial.readStringUntil('\n'); 
}
void readAnalog() {
  uint16_t samples[conf.an_num_samp];
  uint8_t i;
  if (*conf.sensor == "ntc") {
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
void resetPeripheral() {
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
void setupRak() {  
  clearSerialRx();
  Serial.println(F("at+mode=0"));
  Serial.readStringUntil('\n');
  Serial.readStringUntil('\n');
  Serial.readStringUntil('\n');
  Serial.readStringUntil('\n');
  Serial.println(F("at+band=EU868"));
  Serial.readStringUntil('\n');
  const String str = F("at+set_config=");
  Serial.println(str + F("class:0"));
  Serial.readStringUntil('\n');
  Serial.readStringUntil('\n');
  Serial.readStringUntil('\n');
  Serial.readStringUntil('\n');
  Serial.println(str + F("public_net:on"));
  Serial.readStringUntil('\n');
  Serial.println(str + F("adr:off"));
  Serial.readStringUntil('\n');
  Serial.println(str + F("dr:5"));
  Serial.readStringUntil('\n');
  Serial.println(str + F("ch_mask:0,00FF"));
  Serial.readStringUntil('\n');
  Serial.println(str + F("pwr_level:0"));
  Serial.readStringUntil('\n');
  Serial.println(str + F("rx_delay1:1000"));
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
      } else if (str.startsWith(F("interface"))) {
        if (str.indexOf(F("=")) >= 0) {
          str.replace(F("interface="), "");        
          for (uint8_t i = 0; i < str.length(); i++) {
            conf.interface[i] = str[i];
          }        
          usbSerial.println(F("OK")); 
        } else {
          usbSerial.println(*conf.interface);
        }         
      } else if (str.startsWith(F("sensor"))) {
        if (str.indexOf(F("=")) >= 0) {
          str.replace(F("sensor="), "");        
          for (uint8_t i = 0; i < str.length(); i++) {
            conf.sensor[i] = str[i];
          }        
          usbSerial.println(F("OK"));
        } else {
          usbSerial.println(*conf.sensor);
        }   
      } else if (str.startsWith(F("ntc_tmp"))) {
        if (str.indexOf(F("=")) >= 0) {
          str.replace(F("ntc_tmp="), "");
          conf.ntc_tmp = str.toFloat();
          usbSerial.println(F("OK"));
        } else {
          usbSerial.println(conf.ntc_tmp);
        }  
      } else if (str.startsWith(F("ntc_res"))) {
        if (str.indexOf(F("=")) >= 0) {
          str.replace(F("ntc_res="), "");
          conf.ntc_res = str.toFloat();
          usbSerial.println(F("OK"));
        } else {
          usbSerial.println(conf.ntc_res);
        }  
      } else if (str.startsWith(F("ntc_beta"))) {
        if (str.indexOf(F("=")) >= 0) {
          str.replace(F("ntc_beta="), "");
          conf.ntc_beta = str.toFloat();
          usbSerial.println(F("OK"));
        } else {
          usbSerial.println(conf.ntc_beta);
        }   
      } else if (str.startsWith(F("ntc_ser_res"))) {
        if (str.indexOf(F("=")) >= 0) {
          str.replace(F("ntc_ser_res="), "");
          conf.ntc_ser_res = str.toFloat();
          usbSerial.println(F("OK"));
        } else {
          usbSerial.println(conf.ntc_ser_res);
        }  
      } else if (str.startsWith(F("an_num_samp"))) {
        if (str.indexOf(F("=")) >= 0) {
          str.replace(F("an_num_samp="), "");
          conf.an_num_samp = str.toInt();
          usbSerial.println(F("OK")); 
        } else {
          usbSerial.println(conf.an_num_samp);
        }  
      } else if (str.startsWith(F("an_dly_samp"))) {
        if (str.indexOf(F("=")) >= 0) {
          str.replace(F("an_dly_samp="), "");
          conf.an_dly_samp = str.toInt();
          usbSerial.println(F("OK"));  
        } else {
          usbSerial.println(conf.an_dly_samp);
        }  
      } else if (str.startsWith(F("an_alr_en"))) {
        if (str.indexOf(F("=")) >= 0) {
          str.replace(F("an_alr_en="), "");        
          conf.an_alr_en = str;
          usbSerial.println(F("OK")); 
        } else {
          usbSerial.println(conf.an_alr_en);
        }             
      } else if (str.startsWith(F("an_alr_hi_set"))) {
        if (str.indexOf(F("=")) >= 0) {
          str.replace(F("an_alr_hi_set="), "");
          conf.an_alr_hi_set = str.toFloat();
          usbSerial.println(F("OK"));
        } else {
          usbSerial.println(conf.an_alr_hi_set);
        }   
      } else if (str.startsWith(F("an_alr_hi_clr"))) {
        if (str.indexOf(F("=")) >= 0) {
          str.replace(F("an_alr_hi_clr="), "");
          conf.an_alr_hi_clr = str.toFloat();
          usbSerial.println(F("OK"));
        } else {
          usbSerial.println(conf.an_alr_hi_clr);
        }  
      } else if (str.startsWith(F("an_alr_lo_set"))) {
        if (str.indexOf(F("=")) >= 0) {
          str.replace(F("an_alr_lo_set="), "");
          conf.an_alr_lo_set = str.toFloat();
          usbSerial.println(F("OK"));
        } else {
          usbSerial.println(conf.an_alr_lo_set);
        }  
      } else if (str.startsWith(F("an_alr_lo_clr"))) {
        if (str.indexOf(F("=")) >= 0) {
          str.replace(F("an_alr_lo_clr="), "");
          conf.an_alr_lo_clr = str.toFloat();
          usbSerial.println(F("OK")); 
        } else {
          usbSerial.println(conf.an_alr_lo_clr);
        }  
      } else if (str.startsWith(F("dig_alr_hi_en"))) {
        if (str.indexOf(F("=")) >= 0) {
          str.replace(F("dig_alr_hi_en="), "");        
          conf.dig_alr_hi_en = str;
          usbSerial.println(F("OK"));
        } else {
          usbSerial.println(conf.dig_alr_hi_en);
        }   
      } else if (str.startsWith(F("dig_alr_lo_en"))) {
        if (str.indexOf(F("=")) >= 0) {
          str.replace(F("dig_alr_lo_en="), "");        
          conf.dig_alr_lo_en = str;
          usbSerial.println(F("OK"));
        } else {
          usbSerial.println(conf.dig_alr_lo_en);
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
/*
 * void setupConf() {
  conf.period = 10;
  conf.pow_tm = 1;
  conf.bat_lo_v = 3.2;
  *conf.interface = "an";   
  *conf.sensor = "ntc";
  conf.ntc_tmp = 25;
  conf.ntc_res = 10000; 
  conf.ntc_beta = 3950; 
  conf.ntc_ser_res = 10000;
  conf.an_num_samp = 5;
  conf.an_dly_samp = 1;
  conf.an_alr_en = true;
  conf.an_alr_hi_set = 60;
  conf.an_alr_hi_clr = 58;
  conf.an_alr_lo_set = -10;
  conf.an_alr_lo_clr = -9;
  conf.dig_alr_hi_en = true;
  conf.dig_alr_lo_en = true; 
  if (*conf.interface == "dig") {
    digitalWrite(VOUT_CNT_PIN, LOW);       
  }
}
*/
 */
