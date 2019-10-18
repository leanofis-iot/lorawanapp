#include <avr/wdt.h>
#include <avr/power.h>
#include <EEPROM.h>
#include "ADS1118.h"
#include <SPI.h>
//#include <stdlib.h>

const uint8_t DIG_PIN[2]    = {3, 2}; // PD0/SCL/INT0, PD1/SDA/INT1
//const uint8_t SHT_ALR_PIN   = 0;  // PD2/RXD1/INT2
//const uint8_t SHT_RES_PIN   = 1;  // PD3/TXD1/INT3
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
unsigned long prevmillis = millis();
const long interval = 2000;
String str;

struct Conf {  
  float bat_lo_v = 3.1;       
  float alr_max[2] = {100, 100};
  float alr_min[2] = {0, 0};
  float alr_hys[2] = {0.01, 0.01};  
  uint8_t an_type[2] = {1, 1};
  uint8_t dig_type[2] = {1, 1};
};

Conf conf;
ADS1118 ads1118(ADS_CS_PIN);

void setup() {
  setPins();
  Serial.begin(115200);
  while (!Serial);
  analogReference(INTERNAL);
  loadConf();  
  setAds(); 
  /*
  for (uint8_t ch = 0; ch < 2 ; ch++) {
    if (conf.dig_type[ch]) {
      attachInterrupt(digitalPinToInterrupt(DIG_PIN[3 - ch]), wakeUp, conf.dig_type[ch]);    
    }    
  } 
  */   
}
void loop() { 
  unsigned long curmillis = millis();
  if (curmillis - prevmillis >= interval) {
    readAll();
    Serial.println("");
    Serial.println(Val[0]);
    Serial.println(Val[1]);
    Serial.println(BatVolt);
    //Serial.println(isAlarm);
    prevmillis = curmillis;     
  }
  checkUsb();    
}
void readAll() {  
  readBatVolt();
  calcBatAlarm();
  for (uint8_t ch = 0; ch < 2 ; ch++) {
    if (conf.an_type[ch]) {
      adjAds(ch);
      readIn();
      calcR();
      calcVal(ch);
      calcValAlarm(ch); 
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
    pinMode(DIG_PIN[3 - ch], INPUT);
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
void loadConf() {
  EEPROM.get(0, conf);  
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
void wakeUp() {
  isAlarm = true;   
}
void checkUsb() {
  if (Serial.available()) {
    const char chrUsb = (char)Serial.read();
    str += chrUsb;
    if (chrUsb == '\n') {
      str.trim();       
      if (str.startsWith(F("load"))) {       
        loadConf();
        Serial.print(F("OK"));
        Serial.println("loaded");      
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
          conf.alr_max[0] = str.toInt();
          EEPROM.put(0, conf);
          Serial.println(F("OK"));
        } else {
          Serial.print(F("OK"));
          Serial.println(conf.alr_max[0]);
        } 
      } else if (str.startsWith(F("alr_max_2"))) {
        if (str.indexOf(F("=")) >= 0) {
          str.replace(F("alr_max_2="), "");
          conf.alr_max[1] = str.toInt();
          EEPROM.put(0, conf);
          Serial.println(F("OK"));
        } else {
          Serial.print(F("OK"));
          Serial.println(conf.alr_max[1]);
        }
      } else if (str.startsWith(F("alr_min_1"))) {
        if (str.indexOf(F("=")) >= 0) {
          str.replace(F("alr_min_1="), "");
          conf.alr_min[0] = str.toInt();
          EEPROM.put(0, conf);
          Serial.println(F("OK"));
        } else {
          Serial.print(F("OK"));
          Serial.println(conf.alr_min[0]);
        } 
      } else if (str.startsWith(F("alr_min_2"))) {
        if (str.indexOf(F("=")) >= 0) {
          str.replace(F("alr_min_2="), "");
          conf.alr_min[1] = str.toInt();
          EEPROM.put(0, conf);
          Serial.println(F("OK"));
        } else {
          Serial.print(F("OK"));
          Serial.println(conf.alr_min[1]);
        }
      } else if (str.startsWith(F("alr_hys_1"))) {
        if (str.indexOf(F("=")) >= 0) {
          str.replace(F("alr_hys_1="), "");
          conf.alr_hys[0] = str.toInt();
          EEPROM.put(0, conf);
          Serial.println(F("OK"));
        } else {
          Serial.print(F("OK"));
          Serial.println(conf.alr_hys[0]);
        } 
      } else if (str.startsWith(F("alr_hys_2"))) {
        if (str.indexOf(F("=")) >= 0) {
          str.replace(F("alr_hys_2="), "");
          conf.alr_hys[1] = str.toInt();
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
}
