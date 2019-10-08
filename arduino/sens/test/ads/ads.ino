#include <avr/wdt.h>
#include <avr/power.h>
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

float In, Val[2], InFact, ValPrev[2], BatVolt, BatVoltPrev;
bool isAlarm;
const uint8_t batEnDly = 1, batSampDly = 1, batSampNum = 3;
const uint8_t andiff = 1, ansingle = 2, an5v = 1, an10v = 2, an420ma = 3;

struct Conf {
  float bat_lo_v = 3.1;       
  float alr_max[2] = {70, 70};
  float alr_min[2] = {10, 10};
  float alr_hys[2] = {0.01, 0.01};  
  float val_max[2] = {100, 100};
  float val_min[2] = {0, 0}; 
  float in_max[2] = {5, 5};
  float in_min[2] = {0, 0};
  uint8_t an_type[2] = {1, 1};
  uint8_t an_end[2] = {2, 2};  
  uint16_t vout_dly = 2;
};

Conf conf;
ADS1118 ads1118(ADS_CS_PIN);

void setup() {
  setPins();  
  analogReference(INTERNAL);  
  Serial.begin(115200);
  while (!Serial);
  flashLed3();  
  setAds();   
}
void loop() { 
  readAll();
  Serial.println("readAll: ");
  Serial.println("Val[0]: " + String(Val[0]));
  Serial.println("Val[1]: " + String(Val[1]));
  Serial.println("BatVolt: " + String(BatVolt));
  Serial.println("isAlarm: " + String(isAlarm));
  isAlarm = false;
  delay(1000);  
}
void readAll() {
  readBatVolt();
  calcBatAlarm();
  for (uint8_t ch = 0; ch < 2 ; ch++) {
    if (conf.an_type[ch]) {
      adjAds(ch);
      readIn();
      calcVal(ch);
      calcValAlarm(ch); 
    }
  }  
}
void readIn() {
  if (conf.vout_dly) {
    digitalWrite(VOUT_EN_PIN, LOW);
    delay(conf.vout_dly); 
  }  
  In = ads1118.getMilliVolts(); 
  digitalWrite(VOUT_EN_PIN, HIGH);   
  In = In * InFact;
  if (conf.an_type == an420ma) {
    In /= 4700;  // mV / 470R = ma
  } else {
    In /= 1000; // mV / 1000 = volt
  }    
}
void calcVal(const uint8_t ch) {  
  Val[ch] = (In - conf.in_min[ch]) * (conf.val_max[ch] - conf.val_min[ch]) / (conf.in_max[ch] - conf.in_min[ch]) + conf.val_min[ch];  
  //(x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;  
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
void setAds() {
  ads1118.begin();
  //ads1118.setSampligRate(ads1118.RATE_64SPS);  
  ads1118.disablePullup();  
}
void adjAds(const uint8_t ch) {
  if (conf.an_end[ch] == andiff) {
    if (ch == 0) {
      ads1118.setInputSelected(ads1118.DIFF_0_1);
    } else if (ch == 1) {
      ads1118.setInputSelected(ads1118.DIFF_2_3);
    }    
    InFact = (10 + 10 + 2.2) / 2.2;
    if (conf.an_type[ch] == an5v) {
      ads1118.setFullScaleRange(ads1118.FSR_0512);      
    } else if (conf.an_type[ch] == an10v) {
      ads1118.setFullScaleRange(ads1118.FSR_1024);
    } else if (conf.an_type[ch] == an420ma) {
      ads1118.setFullScaleRange(ads1118.FSR_1024);
      InFact = InFact * (10 + 2.2 + 0.47) / (10 + 2.2);  
    }
  } else if (conf.an_end[ch] == ansingle) {
    if (ch == 0) {
      ads1118.setInputSelected(ads1118.AIN_0);
    } else if (ch == 1) {
      ads1118.setInputSelected(ads1118.AIN_1);
    }  
    InFact = (10 + 2.2) / 2.2;
    if (conf.an_type[ch] == an5v) {
      ads1118.setFullScaleRange(ads1118.FSR_1024);
    } else if (conf.an_type[ch] == an10v) {
      ads1118.setFullScaleRange(ads1118.FSR_2048);    
    } else if (conf.an_type[ch] == an420ma) {
      ads1118.setFullScaleRange(ads1118.FSR_2048);
      InFact = InFact * (10 + 10 + 2.2 + 0.47) / (10 + 10 + 2.2);  
    }
  }  
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
