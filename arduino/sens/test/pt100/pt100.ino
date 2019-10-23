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

float In, R, Val[2];
const float rtd_coeff = 0.3851, rtd_r0 = 100, extRef = 2.5, r_ext = 2400;
const uint8_t vrefEnDly = 500;
unsigned long prevmillis = millis();
const long interval = 8000;

ADS1118 ads1118(ADS_CS_PIN);

void setup() {
  setPins();
  //digitalWrite(VREF_EN_PIN, LOW);
  delay(1000);
  setAds();
  Serial.begin(115200);
  while (!Serial);     
}
void loop() { 
  unsigned long curmillis = millis();
  if (curmillis - prevmillis >= interval) {
    
    readAll();
    Serial.println("");
    Serial.println(Val[0]);
    Serial.println(Val[1]);
    
    
       
    prevmillis = curmillis;     
  }    
}
void readAll() {      
  //readIn();
  //calcR();
  //calcVal();
  for (uint8_t ch = 0; ch < 2; ch++) {
    //if (conf.an_type[ch]) {
      adjAds(ch);
      readIn();
      calcR();
      calcVal(ch);           
    //}
  }  
}
void readIn() {
  digitalWrite(VREF_EN_PIN, LOW);
  //ads1118.setInputSelected(ads1118.DIFF_0_1);
  //ads1118.setInputSelected(ads1118.DIFF_2_3);
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
void setAds() {
  ads1118.begin();
  //ads1118.setSampligRate(ads1118.RATE_64SPS);  
  ads1118.disablePullup(); 
  ads1118.setFullScaleRange(ads1118.FSR_0256);
  //ads1118.setInputSelected(ads1118.DIFF_2_3);
  //delay(200); 
}
void adjAds(const uint8_t ch) {
  if (ch == 0) {
    ads1118.setInputSelected(ads1118.DIFF_2_3);
  } else if (ch == 1) {
    ads1118.setInputSelected(ads1118.DIFF_0_1);
  }    
}
void setPins() {
  for (uint8_t ch = 0; ch < 2; ch++) {
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
