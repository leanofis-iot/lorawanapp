#include <avr/wdt.h>
#include <avr/power.h>
#include "LowPower.h"
#include <AltSoftSerial.h>
#include <EEPROM.h>
#include <CayenneLPP.h>
#include <Wire.h>
#include "ClosedCube_SHT31D.h"

const uint8_t SHT_ALR_PIN   = 0;  // PD2/RXD1/INT2
const uint8_t SHT_RES_PIN   = 1;  // PD3/TXD1/INT3
const uint8_t RAK_RES_PIN   = 4;   // PD4/ADC8
const uint8_t LED_PIN       = 10;  // PB6/ADC13/PCINT6
const uint8_t BAT_PIN       = A0;  // PF7/ADC7
const uint8_t BAT_EN_PIN    = A1;  // PF6/ADC6
const uint8_t VREF_EN_PIN   = A2;  // PF5/ADC5

float BatVolt, BatVoltPrev;
volatile bool isAlarm;

const uint8_t batEnDly = 100, batSampDly = 1, batSampNum = 3;

struct Conf {  
  float bat_lo_v = 1.1;      
};

Conf conf;

void setup() {
  setPins();
  analogReference(INTERNAL);
  delay(1000);
  Serial.begin(115200);
  while (!Serial);         
}
void loop() {
  readAll();
  Serial.println(BatVolt);
  delay(2000);  
}
void readAll() {  
  readBatVolt();
  //calcBatAlarm();  
}
void readBatVolt() {
  //power_adc_enable();  
  //digitalWrite(BAT_EN_PIN, LOW);
  delay(batEnDly);
  uint16_t samples[batSampNum];
  for (uint8_t ii = 0; ii < batSampNum; ii++) {
    samples[ii] = analogRead(BAT_PIN);
    delay(batSampDly);
  } 
  //digitalWrite(BAT_EN_PIN, HIGH); 
  //power_adc_disable(); 
  BatVolt = 0;
  for (uint8_t jj = 0; jj < batSampNum; jj++) {
    BatVolt += samples[jj];
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
  pinMode(SHT_ALR_PIN, INPUT);
  pinMode(SHT_RES_PIN, OUTPUT); 
  pinMode(RAK_RES_PIN, OUTPUT);
  pinMode(LED_PIN, OUTPUT);  
  pinMode(BAT_PIN, INPUT);
  pinMode(BAT_EN_PIN, OUTPUT);
  pinMode(VREF_EN_PIN, OUTPUT); 
    
  digitalWrite(SHT_RES_PIN, HIGH);
  digitalWrite(RAK_RES_PIN, HIGH);
  digitalWrite(LED_PIN, HIGH);
  digitalWrite(BAT_EN_PIN, LOW);
  digitalWrite(VREF_EN_PIN, LOW);  
}
void wakeUp() {
  isAlarm = true;   
}
