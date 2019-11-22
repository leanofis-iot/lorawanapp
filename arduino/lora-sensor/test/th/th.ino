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

volatile bool isAlarm;

struct Conf {
  uint16_t read_t;
  uint16_t send_t;
  float bat_lo_v;       
  float alr_max[2];
  float alr_min[2];
  float alr_hys[2];    
};

Conf conf;
ClosedCube_SHT31D sht;
AltSoftSerial rakSerial;
CayenneLPP lpp(51);

void setup() {
  setPins();
  rakSerial.begin(9600);
  //setPeripheral();
  analogReference(INTERNAL);
  loadConf();
  delay(1000);   
  setSht();
  flashLed3();
  Serial.begin(115200);
  while (!Serial);    
  attachInterrupt(digitalPinToInterrupt(SHT_ALR_PIN), wakeUp, CHANGE);    
}
void loop() { 
  Serial.println("");
  if (isAlarm) {
    Serial.println("alarm !!");
    isAlarm = false;     
  }    
  SHT31D result = sht.periodicFetchData();
  lpp.reset();
  //lpp.addAnalogInput(0, BatVolt); 
  lpp.addTemperature(1, result.t);
  lpp.addRelativeHumidity(2, result.rh);
  Serial.println(result.t);
  Serial.println(result.rh);
  Serial.println(lppGetBuffer());
  delay(3000);
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
  digitalWrite(RAK_RES_PIN, LOW);  
  digitalWrite(LED_PIN, HIGH);
  digitalWrite(BAT_EN_PIN, HIGH);
  digitalWrite(VREF_EN_PIN, LOW);  
}
void setPeripheral() {  
  digitalWrite(SHT_RES_PIN, LOW);
  delay(100);
  digitalWrite(SHT_RES_PIN, HIGH);  
}
void setSht() {
  Wire.begin();
  sht.begin(0x44);
  //Serial.print("Serial #");
  //Serial.println(sht.readSerialNumber());
  sht.clearAll();
  sht.periodicStart(SHT3XD_REPEATABILITY_LOW, SHT3XD_FREQUENCY_1HZ);
  sht.writeAlertHigh(conf.alr_max[0] + conf.alr_max[0] * conf.alr_hys[0], conf.alr_max[0] - conf.alr_max[0] * conf.alr_hys[0], 
                    conf.alr_max[1] + conf.alr_max[1] * conf.alr_hys[1], conf.alr_max[1] - conf.alr_max[1] * conf.alr_hys[1]);
  sht.writeAlertLow(conf.alr_min[0] + conf.alr_min[0] * conf.alr_hys[0], conf.alr_min[0] - conf.alr_min[0] * conf.alr_hys[0], 
                    conf.alr_min[1] + conf.alr_min[1] * conf.alr_hys[1], conf.alr_min[1] - conf.alr_min[1] * conf.alr_hys[1]);
  sht.clearAll();
}
void loadConf() {
  EEPROM.get(0, conf);
  if (conf.read_t < 1) {
    conf.read_t = 1; 
  }
  //if (conf.send_t < 5) {
  //  conf.send_t = 5;
  //}  
}
String lppGetBuffer() {
  String str;
  for(uint8_t ii = 0; ii < lpp.getSize(); ii++){    
    if (lpp.getBuffer()[ii] < 16) {
      str += '0';       
    }
    str += String(lpp.getBuffer()[ii], HEX);
    str.toUpperCase();        
  }
  return str;
}
void wakeUp() {
  isAlarm = true;   
}
void flashLed3() {
  for (uint8_t ii = 0; ii < 3; ii++) {
    digitalWrite(LED_PIN, LOW);
    delay(50);
    digitalWrite(LED_PIN, HIGH);
    delay(200);
  }  
}
