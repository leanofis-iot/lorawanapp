#include <avr/wdt.h>
#include <avr/power.h>
#include "LowPower.h"
//#include <stdlib.h>

const uint8_t LED_PIN       = 10;  // PB6/ADC13/PCINT6
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
    //wdt_enable(WDTO_15MS);
    //wdt_reset();
    //while(true);
  }    
}
void sleepAndWake() {  
  LowPower.powerDown(SLEEP_8S, ADC_OFF, BOD_OFF); ///??????????????????????????????? BOD_OFF     
}
void setPins() {  
  pinMode(LED_PIN, OUTPUT);  
  digitalWrite(LED_PIN, HIGH);  
}
