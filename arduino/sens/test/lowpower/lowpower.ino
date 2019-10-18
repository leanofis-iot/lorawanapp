#include <avr/wdt.h>
#include <avr/power.h>
#include "LowPower.h"
//#include <stdlib.h>

const uint8_t LED_PIN       = 10;  // PB6/ADC13/PCINT6
const uint16_t read_t = 1, send_t = 2;
uint16_t minuteRead, minuteSend;

void setup() {
  setPins(); 
  for (uint8_t ii = 0; ii < 20 ; ii++) { 
    digitalWrite(LED_PIN, LOW);
    delay(200);
    digitalWrite(LED_PIN, HIGH);
    delay(200);              
  } 
  //Serial.begin(115200);
  //while (!Serial);
  //Serial.println("powerup");  
}
void loop() {  
  for (uint8_t ii = 0; ii < 8 ; ii++) {   
    sleepAndWake();     
      digitalWrite(LED_PIN, LOW);
      delay(200);
      digitalWrite(LED_PIN, HIGH);    
  }    
  minuteRead++;
  minuteSend++;
  if (minuteRead >= read_t) {
    minuteRead = 0;    
    digitalWrite(LED_PIN, LOW);
    delay(2000);
    digitalWrite(LED_PIN, HIGH); 
  }    
  if (minuteSend >= send_t) {
    minuteRead = 0;
    minuteSend = 0;
    digitalWrite(LED_PIN, LOW);
    delay(5000);
    digitalWrite(LED_PIN, HIGH); 
    wdt_enable(WDTO_15MS);
    wdt_reset();
    while(true);
  }    
}
void sleepAndWake() {  
  LowPower.powerDown(SLEEP_8S, ADC_OFF, BOD_OFF); ///??????????????????????????????? BOD_OFF     
}
void setPins() {  
  pinMode(LED_PIN, OUTPUT);  
  digitalWrite(LED_PIN, HIGH);  
}
