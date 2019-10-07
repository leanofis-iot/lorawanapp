#include <AltSoftSerial.h>

const uint8_t LED_PIN       = 10;  // PB6/ADC13/PCINT6

AltSoftSerial rakSerial;

void setup() {
  setPins();
  Serial.begin(115200);
  while (!Serial);
  rakSerial.begin(9600); 
  delay(5000);
  if (USBSTA >> VBUS & 1) {
    setUsb();
  } 
}
void loop() {
  digitalWrite(LED_PIN, LOW);
  delay(200);
  digitalWrite(LED_PIN, HIGH);
  delay(200);
}
void setPins() {  
  pinMode(LED_PIN, OUTPUT);  
  digitalWrite(LED_PIN, HIGH);  
}
void setUsb() {
  String str;
  while (true) {   
    if (Serial.available()) {
      const char chrUsb = (char)Serial.read();
      str += chrUsb;
      if (chrUsb == '\n') {
        str.trim();       
        if (str.startsWith(F("at"))) {          
          rakSerial.println(str);        
        }
        str = "";        
      }      
    }
    if (rakSerial.available()) {
      const char chrSerial = (char)rakSerial.read();   
      Serial.print(chrSerial);   
    }      
  }    
}
