#include <EEPROM.h>

const uint8_t LED_PIN       = 10;  // PB6/ADC13/PCINT6

struct Conf {
  uint16_t read_t;
  float bat_lo_v;  
  float alr_max[2];  
};

Conf conf;

void setup() {
  setPins();
  loadConf();  
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
void loadConf() {
  EEPROM.get(0, conf);  
}  
void setUsb() {
  digitalWrite(LED_PIN, LOW);
  delay(3000);
  digitalWrite(LED_PIN, HIGH);  
  Serial.begin(115200);
  while (!Serial);  
  Serial.println("hello");
  Serial.println("setusb feb");
  Serial.println(conf.read_t);
  Serial.println(conf.bat_lo_v);
  Serial.println(conf.alr_max[0]);
  Serial.println(conf.alr_max[1]);
  String str;
  while (true) {   
    if (Serial.available()) {
      const char chrUsb = (char)Serial.read();
      str += chrUsb;
      if (chrUsb == '\n') {
        str.trim();       
        if (str.startsWith(F("load"))) {       
          loadConf();
          Serial.print(F("OK"));
          Serial.println("loaded");
        } else if (str.startsWith(F("read_t"))) {
          if (str.indexOf(F("=")) >= 0) {
            str.replace(F("read_t="), "");
            conf.read_t = str.toInt();
            EEPROM.put(0, conf);
            Serial.println(F("OK"));
          } else {
            Serial.print(F("OK"));
            Serial.println(conf.read_t);
          }         
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
        }
        str = "";        
      }      
    }        
  }    
}
