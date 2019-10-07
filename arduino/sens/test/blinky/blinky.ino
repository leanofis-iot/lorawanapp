const uint8_t LED_PIN       = 10;  // PB6/ADC13/PCINT6

void setup() {
  setPins();  
}
void loop() {
  digitalWrite(LED_PIN, LOW);
  delay(500);
  digitalWrite(LED_PIN, HIGH);
  delay(500);
}
void setPins() {  
  pinMode(LED_PIN, OUTPUT);  
  digitalWrite(LED_PIN, HIGH);  
}
