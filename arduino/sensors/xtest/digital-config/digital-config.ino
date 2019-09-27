#include <EEPROM.h>

struct Conf {
  uint16_t read_t = 1;
  float bat_lo_v = 3.2;  
  uint8_t dig_int_1 = 1;
  uint8_t dig_int_2 = 1;
  uint8_t dig_int_3 = 1;
  uint8_t dig_int_4 = 1;
};

Conf conf;

void setup() {
  EEPROM.put(0, conf);
}

void loop() {
}
