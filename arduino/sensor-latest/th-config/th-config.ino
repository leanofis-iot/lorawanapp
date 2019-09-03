#include <EEPROM.h>

struct Conf {
  uint16_t read_period = 2;
  uint16_t send_period = 10;
  float bat_lo_v = 3.2;  
  float tmp_alr_hi_set = 102;
  float tmp_alr_hi_clr = 98;
  float tmp_alr_lo_set = -2;
  float tmp_alr_lo_clr = 2;
  float hum_alr_hi_set = 82;
  float hum_alr_hi_clr = 78;
  float hum_alr_lo_set = 8;
  float hum_alr_lo_clr = 12;  
};

Conf conf;

void setup() {
  EEPROM.put(0, conf);
}

void loop() {
}
