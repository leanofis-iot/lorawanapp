#include <EEPROM.h>

struct Conf {
  const uint16_t read_p = 1;
  const uint16_t send_p = 10;
  const float alr_lo_t = 5;
  const float alr_hi_t = 55;
  const float alr_hys_t = 1;
  const float alr_lo_h = 20;
  const float alr_hi_h = 80;
  const float alr_hys_h = 1;      
};

Conf conf;

void setup() {
  EEPROM.put(0, conf);
}

void loop() {
}
