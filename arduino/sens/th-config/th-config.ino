#include <EEPROM.h>

struct Conf {
  const uint16_t read_t = 1;
  const uint16_t send_t = 2;
  const float bat_lo_v = 3;       
  const float alr_max[2] = {40, 90};
  const float alr_min[2] = {20, 10};
  const float alr_hys[2] = {0.01, 0.01};    
};

Conf conf;

void setup() {
  EEPROM.put(0, conf);
}

void loop() {
}
