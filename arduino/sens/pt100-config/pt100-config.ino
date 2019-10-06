#include <EEPROM.h>

struct Conf {
  const uint16_t read_t = 1;
  const uint16_t send_t = 5;
  const float bat_lo_v = 3.1;       
  const float alr_max[2] = {800, 800};
  const float alr_min[2] = {200, 200};
  const float alr_hys[2] = {0.01, 0.01};  
  const uint8_t an_type[2] = {1, 1};
  const uint8_t dig_type[2] = {1, 1};  
};

Conf conf;

void setup() {
  EEPROM.put(0, conf);
}

void loop() {
}