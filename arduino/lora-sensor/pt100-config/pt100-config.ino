#include <EEPROM.h>

struct Conf {
  const uint16_t read_t = 1;
  const uint16_t send_t = 24;
  const float bat_lo_v = 3;       
  const float alr_max[2] = {40, 40};
  const float alr_min[2] = {10, 10};
  const float alr_hys[2] = {0.01, 0.01};
  const float cal_b[2] = {-6.3, -6.3};  
  const uint8_t an_type[2] = {1, 0};
  const uint8_t dig_type[2] = {1, 0};  
};

Conf conf;

void setup() {
  EEPROM.put(0, conf);
}

void loop() {
}