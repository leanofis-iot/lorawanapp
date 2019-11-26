#include <EEPROM.h>

struct Conf {
  const uint16_t read_p = 1;
  const uint16_t send_p = 30;
  const float alr_max[2] = {55, 80};
  const float alr_min[2] = {5, 20};
  const float alr_hys[2] = {1, 1};
  const float cal_b[2] = {-6.3, -6.3};  
  const uint8_t an_type[2] = {1, 0};
  const uint8_t dig_type[2] = {1, 0}; 
  const uint8_t dr = 2; 
};

Conf conf;

void setup() {
  EEPROM.put(0, conf);
}

void loop() {
}
