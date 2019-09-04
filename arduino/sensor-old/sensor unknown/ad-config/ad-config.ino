#include <EEPROM.h>

struct Conf {
  uint16_t read_period = 2;
  uint16_t send_period = 10;
  float bat_lo_v = 3.2;
  uint8_t an_in = 2;
  uint8_t dig_in = 1;
  float r_ext = 2000;
  float tmp_alr_hi_set = 102;
  float tmp_alr_hi_clr = 98;
  float tmp_alr_lo_set = -2;
  float tmp_alr_lo_clr = 2;
  float rtd_r0 = 100;  
  float rtd_coeff = 0.3851;  
  float ntc_coeff = 3950;  
  float ntc_r0 = 10000;
  float ntc_t0 = 25;  
};

Conf conf;

void setup() {
  EEPROM.put(0, conf);
}

void loop() {
}
