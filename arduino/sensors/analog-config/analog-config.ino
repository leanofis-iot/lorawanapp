#include <EEPROM.h>

struct Conf {
  uint16_t read_t = 1;
  uint16_t send_t = 15;
  float bat_lo_v = 3.2;  
  uint8_t inp_en_1 = 1; 
  uint8_t inp_en_2 = 1;  
  float val_alr_hi_1 = 90;
  float val_alr_hi_2 = 90;
  float val_alr_lo_1 = 10;
  float val_alr_lo_2 = 10;
  float val_alr_hys_1 = 0.01; 
  float val_alr_hys_2 = 0.01; 
  float val_max_1 = 100;
  float val_max_2 = 100;
  float val_min_1 = 0;
  float val_min_2 = 0;
  float volt_max_1 = 10;
  float volt_max_2 = 10;  
  float volt_min_1 = 0;
  float volt_min_2 = 0;
};

Conf conf;

void setup() {
  EEPROM.put(0, conf);
}

void loop() {
}
