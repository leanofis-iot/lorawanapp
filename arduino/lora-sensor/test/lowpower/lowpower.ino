//#include <avr/interrupt.h>
//#include <avr/sleep.h>

#include <avr/wdt.h>
#include <avr/power.h>
#include "LowPower.h"
#include <AltSoftSerial.h>
#include <EEPROM.h>
#include <CayenneLPP.h>
#include <Wire.h>
#include "ClosedCube_SHT31D.h"

const uint8_t SHT_ALR_PIN   = 0;  // PD2/RXD1/INT2
const uint8_t SHT_RES_PIN   = 1;  // PD3/TXD1/INT3
const uint8_t RAK_RES_PIN   = 4;   // PD4/ADC8
const uint8_t LED_PIN       = 10;  // PB6/ADC13/PCINT6
const uint8_t BAT_PIN       = A0;  // PF7/ADC7
const uint8_t BAT_EN_PIN    = A1;  // PF6/ADC6
const uint8_t VREF_EN_PIN   = A2;  // PF5/ADC5

struct Conf {     
  float alr_max[2] = {40, 90};
  float alr_min[2] = {10, 20};
  float alr_hys[2] = {0.01, 0.01};   
};

Conf conf;
ClosedCube_SHT31D sht;
AltSoftSerial rakSerial;
CayenneLPP lpp(51);

void setup() {
  setPins();
  rakSerial.begin(9600);
  setSht();
  for (uint8_t ii = 0; ii < 3 ; ii++) { 
    digitalWrite(LED_PIN, LOW);
    delay(200);
    digitalWrite(LED_PIN, HIGH);
    delay(200);              
  }  
  /*  
  set_sleep_mode(SLEEP_MODE_PWR_DOWN);
  cli();  
  sleep_enable();
  sei();
  sleep_cpu();
  sleep_disable();  
  sei();
  */

  /*
  ADCSRA &= ~(1 << ADEN);
  power_adc_disable();
  power_usart0_disable();
  power_spi_disable();
  power_twi_disable();
  power_timer0_disable();
  power_timer1_disable();
  power_timer2_disable();
  power_timer3_disable();
  //power_timer4_disable();
  power_usart1_disable();
  power_usb_disable();
  power_all_disable();
  ACSR &= ~(1<<ACIE); //disable interrupts on AC
  ACSR &= 1<<ACD; //switch on the AC
  */
  

  /*
  sleep_enable();     // set the sleep enable bit
  set_sleep_mode(SLEEP_MODE_PWR_DOWN);  
  sleep_cpu();        // put the arduino into sleep mode
  sleep_disable();       
  */
  //USBDevice.detach();
  USBCON |= _BV(FRZCLK);  //freeze USB clock
  PLLCSR &= ~_BV(PLLE);   // turn off USB PLL
  USBCON &= ~_BV(USBE);   // disable USB
  USBCON &= ~_BV(OTGPADE);
  USBCON &= ~_BV(VBUSTE);
  UHWCON &= ~_BV(UVREGE);

  PRR0 |= _BV(PRTWI);
  PRR0 |= _BV(PRTIM0);
  PRR0 |= _BV(PRTIM1);
  PRR0 |= _BV(PRSPI);
  //PRR0 |= _BV(PRADC);

  PRR1 |= _BV(PRUSB);
  //PRR1 |= _BV(PRTIM4);
  PRR1 |= _BV(PRTIM3);
  PRR1 |= _BV(PRUSART1);
  
  
  LowPower.powerDown(SLEEP_FOREVER, ADC_OFF, BOD_OFF); ///??????????????????????????????? BOD_OFF 
  
  //LowPower.idle(SLEEP_FOREVER, ADC_OFF, TIMER4_OFF, TIMER3_OFF, TIMER1_OFF, 
  //      TIMER0_OFF, SPI_OFF, USART1_OFF, TWI_OFF, USB_OFF);  
}
void loop() {  
  
}
void setPins() {
  /*
  pinMode(7, INPUT);
  pinMode(11, INPUT);
  pinMode(2, INPUT);
  pinMode(3, INPUT);
  pinMode(5, INPUT);
  pinMode(6, INPUT);
  pinMode(8, INPUT);
  pinMode(9, INPUT);
  pinMode(10, INPUT);
  pinMode(12, INPUT);
  pinMode(13, INPUT);
  pinMode(A3, INPUT);
  pinMode(A4, INPUT);
  pinMode(A5, INPUT);
  */
  
  pinMode(SHT_ALR_PIN, INPUT);
  pinMode(SHT_RES_PIN, OUTPUT); 
  pinMode(RAK_RES_PIN, INPUT);
  pinMode(LED_PIN, OUTPUT);  
  pinMode(BAT_PIN, INPUT);
  pinMode(BAT_EN_PIN, OUTPUT);
  pinMode(VREF_EN_PIN, OUTPUT);    
  digitalWrite(SHT_RES_PIN, HIGH);
  //digitalWrite(RAK_RES_PIN, HIGH);
  digitalWrite(LED_PIN, HIGH);
  digitalWrite(BAT_EN_PIN, HIGH);
  digitalWrite(VREF_EN_PIN, LOW);  
}
void setSht() {
  Wire.begin();
  sht.begin(0x44);  
  sht.periodicStart(SHT3XD_REPEATABILITY_LOW, SHT3XD_FREQUENCY_1HZ);
  sht.writeAlertHigh(conf.alr_max[0] + conf.alr_max[0] * conf.alr_hys[0], conf.alr_max[0] - conf.alr_max[0] * conf.alr_hys[0], 
                    conf.alr_max[1] + conf.alr_max[1] * conf.alr_hys[1], conf.alr_max[1] - conf.alr_max[1] * conf.alr_hys[1]);
  sht.writeAlertLow(conf.alr_min[0] + conf.alr_min[0] * conf.alr_hys[0], conf.alr_min[0] - conf.alr_min[0] * conf.alr_hys[0], 
                    conf.alr_min[1] + conf.alr_min[1] * conf.alr_hys[1], conf.alr_min[1] - conf.alr_min[1] * conf.alr_hys[1]);
  sht.clearAll();     
}
