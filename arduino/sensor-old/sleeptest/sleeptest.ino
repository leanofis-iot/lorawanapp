#include <avr/wdt.h>            // library for default watchdog functions
#include <avr/interrupt.h>      // library for interrupts handling
#include <avr/sleep.h>          // library for sleep
#include <avr/power.h>          // library for power control

int nbr_remaining; 

#define led 13

ISR(WDT_vect) {
  wdt_reset();
}

void configure_wdt(void) { 
  cli();                           // disable interrupts for changing the registers
  MCUSR = 0;                       // reset status register flags
                                   // Put timer in interrupt-only mode:                                       
  WDTCSR |= 0b00011000;            // Set WDCE (5th from left) and WDE (4th from left) to enter config mode,
                                   // using bitwise OR assignment (leaves other bits unchanged).
  WDTCSR =  0b01000000 | 0b100001; // set WDIE: interrupt enabled
                                   // clr WDE: reset disabled
                                   // and set delay interval (right side of bar) to 8 seconds
  sei();                           // re-enable interrupts  
  //  8 seconds: 0b100001 
}

void sleep(int ncycles) {  
  nbr_remaining = ncycles; // defines how many cycles should sleep 
  set_sleep_mode(SLEEP_MODE_PWR_DOWN);  
  power_adc_disable(); 
  
  while (nbr_remaining > 0){ // while some cycles left, sleep!  
  sleep_mode();   
  sleep_disable(); 
  nbr_remaining = nbr_remaining - 1; 
  }
  
  power_all_enable();
 
}

void setup(){  
 
  pinMode(led, OUTPUT);
  digitalWrite(led, LOW);  
  delay(1000);  
  
  configure_wdt();
 
  digitalWrite(led, HIGH);   
  delay(500);               
  digitalWrite(led, LOW);   
  delay(500); 
  digitalWrite(led, HIGH);   
  delay(500);               
  digitalWrite(led, LOW);   
  delay(500); 

}

void loop(){
  
  sleep(5);

  // usefull stuff should be done here before next long sleep
  // blink three times
  digitalWrite(led, HIGH);   
  delay(500);               
  digitalWrite(led, LOW);   
  delay(500); 
  digitalWrite(led, HIGH);   
  delay(500);               
  digitalWrite(led, LOW);   
  delay(500); 
  digitalWrite(led, HIGH);   
  delay(500);               
  digitalWrite(led, LOW);   
  delay(500); 

}
