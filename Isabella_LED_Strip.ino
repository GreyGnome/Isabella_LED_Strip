/*
      Copyright [2019] [Michael Anthony Schwager]

   Licensed under the Apache License, Version 2.0 (the "License");
   you may not use this file except in compliance with the License.
   You may obtain a copy of the License at

       http://www.apache.org/licenses/LICENSE-2.0

   Unless required by applicable law or agreed to in writing, software
   distributed under the License is distributed on an "AS IS" BASIS,
   WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
   See the License for the specific language governing permissions and
   limitations under the License.

   A copy of the license is also provided in a file alongside this source code
   at https://github.com/GreyGnome/IndividualLED/+***SOMETHING***
 */
// === DESCRIPTION =====================================================================
// ATMEL ATMEGA8A / ARDUINO
//
//                           +-\/-+
//     RESET   (D 22)  PC6  1|    |28  PC5  (D 19  A5  SCL ADC5)
//             (D  0)  PD0  2|    |27  PC4  (D 18  A4  SDA ADC4)
//             (D  1)  PD1  3|    |26  PC3  (D 17  A3  ADC3)
//       INT0  (D  2)  PD2  4|    |25  PC2  (D 16  A2  ADC2)
//       INT1  (D  3)  PD3  5|    |24  PC1  (D 15  A1  ADC1)
//             (D  4)  PD4  6|    |23  PC0  (D 14  A0  ADC0)
//                     VCC  7|    |22  GND
//                     GND  8|    |21  AREF
//             (D 20)  PB6  9|    |20  AVCC
//             (D 21)  PB7 10|    |19  PB5  (D 13  SCK) 
//             (D  5)  PD5 11|    |18  PB4  (D 12  MISO) 
//  AIN0       (D  6)  PD6 12|    |17  PB3  (D 11  MOSI OC2)
//  AIN1       (D  7)  PD7 13|    |16  PB2  (D 10  SS OC1A)
//             (D  8)  PB0 14|    |15  PB1  (D  9     OC1B)
//                           +----+
#include <avr/interrupt.h>
#include "digitalWriteFast.h"

uint8_t LED_STRIP = 9;
uint8_t led_on=false;
const uint8_t DOWN_BUTTON=2; // INT0
const uint8_t UP_BUTTON=3; // INT1
uint8_t l1 = 10;
uint8_t l2 = 20;
uint8_t l3 = 40;
uint8_t l4 = 200;
uint8_t l5 = 255;

//lISR(TIMER2_OVF_vect) {
ISR(TIMER2_COMP_vect) {
  bitWrite(PORTB, 1, 1);
  __asm__ __volatile__ ("nop");
  __asm__ __volatile__ ("nop");
  __asm__ __volatile__ ("nop");
  __asm__ __volatile__ ("nop");
  __asm__ __volatile__ ("nop");
  __asm__ __volatile__ ("nop");
  __asm__ __volatile__ ("nop");
  __asm__ __volatile__ ("nop");
  __asm__ __volatile__ ("nop");
  __asm__ __volatile__ ("nop");
  __asm__ __volatile__ ("nop");
  __asm__ __volatile__ ("nop");
  __asm__ __volatile__ ("nop");
  __asm__ __volatile__ ("nop");
  __asm__ __volatile__ ("nop");
  __asm__ __volatile__ ("nop");
  __asm__ __volatile__ ("nop");
  __asm__ __volatile__ ("nop");
  __asm__ __volatile__ ("nop");
  __asm__ __volatile__ ("nop");
  __asm__ __volatile__ ("nop");
  __asm__ __volatile__ ("nop");
  __asm__ __volatile__ ("nop");
  __asm__ __volatile__ ("nop");
  __asm__ __volatile__ ("nop");
  __asm__ __volatile__ ("nop");
  __asm__ __volatile__ ("nop");
  __asm__ __volatile__ ("nop");
  __asm__ __volatile__ ("nop");
  __asm__ __volatile__ ("nop");
  __asm__ __volatile__ ("nop");
  __asm__ __volatile__ ("nop");
  bitWrite(PORTB, 1, 0);
}

void set_all_pins_input() {
  pinMode(LED_STRIP, INPUT_PULLUP);
}

void set_pin_directions(void) {
  pinMode(11, OUTPUT); // DC2
  pinMode(LED_STRIP, OUTPUT);
  pinMode(DOWN_BUTTON, INPUT);
  pinMode(UP_BUTTON, INPUT);
}

void turn_led_off(void) {
  pinMode(LED_STRIP, INPUT);
  led_on = false;
}

void set_light_level(uint8_t pin, uint8_t level) {

}

uint8_t light_level=0;
void turn_led_on(void) {
  pinMode(LED_STRIP, OUTPUT);
  led_on = true;
  set_light_level(LED_STRIP, light_level);
}

// the setup function runs once when you press reset or power the board
uint32_t current_millis=0;
void setup() {
  // Set up timer 2
  SFIOR |= (1 << PSR2);  // reset prescaler. Not sure I need to do this.
  TCCR2 = 0;
  OCR2 = 0x7B;  // *approximately* 1024 times per second.
  TCCR2 &= ~(1 << COM20); // SHUT OFF OUTPUT PIN
  TCCR2 &= ~(1 << COM21); // SHUT OFF OUTPUT PIN
  //TCCR2 |= (1 << WGM21 | 1 << WGM20);  /* Fast PWM mode */
  TCCR2 |= (1 << WGM21);  /* CTC mode */
  TIMSK |= (1 << TOIE2); /* enable timer2 overflow interrupt */
  TIMSK |= (1 << OCIE2); /* enable timer2 compare interrupt */
  //TCCR2 |= (1 << CS20);   // No prescaler 
  TCCR2 |= (1 << CS22);   // x64 prescaler 
  TIFR |= (1 << TOV2);    /* clear interrupt flag */
  // 2222222222222222222222222222222222222222222222222222222222222
  set_pin_directions();
  current_millis=millis();
  ADCSRA &= ~(1<<ADEN); // Disable ADC for better power consumption
}

uint32_t now_millis=0, last_update_millis=0, flash_millis, 
         down_button_press=0, up_button_press=0;
uint32_t down_button_release=0, up_button_release=0;
uint32_t last_down_press=0, last_up_press=0;
bool is_double_press=false; // double_press: down/down within 1 second.
bool down_button=false;
bool up_button=false;
bool waited=false; // better control when push up starts
// the loop function runs forever
/*
 * Buttons:
 * - : light level down. Double-click: turn off
 * + : light level up.   Double-click: turn off
 * both : raise number of hours that the lights will stay on, up till 10.
 *        Then back down to 1, and repeat.
 */
void loop() {

  /*analogWrite(LED_STRIP, 180);
  return; */
  now_millis = millis();
  if (! digitalReadFast(DOWN_BUTTON)) {
    if (! down_button) {
      if ((now_millis - down_button_press) < 1000) {
        is_double_press = true;
      }
      down_button_press = now_millis;
    }
    down_button=true;
  } else {
    if (down_button) down_button_release = now_millis;
    down_button=false;
  }
  if (! digitalReadFast(UP_BUTTON)) {
    if (! up_button) {
      if ((now_millis - up_button_press) < 1000) {
        is_double_press = true;
      }
      up_button_press = now_millis;
    }
    up_button=true;
  } else {
    if (up_button) up_button_release = now_millis;
    up_button=false;
  }
  if ((! down_button) || (! up_button)) {
      is_double_press = false;
  }
  if (is_double_press) {
    current_millis = millis();
    turn_led_off();
    return;
  }
  // One or the other button was pressed.
  if (! led_on) turn_led_on();
  current_millis = millis();
  if (down_button) {
    waited=false;
    if (current_millis - last_update_millis > 50) {
      if (light_level > 100) light_level -= 5;
      else if (light_level > 50) light_level -= 2;
      else if (light_level > 0) light_level -= 1;
      else {
            set_light_level(LED_STRIP, 1);
            delay (50);
            set_light_level(LED_STRIP, 0);
            delay (50);
      }
      set_light_level(LED_STRIP, light_level);
      last_update_millis = current_millis;
    }
    return;
  }
  if (up_button) {
    if (current_millis - last_update_millis > 50) {
      if (light_level < 5) { 
        if (waited) {
          light_level += 1;
          waited=false;
        } else waited=true;
      }
      else if (light_level < 50) light_level += 1;
      else if (light_level < 80) light_level += 2;
      else if (light_level < 120) light_level += 5;
      else {
        light_level=255;
            set_light_level(LED_STRIP, 255);
            delay (50);
            set_light_level(LED_STRIP, 0);
            delay (50);
      }
      set_light_level(LED_STRIP, light_level);
      last_update_millis = current_millis;
    }
    return;
  }
}
