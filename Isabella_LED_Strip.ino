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
// s == seven segment display pin, l == led strip pin, u == up button, d == down button,
// h == hours button, x=debug signal, y=debug clock.
//
//                           +-\/-+
//     RESET   (D 22)  PC6  1|   s|28  PC5  (D 19  A5  SCL ADC5)
//             (D  0)  PD0  2|s  s|27  PC4  (D 18  A4  SDA ADC4)
//             (D  1)  PD1  3|s  s|26  PC3  (D 17  A3  ADC3)
//       INT0  (D  2)  PD2  4|u  s|25  PC2  (D 16  A2  ADC2)
//       INT1  (D  3)  PD3  5|d  l|24  PC1  (D 15  A1  ADC1)
//             (D  4)  PD4  6|s  h|23  PC0  (D 14  A0  ADC0)
//                     VCC  7|    |22  GND
//             (D 21)  PB7 10|    |19  PB5  (D 13  SCK) 
//             (D  5)  PD5 11|    |18  PB4  (D 12  MISO) 
//  AIN0       (D  6)  PD6 12|x   |17  PB3  (D 11  MOSI OC2)
//  AIN1       (D  7)  PD7 13|y   |16  PB2  (D 10  SS OC1A)
//             (D  8)  PB0 14|    |15  PB1  (D  9     OC1B)
//                           +----+
#include <avr/interrupt.h>
#include "digitalWriteFast.h"

/* 7 Segment LED Display - Model 5611H
   http://www.xlitx.com/datasheet/5611BH.pdf
   3, 8: VCC
   a-7, b-6, c-4, d-2, e-1, f-9, g-10, dp-5, pin 1 is to the left of d
        / a \
        f   b
        | g |
        e   c
        \ d / dp
*/
/* 2N7000 transistor
 *  ___
 * /___\
 * S G D
 *
 */

MULTIPLEX_MAX=100; // PWM over this number. The PWM output is a portion of this.

// Requires pgm_read_byte() to get it out. Costs 1 extra cycle. Not worth it on slower CPUs.
// const uint8_t PROGMEM segment_pins[] = {0, 13, 12, 11, 1, 4, 20, 10};
const uint8_t segment_pins[] = {4, 20, 17, 18, 19, 1, 0, 16};
const uint8_t LED_STRIP = 15;
const uint8_t DOWN_BUTTON = 2; // INT0
const uint8_t UP_BUTTON = 3;   // INT1
const uint8_t HOURS_BUTTON = 14;
// These must be on the same port.
const uint8_t DEBUG_X = 6; // bit indicator for debugging
const uint8_t DEBUG_Y = 7; // Clock for the above
const uint8_t x_bit = digitalPinToBit(6);
const uint8_t y_bit = digitalPinToBit(7);
const uint8_t debug_hi = (1 << digitalPinToBit(6)) | (1 << digitalPinToBit(7));
const uint8_t debug_lo = (1 << digitalPinToBit(7));
const uint8_t debug_rst = (uint8_t (~debug_hi)) & (uint8_t (~debug_lo));

uint8_t led_on=false;

volatile uint8_t indicator_value=0;
volatile uint8_t light_level=0;
volatile uint8_t multiplex_sequence=0;
volatile uint8_t segment_bitmap;
uint8_t seven_segment_value=3;

volatile uint8_t segment_state, segment_pin, current_segment = 0;
volatile uint8_t seven_segment_display_update=0;
volatile uint8_t display_bit=0b10000000;

//#define TOGGLE_DEBUG_PIN bitWrite(PORTB, 1, 1); bitWrite(PORTB, 1, 0)
//#define DEBUG_BIT(V) (PORTB |= ((1 << DDB1) & (V)) | (1 << PORTB)); DDRB &= 0b11111001
//#define DEBUG_BIT(V) ((V) != 0) ? PORTD |= 0b11000000 : (PORTD |= 0b10000000); PORTD &= 0b00111111
//#define DEBUG_BIT(V) ((V) != 0) ? PORTD |= debug_hi : (PORTD |= debug_lo); PORTD &= debug_rst
#define DEBUG_BIT(V) ((V) != 0) ? *(digitalPinToPortReg(DEBUG_X)) |= debug_hi : (*(digitalPinToPortReg(DEBUG_X)) |= debug_lo); *(digitalPinToPortReg(DEBUG_X)) &= debug_rst

static inline void isr_display_value(uint8_t value) {
  uint8_t i;
  for (i=0b10000000; i > 0; i = i>>1) {
    DEBUG_BIT(i & value);
  }
}

// On is LOW...
#define SEGMENT_OFF 1
#define SEGMENT_ON 0
//ISR(timer2_ovf_vect) {
ISR(TIMER2_COMP_vect) {
  DEBUG_BIT(1);
  DEBUG_BIT(0);
  DEBUG_BIT(0);
  // do seven segment display

  // NOOP ************************
  // __asm__ __volatile__ ("nop");

  // segment_state = ( (segments[seven_segment_value] & display_bit) > 1 ) ? SEGMENT_ON : SEGMENT_OFF;
  segment_pin = segment_pins[current_segment];
  /* if (current_segment == 1) {
    isr_display_value(current_segment);
    isr_display_value(segment_pin);
  } */
  //isr_display_value(display_bit);
  //isr_display_value(segment_bitmap);

  if ((segment_bitmap & display_bit) != 0) {
    digitalWriteFast(segment_pin, SEGMENT_ON);
  }
  else {
    digitalWriteFast(segment_pin, SEGMENT_OFF);
  }
  display_bit >>= 1; current_segment++;
  if (display_bit == 0) {
    display_bit = 0b10000000; current_segment=0;
  }
  multiplex_sequence++; if (multiplex_sequence == MULTIPLEX_MAX) multiplex_sequence=0;
  DEBUG_BIT(1);
  DEBUG_BIT(1);
  DEBUG_BIT(1);
}

void set_all_pins_input() {
  pinMode(LED_STRIP, INPUT_PULLUP);
  for (uint8_t i=0; i < 8; i++) {
    pinMode(segment_pins[i], INPUT);
  }
}

void indicate (uint8_t value) {
  // debugging
  uint8_t indicator=16;

  if (value==0) value=10;
  pinMode(indicator, OUTPUT);
  for (uint8_t j=0; j < value; j ++) {
    digitalWriteFast(indicator, SEGMENT_ON); // on
    delay (200);
    digitalWriteFast(indicator, SEGMENT_OFF); // off
    delay (200);
  }
  delay(500);
  //
}

void set_pin_directions(void) {
  pinMode(LED_STRIP, OUTPUT); digitalWriteFast(LED_STRIP, 0);
  uint8_t i;
  indicate (4);
  for (i=0; i <= 7; i++) {
    pinMode(segment_pins[i], OUTPUT);
    digitalWriteFast(segment_pins[i], SEGMENT_OFF);
  }
  pinMode(DEBUG_X, OUTPUT);
  pinMode(DEBUG_Y, OUTPUT);
  pinMode(DOWN_BUTTON, INPUT);
  pinMode(UP_BUTTON, INPUT);
  pinMode(HOURS_BUTTON, INPUT);
}

const uint8_t segments[]={
   // abcdefg.
    0b11111100, // 0
    0b01100000, // 1
    0b11011010, // 2
    0b11110010, // 3
    0b01100110, // 4
    0b10110110, // 5
    0b10111110, // 6
    0b11100000, // 7
    0b11111110, // 8
    0b11110110, // 9
    0b00100001  // decimal point + segment c, to represent "10"
};

// the setup function runs once when you press reset or power the board
uint32_t current_millis=0;
void setup() {
  segment_bitmap=segments[seven_segment_value];
  //delay(100);
  // Set up timer 2
  SFIOR |= (1 << PSR2);  // reset prescaler. Not sure I need to do this.
  TCCR2 = 0;
      // 0x7B == *approximately* 1024 times per second at 8MHz clock
  OCR2 = 0x08;  // 8 MHz clock. f = 8,000,000 / (2 * prescalar * (1 + OCR2)
                // then divide by 256.
  TCCR2 &= ~(1 << COM20); // shut off output pin
  TCCR2 &= ~(1 << COM21); // shut off output pin
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
  segment_bitmap=segments[seven_segment_value];
  now_millis = millis();
  if (! digitalReadFast(DOWN_BUTTON)) {
    if (! down_button) {
      down_button_press = now_millis;
    }
    down_button=true;
  } else {
    if (down_button) down_button_release = now_millis;
    down_button=false;
  }
  if (! digitalReadFast(UP_BUTTON)) {
    if (! up_button) {
      up_button_press = now_millis;
    }
    up_button=true;
  } else {
    if (up_button) up_button_release = now_millis;
    up_button=false;
  }
  // One or the other button was pressed.
  current_millis = millis();
  if (! digitalReadFast(HOURS_BUTTON)) {
    if (now_millis - last_update_millis > 750) {
      if (seven_segment_value < 10) {
        seven_segment_value++;
      } else {
        seven_segment_value = 0;
      }
      last_update_millis = current_millis;
    }
    return;
  }
  if (down_button) {
    waited=false;
    if (current_millis - last_update_millis > 50) {
      if (light_level > 100) light_level -= 5;
      else if (light_level > 50) light_level -= 2;
      else if (light_level > 0) light_level -= 1;
      else {
            light_level=1;
            delay (50);
            light_level=0;
            delay (50);
      }
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
        delay (50);
        light_level=0;
        delay (50);
        light_level=255;
      }
      last_update_millis = current_millis;
    }
    return;
  }
}
