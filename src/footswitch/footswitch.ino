/*
 * The MIT License (MIT)
 *
 * Copyright (c) 2017 Jaakko Salo (jaakkos@gmail.com / jaakkos on Freenode)
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in
 * all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 */

#include <SPI.h>
#include <util/crc16.h>
#include <avr/sleep.h>
#include "RF24.h"
#include "printf.h"

#define FOOTSW_VER "1.0"

/* Radio driver. CE on pin D2, CSN on D3 */
RF24 nrf24(2, 3);

/* Radio settings */
const uint64_t nrf24_receiver_txpipe   = 0x5a469bb4460f8939;
const uint64_t nrf24_footswitch_txpipe = 0x0731e80038efeede;

#define NRF24_CHANNEL       105
#define NRF24_PA            RF24_PA_MAX
#define NRF24_RATE          RF24_250KBPS
#define NRF24_PAYLOAD_SZ    8
#define CRC_LEN             2

/* LED segment pins */
const char led_segments[] = {A1, A2, A3, A4, A5, 6, 7};

/*
 * Array index to segment mapping:
 *
 *   +---1---+
 *   |       |
 *   6       2
 *   |       |
 *   +---7---+
 *   |       |
 *   5       3
 *   |       |
 *   +---4---+
 *
 */

/* LED display buffers for multiplex driver, one bit per segment. */
volatile uint8_t preset_buf[2] = {0}; /* Preset display buffer */
volatile uint8_t pot_buf[2]    = {0}; /* Potentiometer display buffer */
volatile uint8_t *led_buffer = preset_buf;

/* Numbers from 0 to 9 */
const uint8_t led_numbers[] = {0x3F, 0x06, 0x5B, 0x4F, 0x66, 0x6D, 0x7D, 0x07,
                               0xFF, 0xEF};

/* LED multiplex digit select pins (FET driver) */
const char led_mplex_pins[] = {10 /* OC1B */, 9 /* OC1A */};

/* LED multiplexer current displayed digit */
volatile int cur_digit = 0;

/* Footswitch button pins */
#define FOOTSW_BUTTON_1 4
#define FOOTSW_BUTTON_2 5

/* Potentiometer setup */
#define FOOTSW_POT_SW             8
#define FOOTSW_POT_ADC            A7
#define FOOTSW_POT_LOWPASS_ALPHA  0.98
#define FOOTSW_POT_DEADZONE       2
#define FOOTSW_POT_MIN_CHANGE     0.4

/* Button state. Bits raised from the interrupt handler */
volatile uint8_t button_state = 0x00;
#define BTN1_STATE_BIT   0x01
#define BTN2_STATE_BIT   0x02
#define POT_SW_STATE_BIT 0x04

/* Button debounce timer (Timer2 ticks, 488 Hz) */
#define BTN_DEBOUNCE_TICKS 20
unsigned int btn1_debounce_ticks = 0;
unsigned int btn2_debounce_ticks = 0;

/* Time to stay awake when there is no user activity (ms) */
#define IDLE_ON_TIME 10000

/* Timestamp for last user activity */
unsigned long last_activity_time = 0;

/* Current preset */
uint8_t preset = 0;

void halt(const char *msg) {
  Serial.println(msg);
  while (1) {
    noInterrupts();
    led_buffer[0] = led_buffer[1] = 0x00;
    interrupts();
    delay(100);

    noInterrupts();
    led_buffer[0] = led_buffer[1] = 0xFF;
    interrupts();
    delay(100);
  }
}

/* Multiplex driver timer interrupt */
ISR(TIMER2_OVF_vect) {
  /* Turn off all drive */
  TCCR1A &= ~(1 << COM1A1);
  TCCR1A &= ~(1 << COM1B1);

  /* Switch digit */
  cur_digit = !cur_digit;

  /* Set up LEDs */
  for (size_t i = 0; i < 7; i++)
    digitalWrite(led_segments[i], led_buffer[cur_digit] & (1 << i));

  /* Drive one of the displays */
  if (cur_digit) TCCR1A |= (1 << COM1A1);
  else           TCCR1A |= (1 << COM1B1);

  /* Update button debounce timers */
  if (btn1_debounce_ticks) btn1_debounce_ticks--;
  if (btn2_debounce_ticks) btn2_debounce_ticks--;
}

void display_setup() {
  /* Initialize LED multiplexer, shut down display */
  pinMode(led_mplex_pins[0], OUTPUT);
  digitalWrite(led_mplex_pins[0], LOW);

  pinMode(led_mplex_pins[1], OUTPUT);
  digitalWrite(led_mplex_pins[1], LOW);

  /* Initialize LED drive */
  for(size_t i = 0; i < sizeof(led_segments); i++) {
    pinMode(led_segments[i], OUTPUT);
    digitalWrite(led_segments[i], LOW);
  }

  noInterrupts();

  /* Setup Timer2 to generate multiplex interrupts */
  TCCR2A = 0;                              /* Zero TC2 control registers */
  TCCR2B = 0;
  TCNT2 = 0;                               /* Initialize counter value */
  TCCR2B |= (1 << CS22);                   /* 1/64 prescaler, 488 Hz */
  TIMSK2 |= (1 << TOIE2);                  /* Enable overflow interrupt */

  /* Setup Timer1 to generate intensity control PWM */
  TCCR1A = 0;                              /* Zero TC1 control registers */
  TCCR1B = 0;
  TCCR1A |= (1 << WGM10) | (1 << WGM12);   /* Fast PWM 8 bit mode, 31.25 kHz */
  TCNT1 = 0;                               /* Initialize counter value */
  TCCR1B |= (1 << CS10);                   /* Prescaler off */
  OCR1A = OCR1B = 255;                     /* Max. intensity */

  interrupts();
}

ISR(PCINT0_vect) {
  /* We don't care about what happened, just that something happened */
  button_state |= POT_SW_STATE_BIT;
}

bool btn_read_debounce(unsigned int  *debounce_ticks,
                       uint8_t       *oldstate,
                       uint8_t        pin)
{
  bool pressed = false;

  if (digitalRead(pin)) {
    if (!(*oldstate)) {
      /* State changed - debounce time passed? */
      if (!(*debounce_ticks))
        pressed = true;

      /* Reset debounce timer on every state change */
      *debounce_ticks = BTN_DEBOUNCE_TICKS;
    }

    *oldstate = 1;
  }

  else {
    if (*oldstate) {
      /* Reset debounce timer on every state change */
      *debounce_ticks = BTN_DEBOUNCE_TICKS;
    }

    *oldstate = 0;
  }

  return pressed;
}

ISR(PCINT2_vect) {
  static uint8_t btn1_state = 0;
  static uint8_t btn2_state = 0;

  if (btn_read_debounce(&btn1_debounce_ticks, &btn1_state, FOOTSW_BUTTON_1))
    button_state |= BTN1_STATE_BIT;

  if (btn_read_debounce(&btn2_debounce_ticks, &btn2_state, FOOTSW_BUTTON_2))
    button_state |= BTN2_STATE_BIT;
}

void input_setup() {
  /* Initialize buttons, internal pullups disabled by default */
  pinMode(FOOTSW_BUTTON_1, INPUT);
  pinMode(FOOTSW_BUTTON_2, INPUT);
  pinMode(FOOTSW_POT_SW, INPUT);

  noInterrupts();
  PCMSK0 |= (1 << PCINT0);  /* Add PCINT0 to PCINT0 vector */
  PCMSK2 |= (1 << PCINT20); /* Add PCINT20 to PCINT2 vector */
  PCMSK2 |= (1 << PCINT21); /* Add PCINT21 to PCINT2 vector */
  PCICR  |= (1 << PCIE0);   /* Enable PCINT0 interrupt */
  PCICR  |= (1 << PCIE2);   /* Enable PCINT2 interrupt */
  interrupts();
}

void radio_setup() {
  Serial.println("Initializing radio module...");
  if (!nrf24.begin()) halt("Unable to initialize radio module");

  nrf24.setChannel(NRF24_CHANNEL);
  nrf24.setDataRate(NRF24_RATE);
  nrf24.setAutoAck(1);
  nrf24.setPayloadSize(NRF24_PAYLOAD_SZ);
  nrf24.openWritingPipe(nrf24_footswitch_txpipe);
  nrf24.openReadingPipe(1, nrf24_receiver_txpipe);
  nrf24.stopListening();

  if (nrf24.getChannel() != NRF24_CHANNEL)
    halt("Failed to set radio channel");

  if (nrf24.getPayloadSize() != NRF24_PAYLOAD_SZ)
    halt("Failed to set payload size");

  Serial.println("Radio module initialized. Current settings:");
  nrf24.printDetails();
}

void setup() {
  display_setup();
  input_setup();

  Serial.begin(38400);
  printf_begin();

  Serial.println("Footswitch version " FOOTSW_VER);
  Serial.println("Jaakko Salo / jaakkos@gmail.com / 2017");

  radio_setup();
}

/*
 * Packet format:
 *
 *  +-------------+-------------------------------+----------------+
 *  | Payload len |      Payload + padding        |      CRC16     |
 *  +-------------+-------------------------------+----------------+
 *      1 byte       NRF24_PAYLOAD_SZ - 3 bytes         2 bytes
 *
 */
bool sendmsg(const char *msg, size_t sz) {
  char msg_out[NRF24_PAYLOAD_SZ] = {0};

  msg_out[0] = sz;
  memcpy(msg_out+1, msg, sz);

  uint16_t crc = 0x00;

  for (size_t i=0; i < (NRF24_PAYLOAD_SZ - CRC_LEN); i++)
    crc = _crc16_update(crc, msg_out[i]);

  memcpy(msg_out+(NRF24_PAYLOAD_SZ - CRC_LEN), &crc, CRC_LEN);

  if (!nrf24.write(msg_out, NRF24_PAYLOAD_SZ)){
    Serial.println("Failed to send/receive ACK");
    return false;
  }

  else return true;
}

void deep_sleep() {
  set_sleep_mode(SLEEP_MODE_PWR_DOWN);
  cli();

  /* Re-check sleep condition to avoid race */
  if (button_state) return;

  /* Shut down display before going to sleep */
  TCCR1A &= ~(1 << COM1A1);
  TCCR1A &= ~(1 << COM1B1);
  preset_buf[0] = preset_buf[1] = 0x00;

  /* Reset debounce timers */
  btn1_debounce_ticks = 0;
  btn2_debounce_ticks = 0;

  sleep_enable();
  sleep_bod_disable();
  sei();
  sleep_cpu(); /* Guaranteed to execute before any interrupts */
  sleep_disable();
}

void potentiometer_mode() {
  static float filtered = 0.0;
  static float filtered_noflap = 0.0;

  int ain = analogRead(FOOTSW_POT_ADC);

  /* Map result to [0, 100] */
  float new_val = 100.0 * (float)ain / 1023.0;

  /* Lowpass filtering */
  filtered = FOOTSW_POT_LOWPASS_ALPHA * filtered +
             (1.0 - FOOTSW_POT_LOWPASS_ALPHA) * new_val;

  /* Prevent flapping by requiring minimum change */
  if (fabs(filtered - filtered_noflap) > FOOTSW_POT_MIN_CHANGE) {
    filtered_noflap = filtered;

    /* Activity detected, so prevent sleep */
    last_activity_time = millis();
  }

  /* Add some dead zone */
  int val = map(filtered_noflap*10.0, 0, 1000, -FOOTSW_POT_DEADZONE*10,
                1000 + FOOTSW_POT_DEADZONE*10);
  if (val < 0) val = 0;
  else if (val > 999) val = 999;
  val /= 10;

  /* Update led intensity */
  OCR1A = OCR1B = 255 * (1.0 + sin(millis()/70.0))/2.0;

  /* Update LED display buffer */
  noInterrupts();
  pot_buf[0] = led_numbers[val / 10];
  pot_buf[1] = led_numbers[val % 10];
  led_buffer = pot_buf;
  interrupts();
}

void update_preset_display() {
  noInterrupts();
  preset_buf[0] = 0x73; /* P */
  preset_buf[1] = led_numbers[preset % 10];
  interrupts();
}

void btn1_pressed() {
  if (preset > 0) preset--;
  update_preset_display();
}

void btn2_pressed() {
  if (preset < 9)  preset++;
  update_preset_display();
}

void preset_display_mode() {
  /* Set LEDs to intensity based on remaining wake-up time */
  int32_t remaining = IDLE_ON_TIME - (millis() - last_activity_time);
  if (remaining < 0) remaining = 0;
  if (remaining > 1000) OCR1A = OCR1B = 255;
  else OCR1A = OCR1B = ((int32_t)255 * remaining) / (int32_t)1000;

  /* Show preset display buffer */
  noInterrupts();
  led_buffer = preset_buf;
  interrupts();
}

void loop() {
  const char msg1[] = "\xB0\x1f\x01";
  const char msg2[] = "\xB0\x1f\x21";
  const char msg3[] = "\xB0\x1f\x41";
  const char msg4[] = "\xB0\x1f\x61";

  while (millis() - last_activity_time < IDLE_ON_TIME) {
    /* Get button events */
    noInterrupts();
    uint8_t cur_state = button_state;
    button_state = 0x00;
    interrupts();
    if (cur_state) last_activity_time = millis();

    /* Potentiometer display mode*/
    if (digitalRead(FOOTSW_POT_SW))
      potentiometer_mode();

    /* Preset display mode */
    else preset_display_mode();

    /* Always handle button events */
    if      (cur_state & BTN1_STATE_BIT) btn1_pressed();
    else if (cur_state & BTN2_STATE_BIT) btn2_pressed();
  }

  deep_sleep();
  last_activity_time = millis();
}
