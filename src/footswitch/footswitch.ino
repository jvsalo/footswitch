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
#include <SoftwareSerial.h>
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

/* LED display buffer for multiplex driver, one bit per segment. */
volatile uint8_t led_buffer[2] = {0};

/* Numbers from 0 to 9 */
const char led_numbers[] = {0x3F, 0x06, 0x5B, 0x4F, 0x66, 0x6D, 0x7D, 0x07,
                            0xFF, 0xEF};

/* LED multiplex digit select pins (FET driver) */
const char led_mplex_pins[] = {10, 9};

/* Footswitch button pins */
#define FOOTSW_BUTTON_1 4
#define FOOTSW_BUTTON_2 5

/* Potentiometer pins */
#define FOOTSW_POT_SW 8
#define FOOTSW_POT_ADC A7

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
ISR(TIMER1_COMPA_vect) {
  /* Current enabled digit */
  static int cur_digit = 0;

  /* Turn off all drive */
  digitalWrite(led_mplex_pins[cur_digit], LOW);

  /* Switch digit */
  cur_digit = !cur_digit;

  /* Set up LEDs */
  for (size_t i = 0; i < 7; i++)
    digitalWrite(led_segments[i], led_buffer[cur_digit] & (1 << i));

  /* Turn on drive */
  digitalWrite(led_mplex_pins[cur_digit], HIGH);
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

  /* Setup Timer1 to generate multiplex interrupts */
  noInterrupts();
  TCCR1A = 0;               /* Zero TC1 control registers */
  TCCR1B = 0;
  TCNT1  = 0;               /* Initialize counter value */
  OCR1A = 16000;            /* At 8MHz, yields 2000 Hz */
  TCCR1B |= (1 << WGM12);   /* Clear Timer on Compare match (CTC) mode */
  TCCR1B |= (1 << CS10);    /* No prescaler */
  TIMSK1 |= (1 << OCIE1A);  /* Enable compare match interrupt */
  interrupts();
}

void input_setup() {
  /* Initialize buttons, internal pullups disabled by default */
  pinMode(FOOTSW_BUTTON_1, INPUT);
  pinMode(FOOTSW_BUTTON_2, INPUT);
  pinMode(FOOTSW_POT_SW, INPUT);
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
 *       1 byte       NRF24_PAYLOAD_SZ - 3 bytes         2 bytes
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

void loop() {
  const char msg1[] = "\xB0\x1f\x01";
  const char msg2[] = "\xB0\x1f\x21";
  const char msg3[] = "\xB0\x1f\x41";
  const char msg4[] = "\xB0\x1f\x61";

  for (size_t i = 0; i < 100; i++) {
    noInterrupts();
    led_buffer[0] = led_numbers[i/10];
    led_buffer[1] = led_numbers[i%10];
    interrupts();
    delay(100);
  }
}
