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

#define FOOTSW_RECEIVER_VER "1.0"

/* Radio driver. CE on pin A7, CSN on A6 */
RF24 nrf24(A7, A6);

/* Radio settings */
const uint64_t nrf24_receiver_txpipe   = 0x5a469bb4460f8939;
const uint64_t nrf24_footswitch_txpipe = 0x0731e80038efeede;

#define NRF24_CHANNEL       105
#define NRF24_PA            RF24_PA_MAX
#define NRF24_RATE          RF24_250KBPS
#define NRF24_PAYLOAD_SZ    8
#define RX_BLINK_MILLIS     100
#define CRC_LEN             2

/* MIDI transmitter. RX (unused) on pin 4, TX on pin 2 */
SoftwareSerial midi_port(4, 2);

/* Indicator LED */
#define LED_PIN 3

void halt(const char *msg) {
  Serial.println(msg);
  while (1) {
    digitalWrite(LED_PIN, LOW);
    delay(100);
    digitalWrite(LED_PIN, HIGH);
    delay(100);
  }
}

bool check_crc(const uint8_t *packet) {
	uint16_t crc = 0x00;

	for (size_t i=0; i < (NRF24_PAYLOAD_SZ - CRC_LEN); i++)
    crc = _crc16_update(crc, packet[i]);

  if (memcmp(&crc, packet + (NRF24_PAYLOAD_SZ - CRC_LEN), CRC_LEN))
    return false; /* Mismatch */

  return true;
}

void setup() {
  pinMode(LED_PIN, OUTPUT);
  digitalWrite(LED_PIN, LOW);

  Serial.begin(34800);
  midi_port.begin(31250);

  delay(1000);

  Serial.println("Footswitch receiver version " FOOTSW_RECEIVER_VER);
  Serial.println("Jaakko Salo / jaakkos@gmail.com / 2017");

  Serial.println("Initializing radio module...");
  if (!nrf24.begin()) halt("Unable to initialize radio module");
  nrf24.setChannel(NRF24_CHANNEL);
  nrf24.setAutoAck(1);
  nrf24.setPayloadSize(NRF24_PAYLOAD_SZ);
  nrf24.openWritingPipe(nrf24_receiver_txpipe);
  nrf24.openReadingPipe(1, nrf24_footswitch_txpipe);
  nrf24.startListening();

  if (nrf24.getChannel() != NRF24_CHANNEL)
    halt("Failed to set radio channel");

  if (nrf24.getPayloadSize() != NRF24_PAYLOAD_SZ)
    halt("Failed to set payload size");

  Serial.println("Radio module initialized. Current settings:");
  nrf24.printDetails();

  digitalWrite(LED_PIN, HIGH);
}

void parse_payload(const uint8_t *payload) {
  uint8_t msg_len = payload[0];

  if (msg_len > (NRF24_PAYLOAD_SZ - CRC_LEN)) {
    Serial.println("Received packet: invalid message lenth");
    return;
  }

  midi_port.write(payload, msg_len);

  Serial.print("Received and sent: ");
  for (size_t i = 0; i < msg_len; i++)
    Serial.print(payload[i], HEX);
  Serial.write('\n');
}

bool led_blink = false;
unsigned long led_blink_start;

void loop() {
  byte pipe;
  uint8_t buf[NRF24_PAYLOAD_SZ] = {0};

  if (led_blink && ((millis() - led_blink_start) > RX_BLINK_MILLIS)) {
    led_blink = false;
    digitalWrite(LED_PIN, HIGH);
  }

  if (nrf24.available(&pipe)) {
    led_blink = true;
    led_blink_start = millis();
    digitalWrite(LED_PIN, LOW);

    nrf24.read(buf, sizeof(buf));

    if (!check_crc(buf)) {
      Serial.println("Received packet: CRC mismatch, discarding");
      return;
    }

    else parse_payload(buf);
  }
}
