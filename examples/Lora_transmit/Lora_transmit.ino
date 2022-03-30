#include <SPI.h>
#include <LoRa.hpp>

LoRa_radio Lora;

static uint32_t counter = 0;

void setup() {
  Lora.begin(915E6);
}

void loop() {
  Lora.beginPacket();
  Lora.print("Packet: ");
  Lora.print(counter);
  Lora.endPacket();
  counter++;
}