#include <SPI.h>
#include <LoRa.hpp>

LoRa_radio Lora;

void setup() {
  Serial.begin(9600);
  Serial.println("Приём данных Lora");
  if (!Lora.begin(915E6)) Serial.println("Ошибка при старте!");
}

void loop() {
  int packetSize = Lora.parsePacket();
  if (packetSize) {
    Serial.print("Received packet: ");
    while (Lora.available()) {
      Serial.print((char)Lora.read());
    }
  }
}