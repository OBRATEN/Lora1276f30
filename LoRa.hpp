#ifndef LORA_H
#define LORA_H

#include <Arduino.h>
#include <SPI.h>

#define LORA_DEFAULT_SPI           SPI
#define LORA_DEFAULT_SPI_FREQUENCY 8E6 
#define LORA_DEFAULT_SS_PIN        10
#define LORA_DEFAULT_RESET_PIN     9
#define LORA_DEFAULT_DIO0_PIN      2

#define PA_OUTPUT_RFO_PIN          0
#define PA_OUTPUT_PA_BOOST_PIN     1

class LoRa_radio : public Stream {
public:
  LoRa_radio();
  int begin(long frequency);
  void end();

  // Print
  virtual size_t write(uint8_t byte);
  virtual size_t write(const uint8_t *buffer, size_t size);

  // Stream
  virtual int available();
  virtual int read();
  virtual int peek();
  virtual void flush();

  int parsePacket(int size = 0);
  int beginPacket(int implicitHeader = false);
  int endPacket(bool async = false);

  void idle();
  void sleep();

  void setTxPower(int level, int outputPin = PA_OUTPUT_PA_BOOST_PIN);
  void setFrequency(long frequency);
  void setOCP(uint8_t mA); // Over Current Protection control

private:
  void explicitHeaderMode();
  void implicitHeaderMode();
  bool isTransmitting();
  uint8_t readRegister(uint8_t address);
  void writeRegister(uint8_t address, uint8_t value);
  uint8_t singleTransfer(uint8_t address, uint8_t value);

protected:
  SPISettings _spiSettings;
  SPIClass* _spi;
  int _ss;
  int _reset;
  int _dio0;
  long _frequency;
  int _packetIndex;
  int _implicitHeaderMode;
  void (*_onReceive)(int);
  void (*_onTxDone)();
};

extern LoRa_radio LoRa;

#endif
