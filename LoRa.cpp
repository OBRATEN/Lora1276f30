#include <LoRa.hpp>

// Регистры
#define REG_FIFO                 0x00
#define REG_OP_MODE              0x01
#define REG_FRF_MSB              0x06
#define REG_FRF_MID              0x07
#define REG_FRF_LSB              0x08
#define REG_PA_CONFIG            0x09
#define REG_OCP                  0x0b
#define REG_LNA                  0x0c
#define REG_FIFO_ADDR_PTR        0x0d
#define REG_FIFO_TX_BASE_ADDR    0x0e
#define REG_FIFO_RX_BASE_ADDR    0x0f
#define REG_FIFO_RX_CURRENT_ADDR 0x10
#define REG_IRQ_FLAGS            0x12
#define REG_RX_NB_BYTES          0x13
#define REG_PKT_SNR_VALUE        0x19
#define REG_PKT_RSSI_VALUE       0x1a
#define REG_RSSI_VALUE           0x1b
#define REG_MODEM_CONFIG_1       0x1d
#define REG_MODEM_CONFIG_2       0x1e
#define REG_PREAMBLE_MSB         0x20
#define REG_PREAMBLE_LSB         0x21
#define REG_PAYLOAD_LENGTH       0x22
#define REG_MODEM_CONFIG_3       0x26
#define REG_FREQ_ERROR_MSB       0x28
#define REG_FREQ_ERROR_MID       0x29
#define REG_FREQ_ERROR_LSB       0x2a
#define REG_RSSI_WIDEBAND        0x2c
#define REG_DETECTION_OPTIMIZE   0x31
#define REG_INVERTIQ             0x33
#define REG_DETECTION_THRESHOLD  0x37
#define REG_SYNC_WORD            0x39
#define REG_INVERTIQ2            0x3b
#define REG_DIO_MAPPING_1        0x40
#define REG_VERSION              0x42
#define REG_PA_DAC               0x4d

// Режимы
#define MODE_LONG_RANGE_MODE     0x80
#define MODE_SLEEP               0x00
#define MODE_STDBY               0x01
#define MODE_TX                  0x03
#define MODE_RX_CONTINUOUS       0x05
#define MODE_RX_SINGLE           0x06

// PA
#define PA_BOOST                 0x80

// IRQ
#define IRQ_TX_DONE_MASK           0x08
#define IRQ_PAYLOAD_CRC_ERROR_MASK 0x20
#define IRQ_RX_DONE_MASK           0x40

#define RF_MID_BAND_THRESHOLD    525E6
#define RSSI_OFFSET_HF_PORT      157
#define RSSI_OFFSET_LF_PORT      164

#define MAX_PKT_LENGTH           255

// Работа с SPI
uint8_t LoRa_radio::readRegister(uint8_t addr) {
  return singleTransfer(addr & 0x7f, 0x00);
}

void LoRa_radio::writeRegister(uint8_t addr, uint8_t value) {
  singleTransfer(addr | 0x80, value);
}

uint8_t LoRa_radio::singleTransfer(uint8_t addr, uint8_t value) {
  uint8_t response;
  digitalWrite(_ss, LOW);
  _spi->beginTransaction(_spiSettings);
  _spi->transfer(addr);
  response = _spi->transfer(value);
  _spi->endTransaction();
  digitalWrite(_ss, HIGH);
  return response;
}

// Инициализация
LoRa_radio::LoRa_radio() :
  _spiSettings(LORA_DEFAULT_SPI_FREQUENCY, MSBFIRST, SPI_MODE0),
  _spi(&LORA_DEFAULT_SPI),
  _ss(LORA_DEFAULT_SS_PIN), _reset(LORA_DEFAULT_RESET_PIN), _dio0(LORA_DEFAULT_DIO0_PIN),
  _frequency(0),
  _packetIndex(0),
  _implicitHeaderMode(0),
  _onReceive(NULL),
  _onTxDone(NULL)
{
  // Таймаут потока
  setTimeout(0);
}

int LoRa_radio::begin(long frequency) {
  // Установка пинов
  pinMode(_ss, OUTPUT);
  digitalWrite(_ss, HIGH);
  if (_reset != -1) {
    pinMode(_reset, OUTPUT);
    digitalWrite(_reset, LOW);
    delay(10);
    digitalWrite(_reset, HIGH);
    delay(10);
  }
  _spi->begin();
  // Проверка версии
  uint8_t version = readRegister(REG_VERSION);
  if (version != 0x12) {
    return 0;
  }
  sleep();
  setFrequency(frequency);
  // Установка базовых адресов
  writeRegister(REG_FIFO_TX_BASE_ADDR, 0);
  writeRegister(REG_FIFO_RX_BASE_ADDR, 0);
  // Буст LNA
  writeRegister(REG_LNA, readRegister(REG_LNA) | 0x03);
  // Авто AGC
  writeRegister(REG_MODEM_CONFIG_3, 0x04);
  // 17 dBm
  setTxPower(17);
  idle();
  return 1;
}

void LoRa_radio::end() {
  sleep();
  _spi->end();
}


// Состояния
void LoRa_radio::sleep() {
  writeRegister(REG_OP_MODE, MODE_LONG_RANGE_MODE | MODE_SLEEP);
}

void LoRa_radio::idle() {
  writeRegister(REG_OP_MODE, MODE_LONG_RANGE_MODE | MODE_STDBY);
}

// Установки настроек
void LoRa_radio::setFrequency(long frequency) {
  _frequency = frequency;
  uint64_t frf = ((uint64_t)frequency << 19) / 32000000;
  writeRegister(REG_FRF_MSB, (uint8_t)(frf >> 16));
  writeRegister(REG_FRF_MID, (uint8_t)(frf >> 8));
  writeRegister(REG_FRF_LSB, (uint8_t)(frf >> 0));
}

void LoRa_radio::setTxPower(int level, int outputPin) {
  if (PA_OUTPUT_RFO_PIN == outputPin) {
    // RFO
    if (level < 0) {
      level = 0;
    } else if (level > 14) {
      level = 14;
    }
    writeRegister(REG_PA_CONFIG, 0x70 | level);
  } else {
    // Буст PA
    if (level > 17) {
      if (level > 20) level = 20;
      // Нужно вычесть 3 из уровня, чтобы 18 - 20 скалировало в 15 - 17
      level -= 3;
      // Высокое напряжение +20 dBm
      writeRegister(REG_PA_DAC, 0x87);
      setOCP(140);
    } else {
      if (level < 2) level = 2;
      // Стандартное значение PA_HF/LF или +17dBm
      writeRegister(REG_PA_DAC, 0x84);
      setOCP(100);
    }
    writeRegister(REG_PA_CONFIG, PA_BOOST | (level - 2));
  }
}

void LoRa_radio::setOCP(uint8_t mA) {
  uint8_t ocpTrim = 27;
  if (mA <= 120) ocpTrim = (mA - 45) / 5;
  else if (mA <=240) ocpTrim = (mA + 30) / 10;
  writeRegister(REG_OCP, 0x20 | (0x1F & ocpTrim));
}

// Работа с чтением/передачей
int LoRa_radio::beginPacket(int implicitHeader) {
  if (isTransmitting()) return 0;
  idle();
  if (implicitHeader) implicitHeaderMode();
  else explicitHeaderMode();
  // FIFO адрес и размер полезной нагрузки
  writeRegister(REG_FIFO_ADDR_PTR, 0);
  writeRegister(REG_PAYLOAD_LENGTH, 0);
  return 1;
}

bool LoRa_radio::isTransmitting() {
  if ((readRegister(REG_OP_MODE) & MODE_TX) == MODE_TX) return true;
  if (readRegister(REG_IRQ_FLAGS) & IRQ_TX_DONE_MASK)
  writeRegister(REG_IRQ_FLAGS, IRQ_TX_DONE_MASK);
  return false;
}

int LoRa_radio::endPacket(bool async) {
  if ((async) && (_onTxDone))
  writeRegister(REG_DIO_MAPPING_1, 0x40); // DIO0 => TXDONE
  // Режим TX
  writeRegister(REG_OP_MODE, MODE_LONG_RANGE_MODE | MODE_TX);
  if (!async) {
    // Ожидание конца передачи
    while ((readRegister(REG_IRQ_FLAGS) & IRQ_TX_DONE_MASK) == 0) yield();
    // Чистка IRQ
    writeRegister(REG_IRQ_FLAGS, IRQ_TX_DONE_MASK);
  }
  return 1;
}

size_t LoRa_radio::write(uint8_t byte) {
  return write(&byte, sizeof(byte));
}

size_t LoRa_radio::write(const uint8_t *buffer, size_t size) {
  int currentLength = readRegister(REG_PAYLOAD_LENGTH);
  // Проверка размера
  if ((currentLength + size) > MAX_PKT_LENGTH) size = MAX_PKT_LENGTH - currentLength;
  // Запись данных
  for (size_t i = 0; i < size; i++) writeRegister(REG_FIFO, buffer[i]);
  // Обновление размера полезной нагрузки
  writeRegister(REG_PAYLOAD_LENGTH, currentLength + size);
  return size;
}

int LoRa_radio::available() {
  return (readRegister(REG_RX_NB_BYTES) - _packetIndex);
}

int LoRa_radio::read() {
  if (!available()) return -1;
  _packetIndex++;
  return readRegister(REG_FIFO);
}

int LoRa_radio::peek()
{
  if (!available()) return -1;
  // Получаем текущий FIFO адрес
  int currentAddress = readRegister(REG_FIFO_ADDR_PTR);
  // Чтение
  uint8_t b = readRegister(REG_FIFO);
  // Перезаписываем FIFO адрес
  writeRegister(REG_FIFO_ADDR_PTR, currentAddress);
  return b;
}

void LoRa_radio::flush() {}

int LoRa_radio::parsePacket(int size) {
  int packetLength = 0;
  int irqFlags = readRegister(REG_IRQ_FLAGS);
  if (size > 0) {
    implicitHeaderMode();
    writeRegister(REG_PAYLOAD_LENGTH, size & 0xff);
  } else explicitHeaderMode();
  // Чистка IRQ
  writeRegister(REG_IRQ_FLAGS, irqFlags);
  if ((irqFlags & IRQ_RX_DONE_MASK) && (irqFlags & IRQ_PAYLOAD_CRC_ERROR_MASK) == 0) {
    _packetIndex = 0;

    // Читаем длину пакета
    if (_implicitHeaderMode) packetLength = readRegister(REG_PAYLOAD_LENGTH);
    else packetLength = readRegister(REG_RX_NB_BYTES);
    // FIFO адрес на текущий RX
    writeRegister(REG_FIFO_ADDR_PTR, readRegister(REG_FIFO_RX_CURRENT_ADDR));
    idle();
  } else if (readRegister(REG_OP_MODE) != (MODE_LONG_RANGE_MODE | MODE_RX_SINGLE)) {
    // Перезапись FIFO
    writeRegister(REG_FIFO_ADDR_PTR, 0);

    // Перевод в режим single RX
    writeRegister(REG_OP_MODE, MODE_LONG_RANGE_MODE | MODE_RX_SINGLE);
  }
  return packetLength;
}

void LoRa_radio::explicitHeaderMode() {
  _implicitHeaderMode = 0;
  writeRegister(REG_MODEM_CONFIG_1, readRegister(REG_MODEM_CONFIG_1) & 0xfe);
}

void LoRa_radio::implicitHeaderMode() {
  _implicitHeaderMode = 1;
  writeRegister(REG_MODEM_CONFIG_1, readRegister(REG_MODEM_CONFIG_1) | 0x01);
}