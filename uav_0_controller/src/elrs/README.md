# ELRS (ExpressLRS) Implementation

ExpressLRS implementation for ESP32 + SX1280 LoRa module.

## References

- **Official Repo**: https://github.com/ExpressLRS/ExpressLRS
- **Protocol Wiki**: https://github.com/ExpressLRS/ExpressLRS/wiki/Protocol
- **SX1280 Datasheet**: https://www.semtech.com/products/wireless-rf/24-ghz-transceivers/sx1280
- **RadioLib**: https://github.com/jgromes/RadioLib

## Files

- `elrs_tx.cpp/h` - Transmitter (ground station)
- `elrs_rx.cpp/h` - Receiver (UAV/aircraft)

## Hardware

- **MCU**: ESP32
- **Radio**: SX1280 2.4GHz LoRa module
- **Mode**: FLRC (Fast Long Range Communication)

## Pin Configuration

```
LORA_MISO   13
LORA_MOSI   11
LORA_SCK    12
LORA_NSS    10
LORA_RST    14
LORA_DIO1   9
LORA_BUSY   8
```

## Usage

### Transmitter (TX)
```cpp
#include "elrs/elrs_tx.h"

void setup() {
  ELRS_TX_Init();
  ELRS_TX_SetPacketRate(250);  // 250 Hz
}

void loop() {
  uint16_t channels[16] = {992, 992, 172, 992, ...};
  ELRS_TX_SetChannels(channels);
  ELRS_TX_Update();
}
```

### Receiver (RX)
```cpp
#include "elrs/elrs_rx.h"

void onNewData(uint16_t* channels) {
  // Process RC data
}

void setup() {
  ELRS_RX_Init();
  ELRS_RX_SetCallback(onNewData);
}

void loop() {
  ELRS_RX_Update();
  
  if (ELRS_RX_IsLinkUp()) {
    // Link active
  }
}
```

## Configuration

- **Frequency**: 2440 MHz
- **Data Rate**: 1.3 Mbps
- **Coding Rate**: 3/4
- **Power**: 10 dBm (adjustable)
- **Default Rate**: 250 Hz

## Channel Format

CRSF 11-bit format:
- Center: 992
- Low: 172
- High: 1811
