# CRSF Protocol Reference

## Overview

CRSF is the protocol used by:
- TBS Crossfire
- ExpressLRS
- Betaflight
- iNav
- This project

**Official Spec**: https://github.com/crsf-wg/crsf/wiki

## Channel Format

### 11-bit Channels
CRSF uses 11-bit channel values (0-2047):

| Value | PWM Equivalent | Meaning |
|-------|----------------|---------|
| 172 | 1000µs | Minimum |
| 992 | 1500µs | Center/Neutral |
| 1811 | 2000µs | Maximum |

### Conversion Formula

```cpp
// PWM to CRSF
CRSF = ((PWM - 1000) / 1000) * 1639 + 172

// CRSF to PWM
PWM = ((CRSF - 172) / 1639) * 1000 + 1000

// Examples:
1000µs → 172   (min)
1500µs → 992   (center)
2000µs → 1811  (max)
```

### Normalized to CRSF

```cpp
// Normalized (-1.0 to 1.0) to CRSF
CRSF = (normalized + 1.0) * 819.5 + 172

// Normalized (0.0 to 1.0) to CRSF (throttle)
CRSF = normalized * 1639 + 172

// Examples:
-1.0 → 172    (full left/down)
 0.0 → 992    (center)
+1.0 → 1811   (full right/up)
```

## Standard Channel Mapping

```
Channel 0:  Roll      (AETR order)
Channel 1:  Pitch
Channel 2:  Throttle
Channel 3:  Yaw

Channel 4:  Arm switch
Channel 5:  Flight mode
Channel 6:  Angle/Acro mode
Channel 7:  Beeper
Channel 8+: Aux channels
```

**Note**: Channel order varies by configuration. Common orders:
- **AETR**: Aileron, Elevator, Throttle, Rudder (most common)
- **TAER**: Throttle, Aileron, Elevator, Rudder

## CRSF Frame Structure

### RC Channels Frame (0x16)

```
Byte 0:    Destination Address (0xC8 = Flight Controller)
Byte 1:    Frame Length (24 for RC channels)
Byte 2:    Frame Type (0x16 = RC Channels)
Byte 3-24: Packed channel data (16 channels × 11 bits = 176 bits = 22 bytes)
Byte 25:   CRC8
```

### Channel Packing

16 channels of 11-bit data packed into 22 bytes:

```cpp
// Channel layout (bit positions)
Ch0:  bits 0-10
Ch1:  bits 11-21
Ch2:  bits 22-32
Ch3:  bits 33-43
...
Ch15: bits 165-175
```

**Implementation**: See `CrsfBridge_PackChannels()` in `crsf_bridge.cpp`

## CRC8 Calculation

CRSF uses CRC8-DVB-S2 polynomial (0xD5):

```cpp
uint8_t crc8(const uint8_t* data, size_t len) {
  uint8_t crc = 0x00;
  for (size_t i = 0; i < len; i++) {
    crc ^= data[i];
    for (int b = 0; b < 8; b++) {
      crc = (crc & 0x80) ? ((crc << 1) ^ 0xD5) : (crc << 1);
    }
  }
  return crc;
}
```

**CRC covers**: Frame type + payload (NOT address or length)

## Serial Configuration

```
Baud Rate: 420000
Data Bits: 8
Parity:    None
Stop Bits: 1
Format:    8N1
```

## Frame Types

| Type | Hex | Name | Direction |
|------|-----|------|-----------|
| RC Channels | 0x16 | RC_CHANNELS_PACKED | TX → FC |
| Link Statistics | 0x14 | LINK_STATISTICS | RX → TX |
| Battery Sensor | 0x08 | BATTERY_SENSOR | FC → TX |
| GPS | 0x02 | GPS | FC → TX |
| Attitude | 0x1E | ATTITUDE | FC → TX |
| Flight Mode | 0x21 | FLIGHT_MODE | FC → TX |

**This project implements**: RC Channels (0x16) only

## Addresses

```
0xC8: Flight Controller (destination for RC data)
0xEA: Radio Transmitter
0xEC: Receiver
0xEE: Transmitter Module
```

## Timing

### Recommended Update Rates

| Rate | Period | Use Case |
|------|--------|----------|
| 50 Hz | 20ms | Standard (most reliable) |
| 100 Hz | 10ms | Low latency |
| 150 Hz | 6.67ms | Racing (if supported) |
| 250 Hz | 4ms | Extreme low latency |

**This project default**: 50 Hz (configurable)

### Jitter Tolerance
- Betaflight can handle irregular CRSF timing
- Try to maintain consistent intervals
- Max gap before failsafe: ~1 second

## Failsafe Values

Standard failsafe configuration:

```cpp
channels[0] = 992;  // Roll - CENTER
channels[1] = 992;  // Pitch - CENTER  
channels[2] = 172;  // Throttle - LOW (critical!)
channels[3] = 992;  // Yaw - CENTER
channels[4] = 172;  // Arm - DISARMED
channels[5+] = 172; // Aux - LOW/OFF
```

**Key**: Always set throttle to 172 (minimum) for safety!

## Betaflight Configuration

### 1. Enable CRSF

In Betaflight Configurator:

**Ports Tab**:
- Select UART (e.g., UART1)
- Enable "Serial RX"
- Save and Reboot

**Configuration Tab**:
- Receiver: "Serial-based receiver"
- Serial Receiver Provider: "CRSF"
- Save and Reboot

### 2. Verify Connection

**Receiver Tab**:
- Should show moving channel bars
- Test all channels
- Set up channel map if needed

### 3. Failsafe

**Failsafe Tab**:
- Failsafe Stage 1: Drop
- Failsafe Stage 2: Set throttle to minimum
- Configure channel fallback values

### 4. CLI Commands

```
# Check CRSF status
status

# Set serial RX on UART1
serial 0 64 115200 57600 0 115200

# Set CRSF
set serialrx_provider = CRSF
save
```

## Example: Sending CRSF Frame

```cpp
void sendCrsfFrame(uint16_t channels[16]) {
  uint8_t frame[26];
  
  // Header
  frame[0] = 0xC8;  // FC address
  frame[1] = 24;    // Length
  frame[2] = 0x16;  // RC channels type
  
  // Pack 16 channels (11-bit each) into 22 bytes
  // See CrsfBridge_PackChannels() for implementation
  packChannels(channels, &frame[3]);
  
  // CRC over type + payload
  frame[25] = crc8(&frame[2], 23);
  
  // Send to FC
  Serial1.write(frame, 26);
}
```

## Example: Parsing CRSF Frame

```cpp
void parseCrsfFrame(uint8_t* frame, size_t len) {
  if (len < 4) return;
  
  uint8_t addr = frame[0];
  uint8_t frameLen = frame[1];
  uint8_t type = frame[2];
  
  // Verify CRC
  uint8_t expectedCrc = crc8(&frame[2], frameLen - 1);
  if (expectedCrc != frame[2 + frameLen]) return;
  
  if (type == 0x16) {
    // Unpack RC channels
    uint16_t channels[16];
    unpackChannels(&frame[3], channels);
    
    // Use channels...
  }
}
```

## Debugging

### Check Frame Format

Use serial monitor or logic analyzer:

```
Expected pattern (hex):
C8 18 16 [22 bytes of packed data] [CRC]

Example (all channels at 992):
C8 18 16 E0 03 1F 7C F0 C1 07 1F 7C F0 C1 07 1F 7C F0 C1 07 1F 7C F0 C1 [CRC]
```

### Common Issues

1. **No channel movement in Betaflight**
   - Check baud rate (420000)
   - Verify UART TX/RX pins
   - Check CRSF is enabled in ports
   - Verify frame format with logic analyzer

2. **Channels move erratically**
   - Check CRC calculation
   - Verify channel packing
   - Check for electrical noise

3. **Failsafe triggering randomly**
   - Check update rate (should be consistent)
   - Verify CRC is correct
   - Check UART signal quality

4. **Wrong channel values**
   - Verify 11-bit range (172-1811)
   - Check channel packing order
   - Verify bit shifting logic

## References

- **CRSF Wiki**: https://github.com/crsf-wg/crsf/wiki
- **Betaflight Source**: https://github.com/betaflight/betaflight/tree/master/src/main/rx
- **ExpressLRS**: https://github.com/ExpressLRS/ExpressLRS
- **TBS**: https://www.team-blacksheep.com/products/prod:crossfire_tx

## Implementation in This Project

### Files
- `crsf_bridge.h/cpp` - CRSF frame generation
- `protocol.h/cpp` - Channel storage (CRSF format)
- Integration files - RC command to CRSF bridge

### Usage
```cpp
CrsfBridge_Init(43, 44, 1);  // TX, RX, Serial port
CrsfBridge_SetUpdateRate(50); // 50 Hz

uint16_t channels[16] = {992, 992, 172, ...};
CrsfBridge_SetChannels(channels);
CrsfBridge_Update();  // Call in loop
```

### Customization
```cpp
// Change update rate
CrsfBridge_SetUpdateRate(100);  // 100 Hz

// Get statistics
CrsfBridgeStats stats;
CrsfBridge_GetStats(&stats);
Serial.printf("Frames sent: %lu\n", stats.framesSent);
```

## Notes

- CRSF is a **one-way protocol** for RC channels (TX → FC)
- Telemetry is separate (requires bidirectional UART)
- This implementation focuses on **RC channels only**
- Betaflight expects frames at regular intervals
- Missing frames trigger failsafe after ~1 second

