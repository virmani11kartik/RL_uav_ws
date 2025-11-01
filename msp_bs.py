#!/usr/bin/env python3

import serial, struct, time, sys
from typing import Optional, Tuple, List

PORT = "/dev/ttyACM0"
BAUD = 115200
TIMEOUT = 1.0

# MSP command IDs
MSP_BATTERY_STATE = 130
MSP_ANALOG = 110

# ---------- MSP helpers (v1) ----------
def msp_checksum(size: int, cmd: int, payload: bytes) -> int:
    c = (size ^ cmd) & 0xFF
    for b in payload:
        c ^= b
    return c & 0xFF

def msp_make_frame(cmd: int, payload: bytes = b"") -> bytes:
    return b"$M<" + bytes([len(payload), cmd]) + payload + bytes([msp_checksum(len(payload), cmd, payload)])

def msp_send(ser: serial.Serial, cmd: int, payload: bytes = b"") -> None:
    ser.write(msp_make_frame(cmd, payload))

def msp_read_frame(ser: serial.Serial) -> Optional[Tuple[int, bytes]]:
    if ser.read(1) != b'$': return None
    if ser.read(1) != b'M': return None
    d = ser.read(1)
    if d not in (b'>', b'!'): return None

    sb = ser.read(1);  cb = ser.read(1)
    if len(sb) != 1 or len(cb) != 1: return None
    size, cmd = sb[0], cb[0]

    payload = ser.read(size)
    if len(payload) != size: return None

    csum_b = ser.read(1)
    if len(csum_b) != 1: return None
    if msp_checksum(size, cmd, payload) != csum_b[0]: return None
    return (cmd, payload)

def msp_request(ser: serial.Serial, cmd: int, payload: bytes = b"", wait_resp: float = 0.25) -> Optional[bytes]:
    ser.reset_input_buffer()
    msp_send(ser, cmd, payload)
    t0 = time.time()
    while time.time() - t0 < wait_resp:
        frame = msp_read_frame(ser)
        if frame is None: 
            continue
        rcmd, rpayload = frame
        if rcmd == cmd:
            return rpayload
    return None

def hex_dump(data: bytes, label: str = ""):
    """Print hex dump of bytes with both hex and decimal values"""
    if label:
        print(f"\n{label}")
        print("=" * len(label))
    
    print(f"Length: {len(data)} bytes")
    print("\nByte-by-byte breakdown:")
    print("Idx | Hex  | Dec   | Bits")
    print("----|------|-------|----------")
    for i, b in enumerate(data):
        print(f" {i:2d} | 0x{b:02X} | {b:3d}   | {b:08b}")
    
    print("\nRaw hex string:")
    print(" ".join(f"{b:02X}" for b in data))
    
    print("\nRaw bytes:")
    print(list(data))

def try_parse_battery_betaflight(pb: bytes):
    """Betaflight MSP_BATTERY_STATE format"""
    print("\n--- Betaflight format attempt ---")
    try:
        i = 0
        cells = pb[i] if len(pb) > i else None; i+=1
        capacity = struct.unpack('<H', pb[i:i+2])[0] if len(pb) >= i+2 else None; i+=2
        voltage = pb[i] if len(pb) > i else None; i+=1  # Note: single byte, in 0.1V
        mah_drawn = struct.unpack('<H', pb[i:i+2])[0] if len(pb) >= i+2 else None; i+=2
        amps = struct.unpack('<H', pb[i:i+2])[0] if len(pb) >= i+2 else None; i+=2
        
        print(f"Cells: {cells}")
        print(f"Capacity: {capacity} mAh")
        print(f"Voltage (byte): {voltage} -> {voltage/10 if voltage else None} V")
        print(f"mAh drawn: {mah_drawn}")
        print(f"Current: {amps} -> {amps/100 if amps else None} A")
    except Exception as e:
        print(f"Parse error: {e}")

def try_parse_analog_betaflight(pb: bytes):
    """Betaflight MSP_ANALOG format"""
    print("\n--- Betaflight ANALOG format attempt ---")
    try:
        i = 0
        vbat = pb[i] if len(pb) > i else None; i+=1  # 0.1V
        mah_drawn = struct.unpack('<H', pb[i:i+2])[0] if len(pb) >= i+2 else None; i+=2
        rssi = struct.unpack('<H', pb[i:i+2])[0] if len(pb) >= i+2 else None; i+=2
        amps = struct.unpack('<H', pb[i:i+2])[0] if len(pb) >= i+2 else None; i+=2
        vbat_u16 = struct.unpack('<H', pb[i:i+2])[0] if len(pb) >= i+2 else None; i+=2
        
        print(f"VBAT (byte): {vbat} -> {vbat/10 if vbat else None} V")
        print(f"mAh drawn: {mah_drawn}")
        print(f"RSSI: {rssi}")
        print(f"Current: {amps} -> {amps/100 if amps else None} A")
        print(f"VBAT (u16): {vbat_u16} -> {vbat_u16/100 if vbat_u16 else None} V")
    except Exception as e:
        print(f"Parse error: {e}")

def main():
    try:
        ser = serial.Serial(PORT, BAUD, timeout=TIMEOUT)
    except Exception as e:
        print(f"ERROR: cannot open {PORT} @ {BAUD}: {e}")
        sys.exit(1)
    
    print(f"Connected to {PORT} @ {BAUD}")
    print("\nFetching battery data for analysis...\n")
    
    # Get MSP_BATTERY_STATE
    print("=" * 60)
    print("REQUESTING MSP_BATTERY_STATE (130)")
    print("=" * 60)
    battery_data = msp_request(ser, MSP_BATTERY_STATE)
    if battery_data:
        hex_dump(battery_data, "MSP_BATTERY_STATE Response")
        try_parse_battery_betaflight(battery_data)
    else:
        print("No response received")
    
    print("\n\n")
    
    # Get MSP_ANALOG
    print("=" * 60)
    print("REQUESTING MSP_ANALOG (110)")
    print("=" * 60)
    analog_data = msp_request(ser, MSP_ANALOG)
    if analog_data:
        hex_dump(analog_data, "MSP_ANALOG Response")
        try_parse_analog_betaflight(analog_data)
    else:
        print("No response received")
    
    ser.close()
    print("\n\nDone. Please share this output to fix the parsing!")

if __name__ == "__main__":
    main()