#!/usr/bin/env python3

import sys, time, struct, serial
from typing import Optional, Tuple, List

# ======== USER-ADJUSTABLE DEFAULTS ========
DEFAULT_PORT   = "/dev/ttyACM0"  
BAUD           = 115200
TIMEOUT        = 1.0

# ======== MSP command IDs ========
MSP_API_VERSION             = 1
MSP_FC_VARIANT              = 2
MSP_FC_VERSION              = 3
MSP_STATUS                  = 101
MSP_MOTOR                   = 104
MSP_SET_MOTOR               = 214
MSP_ARMING_DISABLE_FLAGS    = 242

# ======== MSP v1 helpers ========
def msp_checksum(size: int, cmd: int, payload: bytes) -> int:
    c = (size ^ cmd) & 0xFF
    for b in payload: c ^= b
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
    size_b = ser.read(1); cmd_b = ser.read(1)
    if len(size_b)!=1 or len(cmd_b)!=1: return None
    size, cmd = size_b[0], cmd_b[0]
    payload = ser.read(size)
    if len(payload)!=size: return None
    csum_b = ser.read(1)
    if len(csum_b)!=1 or msp_checksum(size, cmd, payload)!=csum_b[0]: return None
    return (cmd, payload)

def msp_request(ser: serial.Serial, cmd: int, payload: bytes = b"", wait_resp: float = 0.25) -> Optional[bytes]:
    ser.reset_input_buffer()
    msp_send(ser, cmd, payload)
    t0 = time.time()
    while time.time() - t0 < wait_resp:
        fr = msp_read_frame(ser)
        if fr is None: 
            continue
        rcmd, rpl = fr
        if rcmd == cmd:
            return rpl
    return None

# ======== parsing helpers ========
u16 = lambda b: struct.unpack('<H', b)[0]
u32 = lambda b: struct.unpack('<I', b)[0]

def parse_ascii(pb: bytes) -> str:
    return (pb or b"").decode('ascii', errors='ignore').strip()

def parse_api_version(pb: bytes):
    return f"{pb[0]}.{pb[1]}.{pb[2]}" if pb and len(pb)>=3 else "?"

def parse_fc_variant(pb: bytes):
    return parse_ascii(pb) if pb else "?"

def parse_fc_version(pb: bytes):
    return f"{pb[0]}.{pb[1]}.{pb[2]}" if pb and len(pb)>=3 else "?"

def parse_u16_list(pb: bytes) -> List[int]:
    if not pb: return []
    n = len(pb)//2
    return list(struct.unpack('<' + 'H'*n, pb[:2*n]))

# ======== Motor control functions ========
def read_motors(ser: serial.Serial) -> List[int]:
    """Read current motor values"""
    return parse_u16_list(msp_request(ser, MSP_MOTOR) or b"")

def set_motors(ser: serial.Serial, motor_values: List[int]) -> bool:
    """
    Set motor values (typically 1000-2000 range)
    motor_values: list of integers, one per motor
    Returns True if command was sent successfully
    """
    # Pack motor values as uint16 little-endian
    payload = struct.pack('<' + 'H'*len(motor_values), *motor_values)
    msp_send(ser, MSP_SET_MOTOR, payload)
    time.sleep(0.01)  # Small delay for FC to process
    return True

def set_all_motors(ser: serial.Serial, value: int, num_motors: int = 4) -> bool:
    """Set all motors to the same value"""
    return set_motors(ser, [value] * num_motors)

def stop_all_motors(ser: serial.Serial, num_motors: int = 4) -> bool:
    """Set all motors to minimum throttle (1000)"""
    return set_all_motors(ser, 1000, num_motors)

def print_info(ser: serial.Serial):
    """Print FC identification"""
    print("\n" + "="*60)
    print("FLIGHT CONTROLLER INFO")
    print("="*60)
    api = parse_api_version(msp_request(ser, MSP_API_VERSION) or b"")
    variant = parse_fc_variant(msp_request(ser, MSP_FC_VARIANT) or b"")
    version = parse_fc_version(msp_request(ser, MSP_FC_VERSION) or b"")
    print(f"API Version : {api}")
    print(f"FC Variant  : {variant}")
    print(f"FC Version  : {version}")
    print("="*60)

def print_motors(ser: serial.Serial):
    """Print current motor values"""
    motors = read_motors(ser)
    if not motors:
        print("No motor data received")
        return
    print(f"\nCurrent motor values ({len(motors)} motors):")
    for i, val in enumerate(motors):
        print(f"  Motor {i+1}: {val}")

# ======== Interactive control modes ========
def interactive_mode(ser: serial.Serial):
    """Interactive mode - set individual motor values"""
    print("\n" + "="*60)
    print("INTERACTIVE MOTOR CONTROL")
    print("="*60)
    print("Enter motor values separated by spaces (e.g., '1000 1000 1000 1000')")
    print("Valid range: 1000-2000 (1000=stop, 2000=full)")
    print("Type 'read' to read current values")
    print("Type 'stop' to stop all motors (set to 1000)")
    print("Type 'quit' to exit")
    print("="*60)
    
    while True:
        try:
            cmd = input("\nMotor values> ").strip().lower()
            
            if cmd == 'quit' or cmd == 'q':
                print("Stopping all motors before exit...")
                stop_all_motors(ser)
                break
            
            elif cmd == 'read' or cmd == 'r':
                print_motors(ser)
            
            elif cmd == 'stop' or cmd == 's':
                num = input("Number of motors [4]: ").strip()
                num_motors = int(num) if num else 4
                stop_all_motors(ser, num_motors)
                print(f"All {num_motors} motors set to 1000")
                print_motors(ser)
            
            else:
                # Parse motor values
                values = [int(v) for v in cmd.split()]
                
                # Validate range
                for v in values:
                    if v < 1000 or v > 2000:
                        print(f"ERROR: Value {v} out of range (1000-2000)")
                        raise ValueError
                
                # Send command
                set_motors(ser, values)
                print(f"Set {len(values)} motors: {values}")
                time.sleep(0.1)
                print_motors(ser)
                
        except ValueError:
            print("Invalid input. Use format: 1000 1200 1500 2000")
        except KeyboardInterrupt:
            print("\n\nStopping all motors before exit...")
            stop_all_motors(ser)
            break

def sweep_mode(ser: serial.Serial):
    """Sweep mode - gradually increase/decrease motor speeds"""
    print("\n" + "="*60)
    print("MOTOR SWEEP MODE")
    print("="*60)
    
    num = input("Number of motors [4]: ").strip()
    num_motors = int(num) if num else 4
    
    start_val = int(input("Start value [1000]: ").strip() or "1000")
    end_val = int(input("End value [1500]: ").strip() or "1500")
    step = int(input("Step size [50]: ").strip() or "50")
    delay = float(input("Delay between steps (sec) [0.5]: ").strip() or "0.5")
    
    print(f"\nSweeping {num_motors} motors from {start_val} to {end_val}")
    print(f"Step: {step}, Delay: {delay}s")
    print("Press Ctrl+C to stop\n")
    
    try:
        direction = 1 if end_val > start_val else -1
        current = start_val
        
        while True:
            set_all_motors(ser, current, num_motors)
            print(f"Motors: {current}")
            time.sleep(delay)
            
            current += step * direction
            
            # Reverse direction at limits
            if direction > 0 and current > end_val:
                current = end_val
                direction = -1
            elif direction < 0 and current < start_val:
                current = start_val
                direction = 1
                
    except KeyboardInterrupt:
        print("\n\nStopping sweep...")
        stop_all_motors(ser, num_motors)

def test_sequence_mode(ser: serial.Serial):
    """Test sequence - predefined motor test pattern"""
    print("\n" + "="*60)
    print("MOTOR TEST SEQUENCE")
    print("="*60)
    
    num = input("Number of motors [4]: ").strip()
    num_motors = int(num) if num else 4
    
    print(f"\nRunning test sequence on {num_motors} motors")
    print("1. All motors to 1000 (stop)")
    print("2. Individual motor spin-up test")
    print("3. All motors together ramp test")
    print("\nPress Ctrl+C to abort\n")
    
    input("Press Enter to start...")
    
    try:
        # Step 1: All stop
        print("\n[1/3] Setting all motors to 1000...")
        stop_all_motors(ser, num_motors)
        print_motors(ser)
        time.sleep(2)
        
        # Step 2: Individual motor test
        print("\n[2/3] Testing each motor individually...")
        for i in range(num_motors):
            values = [1000] * num_motors
            values[i] = 1300
            print(f"\nMotor {i+1} -> 1300, others -> 1000")
            set_motors(ser, values)
            print_motors(ser)
            time.sleep(2)
            stop_all_motors(ser, num_motors)
            time.sleep(1)
        
        # Step 3: All motors ramp
        print("\n[3/3] Ramping all motors together...")
        for val in range(1000, 1501, 100):
            print(f"\nAll motors -> {val}")
            set_all_motors(ser, val, num_motors)
            print_motors(ser)
            time.sleep(1)
        
        print("\nRamping down...")
        for val in range(1500, 999, -100):
            print(f"\nAll motors -> {val}")
            set_all_motors(ser, val, num_motors)
            print_motors(ser)
            time.sleep(1)
        
        print("\n✓ Test sequence complete!")
        
    except KeyboardInterrupt:
        print("\n\nTest aborted!")
        stop_all_motors(ser, num_motors)

# ======== Main menu ========
def main_menu(ser: serial.Serial):
    """Main menu for motor control"""
    
    print_info(ser)
    print_motors(ser)
    
    while True:
        print("\n" + "="*60)
        print("MSP MOTOR CONTROL MENU")
        print("="*60)
        print("1) Interactive mode - Set individual motor values")
        print("2) Sweep mode - Automated motor sweep")
        print("3) Test sequence - Predefined motor test pattern")
        print("4) Read current motor values")
        print("5) Stop all motors (set to 1000)")
        print("0) Quit")
        print("="*60)
        
        choice = input("Select option> ").strip()
        
        if choice == "1":
            interactive_mode(ser)
        elif choice == "2":
            sweep_mode(ser)
        elif choice == "3":
            test_sequence_mode(ser)
        elif choice == "4":
            print_motors(ser)
        elif choice == "5":
            num = input("Number of motors [4]: ").strip()
            num_motors = int(num) if num else 4
            stop_all_motors(ser, num_motors)
            print(f"All {num_motors} motors set to 1000")
            print_motors(ser)
        elif choice == "0" or choice == "q":
            print("\nStopping all motors before exit...")
            stop_all_motors(ser)
            break
        else:
            print("Invalid choice")

def main():
    port = sys.argv[1] if len(sys.argv) > 1 else DEFAULT_PORT
    
    print("\n" + "="*60)
    print("MSP MOTOR CONTROL SCRIPT")
    print("="*60)
    print("WARNING: This script sends motor commands!")
    print("Remove propellers before testing!")
    print("Ensure your FC is properly configured!")
    print("="*60)
    
    confirm = input("\nHave you removed propellers? (yes/no): ").strip().lower()
    if confirm != "yes":
        print("Aborting for safety. Remove propellers first!")
        sys.exit(0)
    
    try:
        ser = serial.Serial(port, BAUD, timeout=TIMEOUT)
    except Exception as e:
        print(f"ERROR: cannot open {port} @ {BAUD}: {e}")
        sys.exit(1)
    
    print(f"\n✓ Opened {port} @ {BAUD}")
    
    try:
        main_menu(ser)
    except Exception as e:
        print(f"\nERROR: {e}")
        stop_all_motors(ser)
    finally:
        ser.close()
        print("\n✓ Serial closed.")
        print("✓ Safe exit complete.\n")

if __name__ == "__main__":
    main()