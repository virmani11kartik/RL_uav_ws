#!/usr/bin/env python3

import serial
import struct
import time
import sys
from typing import Optional, Tuple, List, Dict
from collections import deque
from PyQt5.QtWidgets import (QApplication, QMainWindow, QWidget, QVBoxLayout, 
                             QHBoxLayout, QGroupBox, QCheckBox, QPushButton, 
                             QLabel, QComboBox, QScrollArea, QGridLayout, QSpinBox)
from PyQt5.QtCore import QTimer, Qt
import pyqtgraph as pg
import numpy as np

# ============================================================================
# CONFIGURATION
# ============================================================================
PORT = "/dev/ttyACM0"
BAUD = 115200
TIMEOUT = 1.0
POLL_RATE_HZ = 10
MAX_PLOT_POINTS = 500

# ============================================================================
# MSP COMMAND IDs
# ============================================================================
MSP_API_VERSION = 1
MSP_FC_VARIANT = 2
MSP_FC_VERSION = 3
MSP_BUILD_INFO = 5
MSP_BOARD_INFO = 4
MSP_NAME = 10

MSP_STATUS = 101
MSP_RAW_IMU = 102
MSP_SERVO = 103
MSP_MOTOR = 104
MSP_RC = 105
MSP_RAW_GPS = 106
MSP_COMP_GPS = 107
MSP_ATTITUDE = 108
MSP_ALTITUDE = 109
MSP_ANALOG = 110
MSP_RC_TUNING = 111
MSP_PID = 112

MSP_BATTERY_STATE = 130
MSP_VOLTAGE_METERS = 128
MSP_CURRENT_METERS = 129
MSP_TEMPERATURE = 132
MSP_ESC_SENSOR_DATA = 134
MSP_CURRENT_METERS_SUM = 135

MSP_SENSOR_STATUS = 151
MSP_SENSOR_ALIGNMENT = 126
MSP_ARMING_DISABLE_FLAGS = 150

# ============================================================================
# MSP PROTOCOL HELPERS
# ============================================================================
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

    sb = ser.read(1); cb = ser.read(1)
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

# Unpack helpers
u8 = lambda b: b[0] if b else 0
u16 = lambda b: struct.unpack('<H', b)[0]
s16 = lambda b: struct.unpack('<h', b)[0]
u32 = lambda b: struct.unpack('<I', b)[0]
s32 = lambda b: struct.unpack('<i', b)[0]

# ============================================================================
# PARSERS
# ============================================================================
def parse_ascii(pb: bytes) -> str:
    return pb.decode('ascii', errors='ignore').strip()

def parse_api_version(pb: bytes):
    return f"{pb[0]}.{pb[1]}.{pb[2]}" if pb and len(pb) >= 3 else "?"

def parse_fc_variant(pb: bytes):
    return parse_ascii(pb) if pb else "?"

def parse_fc_version(pb: bytes):
    return f"{pb[0]}.{pb[1]}.{pb[2]}" if pb and len(pb) >= 3 else "?"

def parse_build_info(pb: bytes):
    return parse_ascii(pb) if pb else "?"

def parse_name(pb: bytes):
    return parse_ascii(pb) if pb else "?"

def parse_status(pb: bytes):
    out = {}
    if not pb: return out
    if len(pb) >= 2: out["cycle_us"] = u16(pb[0:2])
    if len(pb) >= 4: out["i2c_err"] = u16(pb[2:4])
    if len(pb) >= 6: out["sensors"] = u16(pb[4:6])
    if len(pb) >= 10: out["modes"] = u32(pb[6:10])
    if len(pb) >= 11: out["pid_prof"] = pb[10]
    if len(pb) >= 12: out["rate_prof"] = pb[11]
    if len(pb) >= 14: out["cpu_load"] = u16(pb[12:14])
    return out

def parse_raw_imu(pb: bytes):
    if not pb or len(pb) < 18: return {}
    vals = struct.unpack('<9h', pb[:18])
    return {
        "acc_x": vals[0], "acc_y": vals[1], "acc_z": vals[2],
        "gyro_x": vals[3], "gyro_y": vals[4], "gyro_z": vals[5],
        "mag_x": vals[6], "mag_y": vals[7], "mag_z": vals[8],
    }

def parse_attitude(pb: bytes):
    if not pb or len(pb) < 6: return {}
    r10, p10, hdg = struct.unpack('<hhh', pb[:6])
    return {"roll": r10 / 10.0, "pitch": p10 / 10.0, "yaw": float(hdg)}

def parse_altitude(pb: bytes):
    if not pb or len(pb) < 6: return {}
    alt_cm = s32(pb[0:4]); vz_cms = s16(pb[4:6])
    return {"altitude": alt_cm / 100.0, "vario": vz_cms / 100.0}

def parse_analog(pb: bytes):
    """Betaflight MSP_ANALOG: byte vbat, u16 mah, u16 rssi, u16 amps, u16 vbat_u16"""
    d = {}
    try:
        i = 0
        if len(pb) >= 1:
            d["vbat"] = pb[i] / 10.0; i += 1
        if len(pb) >= i + 2:
            d["mah_drawn"] = u16(pb[i:i+2]); i += 2
        if len(pb) >= i + 2:
            d["rssi"] = u16(pb[i:i+2]); i += 2
        if len(pb) >= i + 2:
            d["current"] = u16(pb[i:i+2]) / 100.0; i += 2
        if len(pb) >= i + 2:
            d["vbat_u16"] = u16(pb[i:i+2]) / 100.0; i += 2
    except: pass
    return d

def parse_battery_state(pb: bytes):
    """Betaflight MSP_BATTERY_STATE: byte cells, u16 capacity, byte voltage, u16 mah, u16 amps..."""
    d = {}
    try:
        i = 0
        if len(pb) >= 1:
            d["cells"] = pb[i]; i += 1
        if len(pb) >= i + 2:
            d["capacity"] = u16(pb[i:i+2]); i += 2
        if len(pb) >= i + 1:
            d["voltage"] = pb[i] / 10.0; i += 1
        if len(pb) >= i + 2:
            d["mah_drawn"] = u16(pb[i:i+2]); i += 2
        if len(pb) >= i + 2:
            d["current"] = u16(pb[i:i+2]) / 100.0; i += 2
        if len(pb) >= i + 1:
            d["battery_state"] = pb[i]; i += 1
        if len(pb) >= i + 2:
            d["cell_voltage"] = u16(pb[i:i+2]) / 100.0; i += 2
    except: pass
    return d

def parse_temperature(pb: bytes):
    d = {}
    if pb and len(pb) >= 2:
        d["temp_c"] = s16(pb[0:2]) / 10.0
    return d

def parse_gps(pb: bytes):
    """MSP_RAW_GPS"""
    d = {}
    try:
        i = 0
        if len(pb) >= 1:
            d["fix"] = pb[i]; i += 1
        if len(pb) >= i + 1:
            d["sats"] = pb[i]; i += 1
        if len(pb) >= i + 4:
            d["lat"] = s32(pb[i:i+4]) / 10000000.0; i += 4
        if len(pb) >= i + 4:
            d["lon"] = s32(pb[i:i+4]) / 10000000.0; i += 4
        if len(pb) >= i + 2:
            d["altitude"] = u16(pb[i:i+2]); i += 2
        if len(pb) >= i + 2:
            d["speed"] = u16(pb[i:i+2]); i += 2
        if len(pb) >= i + 2:
            d["ground_course"] = u16(pb[i:i+2]); i += 2
    except: pass
    return d

def parse_u16_list(pb: bytes) -> List[int]:
    if not pb: return []
    n = len(pb) // 2
    return list(struct.unpack('<' + 'H' * n, pb[:2 * n]))

def parse_esc_sensor(pb: bytes):
    """MSP_ESC_SENSOR_DATA"""
    escs = []
    try:
        if not pb: return escs
        # Each ESC: temp(byte), rpm(u16), etc - format varies
        # Simplified: assume 10 bytes per ESC
        i = 0
        while i + 10 <= len(pb):
            esc = {
                "temp": pb[i],
                "rpm": u16(pb[i+1:i+3]),
            }
            escs.append(esc)
            i += 10
    except: pass
    return escs

# ============================================================================
# DATA COLLECTOR
# ============================================================================
class MSPDataCollector:
    def __init__(self, port: str, baud: int, timeout: float):
        self.ser = serial.Serial(port, baud, timeout=timeout)
        self.data_history = {}
        self.max_points = MAX_PLOT_POINTS
        self.time_start = time.time()
        
        # Initialize data buffers
        self.init_buffers()
        
        # Fetch FC info once
        self.fc_info = self.fetch_fc_info()
    
    def init_buffers(self):
        """Initialize deque buffers for all data streams"""
        self.data_history = {
            "time": deque(maxlen=self.max_points),
            # Attitude
            "roll": deque(maxlen=self.max_points),
            "pitch": deque(maxlen=self.max_points),
            "yaw": deque(maxlen=self.max_points),
            # IMU
            "acc_x": deque(maxlen=self.max_points),
            "acc_y": deque(maxlen=self.max_points),
            "acc_z": deque(maxlen=self.max_points),
            "gyro_x": deque(maxlen=self.max_points),
            "gyro_y": deque(maxlen=self.max_points),
            "gyro_z": deque(maxlen=self.max_points),
            "mag_x": deque(maxlen=self.max_points),
            "mag_y": deque(maxlen=self.max_points),
            "mag_z": deque(maxlen=self.max_points),
            # Altitude
            "altitude": deque(maxlen=self.max_points),
            "vario": deque(maxlen=self.max_points),
            # Battery
            "voltage": deque(maxlen=self.max_points),
            "current": deque(maxlen=self.max_points),
            "mah_drawn": deque(maxlen=self.max_points),
            "rssi": deque(maxlen=self.max_points),
            # Status
            "cpu_load": deque(maxlen=self.max_points),
            "cycle_us": deque(maxlen=self.max_points),
            # Motors
            "motor_1": deque(maxlen=self.max_points),
            "motor_2": deque(maxlen=self.max_points),
            "motor_3": deque(maxlen=self.max_points),
            "motor_4": deque(maxlen=self.max_points),
            # RC
            "rc_roll": deque(maxlen=self.max_points),
            "rc_pitch": deque(maxlen=self.max_points),
            "rc_throttle": deque(maxlen=self.max_points),
            "rc_yaw": deque(maxlen=self.max_points),
            # GPS
            "gps_lat": deque(maxlen=self.max_points),
            "gps_lon": deque(maxlen=self.max_points),
            "gps_sats": deque(maxlen=self.max_points),
            "gps_speed": deque(maxlen=self.max_points),
            # Temperature
            "temp_c": deque(maxlen=self.max_points),
        }
    
    def fetch_fc_info(self) -> Dict:
        """Fetch flight controller identification"""
        info = {}
        r = msp_request(self.ser, MSP_API_VERSION)
        info["api_version"] = parse_api_version(r) if r else "?"
        r = msp_request(self.ser, MSP_FC_VARIANT)
        info["variant"] = parse_fc_variant(r) if r else "?"
        r = msp_request(self.ser, MSP_FC_VERSION)
        info["version"] = parse_fc_version(r) if r else "?"
        r = msp_request(self.ser, MSP_NAME)
        info["name"] = parse_name(r) if r else "?"
        return info
    
    def poll_data(self, categories: Dict[str, bool]) -> Dict:
        """Poll selected data categories from FC"""
        current_time = time.time() - self.time_start
        self.data_history["time"].append(current_time)
        
        latest = {"time": current_time}
        
        # Attitude
        if categories.get("attitude", False):
            r = msp_request(self.ser, MSP_ATTITUDE, wait_resp=0.1)
            if r:
                att = parse_attitude(r)
                for k, v in att.items():
                    self.data_history[k].append(v)
                    latest[k] = v
        
        # IMU
        if categories.get("imu", False):
            r = msp_request(self.ser, MSP_RAW_IMU, wait_resp=0.1)
            if r:
                imu = parse_raw_imu(r)
                for k, v in imu.items():
                    self.data_history[k].append(v)
                    latest[k] = v
        
        # Altitude
        if categories.get("altitude", False):
            r = msp_request(self.ser, MSP_ALTITUDE, wait_resp=0.1)
            if r:
                alt = parse_altitude(r)
                for k, v in alt.items():
                    self.data_history[k].append(v)
                    latest[k] = v
        
        # Battery
        if categories.get("battery", False):
            r = msp_request(self.ser, MSP_BATTERY_STATE, wait_resp=0.1)
            if r:
                bat = parse_battery_state(r)
                if "voltage" in bat:
                    self.data_history["voltage"].append(bat["voltage"])
                    latest["voltage"] = bat["voltage"]
                if "current" in bat:
                    self.data_history["current"].append(bat["current"])
                    latest["current"] = bat["current"]
                if "mah_drawn" in bat:
                    self.data_history["mah_drawn"].append(bat["mah_drawn"])
                    latest["mah_drawn"] = bat["mah_drawn"]
                latest["cells"] = bat.get("cells", "?")
        
        # Analog (for RSSI)
        if categories.get("analog", False):
            r = msp_request(self.ser, MSP_ANALOG, wait_resp=0.1)
            if r:
                ana = parse_analog(r)
                if "rssi" in ana:
                    self.data_history["rssi"].append(ana["rssi"])
                    latest["rssi"] = ana["rssi"]
        
        # Status
        if categories.get("status", False):
            r = msp_request(self.ser, MSP_STATUS, wait_resp=0.1)
            if r:
                st = parse_status(r)
                if "cpu_load" in st:
                    self.data_history["cpu_load"].append(st["cpu_load"])
                    latest["cpu_load"] = st["cpu_load"]
                if "cycle_us" in st:
                    self.data_history["cycle_us"].append(st["cycle_us"])
                    latest["cycle_us"] = st["cycle_us"]
        
        # Motors
        if categories.get("motors", False):
            r = msp_request(self.ser, MSP_MOTOR, wait_resp=0.1)
            if r:
                motors = parse_u16_list(r)
                for i, val in enumerate(motors[:4]):
                    key = f"motor_{i+1}"
                    self.data_history[key].append(val)
                    latest[key] = val
        
        # RC
        if categories.get("rc", False):
            r = msp_request(self.ser, MSP_RC, wait_resp=0.1)
            if r:
                rc = parse_u16_list(r)
                rc_names = ["rc_roll", "rc_pitch", "rc_throttle", "rc_yaw"]
                for i, name in enumerate(rc_names):
                    if i < len(rc):
                        self.data_history[name].append(rc[i])
                        latest[name] = rc[i]
        
        # GPS
        if categories.get("gps", False):
            r = msp_request(self.ser, MSP_RAW_GPS, wait_resp=0.1)
            if r:
                gps = parse_gps(r)
                for k in ["gps_lat", "gps_lon", "gps_sats", "gps_speed"]:
                    gps_key = k.replace("gps_", "")
                    if gps_key in gps:
                        self.data_history[k].append(gps[gps_key])
                        latest[k] = gps[gps_key]
        
        # Temperature
        if categories.get("temperature", False):
            r = msp_request(self.ser, MSP_TEMPERATURE, wait_resp=0.1)
            if r:
                temp = parse_temperature(r)
                if "temp_c" in temp:
                    self.data_history["temp_c"].append(temp["temp_c"])
                    latest["temp_c"] = temp["temp_c"]
        
        return latest
    
    def close(self):
        self.ser.close()

# ============================================================================
# PyQt GUI
# ============================================================================
class MSPDashboard(QMainWindow):
    def __init__(self):
        super().__init__()
        self.collector = None
        self.timer = QTimer()
        self.timer.timeout.connect(self.update_data)
        self.plots = {}
        self.curves = {}
        
        # Categories to poll
        self.categories = {
            "attitude": True,
            "imu": True,
            "altitude": True,
            "battery": True,
            "analog": True,
            "status": True,
            "motors": False,
            "rc": True,
            "gps": False,
            "temperature": False,
        }
        
        # Plot selections
        self.plot_selections = {
            "roll": True,
            "pitch": True,
            "yaw": False,
            "acc_x": False,
            "acc_y": False,
            "acc_z": False,
            "gyro_x": False,
            "gyro_y": False,
            "gyro_z": False,
            "altitude": True,
            "voltage": True,
            "current": True,
            "motor_1": False,
            "motor_2": False,
            "motor_3": False,
            "motor_4": False,
            "rc_throttle": False,
        }
        
        self.init_ui()
        self.connect_fc()
    
    def init_ui(self):
        self.setWindowTitle("MSP Flight Controller Dashboard")
        self.setGeometry(100, 100, 1600, 900)
        
        # Main widget and layout
        main_widget = QWidget()
        self.setCentralWidget(main_widget)
        main_layout = QHBoxLayout(main_widget)
        
        # Left panel: controls
        left_panel = self.create_control_panel()
        main_layout.addWidget(left_panel, stretch=1)
        
        # Right panel: plots
        right_panel = self.create_plot_panel()
        main_layout.addWidget(right_panel, stretch=4)
    
    def create_control_panel(self):
        panel = QWidget()
        layout = QVBoxLayout(panel)
        
        # FC Info
        info_group = QGroupBox("Flight Controller Info")
        info_layout = QVBoxLayout()
        self.info_label = QLabel("Connecting...")
        info_layout.addWidget(self.info_label)
        info_group.setLayout(info_layout)
        layout.addWidget(info_group)
        
        # Category selection
        cat_group = QGroupBox("Data Categories")
        cat_layout = QVBoxLayout()
        self.cat_checkboxes = {}
        for cat in self.categories.keys():
            cb = QCheckBox(cat.upper())
            cb.setChecked(self.categories[cat])
            cb.stateChanged.connect(lambda state, c=cat: self.toggle_category(c, state))
            self.cat_checkboxes[cat] = cb
            cat_layout.addWidget(cb)
        cat_group.setLayout(cat_layout)
        
        cat_scroll = QScrollArea()
        cat_scroll.setWidget(cat_group)
        cat_scroll.setWidgetResizable(True)
        layout.addWidget(cat_scroll)
        
        # Plot selection
        plot_group = QGroupBox("Plot Selection")
        plot_layout = QVBoxLayout()
        self.plot_checkboxes = {}
        for key in self.plot_selections.keys():
            cb = QCheckBox(key.replace("_", " ").upper())
            cb.setChecked(self.plot_selections[key])
            cb.stateChanged.connect(lambda state, k=key: self.toggle_plot(k, state))
            self.plot_checkboxes[key] = cb
            plot_layout.addWidget(cb)
        plot_group.setLayout(plot_layout)
        
        plot_scroll = QScrollArea()
        plot_scroll.setWidget(plot_group)
        plot_scroll.setWidgetResizable(True)
        layout.addWidget(plot_scroll)
        
        # Control buttons
        btn_layout = QHBoxLayout()
        self.start_btn = QPushButton("Start")
        self.start_btn.clicked.connect(self.start_polling)
        self.stop_btn = QPushButton("Stop")
        self.stop_btn.clicked.connect(self.stop_polling)
        self.stop_btn.setEnabled(False)
        btn_layout.addWidget(self.start_btn)
        btn_layout.addWidget(self.stop_btn)
        layout.addLayout(btn_layout)
        
        # Poll rate
        rate_layout = QHBoxLayout()
        rate_layout.addWidget(QLabel("Poll Rate (Hz):"))
        self.rate_spin = QSpinBox()
        self.rate_spin.setRange(1, 50)
        self.rate_spin.setValue(POLL_RATE_HZ)
        rate_layout.addWidget(self.rate_spin)
        layout.addLayout(rate_layout)
        
        # Latest values
        values_group = QGroupBox("Latest Values")
        values_layout = QVBoxLayout()
        self.values_label = QLabel("Waiting for data...")
        self.values_label.setWordWrap(True)
        values_layout.addWidget(self.values_label)
        values_group.setLayout(values_layout)
        layout.addWidget(values_group)
        
        layout.addStretch()
        return panel
    
    def create_plot_panel(self):
        panel = QWidget()
        layout = QVBoxLayout(panel)
        
        # Create plot widget with subplots
        self.plot_widget = pg.GraphicsLayoutWidget()
        layout.addWidget(self.plot_widget)
        
        # Initialize empty plots
        self.rebuild_plots()
        
        return panel
    
    def rebuild_plots(self):
        """Rebuild plot layout based on selections"""
        self.plot_widget.clear()
        self.plots.clear()
        self.curves.clear()
        
        selected = [k for k, v in self.plot_selections.items() if v]
        
        if not selected:
            return
        
        # Organize plots in grid
        cols = 2
        for i, key in enumerate(selected):
            row = i // cols
            col = i % cols
            
            plot = self.plot_widget.addPlot(row=row, col=col, title=key.replace("_", " ").upper())
            plot.setLabel('left', key)
            plot.setLabel('bottom', 'Time', units='s')
            plot.showGrid(x=True, y=True)
            plot.addLegend()
            
            curve = plot.plot(pen=pg.mkPen(color='y', width=2), name=key)
            
            self.plots[key] = plot
            self.curves[key] = curve
    
    def toggle_category(self, category: str, state: int):
        self.categories[category] = bool(state)
    
    def toggle_plot(self, key: str, state: int):
        self.plot_selections[key] = bool(state)
        self.rebuild_plots()
    
    def connect_fc(self):
        try:
            self.collector = MSPDataCollector(PORT, BAUD, TIMEOUT)
            info = self.collector.fc_info
            info_text = f"Name: {info.get('name', '?')}\n"
            info_text += f"Variant: {info.get('variant', '?')}\n"
            info_text += f"Version: {info.get('version', '?')}\n"
            info_text += f"API: {info.get('api_version', '?')}"
            self.info_label.setText(info_text)
        except Exception as e:
            self.info_label.setText(f"Connection Error:\n{str(e)}")
    
    def start_polling(self):
        if not self.collector:
            return
        
        poll_rate = self.rate_spin.value()
        interval_ms = int(1000 / poll_rate)
        self.timer.start(interval_ms)
        self.start_btn.setEnabled(False)
        self.stop_btn.setEnabled(True)
    
    def stop_polling(self):
        self.timer.stop()
        self.start_btn.setEnabled(True)
        self.stop_btn.setEnabled(False)
    
    def update_data(self):
        if not self.collector:
            return
        
        # Poll data
        latest = self.collector.poll_data(self.categories)
        
        # Update plots
        time_data = np.array(self.collector.data_history["time"])
        for key, curve in self.curves.items():
            if key in self.collector.data_history:
                y_data = np.array(self.collector.data_history[key])
                if len(y_data) > 0 and len(time_data) == len(y_data):
                    curve.setData(time_data, y_data)
        
        # Update latest values display
        values_text = ""
        for k, v in latest.items():
            if k != "time" and isinstance(v, (int, float)):
                values_text += f"{k}: {v:.2f}\n"
            elif k != "time":
                values_text += f"{k}: {v}\n"
        
        self.values_label.setText(values_text if values_text else "No data")
    
    def closeEvent(self, event):
        if self.collector:
            self.collector.close()
        event.accept()

# ============================================================================
# MAIN
# ============================================================================
def main():
    app = QApplication(sys.argv)
    window = MSPDashboard()
    window.show()
    sys.exit(app.exec_())

if __name__ == "__main__":
    main()