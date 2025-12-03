import sys
import json
import os
import time
import struct
import re
import binascii

# --- COMPATIBILITY CHECK ---
try:
    from PyQt5.QtWidgets import (QApplication, QMainWindow, QWidget, QVBoxLayout, 
                                 QHBoxLayout, QComboBox, QLabel, QPushButton, 
                                 QTextEdit, QFrame, QMessageBox, QTableWidget, 
                                 QTableWidgetItem, QHeaderView, QSplitter, 
                                 QProgressBar, QFileDialog, QCheckBox, QTabWidget, 
                                 QLineEdit, QSizePolicy, QScrollArea, QGroupBox, 
                                 QGridLayout, QRadioButton, QButtonGroup, QListWidget)
    from PyQt5.QtGui import QColor, QFont, QTextCursor, QPixmap, QDesktopServices
    from PyQt5.QtCore import Qt, QThread, pyqtSignal, QUrl
except ImportError:
    print("Error: PyQt5 is missing. Please run: pip install pyqt5")
    sys.exit(1)

try:
    from pyftdi.ftdi import Ftdi 
    from pyftdi.spi import SpiController
    from pyftdi.i2c import I2cController
    from pyftdi.usbtools import UsbTools
    import pyftdi.serialext 
    PYFTDI_AVAILABLE = True
except ImportError:
    PYFTDI_AVAILABLE = False

# ==========================================
# 1. DATABASE & PINOUTS
# ==========================================
PACKAGE_MAP = {
    "SOP8_SPI":  {"CS": 1, "MISO": 2, "WP": 3, "GND": 4, "MOSI": 5, "CLK": 6, "HOLD": 7, "VCC": 8},
    "SOP8_I2C":  {"A0": 1, "A1": 2, "A2": 3, "GND": 4, "SDA": 5, "SCL": 6, "WP": 7, "VCC": 8},
    "SOP8_MW":   {"CS": 1, "CLK": 2, "DI": 3, "DO": 4, "VSS": 5, "ORG": 6, "NC": 7, "VCC": 8},
    "SOP8_AT45": {"SI (MOSI)": 1, "SCK": 2, "RESET": 3, "CS": 4, "WP": 5, "VCC": 6, "GND": 7, "SO (MISO)": 8}
}

CHIP_DB = None
JEDEC_LOOKUP = {}

def load_chip_database():
    global CHIP_DB, JEDEC_LOOKUP
    db_path = "chips_database.json"
    if os.path.exists(db_path):
        try:
            with open(db_path, 'r') as f:
                CHIP_DB = json.load(f)
            for vendor, chips in CHIP_DB.get("Chips", {}).items():
                for chip_name, chip_data in chips.items():
                    if "jedec_id" in chip_data:
                        full_id = chip_data["jedec_id"]
                        if len(full_id) >= 6: JEDEC_LOOKUP[full_id[:6]] = (vendor, chip_name)
                        if len(full_id) >= 8: JEDEC_LOOKUP[full_id[:8]] = (vendor, chip_name)
                        JEDEC_LOOKUP[full_id] = (vendor, chip_name)
            return True
        except Exception as e:
            return False
    return False

if not load_chip_database():
    CHIP_DB = { "Chips": {} }

PROTO_DB = {
    "protocols": {
        "SPI_25xx": {
            "name": "SPI Flash (25xx)", "header": "SPI2", "package": "SOP8_SPI",
            "pins": {
                "CS":   "SPI2 [Pin 5]", "MISO": "SPI2 [Pin 1]", "WP":   "SPI1 [Pin 2]",
                "GND":  "SPI2 [Pin 6]", "MOSI": "SPI2 [Pin 4]", "CLK":  "SPI2 [Pin 3]",
                "HOLD": "SPI1 [Pin 2]", "VCC":  "SPI2 [Pin 2]"
            }
        },
        "SPI_AT45": {
            "name": "SPI DataFlash (45DB)", "header": "SPI2", "package": "SOP8_AT45",
            "pins": { 
                "CS": "SPI2 [Pin 5]", "SO (MISO)": "SPI2 [Pin 1]", "WP": "TTL [Pin 9]", 
                "GND": "SPI2 [Pin 6]", "SI (MOSI)": "SPI2 [Pin 4]", "SCK": "SPI2 [Pin 3]", 
                "RESET": "TTL [Pin 9]", "VCC": "SPI2 [Pin 2]" 
            }
        },
        "I2C_24xx": {
            "name": "I2C EEPROM (24xx)", "header": "SPI2", "package": "SOP8_I2C",
            "pins": {
                "SCL": "SPI2 [Pin 3]", 
                "SDA": "SPI2 [Pin 4] + [Pin 1]", 
                "VCC": "SPI2 [Pin 2]", 
                "GND": "SPI2 [Pin 6]", 
                "WP":  "SPI1 [Pin 6] (To Enable Write)"
            }
        }
    },
    "i2c_chips": {
        "24C01": {"size": 128}, "24C02": {"size": 256}, "24C04": {"size": 512}, 
        "24C08": {"size": 1024}, "24C16": {"size": 2048}, "24C32": {"size": 4096}, 
        "24C64": {"size": 8192}, "24C512": {"size": 65536}
    }
}

# ==========================================
# 2. HARDWARE BACKEND
# ==========================================
class TiaoBackend:
    def __init__(self):
        self.VID = 0x0403
        self.PID = 0x8a98
        self.TIAO_URL = f'ftdi://{hex(self.VID)}:{hex(self.PID)}/2'
        self.SERIAL_URL = f'ftdi://{hex(self.VID)}:{hex(self.PID)}/2'
        if PYFTDI_AVAILABLE:
            try: Ftdi.add_custom_product(self.VID, self.PID)
            except (ValueError, TypeError): pass
        self.spi = None; self.i2c = None

    def connect(self):
        if not PYFTDI_AVAILABLE: return False
        try: return True
        except: return False

    def get_spi_port(self, freq=3000000):
        if not self.spi: 
            self.spi = SpiController(cs_count=1)
            self.spi.configure(self.TIAO_URL, turbo=False)
        return self.spi.get_port(cs=0, freq=freq, mode=0)

    def get_i2c_port(self):
        if not self.i2c: self.i2c = I2cController(); self.i2c.configure(self.TIAO_URL)
        return self.i2c.get_port(0x50)

    def _log(self, logger, msg):
        if not logger: return
        try: logger.emit(msg)
        except AttributeError: logger(msg)

    def scan_spi_id(self, log_signal=None):
        if not PYFTDI_AVAILABLE: return "LIB_ERROR"
        try:
            if self.spi: 
                try: self.spi.close()
                except: pass
                self.spi = None
            if log_signal: self._log(log_signal, "Scanning SPI @ 50kHz...")
            port = self.get_spi_port(freq=50000) 
            jedec = port.exchange([0x9F], 3)
            if self.spi: self.spi.close(); self.spi = None
            hex_id = jedec.hex().upper()
            if hex_id == "000000": return "ERROR_ZEROS"
            if hex_id == "FFFFFF": return "ERROR_FLOATING"
            return jedec
        except Exception as e: return f"ERROR: {e}"

    def check_presence(self, protocol):
        if not PYFTDI_AVAILABLE: return False, "Lib Error"
        try:
            if "SPI" in protocol:
                res = self.scan_spi_id()
                if isinstance(res, str) and "ERROR" in res: return False, res, None
                return True, f"SPI Chip OK", res
            elif "I2C" in protocol:
                tmp_i2c = I2cController(); tmp_i2c.configure(self.TIAO_URL)
                try: tmp_i2c.get_port(0x50).read_from(0x00, 1); return True, "I2C Chip OK", None
                except: return False, "I2C NACK", None
                finally: tmp_i2c.terminate()
            return True, "Ready", None
        except Exception as e: return False, str(e), None

    def detect_board(self):
        if not PYFTDI_AVAILABLE: return False, "PyFTDI Library Missing"
        try:
            devs = UsbTools.find_all([(self.VID, self.PID)])
            if not devs: return False, "TIAO Adapter Not Found"
            return True, "TIAO Adapter V2 Connected"
        except Exception as e: return True, str(e)

    def close(self):
        if self.spi: self.spi.close()
        if self.i2c: self.i2c.terminate()

class ProgrammerWorker(QThread):
    progress = pyqtSignal(int); log = pyqtSignal(str); finished_data = pyqtSignal(bytes); finished_success = pyqtSignal(bool)
    def __init__(self, mode, protocol, size, data=None):
        super().__init__(); self.mode = mode; self.protocol = protocol; self.size = size; self.write_data = data; self.backend = TiaoBackend(); self.running = True
    def stop(self): self.running = False
    def run(self):
        try:
            if "SPI" in self.protocol:
                if self.mode == 'read': self.spi_read()
                elif self.mode == 'erase': self.spi_erase()
                elif self.mode == 'write': self.spi_write()
                elif self.mode == 'verify': self.spi_verify()
            elif "I2C" in self.protocol:
                if self.mode == 'read': self.i2c_read()
        except Exception as e: 
            self.log.emit(f"❌ Error: {str(e)}"); self.finished_success.emit(False)
        finally: self.backend.close()

    def spi_read(self):
        self.log.emit(f"Reading {self.size} bytes (SPI2)...")
        data = bytearray(); chunk = 4096
        try: slave = self.backend.get_spi_port(freq=3000000)
        except Exception as e: self.log.emit(f"❌ HW Init Error: {e}"); self.finished_success.emit(False); return
        if "AT45" in self.protocol:
             cmd = [0xD2, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00]
             slave.write(cmd, start=True, stop=False)
        else:
             slave.write([0x03, 0x00, 0x00, 0x00], start=True, stop=False)
        for i in range(0, self.size, chunk):
            if not self.running: break
            data.extend(slave.read(min(chunk, self.size - i), start=False, stop=False))
            self.progress.emit(int((i / self.size) * 100))
        slave.write([], start=False, stop=True)
        if self.running: self.progress.emit(100); self.finished_data.emit(bytes(data)); self.finished_success.emit(True)
        else: self.finished_success.emit(False)

    def spi_erase(self):
        self.log.emit("Erasing (SPI2)..."); 
        try: slave = self.backend.get_spi_port(freq=3000000)
        except Exception as e: self.log.emit(f"❌ HW Error: {e}"); self.finished_success.emit(False); return
        if "AT45" in self.protocol:
            self.log.emit("Unlocking AT45 Protection...")
            slave.write([0x3D, 0x2A, 0x7F, 0x9A]); time.sleep(0.1)
            slave.exchange([0xC7, 0x94, 0x80, 0x9A], 0); status_cmd = [0xD7]
        else:
            slave.exchange([0x06], 0); slave.exchange([0x60], 0); status_cmd = [0x05]
        while self.running:
            status = slave.exchange(status_cmd, 1)[0]
            if "AT45" in self.protocol:
                if (status & 0x80): break 
            else:
                if not (status & 0x01): break 
            time.sleep(0.5)
        if self.running: self.log.emit("✅ Erased."); self.finished_success.emit(True)
        else: self.finished_success.emit(False)

    def spi_write(self):
        if "AT45" in self.protocol: self.log.emit("⚠️ AT45 Write complex. Implemented generic only.")
        self.log.emit("Writing (SPI2)..."); 
        try: slave = self.backend.get_spi_port(freq=3000000); data = self.write_data
        except Exception as e: self.log.emit(f"❌ HW Error: {e}"); self.finished_success.emit(False); return
        for i in range(0, len(data), 256):
            if not self.running: break
            slave.exchange([0x06], 0); cmd = [0x02] + [(i>>16)&0xFF, (i>>8)&0xFF, i&0xFF]
            slave.write(cmd, start=True, stop=False); slave.write(data[i:i+256], start=False, stop=True)
            while slave.exchange([0x05], 1)[0] & 0x01: pass
            self.progress.emit(int((i / len(data)) * 100))
        if self.running: self.progress.emit(100); self.log.emit("✅ Written."); self.finished_success.emit(True)
        else: self.finished_success.emit(False)

    def spi_verify(self):
        self.log.emit("Verifying (SPI2)..."); 
        try: slave = self.backend.get_spi_port(freq=3000000); orig = self.write_data; chunk = 4096
        except Exception as e: self.log.emit(f"❌ HW Error: {e}"); self.finished_success.emit(False); return
        if "AT45" in self.protocol: cmd = [0xD2, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00]
        else: cmd = [0x03, 0x00, 0x00, 0x00]
        slave.write(cmd, start=True, stop=False)
        for i in range(0, len(orig), chunk):
            if not self.running: break
            read = slave.read(min(chunk, len(orig)-i), start=False, stop=False)
            if read != orig[i:i+len(read)]:
                slave.write([], start=False, stop=True); self.log.emit("❌ Mismatch!"); self.finished_success.emit(False); return
            self.progress.emit(int((i / len(orig)) * 100))
        slave.write([], start=False, stop=True)
        if self.running: self.log.emit("✅ Verified."); self.finished_success.emit(True)
        else: self.finished_success.emit(False)

    def i2c_read(self):
        self.log.emit("Reading I2C..."); port = self.backend.get_i2c_port(); port.write_to(0x00, [])
        data = port.read_from(0x00, self.size); self.finished_data.emit(data); self.finished_success.emit(True)

# ==========================================
# AUTO BAUD DETECTION (WAIT FOR DATA)
# ==========================================
class AutoBaudWorker(QThread):
    result_signal = pyqtSignal(int, str)
    progress_signal = pyqtSignal(str)

    def __init__(self, backend_url):
        super().__init__()
        self.url = backend_url
        self.rates = [115200, 57600, 38400, 19200, 9600, 230400, 460800, 921600]
        self.running = True

    def run(self):
        self.progress_signal.emit("--- Starting Smart Auto-Detect ---")
        self.progress_signal.emit("Waiting INDEFINITELY for data at each speed.")
        
        for rate in self.rates:
            if not self.running: break
            self.progress_signal.emit(f"\n[TESTING] {rate} baud...")
            self.progress_signal.emit("Waiting for device to send data...")
            try:
                port = pyftdi.serialext.serial_for_url(self.url, baudrate=rate, timeout=0.1)
                first_byte = b""
                while self.running and not first_byte:
                    try:
                        first_byte = port.read(1)
                        if not first_byte: time.sleep(0.01)
                    except Exception: pass
                
                if not self.running: port.close(); break

                self.progress_signal.emit(">> Data Received! Analyzing...")
                data_buffer = first_byte
                t_end = time.time() + 0.5
                while time.time() < t_end:
                    chunk = port.read(128)
                    if chunk: data_buffer += chunk
                port.close()
                time.sleep(0.1) 
                
                if len(data_buffer) > 0:
                    printable = 0
                    for byte in data_buffer:
                        if 32 <= byte <= 126 or byte in [9, 10, 13]: printable += 1
                    score = (printable / len(data_buffer)) * 100
                    preview = data_buffer[:20].decode('ascii', errors='replace').replace('\n',' ')
                    self.progress_signal.emit(f"Sample: {preview}...")
                    self.progress_signal.emit(f"Score: {int(score)}% readable")
                    if score > 70:
                        self.result_signal.emit(rate, f"SUCCESS! Detected {rate} baud.")
                        return
                    else:
                        self.progress_signal.emit("-> Garbage. Trying next rate...")
            except Exception as e: self.progress_signal.emit(f"Error: {e}")
        self.result_signal.emit(0, "Stopped or no valid rate found.")

# ==========================================
# SERIAL WORKER WITH AUTO-INTERRUPT
# ==========================================
class SerialWorker(QThread):
    rx_data = pyqtSignal(bytes)
    
    def __init__(self, backend_ref, settings, interrupt_enabled=False, interrupt_key_index=0):
        super().__init__()
        self.url = backend_ref.SERIAL_URL 
        self.settings = settings
        self.running = True; self.port = None; self.capture_file = None
        
        self.interrupt_enabled = interrupt_enabled
        self.interrupt_key = b'\n'
        if interrupt_key_index == 1: self.interrupt_key = b' '
        elif interrupt_key_index == 2: self.interrupt_key = b'\x03' # Ctrl+C
        elif interrupt_key_index == 3: self.interrupt_key = b'\x1b' # ESC
        self.interrupt_triggered = False
        
    def start_capture(self, filename):
        try: self.capture_file = open(filename, 'wb'); return True
        except: return False
        
    def stop_capture(self):
        if self.capture_file: self.capture_file.close(); self.capture_file = None
        
    def run(self):
        try:
            self.port = pyftdi.serialext.serial_for_url(self.url, **self.settings, timeout=0.1)
            self.rx_data.emit(b"\n[SUCCESS: Port Opened]\n")
            if self.interrupt_enabled:
                self.rx_data.emit(b"[SYSTEM ARMED: Waiting for first byte to SPAM interrupt key...]\n")
            
            while self.running:
                try:
                    data = self.port.read(4096)
                    if data:
                        if self.interrupt_enabled and not self.interrupt_triggered:
                            self.interrupt_triggered = True
                            self.rx_data.emit(b"\n[!!! TRIGGER DETECTED - SPAMMING KEY !!!]\n")
                            for _ in range(50):
                                self.port.write(self.interrupt_key)
                                time.sleep(0.02)
                            self.rx_data.emit(b"\n[!!! SPAM COMPLETE !!!]\n")

                        if self.capture_file: 
                            self.capture_file.write(data)
                            self.capture_file.flush()
                        self.rx_data.emit(data)
                except Exception:
                    pass
        except Exception as e: 
            self.rx_data.emit(f"\n[Error: {e}]\n".encode())
        finally:
            if self.port: 
                try: self.port.close()
                except: pass
            if self.capture_file: self.capture_file.close()
            time.sleep(0.1)
            
    def send(self, data_bytes):
        if self.port and self.port.is_open: 
            try: self.port.write(data_bytes)
            except Exception as e: pass
    def stop(self): self.running = False

# ==========================================
# 3. GUI TABS
# ==========================================
class ChipBuilderTab(QWidget):
    TIAO_SPI2_MAP = {
        "MISO": "SPI2 [Pin 1] (MISO)", "SCK":  "SPI2 [Pin 3] (SCK)",
        "CLK":  "SPI2 [Pin 3] (SCK)", "MOSI": "SPI2 [Pin 4] (MOSI)",
        "CS":   "SPI2 [Pin 5] (CS)", "SDA":  "SPI2 [Pin 4] + [Pin 1] (SDA)", 
        "SCL":  "SPI2 [Pin 3] (SCL)"
    }
    TIAO_3V3_POOL = ["SPI2 [Pin 2]", "SPI1 [Pin 2] (Aux)", "TTL [Pin 9]"]
    TIAO_5V_POOL  = ["TTL [Pin 10] (5V)", "DB9 [Pin 10] (5V)"]
    TIAO_GND_POOL = ["SPI2 [Pin 6]", "SPI1 [Pin 6]", "TTL [Pin 11]"]

    SIZE_OPTIONS = {
        "64 KB / 512 Kb": 64, "128 KB / 1 Mb": 128, "256 KB / 2 Mb": 256,
        "512 KB / 4 Mb": 512, "1 MB / 8 Mb": 1024, "2 MB / 16 Mb": 2048,
        "4 MB / 32 Mb": 4096, "8 MB / 64 Mb": 8192, "16 MB / 128 Mb": 16384,
        "32 MB / 256 Mb": 32768, "64 MB / 512 Mb": 65536, "128 MB / 1 Gb": 131072
    }

    def __init__(self):
        super().__init__(); self.json_file = "chips_database.json"; self.pin_combos = []; self.initUI()
        
    def initUI(self):
        layout = QHBoxLayout(self); splitter = QSplitter(Qt.Horizontal); layout.addWidget(splitter)
        left_widget = QWidget(); left_layout = QVBoxLayout(left_widget)
        grp_cfg = QGroupBox("New Chip Identity"); gl = QGridLayout(grp_cfg)
        self.txt_vendor = QLineEdit(); self.txt_vendor.setPlaceholderText("e.g. Macronix")
        gl.addWidget(QLabel("Vendor:"), 0, 0); gl.addWidget(self.txt_vendor, 0, 1)
        self.txt_name = QLineEdit(); self.txt_name.setPlaceholderText("e.g. MX25L6406E")
        self.txt_id = QLineEdit(); self.txt_id.setPlaceholderText("e.g. 0xc22017")
        gl.addWidget(QLabel("Chip Name:"), 1, 0); gl.addWidget(self.txt_name, 1, 1)
        gl.addWidget(QLabel("Chip ID (Hex):"), 2, 0); gl.addWidget(self.txt_id, 2, 1)
        self.combo_size = QComboBox(); self.combo_size.addItems(self.SIZE_OPTIONS.keys())
        self.combo_size.setCurrentText("4 MB / 32 Mb")
        gl.addWidget(QLabel("Size:"), 3, 0); gl.addWidget(self.combo_size, 3, 1)
        self.rb_3v = QRadioButton("3.3V (Standard)"); self.rb_3v.setChecked(True)
        self.rb_5v = QRadioButton("5V (Legacy)")
        bg_volt = QButtonGroup(self); bg_volt.addButton(self.rb_3v); bg_volt.addButton(self.rb_5v)
        v_layout = QHBoxLayout(); v_layout.addWidget(self.rb_3v); v_layout.addWidget(self.rb_5v)
        gl.addWidget(QLabel("Voltage:"), 4, 0); gl.addLayout(v_layout, 4, 1)
        self.rb_8pin = QRadioButton("8 Pins"); self.rb_8pin.setChecked(True)
        self.rb_16pin = QRadioButton("16 Pins")
        bg_pins = QButtonGroup(self); bg_pins.addButton(self.rb_8pin); bg_pins.addButton(self.rb_16pin)
        p_layout = QHBoxLayout(); p_layout.addWidget(self.rb_8pin); p_layout.addWidget(self.rb_16pin)
        bg_pins.buttonClicked.connect(self.update_pin_grid)
        self.rb_3v.toggled.connect(self.generate_wiring); self.rb_5v.toggled.connect(self.generate_wiring)
        gl.addWidget(QLabel("Package:"), 5, 0); gl.addLayout(p_layout, 5, 1)
        left_layout.addWidget(grp_cfg)
        self.scroll = QScrollArea(); self.scroll.setWidgetResizable(True)
        self.pin_container = QWidget(); self.pin_layout = QGridLayout(self.pin_container)
        self.scroll.setWidget(self.pin_container); left_layout.addWidget(QLabel("<b>Pin Function Map (Check Datasheet):</b>")); left_layout.addWidget(self.scroll)
        btn_layout = QHBoxLayout(); btn_clear = QPushButton("Clear"); btn_clear.clicked.connect(self.clear_form)
        btn_save = QPushButton("Save to Database"); btn_save.setStyleSheet("background:#00aa00; font-weight:bold"); btn_save.clicked.connect(self.save_to_json)
        btn_layout.addWidget(btn_clear); btn_layout.addWidget(btn_save); left_layout.addLayout(btn_layout)
        right_widget = QWidget(); right_layout = QVBoxLayout(right_widget)
        right_layout.addWidget(QLabel("<b>Auto-Generated TIAO Wiring Guide:</b>")); self.txt_wiring = QTextEdit(); self.txt_wiring.setReadOnly(True); self.txt_wiring.setStyleSheet("background:#111; color:#0ff; font-family:Consolas; font-size:11pt"); right_layout.addWidget(self.txt_wiring)
        splitter.addWidget(left_widget); splitter.addWidget(right_widget); splitter.setStretchFactor(0, 3); splitter.setStretchFactor(1, 2); self.update_pin_grid()

    def update_pin_grid(self):
        for i in reversed(range(self.pin_layout.count())): self.pin_layout.itemAt(i).widget().setParent(None)
        self.pin_combos = []; count = 16 if self.rb_16pin.isChecked() else 8; functions = ["NC", "VCC", "GND", "CS", "MOSI", "MISO", "CLK", "SDA", "SCL", "WP", "HOLD", "RESET"]
        for i in range(count):
            lbl = QLabel(f"Pin {i+1}:"); cb = QComboBox(); cb.addItems(functions); cb.currentIndexChanged.connect(self.generate_wiring); self.pin_combos.append(cb)
            row = i // 2; col = (i % 2) * 2; self.pin_layout.addWidget(lbl, row, col); self.pin_layout.addWidget(cb, row, col+1)
        self.generate_wiring()

    def generate_wiring(self):
        wiring_text = f"=== WIRING GUIDE FOR {self.txt_name.text().upper() or 'NEW CHIP'} ===\n"; is_5v = self.rb_5v.isChecked()
        wiring_text += "⚡ VOLTAGE: 5V (Legacy Mode)\n⚠️ WARNING: Do NOT use SPI Headers (3.3V Only)!\n\n" if is_5v else "⚡ VOLTAGE: 3.3V (Standard TIAO)\n🌐 PORT: SPI2 (Port B) Priority\n\n"
        valid_pins = 0; vcc_idx = 0; gnd_idx = 0
        for i, cb in enumerate(self.pin_combos):
            func = cb.currentText(); pin_num = i + 1
            if func == "NC": continue
            valid_pins += 1; dest = "Unknown / Custom"
            if func == "VCC": pool = self.TIAO_5V_POOL if is_5v else self.TIAO_3V3_POOL; dest = pool[vcc_idx % len(pool)]; vcc_idx += 1
            elif func == "GND": pool = self.TIAO_GND_POOL; dest = pool[gnd_idx % len(pool)]; gnd_idx += 1
            elif func in self.TIAO_SPI2_MAP: dest = self.TIAO_SPI2_MAP[func]
            elif func in ["WP", "HOLD", "RESET"]: dest = "TTL [Pin 10] (5V) or Pull-Up" if is_5v else f"{self.TIAO_3V3_POOL[vcc_idx % len(self.TIAO_3V3_POOL)]} (High)"; vcc_idx += 1
            wiring_text += f"Chip Pin {pin_num:<2} ({func}) --> {dest}\n"
        self.txt_wiring.setText(wiring_text)

    def validate_config(self):
        counts = {}; errors = []
        for cb in self.pin_combos: f = cb.currentText(); counts[f] = counts.get(f, 0) + 1
        for func, count in counts.items():
            if count > 1 and func not in ["NC", "VCC", "GND"]: errors.append(f"Duplicate Signal Pin: {func}")
        if not self.txt_vendor.text(): errors.append("Vendor is required.")
        if not self.txt_name.text(): errors.append("Chip Name is required.")
        if errors: QMessageBox.critical(self, "Validation Error", "\n".join(errors)); return False
        return True

    def save_to_json(self):
        if not self.validate_config(): return
        vendor = self.txt_vendor.text().strip()
        chip_name = self.txt_name.text().strip()
        size_str = self.combo_size.currentText()
        size_kb = self.SIZE_OPTIONS[size_str]
        pins = {f"Pin {i+1}": cb.currentText() for i, cb in enumerate(self.pin_combos)}
        entry = {
            "bus": "SPI", "jedec_id": self.txt_id.text().strip(), "size_kb": size_kb, "size_str": size_str,
            "voltage_min_mv": 4500 if self.rb_5v.isChecked() else 2700, "voltage_max_mv": 5500 if self.rb_5v.isChecked() else 3600,
            "package": "16 Pin" if self.rb_16pin.isChecked() else "8 Pin", "pinout": pins, "name": chip_name
        }
        db = { "Chips": {} }
        if os.path.exists(self.json_file):
            try:
                with open(self.json_file, 'r') as f: db = json.load(f)
            except: pass
        if vendor not in db["Chips"]: db["Chips"][vendor] = {}
        db["Chips"][vendor][chip_name] = entry
        try:
            with open(self.json_file, 'w') as f: json.dump(db, f, indent=4)
            load_chip_database()
            QMessageBox.information(self, "Success", f"Saved {chip_name} to Database.\nIt will now appear in Manual Selection.")
            self.clear_form()
        except Exception as e: QMessageBox.critical(self, "Save Error", str(e))

    def clear_form(self):
        self.txt_vendor.clear(); self.txt_name.clear(); self.txt_id.clear()
        self.rb_3v.setChecked(True); self.rb_8pin.setChecked(True); self.update_pin_grid()

    def prefill_from_detect(self, jedec_id):
        self.clear_form()
        self.txt_id.setText(jedec_id)
        self.txt_vendor.setFocus()
        QMessageBox.information(self, "New Chip", f"JEDEC ID {jedec_id} detected.\nPlease fill in Vendor, Name, and Size to add to database.")

class ProgrammerTab(QWidget):
    # ... (No changes here, standard code) ...
    def __init__(self, backend, db):
        super().__init__()
        self.backend = backend
        self.db = db
        self.buffer = b""
        self.detected_chip_data = None
        self.worker = None
        self.initUI()

    def initUI(self):
        layout = QHBoxLayout(self)
        main_splitter = QSplitter(Qt.Horizontal)
        layout.addWidget(main_splitter)
        left_widget = QWidget(); left_vbox = QVBoxLayout(left_widget); left_vbox.setContentsMargins(0, 0, 0, 0)
        top_left_container = QWidget(); top_left_layout = QVBoxLayout(top_left_container); top_left_layout.setContentsMargins(0, 0, 0, 0)
        grp_detect = QGroupBox("1. Automatic Detection"); l_det = QVBoxLayout(grp_detect)
        btn_det = QPushButton("Automatic Detection"); btn_det.setStyleSheet("background:#0077cc; height:35px; font-weight:bold; font-size:11pt;"); btn_det.clicked.connect(self.detect_chip)
        self.lbl_chip_status = QLabel("Status: Waiting for detection..."); self.lbl_chip_status.setStyleSheet("background:#222; padding:8px; border-radius:4px; color:yellow;"); self.lbl_chip_status.setWordWrap(True)
        l_det.addWidget(btn_det); l_det.addWidget(self.lbl_chip_status); top_left_layout.addWidget(grp_detect)
        grp_manual = QGroupBox("2. Manual Selection"); l_man = QGridLayout(grp_manual)
        self.combo_proto = QComboBox(); self.combo_proto.addItems(self.db['protocols'].keys()); self.combo_proto.currentTextChanged.connect(self.update_manual_options)
        self.combo_vendor = QComboBox(); self.combo_vendor.currentTextChanged.connect(self.update_chip_list)
        self.combo_chip = QComboBox(); 
        self.btn_manual_select = QPushButton("Manual Connection"); 
        self.btn_manual_select.clicked.connect(self.use_manual_selection)
        l_man.addWidget(QLabel("Protocol:"), 0, 0); l_man.addWidget(self.combo_proto, 0, 1)
        l_man.addWidget(QLabel("Vendor:"), 1, 0); l_man.addWidget(self.combo_vendor, 1, 1)
        l_man.addWidget(QLabel("Chip:"), 2, 0); l_man.addWidget(self.combo_chip, 2, 1); l_man.addWidget(self.btn_manual_select, 3, 0, 1, 2)
        top_left_layout.addWidget(grp_manual)
        self.grp_ops = QGroupBox("3. Operations"); l_ops = QGridLayout(self.grp_ops)
        self.btn_read = QPushButton("READ"); self.btn_read.setStyleSheet("background:#00aa00; font-size:11pt; font-weight:bold;"); self.btn_read.setMinimumHeight(45); self.btn_read.clicked.connect(self.do_read)
        self.btn_write = QPushButton("WRITE"); self.btn_write.setStyleSheet("background:#cc6600; font-size:11pt; font-weight:bold;"); self.btn_write.setMinimumHeight(45); self.btn_write.clicked.connect(self.do_write)
        self.btn_erase = QPushButton("ERASE"); self.btn_erase.setStyleSheet("background:#aa0000; font-size:11pt; font-weight:bold;"); self.btn_erase.setMinimumHeight(45); self.btn_erase.clicked.connect(self.do_erase)
        self.btn_stop = QPushButton("STOP"); self.btn_stop.setStyleSheet("background:red; font-weight:bold"); self.btn_stop.setMinimumHeight(45); self.btn_stop.setEnabled(False); self.btn_stop.clicked.connect(self.do_stop)
        self.chk_verify = QCheckBox("Verify after Write"); self.chk_verify.setChecked(True)
        l_ops.addWidget(self.btn_read, 0, 0); l_ops.addWidget(self.btn_write, 0, 1); l_ops.addWidget(self.btn_erase, 1, 0); l_ops.addWidget(self.btn_stop, 1, 1); l_ops.addWidget(self.chk_verify, 2, 0, 1, 2)
        top_left_layout.addWidget(self.grp_ops)
        self.grp_file = QGroupBox("File I/O"); l_f = QHBoxLayout(self.grp_file)
        b_save = QPushButton("Save Buffer"); b_save.clicked.connect(self.save_f); b_load = QPushButton("Load Buffer"); b_load.clicked.connect(self.load_f)
        l_f.addWidget(b_save); l_f.addWidget(b_load); top_left_layout.addWidget(self.grp_file); top_left_layout.addStretch()
        self.log = QTextEdit(); self.log.setReadOnly(True); self.log.setStyleSheet("background:black; color:#0f0; font-family:Consolas; border:1px solid #555")
        left_splitter = QSplitter(Qt.Vertical); left_splitter.addWidget(top_left_container); left_splitter.addWidget(self.log); left_splitter.setStretchFactor(0, 0); left_splitter.setStretchFactor(1, 1); left_vbox.addWidget(left_splitter)
        right_widget = QWidget(); right_vbox = QVBoxLayout(right_widget); right_vbox.setContentsMargins(0,0,0,0)
        self.table = QTableWidget(); self.table.setColumnCount(3); self.table.setHorizontalHeaderLabels(["Generic Connection", "Function", "Chip Pin"])
        self.table.horizontalHeader().setSectionResizeMode(QHeaderView.Stretch); self.table.setStyleSheet("background:#222; color:white; gridline-color:#555"); self.table.setMinimumHeight(200)
        self.hex = QTextEdit(); self.hex.setReadOnly(True); self.hex.setStyleSheet("background:#1a1a1a; color:cyan; font-family:Consolas")
        self.pbar = QProgressBar(); self.pbar.setValue(0); self.pbar.setStyleSheet("text-align:center; background:#333; color:white;")
        right_splitter = QSplitter(Qt.Vertical); right_splitter.addWidget(self.table); right_splitter.addWidget(QLabel("Buffer Preview (First 16KB):")); right_splitter.addWidget(self.hex); right_splitter.addWidget(self.pbar); right_vbox.addWidget(right_splitter)
        main_splitter.addWidget(left_widget); main_splitter.addWidget(right_widget); main_splitter.setStretchFactor(0, 1); main_splitter.setStretchFactor(1, 2)
        self.update_manual_options(self.combo_proto.currentText())
        self.toggle_ops_buttons(False)

    def log_msg(self, t, color="#00ff00"): 
        self.log.setTextColor(QColor(color)); self.log.append(f"> {t}")
    def set_active_chip(self, chip_data):
        self.detected_chip_data = chip_data
        if chip_data:
            status_text = f"<b>{chip_data.get('name', 'Unknown')}</b><br>JEDEC ID: {chip_data.get('jedec_id', 'N/A')}<br>Size: {chip_data.get('size_str', 'N/A')}"
            self.lbl_chip_status.setText(status_text); self.lbl_chip_status.setStyleSheet("background:#004400; padding:8px; border-radius:4px; color:lime;"); self.toggle_ops_buttons(True)
        else:
            self.lbl_chip_status.setText("Status: No chip selected."); self.lbl_chip_status.setStyleSheet("background:#222; padding:8px; border-radius:4px; color:yellow;"); self.toggle_ops_buttons(False)
    def get_size(self):
        if self.detected_chip_data: return self.detected_chip_data.get("size_kb", 0) * 1024 or self.detected_chip_data.get("size", 0)
        return 0
    def update_manual_options(self, proto):
        self.combo_vendor.clear(); self.combo_chip.clear(); is_spi = "SPI" in proto; self.combo_vendor.setEnabled(is_spi); self.btn_manual_select.setEnabled(True)
        if is_spi: self.combo_vendor.addItems([""] + sorted(CHIP_DB.get("Chips", {}).keys()))
        else: self.combo_vendor.addItem("Generic I2C"); self.combo_chip.addItems([""] + list(self.db['i2c_chips'].keys()))
        
        # --- NEW: Dynamic Header Name ---
        if "AT45" in proto:
            self.table.setHorizontalHeaderLabels(["Generic Connection (AT45)", "Function", "Chip Pin"])
        elif "I2C" in proto:
            self.table.setHorizontalHeaderLabels(["Generic Connection (24xx)", "Function", "Chip Pin"])
        else:
            self.table.setHorizontalHeaderLabels(["Generic Connection (25xx)", "Function", "Chip Pin"])
            
        self.update_wiring_table(proto)
        
    def update_chip_list(self, vendor):
        self.combo_chip.clear()
        if vendor: self.combo_chip.addItems([""] + sorted(CHIP_DB.get("Chips", {}).get(vendor, {}).keys()))
    def update_wiring_table(self, proto):
        self.table.setRowCount(0); pins = self.db['protocols'][proto]['pins']; pkg = PACKAGE_MAP.get(self.db['protocols'][proto].get('package'), {}); self.table.setRowCount(len(pins))
        for r, (func, tiao_pin) in enumerate(pins.items()):
            self.table.setItem(r, 0, QTableWidgetItem(tiao_pin))
            self.table.setItem(r, 1, QTableWidgetItem(func))
            self.table.setItem(r, 2, QTableWidgetItem(str(pkg.get(func,'-'))))
    def detect_chip(self):
        self.set_active_chip(None); res = self.backend.scan_spi_id(self.log_msg)
        if isinstance(res, str): self.log_msg(f"❌ {res}", "#ff5555"); return
        jedec_bytes = bytes(res); jedec_id_str = "0x" + jedec_bytes.hex()
        if jedec_id_str == "0x000000" or jedec_id_str == "0xffffff":
             self.log_msg(f"❌ Invalid ID ({jedec_id_str}). Check connections.", "#ff5555"); return
        for i in [3, 2]:
            key = "0x" + jedec_bytes[:i].hex()
            if key in JEDEC_LOOKUP:
                vendor, chip_name = JEDEC_LOOKUP[key]; self.log_msg(f"✅ Match Found: {chip_name}", "lime"); self.set_active_chip(CHIP_DB["Chips"][vendor][chip_name]); return
        self.log_msg(f"⚠️ Unknown JEDEC ID: {jedec_id_str}", "yellow")
        if QMessageBox.question(self, "Unknown Chip", f"JEDEC ID {jedec_id_str} not found in database.\n\nWould you like to add it now?", QMessageBox.Yes | QMessageBox.No) == QMessageBox.Yes:
            self.window().tabs.setCurrentIndex(1); self.window().builder_tab.prefill_from_detect(jedec_id_str)
    def use_manual_selection(self):
        proto = self.combo_proto.currentText()
        selected_chip = None
        if "I2C" in proto:
            chip_name = self.combo_chip.currentText()
            if chip_name: cd = self.db['i2c_chips'].get(chip_name); selected_chip = {"name": chip_name, "size": cd['size'], "size_str": f"{cd['size']} bytes", "protocol": "I2C"}
        else:
            vendor = self.combo_vendor.currentText(); chip_name = self.combo_chip.currentText()
            if vendor and chip_name: selected_chip = CHIP_DB.get("Chips", {}).get(vendor, {}).get(chip_name); selected_chip["protocol"] = "SPI"
        if not selected_chip: return
        if "SPI" in selected_chip.get("protocol", ""):
            self.log_msg("Verifying Manual Selection against Hardware...")
            res = self.backend.scan_spi_id(self.log_msg)
            if isinstance(res, str): 
                QMessageBox.critical(self, "Connection Failed", f"Could not communicate with programmer.\nError: {res}\n\nPlease check USB connection.")
                return 
            else:
                detected_hex = "0x" + bytes(res).hex().lower()
                expected_hex = selected_chip.get("jedec_id", "").lower()
                if detected_hex == "0x000000" or detected_hex == "0xffffff":
                     QMessageBox.critical(self, "Connection Error", f"Chip returned invalid ID ({detected_hex}).\nCheck clip/wires and try again.")
                     return
                elif expected_hex and not detected_hex.startswith(expected_hex):
                    reply = QMessageBox.warning(self, "ID Mismatch", 
                        f"<b>Hardware Mismatch Detected!</b><br><br>Selected: {selected_chip['name']} (Expects: {expected_hex})<br>Detected: {detected_hex}<br><br>Enable operations anyway?", QMessageBox.Yes | QMessageBox.No)
                    if reply == QMessageBox.No: return
                    self.log_msg("⚠️ User overrode ID mismatch warning.", "yellow")
                else:
                    self.log_msg("✅ ID Verified matches selection.", "lime")
        elif "I2C" in selected_chip.get("protocol", ""):
            is_present, msg, _ = self.backend.check_presence("I2C_24xx")
            if not is_present:
                 if QMessageBox.warning(self, "I2C Error", f"I2C Scan Failed: {msg}\nForce enable?", QMessageBox.Yes|QMessageBox.No) == QMessageBox.No: return
        self.set_active_chip(selected_chip)
    def preflight_check(self):
        if not self.detected_chip_data: QMessageBox.critical(self, "Error", "No chip selected."); return False
        return True 
    def start_op(self, mode):
        self.toggle_ops_buttons(False, stop=True); proto = "SPI_AT45" if "AT45" in self.detected_chip_data.get("name", "") else ("I2C_24xx" if "24C" in self.detected_chip_data.get('name', '') else "SPI_25xx")
        self.worker = ProgrammerWorker(mode, proto, self.get_size(), self.buffer)
        self.worker.log.connect(self.log_msg); self.worker.progress.connect(self.pbar.setValue); self.worker.finished_data.connect(self.done_read); self.worker.finished_success.connect(self.done_op); self.worker.start()
    def do_read(self): 
        if self.preflight_check(): self.start_op('read')
    def do_write(self): 
        if self.buffer and self.preflight_check(): self.start_op('write')
    def do_erase(self): 
        if self.preflight_check() and QMessageBox.question(self, "Confirm", "Erase Chip?", QMessageBox.Yes|QMessageBox.No) == QMessageBox.Yes: self.start_op('erase')
    def do_stop(self): 
        if self.worker: self.worker.stop()
    def done_read(self, d): 
        self.buffer = d; self.log_msg(f"Read {len(d)} bytes.", "lime"); self.display(d[:16384]) 
    def done_op(self, ok): self.toggle_ops_buttons(True)
    def toggle_ops_buttons(self, enable, stop=False): 
        self.grp_ops.setEnabled(enable or stop)
        self.grp_file.setEnabled(enable)
        self.btn_read.setEnabled(enable); self.btn_write.setEnabled(enable); self.btn_erase.setEnabled(enable); self.btn_stop.setEnabled(stop)
    def save_f(self):
        f, _ = QFileDialog.getSaveFileName(self, "Save", "", "Bin (*.bin)")
        if f: 
            with open(f, 'wb') as h: h.write(self.buffer); self.log_msg(f"Saved {len(self.buffer)} bytes")
    def load_f(self):
        f, _ = QFileDialog.getOpenFileName(self, "Open", "", "All (*.*)")
        if f: 
            with open(f, 'rb') as h: self.buffer = h.read(); self.log_msg(f"Loaded {len(self.buffer)} bytes"); self.display(self.buffer[:16384])
    def display(self, d):
        t = ""
        for i in range(0, len(d), 16):
            c = d[i:i+16]; h = " ".join(f"{b:02X}" for b in c); a = "".join(chr(b) if 32<=b<=126 else "." for b in c)
            t += f"{i:08X}  {h:<48} |{a}|\n"
        if len(self.buffer) > len(d): t += "\n... (Display limited to prevent lag. Save file to view all.)"
        self.hex.setPlainText(t)

class SerialTab(QWidget):
    def __init__(self):
        super().__init__(); self.worker = None; self.auto_worker = None; self.has_unsaved_data = False; self.initUI()
    
    def initUI(self):
        layout = QVBoxLayout(self)
        settings_group = QGroupBox("Configuration"); settings_group.setStyleSheet("QGroupBox { font-weight: bold; border: 1px solid gray; border-radius: 5px; margin-top: 10px; } QGroupBox::title { subcontrol-origin: margin; left: 10px; padding: 0 3px 0 3px; }"); settings_layout = QGridLayout(settings_group)
        self.combo_baud = QComboBox(); self.combo_baud.addItems(["9600", "19200", "38400", "57600", "115200", "230400", "460800", "921600"]); 
        self.combo_baud.setCurrentText("115200"); 
        
        # ROW 0
        settings_layout.addWidget(QLabel("Baud Rate:"), 0, 0); settings_layout.addWidget(self.combo_baud, 0, 1)
        self.combo_data = QComboBox(); self.combo_data.addItems(["5", "6", "7", "8"]); self.combo_data.setCurrentText("8"); settings_layout.addWidget(QLabel("Data Bits:"), 0, 2); settings_layout.addWidget(self.combo_data, 0, 3)
        self.combo_stop = QComboBox(); self.combo_stop.addItems(["1", "1.5", "2"]); self.combo_stop.setCurrentText("1"); settings_layout.addWidget(QLabel("Stop Bits:"), 0, 4); settings_layout.addWidget(self.combo_stop, 0, 5)
        self.btn_auto = QPushButton("Auto-Detect Baud"); self.btn_auto.setStyleSheet("background:#7700cc; font-weight:bold; color:white; height:30px"); self.btn_auto.clicked.connect(self.start_auto_detect)
        settings_layout.addWidget(self.btn_auto, 0, 6)
        
        # ROW 1
        self.combo_parity = QComboBox(); self.combo_parity.addItems(["None", "Even", "Odd", "Mark", "Space"]); settings_layout.addWidget(QLabel("Parity:"), 1, 0); settings_layout.addWidget(self.combo_parity, 1, 1)
        self.combo_flow = QComboBox(); self.combo_flow.addItems(["None", "RTS/CTS", "XON/XOFF"]); settings_layout.addWidget(QLabel("Flow Control:"), 1, 2); settings_layout.addWidget(self.combo_flow, 1, 3)
        
        # ROW 2
        grp_int = QGroupBox("Boot Interrupter"); int_layout = QHBoxLayout(grp_int); int_layout.setContentsMargins(5,5,5,5)
        self.chk_interrupt = QCheckBox("Stop OS"); self.chk_interrupt.setStyleSheet("color:#ff5555; font-weight:bold")
        self.combo_int_key = QComboBox(); self.combo_int_key.addItems(["Enter (\\n)", "Space", "Ctrl+C", "Esc"])
        int_layout.addWidget(self.chk_interrupt); int_layout.addWidget(self.combo_int_key)
        settings_layout.addWidget(grp_int, 2, 0, 1, 3) 

        self.btn_conn = QPushButton("Open Port"); self.btn_conn.setStyleSheet("background:#0077cc; font-weight:bold; height:30px"); self.btn_conn.clicked.connect(self.toggle_conn)
        settings_layout.addWidget(self.btn_conn, 2, 3, 1, 4) 

        layout.addWidget(settings_group)
        
        tools_layout = QHBoxLayout(); self.btn_cap = QPushButton("Start Capture (Download)"); self.btn_cap.setStyleSheet("background:#cc6600"); self.btn_cap.setEnabled(False); self.btn_cap.clicked.connect(self.toggle_capture); tools_layout.addWidget(self.btn_cap)
        btn_save_log = QPushButton("Save Log"); btn_save_log.clicked.connect(self.save_log); tools_layout.addWidget(btn_save_log); btn_clr = QPushButton("Clear Screen"); btn_clr.clicked.connect(self.clear_term); tools_layout.addWidget(btn_clr); layout.addLayout(tools_layout)
        self.term = QTextEdit(); self.term.setReadOnly(True); self.term.setStyleSheet("background:black; color:#00ff00; font-family:Consolas; font-size:12px"); layout.addWidget(self.term)
        bot = QHBoxLayout(); self.inp = QLineEdit(); self.inp.setPlaceholderText("Type command and press ENTER..."); self.inp.setStyleSheet("background:#222; color:white; padding:8px; border:1px solid #555"); self.inp.returnPressed.connect(self.send_data)
        
        # --- FIXED COLOR HERE ---
        self.chk_hex_send = QCheckBox("Hex Mode"); self.chk_hex_send.setStyleSheet("color:black; font-weight:bold; margin-right:5px")
        
        bot.addWidget(QLabel(">")); bot.addWidget(self.inp); bot.addWidget(self.chk_hex_send); layout.addLayout(bot)
    
    def start_auto_detect(self):
        if self.worker: self.toggle_conn()
        self.btn_auto.setEnabled(False); self.btn_auto.setText("Scanning...")
        self.term.append("\n--- Starting Auto-Baud Detection ---\n")
        self.auto_worker = AutoBaudWorker(window.backend.SERIAL_URL)
        self.auto_worker.progress_signal.connect(self.term.append)
        self.auto_worker.result_signal.connect(self.finish_auto_detect)
        self.auto_worker.start()

    def finish_auto_detect(self, rate, msg):
        self.auto_worker = None
        self.btn_auto.setEnabled(True); self.btn_auto.setText("Auto-Detect Baud")
        self.term.append(f"\n[RESULT] {msg}\n")
        if rate > 0:
            self.combo_baud.setCurrentText(str(rate))
            self.toggle_conn()

    def toggle_conn(self):
        if self.worker:
            self.worker.stop(); self.worker.wait(); self.worker = None; self.btn_conn.setText("Open Port"); self.btn_conn.setStyleSheet("background:#0077cc"); self.btn_cap.setEnabled(False); self.btn_cap.setText("Start Capture (Download)"); self.term.append("--- Closed ---")
        else:
            backend_ref = window.backend; parity_map = {"None": 'N', "Even": 'E', "Odd": 'O', "Mark": 'M', "Space": 'S'}; flow_map = {"None": False, "RTS/CTS": True, "XON/XOFF": False}; xon = (self.combo_flow.currentText() == "XON/XOFF")
            settings = {'baudrate': int(self.combo_baud.currentText()), 'bytesize': int(self.combo_data.currentText()), 'stopbits': float(self.combo_stop.currentText()), 'parity': parity_map[self.combo_parity.currentText()], 'rtscts': flow_map[self.combo_flow.currentText()], 'xonxoff': xon}
            try:
                # PASS INTERRUPT SETTINGS TO WORKER
                self.worker = SerialWorker(backend_ref, settings, 
                                         interrupt_enabled=self.chk_interrupt.isChecked(),
                                         interrupt_key_index=self.combo_int_key.currentIndex())
                self.worker.rx_data.connect(self.on_rx); self.worker.start(); self.btn_conn.setText("Close Port"); self.btn_conn.setStyleSheet("background:#cc0000"); self.btn_cap.setEnabled(True); self.term.append(f"--- Connected ---"); self.inp.setFocus()
            except Exception as e: QMessageBox.critical(self, "Error", str(e))
    
    def on_rx(self, data): 
        self.has_unsaved_data = True
        text = data.decode('utf-8', errors='replace')
        def hex_replacer(match):
            hex_str = match.group(0)
            try:
                clean_hex = hex_str.replace(' ', '')
                if len(clean_hex) < 6: return hex_str 
                ascii_text = binascii.unhexlify(clean_hex).decode('ascii', errors='replace')
                clean_ascii = "".join([c if 32 <= ord(c) <= 126 else '.' for c in ascii_text])
                return f"{hex_str}  [ASCII: {clean_ascii}]"
            except:
                return hex_str
        text = re.sub(r'([0-9a-fA-F]{2}\s){3,}[0-9a-fA-F]{2}', hex_replacer, text)
        self.term.moveCursor(QTextCursor.End)
        self.term.insertPlainText(text)
        self.term.moveCursor(QTextCursor.End)

    def send_data(self):
        if not self.worker: return
        raw_text = self.inp.text()
        
        if self.chk_hex_send.isChecked():
            # SMART HYBRID MODE
            hex_str = re.sub(r'[^0-9a-fA-F]', '', raw_text)
            is_valid_hex = (len(hex_str) == len(re.sub(r'\s+', '', raw_text))) and (len(hex_str) > 0)

            if is_valid_hex:
                if len(hex_str) % 2 != 0: hex_str = "0" + hex_str 
                try:
                    data_bytes = binascii.unhexlify(hex_str)
                    self.worker.send(data_bytes)
                    self.term.append(f'<span style="background-color:#444400; color:white;">[TX HEX] {hex_str}</span>')
                except Exception as e:
                    self.term.append(f"\n[TX ERROR] {e}\n")
            else:
                data_bytes = raw_text.encode('utf-8')
                self.worker.send(data_bytes)
                hex_view = data_bytes.hex().upper()
                self.term.append(f'<span style="background-color:#444400; color:white;">[TX TEXT-AS-HEX] {hex_view} ({raw_text})</span>')

        else:
            txt = raw_text + "\n"
            self.worker.send(txt.encode())
            
        self.inp.clear()

    def toggle_capture(self):
        if self.worker.capture_file: self.worker.stop_capture(); self.btn_cap.setText("Start Capture (Download)"); self.btn_cap.setStyleSheet("background:#cc6600"); self.term.append("[Capture Stopped]")
        else:
            f, _ = QFileDialog.getSaveFileName(self, "Save Capture", "", "All Files (*)")
            if f and self.worker.start_capture(f): self.btn_cap.setText("Scanning to File..."); self.btn_cap.setStyleSheet("background:#00aa00"); self.term.append(f"[Capturing to {f}]")
    def save_log(self):
        if self.term.document().isEmpty(): return
        f, _ = QFileDialog.getSaveFileName(self, "Save Log", "serial_log.txt", "Text Files (*.txt)")
        if f:
            with open(f, 'w', encoding='utf-8') as h: h.write(self.term.toPlainText())
            self.has_unsaved_data = False 
    def clear_term(self): self.term.clear(); self.has_unsaved_data = False

class ReferenceTab(QWidget):
    # ... (No changes here, standard code) ...
    def __init__(self): super().__init__(); self.initUI()
    def initUI(self):
        layout = QVBoxLayout(self); scroll = QScrollArea(); scroll.setWidgetResizable(True); layout.addWidget(scroll); content = QWidget(); scroll.setWidget(content); vbox = QVBoxLayout(content); lbl_img = QLabel(); lbl_img.setAlignment(Qt.AlignCenter)
        if os.path.exists("tiao_board.png"): lbl_img.setPixmap(QPixmap("tiao_board.png").scaledToWidth(400, Qt.SmoothTransformation))
        else: lbl_img.setText("Image 'tiao_board.png' not found."); lbl_img.setStyleSheet("color:red; font-size:16px; border:1px dashed red")
        vbox.addWidget(lbl_img); info = QTextEdit(); info.setReadOnly(True); info.setStyleSheet("background:#222; color:#ddd; font-family:Consolas")
        info.setHtml("""<h3>TIAO v2 Pinout Reference</h3>
        <table border='1' cellspacing='0' cellpadding='5' width='100%'>
        <tr><th colspan='2' style='color:cyan'>JTAG (20-Pin)</th><th colspan='2' style='color:cyan'>SPI1 Header</th><th colspan='2' style='color:cyan'>RS232 (DB9)</th></tr>
        <tr><td>1: VTAR</td><td>2: NC</td><td>1: MISO</td><td>2: VCC</td><td>2: RX</td><td>3: TX</td></tr>
        <tr><td>3: nTRST</td><td>4: GND</td><td>3: SCK</td><td>4: MOSI</td><td>5: GND</td><td></td></tr>
        <tr><td>5: TDI</td><td>6: GND</td><td>5: CS</td><td>6: GND</td><td colspan='2' style='color:cyan'><b>TTL-COM (Bottom)</b></td></tr>
        <tr><td>7: TMS</td><td>8: GND</td><td colspan='2' style='color:cyan'><b>SPI2 Header</b></td><td>1: TX</td><td>2: RX</td></tr>
        <tr><td>9: TCK</td><td>10: GND</td><td>1: MISO</td><td>2: VCC</td><td>9: 3.3V</td><td>10: 5V</td></tr>
        <tr><td>11: RTCK</td><td>12: GND</td><td>3: SCK</td><td>4: MOSI</td><td>11: GND</td><td>12: GND</td></tr>
        <tr><td>13: TDO</td><td>14: GND</td><td>5: CS</td><td>6: GND</td><td colspan='2'><b>Jumpers</b></td></tr>
        <tr><td>15: RST</td><td>16: GND</td><td colspan='2'></td><td>Jumper VCC</td><td>3.3V Only</td></tr>
        <tr><td>17: DBGRQ</td><td>18: GND</td><td colspan='2'></td><td>Buffer En</td><td>ON</td></tr>
        <tr><td>19: DBGACK</td><td>20: GND</td><td colspan='2'></td><td>Auto Pwr</td><td>OFF</td></tr>
        </table>"""); info.setMinimumHeight(400); vbox.addWidget(info)

class CreditsTab(QWidget):
    # ... (No changes here, standard code) ...
    def __init__(self): super().__init__(); self.initUI()
    def initUI(self):
        self.setAttribute(Qt.WA_StyledBackground, True); self.setStyleSheet("background-color: #2b2b2b;"); layout = QVBoxLayout(self); layout.setAlignment(Qt.AlignCenter)
        lbl_title = QLabel("TIAO Universal Commander"); lbl_title.setStyleSheet("font-size: 24px; font-weight: bold; color: #00aaff;"); lbl_title.setAlignment(Qt.AlignCenter); layout.addWidget(lbl_title)
        lbl_ver = QLabel("Version 34 (Dynamic Headers & Pinout Fixes)"); lbl_ver.setStyleSheet("font-size: 14px; color: #aaaaaa;"); lbl_ver.setAlignment(Qt.AlignCenter); layout.addWidget(lbl_ver); layout.addSpacing(40)
        lbl_dev = QLabel("Created & Designed by:"); lbl_dev.setStyleSheet("font-size: 16px; color: white;"); lbl_dev.setAlignment(Qt.AlignCenter); layout.addWidget(lbl_dev)
        lbl_peter = QLabel("Peterpt"); lbl_peter.setStyleSheet("font-size: 20px; font-weight: bold; color: #00ff00;"); lbl_peter.setAlignment(Qt.AlignCenter); layout.addWidget(lbl_peter)
        lbl_git = QLabel('<a href="http://github.com/peterpt" style="color: yellow; text-decoration: none;">http://github.com/peterpt</a>'); lbl_git.setOpenExternalLinks(True); lbl_git.setFont(QFont("Consolas", 12)); lbl_git.setAlignment(Qt.AlignCenter); layout.addWidget(lbl_git); layout.addSpacing(40)
        lbl_ack = QLabel("Credits to Flashrom Team for ICS Database:"); lbl_ack.setStyleSheet("font-size: 14px; color: #cccccc;"); lbl_ack.setAlignment(Qt.AlignCenter); layout.addWidget(lbl_ack)
        lbl_flashrom = QLabel('<a href="https://www.flashrom.org" style="color: cyan; text-decoration: none;">https://www.flashrom.org</a>'); lbl_flashrom.setOpenExternalLinks(True); lbl_flashrom.setFont(QFont("Consolas", 12)); lbl_flashrom.setAlignment(Qt.AlignCenter); layout.addWidget(lbl_flashrom)
        layout.addSpacing(20)
        lbl_ai = QLabel("Code Assistance & Architecture:"); lbl_ai.setStyleSheet("font-size: 14px; color: #cccccc;"); lbl_ai.setAlignment(Qt.AlignCenter); layout.addWidget(lbl_ai)
        lbl_ai_name = QLabel("Google Gemini Model"); lbl_ai_name.setStyleSheet("font-size: 16px; font-style: italic; color: cyan; font-weight: bold;"); lbl_ai_name.setAlignment(Qt.AlignCenter); layout.addWidget(lbl_ai_name); layout.addStretch()
        
class TiaoPro(QMainWindow):
    def __init__(self):
        super().__init__()
        self.setWindowTitle("TIAO Universal Commander [PRO]")
        self.setGeometry(100, 100, 1200, 800)
        self.tabs = QTabWidget(); self.tabs.setStyleSheet("""QTabWidget::pane { border: 1px solid #444; } QTabBar::tab { background: #333; color: #aaa; padding: 5px 15px; font-size: 10pt; min-width: 100px; } QTabBar::tab:selected { background: #555; color: white; font-weight: bold; }""")
        self.setCentralWidget(self.tabs)
        self.backend = TiaoBackend()
        self.prog_tab = ProgrammerTab(self.backend, PROTO_DB)
        self.serial_tab = SerialTab()
        self.builder_tab = ChipBuilderTab()
        self.ref_tab = ReferenceTab()
        self.cred_tab = CreditsTab()
        self.tabs.addTab(self.prog_tab, "Programmer"); self.tabs.addTab(self.builder_tab, "Chip Builder"); self.tabs.addTab(self.serial_tab, "Serial Console"); self.tabs.addTab(self.ref_tab, "Board Reference"); self.tabs.addTab(self.cred_tab, "About / Credits")
        self.tabs.currentChanged.connect(self.on_tab_change); self.last_tab = 0; found, msg = self.backend.detect_board(); self.prog_tab.log_msg(msg, "lime" if found else "#ff5555")
    def on_tab_change(self, index):
        prev_tab_widget = self.tabs.widget(self.last_tab)
        if isinstance(prev_tab_widget, SerialTab):
            if prev_tab_widget.has_unsaved_data:
                if QMessageBox.question(self, "Save Log?", "Unsaved serial data. Save it?", QMessageBox.Yes | QMessageBox.No) == QMessageBox.Yes: prev_tab_widget.save_log()
                else: prev_tab_widget.has_unsaved_data = False
            if prev_tab_widget.worker: prev_tab_widget.toggle_conn()
        self.last_tab = index
    def closeEvent(self, event):
        if self.serial_tab.has_unsaved_data:
            reply = QMessageBox.question(self, "Save Log?", "Unsaved serial data. Save before exit?", QMessageBox.Yes | QMessageBox.No | QMessageBox.Cancel)
            if reply == QMessageBox.Yes: self.serial_tab.save_log(); event.accept()
            elif reply == QMessageBox.Cancel: event.ignore()
            else: event.accept()
        else: event.accept()

if __name__ == "__main__":
    app = QApplication(sys.argv)
    window = TiaoPro()
    window.show()
    sys.exit(app.exec_())
