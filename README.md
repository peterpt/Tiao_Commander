    
# TIAO Commander 1.0
<img width="1024" height="741" alt="image" src="https://github.com/user-attachments/assets/5eda1cfd-3541-45a9-be89-bd6bdb70f0e0" />

**A powerful, all-in-one GUI toolkit for the TIAO USB Multi-Protocol Adapter (TUMPA) v2.**

Designed for hardware hackers, reverse engineers, and electronics repair technicians, this tool combines a high-speed SPI/I2C Flash Programmer with an advanced Serial Console specifically tuned for embedded device hacking.

**GitHub Repository:** [https://github.com/peterpt/Tiao_Commander.git](https://github.com/peterpt/Tiao_Commander.git)

## Pictures
* Chip Builder
<img width="1024" height="741" alt="image" src="https://github.com/user-attachments/assets/0b320d57-c308-4915-87eb-71d3b365a07c" />
* Serial Console
  <img width="1024" height="741" alt="image" src="https://github.com/user-attachments/assets/3353df58-26ba-4c53-adf7-b4c9ff3c8cfb" />
* Board Reference
 <img width="1024" height="741" alt="image" src="https://github.com/user-attachments/assets/ecebc255-bac1-497b-b8d7-cf685d82a18b" />
* Tiao Multi Protocol Adapter Interface
  <img width="800" height="800" alt="image" src="https://github.com/user-attachments/assets/c8824372-ee6e-4c12-947d-a74c923a26fc" />

---

## ⚡ Key Features

### 1. Universal Programmer
*   **Protocols:** Supports **SPI Flash** (BIOS chips, 25xx series), **DataFlash** (AT45 series), and **I2C EEPROM** (24xx series).
*   **Auto-Detection:** Automatically identifies chips via JEDEC ID.
*   **Operations:** Read, Write, Erase, and Verify with progress bars and hex preview.
*   **Dynamic Wiring Guide:** Automatically displays the correct wiring diagram (TUMPA Pin to Chip Pin) based on the selected protocol.
*   **Database:** Includes a vast database of chips (derived from the Flashrom project).

### 2. Advanced Serial Console (IoT Hacking Suite)
*   **Channel B Support:** Correctly routes UART traffic to Interface 2 (standard for TUMPA v2).
*   **Smart Auto-Baud:** Unlike standard tools, this detects baud rate by *waiting for actual data* and analyzing the text/garbage ratio to find the perfect speed.
*   **Boot Interrupter:** A specialized tool to stop bootloaders (U-Boot) from loading the OS. It "arms" the system and machine-guns a specific key (Enter, Space, Ctrl+C, Esc) the moment the device powers on.
*   **Smart Hex Mode:**
    *   Sends raw Hex bytes if you type hex (e.g., `AA BB`).
    *   Automatically converts text to hex if you type words (e.g., typing `help` sends `68 65 6C 70`).
*   **Logging:** Capture session data to file automatically.

### 3. Chip Builder
*   Encountered a chip not in the database? Use the visual Chip Builder to define pinouts, voltage, and size.
*   Generates a custom entry in `chips_database.json` for immediate use.

---

## 🛠️ Hardware Requirements

*   **TIAO USB Multi-Protocol Adapter (TUMPA) v2**
*   Target device (Router, Camera, BIOS Chip, etc.)
*   Standard Jumper wires or SOIC8 Test Clip.

---

## 📥 Installation

### 1. Prerequisites
You need Python 3 installed.

### 2. Install Dependencies
This tool relies on `PyQt5` for the GUI and `pyftdi` for hardware communication.

```bash
pip install pyqt5 pyftdi pyserial

  

3. Drivers & Permissions

Linux:
You need to grant permission to access the USB device without sudo. Create a udev rule:
code Bash

    
echo 'SUBSYSTEM=="usb", ATTR{idVendor}=="0403", ATTR{idProduct}=="8a98", GROUP="plugdev", MODE="0666"' | sudo tee /etc/udev/rules.d/99-tiao.rules
sudo udevadm control --reload-rules
sudo udevadm trigger

  

🚀 Usage

Run the main script:
code Bash

    
python3 tiao_comander.py

  

Tab 1: Programmer

    Auto Detect: Click "Automatic Detection". If the chip is wired correctly and supported, it will appear.

    Manual: If detection fails, select the Protocol (SPI/I2C) and Chip Vendor manually.

    Wiring Guide: Look at the table on the right. It tells you exactly where to connect wires.

        Note: For I2C Chips (24xx), you must connect the chip's Pin 7 (WP) to Ground (SPI1 Pin 6 on TIAO) to enable writing.

    Read/Write: Use the buttons to dump firmware or flash a .bin file.

Tab 2: Chip Builder

Use this if you have a rare chip. Enter the details from the datasheet, select the pinout, and click "Save". It will be added to the local database.
Tab 3: Serial Console

    Configuration: Select Baud Rate (default 115200).

    Auto-Detect: If you see garbage text, click "Auto-Detect Baud". Power cycle your target device. The app will listen and find the correct speed.

    Boot Interrupter:

        Check "Stop OS".

        Select the key (usually Ctrl+C or Enter).

        Click "Open Port".

        Power on your device. The app will spam the key immediately upon receiving the first spark of electricity, dropping you into the U-Boot shell.

⚠️ Common Issues

"LibUSB Error" or "Access Denied"

    Linux: Ensure you added the udev rules mentioned in Installation.

    Windows: Use Zadig to reinstall the driver.

"No Data" in Serial Console

    Ensure you are connected to the TTL-COM header on the TIAO board (bottom header).

    Ensure RX goes to TX, and TX goes to RX.

"I2C Write Failed"

    Check the wiring table in the app. Ensure the Write Protect (WP) pin on the chip is connected to GND.

🏆 Credits & Acknowledgments

    Created & Designed by: Peterpt

    Code Architecture & Logic: Google Gemini (AI)

    Database Source: Special thanks to the Flashrom Project. The chip database structure and JEDEC IDs used in this tool are derived from their extensive open-source documentation.

📄 License

This project is open-source. Feel free to fork, modify, and contribute.
