# OBD-II CAN Bus Console

A Python desktop application for communicating with a vehicle's OBD-II port using an Arduino-based CAN Bus Shield. Built with PyQt6 for the UI and pyserial for serial communication.

---

## Features

- Connect to an OBD-II device over USB serial
- Browse and query all standard OBD-II Service Modes (01–0A)
- PID list automatically filtered to only the PIDs your vehicle supports
- Decodes ~50 Mode 01 PIDs into human-readable values (RPM, temperatures, pressures, etc.)
- Reads and displays Diagnostic Trouble Codes (stored, pending, and permanent)
- Clear DTCs with a confirmation dialog
- Fetches vehicle identity data: VIN, model year, manufacturer, calibration ID, ECU name
- Device name auto-detected from Arduino firmware on connect

---

## Hardware Required

| Component | Details |
|-----------|---------|
| Arduino (Uno or Nano) | Any 5V Arduino compatible board |
| Seeed Studio CAN Bus Shield V2 | Uses MCP2515 CAN controller |
| OBD-II cable or adapter | Standard OBD-II to DB9 or bare wire |
| USB cable | To connect Arduino to PC |

The Arduino must be flashed with the firmware in `OBDII_PIDs_updated/OBDII_PIDs_updated.ino`.

> **Note:** The CAN Bus Shield communicates at **500 kbps**, which is standard for most OBD-II vehicles. The serial connection runs at **115200 baud** on **COM5** by default — change `COM_PORT` in `obd_query.py` if your Arduino appears on a different port.

---

## Software Requirements

- Python 3.10+
- PyQt6
- pyserial

Install dependencies:

```
pip install PyQt6 pyserial
```

---

## Project Structure

```
CanBusShieldV2/
├── obd_console.py          # Main application entry point (UI logic)
├── obd_console.ui          # Qt Designer UI layout file
├── obd_query.py            # Backend: serial comms, PID dictionaries, decoders
├── OBDII_PIDs_updated/
│   └── OBDII_PIDs_updated.ino  # Arduino firmware
└── resources/              # UI icons and SVG assets
```

---

## How to Run

1. Flash `OBDII_PIDs_updated.ino` to your Arduino using the Arduino IDE
2. Plug the CAN Bus Shield into your vehicle's OBD-II port
3. Connect the Arduino to your PC via USB
4. Run the app:

```
python obd_console.py
```

5. Click **Connect/ReConnect USB Device** to open the serial connection
6. Click **Get Vehicle Data** to fetch VIN and build the supported PID list
7. Select a service mode from the dropdown, choose a PID, and click **Send PID**

---

## Supported OBD-II Service Modes

| Mode | Description |
|------|-------------|
| 01 | Show current sensor data |
| 02 | Show freeze frame data |
| 03 | Show stored Diagnostic Trouble Codes |
| 04 | Clear DTCs and freeze frame data |
| 05 | O2 sensor monitoring (non-CAN only) |
| 07 | Show pending DTCs |
| 09 | Vehicle information (VIN, Cal ID, ECU name) |
| 0A | Show permanent DTCs |

---

## Decoding

Mode 01 sensor values are decoded using standard OBD-II formulas. Examples:

- **Engine RPM** — `(A*256+B) / 4`
- **Coolant Temp** — `A - 40 °C`
- **Vehicle Speed** — `A km/h`
- **Throttle Position** — `A * 100/255 %`

VIN decoding uses the World Manufacturer Identifier (WMI) — the first 3 characters — to identify the manufacturer, and position 10 to determine the model year.

---

## License

MIT License — free to use, modify, and distribute.
