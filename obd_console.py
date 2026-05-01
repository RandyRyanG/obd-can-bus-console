from PyQt6 import QtWidgets, uic
from PyQt6.QtCore import QStringListModel, Qt
import sys
import serial
import time
from obd_query import (send_obd, handle_mode01, MODE_NAMES, MODE01_PIDS, MODE05_MONITORS,
                       MODE09_PARAMS, COM_PORT, BAUD_RATE, decode_mode01, NO_PARAM_MODES,
                       decode_dtc_list, decode_vin, get_vin, get_mode09_ascii, try_ascii,
                       decode_mode09_supported, decode_mode01_supported)


class OBDConsole(QtWidgets.QMainWindow):
    def __init__(self):
        super().__init__()
        uic.loadUi("obd_console.ui", self)
        self.ser = None
        self.supported_mode09_params = None
        self.supported_mode01_pids = None
        self.sel_service_mode.currentTextChanged.connect(self.update_pid_list)
        self.connect_device_button.clicked.connect(self.connect_device)
        self.send_button.clicked.connect(self.send_pid)
        self.clear_results_button.clicked.connect(self.response_text.clear)
        self.get_vehicle_data.clicked.connect(self.fetch_vehicle_data)
        self.pid_list.clicked.connect(self.on_pid_selected)
        self.connected_checkbox.setAttribute(Qt.WidgetAttribute.WA_TransparentForMouseEvents)


    def update_pid_list(self, selected_text):
        if selected_text.startswith("01"):
            if self.supported_mode01_pids:
                items = [f"0x{k:02X}  {v}" for k, v in MODE01_PIDS.items()
                         if k in self.supported_mode01_pids]
            else:
                items = [f"0x{k:02X}  {v}" for k, v in MODE01_PIDS.items()]
        elif selected_text.startswith("09"):
            if self.supported_mode09_params:
                items = [f"0x{k:02X}  {v}" for k, v in MODE09_PARAMS.items()
                         if k in self.supported_mode09_params]
            else:
                items = [f"0x{k:02X}  {v}" for k, v in MODE09_PARAMS.items()]
        elif selected_text.startswith("05"):
            items = [f"0x{k:02X}  {v}" for k, v in MODE05_MONITORS.items()]
        else:
            items = []
        self.pid_list.setModel(QStringListModel(items))
    def on_pid_selected(self, index):
        # Extract just the hex code from "0x0C  Engine RPM" and put it in the text box
        hex_code = index.data().split()[0][2:]  # strips "0x" prefix
        self.user_pid_text.setText(hex_code)

    def connect_device(self):
        try:
            self.ser = serial.Serial(COM_PORT, BAUD_RATE, timeout=0.1)
            self.connected_checkbox.setChecked(True)
            self.response_text.append(f"Connected on {COM_PORT}")
            self.device_name.setText(f"Connected: {COM_PORT}")
            deadline = time.time() + 4.0
            while time.time() < deadline:
                line = self.ser.readline().decode('utf-8', errors='ignore').strip()
                if line.startswith("DEVICE:"):
                    self.device_name.setText(line[7:].strip())
                    break
        except Exception as e:
            self.connected_checkbox.setChecked(False)
            self.response_text.append(f"Connection failed: {e}")




    

    







    def fetch_vehicle_data(self):
        if self.ser is None:
            self.vehicle_data_display.append("Not connected")
            return

        self.vehicle_data_display.clear()

        raw_support, _ = send_obd(self.ser, 0x09, 0x00)
        self.supported_mode09_params = decode_mode09_supported(raw_support)

        supported_pids = []
        for base in [0x00, 0x20, 0x40, 0x60]:
            raw_pid, _ = send_obd(self.ser, 0x01, base)
            supported_pids.extend(decode_mode01_supported(raw_pid, base))
        self.supported_mode01_pids = set(supported_pids)

        vin = get_vin(self.ser)
        if vin:
            info = decode_vin(vin)
            self.vehicle_data_display.append(f"VIN:          {info['vin']}")
            self.vehicle_data_display.append(f"Year:         {info['year']}")
            self.vehicle_data_display.append(f"Manufacturer: {info['manufacturer']}")
            self.device_name.setText(f"{info['year']} {info['manufacturer']}  |  {COM_PORT}")
        else:
            self.vehicle_data_display.append("VIN:          No response")

        cal_id = get_mode09_ascii(self.ser, 0x04)
        self.vehicle_data_display.append(f"Cal ID:       {cal_id or 'No response'}")

        ecu_name = get_mode09_ascii(self.ser, 0x0A)
        self.vehicle_data_display.append(f"ECU Name:     {ecu_name or 'No response'}")

    def send_pid(self):
        if self.ser is None:
            self.response_text.append("Not connected")
            return

        # Get the mode number from the first two characters of the dropdown text
        mode_text = self.sel_service_mode.currentText()
        try:
            mode = int(mode_text[:2], 16)
        except ValueError:
            self.response_text.append("No mode selected")
            return

        # DTC modes don't use a PID — handle them directly
        if mode in NO_PARAM_MODES:
            if mode == 0x04:
                confirm = QtWidgets.QMessageBox.question(
                    self, "Clear DTCs",
                    "This will clear all DTCs and freeze frame data. Continue?",
                    QtWidgets.QMessageBox.StandardButton.Yes | QtWidgets.QMessageBox.StandardButton.No
                )
                if confirm != QtWidgets.QMessageBox.StandardButton.Yes:
                    return
            label = {0x03: "stored", 0x07: "pending", 0x0A: "permanent", 0x04: ""}.get(mode, "")
            self.response_text.append(f"Reading {label} DTCs..." if mode != 0x04 else "Clearing DTCs...")
            raw, _ = send_obd(self.ser, mode, 0x00, timeout=4.0)
            if mode == 0x04:
                self.response_text.append("  DTCs cleared" if raw and 0x44 in raw else "  No confirmation received")
                return
            resp_code = mode + 0x40
            if not raw:
                self.response_text.append("  No response")
                return
            data = raw[1:] if len(raw) >= 2 and raw[1] == resp_code else raw
            dtcs = decode_dtc_list(data)
            if dtcs:
                self.response_text.append(f"  {len(dtcs)} DTC(s) found:")
                for dtc in dtcs:
                    self.response_text.append(f"    {dtc}")
            else:
                self.response_text.append("  No DTCs stored")
            return

        # Get the PID — prefer manual text input, fall back to pid_list selection
        typed = self.user_pid_text.text().strip()
        if typed:
            try:
                pid = int(typed, 16)
            except ValueError:
                self.response_text.append(f"Invalid PID: {typed}")
                return
        else:
            index = self.pid_list.currentIndex()
            if not index.isValid():
                self.response_text.append("No PID selected")
                return
            # pid_list items look like "0x0C  Engine RPM" — grab the first word
            pid = int(index.data().split()[0], 16)

        # Send the command and display the result
        self.response_text.append(f"Sending mode {mode:02X} PID {pid:02X}...")
        raw, vin = send_obd(self.ser, mode, pid)
        if vin:
            self.response_text.append(f"  VIN: {vin}")
        elif raw:
            if mode == 0x01 and len(raw) >= 7 and raw[1] == 0x41:
                data = raw[3:7]
                decoded = decode_mode01(pid, data)
                if decoded:
                    self.response_text.append(f"  {decoded}")
                else:
                    self.response_text.append(f"  Raw bytes: {[hex(b) for b in raw]}")
            elif mode == 0x09:
                # Extract payload — skip resp_code, param, count bytes
                if raw[0] == 0x49:
                    payload = raw[3:]
                elif len(raw) >= 2 and raw[1] == 0x49:
                    payload = raw[4:]
                else:
                    payload = raw
                if pid == 0x00:
                    self.response_text.append(f"  Supported params bitmask: 0x{payload[0]:02X}")
                elif pid == 0x06:
                    self.response_text.append(f"  CVN: {''.join(f'{b:02X}' for b in payload if b)}")
                else:
                    self.response_text.append(f"  {try_ascii(payload).strip('.')}")
            else:
                self.response_text.append(f"  Raw bytes: {[hex(b) for b in raw]}")
        else:
            self.response_text.append("  No response")

def closeEvent(self, event):
    if self.ser and self.ser.is_open:
        self.ser.close()
    event.accept()


if __name__ == '__main__':
    app = QtWidgets.QApplication(sys.argv)
    window = OBDConsole()
    window.show()
    sys.exit(app.exec())
