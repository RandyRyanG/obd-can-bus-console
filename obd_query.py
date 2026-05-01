import serial
import time
import re

COM_PORT  = 'COM5'
BAUD_RATE = 115200

# ---------------------------------------------------------------------------
# Mode descriptions
# ---------------------------------------------------------------------------
MODE_NAMES = {
    0x01: "Current Data",
    0x02: "Freeze Frame Data",
    0x03: "Stored DTCs",
    0x04: "Clear DTCs",
    0x07: "Pending DTCs",
    0x09: "Vehicle Information",
    0x0A: "Permanent DTCs",
}

# Modes that take no PID parameter
NO_PARAM_MODES = {0x03, 0x04, 0x07, 0x0A}

# ---------------------------------------------------------------------------
# Mode 01 / Mode 02 PID descriptions (with formula hints)
# ---------------------------------------------------------------------------
MODE01_PIDS = {
    0x01: "Monitor status / DTC count",
    0x02: "Freeze frame DTC",
    0x03: "Fuel system status",
    0x04: "Engine load %           (A*100/255)",
    0x05: "Coolant temp            (A-40 °C)",
    0x06: "ST fuel trim Bank 1     (A/1.28-100 %)",
    0x07: "LT fuel trim Bank 1     (A/1.28-100 %)",
    0x08: "ST fuel trim Bank 2     (A/1.28-100 %)",
    0x09: "LT fuel trim Bank 2     (A/1.28-100 %)",
    0x0A: "Fuel pressure           (A*3 kPa)",
    0x0B: "Intake manifold press   (A kPa)",
    0x0C: "Engine RPM              ((A*256+B)/4)",
    0x0D: "Vehicle speed           (A km/h)",
    0x0E: "Timing advance          (A/2-64 °)",
    0x0F: "Intake air temp         (A-40 °C)",
    0x10: "MAF rate                ((A*256+B)/100 g/s)",
    0x11: "Throttle position       (A*100/255 %)",
    0x12: "Secondary air status",
    0x13: "O2 sensors present (2 banks)",
    0x14: "O2 sensor B1S1          (A/200 V, B/1.28-100 %)",
    0x15: "O2 sensor B1S2",
    0x16: "O2 sensor B1S3",
    0x17: "O2 sensor B1S4",
    0x18: "O2 sensor B2S1",
    0x19: "O2 sensor B2S2",
    0x1A: "O2 sensor B2S3",
    0x1B: "O2 sensor B2S4",
    0x1C: "OBD standard",
    0x1D: "O2 sensors present (4 banks)",
    0x1E: "Auxiliary input status",
    0x1F: "Run time since start    (A*256+B s)",
    0x21: "Distance with MIL on    (A*256+B km)",
    0x22: "Fuel rail pressure      ((A*256+B)*0.079 kPa)",
    0x23: "Fuel rail gauge press   ((A*256+B)*10 kPa)",
    0x24: "O2 sensor 1 wide range",
    0x25: "O2 sensor 2 wide range",
    0x26: "O2 sensor 3 wide range",
    0x27: "O2 sensor 4 wide range",
    0x28: "O2 sensor 5 wide range",
    0x29: "O2 sensor 6 wide range",
    0x2A: "O2 sensor 7 wide range",
    0x2B: "O2 sensor 8 wide range",
    0x2C: "Commanded EGR           (A*100/255 %)",
    0x2D: "EGR error               (A/1.28-100 %)",
    0x2E: "Commanded evap purge    (A*100/255 %)",
    0x2F: "Fuel tank level         (A*100/255 %)",
    0x30: "Warm-ups since codes cleared",
    0x31: "Distance since codes cleared (A*256+B km)",
    0x32: "Evap system vapor press (A*256+B Pa)",
    0x33: "Barometric pressure     (A kPa)",
    0x34: "O2 sensor 1 wide range equivalence",
    0x35: "O2 sensor 2 wide range equivalence",
    0x36: "O2 sensor 3 wide range equivalence",
    0x37: "O2 sensor 4 wide range equivalence",
    0x38: "O2 sensor 5 wide range equivalence",
    0x39: "O2 sensor 6 wide range equivalence",
    0x3A: "O2 sensor 7 wide range equivalence",
    0x3B: "O2 sensor 8 wide range equivalence",
    0x3C: "Catalyst temp B1S1      ((A*256+B)/10-40 °C)",
    0x3D: "Catalyst temp B2S1",
    0x3E: "Catalyst temp B1S2",
    0x3F: "Catalyst temp B2S2",
    0x41: "Monitor status this drive cycle",
    0x42: "Control module voltage  ((A*256+B)/1000 V)",
    0x43: "Absolute load value     ((A*256+B)*100/255 %)",
    0x44: "Commanded air-fuel ratio",
    0x45: "Relative throttle pos   (A*100/255 %)",
    0x46: "Ambient air temp        (A-40 °C)",
    0x47: "Throttle position B     (A*100/255 %)",
    0x48: "Throttle position C     (A*100/255 %)",
    0x49: "Accel pedal position D  (A*100/255 %)",
    0x4A: "Accel pedal position E  (A*100/255 %)",
    0x4B: "Accel pedal position F  (A*100/255 %)",
    0x4C: "Commanded throttle      (A*100/255 %)",
    0x4D: "Time with MIL on        (A*256+B min)",
    0x4E: "Time since codes cleared (A*256+B min)",
    0x4F: "Max values (various)",
    0x50: "Max MAF rate            (A*10 g/s)",
    0x51: "Fuel type",
    0x52: "Ethanol fuel %          (A*100/255)",
    0x53: "Abs evap vapor press    ((A*256+B)/200 kPa)",
    0x54: "Evap vapor pressure",
    0x55: "ST secondary O2 trim B1/B3",
    0x56: "LT secondary O2 trim B1/B3",
    0x57: "ST secondary O2 trim B2/B4",
    0x58: "LT secondary O2 trim B2/B4",
    0x59: "Fuel rail abs pressure  ((A*256+B)*10 kPa)",
    0x5A: "Rel accel pedal pos     (A*100/255 %)",
    0x5B: "Hybrid battery life     (A*100/255 %)",
    0x5C: "Engine oil temp         (A-40 °C)",
    0x5D: "Fuel injection timing   ((A*256+B)/128-210 °)",
    0x5E: "Engine fuel rate        ((A*256+B)*0.05 L/h)",
    0x5F: "Emission requirements",
    0x61: "Driver demanded torque  (A-125 %)",
    0x62: "Actual engine torque    (A-125 %)",
    0x63: "Engine reference torque (A*256+B Nm)",
    0x64: "Engine torque data",
    0x65: "Auxiliary I/O supported",
    0x66: "MAF sensor",
    0x67: "Engine coolant temp",
    0x68: "Intake air temp sensor",
    0x69: "Commanded EGR / error",
    0x6A: "Commanded diesel intake air",
    0x6B: "EGR temp",
    0x6C: "Commanded throttle control",
    0x6D: "Fuel pressure control",
    0x6E: "Injection pressure control",
    0x6F: "Turbo compressor inlet press",
    0x70: "Boost pressure control",
    0x71: "Variable geometry turbo",
    0x72: "Wastegate control",
    0x73: "Exhaust pressure",
    0x74: "Turbocharger RPM",
    0x75: "Turbocharger temp A",
    0x76: "Turbocharger temp B",
    0x77: "Charge air cooler temp",
    0x78: "EGT bank 1",
    0x79: "EGT bank 2",
    0x7A: "Diesel particulate filter",
    0x7B: "DPF bank 2",
    0x7C: "DPF temp",
    0x7D: "NOx NTE control status",
    0x7E: "PM NTE control status",
    0x7F: "Engine run time AECD",
}

# ---------------------------------------------------------------------------
# Mode 09 parameter descriptions
# ---------------------------------------------------------------------------
MODE09_PARAMS = {
    0x02: "VIN (ASCII)",
    0x04: "Calibration ID (ASCII)",
    0x06: "CVN",
    0x08: "In-use performance tracking",
    0x0A: "ECU name (ASCII)",
}

def decode_mode01_supported(raw, base_pid):
    """Decode a Mode 01 support bitmask into a list of supported PID numbers.
    base_pid is the PID that was queried (0x00, 0x20, 0x40, or 0x60).
    Each call covers 32 PIDs starting at base_pid + 1.
    """
    if not raw:
        return []
    if len(raw) >= 7 and raw[1] == 0x41:
        data = raw[3:7]
    elif len(raw) >= 6 and raw[0] == 0x41:
        data = raw[2:6]
    else:
        return []
    bitmask = (data[0] << 24) | (data[1] << 16) | (data[2] << 8) | data[3]
    return [base_pid + i + 1 for i in range(32) if (bitmask >> (31 - i)) & 1]

def decode_mode09_supported(raw):
    """Decode Mode 09 PID 00 bitmask into a list of supported param numbers (1-8)."""
    if not raw:
        return []
    if raw[0] == 0x49:
        bitmask = raw[3] if len(raw) > 3 else 0
    elif len(raw) >= 2 and raw[1] == 0x49:
        bitmask = raw[4] if len(raw) > 4 else 0
    else:
        return []
    return [i + 1 for i in range(8) if (bitmask >> (7 - i)) & 1]

# ---------------------------------------------------------------------------
# Mode 05 monitor IDs (O2 sensor monitoring — non-CAN only)
# ---------------------------------------------------------------------------
MODE05_MONITORS = {
    0x0100: "OBD Monitor IDs supported",
    0x0101: "O2 Sensor Monitor Bank 1 Sensor 1",
    0x0102: "O2 Sensor Monitor Bank 1 Sensor 2",
    0x0103: "O2 Sensor Monitor Bank 1 Sensor 3",
    0x0104: "O2 Sensor Monitor Bank 1 Sensor 4",
    0x0105: "O2 Sensor Monitor Bank 2 Sensor 1",
    0x0106: "O2 Sensor Monitor Bank 2 Sensor 2",
    0x0107: "O2 Sensor Monitor Bank 2 Sensor 3",
    0x0108: "O2 Sensor Monitor Bank 2 Sensor 4",
    0x0109: "O2 Sensor Monitor Bank 3 Sensor 1",
    0x010A: "O2 Sensor Monitor Bank 3 Sensor 2",
    0x010B: "O2 Sensor Monitor Bank 3 Sensor 3",
    0x010C: "O2 Sensor Monitor Bank 3 Sensor 4",
    0x010D: "O2 Sensor Monitor Bank 4 Sensor 1",
    0x010E: "O2 Sensor Monitor Bank 4 Sensor 2",
    0x010F: "O2 Sensor Monitor Bank 4 Sensor 3",
    0x0110: "O2 Sensor Monitor Bank 4 Sensor 4",
    0x0201: "O2 Sensor Monitor Bank 1 Sensor 1 (Lean to Rich threshold)",
    0x0202: "O2 Sensor Monitor Bank 1 Sensor 2 (Lean to Rich threshold)",
    0x0203: "O2 Sensor Monitor Bank 1 Sensor 3 (Lean to Rich threshold)",
    0x0204: "O2 Sensor Monitor Bank 1 Sensor 4 (Lean to Rich threshold)",
    0x0205: "O2 Sensor Monitor Bank 2 Sensor 1 (Lean to Rich threshold)",
    0x0206: "O2 Sensor Monitor Bank 2 Sensor 2 (Lean to Rich threshold)",
    0x0207: "O2 Sensor Monitor Bank 2 Sensor 3 (Lean to Rich threshold)",
    0x0208: "O2 Sensor Monitor Bank 2 Sensor 4 (Lean to Rich threshold)",
    0x0209: "O2 Sensor Monitor Bank 3 Sensor 1 (Lean to Rich threshold)",
    0x020A: "O2 Sensor Monitor Bank 3 Sensor 2 (Lean to Rich threshold)",
    0x020B: "O2 Sensor Monitor Bank 3 Sensor 3 (Lean to Rich threshold)",
    0x020C: "O2 Sensor Monitor Bank 3 Sensor 4 (Lean to Rich threshold)",
    0x020D: "O2 Sensor Monitor Bank 4 Sensor 1 (Lean to Rich threshold)",
    0x020E: "O2 Sensor Monitor Bank 4 Sensor 2 (Lean to Rich threshold)",
    0x020F: "O2 Sensor Monitor Bank 4 Sensor 3 (Lean to Rich threshold)",
    0x0210: "O2 Sensor Monitor Bank 4 Sensor 4 (Lean to Rich threshold)",
}

# ---------------------------------------------------------------------------
# Mode 01 decoder — converts raw A/B/C/D bytes into a human-readable value
# ---------------------------------------------------------------------------
def decode_mode01(pid, data):
    """Return a decoded string for a Mode 01 PID, or None if unknown."""
    if len(data) < 4:
        return None
    A, B, C, D = data[0], data[1], data[2], data[3]

    fuel_status = {
        0x01: "Open loop (cold)", 0x02: "Closed loop",
        0x04: "Open loop (load)", 0x08: "Open loop (fault)",
        0x10: "Closed loop (O2 fault)",
    }
    obd_standard = {
        1: "OBD-II (CARB)", 2: "OBD (EPA)", 3: "OBD and OBD-II",
        4: "OBD-I", 5: "Not OBD compliant", 6: "EOBD",
        7: "EOBD and OBD-II", 8: "EOBD and OBD", 9: "JOBD",
        10: "JOBD and OBD-II", 11: "JOBD and EOBD", 12: "JOBD, EOBD and OBD-II",
    }

    decoders = {
        0x01: lambda: f"MIL: {'ON' if A & 0x80 else 'OFF'}  DTCs: {A & 0x7F}  (Monitor status)",
        0x03: lambda: f"Bank1: {fuel_status.get(A, f'0x{A:02X}')}  Bank2: {fuel_status.get(B, 'N/A')}  (Fuel system status)",
        0x04: lambda: f"{A * 100 / 255:.1f} %  (Engine load)",
        0x05: lambda: f"{A - 40} °C  (Coolant temp)",
        0x06: lambda: f"{A / 1.28 - 100:.2f} %  (ST fuel trim Bank 1)",
        0x07: lambda: f"{A / 1.28 - 100:.2f} %  (LT fuel trim Bank 1)",
        0x08: lambda: f"{A / 1.28 - 100:.2f} %  (ST fuel trim Bank 2)",
        0x09: lambda: f"{A / 1.28 - 100:.2f} %  (LT fuel trim Bank 2)",
        0x0A: lambda: f"{A * 3} kPa  (Fuel pressure)",
        0x0B: lambda: f"{A} kPa  (Intake manifold pressure)",
        0x0C: lambda: f"{(A * 256 + B) / 4:.1f} RPM  (Engine RPM)",
        0x0D: lambda: f"{A} km/h  (Vehicle speed)",
        0x0E: lambda: f"{A / 2 - 64:.1f} °  (Timing advance)",
        0x0F: lambda: f"{A - 40} °C  (Intake air temp)",
        0x10: lambda: f"{(A * 256 + B) / 100:.2f} g/s  (MAF rate)",
        0x11: lambda: f"{A * 100 / 255:.1f} %  (Throttle position)",
        0x13: lambda: f"B1: S1:{A&1} S2:{A>>1&1} S3:{A>>2&1} S4:{A>>3&1}  B2: S1:{A>>4&1} S2:{A>>5&1} S3:{A>>6&1} S4:{A>>7&1}  (O2 sensors present)",
        0x14: lambda: f"{A / 200:.3f} V  ST trim: {B / 1.28 - 100:.1f} %  (O2 B1S1)" if B != 0xFF else f"{A / 200:.3f} V  (O2 B1S1)",
        0x15: lambda: f"{A / 200:.3f} V  ST trim: {B / 1.28 - 100:.1f} %  (O2 B1S2)" if B != 0xFF else f"{A / 200:.3f} V  (O2 B1S2)",
        0x18: lambda: f"{A / 200:.3f} V  ST trim: {B / 1.28 - 100:.1f} %  (O2 B2S1)" if B != 0xFF else f"{A / 200:.3f} V  (O2 B2S1)",
        0x19: lambda: f"{A / 200:.3f} V  ST trim: {B / 1.28 - 100:.1f} %  (O2 B2S2)" if B != 0xFF else f"{A / 200:.3f} V  (O2 B2S2)",
        0x1C: lambda: f"{obd_standard.get(A, f'Standard 0x{A:02X}')}  (OBD standard)",
        0x1F: lambda: f"{A * 256 + B} s  (Run time since start)",
        0x21: lambda: f"{A * 256 + B} km  (Distance with MIL on)",
        0x22: lambda: f"{(A * 256 + B) * 0.079:.1f} kPa  (Fuel rail pressure)",
        0x2C: lambda: f"{A * 100 / 255:.1f} %  (Commanded EGR)",
        0x2D: lambda: f"{A / 1.28 - 100:.2f} %  (EGR error)",
        0x2E: lambda: f"{A * 100 / 255:.1f} %  (Commanded evap purge)",
        0x2F: lambda: f"{A * 100 / 255:.1f} %  (Fuel tank level)",
        0x30: lambda: f"{A}  (Warm-ups since codes cleared)",
        0x31: lambda: f"{A * 256 + B} km  (Distance since codes cleared)",
        0x32: lambda: f"{A * 256 + B if A * 256 + B < 32768 else A * 256 + B - 65536} Pa  (Evap vapor pressure)",
        0x33: lambda: f"{A} kPa  (Barometric pressure)",
        0x3C: lambda: f"{(A * 256 + B) / 10 - 40:.1f} °C  (Catalyst temp B1S1)",
        0x3D: lambda: f"{(A * 256 + B) / 10 - 40:.1f} °C  (Catalyst temp B2S1)",
        0x3E: lambda: f"{(A * 256 + B) / 10 - 40:.1f} °C  (Catalyst temp B1S2)",
        0x3F: lambda: f"{(A * 256 + B) / 10 - 40:.1f} °C  (Catalyst temp B2S2)",
        0x41: lambda: f"Monitor status: 0x{A:02X}{B:02X}{C:02X}{D:02X}  (Drive cycle monitors)",
        0x42: lambda: f"{(A * 256 + B) / 1000:.3f} V  (Control module voltage)",
        0x43: lambda: f"{(A * 256 + B) * 100 / 255:.1f} %  (Absolute load)",
        0x44: lambda: f"{(A * 256 + B) / 32768:.4f} lambda  (Commanded AFR)",
        0x45: lambda: f"{A * 100 / 255:.1f} %  (Relative throttle pos)",
        0x46: lambda: f"{A - 40} °C  (Ambient air temp)",
        0x47: lambda: f"{A * 100 / 255:.1f} %  (Throttle position B)",
        0x49: lambda: f"{A * 100 / 255:.1f} %  (Accel pedal position D)",
        0x4A: lambda: f"{A * 100 / 255:.1f} %  (Accel pedal position E)",
        0x4B: lambda: f"{A * 100 / 255:.1f} %  (Accel pedal position F)",
        0x4C: lambda: f"{A * 100 / 255:.1f} %  (Commanded throttle)",
        0x4D: lambda: f"{A * 256 + B} min  (Time with MIL on)",
        0x4E: lambda: f"{A * 256 + B} min  (Time since codes cleared)",
        0x52: lambda: f"{A * 100 / 255:.1f} %  (Ethanol fuel)",
        0x5A: lambda: f"{A * 100 / 255:.1f} %  (Rel accel pedal pos)",
        0x5B: lambda: f"{A * 100 / 255:.1f} %  (Hybrid battery life)",
        0x5C: lambda: f"{A - 40} °C  (Engine oil temp)",
        0x5E: lambda: f"{(A * 256 + B) * 0.05:.2f} L/h  (Engine fuel rate)",
        0x61: lambda: f"{A - 125} %  (Driver demanded torque)",
        0x62: lambda: f"{A - 125} %  (Actual engine torque)",
        0x63: lambda: f"{A * 256 + B} Nm  (Engine reference torque)",
    }
    decoder = decoders.get(pid)
    return decoder() if decoder else None

# ---------------------------------------------------------------------------
# VIN decoder
# ---------------------------------------------------------------------------
_WMI_MAP = {
    '1G1': 'Chevrolet', '1G2': 'Pontiac', '1G4': 'Buick', '1G6': 'Cadillac',
    '1GC': 'Chevrolet Truck', '1GT': 'GMC Truck', '1GM': 'Pontiac',
    '1FA': 'Ford', '1FB': 'Ford', '1FC': 'Ford', '1FD': 'Ford Truck', '1FT': 'Ford Truck',
    '1HG': 'Honda', '1J4': 'Jeep', '1J8': 'Jeep',
    '1N4': 'Nissan', '1N6': 'Nissan Truck',
    '1C3': 'Chrysler', '1C4': 'Chrysler/Jeep', '1C6': 'Ram Truck',
    '1D3': 'Dodge', '1D7': 'Dodge Truck', '1B3': 'Dodge', '1B4': 'Dodge',
    '1VW': 'Volkswagen', '1YV': 'Mazda',
    '2HG': 'Honda', '2HK': 'Honda',
    '2G1': 'Chevrolet', '2G2': 'Pontiac',
    '2T1': 'Toyota', '2T2': 'Lexus',
    '3VW': 'Volkswagen', '3N1': 'Nissan',
    '4T1': 'Toyota', '4T3': 'Toyota', '4T4': 'Toyota',
    '4S3': 'Subaru', '4S4': 'Subaru',
    '5NP': 'Hyundai', '5NM': 'Hyundai',
    '5TD': 'Toyota', '5TE': 'Toyota Truck', '5YJ': 'Tesla',
    'JHM': 'Honda', 'JH4': 'Acura',
    'JN1': 'Nissan', 'JN6': 'Nissan',
    'JT2': 'Toyota', 'JT3': 'Toyota', 'JT4': 'Toyota', 'JT6': 'Lexus',
    'JF1': 'Subaru', 'JF2': 'Subaru',
    'JM1': 'Mazda', 'JM3': 'Mazda',
    'JA3': 'Mitsubishi', 'JA4': 'Mitsubishi',
    'KM8': 'Hyundai', 'KMH': 'Hyundai', 'KNA': 'Kia', 'KND': 'Kia',
    'WBA': 'BMW', 'WBS': 'BMW M',
    'WDB': 'Mercedes-Benz', 'WDC': 'Mercedes-Benz',
    'WVW': 'Volkswagen', 'WAU': 'Audi', 'WA1': 'Audi',
    'WP0': 'Porsche', 'WP1': 'Porsche',
    'SAJ': 'Jaguar', 'SAL': 'Land Rover',
    'YV1': 'Volvo', 'YV4': 'Volvo',
}

_VIN_YEAR_MAP = {
    'A': 1980, 'B': 1981, 'C': 1982, 'D': 1983, 'E': 1984,
    'F': 1985, 'G': 1986, 'H': 1987, 'J': 1988, 'K': 1989,
    'L': 1990, 'M': 1991, 'N': 1992, 'P': 1993, 'R': 1994,
    'S': 1995, 'T': 1996, 'V': 1997, 'W': 1998, 'X': 1999,
    'Y': 2000, '1': 2001, '2': 2002, '3': 2003, '4': 2004,
    '5': 2005, '6': 2006, '7': 2007, '8': 2008, '9': 2009,
}

def decode_vin(vin):
    """Decode manufacturer and model year from a VIN string."""
    if not vin or len(vin) < 11:
        return {}
    vin = vin.upper().strip()
    year = _VIN_YEAR_MAP.get(vin[9])
    if year and year < 1996:  # OBD-II vehicles can't be pre-1996; shift to 2010+ cycle
        year += 30
    return {
        'vin':          vin,
        'year':         year or 'Unknown',
        'manufacturer': _WMI_MAP.get(vin[:3], f'Unknown ({vin[:3]})'),
    }

def get_vin(ser):
    """Request VIN using the reliable 'v' shortcut command."""
    ser.reset_input_buffer()
    ser.write(b"v\n")
    deadline = time.time() + 4.0
    while time.time() < deadline:
        line = ser.readline().decode('utf-8', errors='ignore').strip()
        if line.startswith('VIN:'):
            return line[4:].strip()
    return None

def get_mode09_ascii(ser, param):
    """Fetch a Mode 09 ASCII string response (calibration ID, ECU name, etc.)."""
    raw, _ = send_obd(ser, 0x09, param, timeout=4.0)
    if raw:
        resp_code = 0x49
        if raw[0] == resp_code:
            payload = raw[3:]
        elif len(raw) >= 2 and raw[1] == resp_code:
            payload = raw[4:]
        else:
            payload = raw
        result = try_ascii(payload).strip('.')
        return result if result.strip() else None
    return None

# ---------------------------------------------------------------------------
# Helpers
# ---------------------------------------------------------------------------
def parse_hex_line(line):
    """Parse '0x6\\t0x41\\t0xC\\t...' into list of ints."""
    return [int(x, 16) for x in re.findall(r'0x[0-9a-fA-F]+', line)]

def decode_dtc(high, low):
    """Decode two bytes into a DTC string e.g. P0301."""
    prefix = ['P', 'C', 'B', 'U'][(high >> 6) & 0x03]
    d1 = (high >> 4) & 0x03
    d2 =  high       & 0x0F
    d3 = (low  >> 4) & 0x0F
    d4 =  low        & 0x0F
    return f"{prefix}{d1}{d2:X}{d3:X}{d4:X}"

def decode_dtc_list(data):
    """
    Decode a DTC response payload into a list of DTC strings.
    data[0] = response code (0x43 / 0x47 / 0x4A)
    data[1] = number of DTCs
    data[2:] = DTC pairs
    """
    if len(data) < 2:
        return []
    num = data[1]
    dtcs = []
    for i in range(num):
        offset = 2 + i * 2
        if offset + 1 < len(data):
            h, l = data[offset], data[offset + 1]
            if h != 0 or l != 0:
                dtcs.append(decode_dtc(h, l))
    return dtcs

def try_ascii(data):
    """Try to decode bytes as ASCII, replacing non-printable with '.'"""
    return ''.join(chr(b) if 32 <= b < 127 else '.' for b in data)

# ---------------------------------------------------------------------------
# Serial communication
# ---------------------------------------------------------------------------
def send_obd(ser, mode, param=0x00, timeout=3.0):
    """
    Send MMPP command to Arduino and collect response.
    Returns (raw_bytes, vin_string) — one or both may be None.
    raw_bytes: list of ints from the assembled response line
    vin_string: decoded VIN string if present
    """
    ser.reset_input_buffer()
    ser.write(f"{mode:02x}{param:02x}\n".encode())

    response_code = mode + 0x40
    raw_bytes  = None
    vin_string = None

    deadline = time.time() + timeout
    while time.time() < deadline:
        line = ser.readline().decode('utf-8', errors='ignore').strip()
        if not line:
            continue

        if line.startswith('VIN:'):
            vin_string = line[4:].strip()
            break

        if line.startswith('0x') and '\t' in line:
            parsed = parse_hex_line(line)
            if not parsed:
                continue
            # Single frame:  [len, resp_code, param, A, B, C, D, pad]
            # Multi-frame:   [resp_code, param, ...]
            if len(parsed) >= 2 and parsed[1] == response_code:
                raw_bytes = parsed
                break
            elif parsed[0] == response_code:
                raw_bytes = parsed
                break

    return raw_bytes, vin_string

def prompt_hex(prompt_text):
    raw = input(prompt_text).strip().lower()
    if raw in ('q', 'quit', 'exit'):
        return 'q'
    if not raw:
        return None
    try:
        return int(raw, 16)
    except ValueError:
        print("  Invalid hex value")
        return None

# ---------------------------------------------------------------------------
# Mode handlers
# ---------------------------------------------------------------------------
def handle_mode01(ser, pid):
    name = MODE01_PIDS.get(pid, f"PID 0x{pid:02X}")
    print(f"  Requesting {name}...")
    raw, _ = send_obd(ser, 0x01, pid)
    if raw and len(raw) >= 7 and raw[1] == 0x41:
        data = raw[3:7]
        print(f"  Data bytes (A B C D): {[hex(b) for b in data]}")
        print(f"  Formula hint: see description above")
    elif raw:
        print(f"  Raw: {[hex(b) for b in raw]}")
    else:
        print("  No response (PID may not be supported)")

def handle_mode02(ser, pid):
    name = MODE01_PIDS.get(pid, f"PID 0x{pid:02X}")
    print(f"  Requesting freeze frame: {name}...")
    raw, _ = send_obd(ser, 0x02, pid)
    if raw and len(raw) >= 7 and raw[1] == 0x42:
        data = raw[3:7]
        print(f"  Data bytes (A B C D): {[hex(b) for b in data]}")
    elif raw:
        print(f"  Raw: {[hex(b) for b in raw]}")
    else:
        print("  No response (no freeze frame stored, or PID not supported)")

def handle_dtc_mode(ser, mode):
    label = {0x03: "stored", 0x07: "pending", 0x0A: "permanent"}.get(mode, "")
    print(f"  Requesting {label} DTCs...")
    raw, _ = send_obd(ser, mode, 0x00, timeout=4.0)
    resp_code = mode + 0x40

    if not raw:
        print("  No response")
        return

    # Locate response code in raw data
    if len(raw) >= 2 and raw[1] == resp_code:
        data = raw[1:]   # single frame: skip length byte
    elif raw[0] == resp_code:
        data = raw       # multi-frame assembled
    else:
        print(f"  Unexpected response: {[hex(b) for b in raw]}")
        return

    dtcs = decode_dtc_list(data)
    if dtcs:
        print(f"  {len(dtcs)} DTC(s) found:")
        for dtc in dtcs:
            print(f"    {dtc}")
    else:
        print("  No DTCs stored")

def handle_mode04(ser):
    confirm = input("  WARNING: This will clear all DTCs. Type YES to confirm: ").strip()
    if confirm != "YES":
        print("  Cancelled")
        return
    print("  Sending clear DTCs command...")
    raw, _ = send_obd(ser, 0x04, 0x00)
    if raw and 0x44 in raw:
        print("  DTCs cleared successfully")
    else:
        print("  No confirmation received")

def handle_mode09(ser, param):
    name = MODE09_PARAMS.get(param, f"InfoType 0x{param:02X}")
    print(f"  Requesting {name}...")

    if param == 0x02:
        # VIN — use 'v' shortcut for reliable response
        ser.reset_input_buffer()
        ser.write(b"v\n")
        deadline = time.time() + 4.0
        while time.time() < deadline:
            line = ser.readline().decode('utf-8', errors='ignore').strip()
            if line.startswith('VIN:'):
                print(f"  VIN: {line[4:].strip()}")
                return
        print("  No VIN response received")
    else:
        raw, vin = send_obd(ser, 0x09, param, timeout=4.0)
        if vin:
            print(f"  {vin}")
        elif raw:
            # Try ASCII decode for string responses (calibration ID, ECU name)
            resp_code = 0x49
            if raw[0] == resp_code:
                payload = raw[3:]   # skip resp_code, InfoType, count
            elif len(raw) >= 2 and raw[1] == resp_code:
                payload = raw[4:]
            else:
                payload = raw
            print(f"  Raw:   {[hex(b) for b in raw]}")
            print(f"  ASCII: {try_ascii(payload)}")
        else:
            print("  No response")

# ---------------------------------------------------------------------------
# Main loop
# ---------------------------------------------------------------------------
def main():
    print(f"Connecting on {COM_PORT} at {BAUD_RATE} baud...")
    ser = serial.Serial(COM_PORT, BAUD_RATE, timeout=0.1)
    time.sleep(2)
    ser.reset_input_buffer()
    print("Connected.\n")

    print("Available modes:")
    for code, name in MODE_NAMES.items():
        print(f"  {code:02X}  {name}")
    print("  q   Quit\n")

    while True:
        mode = prompt_hex("Mode > ")
        if mode == 'q':
            break
        if mode is None:
            continue
        if mode not in MODE_NAMES:
            print(f"  Unknown mode 0x{mode:02X}")
            continue

        # Modes that need a PID/param
        if mode not in NO_PARAM_MODES:
            if mode == 0x01:
                print(f"\n  Mode 01 PIDs (your truck's supported PIDs: 01 03-09 0B-11 13-15 18-19 1C 1F 20+):")
                for pid, desc in MODE01_PIDS.items():
                    print(f"    {pid:02X}  {desc}")
                print()
            elif mode == 0x02:
                print("  Enter PID (same as Mode 01):")
            elif mode == 0x09:
                print("  Mode 09 InfoTypes:")
                for p, desc in MODE09_PARAMS.items():
                    print(f"    {p:02X}  {desc}")
                print()

            param = prompt_hex("PID  > ")
            if param == 'q':
                break
            if param is None:
                continue
        else:
            param = 0x00

        print()

        if   mode == 0x01: handle_mode01(ser, param)
        elif mode == 0x02: handle_mode02(ser, param)
        elif mode == 0x03: handle_dtc_mode(ser, 0x03)
        elif mode == 0x04: handle_mode04(ser)
        elif mode == 0x07: handle_dtc_mode(ser, 0x07)
        elif mode == 0x09: handle_mode09(ser, param)
        elif mode == 0x0A: handle_dtc_mode(ser, 0x0A)

        print()

    ser.close()
    print("Disconnected.")

if __name__ == '__main__':
    main()
