"""
STL-19P LiDAR Sensor - Python Driver
=====================================
Hardware:  LDROBOT STL-19P (Waveshare D500 LiDAR Kit)
Interface: UART @ 230400 baud, 8N1, one-way (LiDAR → host only)
Connector: ZH1.5T-4P1.5mm  →  Pin1=Tx, Pin2=PWM, Pin3=GND, Pin4=5V

Packet format (47 bytes total, same protocol as LD19/LD06):
  [0]     Header    = 0x54
  [1]     Ver_Len   = 0x2C  (type=1 in upper 3 bits, 12 points in lower 5 bits)
  [2:4]   Speed     uint16 LE  (deg/s)
  [4:6]   StartAngle uint16 LE (units: 0.01 deg)
  [6:42]  12 × Point (3 bytes each):
              [0:2]  distance  uint16 LE  (mm)
              [2]    intensity uint8
  [42:44] EndAngle  uint16 LE  (units: 0.01 deg)
  [44:46] Timestamp uint16 LE  (ms, wraps at 30000)
  [46]    CRC8

Angle of each point is linearly interpolated between StartAngle and EndAngle.

Usage
-----
  python stl19p_lidar.py                     # prints live readings
  python stl19p_lidar.py --port COM3         # Windows
  python stl19p_lidar.py --port /dev/ttyUSB0 # Linux/macOS
  python stl19p_lidar.py --plot              # live matplotlib polar plot

Requirements:  pip install pyserial matplotlib numpy
"""

import argparse
import math
import struct
import time
import threading
import operator

from collections import deque

import serial

import numpy as np

CONDITIONS = {
    "<": operator.lt,
    "<=": operator.le,
    "==": operator.eq,
    ">": operator.gt,
    ">=": operator.ge,
    "!=": operator.ne
}

# ──────────────────────────────────────────── Protocol constants ────────────────────────────────────────────
HEADER_BYTE    = 0x54
VER_LEN_BYTE   = 0x2C
POINT_PER_PACK = 12
PACKET_LENGTH  = 47          # bytes in one complete packet
BAUD_RATE      = 230400

# CRC-8 lookup table (polynomial 0x4D, used by LDROBOT protocol)
CRC_TABLE = [
    0x00, 0x4D, 0x9A, 0xD7, 0x79, 0x34, 0xE3, 0xAE,
    0xF2, 0xBF, 0x68, 0x25, 0x8B, 0xC6, 0x11, 0x5C,
    0xA9, 0xE4, 0x33, 0x7E, 0xD0, 0x9D, 0x4A, 0x07,
    0x5B, 0x16, 0xC1, 0x8C, 0x22, 0x6F, 0xB8, 0xF5,
    0x1F, 0x52, 0x85, 0xC8, 0x66, 0x2B, 0xFC, 0xB1,
    0xED, 0xA0, 0x77, 0x3A, 0x94, 0xD9, 0x0E, 0x43,
    0xB6, 0xFB, 0x2C, 0x61, 0xCF, 0x82, 0x55, 0x18,
    0x44, 0x09, 0xDE, 0x93, 0x3D, 0x70, 0xA7, 0xEA,
    0x3E, 0x73, 0xA4, 0xE9, 0x47, 0x0A, 0xDD, 0x90,
    0xCC, 0x81, 0x56, 0x1B, 0xB5, 0xF8, 0x2F, 0x62,
    0x97, 0xDA, 0x0D, 0x40, 0xEE, 0xA3, 0x74, 0x39,
    0x65, 0x28, 0xFF, 0xB2, 0x1C, 0x51, 0x86, 0xCB,
    0x21, 0x6C, 0xBB, 0xF6, 0x58, 0x15, 0xC2, 0x8F,
    0xD3, 0x9E, 0x49, 0x04, 0xAA, 0xE7, 0x30, 0x7D,
    0x88, 0xC5, 0x12, 0x5F, 0xF1, 0xBC, 0x6B, 0x26,
    0x7A, 0x37, 0xE0, 0xAD, 0x03, 0x4E, 0x99, 0xD4,
    0x7C, 0x31, 0xE6, 0xAB, 0x05, 0x48, 0x9F, 0xD2,
    0x8E, 0xC3, 0x14, 0x59, 0xF7, 0xBA, 0x6D, 0x20,
    0xD5, 0x98, 0x4F, 0x02, 0xAC, 0xE1, 0x36, 0x7B,
    0x27, 0x6A, 0xBD, 0xF0, 0x5E, 0x13, 0xC4, 0x89,
    0x63, 0x2E, 0xF9, 0xB4, 0x1A, 0x57, 0x80, 0xCD,
    0x91, 0xDC, 0x0B, 0x46, 0xE8, 0xA5, 0x72, 0x3F,
    0xCA, 0x87, 0x50, 0x1D, 0xB3, 0xFE, 0x29, 0x64,
    0x38, 0x75, 0xA2, 0xEF, 0x41, 0x0C, 0xDB, 0x96,
    0x42, 0x0F, 0xD8, 0x95, 0x3B, 0x76, 0xA1, 0xEC,
    0xB0, 0xFD, 0x2A, 0x67, 0xC9, 0x84, 0x53, 0x1E,
    0xEB, 0xA6, 0x71, 0x3C, 0x92, 0xDF, 0x08, 0x45,
    0x19, 0x54, 0x83, 0xCE, 0x60, 0x2D, 0xFA, 0xB7,
    0x5D, 0x10, 0xC7, 0x8A, 0x24, 0x69, 0xBE, 0xF3,
    0xAF, 0xE2, 0x35, 0x78, 0xD6, 0x9B, 0x4C, 0x01,
    0xF4, 0xB9, 0x6E, 0x23, 0x8D, 0xC0, 0x17, 0x5A,
    0x06, 0x4B, 0x9C, 0xD1, 0x7F, 0x32, 0xE5, 0xA8,
]


def crc8(data: bytes) -> int:
    """Calculate CRC-8 checksum over 'data'."""
    crc = 0
    for byte in data:
        crc = CRC_TABLE[(crc ^ byte) & 0xFF]
    return crc


# ──────────────────────────────────────────── Data structures ────────────────────────────────────────────

class LidarPoint:
    """A single ranging point."""
    __slots__ = ("angle", "distance", "intensity", "x", "y")

    def __init__(self, angle: float | np.floating, distance: int, intensity: int):
        self.angle     = angle       # degrees  [0, 360)
        self.distance  = distance    # mm
        self.intensity = intensity   # 0-255
        rad = math.radians(angle)
        self.x = distance * math.cos(rad)   # mm
        self.y = distance * math.sin(rad)   # mm  (left-hand coord: clockwise)

    def __repr__(self):
        return (f"LidarPoint(angle={self.angle:.2f}°, "
                f"dist={self.distance}mm, intensity={self.intensity})")
    
    def __getitem__(self, key: str):
        return getattr(self, key)


class LidarPacket:
    """Parsed 47-byte STL-19P data packet."""
    __slots__ = ("speed", "start_angle", "end_angle", "timestamp", "points")

    def __init__(self, speed, start_angle, end_angle, timestamp, points):
        self.speed       = speed         # deg/s
        self.start_angle = start_angle   # deg
        self.end_angle   = end_angle     # deg
        self.timestamp   = timestamp     # ms
        self.points      = points        # list[LidarPoint]


# ──────────────────────────────────────────── Parser ────────────────────────────────────────────

def parse_packet(raw: bytes) -> LidarPacket | None:
    """
    Parse a 47-byte raw packet.
    Returns a LidarPacket on success, None on failure.
    """
    if len(raw) != PACKET_LENGTH:
        return None
    if raw[0] != HEADER_BYTE or raw[1] != VER_LEN_BYTE:
        return None

    received_crc   = raw[PACKET_LENGTH - 1]
    calculated_crc = crc8(raw[:PACKET_LENGTH - 1])
    if received_crc != calculated_crc:
        return None

    speed       = struct.unpack_from("<H", raw, 2)[0]          # deg/s
    start_angle = struct.unpack_from("<H", raw, 4)[0] / 100.0  # deg
    end_angle   = struct.unpack_from("<H", raw, 42)[0] / 100.0 # deg
    timestamp   = struct.unpack_from("<H", raw, 44)[0]         # ms

    # Angular span (handle wrap-around at 360°)
    angle_span = (end_angle - start_angle) % 360.0
    angle_step = angle_span / (POINT_PER_PACK - 1) if POINT_PER_PACK > 1 else 0

    points = []
    offset = 6
    for i in range(POINT_PER_PACK):
        dist, intens = struct.unpack_from("<HB", raw, offset)
        offset += 3
        angle = (start_angle + angle_step * i) % 360.0
        if dist > 0:                      # 0 means invalid measurement
            points.append(LidarPoint(angle, dist, intens))

    return LidarPacket(speed, start_angle, end_angle, timestamp, points)


# ──────────────────────────────────────────── Reader ────────────────────────────────────────────

class STL19P:
    """
    Reads data from the STL-19P LiDAR over a serial port.

    Example
    -------
    with STL19P("/dev/ttyUSB0") as lidar:
        for scan in lidar.scans():
            for point in scan:
                print(point)
    """

    def __init__(self, port: str, timeout: float = 2.0, filter: dict = None):
        self.port       = port
        self.timeout    = timeout
        self._ser       = None
        self.filter     = {} if filter is None else filter
        self.cfilter    = []

        self._compile_filters()

    def open(self):
        self._ser = serial.Serial(
            port     = self.port,
            baudrate = BAUD_RATE,
            bytesize = serial.EIGHTBITS,
            parity   = serial.PARITY_NONE,
            stopbits = serial.STOPBITS_ONE,
            timeout  = self.timeout,
        )
        self._ser.reset_input_buffer()
        print(f"[STL-19P] Opened {self.port} @ {BAUD_RATE} baud")

    def close(self):
        if self._ser and self._ser.is_open:
            self._ser.close()
            print("[STL-19P] Port closed")

    def __enter__(self):
        self.open()
        return self

    def __exit__(self, *_):
        self.close()

    def _sync(self) -> bool:
        """Advance the serial stream until the two-byte packet header is found."""
        while True:
            b = self._ser.read(1)
            if not b:
                return False          # timeout
            if b[0] == HEADER_BYTE:
                b2 = self._ser.read(1)
                if b2 and b2[0] == VER_LEN_BYTE:
                    return True
    
    def _compile_filters(self):
        self.cfilter

        for key, value in self.filter.items():
            if type(value) is list:
                for cond, v in value:
                    self.cfilter.append((key, CONDITIONS[cond], v))
            
            else:
                self.cfilter.append((key, CONDITIONS[value[0]], value[1]))

    def filter_scans(self, ls_scans):
        """
        Filter the LidarPoint based on the conditions
        """
        if not self.filter:
            return ls_scans

        return [
            point for point in ls_scans
            if all(fn(point[key], value) for key, fn, value in self.cfilter)
        ]
    
    def read_packet(self) -> LidarPacket | None:
        """
        Read and return the next valid packet.
        Blocks until a packet arrives or timeout occurs.
        Returns None on timeout or CRC failure.
        """
        if not self._sync():
            return None
        # Already consumed 2 header bytes; read remaining 45
        rest = self._ser.read(PACKET_LENGTH - 2)
        if len(rest) != PACKET_LENGTH - 2:
            return None
        raw = bytes([HEADER_BYTE, VER_LEN_BYTE]) + rest
        return parse_packet(raw)

    def packets(self):
        """Generator that yields LidarPacket objects indefinitely."""
        while True:
            pkt = self.read_packet()
            if pkt is not None:
                yield pkt

    def scans(self):
        """
        Generator that yields one complete 360° scan at a time.
        A new scan is detected when the angle wraps around (end > start of next).
        Each scan is a list[LidarPoint] sorted by angle.
        """
        current_scan: list[LidarPoint] = []
        prev_end_angle = None

        for pkt in self.packets():
            # Detect a new revolution: current start_angle < previous end_angle
            if (prev_end_angle is not None and
                    pkt.start_angle < prev_end_angle and
                    current_scan):
                current_scan = self.filter_scans(current_scan)
                yield sorted(current_scan, key=lambda p: p.angle)
                current_scan = []

            current_scan.extend(pkt.points)
            prev_end_angle = pkt.end_angle


# ──────────────────────────────────────────── Live polar plot (optional) ────────────────────────────────────────────

def run_plot(lidar: STL19P):
    """Display a live polar plot of the LiDAR scan (requires matplotlib)."""
    import numpy as np
    import matplotlib.pyplot as plt
    import matplotlib.animation as animation

    scan_buffer: deque = deque(maxlen=1)

    def reader():
        for scan in lidar.scans():
            scan_buffer.append(scan)

    t = threading.Thread(target=reader, daemon=True)
    t.start()

    fig = plt.figure(figsize=(7, 7))
    ax  = fig.add_subplot(111, projection="polar")
    ax.set_title("STL-19P Live Scan", va="bottom")
    ax.set_theta_zero_location("N")      # 0° at top
    ax.set_theta_direction(-1)           # clockwise (matches sensor coord)
    scat = ax.scatter([], [], s=2, c=[], cmap="plasma", vmin=0, vmax=255)

    def update(_):
        if not scan_buffer:
            return (scat,)
        scan = scan_buffer[-1]
        scan = [s for s in scan if s.intensity > 00]  # Filter out weak points
        if not scan:
            return (scat,)
        angles    = np.radians([p.angle for p in scan])
        distances = np.array([p.distance / 1000.0 for p in scan])   # → metres
        intensities = np.array([p.intensity for p in scan])
        scat.set_offsets(np.column_stack([angles, distances]))
        scat.set_array(intensities)
        ax.set_rmax(max(distances) * 1.1 if distances.size else 1.0)
        return (scat,)

    ani = animation.FuncAnimation(fig, update, interval=100, blit=False)
    plt.tight_layout()
    plt.show()


# ──────────────────────────────────────────── CLI entry point ────────────────────────────────────────────

def main():
    parser = argparse.ArgumentParser(
        description="Read data from LDROBOT STL-19P / Waveshare D500 LiDAR"
    )
    parser.add_argument(
        "--port", default="/dev/ttyUSB0",
        help="Serial port (e.g. /dev/ttyUSB0 or COM3). Default: /dev/ttyUSB0"
    )
    parser.add_argument(
        "--plot", action="store_true",
        help="Show a live polar plot instead of printing to console"
    )
    parser.add_argument(
        "--packets", action="store_true",
        help="Print individual packets instead of full scans"
    )
    parser.add_argument(
        "--max-scans", type=int, default=0,
        help="Stop after this many scans (0 = run forever)"
    )
    parser.add_argument(
        "--scan", action="store_true",
        help="Perform a single scan and print all the points with a certain intensity threshold"
    )
    args = parser.parse_args()

    with STL19P(args.port, filter={"distance": ("<", 4000), "angle": [("<", 270), (">", 90)]}) as lidar:
        if args.plot:
            run_plot(lidar)
            return

        if args.packets:
            print("Reading packets (Ctrl-C to stop)…")
            for pkt in lidar.packets():
                print(f"  speed={pkt.speed}°/s  "
                      f"angles={pkt.start_angle:.1f}→{pkt.end_angle:.1f}°  "
                      f"ts={pkt.timestamp}ms  "
                      f"points={len(pkt.points)}")
        
        elif args.scan:
            print("Performing a single scan with intensity > 200…")
            for scan in lidar.scans():
                strong_points = [p for p in scan if p["intensity"] > 0]
                print(f"Scan with {len(strong_points)} strong points:")
                for pt in strong_points[:200]:
                    print(f"LidarPoint(angle={pt.angle:.2f}°, dist={pt.distance}mm, intensity={pt.intensity}, x={pt.x:.01f}mm, y={pt.y:.01f}mm)")
                break

        else:
            print("Reading full 360° scans (Ctrl-C to stop)…")
            count = 0
            for scan in lidar.scans():
                count += 1
                closest = min(scan, key=lambda p: p.distance) if scan else None
                print(f"\n── Scan #{count}  ({len(scan)} points) ──")
                if closest:
                    print(f"  Closest: {closest.distance}mm @ {closest.angle:.1f}°")
                # Print a few sample points
                for pt in scan[::max(1, len(scan) // 8)]:
                    print(f"LidarPoint(angle={pt.angle:.2f}°, dist={pt.distance}mm, intensity={pt.intensity}, x={pt.x:.01f}mm, y={pt.y:.01f}mm)")
                if args.max_scans and count >= args.max_scans:
                    break


if __name__ == "__main__":
    try:
        main()
    except KeyboardInterrupt:
        print("\n[STL-19P] Stopped by user.")