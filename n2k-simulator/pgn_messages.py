"""
NMEA 2000 PGN Message Generator

Author: Colin Bitterfield
Email: colin@bitterfield.com
Date Created: 2025-11-30
Date Updated: 2025-11-30
Version: 0.1.0

Generates NMEA 2000 PGN messages, particularly PGN 129029 for GPS data.
"""

import struct
import time
import math
from dataclasses import dataclass
from typing import Tuple


@dataclass
class GPSPosition:
    """GPS position data"""
    latitude: float  # degrees
    longitude: float  # degrees
    altitude: float = 0.0  # meters
    speed_over_ground: float = 0.0  # m/s
    course_over_ground: float = 0.0  # radians
    timestamp: float = None

    def __post_init__(self):
        if self.timestamp is None:
            self.timestamp = time.time()


@dataclass
class VesselHeading:
    """Vessel heading data"""
    heading: float  # radians (0 = North, clockwise)
    deviation: float = 0.0  # radians (magnetic deviation)
    variation: float = 0.0  # radians (magnetic variation)
    reference: str = 'Magnetic'  # 'Magnetic' or 'True'
    timestamp: float = None

    def __post_init__(self):
        if self.timestamp is None:
            self.timestamp = time.time()


@dataclass
class DeviceInfo:
    """NMEA 2000 device identification"""
    manufacturer_code: int = 2046  # 2046 = Unknown/Test
    unique_number: int = 123456
    device_function: int = 130  # 130 = PC Gateway
    device_class: int = 25  # 25 = Inter/Intranetwork Device
    device_instance: int = 0
    system_instance: int = 0
    industry_group: int = 4  # 4 = Marine


class PGN129029:
    """
    PGN 129029 - GNSS Position Data

    This PGN provides position information from GNSS (GPS) systems.

    Field Layout (total 51 bytes):
    - SID (1 byte)
    - Date (2 bytes) - days since 1970-01-01
    - Time (4 bytes) - seconds since midnight
    - Latitude (8 bytes) - 1e-16 degrees
    - Longitude (8 bytes) - 1e-16 degrees
    - Altitude (8 bytes) - 1e-6 meters
    - GNSS Type (4 bits)
    - Method (4 bits)
    - Integrity (2 bits)
    - Reserved (6 bits)
    - Number of SVs (1 byte)
    - HDOP (2 bytes) - 0.01
    - PDOP (2 bytes) - 0.01
    - Geoidal Separation (4 bytes) - 0.01 meters
    - Reference Stations (1 byte)
    - Reference Station Type (4 bits)
    - Reference Station ID (12 bits)
    - Age of Correction (2 bytes) - 0.01 seconds
    """

    PGN = 129029

    @staticmethod
    def encode(position: GPSPosition, sid: int = 0) -> bytes:
        """
        Encode GPS position into PGN 129029 message.

        Returns: Raw PGN 129029 data bytes
        """
        # Convert timestamp to days and seconds
        days_since_1970 = int(position.timestamp / 86400)
        seconds_since_midnight = position.timestamp % 86400

        # Convert to N2K units
        lat_n2k = int(position.latitude * 1e7)  # 1e-7 degrees resolution
        lon_n2k = int(position.longitude * 1e7)
        alt_n2k = int(position.altitude * 100)  # 0.01 meter resolution

        # Build the message
        data = bytearray()

        # SID (Sequence ID)
        data.append(sid & 0xFF)

        # Date (days since 1970-01-01)
        data.extend(struct.pack('<H', days_since_1970 & 0xFFFF))

        # Time (seconds since midnight in 0.0001 second resolution)
        time_n2k = int(seconds_since_midnight * 10000)
        data.extend(struct.pack('<I', time_n2k))

        # Latitude (1e-7 degrees)
        data.extend(struct.pack('<q', lat_n2k))

        # Longitude (1e-7 degrees)
        data.extend(struct.pack('<q', lon_n2k))

        # Altitude (0.01 meters)
        data.extend(struct.pack('<q', alt_n2k))

        # GNSS Type (4 bits) + Method (4 bits)
        # Type: 0=GPS, 1=GLONASS, 2=GPS+GLONASS, 3=GPS+SBAS/WAAS
        # Method: 0=no GNSS, 1=GNSS fix, 2=DGNSS fix, 3=Precise GNSS
        gnss_type = 0  # GPS
        method = 1  # GNSS fix
        data.append((method << 4) | gnss_type)

        # Integrity (2 bits) + Reserved (6 bits)
        integrity = 0  # No integrity checking
        data.append(integrity)

        # Number of SVs (satellites)
        data.append(8)  # Simulate 8 satellites

        # HDOP (Horizontal Dilution of Precision) - 0.01 resolution
        hdop_n2k = int(1.2 * 100)  # 1.2 HDOP
        data.extend(struct.pack('<H', hdop_n2k))

        # PDOP (Position Dilution of Precision) - 0.01 resolution
        pdop_n2k = int(2.1 * 100)  # 2.1 PDOP
        data.extend(struct.pack('<H', pdop_n2k))

        # Geoidal Separation - 0.01 meters
        geoid_sep = int(0.0 * 100)
        data.extend(struct.pack('<i', geoid_sep))

        # Number of Reference Stations
        data.append(0)

        # Reference Station Type (4 bits) + Reference Station ID (12 bits)
        data.extend(struct.pack('<H', 0))

        # Age of Correction - 0.01 seconds
        data.extend(struct.pack('<H', 0))

        return bytes(data)

    @staticmethod
    def to_nmea0183(position: GPSPosition) -> str:
        """
        Convert position to NMEA 0183 format (for compatibility).
        Returns GPGGA sentence.
        """
        timestamp = time.gmtime(position.timestamp)

        # Convert latitude to NMEA format (DDMM.MMMM)
        lat_deg = abs(int(position.latitude))
        lat_min = (abs(position.latitude) - lat_deg) * 60
        lat_str = f"{lat_deg:02d}{lat_min:07.4f}"
        lat_dir = 'N' if position.latitude >= 0 else 'S'

        # Convert longitude to NMEA format (DDDMM.MMMM)
        lon_deg = abs(int(position.longitude))
        lon_min = (abs(position.longitude) - lon_deg) * 60
        lon_str = f"{lon_deg:03d}{lon_min:07.4f}"
        lon_dir = 'E' if position.longitude >= 0 else 'W'

        # Build GPGGA sentence
        time_str = f"{timestamp.tm_hour:02d}{timestamp.tm_min:02d}{timestamp.tm_sec:02d}.00"

        sentence = (
            f"GPGGA,{time_str},{lat_str},{lat_dir},{lon_str},{lon_dir},"
            f"1,08,1.2,{position.altitude:.1f},M,0.0,M,,"
        )

        # Calculate checksum
        checksum = 0
        for char in sentence:
            checksum ^= ord(char)

        return f"${sentence}*{checksum:02X}\r\n"


class PGN127250:
    """
    PGN 127250 - Vessel Heading

    This PGN provides vessel heading information from magnetic or true compass.

    Field Layout (8 bytes):
    - SID (1 byte)
    - Heading (2 bytes) - radians, 0.0001 resolution
    - Deviation (2 bytes) - radians, 0.0001 resolution
    - Variation (2 bytes) - radians, 0.0001 resolution
    - Reference (2 bits) - 0=True, 1=Magnetic, 2=Error, 3=Null
    - Reserved (6 bits)
    """

    PGN = 127250

    @staticmethod
    def encode(heading: VesselHeading, sid: int = 0) -> bytes:
        """
        Encode vessel heading into PGN 127250 message.

        Returns: Raw PGN 127250 data bytes
        """
        # Convert to N2K units (0.0001 radians)
        heading_n2k = int(heading.heading * 10000) & 0xFFFF
        deviation_n2k = int(heading.deviation * 10000) & 0xFFFF
        variation_n2k = int(heading.variation * 10000) & 0xFFFF

        # Reference field
        reference_map = {'True': 0, 'Magnetic': 1}
        reference = reference_map.get(heading.reference, 1)

        # Build the message
        data = bytearray()

        # SID
        data.append(sid & 0xFF)

        # Heading (2 bytes)
        data.extend(struct.pack('<H', heading_n2k))

        # Deviation (2 bytes)
        data.extend(struct.pack('<h', deviation_n2k))

        # Variation (2 bytes)
        data.extend(struct.pack('<h', variation_n2k))

        # Reference (2 bits) + Reserved (6 bits)
        data.append(reference & 0x03)

        return bytes(data)

    @staticmethod
    def to_nmea0183(heading: VesselHeading) -> str:
        """
        Convert heading to NMEA 0183 format.
        Returns HDM (Heading Magnetic) or HDT (Heading True) sentence.
        """
        heading_degrees = math.degrees(heading.heading) % 360

        if heading.reference == 'True':
            # HDT - Heading True
            sentence = f"HDT,{heading_degrees:.1f},T"
        else:
            # HDM - Heading Magnetic
            sentence = f"HDM,{heading_degrees:.1f},M"

        # Calculate checksum
        checksum = 0
        for char in sentence:
            checksum ^= ord(char)

        return f"${sentence}*{checksum:02X}\r\n"


class PGN60928:
    """
    PGN 60928 - ISO Address Claim

    This PGN is used for device identification on the NMEA 2000 network.

    Field Layout (8 bytes):
    - Unique Number (21 bits)
    - Manufacturer Code (11 bits)
    - Device Instance Lower (3 bits)
    - Device Instance Upper (5 bits)
    - Device Function (8 bits)
    - Device Class (7 bits)
    - Reserved (1 bit)
    - System Instance (4 bits)
    - Industry Group (3 bits)
    - Reserved (1 bit)
    """

    PGN = 60928

    @staticmethod
    def encode(device: DeviceInfo) -> bytes:
        """
        Encode device information into PGN 60928 message.

        Returns: Raw PGN 60928 data bytes
        """
        # Build NAME field (64 bits)
        name = 0

        # Unique Number (bits 0-20)
        name |= (device.unique_number & 0x1FFFFF)

        # Manufacturer Code (bits 21-31)
        name |= ((device.manufacturer_code & 0x7FF) << 21)

        # Device Instance (bits 32-39, split into lower 3 + upper 5)
        device_instance = device.device_instance & 0xFF
        name |= ((device_instance & 0xFF) << 32)

        # Device Function (bits 40-47)
        name |= ((device.device_function & 0xFF) << 40)

        # Device Class (bits 48-54)
        name |= ((device.device_class & 0x7F) << 48)

        # System Instance (bits 56-59)
        name |= ((device.system_instance & 0x0F) << 56)

        # Industry Group (bits 60-62)
        name |= ((device.industry_group & 0x07) << 60)

        # Pack as 8 bytes (little-endian)
        data = struct.pack('<Q', name)

        return bytes(data)


def calculate_new_position(
    lat: float,
    lon: float,
    speed_ms: float,
    course_rad: float,
    time_delta: float
) -> Tuple[float, float]:
    """
    Calculate new position based on speed and course.

    Args:
        lat: Current latitude (degrees)
        lon: Current longitude (degrees)
        speed_ms: Speed in meters/second
        course_rad: Course in radians (0 = North, clockwise)
        time_delta: Time elapsed in seconds

    Returns:
        (new_lat, new_lon) in degrees
    """
    # Earth radius in meters
    EARTH_RADIUS = 6371000.0

    # Distance traveled
    distance = speed_ms * time_delta

    # Convert to radians
    lat_rad = math.radians(lat)
    lon_rad = math.radians(lon)

    # Calculate new position using great circle navigation
    new_lat_rad = math.asin(
        math.sin(lat_rad) * math.cos(distance / EARTH_RADIUS) +
        math.cos(lat_rad) * math.sin(distance / EARTH_RADIUS) * math.cos(course_rad)
    )

    new_lon_rad = lon_rad + math.atan2(
        math.sin(course_rad) * math.sin(distance / EARTH_RADIUS) * math.cos(lat_rad),
        math.cos(distance / EARTH_RADIUS) - math.sin(lat_rad) * math.sin(new_lat_rad)
    )

    return math.degrees(new_lat_rad), math.degrees(new_lon_rad)
