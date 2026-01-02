"""
NMEA 2000 CAN Bus Interface

Author: Colin Bitterfield
Email: colin@bitterfield.com
Date Created: 2025-12-22
Date Updated: 2025-12-22
Version: 0.1.0

Provides CAN bus interface for NMEA 2000 PGN message transmission.
Implements Transport Protocol (TP) for multi-frame messages.
"""

import struct
import logging
import time
from typing import Optional, List
from dataclasses import dataclass

try:
    import can
    CAN_AVAILABLE = True
except ImportError:
    CAN_AVAILABLE = False

logger = logging.getLogger(__name__)


@dataclass
class N2KMessage:
    """NMEA 2000 message container"""
    pgn: int
    priority: int
    source: int
    destination: int
    data: bytes


class NMEA2000CANInterface:
    """
    NMEA 2000 CAN Bus Interface

    Supports:
    - Single-frame messages (≤8 bytes)
    - Multi-frame messages via Transport Protocol (TP)
    - BAM (Broadcast Announce Message) for broadcast
    - RTS/CTS for directed messages
    """

    # NMEA 2000 Constants
    BROADCAST_ADDRESS = 255
    NULL_ADDRESS = 254

    # Transport Protocol PGNs
    PGN_TP_CM = 60416  # Transport Protocol - Connection Management
    PGN_TP_DT = 60160  # Transport Protocol - Data Transfer

    # TP.CM Control Bytes
    TP_CM_BAM = 32     # Broadcast Announce Message
    TP_CM_RTS = 16     # Request To Send
    TP_CM_CTS = 17     # Clear To Send
    TP_CM_EOM = 19     # End of Message
    TP_CM_ABORT = 255  # Connection Abort

    def __init__(
        self,
        channel: str = 'can0',
        bustype: str = 'socketcan',
        bitrate: int = 250000,
        source_address: int = 42,
        enabled: bool = False
    ):
        """
        Initialize CAN bus interface.

        Args:
            channel: CAN interface name (e.g., 'can0', '/dev/ttyUSB0')
            bustype: Bus type ('socketcan', 'slcan', 'pcan', etc.)
            bitrate: CAN bitrate (NMEA 2000 standard is 250kbps)
            source_address: Our source address on the N2K network (0-251)
            enabled: Enable CAN output on initialization
        """
        self.channel = channel
        self.bustype = bustype
        self.bitrate = bitrate
        self.source_address = source_address
        self.enabled = enabled
        self.bus: Optional[can.BusABC] = None
        self.sequence_id = 0

        if not CAN_AVAILABLE:
            logger.warning("python-can not installed, CAN bus disabled")
            self.enabled = False
            return

        if self.enabled:
            self.connect()

    def connect(self) -> bool:
        """
        Connect to CAN bus.

        Returns:
            True if connected successfully
        """
        if not CAN_AVAILABLE:
            logger.error("python-can not installed")
            return False

        try:
            # For SLCAN (serial CAN adapters like CANable)
            if self.bustype == 'slcan':
                self.bus = can.interface.Bus(
                    channel=self.channel,
                    bustype='slcan',
                    bitrate=self.bitrate
                )
            # For SocketCAN (Linux native CAN)
            elif self.bustype == 'socketcan':
                self.bus = can.interface.Bus(
                    channel=self.channel,
                    bustype='socketcan',
                    bitrate=self.bitrate
                )
            else:
                # Generic interface
                self.bus = can.interface.Bus(
                    channel=self.channel,
                    bustype=self.bustype,
                    bitrate=self.bitrate
                )

            logger.info(f"CAN bus connected: {self.bustype} on {self.channel} @ {self.bitrate}bps")
            self.enabled = True
            return True

        except Exception as e:
            logger.error(f"Failed to connect to CAN bus: {e}")
            self.enabled = False
            return False

    def disconnect(self):
        """Disconnect from CAN bus"""
        if self.bus:
            try:
                self.bus.shutdown()
                logger.info("CAN bus disconnected")
            except Exception as e:
                logger.error(f"Error disconnecting CAN bus: {e}")
            finally:
                self.bus = None
                self.enabled = False

    def send_message(self, msg: N2KMessage) -> bool:
        """
        Send NMEA 2000 message.

        Automatically handles single-frame or multi-frame transmission.

        Args:
            msg: NMEA 2000 message to send

        Returns:
            True if sent successfully
        """
        if not self.enabled or not self.bus:
            return False

        try:
            # Single-frame message (≤8 bytes)
            if len(msg.data) <= 8:
                return self._send_single_frame(msg)

            # Multi-frame message (>8 bytes) - use Transport Protocol
            else:
                if msg.destination == self.BROADCAST_ADDRESS:
                    return self._send_bam(msg)
                else:
                    return self._send_rts_cts(msg)

        except Exception as e:
            logger.error(f"Error sending N2K message: {e}")
            return False

    def _send_single_frame(self, msg: N2KMessage) -> bool:
        """Send single-frame message (≤8 bytes)"""
        can_id = self._build_can_id(msg.pgn, msg.priority, msg.source, msg.destination)

        can_msg = can.Message(
            arbitration_id=can_id,
            data=msg.data,
            is_extended_id=True
        )

        self.bus.send(can_msg)
        logger.debug(f"Sent single-frame PGN {msg.pgn}: {len(msg.data)} bytes")
        return True

    def _send_bam(self, msg: N2KMessage) -> bool:
        """
        Send multi-frame message using BAM (Broadcast Announce Message).

        Used for broadcast messages >8 bytes.
        """
        data_len = len(msg.data)
        num_packets = (data_len + 6) // 7  # 7 bytes per TP.DT packet

        # Send TP.CM BAM message
        bam_data = struct.pack(
            '<BBHBB',
            self.TP_CM_BAM,     # Control byte
            data_len & 0xFF,    # Data length LSB
            (data_len >> 8) & 0xFF,  # Data length MSB (combined with next byte)
            num_packets,        # Number of packets
            0xFF,               # Reserved
        )
        # Fix: data length is 2 bytes, then num packets, then reserved
        bam_data = struct.pack(
            '<BHBBB',
            self.TP_CM_BAM,           # Control byte
            data_len,                 # Data length (2 bytes)
            num_packets,              # Number of packets
            0xFF,                     # Reserved
            msg.pgn & 0xFF            # PGN byte 0
        )
        bam_data += struct.pack('<H', (msg.pgn >> 8) & 0xFFFF)  # PGN bytes 1-2

        bam_can_id = self._build_can_id(
            self.PGN_TP_CM,
            msg.priority,
            msg.source,
            self.BROADCAST_ADDRESS
        )

        bam_msg = can.Message(
            arbitration_id=bam_can_id,
            data=bam_data,
            is_extended_id=True
        )
        self.bus.send(bam_msg)
        logger.debug(f"Sent TP.CM BAM for PGN {msg.pgn}: {num_packets} packets")

        # Send TP.DT packets
        for i in range(num_packets):
            self.sequence_id = (self.sequence_id + 1) % 256

            # Build packet data (sequence byte + up to 7 data bytes)
            packet_data = bytearray([self.sequence_id])

            start_idx = i * 7
            end_idx = min(start_idx + 7, data_len)
            packet_data.extend(msg.data[start_idx:end_idx])

            # Pad to 8 bytes with 0xFF
            while len(packet_data) < 8:
                packet_data.append(0xFF)

            dt_can_id = self._build_can_id(
                self.PGN_TP_DT,
                msg.priority,
                msg.source,
                self.BROADCAST_ADDRESS
            )

            dt_msg = can.Message(
                arbitration_id=dt_can_id,
                data=bytes(packet_data),
                is_extended_id=True
            )
            self.bus.send(dt_msg)

            # Small delay between packets (NMEA 2000 recommends 50-200ms)
            time.sleep(0.05)

        logger.debug(f"Sent BAM complete for PGN {msg.pgn}: {data_len} bytes in {num_packets} packets")
        return True

    def _send_rts_cts(self, msg: N2KMessage) -> bool:
        """
        Send multi-frame message using RTS/CTS (Request To Send / Clear To Send).

        Used for directed messages >8 bytes.
        Not implemented in this version (requires receiving CTS responses).
        """
        logger.warning("RTS/CTS not implemented, using BAM for directed message")
        # Fall back to BAM
        return self._send_bam(msg)

    def _build_can_id(
        self,
        pgn: int,
        priority: int,
        source: int,
        destination: int
    ) -> int:
        """
        Build 29-bit CAN identifier for NMEA 2000.

        Format:
        - Priority (3 bits): 0-7 (lower is higher priority)
        - Reserved (1 bit): always 0
        - Data Page (1 bit): bit 24 of PGN
        - PDU Format (8 bits): bits 16-23 of PGN
        - PDU Specific (8 bits): bits 8-15 of PGN OR destination address
        - Source Address (8 bits): 0-251

        Args:
            pgn: Parameter Group Number
            priority: Message priority (0-7)
            source: Source address (0-251)
            destination: Destination address (0-255, 255=broadcast)

        Returns:
            29-bit CAN identifier
        """
        # Extract PGN components
        pdu_format = (pgn >> 8) & 0xFF

        # PDU1 format (0-239): point-to-point, PDU Specific = destination
        # PDU2 format (240-255): broadcast, PDU Specific = group extension
        if pdu_format < 240:
            # PDU1 - replace bits 8-15 with destination
            pgn_modified = (pgn & 0xFF0000) | (destination << 8) | (pgn & 0xFF)
        else:
            # PDU2 - keep PGN as-is
            pgn_modified = pgn

        # Build 29-bit CAN ID
        can_id = (
            ((priority & 0x07) << 26) |  # Priority (bits 26-28)
            (pgn_modified << 8) |         # PGN (bits 8-25)
            (source & 0xFF)               # Source (bits 0-7)
        )

        return can_id

    def send_pgn_129029(self, pgn_data: bytes) -> bool:
        """
        Send PGN 129029 (GNSS Position Data).

        Args:
            pgn_data: Encoded PGN 129029 data (51 bytes)

        Returns:
            True if sent successfully
        """
        msg = N2KMessage(
            pgn=129029,
            priority=3,  # Normal priority for position data
            source=self.source_address,
            destination=self.BROADCAST_ADDRESS,
            data=pgn_data
        )

        return self.send_message(msg)

    def get_status(self) -> dict:
        """Get CAN interface status"""
        return {
            'enabled': self.enabled,
            'connected': self.bus is not None,
            'channel': self.channel,
            'bustype': self.bustype,
            'bitrate': self.bitrate,
            'source_address': self.source_address,
            'python_can_available': CAN_AVAILABLE
        }


def setup_slcan_interface(device: str = '/dev/ttyUSB0', bitrate: int = 250000) -> bool:
    """
    Setup SLCAN interface (for USB serial CAN adapters).

    This function configures the serial CAN adapter and creates a SocketCAN interface.
    Run with sudo permissions.

    Args:
        device: Serial device path (e.g., '/dev/ttyUSB0')
        bitrate: CAN bitrate (250000 for NMEA 2000)

    Returns:
        True if setup successful

    Example:
        setup_slcan_interface('/dev/ttyUSB0', 250000)
    """
    import subprocess

    try:
        # Calculate SLCAN speed code (S0-S8)
        # S0=10k, S1=20k, S2=50k, S3=100k, S4=125k, S5=250k, S6=500k, S7=800k, S8=1M
        speed_map = {
            10000: '0',
            20000: '1',
            50000: '2',
            100000: '3',
            125000: '4',
            250000: '5',
            500000: '6',
            800000: '7',
            1000000: '8'
        }
        speed_code = speed_map.get(bitrate, '5')  # Default to 250k

        # Load slcan module
        subprocess.run(['sudo', 'modprobe', 'slcan'], check=True)

        # Attach SLCAN to serial device
        subprocess.run([
            'sudo', 'slcand',
            f'-o',  # Open device
            f'-s{speed_code}',  # Set speed
            f'-t', 'hw',  # Hardware timestamps
            f'-S', '1000000',  # Serial baud rate
            device,
            'slcan0'
        ], check=True)

        # Bring up interface
        subprocess.run(['sudo', 'ip', 'link', 'set', 'slcan0', 'up'], check=True)

        logger.info(f"SLCAN interface setup: slcan0 on {device} @ {bitrate}bps")
        return True

    except subprocess.CalledProcessError as e:
        logger.error(f"Failed to setup SLCAN interface: {e}")
        return False
    except Exception as e:
        logger.error(f"Error setting up SLCAN: {e}")
        return False
