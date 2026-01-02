# CAN Bus Setup Guide

**Author:** Colin Bitterfield
**Email:** colin@bitterfield.com
**Date Created:** 2025-12-22
**Date Updated:** 2025-12-22
**Version:** 0.1.0

## Overview

The N2K Simulator now supports outputting NMEA 2000 GPS data via CAN bus using USB CAN adapters. This allows direct connection to physical NMEA 2000 networks for testing and development.

## Supported Hardware

### USB CAN Adapters

The simulator supports any CAN adapter compatible with python-can, including:

- **CANable** (SLCAN protocol)
- **PEAK PCAN-USB**
- **Kvaser interfaces**
- **Generic CH340-based adapters** (0x1a86:0x7523)

### Your Hardware

Based on your USB device info:
```
USB Vendor ID:  0x1a86
USB Product ID: 0x7523
Connection:     /dev/ttyUSB0 (typical)
```

This is a **CH340 USB-to-Serial adapter**, commonly used with SLCAN CAN adapters.

## Architecture

### NMEA 2000 CAN Interface

The simulator implements full NMEA 2000 protocol including:

- **29-bit CAN identifiers** (CAN 2.0B extended frame format)
- **Transport Protocol (TP)** for multi-frame messages
  - BAM (Broadcast Announce Message) for broadcast
  - RTS/CTS for directed messages (future)
- **PGN 129029** (GNSS Position Data) - 51 bytes
  - Uses TP.CM and TP.DT for multi-frame transmission
  - 7 data bytes per TP.DT packet
  - 50ms delay between packets

### CAN Frame Format

```
29-bit CAN ID Structure:
[28:26] Priority (3 bits)      - 0-7 (lower = higher priority)
[25]    Reserved (1 bit)       - Always 0
[24]    Data Page (1 bit)      - Bit 24 of PGN
[23:16] PDU Format (8 bits)    - Bits 16-23 of PGN
[15:8]  PDU Specific (8 bits)  - Bits 8-15 of PGN OR destination
[7:0]   Source Address (8 bits) - 0-251
```

## Setup Instructions

### 1. Hardware Connection

Connect your USB CAN adapter to your computer:

```bash
# Verify USB device
lsusb | grep 1a86:7523

# Check device node (usually /dev/ttyUSB0)
ls -l /dev/ttyUSB*
```

### 2. Install Required Packages (Linux)

#### Ubuntu/Debian

```bash
# Install can-utils and SLCAN tools
sudo apt-get update
sudo apt-get install can-utils

# Load kernel modules
sudo modprobe can
sudo modprobe slcan
sudo modprobe can-raw
```

#### macOS

```bash
# Install can-utils via Homebrew
brew install can-utils

# Note: macOS requires additional setup for USB CAN adapters
# Consider using a Linux VM or Docker with --privileged mode
```

### 3. Configure SLCAN Interface

#### Option A: Manual Setup (Recommended)

```bash
# Set serial port permissions
sudo chmod 666 /dev/ttyUSB0

# Attach SLCAN to serial device
# S5 = 250kbps (NMEA 2000 standard)
sudo slcand -o -s5 -t hw -S 1000000 /dev/ttyUSB0 slcan0

# Bring up interface
sudo ip link set slcan0 up

# Verify interface is up
ip link show slcan0
```

#### Option B: Using Helper Script

```bash
# Run setup script (included in can_interface.py)
python3 -c "from n2k-simulator.can_interface import setup_slcan_interface; \
            setup_slcan_interface('/dev/ttyUSB0', 250000)"
```

#### Option C: Persistent Configuration (systemd)

Create `/etc/systemd/system/slcan0.service`:

```ini
[Unit]
Description=SLCAN0 CAN Interface
After=network.target

[Service]
Type=forking
ExecStart=/usr/bin/slcand -o -s5 -t hw -S 1000000 /dev/ttyUSB0 slcan0
ExecStartPost=/sbin/ip link set slcan0 up
ExecStop=/sbin/ip link set slcan0 down
ExecStopPost=/usr/bin/killall slcand
Restart=on-failure

[Install]
WantedBy=multi-user.target
```

Enable and start:

```bash
sudo systemctl daemon-reload
sudo systemctl enable slcan0.service
sudo systemctl start slcan0.service
```

### 4. Enable CAN in N2K Simulator

#### Environment Variables

Edit `docker-compose.yml` or set environment variables:

```yaml
environment:
  - CAN_ENABLED=true
  - CAN_CHANNEL=slcan0
  - CAN_BUSTYPE=socketcan
  - CAN_BITRATE=250000
  - CAN_SOURCE_ADDRESS=42
```

#### Docker Device Access

Uncomment in `docker-compose.yml`:

```yaml
devices:
  - /dev/ttyUSB0:/dev/ttyUSB0
network_mode: host  # Required for SocketCAN access
cap_add:
  - NET_ADMIN       # Required for network interface management
```

#### API Configuration

Enable CAN at runtime via API:

```bash
# Enable CAN bus
curl -X POST http://localhost:9001/api/can/enable

# Configure CAN parameters
curl -X POST http://localhost:9001/api/can/config \
  -H "Content-Type: application/json" \
  -d '{
    "channel": "slcan0",
    "bustype": "socketcan",
    "bitrate": 250000,
    "source_address": 42
  }'

# Check CAN status
curl http://localhost:9001/api/status | jq '.can'
```

### 5. Verify CAN Output

#### Using candump

Monitor CAN traffic:

```bash
# Watch all CAN frames
candump slcan0

# Filter for PGN 129029 (GNSS Position Data)
candump slcan0 | grep "129029"

# Show with timestamps and decode
candump -ta -d slcan0
```

#### Using cansniffer

Interactive CAN monitor:

```bash
cansniffer slcan0
```

#### Expected Output

When simulator is running, you should see:

```
slcan0  09F8012A   [8]  20 33 00 08 FF 05 F8 01    # TP.CM BAM
slcan0  09EB012A   [8]  01 FA 00 60 D4 0B 00 00    # TP.DT packet 1
slcan0  09EB012A   [8]  02 7A 42 01 00 80 7C F4    # TP.DT packet 2
slcan0  09EB012A   [8]  03 CA FE 00 00 00 00 00    # TP.DT packet 3
...
```

## Configuration Reference

### Environment Variables

| Variable | Default | Description |
|----------|---------|-------------|
| `CAN_ENABLED` | `false` | Enable CAN bus output |
| `CAN_CHANNEL` | `slcan0` | CAN interface name |
| `CAN_BUSTYPE` | `socketcan` | python-can bus type |
| `CAN_BITRATE` | `250000` | CAN bitrate (NMEA 2000 standard) |
| `CAN_SOURCE_ADDRESS` | `42` | Source address on N2K network (0-251) |

### Supported Bus Types

| Bus Type | Description | Use Case |
|----------|-------------|----------|
| `socketcan` | Linux SocketCAN | Native Linux CAN interface |
| `slcan` | Serial Line CAN | USB serial CAN adapters |
| `pcan` | PEAK PCAN | PEAK USB/PCI interfaces |
| `kvaser` | Kvaser | Kvaser USB/PCIe interfaces |

### SLCAN Speed Codes

| Code | Bitrate | NMEA 2000 |
|------|---------|-----------|
| S0 | 10 kbps | No |
| S1 | 20 kbps | No |
| S2 | 50 kbps | No |
| S3 | 100 kbps | No |
| S4 | 125 kbps | No |
| **S5** | **250 kbps** | **Yes** |
| S6 | 500 kbps | No |
| S7 | 800 kbps | No |
| S8 | 1 Mbps | No |

## API Reference

### Enable CAN Bus

```bash
POST /api/can/enable
```

Response:
```json
{
  "status": "success",
  "can": {
    "enabled": true,
    "connected": true,
    "channel": "slcan0",
    "bustype": "socketcan",
    "bitrate": 250000,
    "source_address": 42,
    "python_can_available": true
  }
}
```

### Disable CAN Bus

```bash
POST /api/can/disable
```

### Configure CAN Parameters

```bash
POST /api/can/config
Content-Type: application/json

{
  "channel": "slcan0",
  "bustype": "socketcan",
  "bitrate": 250000,
  "source_address": 42
}
```

### Get Status (includes CAN info)

```bash
GET /api/status
```

## Troubleshooting

### Issue: "python-can not installed"

**Solution:**
```bash
pip install python-can>=4.0.0
```

### Issue: "Failed to connect to CAN bus"

**Cause:** SLCAN interface not configured

**Solution:**
```bash
# Verify interface exists
ip link show slcan0

# If not, run setup
sudo slcand -o -s5 -t hw -S 1000000 /dev/ttyUSB0 slcan0
sudo ip link set slcan0 up
```

### Issue: "Permission denied" on /dev/ttyUSB0

**Solution:**
```bash
# Add user to dialout group
sudo usermod -aG dialout $USER

# OR set permissions (temporary)
sudo chmod 666 /dev/ttyUSB0
```

### Issue: Docker can't access CAN interface

**Solution:**

1. Use `network_mode: host` in docker-compose.yml
2. Add `NET_ADMIN` capability
3. Ensure SLCAN setup runs on host, not in container

```yaml
network_mode: host
cap_add:
  - NET_ADMIN
```

### Issue: No frames visible with candump

**Check:**
```bash
# Verify simulator is running
curl http://localhost:9001/api/status | jq '.running'

# Check CAN status
curl http://localhost:9001/api/status | jq '.can'

# Verify interface is UP
ip link show slcan0 | grep UP

# Check for errors
dmesg | grep slcan
```

### Issue: Frames visible but garbled

**Cause:** Bitrate mismatch

**Solution:**
```bash
# Ensure 250kbps (S5) for NMEA 2000
sudo ip link set slcan0 down
sudo slcand -o -s5 -t hw -S 1000000 /dev/ttyUSB0 slcan0
sudo ip link set slcan0 up
```

## Testing

### End-to-End Test

1. Start N2K Simulator with CAN enabled
2. Start simulator via API
3. Monitor CAN traffic

```bash
# Terminal 1: Start simulator
docker-compose up n2k-simulator

# Terminal 2: Enable CAN
curl -X POST http://localhost:9001/api/can/enable

# Terminal 3: Start broadcasting
curl -X POST http://localhost:9001/api/start

# Terminal 4: Monitor CAN
candump -ta slcan0
```

You should see PGN 129029 messages appearing at the configured update rate (default 2 Hz).

### Decode PGN Messages

Use `candump` with analyzer:

```bash
candump -ta -d slcan0 | grep "F801"
```

Or use NMEA 2000 analysis tools like:
- **Actisense NMEA Reader**
- **CANboat analyzer**
- **Yacht Devices YDWG-02 gateway**

## Integration Examples

### Connect to Actual NMEA 2000 Network

```
Computer → USB CAN Adapter → NMEA 2000 Network → Marine Devices
           (CH340)            (12V backbone)      (MFD, autopilot, etc.)
```

**Important:** NMEA 2000 uses 12V power. Ensure proper isolation or use a gateway device.

### Virtual Testing Network

```
N2K Simulator → slcan0 → candump (monitoring)
              ↓
              → OpenCPN (via TCP/UDP, not CAN)
              ↓
              → Anchor Alarm Simulator (via TCP)
```

## References

- [NMEA 2000 Standard](https://www.nmea.org/content/STANDARDS/NMEA_2000)
- [python-can Documentation](https://python-can.readthedocs.io/)
- [SocketCAN Documentation](https://www.kernel.org/doc/html/latest/networking/can.html)
- [CANboat Project](https://github.com/canboat/canboat)
- [PGN 129029 Specification](https://www.nmea.org/Assets/20140710%20nmea-2000-corrigendum-tc201401031%20pgn%20126208.pdf)

## Support

For issues or questions:
- **GitHub Issues:** https://github.com/yourusername/anchor-drag-alarm/issues
- **Email:** colin@bitterfield.com
- **Documentation:** See `CLAUDE.md` for project overview
