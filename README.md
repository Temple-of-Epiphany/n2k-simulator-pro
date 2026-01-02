# N2K Simulator Pro

**Version:** 1.0.0
**Author:** Colin Bitterfield
**Email:** colin@bitterfield.com
**Date Created:** 2025-01-02
**Date Updated:** 2025-01-02

Professional NMEA 2000 (N2K) GPS and navigation data simulator for marine electronics testing and development.

## Overview

N2K Simulator Pro generates realistic NMEA 2000 PGN messages and NMEA 0183 sentences for testing marine navigation equipment, chart plotters, and autopilot systems. It supports multiple output methods including UDP/TCP networking, CAN bus, and RS485 serial connections.

## Features

### NMEA 2000 Support
- **PGN 129029** - GNSS Position Data (GPS)
- **PGN 127250** - Vessel Heading (Magnetic/True Compass)
- **PGN 60928** - ISO Address Claim (Device Identification)
- Configurable manufacturer ID and device information
- Default configuration: Garmin GPS (Manufacturer Code 172)

### Simulation Modes
- **Stationary** - GPS position with realistic jitter
- **Drift** - Linear movement at specified speed/course (simulates anchor drag)
- **Circle** - Circular motion around anchor point (simulates swing at anchor)
- **Anchoring** - Realistic anchor drop with pendulum physics and wind effects
- **Waypoint** - Follow GPX route files with configurable speed

### Output Methods
1. **UDP Broadcast** - NMEA 0183 on port 2000
2. **TCP Streaming** - NMEA 0183 on port 10110 (multiple clients supported)
3. **CAN Bus** - NMEA 2000 PGNs via SocketCAN or SLCAN
4. **RS485 Serial** - NMEA 0183 via USB-to-RS485 adapters

### Hardware Support
- **USB Serial Ports** - /dev/ttyUSB*, /dev/ttyACM*, /dev/tty.usbserial*
- **CAN Bus Adapters**:
  - CANable (SLCAN)
  - PEAK PCAN
  - Kvaser
  - Waveshare USB-CAN adapters
- **RS485 Adapters**:
  - Waveshare USB-to-RS485 converters
  - Generic FTDI/CH340-based adapters
  - Industrial RS485 interfaces

## Quick Start

### Prerequisites
- Docker and Docker Compose
- For CAN bus: Linux with SocketCAN or macOS with can-utils
- For RS485: USB-to-RS485 adapter (optional)

### Installation

```bash
# Clone the repository
git clone https://github.com/YOUR_ORG/n2k-simulator-pro.git
cd n2k-simulator-pro

# Start the simulator
docker-compose up -d

# Access web interface
open http://192.168.68.51:9001
```

### Basic Usage

```bash
# Start GPS broadcast
curl -X POST http://192.168.68.51:9001/api/start

# Set drift mode (simulates anchor drag)
curl -X POST http://192.168.68.51:9001/api/mode \
  -H "Content-Type: application/json" \
  -d '{"mode":"drift"}'

# Set wind conditions
curl -X POST http://192.168.68.51:9001/api/wind \
  -H "Content-Type: application/json" \
  -d '{"speed":10.0,"direction":180.0}'
```

## Hardware Configuration

### CAN Bus Setup (Linux)

```bash
# Initialize SocketCAN interface (Linux)
sudo ip link set can0 type can bitrate 250000
sudo ip link set can0 up

# For SLCAN adapters (CANable, etc.)
sudo slcand -o -s5 -t hw -S 1000000 /dev/ttyUSB0 slcan0
sudo ip link set slcan0 up

# Enable CAN in simulator
curl -X POST http://192.168.68.51:9001/api/can/enable
```

See [docs/can_bus_setup.md](docs/can_bus_setup.md) for complete CAN bus setup instructions.

### CAN Bus Setup (macOS)

See [docs/can_setup_macos.md](docs/can_setup_macos.md) for macOS-specific CAN adapter configuration.

### RS485 Serial Setup

```bash
# Identify USB-to-RS485 adapter
ls -la /dev/ttyUSB*

# Configure serial port in simulator
curl -X POST http://192.168.68.51:9001/api/serial/config \
  -H "Content-Type: application/json" \
  -d '{"port":"/dev/ttyUSB0","baudrate":4800}'

# Enable serial output
curl -X POST http://192.168.68.51:9001/api/serial/enable
```

### Waveshare Adapter Configuration

For Waveshare USB-CAN and USB-RS485 adapters:

```bash
# Waveshare USB-CAN typically appears as /dev/ttyACM0
# Waveshare USB-RS485 typically appears as /dev/ttyUSB0 or /dev/ttyACM0

# Check device permissions
ls -l /dev/ttyUSB0

# Add user to dialout group (Linux)
sudo usermod -a -G dialout $USER

# Or use udev rules for persistent permissions
# See docs/can_bus_setup.md for udev configuration
```

## Web Interface

Access the web UI at: http://192.168.68.51:9001

Features:
- Real-time GPS position map
- Simulation mode controls
- Wind and weather settings
- Anchor position tracking
- Device identification configuration
- Network status monitoring

## API Endpoints

### Simulator Control
- `POST /api/start` - Start GPS broadcast
- `POST /api/stop` - Stop GPS broadcast
- `POST /api/reset` - Reset to initial state

### Configuration
- `POST /api/mode` - Set simulation mode
- `POST /api/position` - Set GPS position
- `POST /api/wind` - Set wind conditions
- `POST /api/heading` - Configure compass heading

### Device Information
- `GET /api/device` - Get N2K device info
- `POST /api/device` - Update device info
- `GET /api/status` - Get simulator status

### Hardware Interfaces
- `POST /api/can/enable` - Enable CAN bus output
- `POST /api/can/disable` - Disable CAN bus output
- `POST /api/serial/enable` - Enable RS485 serial output
- `POST /api/serial/config` - Configure serial port

See API documentation in [docs/api.md](docs/api.md) (coming soon).

## Environment Variables

Key configuration options (see docker-compose.yml):

```bash
# Network binding
HOST=0.0.0.0
PORT=8081
UDP_PORT=2000
TCP_PORT=10110

# Starting position
START_LAT=30.0245
START_LON=-90.034533

# NMEA 2000 Device ID (Garmin GPS)
N2K_MANUFACTURER_CODE=172
N2K_UNIQUE_NUMBER=2025001
N2K_DEVICE_FUNCTION=140

# CAN bus
CAN_ENABLED=false
CAN_CHANNEL=slcan0
CAN_BUSTYPE=socketcan
CAN_BITRATE=250000

# Serial port (RS485)
SERIAL_ENABLED=false
SERIAL_PORT=/dev/ttyUSB0
SERIAL_BAUDRATE=4800
```

## Testing

### Test UDP Output

```bash
# Listen for UDP broadcasts
nc -u -l 2000
```

### Test TCP Stream

```bash
# Connect to TCP stream
nc 192.168.68.51 10110
```

### Test CAN Bus

```bash
# Monitor CAN traffic (Linux)
candump can0

# Or with SLCAN
candump slcan0
```

### Test RS485 Serial

```bash
# Monitor serial output
screen /dev/ttyUSB0 4800

# Or using minicom
minicom -D /dev/ttyUSB0 -b 4800
```

## Development

### Local Development (Python)

```bash
# Create virtual environment
python3 -m venv venv
source venv/bin/activate

# Install dependencies
cd n2k-simulator
pip install -r requirements.txt

# Run simulator
python main.py
```

### Building Docker Image

```bash
# Build image
docker-compose build

# Run with custom config
docker-compose up
```

## Troubleshooting

### CAN Bus Issues

**Problem**: CAN interface not found
```bash
# Check CAN interfaces
ip link show | grep can

# Check kernel modules (Linux)
lsmod | grep can
```

**Problem**: Permission denied on /dev/ttyUSB0
```bash
# Add user to dialout group
sudo usermod -a -G dialout $USER
# Log out and back in

# Or change device permissions (temporary)
sudo chmod 666 /dev/ttyUSB0
```

### Serial Port Issues

**Problem**: Device not found
```bash
# List all USB serial devices
ls -la /dev/tty* | grep -i usb

# Check dmesg for device attachment
dmesg | tail -20
```

**Problem**: Docker container can't access device
```bash
# Ensure device is mapped in docker-compose.yml
# Add to devices section:
#   - /dev/ttyUSB0:/dev/ttyUSB0
```

## Documentation

- [CAN Bus Setup Guide](docs/can_bus_setup.md)
- [macOS CAN Setup](docs/can_setup_macos.md)
- [N2K Compass and Device ID](docs/n2k_compass_and_device_id.md)

## License

Copyright (c) 2025 Colin Bitterfield

## Support

For issues and feature requests, please use GitHub Issues.

## Changelog

### Version 1.0.0 (2025-01-02)
- Initial release as standalone project
- NMEA 2000 PGN 129029 (GPS Position) support
- PGN 127250 (Vessel Heading) support
- PGN 60928 (ISO Address Claim) support
- UDP/TCP NMEA 0183 output
- CAN bus support (SocketCAN, SLCAN)
- RS485 serial port support
- Garmin GPS device identification
- Multiple simulation modes
- Web-based control interface
- Docker containerization
