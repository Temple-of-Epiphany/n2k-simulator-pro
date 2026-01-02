# N2K Compass and Device ID Configuration

**Author:** Colin Bitterfield
**Email:** colin@bitterfield.com
**Date:** 2025-12-22
**Version:** 0.1.0

## Overview

The N2K Simulator now supports:
1. **Compass/Heading Data** - PGN 127250 (Vessel Heading)
2. **Device Identification** - PGN 60928 (ISO Address Claim)
3. **Manufacturer Selection** - 30+ pre-configured marine manufacturers

## Compass/Heading (PGN 127250)

### Features

- Broadcasts vessel heading via CAN, UDP, TCP
- Supports both Magnetic and True heading
- Magnetic deviation and variation configurable
- Updates at 1 Hz (default)
- Automatically updates from course when in motion
- NMEA 0183 output: HDM (Magnetic) or HDT (True)

### API Endpoints

#### Get Current Heading

```bash
GET /api/heading

Response:
{
  "heading": 45.0,           # degrees (0-360)
  "heading_rad": 0.7854,     # radians
  "deviation": 0.0,          # magnetic deviation (degrees)
  "variation": 0.0,          # magnetic variation (degrees)
  "reference": "Magnetic",   # "Magnetic" or "True"
  "enabled": true,
  "update_rate": 1.0         # Hz
}
```

#### Set Heading

```bash
POST /api/heading
Content-Type: application/json

{
  "heading": 90.0,           # Set to 90° East
  "deviation": -2.5,         # Optional: deviation
  "variation": 10.0,         # Optional: variation
  "reference": "Magnetic",   # Optional: "Magnetic" or "True"
  "enabled": true            # Optional: enable/disable broadcasts
}
```

### Examples

```bash
# Set heading to North (0°)
curl -X POST http://localhost:9001/api/heading \
  -H "Content-Type: application/json" \
  -d '{"heading": 0}'

# Set to 180° South with variation
curl -X POST http://localhost:9001/api/heading \
  -H "Content-Type: application/json" \
  -d '{"heading": 180, "variation": -12.5, "reference": "Magnetic"}'

# Disable heading broadcasts
curl -X POST http://localhost:9001/api/heading \
  -H "Content-Type: application/json" \
  -d '{"enabled": false}'

# Check current heading
curl http://localhost:9001/api/heading | jq
```

### Environment Variables

```bash
# Enable/disable heading output
HEADING_ENABLED=true           # Default: true

# Heading starts at North (0°) and updates from course during motion
```

### NMEA 0183 Output

**Magnetic Heading (HDM):**
```
$HDGLM,090.0,M*3C
```

**True Heading (HDT):**
```
$HDT,090.0,T*2B
```

### CAN Bus Output

**PGN 127250 - Single Frame (8 bytes):**
- Priority: 2 (high priority for navigation)
- Update rate: 1 Hz
- Broadcast to all devices (destination 255)

Example frame:
```
CAN ID: 0x09F11200  (PGN 127250, SA=42)
Data: [00 90 D0 00 00 00 00 01]
      │  │  │  │  │  │  │  └─ Reference (01=Magnetic)
      │  │  └──┴──┴──┴─────── Variation (0.0 rad)
      │  └───────────────────── Deviation (0.0 rad)
      └──────────────────────── Heading (90° = 1.5708 rad)
```

## Device Identification (PGN 60928)

### Features

- NMEA 2000 ISO Address Claim
- Identifies simulator on N2K network
- Configurable manufacturer code
- Unique device serial number
- Device class and function codes

### API Endpoints

#### Get Device Info

```bash
GET /api/device

Response:
{
  "manufacturer_code": 2046,
  "manufacturer_name": "Test/Unknown",
  "unique_number": 123456,
  "device_function": 130,      # PC Gateway
  "device_class": 25,           # Network Device
  "device_instance": 0,
  "system_instance": 0,
  "industry_group": 4           # Marine
}
```

#### Set Device Info

```bash
POST /api/device
Content-Type: application/json

{
  "manufacturer_code": 172,    # Garmin
  "unique_number": 654321,
  "device_function": 130,
  "device_class": 25,
  "device_instance": 0
}
```

### Supported Manufacturers

| Code | Manufacturer | Common Devices |
|------|--------------|----------------|
| 8 | Actisense | NMEA 2000 gateways |
| 80 | Furuno | GPS, radar, autopilot |
| 135 | Airmar | Weather stations, depth |
| 137 | Lowrance | Fish finders, chartplotters |
| 140 | Raymarine | MFDs, autopilot |
| 144 | Mercury | Engine controllers |
| 165 | Kohler | Generators |
| 168 | Volvo Penta | Engine controllers |
| 172 | **Garmin** | GPS, chartplotters, MFDs |
| 198 | Maretron | Sensors, displays |
| 229 | B&G | Sailing instruments |
| 233 | Digital Yacht | NMEA gateways |
| 275 | Yanmar | Engine controllers |
| 304 | Simrad | MFDs, autopilot |
| 378 | Navico | Chartplotters, radar |
| 381 | Honda | Engine controllers |
| 419 | Victron | Power systems |
| 529 | Yacht Devices | NMEA converters |
| 717 | SailorHat | Marine gateways |
| 2046 | **Test/Unknown** | Testing devices |

### Examples

```bash
# Set as Garmin device
curl -X POST http://localhost:9001/api/device \
  -H "Content-Type: application/json" \
  -d '{"manufacturer_code": 172, "unique_number": 123456}'

# Set as Raymarine device
curl -X POST http://localhost:9001/api/device \
  -H "Content-Type: application/json" \
  -d '{"manufacturer_code": 140, "unique_number": 789012}'

# Check current device info
curl http://localhost:9001/api/device | jq
```

### Environment Variables

```bash
# Device identification
N2K_MANUFACTURER_CODE=2046     # Default: 2046 (Test/Unknown)
N2K_UNIQUE_NUMBER=123456       # Default: 123456
N2K_DEVICE_FUNCTION=130        # Default: 130 (PC Gateway)
N2K_DEVICE_CLASS=25            # Default: 25 (Network Device)
N2K_DEVICE_INSTANCE=0          # Default: 0
N2K_SYSTEM_INSTANCE=0          # Default: 0
N2K_INDUSTRY_GROUP=4           # Default: 4 (Marine)
```

### Device Function Codes

| Code | Function |
|------|----------|
| 130 | PC Gateway |
| 140 | Navigation |
| 150 | Engine Gateway |
| 160 | Proprietary |

### Device Class Codes

| Code | Class |
|------|-------|
| 10 | System Tools |
| 20 | Safety Systems |
| 25 | Inter/Intranetwork Device |
| 30 | Navigation |
| 40 | Communication |
| 50 | Instrumentation |
| 60 | Control |

## Complete Example

### Setup Simulator as Garmin GPS

```bash
# 1. Configure device as Garmin
curl -X POST http://localhost:9001/api/device \
  -H "Content-Type: application/json" \
  -d '{
    "manufacturer_code": 172,
    "unique_number": 987654,
    "device_function": 140,
    "device_class": 30
  }'

# 2. Set initial heading
curl -X POST http://localhost:9001/api/heading \
  -H "Content-Type: application/json" \
  -d '{
    "heading": 45.0,
    "deviation": -1.5,
    "variation": 12.0,
    "reference": "Magnetic"
  }'

# 3. Start simulator
curl -X POST http://localhost:9001/api/start

# 4. Monitor CAN output
python3 scripts/monitor_can.py
```

You should see:
- PGN 129029 (GPS) every 0.5 seconds
- PGN 127250 (Heading) every 1.0 second
- PGN 60928 (Address Claim) on startup

## Integration with CAN Bus

When CAN is enabled, the simulator broadcasts:

1. **PGN 60928** - Once on startup (Address Claim)
2. **PGN 129029** - GPS position (2 Hz, multi-frame via TP)
3. **PGN 127250** - Vessel heading (1 Hz, single frame)

All messages use the configured manufacturer code and source address.

## Testing

```bash
# Test complete setup
make n2k-can-daemon-mac
curl -X POST http://localhost:9001/api/device -d '{"manufacturer_code": 172}'
curl -X POST http://localhost:9001/api/heading -d '{"heading": 90}'
curl -X POST http://localhost:9001/api/start

# Monitor output
python3 scripts/monitor_can.py

# Check status
curl http://localhost:9001/api/status | jq '{running, heading: .heading, device: .device}'
```

## Notes

- **Heading** automatically updates from `course_over_ground` when vessel is in motion (speed > 0.1 m/s)
- **Static heading** is used when stationary or manually set via API
- **Deviation** and **variation** are optional corrections
- **Reference** determines if heading is True or Magnetic
- **Device info** should match your intended N2K device type for compatibility

## References

- [PGN 127250 Specification](https://www.nmea.org/)
- [PGN 60928 ISO Address Claim](https://www.nmea.org/)
- [NMEA 2000 Manufacturer Codes](https://www.nmea.org/Assets/2000-manufacturer-codes-april-2021.pdf)
