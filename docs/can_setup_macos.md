# CAN Bus Setup - macOS Guide

**Author:** Colin Bitterfield
**Email:** colin@bitterfield.com
**Date Created:** 2025-12-22
**Date Updated:** 2025-12-22
**Version:** 0.1.0

## Quick Start for macOS

Your USB CAN adapter: `/dev/cu.usbserial-1120`

### Step 1: Verify Device

```bash
# Check that device exists
ls -l /dev/cu.usbserial-1120

# Should show something like:
# crw-rw-rw-  1 root  wheel   18,  10 Dec 22 10:30 /dev/cu.usbserial-1120
```

### Step 2: Set Permissions (if needed)

```bash
sudo chmod 666 /dev/cu.usbserial-1120
```

### Step 3: Start with Docker (Recommended)

```bash
# Build the image
docker-compose build n2k-simulator

# Start with macOS configuration
docker-compose -f docker-compose.yml -f docker-compose.mac.yml up n2k-simulator

# Or in detached mode
docker-compose -f docker-compose.yml -f docker-compose.mac.yml up -d n2k-simulator
```

### Step 4: Enable CAN Output

```bash
# Check status
curl http://localhost:9001/api/status | jq '.can'

# Enable CAN
curl -X POST http://localhost:9001/api/can/enable

# Start simulator
curl -X POST http://localhost:9001/api/start
```

### Step 5: Test CAN Output

Since macOS doesn't have native SocketCAN tools, you have a few options:

#### Option A: Monitor via Python Script

Create `test_can_receive.py`:

```python
#!/usr/bin/env python3
import can
import sys

try:
    bus = can.interface.Bus(
        channel='/dev/cu.usbserial-1120',
        bustype='slcan',
        bitrate=250000
    )
    print("Listening for CAN messages... (Ctrl-C to stop)")

    for msg in bus:
        print(f"ID: 0x{msg.arbitration_id:08X}  Data: {msg.data.hex()}")

except KeyboardInterrupt:
    print("\nStopped")
except Exception as e:
    print(f"Error: {e}")
    sys.exit(1)
```

Run it:
```bash
pip3 install python-can
python3 test_can_receive.py
```

#### Option B: Use can-utils in Docker

```bash
# Run candump in a separate container with device access
docker run --rm --device=/dev/cu.usbserial-1120:/dev/ttyUSB0 \
  python:3.11-slim bash -c \
  "pip install python-can && python -c \"
import can
bus = can.interface.Bus(channel='/dev/ttyUSB0', bustype='slcan', bitrate=250000)
for msg in bus:
    print(f'0x{msg.arbitration_id:08X} [{msg.dlc}] {msg.data.hex()}')
\""
```

#### Option C: Use a Linux VM

For full SocketCAN support with candump:

1. Install UTM or VirtualBox
2. Create Ubuntu VM
3. Enable USB passthrough for the CAN adapter
4. Follow Linux setup from `docs/can_bus_setup.md`

## Configuration Files

### docker-compose.mac.yml

This file overrides the default docker-compose.yml for macOS:

```yaml
services:
  n2k-simulator:
    devices:
      - /dev/cu.usbserial-1120:/dev/ttyUSB0
    environment:
      - CAN_ENABLED=true
      - CAN_CHANNEL=/dev/ttyUSB0
      - CAN_BUSTYPE=slcan
      - CAN_BITRATE=250000
    privileged: true
```

### Update Device Path

If your device path is different:

```bash
# Edit docker-compose.mac.yml
nano docker-compose.mac.yml

# Change this line:
- /dev/cu.usbserial-1120:/dev/ttyUSB0

# To your device:
- /dev/cu.usbserial-XXXX:/dev/ttyUSB0
```

## Makefile Integration

Add macOS-specific targets to Makefile:

```makefile
# macOS CAN-enabled N2K Simulator
n2k-can-run-mac:
	docker-compose -f docker-compose.yml -f docker-compose.mac.yml up n2k-simulator

n2k-can-daemon-mac:
	docker-compose -f docker-compose.yml -f docker-compose.mac.yml up -d n2k-simulator

n2k-can-stop-mac:
	docker-compose -f docker-compose.yml -f docker-compose.mac.yml down
```

Usage:
```bash
make n2k-can-run-mac
```

## Troubleshooting

### Issue: Device not found

```bash
# List all USB serial devices
ls -l /dev/{tty,cu}.{usbserial,wchusbserial}*

# Check USB devices
system_profiler SPUSBDataType | grep -A 10 "1a86"
```

### Issue: Permission denied

```bash
# Fix permissions
sudo chmod 666 /dev/cu.usbserial-1120

# Or add yourself to the correct group
sudo dseditgroup -o edit -a $(whoami) -t user _dialout
```

### Issue: Docker can't access device

```bash
# Verify Docker has access
docker run --rm --device=/dev/cu.usbserial-1120:/dev/ttyUSB0 \
  alpine ls -l /dev/ttyUSB0

# Should show the device exists in container
```

### Issue: CAN messages not appearing

1. Check device path is correct:
   ```bash
   ls -l /dev/cu.usbserial-1120
   ```

2. Check container logs:
   ```bash
   docker-compose logs n2k-simulator
   ```

3. Check CAN status via API:
   ```bash
   curl http://localhost:9001/api/status | jq '.can'
   ```

4. Verify simulator is running:
   ```bash
   curl http://localhost:9001/api/status | jq '.running'
   ```

### Issue: SLCAN not working

The CH340 adapter needs proper initialization. Try:

```python
# test_slcan.py
import serial
import time

ser = serial.Serial('/dev/cu.usbserial-1120', 115200, timeout=1)

# Send SLCAN initialization commands
ser.write(b'C\r')  # Close channel
time.sleep(0.1)
ser.write(b'S5\r')  # Set 250kbps
time.sleep(0.1)
ser.write(b'O\r')  # Open channel
time.sleep(0.1)

print("SLCAN initialized")
ser.close()
```

Run before starting Docker:
```bash
python3 test_slcan.py
```

## Alternative: Run Without Docker

For testing without Docker:

```bash
# Install dependencies
cd n2k-simulator
pip3 install -r requirements.txt

# Set environment
export CAN_ENABLED=true
export CAN_CHANNEL=/dev/cu.usbserial-1120
export CAN_BUSTYPE=slcan
export CAN_BITRATE=250000

# Run directly
python3 main.py
```

## Expected Output

When working correctly:

### Console Output
```
INFO:can_interface:CAN bus connected: slcan on /dev/ttyUSB0 @ 250000bps
INFO:main:CAN bus initialized: slcan on /dev/ttyUSB0
INFO:main:Simulator auto-started
```

### API Status
```json
{
  "can": {
    "enabled": true,
    "connected": true,
    "channel": "/dev/ttyUSB0",
    "bustype": "slcan",
    "bitrate": 250000,
    "source_address": 42,
    "python_can_available": true
  }
}
```

### CAN Monitor Output
```
0x09F8012A [8] 20330008FF05F801    # TP.CM BAM
0x09EB012A [8] 01FA0060D40B0000    # TP.DT frame 1
0x09EB012A [8] 027A420100807CF4    # TP.DT frame 2
0x09EB012A [8] 03CAFE0000000000    # TP.DT frame 3
...
```

## Testing Checklist

- [ ] Device shows up in `/dev/cu.usbserial-1120`
- [ ] Device has read/write permissions
- [ ] Docker can access the device
- [ ] Container starts without errors
- [ ] CAN status shows `connected: true`
- [ ] Simulator is running
- [ ] CAN messages are being transmitted (verified via monitoring)

## Resources

- Main CAN Setup Guide: `docs/can_bus_setup.md`
- python-can Documentation: https://python-can.readthedocs.io/
- CH340 Driver: https://github.com/adrianmihalko/ch340g-ch34g-ch34x-mac-os-x-driver

## Support

For issues specific to macOS CAN setup:
- GitHub Issues: https://github.com/yourusername/anchor-drag-alarm/issues
- Email: colin@bitterfield.com
