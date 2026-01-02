"""
N2K Data Simulator - Main Application

Author: Colin Bitterfield
Email: colin@bitterfield.com
Date Created: 2025-11-30
Date Updated: 2025-11-30
Version: 0.1.0

Simulates NMEA 2000 GPS data (PGN 129029) for marine electronics testing.
Provides UDP/TCP output and web interface for control.
"""

import os
import sys
import time
import math
import socket
import threading
import logging
import uuid
import random
import xml.etree.ElementTree as ET
from collections import deque
from flask import Flask, render_template, jsonify, request
from flask_socketio import SocketIO, emit
from werkzeug.utils import secure_filename

from pgn_messages import (GPSPosition, PGN129029, VesselHeading, PGN127250,
                          DeviceInfo, PGN60928, calculate_new_position)
from can_interface import NMEA2000CANInterface

# Configure logging based on DEBUG env var
debug_mode = os.environ.get('DEBUG', 'false').lower() == 'true'
logging.basicConfig(
    level=logging.DEBUG if debug_mode else logging.INFO,
    format='%(levelname)s:%(name)s:%(message)s'
)
logger = logging.getLogger(__name__)

app = Flask(__name__)
app.config['SECRET_KEY'] = os.environ.get('SECRET_KEY', 'n2k-simulator-dev-key')
socketio = SocketIO(app, cors_allowed_origins="*", async_mode='eventlet')

# Global state
class SimulatorState:
    def __init__(self):
        self.running = False
        # Store initial position for reset functionality
        self.initial_lat = float(os.environ.get('START_LAT', 37.7749))
        self.initial_lon = float(os.environ.get('START_LON', -122.4194))
        self.position = GPSPosition(
            latitude=self.initial_lat,
            longitude=self.initial_lon,
            altitude=0.0,
            speed_over_ground=0.0,
            course_over_ground=0.0
        )
        # Vessel heading (compass)
        self.heading = VesselHeading(
            heading=0.0,  # North
            deviation=0.0,
            variation=0.0,
            reference='Magnetic'
        )
        self.heading_enabled = os.environ.get('HEADING_ENABLED', 'true').lower() == 'true'
        self.heading_update_rate = 1.0  # Hz (1x per second for heading)

        # Device identification
        self.device_info = DeviceInfo(
            manufacturer_code=int(os.environ.get('N2K_MANUFACTURER_CODE', 2046)),  # 2046 = Test/Unknown
            unique_number=int(os.environ.get('N2K_UNIQUE_NUMBER', 123456)),
            device_function=int(os.environ.get('N2K_DEVICE_FUNCTION', 130)),  # 130 = PC Gateway
            device_class=int(os.environ.get('N2K_DEVICE_CLASS', 25)),  # 25 = Network Device
            device_instance=int(os.environ.get('N2K_DEVICE_INSTANCE', 0)),
            system_instance=int(os.environ.get('N2K_SYSTEM_INSTANCE', 0)),
            industry_group=int(os.environ.get('N2K_INDUSTRY_GROUP', 4))  # 4 = Marine
        )
        self.mode = 'stationary'  # stationary, drift, circle, waypoint
        self.drift_speed = 0.5  # m/s
        self.drift_course = 0.0  # radians
        self.manual_drift_speed = 0.5  # knots (manual drift when no wind)
        self.manual_drift_direction = 0.0  # degrees (manual drift direction)
        self.circle_radius = 500.0  # meters (large circle for visibility)
        self.circle_center_lat = self.position.latitude
        self.circle_center_lon = self.position.longitude
        self.update_rate = 2.0  # Hz (default 2x/second)
        self.sid = 0  # GPS sequence ID
        self.heading_sid = 0  # Heading sequence ID
        self.last_heading_update = 0.0  # Last heading update timestamp

        # Wind conditions
        self.wind_speed = 0.0  # knots
        self.wind_direction = 0.0  # degrees
        self.variable_wind = False
        self.variable_wind_base_speed = 0.0
        self.variable_wind_base_direction = 0.0
        self.variable_wind_last_update = 0.0

        # Anchoring state
        self.anchor_rode_distance = 50.0  # meters (scope length)
        self.anchor_position = None  # (lat, lon) where anchor is dropped
        self.anchor_drop_phase = 'idle'  # idle, dropping, swinging
        self.anchor_drop_start_time = 0.0
        self.anchor_swing_angle = 0.0  # radians - current swing position
        self.anchor_swing_velocity = 0.0  # radians/sec - angular velocity
        self.anchor_trail = deque(maxlen=200)  # Circular buffer for visualization
        self.saved_anchor_position = None  # Store original anchor when switching to drift

        # Route management
        self.routes = {}  # {route_id: route_data}
        self.active_route = None
        self.current_waypoint_idx = 0
        self.route_speed = 5.0  # m/s

        # Network outputs
        self.udp_host = os.environ.get('UDP_HOST', '0.0.0.0')
        self.udp_port = int(os.environ.get('UDP_PORT', 2000))
        self.tcp_host = os.environ.get('TCP_HOST', '0.0.0.0')
        self.tcp_port = int(os.environ.get('TCP_PORT', 10110))

        self.udp_socket = None
        self.tcp_socket = None
        self.tcp_clients = []

        # CAN bus configuration
        self.can_enabled = os.environ.get('CAN_ENABLED', 'false').lower() == 'true'
        self.can_channel = os.environ.get('CAN_CHANNEL', 'slcan0')
        self.can_bustype = os.environ.get('CAN_BUSTYPE', 'socketcan')
        self.can_bitrate = int(os.environ.get('CAN_BITRATE', 250000))
        self.can_source_address = int(os.environ.get('CAN_SOURCE_ADDRESS', 42))
        self.can_interface = None

        # Serial port configuration (RS485/RS232 for NMEA 0183)
        self.serial_enabled = os.environ.get('SERIAL_ENABLED', 'false').lower() == 'true'
        self.serial_port = os.environ.get('SERIAL_PORT', '/dev/ttyUSB1')
        self.serial_baudrate = int(os.environ.get('SERIAL_BAUDRATE', 4800))
        self.serial_interface = None

        # Simulator thread management
        self.simulator_thread = None

state = SimulatorState()

# Global hardware state (receives data from LVGL UI via hardware_sim)
class HardwareState:
    def __init__(self):
        self.buzzer_active = False
        self.relay_active = False
        self.led_state = 0  # 0=OFF, 1=RED, 2=YELLOW, 3=GREEN, 4=BLINKING_RED
        self.alarm_active = False
        self.anchor_alert_active = False
        self.mute_active = False

hardware_state = HardwareState()


def init_network():
    """Initialize UDP and TCP sockets"""
    global state

    # Skip if running in Flask reloader subprocess
    # Only run in main process (werkzeug sets WERKZEUG_RUN_MAIN in reloader)
    import os
    if os.environ.get('WERKZEUG_RUN_MAIN') == 'true':
        logger.debug("Running in Flask reloader subprocess, skipping network init")
        return

    # Skip if already initialized (Flask reloader causes duplicate calls)
    if state.udp_socket is not None or state.tcp_socket is not None:
        logger.debug("Network already initialized, skipping")
        return

    # UDP socket for broadcasting
    try:
        state.udp_socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        state.udp_socket.setsockopt(socket.SOL_SOCKET, socket.SO_BROADCAST, 1)
        logger.info(f"UDP socket ready on port {state.udp_port}")
    except Exception as e:
        logger.error(f"Failed to create UDP socket: {e}")

    # TCP socket for streaming
    try:
        state.tcp_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        state.tcp_socket.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        state.tcp_socket.bind((state.tcp_host, state.tcp_port))
        state.tcp_socket.listen(5)
        state.tcp_socket.settimeout(1.0)
        logger.info(f"TCP server listening on {state.tcp_host}:{state.tcp_port}")

        # Start TCP accept thread
        threading.Thread(target=tcp_accept_loop, daemon=True).start()
        logger.debug("TCP accept loop thread started")
    except Exception as e:
        logger.error(f"Failed to create TCP socket: {e}")

    # CAN bus interface
    if state.can_enabled:
        try:
            state.can_interface = NMEA2000CANInterface(
                channel=state.can_channel,
                bustype=state.can_bustype,
                bitrate=state.can_bitrate,
                source_address=state.can_source_address,
                enabled=True
            )
            if state.can_interface.enabled:
                logger.info(f"CAN bus initialized: {state.can_bustype} on {state.can_channel}")
            else:
                logger.warning("CAN bus initialization failed")
        except Exception as e:
            logger.error(f"Failed to initialize CAN interface: {e}")
            state.can_enabled = False

    # Serial port interface (RS485/RS232 for NMEA 0183)
    if state.serial_enabled:
        try:
            import serial
            state.serial_interface = serial.Serial(
                port=state.serial_port,
                baudrate=state.serial_baudrate,
                bytesize=serial.EIGHTBITS,
                parity=serial.PARITY_NONE,
                stopbits=serial.STOPBITS_ONE,
                timeout=1
            )
            logger.info(f"Serial port initialized: {state.serial_port} @ {state.serial_baudrate} bps")
        except Exception as e:
            logger.error(f"Failed to initialize serial port: {e}")
            state.serial_enabled = False


def tcp_accept_loop():
    """Accept incoming TCP connections"""
    # Accept connections regardless of simulator running state
    # Connections are queued, but data only sent when simulator is running
    while True:
        try:
            client_sock, addr = state.tcp_socket.accept()
            logger.info(f"TCP client connected from {addr}")
            state.tcp_clients.append(client_sock)
            logger.info(f"Client added. Total TCP clients: {len(state.tcp_clients)}")
        except socket.timeout:
            continue
        except Exception as e:
            logger.error(f"TCP accept error: {e}")
            time.sleep(1)


def broadcast_position(position: GPSPosition):
    """Broadcast position via UDP, TCP, and CAN"""
    # Encode PGN 129029
    pgn_data = PGN129029.encode(position, state.sid)
    state.sid = (state.sid + 1) % 256

    # Also generate NMEA 0183 for compatibility
    nmea_sentence = PGN129029.to_nmea0183(position)

    # Debug logging
    logger.debug(f"Broadcasting: {nmea_sentence.strip()}")
    logger.debug(f"TCP clients: {len(state.tcp_clients)}")

    # UDP broadcast
    if state.udp_socket:
        try:
            # Broadcast NMEA 0183
            state.udp_socket.sendto(
                nmea_sentence.encode(),
                ('<broadcast>', state.udp_port)
            )
            logger.debug(f"UDP broadcast sent")
        except Exception as e:
            logger.error(f"UDP send error: {e}")

    # TCP send to all clients
    dead_clients = []
    for client in state.tcp_clients:
        try:
            client.sendall(nmea_sentence.encode())
            logger.debug(f"TCP sent to client {client.getpeername()}")
        except Exception as e:
            logger.error(f"TCP send error to {client}: {type(e).__name__}: {e}")
            dead_clients.append(client)

    # Remove dead clients
    for client in dead_clients:
        state.tcp_clients.remove(client)
        try:
            client.close()
        except:
            pass

    # CAN bus broadcast
    if state.can_enabled and state.can_interface:
        try:
            state.can_interface.send_pgn_129029(pgn_data)
            logger.debug(f"CAN broadcast sent: PGN 129029, {len(pgn_data)} bytes")
        except Exception as e:
            logger.error(f"CAN send error: {e}")

    # Serial port broadcast (RS485/RS232)
    if state.serial_enabled and state.serial_interface:
        try:
            state.serial_interface.write(nmea_sentence.encode())
            logger.debug(f"Serial broadcast sent: {len(nmea_sentence)} bytes")
        except Exception as e:
            logger.error(f"Serial send error: {e}")


def broadcast_heading(heading: VesselHeading):
    """Broadcast vessel heading via UDP, TCP, and CAN"""
    # Encode PGN 127250
    pgn_data = PGN127250.encode(heading, state.heading_sid)
    state.heading_sid = (state.heading_sid + 1) % 256

    # Also generate NMEA 0183 for compatibility
    nmea_sentence = PGN127250.to_nmea0183(heading)

    logger.debug(f"Broadcasting heading: {nmea_sentence.strip()}")

    # UDP broadcast
    if state.udp_socket:
        try:
            state.udp_socket.sendto(
                nmea_sentence.encode(),
                ('<broadcast>', state.udp_port)
            )
        except Exception as e:
            logger.error(f"UDP send error (heading): {e}")

    # TCP send to all clients
    dead_clients = []
    for client in state.tcp_clients:
        try:
            client.sendall(nmea_sentence.encode())
        except Exception as e:
            logger.error(f"TCP send error (heading): {e}")
            dead_clients.append(client)

    # Remove dead clients
    for client in dead_clients:
        if client in state.tcp_clients:
            state.tcp_clients.remove(client)
            try:
                client.close()
            except:
                pass

    # CAN bus broadcast (PGN 127250 is 8 bytes - single frame)
    if state.can_enabled and state.can_interface:
        try:
            from can_interface import N2KMessage
            msg = N2KMessage(
                pgn=127250,
                priority=2,  # High priority for navigation data
                source=state.can_source_address,
                destination=255,  # Broadcast
                data=pgn_data
            )
            state.can_interface.send_message(msg)
            logger.debug(f"CAN broadcast sent: PGN 127250, {len(pgn_data)} bytes")
        except Exception as e:
            logger.error(f"CAN send error (heading): {e}")

    # Serial port broadcast (RS485/RS232)
    if state.serial_enabled and state.serial_interface:
        try:
            state.serial_interface.write(nmea_sentence.encode())
            logger.debug(f"Serial broadcast sent (heading): {len(nmea_sentence)} bytes")
        except Exception as e:
            logger.error(f"Serial send error (heading): {e}")


def update_position():
    """Update position based on current mode"""
    current_time = time.time()
    time_delta = 1.0 / state.update_rate

    if state.mode == 'stationary':
        # Add small random jitter to simulate GPS noise
        state.position.latitude += (hash(current_time) % 1000 - 500) * 1e-8
        state.position.longitude += (hash(current_time * 2) % 1000 - 500) * 1e-8

    elif state.mode == 'drift':
        # Linear drift - wind-driven if wind present, manual otherwise
        if state.wind_speed > 0:
            # Wind-driven drift: drift in direction wind is blowing TO (downwind)
            # Wind direction is where wind comes FROM, add 180° for downwind drift
            drift_direction_rad = math.radians(state.wind_direction) + math.pi
            # Anchor drag rate: 1m per 10 minutes = 0.00324 knots
            drift_speed = 0.00324  # knots (realistic anchor drag speed)
            logger.debug(f"Wind-driven drift: {drift_speed:.5f} kts at {math.degrees(drift_direction_rad):.0f}°")
        else:
            # Manual drift: use user-specified direction and speed
            drift_direction_rad = math.radians(state.manual_drift_direction)
            drift_speed = state.manual_drift_speed  # knots
            logger.debug(f"Manual drift: {drift_speed:.1f} kts at {math.degrees(drift_direction_rad):.0f}°")

        # Calculate new position
        new_lat, new_lon = calculate_new_position(
            state.position.latitude,
            state.position.longitude,
            drift_speed,
            drift_direction_rad,
            time_delta
        )
        state.position.latitude = new_lat
        state.position.longitude = new_lon
        state.position.speed_over_ground = drift_speed
        state.position.course_over_ground = drift_direction_rad

    elif state.mode == 'circle':
        # Circular motion around anchor point
        # Calculate angle based on time
        angular_velocity = state.drift_speed / state.circle_radius  # rad/s
        angle = (current_time * angular_velocity) % (2 * math.pi)

        # Calculate position on circle
        lat_offset = (state.circle_radius * math.cos(angle)) / 111320.0  # ~111km per degree
        lon_offset = (state.circle_radius * math.sin(angle)) / (111320.0 * math.cos(math.radians(state.circle_center_lat)))

        state.position.latitude = state.circle_center_lat + lat_offset
        state.position.longitude = state.circle_center_lon + lon_offset
        state.position.speed_over_ground = state.drift_speed
        state.position.course_over_ground = angle + math.pi / 2  # Tangent to circle

    elif state.mode == 'waypoint' and state.active_route:
        # Follow waypoints in route
        waypoints = state.active_route['waypoints']

        if state.current_waypoint_idx < len(waypoints):
            target_wp = waypoints[state.current_waypoint_idx]
            target_lat = target_wp['lat']
            target_lon = target_wp['lon']

            # Calculate distance and bearing to target waypoint
            dlat = math.radians(target_lat - state.position.latitude)
            dlon = math.radians(target_lon - state.position.longitude)

            # Haversine distance
            a = math.sin(dlat/2)**2 + math.cos(math.radians(state.position.latitude)) * \
                math.cos(math.radians(target_lat)) * math.sin(dlon/2)**2
            c = 2 * math.atan2(math.sqrt(a), math.sqrt(1-a))
            distance = 6371000 * c  # meters

            # Bearing
            y = math.sin(dlon) * math.cos(math.radians(target_lat))
            x = math.cos(math.radians(state.position.latitude)) * math.sin(math.radians(target_lat)) - \
                math.sin(math.radians(state.position.latitude)) * math.cos(math.radians(target_lat)) * math.cos(dlon)
            bearing = math.atan2(y, x)

            # Check if we've reached the waypoint (within 10 meters)
            if distance < 10.0:
                state.current_waypoint_idx += 1
                logger.info(f"Reached waypoint {state.current_waypoint_idx}, advancing to next")

                # Loop back to start if we've completed the route
                if state.current_waypoint_idx >= len(waypoints):
                    state.current_waypoint_idx = 0
                    logger.info("Route complete, looping back to start")
            else:
                # Move towards waypoint
                new_lat, new_lon = calculate_new_position(
                    state.position.latitude,
                    state.position.longitude,
                    state.route_speed,
                    bearing,
                    time_delta
                )
                state.position.latitude = new_lat
                state.position.longitude = new_lon
                state.position.speed_over_ground = state.route_speed
                state.position.course_over_ground = bearing

    elif state.mode == 'anchoring':
        # Anchoring mode with drop and swing physics
        if state.anchor_drop_phase == 'idle':
            # Initialize anchoring - drop anchor at current position
            state.anchor_position = {
                'lat': state.position.latitude,
                'lon': state.position.longitude
            }
            state.anchor_drop_phase = 'dropping'
            state.anchor_drop_start_time = current_time
            state.anchor_trail.clear()
            logger.info(f"Anchor dropped at {state.anchor_position['lat']:.6f}, {state.anchor_position['lon']:.6f}")

        elif state.anchor_drop_phase == 'dropping':
            # Move boat away from anchor until rode distance reached
            anchor_lat = state.anchor_position['lat']
            anchor_lon = state.anchor_position['lon']

            # Calculate current distance from anchor (Haversine)
            dlat = math.radians(state.position.latitude - anchor_lat)
            dlon = math.radians(state.position.longitude - anchor_lon)
            a = math.sin(dlat/2)**2 + math.cos(math.radians(anchor_lat)) * \
                math.cos(math.radians(state.position.latitude)) * math.sin(dlon/2)**2
            c = 2 * math.atan2(math.sqrt(a), math.sqrt(1-a))
            distance_from_anchor = 6371000 * c  # meters

            if distance_from_anchor >= state.anchor_rode_distance:
                # Transition to swinging phase
                state.anchor_drop_phase = 'swinging'
                # Calculate initial swing angle
                state.anchor_swing_angle = math.atan2(
                    state.position.longitude - anchor_lon,
                    state.position.latitude - anchor_lat
                )
                state.anchor_swing_velocity = 0.0
                logger.info(f"Anchor set - boat at rode distance {distance_from_anchor:.1f}m, entering swing mode")
            else:
                # Continue moving away from anchor
                if state.wind_speed > 0:
                    # Move in direction wind is blowing TO (add 180° to wind direction)
                    # Wind direction is where wind comes FROM, boat drifts TO opposite direction
                    wind_direction_rad = math.radians(state.wind_direction) + math.pi
                    new_lat, new_lon = calculate_new_position(
                        state.position.latitude,
                        state.position.longitude,
                        state.wind_speed,
                        wind_direction_rad,
                        time_delta
                    )
                else:
                    # No wind - back away at 0.5 knots
                    reverse_course = (state.position.course_over_ground + math.pi) % (2 * math.pi)
                    new_lat, new_lon = calculate_new_position(
                        state.position.latitude,
                        state.position.longitude,
                        0.5,  # knots
                        reverse_course,
                        time_delta
                    )
                state.position.latitude = new_lat
                state.position.longitude = new_lon
                state.position.speed_over_ground = state.wind_speed if state.wind_speed > 0 else 0.5

        elif state.anchor_drop_phase == 'swinging':
            # Pendulum swing physics influenced by wind
            anchor_lat = state.anchor_position['lat']
            anchor_lon = state.anchor_position['lon']

            # Physics constants
            DAMPING = 0.1  # Damping coefficient
            WIND_FORCE_FACTOR = 0.05  # Wind influence strength
            RANDOM_PERTURBATION = 0.01  # Random movement
            METERS_PER_DEGREE_LAT = 111320.0

            # Calculate meters per degree longitude at this latitude
            meters_per_degree_lon = 111320.0 * math.cos(math.radians(anchor_lat))

            # Wind force calculation
            # Wind direction is where wind comes FROM, add 180° to get direction boat is pushed TO
            wind_push_direction_rad = math.radians(state.wind_direction) + math.pi
            wind_force = state.wind_speed * math.sin(wind_push_direction_rad - state.anchor_swing_angle) * WIND_FORCE_FACTOR

            # Angular acceleration (pendulum physics)
            angular_accel = wind_force / max(state.anchor_rode_distance, 1.0)
            angular_accel -= DAMPING * state.anchor_swing_velocity
            angular_accel += random.uniform(-RANDOM_PERTURBATION, RANDOM_PERTURBATION)

            # Update velocity and angle
            state.anchor_swing_velocity += angular_accel * time_delta
            state.anchor_swing_angle += state.anchor_swing_velocity * time_delta

            # Calculate distance with variation (70-100% of rode distance)
            distance_variation = 0.7 + 0.3 * random.random()
            current_distance = state.anchor_rode_distance * distance_variation

            # Convert polar to Cartesian (relative to anchor)
            new_lat = anchor_lat + (current_distance * math.cos(state.anchor_swing_angle)) / METERS_PER_DEGREE_LAT
            new_lon = anchor_lon + (current_distance * math.sin(state.anchor_swing_angle)) / meters_per_degree_lon

            # Add small random jitter (wave action)
            jitter_distance = 2.0  # meters
            new_lat += random.uniform(-jitter_distance, jitter_distance) / METERS_PER_DEGREE_LAT
            new_lon += random.uniform(-jitter_distance, jitter_distance) / meters_per_degree_lon

            # Update position
            state.position.latitude = new_lat
            state.position.longitude = new_lon
            state.position.speed_over_ground = abs(state.anchor_swing_velocity * state.anchor_rode_distance)
            state.position.course_over_ground = state.anchor_swing_angle

            # Record trail point
            state.anchor_trail.append((new_lat, new_lon, current_time))

    state.position.timestamp = current_time


def simulator_loop():
    """Main simulator loop"""
    logger.info("Simulator loop started")

    while state.running:
        try:
            # Update variable wind if enabled
            if state.variable_wind:
                current_time = time.time()
                if current_time - state.variable_wind_last_update >= 30.0:
                    # Vary speed up to 20 knots from base
                    import random
                    speed_variation = random.uniform(0, 20)
                    state.wind_speed = max(0, state.variable_wind_base_speed + random.uniform(-speed_variation, speed_variation))

                    # Vary direction ±30 degrees from base
                    dir_variation = random.uniform(-30, 30)
                    state.wind_direction = (state.variable_wind_base_direction + dir_variation) % 360

                    state.variable_wind_last_update = current_time
                    logger.debug(f"Variable wind updated: {state.wind_speed:.1f} kts at {state.wind_direction:.0f}°")

            # Update position
            update_position()

            # Update heading from course (if in motion)
            if state.position.speed_over_ground > 0.1:  # Moving
                state.heading.heading = state.position.course_over_ground
                state.heading.timestamp = state.position.timestamp

            # Broadcast GPS position
            broadcast_position(state.position)

            # Broadcast heading (at slower rate)
            current_time = time.time()
            if state.heading_enabled and (current_time - state.last_heading_update >= 1.0 / state.heading_update_rate):
                broadcast_heading(state.heading)
                state.last_heading_update = current_time

            # Generate NMEA sentence for UI display
            from pgn_messages import PGN129029
            nmea_sentence = PGN129029.to_nmea0183(state.position)

            # Emit to web clients (requires app context from background thread)
            with app.app_context():
                socketio.emit('gps_update', {
                    'latitude': state.position.latitude,
                    'longitude': state.position.longitude,
                    'altitude': state.position.altitude,
                    'speed': state.position.speed_over_ground,
                    'course': math.degrees(state.position.course_over_ground),
                    'heading': math.degrees(state.heading.heading),
                    'timestamp': state.position.timestamp,
                    'nmea': nmea_sentence.strip()
                })

            # Sleep (use socketio.sleep for eventlet compatibility)
            socketio.sleep(1.0 / state.update_rate)

        except Exception as e:
            logger.error(f"Simulator loop error: {e}")
            time.sleep(1)

    logger.info("Simulator loop stopped")


def start_simulator_thread():
    """
    Safely start the simulator thread.
    Prevents multiple threads from running simultaneously.
    Returns True if started, False if already running.
    """
    # Check if simulator is already running using the state flag
    if state.running:
        logger.warning("Simulator already running, ignoring start request")
        return False

    # Start new thread using socketio.start_background_task for eventlet compatibility
    state.running = True
    state.simulator_thread = socketio.start_background_task(simulator_loop)
    logger.info("Simulator thread started")
    return True


# Flask routes
@app.route('/')
def index():
    """Main web interface"""
    return render_template('n2k_index.html')


@app.route('/api/status')
def get_status():
    """Get simulator status"""
    status = {
        'running': state.running,
        'mode': state.mode,
        'position': {
            'latitude': state.position.latitude,
            'longitude': state.position.longitude,
            'altitude': state.position.altitude,
            'speed': state.position.speed_over_ground,
            'course': math.degrees(state.position.course_over_ground)
        },
        'update_rate': state.update_rate,
        'wind': {
            'speed': state.wind_speed,
            'direction': state.wind_direction
        },
        'network': {
            'udp_port': state.udp_port,
            'tcp_port': state.tcp_port,
            'tcp_clients': len(state.tcp_clients)
        }
    }

    # Add CAN status if available
    if state.can_interface:
        status['can'] = state.can_interface.get_status()
    else:
        status['can'] = {
            'enabled': state.can_enabled,
            'connected': False,
            'channel': state.can_channel,
            'bustype': state.can_bustype,
            'bitrate': state.can_bitrate,
            'source_address': state.can_source_address
        }

    return jsonify(status)


@app.route('/api/start', methods=['POST'])
def start_simulator():
    """Start the simulator"""
    started = start_simulator_thread()
    if started:
        logger.info("Simulator started via API")
    return jsonify({'status': 'running' if state.running else 'already_running'})


@app.route('/api/stop', methods=['POST'])
def stop_simulator():
    """Stop the simulator"""
    state.running = False
    logger.info("Simulator stopped")
    return jsonify({'status': 'stopped'})


@app.route('/api/reset', methods=['POST'])
def reset_simulator():
    """Reset simulator to initial state"""
    # Stop simulator
    state.running = False

    # Reset position to initial values
    state.position.latitude = state.initial_lat
    state.position.longitude = state.initial_lon
    state.position.speed_over_ground = 0.0
    state.position.course_over_ground = 0.0
    state.position.altitude = 0.0

    # Reset mode to stationary
    state.mode = 'stationary'

    # Reset wind conditions
    state.wind_speed = 0.0
    state.wind_direction = 0.0
    state.variable_wind = False

    # Reset anchoring state
    state.anchor_position = None
    state.anchor_drop_phase = 'idle'
    state.saved_anchor_position = None
    state.anchor_trail.clear()

    # Reset circle center to initial position
    state.circle_center_lat = state.initial_lat
    state.circle_center_lon = state.initial_lon

    # Clear active route
    state.active_route = None
    state.current_waypoint_idx = 0

    logger.info("Simulator reset to initial state")
    return jsonify({
        'status': 'reset',
        'position': {
            'lat': state.position.latitude,
            'lon': state.position.longitude
        }
    })


@app.route('/api/mode', methods=['POST'])
def set_mode():
    """Set simulator mode"""
    data = request.json
    mode = data.get('mode', 'stationary')

    if mode in ['stationary', 'drift', 'circle', 'waypoint', 'anchoring']:
        # Save anchor position when switching from anchoring to drift
        if state.mode == 'anchoring' and mode == 'drift' and state.anchor_position:
            state.saved_anchor_position = state.anchor_position.copy()
            logger.info(f"Saved anchor position for drift visualization: {state.saved_anchor_position}")

        # Clear saved anchor when entering anchoring mode or other modes
        if mode in ['anchoring', 'stationary', 'circle', 'waypoint']:
            state.saved_anchor_position = None

        state.mode = mode

        if mode == 'circle':
            # Set circle center to current position
            state.circle_center_lat = state.position.latitude
            state.circle_center_lon = state.position.longitude
        elif mode == 'drift':
            # Set default wind if none present: 5 knots from 180° (south wind, pushes boat north)
            if state.wind_speed == 0:
                state.wind_speed = 5.0
                state.wind_direction = 180.0
                logger.info("Drift mode: Set default wind to 5 kts from 180°")
        elif mode == 'anchoring':
            # Reset anchoring state
            state.anchor_drop_phase = 'idle'
            # Set default wind if none present: 5 knots from 180° (south wind, pushes boat north)
            if state.wind_speed == 0:
                state.wind_speed = 5.0
                state.wind_direction = 180.0
                logger.info("Anchoring mode: Set default wind to 5 kts from 180°")

        logger.info(f"Mode changed to: {mode}")
        return jsonify({'mode': mode, 'wind': {'speed': state.wind_speed, 'direction': state.wind_direction}})

    return jsonify({'error': 'Invalid mode'}), 400


@app.route('/api/position', methods=['POST'])
def set_position():
    """Set current position"""
    data = request.json

    if 'latitude' in data:
        state.position.latitude = float(data['latitude'])
    if 'longitude' in data:
        state.position.longitude = float(data['longitude'])
    if 'altitude' in data:
        state.position.altitude = float(data['altitude'])

    # Update circle center if in circle mode
    if state.mode == 'circle':
        state.circle_center_lat = state.position.latitude
        state.circle_center_lon = state.position.longitude

    logger.info(f"Position set to: {state.position.latitude}, {state.position.longitude}")
    return jsonify({
        'status': 'success',
        'position': {
            'latitude': state.position.latitude,
            'longitude': state.position.longitude,
            'altitude': state.position.altitude
        }
    })


@app.route('/api/drift', methods=['POST'])
def set_drift():
    """Set drift parameters"""
    data = request.json

    if 'speed' in data:
        state.drift_speed = float(data['speed'])
    if 'course' in data:
        state.drift_course = math.radians(float(data['course']))

    logger.info(f"Drift set to: {state.drift_speed} m/s at {math.degrees(state.drift_course)} degrees")
    return jsonify({
        'speed': state.drift_speed,
        'course': math.degrees(state.drift_course)
    })


@app.route('/api/wind', methods=['POST'])
def set_wind():
    """Set wind parameters"""
    data = request.json

    if 'speed' in data:
        state.wind_speed = float(data['speed'])
    if 'direction' in data:
        state.wind_direction = float(data['direction'])

    # Update base values for variable wind if enabled
    if state.variable_wind:
        state.variable_wind_base_speed = state.wind_speed
        state.variable_wind_base_direction = state.wind_direction

    logger.info(f"Wind set to: {state.wind_speed} knots at {state.wind_direction} degrees")
    return jsonify({
        'speed': state.wind_speed,
        'direction': state.wind_direction
    })


@app.route('/api/drift/manual', methods=['POST'])
def set_manual_drift():
    """Set manual drift parameters (used when wind speed is 0)"""
    data = request.json

    if 'direction' in data:
        state.manual_drift_direction = float(data['direction'])
    if 'speed' in data:
        state.manual_drift_speed = float(data['speed'])

    logger.info(f"Manual drift set to: {state.manual_drift_speed} knots at {state.manual_drift_direction} degrees")
    return jsonify({
        'status': 'success',
        'direction': state.manual_drift_direction,
        'speed': state.manual_drift_speed
    })


@app.route('/api/wind/variable', methods=['POST'])
def set_variable_wind():
    """Enable/disable variable wind mode"""
    data = request.json
    enabled = data.get('enabled', False)

    state.variable_wind = enabled
    if enabled:
        # Store current values as base for variation
        state.variable_wind_base_speed = state.wind_speed
        state.variable_wind_base_direction = state.wind_direction
        state.variable_wind_last_update = time.time()
        logger.info("Variable wind enabled")
    else:
        logger.info("Variable wind disabled")

    return jsonify({'enabled': state.variable_wind})


@app.route('/api/anchor/rode-distance', methods=['POST'])
def set_anchor_rode_distance():
    """Set anchor rode distance"""
    data = request.json
    distance = float(data.get('distance', 50.0))

    if 10.0 <= distance <= 300.0:
        state.anchor_rode_distance = distance
        logger.info(f"Anchor rode distance set to: {distance} meters")
        return jsonify({'status': 'success', 'distance': distance})

    return jsonify({'error': 'Distance must be between 10 and 300 meters'}), 400


@app.route('/api/anchor/state', methods=['GET'])
def get_anchor_state():
    """Get current anchor state"""
    if state.anchor_position:
        # Calculate current distance from anchor
        anchor_lat = state.anchor_position['lat']
        anchor_lon = state.anchor_position['lon']

        dlat = math.radians(state.position.latitude - anchor_lat)
        dlon = math.radians(state.position.longitude - anchor_lon)
        a = math.sin(dlat/2)**2 + math.cos(math.radians(anchor_lat)) * \
            math.cos(math.radians(state.position.latitude)) * math.sin(dlon/2)**2
        c = 2 * math.atan2(math.sqrt(a), math.sqrt(1-a))
        current_distance = 6371000 * c  # meters

        # Convert trail deque to list for JSON serialization
        trail_list = [[lat, lon] for lat, lon, _ in list(state.anchor_trail)]

        # Add saved anchor position for drift visualization
        saved_anchor = None
        if state.saved_anchor_position:
            saved_anchor = [state.saved_anchor_position['lat'], state.saved_anchor_position['lon']]

        # Calculate anchor circle from trail points
        # Use actual anchor position as center (since we know where it is)
        calculated_anchor = None
        if len(trail_list) >= 10:
            # Calculate maximum distance from actual anchor position
            max_distance_m = 0.0
            avg_distance_m = 0.0
            for point_lat, point_lon in trail_list:
                # Haversine distance calculation from actual anchor
                dlat = math.radians(point_lat - anchor_lat)
                dlon = math.radians(point_lon - anchor_lon)
                a = math.sin(dlat/2)**2 + math.cos(math.radians(anchor_lat)) * \
                    math.cos(math.radians(point_lat)) * math.sin(dlon/2)**2
                c = 2 * math.atan2(math.sqrt(a), math.sqrt(1-a))
                dist = 6371000 * c  # meters
                max_distance_m = max(max_distance_m, dist)
                avg_distance_m += dist

            avg_distance_m = avg_distance_m / len(trail_list)

            calculated_anchor = {
                'center_lat': anchor_lat,
                'center_lon': anchor_lon,
                'radius_m': round(max_distance_m, 2),
                'radius_ft': round(max_distance_m * 3.28084, 1),
                'avg_radius_m': round(avg_distance_m, 2),
                'avg_radius_ft': round(avg_distance_m * 3.28084, 1),
                'points_used': len(trail_list)
            }

        return jsonify({
            'anchor_position': [anchor_lat, anchor_lon],
            'boat_position': [state.position.latitude, state.position.longitude],
            'rode_distance': state.anchor_rode_distance,
            'current_distance': round(current_distance, 2),
            'phase': state.anchor_drop_phase,
            'trail': trail_list,
            'wind': {
                'speed': state.wind_speed,
                'direction': state.wind_direction
            },
            'mode': state.mode,
            'saved_anchor_position': saved_anchor,
            'calculated_anchor': calculated_anchor
        })
    else:
        # Check if we have a saved anchor position from drift mode
        saved_anchor = None
        if state.saved_anchor_position:
            saved_anchor = [state.saved_anchor_position['lat'], state.saved_anchor_position['lon']]

        return jsonify({
            'anchor_position': None,
            'boat_position': [state.position.latitude, state.position.longitude],
            'rode_distance': state.anchor_rode_distance,
            'current_distance': 0,
            'phase': state.anchor_drop_phase,
            'trail': [],
            'wind': {
                'speed': state.wind_speed,
                'direction': state.wind_direction
            },
            'mode': state.mode,
            'saved_anchor_position': saved_anchor
        })


@app.route('/api/settings', methods=['GET', 'POST'])
def settings():
    """Get or update simulator settings"""
    if request.method == 'POST':
        data = request.json

        if 'update_rate' in data:
            new_rate = float(data['update_rate'])
            if 0.1 <= new_rate <= 10.0:
                state.update_rate = new_rate
                logger.info(f"Update rate changed to: {state.update_rate} Hz")
            else:
                return jsonify({'error': 'Update rate must be between 0.1 and 10 Hz'}), 400

        if 'anchor_rode_distance' in data:
            distance = float(data['anchor_rode_distance'])
            if 10.0 <= distance <= 300.0:
                state.anchor_rode_distance = distance
                logger.info(f"Anchor rode distance set to: {distance} meters")
            else:
                return jsonify({'error': 'Anchor rode distance must be between 10 and 300 meters'}), 400

    return jsonify({
        'update_rate': state.update_rate,
        'anchor_rode_distance': state.anchor_rode_distance,
        'position': {
            'latitude': state.position.latitude,
            'longitude': state.position.longitude
        }
    })


def parse_gpx(file_content):
    """Parse GPX file and extract route waypoints"""
    try:
        root = ET.fromstring(file_content)

        # Define namespaces
        ns = {'gpx': 'http://www.topografix.com/GPX/1/1'}

        # Try with namespace first
        routes = root.findall('.//gpx:rte', ns)
        if not routes:
            # Try without namespace
            routes = root.findall('.//rte')

        if not routes:
            return None

        route = routes[0]  # Use first route

        # Get route name
        route_name_elem = route.find('.//gpx:name', ns)
        if route_name_elem is None:
            route_name_elem = route.find('.//name')
        route_name = route_name_elem.text if route_name_elem is not None else 'Unnamed Route'

        # Get waypoints
        waypoints = []
        rtepts = route.findall('.//gpx:rtept', ns)
        if not rtepts:
            rtepts = route.findall('.//rtept')

        for rtept in rtepts:
            lat = float(rtept.get('lat'))
            lon = float(rtept.get('lon'))

            name_elem = rtept.find('.//gpx:name', ns)
            if name_elem is None:
                name_elem = rtept.find('.//name')
            name = name_elem.text if name_elem is not None else f'WP{len(waypoints) + 1}'

            waypoints.append({'lat': lat, 'lon': lon, 'name': name})

        return {
            'name': route_name,
            'waypoints': waypoints
        }
    except Exception as e:
        logger.error(f"GPX parse error: {e}")
        return None


@app.route('/api/route/upload', methods=['POST'])
def upload_route():
    """Upload and parse a GPX route file"""
    if 'file' not in request.files:
        return jsonify({'status': 'error', 'error': 'No file uploaded'}), 400

    file = request.files['file']
    if file.filename == '':
        return jsonify({'status': 'error', 'error': 'No file selected'}), 400

    try:
        content = file.read().decode('utf-8')
        route_data = parse_gpx(content)

        if not route_data:
            return jsonify({'status': 'error', 'error': 'Failed to parse GPX file'}), 400

        # Generate unique ID
        route_id = str(uuid.uuid4())[:8]
        route_data['id'] = route_id
        route_data['waypoint_count'] = len(route_data['waypoints'])

        # Store route
        state.routes[route_id] = route_data

        logger.info(f"Route uploaded: {route_data['name']} with {route_data['waypoint_count']} waypoints")

        return jsonify({
            'status': 'success',
            'route': {
                'id': route_id,
                'name': route_data['name'],
                'waypoint_count': route_data['waypoint_count']
            }
        })
    except Exception as e:
        logger.error(f"Route upload error: {e}")
        return jsonify({'status': 'error', 'error': str(e)}), 500


@app.route('/api/route/list')
def list_routes():
    """List all uploaded routes"""
    routes = [
        {
            'id': rid,
            'name': rdata['name'],
            'waypoint_count': rdata['waypoint_count']
        }
        for rid, rdata in state.routes.items()
    ]
    return jsonify({'routes': routes})


@app.route('/api/route/activate/<route_id>', methods=['POST'])
def activate_route(route_id):
    """Activate a route for playback"""
    if route_id not in state.routes:
        return jsonify({'status': 'error', 'error': 'Route not found'}), 404

    state.active_route = state.routes[route_id]
    state.current_waypoint_idx = 0

    # Set position to first waypoint
    if state.active_route['waypoints']:
        first_wp = state.active_route['waypoints'][0]
        state.position.latitude = first_wp['lat']
        state.position.longitude = first_wp['lon']

    logger.info(f"Route activated: {state.active_route['name']}")

    return jsonify({
        'status': 'success',
        'route': state.active_route
    })


@app.route('/api/route/start', methods=['POST'])
def start_route():
    """Start following the active route"""
    if not state.active_route:
        return jsonify({'status': 'error', 'error': 'No route activated'}), 400

    data = request.json
    if 'speed' in data:
        state.route_speed = float(data['speed'])

    state.mode = 'waypoint'
    logger.info(f"Route playback started at {state.route_speed} m/s")

    return jsonify({'status': 'success'})


@app.route('/api/route/stop', methods=['POST'])
def stop_route():
    """Stop following the route"""
    state.mode = 'stationary'
    logger.info("Route playback stopped")
    return jsonify({'status': 'success'})


@app.route('/api/hardware/status', methods=['GET'])
def get_hardware_status():
    """Get current hardware output status (from LVGL UI)"""
    return jsonify({
        'buzzer_active': hardware_state.buzzer_active,
        'relay_active': hardware_state.relay_active,
        'led_state': hardware_state.led_state,
        'alarm_active': hardware_state.alarm_active,
        'anchor_alert_active': hardware_state.anchor_alert_active,
        'mute_active': hardware_state.mute_active
    })


@app.route('/api/hardware/control', methods=['POST'])
def set_hardware_control():
    """Receive hardware control signals from LVGL UI"""
    data = request.json

    if 'buzzer_active' in data:
        hardware_state.buzzer_active = bool(data['buzzer_active'])
        logger.info(f"Buzzer: {'ON' if hardware_state.buzzer_active else 'OFF'}")

    if 'relay_active' in data:
        hardware_state.relay_active = bool(data['relay_active'])
        logger.info(f"Relay: {'ON' if hardware_state.relay_active else 'OFF'}")

    if 'led_state' in data:
        hardware_state.led_state = int(data['led_state'])
        led_names = ['OFF', 'RED', 'YELLOW', 'GREEN', 'BLINKING_RED']
        led_name = led_names[hardware_state.led_state] if hardware_state.led_state < len(led_names) else str(hardware_state.led_state)
        logger.info(f"LED State: {led_name}")

    if 'alarm_active' in data:
        hardware_state.alarm_active = bool(data['alarm_active'])
        logger.info(f"Alarm Panel: {'ACTIVE' if hardware_state.alarm_active else 'OFF'}")

    if 'anchor_alert_active' in data:
        hardware_state.anchor_alert_active = bool(data['anchor_alert_active'])
        logger.info(f"Anchor Alert: {'ACTIVE' if hardware_state.anchor_alert_active else 'OFF'}")

    # Emit to all web clients via WebSocket
    with app.app_context():
        socketio.emit('hardware_update', {
            'buzzer_active': hardware_state.buzzer_active,
            'relay_active': hardware_state.relay_active,
            'led_state': hardware_state.led_state,
            'alarm_active': hardware_state.alarm_active,
            'anchor_alert_active': hardware_state.anchor_alert_active,
            'mute_active': hardware_state.mute_active
        })

    return jsonify({'status': 'success'})


@app.route('/api/hardware/mute', methods=['POST'])
def toggle_mute():
    """Toggle mute state"""
    hardware_state.mute_active = not hardware_state.mute_active
    logger.info(f"Mute toggled: {'ON' if hardware_state.mute_active else 'OFF'}")

    # If muted, turn off buzzer
    if hardware_state.mute_active:
        hardware_state.buzzer_active = False

    # Emit to all web clients via WebSocket
    with app.app_context():
        socketio.emit('hardware_update', {
            'buzzer_active': hardware_state.buzzer_active,
            'relay_active': hardware_state.relay_active,
            'led_state': hardware_state.led_state,
            'alarm_active': hardware_state.alarm_active,
            'anchor_alert_active': hardware_state.anchor_alert_active,
            'mute_active': hardware_state.mute_active
        })

    return jsonify({
        'status': 'success',
        'mute_active': hardware_state.mute_active
    })


@app.route('/api/can/enable', methods=['POST'])
def enable_can():
    """Enable CAN bus output"""
    if not state.can_interface:
        # Initialize CAN interface
        try:
            state.can_interface = NMEA2000CANInterface(
                channel=state.can_channel,
                bustype=state.can_bustype,
                bitrate=state.can_bitrate,
                source_address=state.can_source_address,
                enabled=True
            )
            state.can_enabled = state.can_interface.enabled
            logger.info("CAN bus enabled via API")
        except Exception as e:
            logger.error(f"Failed to enable CAN bus: {e}")
            return jsonify({'status': 'error', 'error': str(e)}), 500
    else:
        # Reconnect if disconnected
        if not state.can_interface.enabled:
            if state.can_interface.connect():
                state.can_enabled = True
                logger.info("CAN bus reconnected via API")
            else:
                return jsonify({'status': 'error', 'error': 'Failed to connect'}), 500
        else:
            logger.info("CAN bus already enabled")

    return jsonify({
        'status': 'success',
        'can': state.can_interface.get_status() if state.can_interface else None
    })


@app.route('/api/can/disable', methods=['POST'])
def disable_can():
    """Disable CAN bus output"""
    if state.can_interface:
        state.can_interface.disconnect()
        state.can_enabled = False
        logger.info("CAN bus disabled via API")

    return jsonify({'status': 'success', 'enabled': False})


@app.route('/api/can/config', methods=['POST'])
def configure_can():
    """Configure CAN bus parameters (requires restart)"""
    data = request.json

    if 'channel' in data:
        state.can_channel = data['channel']
    if 'bustype' in data:
        state.can_bustype = data['bustype']
    if 'bitrate' in data:
        state.can_bitrate = int(data['bitrate'])
    if 'source_address' in data:
        state.can_source_address = int(data['source_address'])

    logger.info(f"CAN config updated: {state.can_bustype} {state.can_channel} @ {state.can_bitrate}bps, SA={state.can_source_address}")

    # If CAN is currently enabled, restart it with new config
    if state.can_enabled and state.can_interface:
        state.can_interface.disconnect()
        try:
            state.can_interface = NMEA2000CANInterface(
                channel=state.can_channel,
                bustype=state.can_bustype,
                bitrate=state.can_bitrate,
                source_address=state.can_source_address,
                enabled=True
            )
            state.can_enabled = state.can_interface.enabled
            logger.info("CAN bus restarted with new configuration")
        except Exception as e:
            logger.error(f"Failed to restart CAN bus: {e}")
            state.can_enabled = False
            return jsonify({'status': 'error', 'error': str(e)}), 500

    return jsonify({
        'status': 'success',
        'config': {
            'channel': state.can_channel,
            'bustype': state.can_bustype,
            'bitrate': state.can_bitrate,
            'source_address': state.can_source_address
        },
        'can': state.can_interface.get_status() if state.can_interface else None
    })


@app.route('/api/heading', methods=['GET'])
def get_heading():
    """Get current heading"""
    return jsonify({
        'heading': math.degrees(state.heading.heading),
        'heading_rad': state.heading.heading,
        'deviation': math.degrees(state.heading.deviation),
        'variation': math.degrees(state.heading.variation),
        'reference': state.heading.reference,
        'enabled': state.heading_enabled,
        'update_rate': state.heading_update_rate
    })


@app.route('/api/heading', methods=['POST'])
def set_heading():
    """Set heading parameters"""
    data = request.json

    if 'heading' in data:
        state.heading.heading = math.radians(float(data['heading']))
    if 'deviation' in data:
        state.heading.deviation = math.radians(float(data['deviation']))
    if 'variation' in data:
        state.heading.variation = math.radians(float(data['variation']))
    if 'reference' in data:
        ref = data['reference']
        if ref in ['True', 'Magnetic']:
            state.heading.reference = ref
    if 'enabled' in data:
        state.heading_enabled = bool(data['enabled'])

    logger.info(f"Heading updated: {math.degrees(state.heading.heading):.1f}° ({state.heading.reference})")

    return jsonify({
        'status': 'success',
        'heading': {
            'heading': math.degrees(state.heading.heading),
            'deviation': math.degrees(state.heading.deviation),
            'variation': math.degrees(state.heading.variation),
            'reference': state.heading.reference,
            'enabled': state.heading_enabled
        }
    })


@app.route('/api/device', methods=['GET'])
def get_device_info():
    """Get NMEA 2000 device identification"""
    return jsonify({
        'manufacturer_code': state.device_info.manufacturer_code,
        'manufacturer_name': get_manufacturer_name(state.device_info.manufacturer_code),
        'unique_number': state.device_info.unique_number,
        'device_function': state.device_info.device_function,
        'device_class': state.device_info.device_class,
        'device_instance': state.device_info.device_instance,
        'system_instance': state.device_info.system_instance,
        'industry_group': state.device_info.industry_group
    })


@app.route('/api/device', methods=['POST'])
def set_device_info():
    """Set NMEA 2000 device identification"""
    data = request.json

    if 'manufacturer_code' in data:
        state.device_info.manufacturer_code = int(data['manufacturer_code'])
    if 'unique_number' in data:
        state.device_info.unique_number = int(data['unique_number'])
    if 'device_function' in data:
        state.device_info.device_function = int(data['device_function'])
    if 'device_class' in data:
        state.device_info.device_class = int(data['device_class'])
    if 'device_instance' in data:
        state.device_info.device_instance = int(data['device_instance'])
    if 'system_instance' in data:
        state.device_info.system_instance = int(data['system_instance'])
    if 'industry_group' in data:
        state.device_info.industry_group = int(data['industry_group'])

    logger.info(f"Device info updated: Manufacturer={state.device_info.manufacturer_code}, "
                f"Unique={state.device_info.unique_number}")

    return jsonify({
        'status': 'success',
        'device': {
            'manufacturer_code': state.device_info.manufacturer_code,
            'manufacturer_name': get_manufacturer_name(state.device_info.manufacturer_code),
            'unique_number': state.device_info.unique_number,
            'device_function': state.device_info.device_function,
            'device_class': state.device_info.device_class,
            'device_instance': state.device_info.device_instance,
            'system_instance': state.device_info.system_instance,
            'industry_group': state.device_info.industry_group
        }
    })


def get_manufacturer_name(code):
    """Get manufacturer name from code"""
    manufacturers = {
        8: 'Actisense',
        80: 'Furuno',
        135: 'Airmar',
        137: 'Lowrance',
        140: 'Raymarine',
        144: 'Mercury',
        147: 'Nauticus',
        154: 'Evinrude',
        165: 'Kohler',
        168: 'Volvo Penta',
        172: 'Garmin',
        198: 'Maretron',
        229: 'B&G',
        233: 'Digital Yacht',
        275: 'Yanmar',
        304: 'Simrad',
        378: 'Navico',
        381: 'Honda',
        419: 'Victron',
        529: 'Yacht Devices',
        573: 'Ocean Signal',
        717: 'SailorHat',
        799: 'Navionics',
        2046: 'Test/Unknown'
    }
    return manufacturers.get(code, f'Unknown ({code})')


@app.route('/api/serial/enable', methods=['POST'])
def enable_serial():
    """Enable serial port output"""
    if not state.serial_interface:
        # Initialize serial port
        try:
            import serial
            state.serial_interface = serial.Serial(
                port=state.serial_port,
                baudrate=state.serial_baudrate,
                bytesize=serial.EIGHTBITS,
                parity=serial.PARITY_NONE,
                stopbits=serial.STOPBITS_ONE,
                timeout=1
            )
            state.serial_enabled = True
            logger.info("Serial port enabled via API")
        except Exception as e:
            logger.error(f"Failed to enable serial port: {e}")
            return jsonify({'status': 'error', 'error': str(e)}), 500
    else:
        if not state.serial_interface.is_open:
            try:
                state.serial_interface.open()
                state.serial_enabled = True
                logger.info("Serial port reconnected via API")
            except Exception as e:
                return jsonify({'status': 'error', 'error': str(e)}), 500
        else:
            logger.info("Serial port already enabled")

    return jsonify({
        'status': 'success',
        'serial': {
            'enabled': state.serial_enabled,
            'port': state.serial_port,
            'baudrate': state.serial_baudrate
        }
    })


@app.route('/api/serial/disable', methods=['POST'])
def disable_serial():
    """Disable serial port output"""
    if state.serial_interface and state.serial_interface.is_open:
        state.serial_interface.close()
        state.serial_enabled = False
        logger.info("Serial port disabled via API")

    return jsonify({'status': 'success', 'enabled': False})


@app.route('/api/serial/config', methods=['POST'])
def configure_serial():
    """Configure serial port parameters (requires restart)"""
    data = request.json

    if 'port' in data:
        state.serial_port = data['port']
    if 'baudrate' in data:
        state.serial_baudrate = int(data['baudrate'])

    logger.info(f"Serial config updated: {state.serial_port} @ {state.serial_baudrate} bps")

    # If serial is currently enabled, restart it with new config
    if state.serial_enabled and state.serial_interface:
        state.serial_interface.close()
        try:
            import serial
            state.serial_interface = serial.Serial(
                port=state.serial_port,
                baudrate=state.serial_baudrate,
                bytesize=serial.EIGHTBITS,
                parity=serial.PARITY_NONE,
                stopbits=serial.STOPBITS_ONE,
                timeout=1
            )
            state.serial_enabled = True
            logger.info("Serial port restarted with new configuration")
        except Exception as e:
            logger.error(f"Failed to restart serial port: {e}")
            state.serial_enabled = False
            return jsonify({'status': 'error', 'error': str(e)}), 500

    return jsonify({
        'status': 'success',
        'config': {
            'port': state.serial_port,
            'baudrate': state.serial_baudrate,
            'enabled': state.serial_enabled
        }
    })


@app.route('/api/serial/status', methods=['GET'])
def get_serial_status():
    """Get serial port status"""
    return jsonify({
        'enabled': state.serial_enabled,
        'connected': state.serial_interface.is_open if state.serial_interface else False,
        'port': state.serial_port,
        'baudrate': state.serial_baudrate
    })


# WebSocket events
@socketio.on('connect')
def handle_connect():
    """Handle client connection"""
    emit('status', {
        'running': state.running,
        'mode': state.mode
    })


@socketio.on('start')
def handle_start():
    """Start simulator via WebSocket"""
    started = start_simulator_thread()
    if started:
        logger.info("Simulator started via WebSocket")
    emit('status', {'running': state.running})


@socketio.on('stop')
def handle_stop():
    """Stop simulator via WebSocket"""
    state.running = False
    logger.info("Simulator stopped via WebSocket")
    emit('status', {'running': False})


def main():
    """Main entry point"""
    print("=" * 60)
    print("  N2K Data Simulator")
    print("=" * 60)
    print()
    print(f"Starting position: {state.position.latitude}, {state.position.longitude}")
    print(f"UDP output: port {state.udp_port}")
    print(f"TCP output: port {state.tcp_port}")
    print()

    # Set running to True if AUTO_START is enabled
    if os.environ.get('AUTO_START', 'true').lower() == 'true':
        state.running = True

    # Initialize network
    init_network()

    # Start simulator loop (if AUTO_START is enabled)
    if state.running:
        if start_simulator_thread():
            print("Simulator auto-started")
        else:
            print("Warning: Simulator thread already running")

    # Start web server
    host = os.environ.get('HOST', '0.0.0.0')
    port = int(os.environ.get('PORT', 8081))
    debug = os.environ.get('DEBUG', 'false').lower() == 'true'

    print(f"Web interface: http://{host}:{port}")
    print("=" * 60)

    socketio.run(app, host=host, port=port, debug=debug)


if __name__ == '__main__':
    main()
