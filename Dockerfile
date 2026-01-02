# Dockerfile for N2K Data Simulator
# Simulates NMEA 2000 GPS data for marine electronics testing

FROM python:3.11-slim-bookworm

LABEL maintainer="Colin Bitterfield <colin@bitterfield.com>"
LABEL description="N2K/NMEA 2000 GPS data simulator"
LABEL version="0.1.0"

# Set environment variables
ENV PYTHONUNBUFFERED=1 \
    DEBIAN_FRONTEND=noninteractive

# Install system dependencies
RUN apt-get update && apt-get install -y --no-install-recommends \
    curl \
    && rm -rf /var/lib/apt/lists/*

# Create application directory
WORKDIR /app

# Copy requirements and install Python dependencies
COPY n2k-simulator/requirements.txt /app/
RUN pip install --no-cache-dir -r requirements.txt

# Copy application files
COPY n2k-simulator/ /app/

# Create template directory
RUN mkdir -p /app/templates

# Expose ports
# 8081 - Web interface
# 2000 - UDP NMEA output
# 10110 - TCP NMEA output
EXPOSE 8081 2000/udp 10110

# Health check
HEALTHCHECK --interval=30s --timeout=10s --start-period=5s --retries=3 \
    CMD curl -f http://localhost:8081/api/status || exit 1

# Default command
CMD ["python", "/app/main.py"]
