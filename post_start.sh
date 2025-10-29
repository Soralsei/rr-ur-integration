#!/bin/bash
set -e

# Use the environment variable from the host
DEVICE_IP=${DEVICE_IP:-192.168.0.115}  # fallback if unset
echo "$DEVICE_IP rr-100-07.local" | sudo tee -a /etc/hosts > /dev/null
echo "Mapped rr-100-07.local -> $DEVICE_IP"

# Start main process
exec "$@"
